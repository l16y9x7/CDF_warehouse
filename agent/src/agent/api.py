import asyncio
import logging
import os
import time
import uuid
from contextlib import asynccontextmanager
from typing import Any

from fastapi import FastAPI, Request
from fastapi.exceptions import RequestValidationError
from fastapi.responses import JSONResponse
from pydantic import BaseModel, ConfigDict, Field, ValidationError

from agent.application import (
    AgentApplication,
    build_application,
    build_application_from_capabilities,
)
from agent.config import load_callback_url
from agent.contracts import AgentError
from agent.debug import DebugService, create_debug_router
from agent.debug.service import _mock_capabilities
from agent.models import ReviewItemCount
from agent.observability import (
    configure_logging,
    current_log_context,
    log_context,
    log_event,
)
from agent.orders import OrderService, create_order_router
from agent.orders.catalog import load_product_catalog
from agent.workflows.review import ReviewInput
from agent.workflows.sorting import SortingFinishInput, SortingItemInput

logger = logging.getLogger(__name__)


class StrictModel(BaseModel):
    model_config = ConfigDict(extra="forbid")


class TaskRequest(StrictModel):
    task_id: str = Field(min_length=1)
    mock: bool = False


class BasketPositionRequest(TaskRequest):
    basket_row: str = Field(pattern=r"^L[1-4]$")
    basket_column: str = Field(pattern=r"^[1-5]$")


class SortingItemRequest(BasketPositionRequest):
    sku_id: str = Field(min_length=1)
    name: str = Field(min_length=1)
    agv_row: str = Field(pattern=r"^L[1-5]$")
    agv_column: str = Field(pattern=r"^[12]$")


class ExpectedItem(StrictModel):
    sku_id: str = Field(min_length=1)
    count: int = Field(gt=0)


class ReviewRequest(BasketPositionRequest):
    order_id: str = Field(min_length=1)
    expected_items: list[ExpectedItem]


class TerminateRequest(StrictModel):
    mock: bool = False


def _public_mock_application(real: AgentApplication) -> AgentApplication:
    return build_application_from_capabilities(
        _mock_capabilities(),
        database_path=f"{real.store.path}.interface-mock",
        policy=real.policy,
        sku_catalog=real.skills["pick_sku_standard"].sku_catalog,
    )


def create_app(
    application: AgentApplication | None = None,
    *,
    callback_url: str | None = None,
    mock_application: AgentApplication | None = None,
) -> FastAPI:
    database_path = os.getenv("AGENT_DATABASE_PATH", "agent-tasks.db")
    service = application or build_application(database_path=database_path)
    mock_service = mock_application or _public_mock_application(service)
    resolved_callback_url = callback_url if callback_url is not None else load_callback_url()
    logging_manager = configure_logging(database_path=service.store.path)
    debug = DebugService(
        service, database_path=service.store.path, log_store=logging_manager.store,
    )
    products_path = os.getenv("AGENT_PRODUCTS_PATH", "configs/products.yaml")
    sku_catalog = getattr(service.skills.get("pick_sku_standard"), "sku_catalog", {})
    orders = OrderService(
        service,
        database_path=service.store.path,
        products=load_product_catalog(products_path, sku_catalog),
        mock_application=mock_service,
    )
    debug.watch_application(mock_service)

    @asynccontextmanager
    async def lifespan(app: FastAPI):
        log_event(
            logger, logging.INFO, "process.started", "机器人 Agent 服务已启动",
            status="SUCCEEDED",
        )
        service.runtime.seal_interrupted_tasks()
        mock_service.runtime.seal_interrupted_tasks()
        yield
        log_event(
            logger, logging.INFO, "process.stopping", "机器人 Agent 服务正在安全关闭",
            status="RUNNING",
        )
        orders.close()
        debug.shutdown()
        service.close()
        if mock_service is not service:
            mock_service.close()
        logging_manager.shutdown()

    app = FastAPI(title="Robot Agent", lifespan=lifespan)
    app.state.agent, app.state.mock_agent = service, mock_service
    app.state.debug, app.state.orders = debug, orders
    app.state.logging_manager = logging_manager
    app.include_router(create_debug_router(debug))
    app.include_router(create_order_router(orders))

    @app.middleware("http")
    async def request_logging(request: Request, call_next):
        request_id = request.headers.get("X-Request-ID") or uuid.uuid4().hex
        started = time.monotonic()
        with log_context(request_id=request_id):
            try:
                response = await call_next(request)
            except Exception as exc:
                log_event(
                    logger, logging.ERROR, "http.request.completed",
                    "HTTP 请求处理失败：服务发生未处理异常",  # noqa: RUF001
                    request_id=request_id,
                    method=request.method, route=request.scope.get("route").path
                    if request.scope.get("route") else request.url.path,
                    status="FAILED", http_status=500,
                    duration_ms=round((time.monotonic() - started) * 1000, 3),
                    error_code="UNHANDLED_EXCEPTION", error_message=str(exc),
                    suggestion="请根据请求编号检查关联日志", exc_info=True,
                )
                raise
            response.headers["X-Request-ID"] = request_id
            status = "SUCCEEDED" if response.status_code < 400 else "FAILED"
            route = request.scope.get("route")
            log_event(
                logger, logging.INFO if status == "SUCCEEDED" else logging.WARNING,
                "http.request.completed",
                "HTTP 请求处理完成" if status == "SUCCEEDED" else "HTTP 请求处理未成功",
                method=request.method, route=route.path if route else request.url.path,
                status=status, http_status=response.status_code,
                duration_ms=round((time.monotonic() - started) * 1000, 3),
            )
            return response

    @app.exception_handler(ValueError)
    async def value_error_handler(request: Request, exc: ValueError):
        return JSONResponse(
            status_code=422, content={"error_code": "INVALID_INPUT", "message": str(exc)}
        )

    @app.exception_handler(RequestValidationError)
    async def request_validation_handler(request: Request, exc: RequestValidationError):
        return JSONResponse(
            status_code=422,
            content={"error_code": "INVALID_INPUT", "message": str(exc)},
        )

    @app.exception_handler(AgentError)
    async def agent_error_handler(request: Request, exc: AgentError):
        status_code = 409 if exc.code in {"ROBOT_BUSY", "NO_ACTIVE_TASK"} else 422
        return JSONResponse(
            status_code=status_code,
            content={"error_code": exc.code, "message": exc.message},
        )

    @app.post("/agent/sorting/item")
    async def sorting_item(request: SortingItemRequest):
        data = SortingItemInput(
            request.sku_id,
            request.name,
            request.agv_row,
            request.agv_column,
            request.basket_row,
            request.basket_column,
        )
        return await _accept(
            _target(service, mock_service, request.mock),
            "sorting_item",
            request,
            data,
            resolved_callback_url,
            debug,
        )

    @app.post("/agent/sorting/finish")
    async def sorting_finish(request: BasketPositionRequest):
        return await _accept(
            _target(service, mock_service, request.mock),
            "sorting_finish",
            request,
            SortingFinishInput(request.basket_row, request.basket_column),
            resolved_callback_url,
            debug,
        )

    @app.post("/agent/review")
    async def review(request: ReviewRequest):
        seen = [item.sku_id for item in request.expected_items]
        if len(seen) != len(set(seen)):
            raise ValueError("expected_items sku_id must be unique")
        data = ReviewInput(
            request.order_id,
            request.basket_row,
            request.basket_column,
            tuple(ReviewItemCount(item.sku_id, item.count) for item in request.expected_items),
        )
        return await _accept(
            _target(service, mock_service, request.mock),
            "review",
            request,
            data,
            resolved_callback_url,
            debug,
        )

    @app.post("/agent/terminate")
    async def terminate(request: Request):
        raw = await request.body()
        use_mock = False
        if raw.strip():
            try:
                use_mock = TerminateRequest.model_validate_json(raw).mock
            except ValidationError as exc:
                raise ValueError(str(exc)) from exc
        task_id = _target(service, mock_service, use_mock).runtime.terminate()
        return {"task_id": task_id, "status": "ACCEPTED"}

    orders.bind_app(app)
    orders.watch_runtime(service)
    orders.watch_runtime(mock_service)
    return app


def _target(real: AgentApplication, mock: AgentApplication, use_mock: bool) -> AgentApplication:
    return mock if use_mock else real


async def _accept(
    service: AgentApplication,
    task_type: str,
    request: TaskRequest,
    data: Any,
    callback_url: str,
    debug: DebugService,
) -> dict[str, str]:
    if service.store.get(request.task_id) is not None:
        return {"task_id": request.task_id, "status": "ACCEPTED"}
    try:
        await asyncio.wrap_future(service.runtime.executor.submit(service.preflight, task_type))
    except Exception as exc:
        return JSONResponse(
            status_code=503, content={"error_code": "CAPABILITY_UNAVAILABLE", "message": str(exc)}
        )
    run_id = debug.record_agent_task(
        "mock" if request.mock else "real", task_type, request.task_id, request.model_dump()
    )
    try:
        service.runtime.accept(
            request.task_id, task_type, callback_url, request.model_dump(), data,
            metadata={
                "request_id": current_log_context().get("request_id"),
                "run_id": run_id,
            },
        )
    except Exception as exc:
        debug.fail_agent_task(run_id, exc)
        raise
    return {"task_id": request.task_id, "status": "ACCEPTED"}
