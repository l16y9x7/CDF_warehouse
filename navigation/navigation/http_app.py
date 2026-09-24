"""能力层 HTTP 皮（:8001）。路由只转发，逻辑在 service.py。

和中免 :8081 不一样：这里缺字段 / 忙 / 未就绪多数仍是 HTTP 200，
body 里 accepted=false + error_code。未知路径才 404。
zhongmian 调的是 POST /goto 和 GET /status/{request_id}。
"""

from __future__ import annotations

import logging
from typing import Any, Dict, Optional

from fastapi import Body, FastAPI, Request
from fastapi.exceptions import RequestValidationError
from fastapi.responses import JSONResponse
from starlette.exceptions import HTTPException as StarletteHTTPException

from navigation.service import NavigationService

LOGGER = logging.getLogger(__name__)


def create_app(service: NavigationService) -> FastAPI:
    app = FastAPI(
        title="Navigation",
        version="0.1",
        docs_url="/docs",
        redoc_url=None,
    )
    _install_contract_errors(app, domain="NAVIGATION")

    @app.middleware("http")
    async def log_requests(request: Request, call_next):
        response = await call_next(request)
        LOGGER.info(
            "HTTP %s %s from %s -> %s",
            request.method,
            request.url.path,
            request.client.host if request.client else "-",
            response.status_code,
        )
        return response

    @app.get("/health")
    def health() -> Dict[str, Any]:
        # {"ok": true} 才算就绪；珞石未定位（location_state != 3）为 false。
        return service.health()

    @app.get("/state")
    def state() -> Dict[str, Any]:
        return service.state()

    @app.get("/status")
    def status() -> Dict[str, Any]:
        return service.current_status()

    @app.get("/status/{request_id}")
    def status_of(request_id: str) -> Dict[str, Any]:
        # zhongmian 轮询这个看终态：SUCCEEDED / FAILED / ...
        return service.status_of(request_id)

    @app.get("/map")
    def map_info() -> Dict[str, Any]:
        return service.map_info()

    @app.post("/refresh_stations")
    def refresh_stations() -> Dict[str, Any]:
        # 启动已拉过一次。这个口给现场手动再刷；不切 MATRIX 原图。
        return service.refresh_stations()

    @app.post("/goto")
    def goto(body: Dict[str, Any] = Body(default_factory=dict)) -> Dict[str, Any]:
        # 先回 ACCEPTED，车在后台线程里走；不要以为 200 就是到站了。
        return service.goto(body)

    @app.post("/stop")
    def stop(body: Optional[Dict[str, Any]] = Body(default_factory=dict)) -> Dict[str, Any]:
        return service.stop(body or {})

    @app.post("/cancel")
    def cancel(body: Dict[str, Any] = Body(default_factory=dict)) -> Dict[str, Any]:
        return service.cancel(body)

    @app.post("/load_map")
    def load_map(body: Dict[str, Any] = Body(default_factory=dict)) -> Dict[str, Any]:
        # 加载当前图：回站点 + occupancy，不切 MATRIX。
        return service.load_map(body)

    @app.post("/maps/import")
    def import_map(body: Dict[str, Any] = Body(default_factory=dict)) -> Dict[str, Any]:
        # 把 json 打成 FMS 包写进底盘目录。不切换当前正在用的图。
        return service.import_map(body)

    return app


def _install_contract_errors(app: FastAPI, *, domain: str) -> None:
    @app.exception_handler(StarletteHTTPException)
    async def http_error(_request: Request, exc: StarletteHTTPException) -> JSONResponse:
        if exc.status_code == 404:
            LOGGER.warning("HTTP %s %s -> 404", _request.method, _request.url.path)
            return JSONResponse(
                status_code=404,
                content={"ok": False, "error_code": "NOT_FOUND"},
            )
        return JSONResponse(
            status_code=exc.status_code,
            content={"ok": False, "error_code": f"{domain}_HTTP", "message": str(exc.detail)},
        )

    @app.exception_handler(RequestValidationError)
    async def invalid_payload(_request: Request, exc: RequestValidationError) -> JSONResponse:
        LOGGER.warning("invalid JSON: %s", exc.errors())
        return JSONResponse(
            status_code=400,
            content={"ok": False, "error_code": "INVALID_JSON", "message": "invalid JSON"},
        )
