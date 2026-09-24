import asyncio
import os
from pathlib import Path
from typing import Any

from fastapi import APIRouter, Query, Request
from fastapi.responses import FileResponse, JSONResponse, RedirectResponse, Response
from pydantic import BaseModel, ConfigDict, Field

from .media import depth_preview_png
from .service import DebugService

STATIC_DIR = Path(__file__).with_name("static")
MOCK_PNG = bytes.fromhex(
    "89504e470d0a1a0a0000000d4948445200000001000000010802000000907753de"
    "0000000c4944415408d76360f8cfc000000301010018dd8db10000000049454e44ae426082"
)


class StrictModel(BaseModel):
    model_config = ConfigDict(extra="forbid")


class DebugRequest(StrictModel):
    target: str
    layer: str
    operation: str
    payload: dict[str, Any] = Field(default_factory=dict)
    confirmation_token: str | None = None


class WorkflowDebugRequest(StrictModel):
    target: str
    payload: dict[str, Any] = Field(default_factory=dict)


class TerminateDebugRequest(StrictModel):
    target: str


def create_debug_router(service: DebugService) -> APIRouter:
    router = APIRouter()

    async def run_blocking(function, *args, **kwargs):
        future = service.executor.submit(function, *args, **kwargs)
        while not future.done():
            await asyncio.sleep(0.01)
        return future.result()

    @router.get("/debug", include_in_schema=False)
    async def debug_redirect():
        return RedirectResponse("/debug/", status_code=307)

    @router.get("/debug/")
    async def index():
        return Response(
            (STATIC_DIR / "index.html").read_bytes(),
            media_type="text/html",
            headers={"Cache-Control": "no-store"},
        )

    @router.get("/debug/assets/{filename}")
    async def asset(filename: str):
        if filename not in {"app.js", "styles.css"}:
            return JSONResponse({"error_code": "NOT_FOUND"}, status_code=404)
        media_type = "text/javascript" if filename.endswith(".js") else "text/css"
        return Response(
            (STATIC_DIR / filename).read_bytes(),
            media_type=media_type,
            headers={"Cache-Control": "no-store"},
        )

    @router.get("/debug/api/catalog")
    async def catalog():
        return service.catalog()

    @router.get("/debug/api/test-cases")
    async def test_cases():
        return await run_blocking(service.test_cases)

    @router.get("/debug/api/targets")
    async def targets():
        return await run_blocking(service.targets)

    @router.post("/debug/api/confirmations")
    async def confirmation(request: DebugRequest):
        return service.confirmation(
            request.target, request.layer, request.operation, request.payload
        )

    @router.post("/debug/api/runs")
    async def run(request: DebugRequest):
        return await run_blocking(
            service.run,
            request.target,
            request.layer,
            request.operation,
            request.payload,
            confirmation_token=request.confirmation_token,
        )

    @router.post("/debug/api/workflows/{operation}")
    async def workflow(operation: str, body: WorkflowDebugRequest, request: Request):
        return await run_blocking(
            service.submit_workflow,
            body.target,
            operation,
            body.payload,
            callback_base_url=str(request.base_url),
        )

    @router.post("/debug/api/terminate")
    async def terminate(body: TerminateDebugRequest):
        return service.terminate(body.target)

    @router.post("/debug/api/callbacks/{run_id}")
    async def callback(run_id: str, payload: dict[str, Any]):
        try:
            service.callback(run_id, payload)
        except KeyError:
            return JSONResponse({"error_code": "RUN_NOT_FOUND"}, status_code=404)
        return Response(status_code=204)

    @router.get("/debug/api/runs")
    async def runs(
        target: str | None = None,
        layer: str | None = None,
        status: str | None = None,
        limit: int = Query(default=100, ge=1, le=500),
    ):
        return {
            "runs": service.store.list_runs(target=target, layer=layer, status=status, limit=limit)
        }

    @router.get("/debug/api/runs/{run_id}")
    async def run_detail(run_id: str):
        run = service.get_run(run_id)
        if run is None:
            return JSONResponse({"error_code": "RUN_NOT_FOUND"}, status_code=404)
        return run

    @router.get("/debug/api/runs/{run_id}/trace")
    async def run_trace(run_id: str):
        trace = service.get_trace(run_id)
        if trace is None:
            return JSONResponse({"error_code": "RUN_NOT_FOUND"}, status_code=404)
        return trace

    @router.get("/debug/api/logs")
    async def logs(
        level: str | None = None, event: str | None = None, task_id: str | None = None,
        request_id: str | None = None, run_id: str | None = None,
        span_id: str | None = None,
        workflow: str | None = None, skill: str | None = None,
        capability: str | None = None, component: str | None = None,
        from_time: float | None = Query(default=None, alias="from"),
        to_time: float | None = Query(default=None, alias="to"), search: str | None = None,
        cursor: int | None = Query(default=None, ge=1), limit: int = Query(default=100, ge=1, le=500),
    ):
        records, next_cursor = await run_blocking(
            service.log_store.query, level=level, event=event, task_id=task_id,
            request_id=request_id, run_id=run_id, workflow=workflow, skill=skill,
            span_id=span_id,
            capability=capability, from_time=from_time, to_time=to_time,
            component=component, search=search, cursor=cursor, limit=limit,
        )
        return {"logs": records, "next_cursor": next_cursor}

    @router.get("/debug/api/media")
    async def media(path: str):
        candidate = Path(path).resolve()
        roots = [
            Path(item).resolve()
            for item in os.getenv("AGENT_DEBUG_MEDIA_ROOTS", "/shared/frames").split(os.pathsep)
            if item
        ]
        if not any(candidate == root or root in candidate.parents for root in roots):
            return JSONResponse({"error_code": "MEDIA_PATH_FORBIDDEN"}, status_code=403)
        if path.startswith("/shared/frames/capture-") and not candidate.exists():
            return Response(MOCK_PNG, media_type="image/png")
        if not candidate.is_file():
            return JSONResponse({"error_code": "MEDIA_NOT_FOUND"}, status_code=404)
        if candidate.suffix.lower() == ".npy":
            try:
                preview = await run_blocking(depth_preview_png, candidate)
            except (OSError, ValueError) as exc:
                return JSONResponse(
                    {"error_code": "DEPTH_PREVIEW_INVALID", "message": str(exc)},
                    status_code=422,
                )
            return Response(preview, media_type="image/png")
        return FileResponse(candidate)

    return router
