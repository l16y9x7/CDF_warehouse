"""FastAPI 相机 / 推流合同口。"""

from __future__ import annotations

import logging
from typing import Any, Dict

from fastapi import Body, FastAPI, Request
from fastapi.exceptions import RequestValidationError
from fastapi.responses import JSONResponse
from starlette.exceptions import HTTPException as StarletteHTTPException

from vision.service import CameraService, MediaService

LOGGER = logging.getLogger(__name__)


def create_camera_app(service: CameraService) -> FastAPI:
    app = FastAPI(title="Vision Camera", version="0.1", docs_url="/docs", redoc_url=None)
    _install_contract_errors(app, logger_name="camera")

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
        return service.health()

    @app.get("/state")
    def state() -> Dict[str, Any]:
        return service.state()

    @app.get("/list")
    def listing() -> Dict[str, Any]:
        return service.listing()

    @app.get("/frame/{camera_id}")
    def frame(camera_id: str) -> JSONResponse:
        payload = service.frame(camera_id)
        code = str(payload.get("error_code") or "")
        if code in {"CAMERA_NOT_FOUND", "CAMERA_NOT_READY"}:
            return JSONResponse(status_code=404, content=payload)
        return JSONResponse(status_code=200, content=payload)

    return app


def create_media_app(service: MediaService) -> FastAPI:
    app = FastAPI(title="Vision Media", version="0.1", docs_url="/docs", redoc_url=None)
    _install_contract_errors(app, logger_name="media")

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
        return service.health()

    @app.get("/state")
    def state() -> Dict[str, Any]:
        return service.state()

    @app.get("/stream/{camera_id}")
    def stream(camera_id: str) -> JSONResponse:
        payload = service.stream(camera_id)
        if payload.get("error_code") == "CAMERA_NOT_FOUND":
            return JSONResponse(status_code=404, content=payload)
        return JSONResponse(status_code=200, content=payload)

    @app.get("/push")
    def get_push() -> Dict[str, Any]:
        return service.push_status()

    @app.post("/push/start")
    def start_push(body: Dict[str, Any] = Body(default_factory=dict)) -> JSONResponse:
        payload = dict(body or {})
        result = service.start_push(
            camera_id=str(payload.get("camera_id") or ""),
            device_sn=str(payload.get("device_sn") or payload.get("sn") or ""),
            stream_key=str(payload.get("stream_key") or ""),
        )
        status = 404 if result.get("error_code") == "CAMERA_NOT_FOUND" else 200
        if result.get("error_code") == "MEDIA_PUSH_DISABLED":
            status = 400
        return JSONResponse(status_code=status, content=result)

    @app.post("/push/stop")
    def stop_push(body: Dict[str, Any] = Body(default_factory=dict)) -> JSONResponse:
        payload = dict(body or {})
        result = service.stop_push(camera_id=str(payload.get("camera_id") or ""))
        status = 404 if result.get("error_code") == "CAMERA_NOT_FOUND" else 200
        return JSONResponse(status_code=status, content=result)

    @app.on_event("startup")
    def on_startup() -> None:
        if service.push.autostart:
            service.start_push()

    @app.on_event("shutdown")
    def on_shutdown() -> None:
        service.push.stop_all()

    return app


def _install_contract_errors(app: FastAPI, *, logger_name: str) -> None:
    del logger_name

    @app.exception_handler(StarletteHTTPException)
    async def http_error(request: Request, exc: StarletteHTTPException) -> JSONResponse:
        if exc.status_code == 404:
            LOGGER.warning("HTTP %s %s -> 404", request.method, request.url.path)
            return JSONResponse(status_code=404, content={"ok": False, "error_code": "NOT_FOUND"})
        return JSONResponse(
            status_code=exc.status_code,
            content={"ok": False, "error_code": "CAMERA_HTTP", "message": str(exc.detail)},
        )

    @app.exception_handler(RequestValidationError)
    async def invalid_payload(_request: Request, exc: RequestValidationError) -> JSONResponse:
        LOGGER.warning("invalid JSON: %s", exc.errors())
        return JSONResponse(
            status_code=400,
            content={"ok": False, "error_code": "INVALID_JSON", "message": "invalid JSON"},
        )
