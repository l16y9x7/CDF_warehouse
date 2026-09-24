"""临时本地 HTTP：OSD task、地图同步、推流、场景事件上报。不是平台命令入口。"""

from __future__ import annotations

import logging
import socket
from typing import Any, Dict, Iterable, List, Mapping, Sequence

from fastapi import Body, FastAPI, Request
from fastapi.exceptions import RequestValidationError
from fastapi.responses import JSONResponse
from starlette.exceptions import HTTPException as StarletteHTTPException

from gateway.http_server import UvicornHttpServer
from gateway.map_sync import MapSyncError
from gateway.records import TaskStateStore

LOGGER = logging.getLogger(__name__)

_STATUS_ALIASES = {
    "idle": 0,
    "0": 0,
    "running": 1,
    "1": 1,
    "completed": 2,
    "succeeded": 2,
    "success": 2,
    "2": 2,
    "failed": 3,
    "error": 3,
    "3": 3,
}


def parse_task_status(value: Any) -> int:
    if isinstance(value, bool) or value is None:
        raise ValueError("status is required")
    if isinstance(value, int):
        if value not in {0, 1, 2, 3}:
            raise ValueError("status must be 0, 1, 2 or 3")
        return value
    key = str(value).strip().lower()
    if key not in _STATUS_ALIASES:
        raise ValueError("status must be idle/running/completed/failed or 0/1/2/3")
    return _STATUS_ALIASES[key]


def _publish_osd_now(gateway) -> bool:
    reporter = getattr(gateway, "osd_reporter", None)
    if reporter is None:
        return False
    try:
        return bool(reporter.report_now())
    except Exception as exc:
        LOGGER.warning("OSD publish after HTTP task update failed: %s", exc)
        return False


def unwrap_uplink_body(body: Any) -> Dict[str, Any]:
    """允许场景直接 POST 事件字段，或套一层 MQTT `data` 信封。"""

    if not isinstance(body, dict):
        raise ValueError("JSON object is required")
    nested = body.get("data")
    top_task = str(body.get("task_id") or "").strip()
    top_terminal = str(body.get("terminal_state") or body.get("stage") or "").strip()
    if (
        isinstance(nested, dict)
        and not top_task
        and not top_terminal
    ):
        return dict(nested)
    return dict(body)


def apply_osd_task_update(store: TaskStateStore, body: Dict[str, Any]) -> Dict[str, Any]:
    if "status" not in body:
        raise ValueError("status is required")
    status = parse_task_status(body.get("status"))
    return store.set(
        task_id=str(body.get("task_id") or ""),
        status=status,
        error=str(body.get("error") or ""),
        forced=True,
    )


def create_app(gateway) -> FastAPI:
    app = FastAPI(
        title="Gateway debug HTTP",
        version="0.1",
        docs_url="/docs",
        redoc_url=None,
    )

    @app.exception_handler(StarletteHTTPException)
    async def http_error(request: Request, exc: StarletteHTTPException) -> JSONResponse:
        if exc.status_code == 404:
            LOGGER.warning("HTTP %s %s -> 404", request.method, request.url.path)
            return JSONResponse(status_code=404, content={"ok": False, "error": "NOT_FOUND"})
        return JSONResponse(
            status_code=exc.status_code,
            content={"ok": False, "error": str(exc.detail)},
        )

    @app.exception_handler(RequestValidationError)
    async def invalid_json(request: Request, exc: RequestValidationError) -> JSONResponse:
        LOGGER.warning(
            "HTTP %s %s -> 400 invalid JSON: %s",
            request.method,
            request.url.path,
            exc.errors(),
        )
        return JSONResponse(
            status_code=400,
            content={"ok": False, "error": "INVALID_JSON", "message": "invalid JSON"},
        )

    def _uplink():
        return getattr(gateway, "uplink", None)

    def _publish_uplink(kind: str, body: Dict[str, Any]) -> JSONResponse:
        publisher = _uplink()
        if publisher is None:
            return JSONResponse(
                status_code=503,
                content={"ok": False, "error": "UPLINK_UNAVAILABLE"},
            )
        try:
            payload = unwrap_uplink_body(body)
            if kind == "result":
                result = publisher.publish_task_result(payload)
            else:
                result = publisher.publish_task_event(payload)
        except ValueError as exc:
            LOGGER.warning(
                "HTTP POST /%s -> 400 INVALID_REQUEST: %s body=%s",
                "results" if kind == "result" else "events",
                exc,
                body,
            )
            return JSONResponse(
                status_code=400,
                content={"ok": False, "error": "INVALID_REQUEST", "message": str(exc)},
            )
        published = bool(result.get("published"))
        _publish_osd_now(gateway)
        LOGGER.info(
            "HTTP POST /%s -> %s published=%s task_id=%s stage=%s",
            "results" if kind == "result" else "events",
            200 if published else 502,
            published,
            (result.get("data") or {}).get("task_id"),
            (result.get("data") or {}).get("stage"),
        )
        return JSONResponse(
            status_code=200 if published else 502,
            content={"ok": published, **result},
        )

    def _current_task() -> Dict[str, Any]:
        snapshot = gateway.task_state.snapshot()
        LOGGER.info(
            "HTTP GET task -> 200 task_id=%s status=%s forced=%s",
            snapshot.get("task_id") or "",
            snapshot.get("status"),
            gateway.task_state.forced,
        )
        return {"ok": True, "task": snapshot}

    @app.get("/")
    def index() -> Dict[str, Any]:
        return {
            "ok": True,
            "service": "gateway-debug-http",
            "docs": "/docs",
            "endpoints": [
                "GET /health",
                "GET /osd/task",
                "POST /osd/task",
                "POST /osd/task/clear",
                "POST /events",
                "POST /results",
                "GET /map/sync",
                "POST /map/sync",
                "GET /media/push",
                "POST /media/push/start",
                "POST /media/push/stop",
            ],
        }

    @app.get("/health")
    def health() -> Dict[str, Any]:
        return _current_task()

    @app.post("/events")
    def post_event(
        request: Request, body: Dict[str, Any] = Body(default_factory=dict)
    ) -> JSONResponse:
        LOGGER.info(
            "HTTP POST /events from %s body=%s",
            request.client.host if request.client else "-",
            body,
        )
        return _publish_uplink("event", body)

    @app.post("/results")
    def post_result(
        request: Request, body: Dict[str, Any] = Body(default_factory=dict)
    ) -> JSONResponse:
        LOGGER.info(
            "HTTP POST /results from %s body=%s",
            request.client.host if request.client else "-",
            body,
        )
        return _publish_uplink("result", body)

    @app.get("/osd/task")
    def get_task() -> Dict[str, Any]:
        return _current_task()

    @app.post("/osd/task")
    def set_task(request: Request, body: Dict[str, Any] = Body(default_factory=dict)) -> JSONResponse:
        LOGGER.info(
            "HTTP POST /osd/task from %s body=%s",
            request.client.host if request.client else "-",
            body,
        )
        before = gateway.task_state.snapshot()
        try:
            snapshot = apply_osd_task_update(gateway.task_state, body)
        except ValueError as exc:
            LOGGER.warning(
                "HTTP POST /osd/task -> 400 INVALID_REQUEST: %s body=%s",
                exc,
                body,
            )
            return JSONResponse(
                status_code=400,
                content={"ok": False, "error": "INVALID_REQUEST", "message": str(exc)},
            )
        published = _publish_osd_now(gateway)
        LOGGER.info(
            "OSD task updated via HTTP: before=%s after=%s forced=True; "
            "osd_published=%s",
            before,
            snapshot,
            published,
        )
        return JSONResponse(status_code=200, content={"ok": True, "task": snapshot})

    @app.post("/osd/task/clear")
    def clear_task(request: Request) -> Dict[str, Any]:
        LOGGER.info(
            "HTTP POST /osd/task/clear from %s",
            request.client.host if request.client else "-",
        )
        before = gateway.task_state.snapshot()
        snapshot = gateway.task_state.clear()
        published = _publish_osd_now(gateway)
        LOGGER.info(
            "OSD task cleared via HTTP: before=%s after=%s; osd_published=%s",
            before,
            snapshot,
            published,
        )
        return {"ok": True, "task": snapshot}

    def _map_sync():
        service = getattr(gateway, "map_sync", None)
        if service is None:
            return None
        return service

    @app.get("/map/sync")
    def get_map_sync() -> JSONResponse:
        service = _map_sync()
        if service is None:
            return JSONResponse(
                status_code=503,
                content={"ok": False, "error": "MAP_SYNC_UNAVAILABLE"},
            )
        try:
            status = service.status()
        except MapSyncError as exc:
            LOGGER.warning("HTTP GET /map/sync -> %s %s", exc.code, exc)
            return JSONResponse(
                status_code=exc.http_status,
                content={"ok": False, "error": exc.code, "message": str(exc)},
            )
        LOGGER.info(
            "HTTP GET /map/sync -> 200 source=%s map=%s uploaded=%s",
            status.get("current_source_map_id") or "",
            (status.get("current") or {}).get("map_id") or "",
            (status.get("current") or {}).get("last_upload_ok"),
        )
        return JSONResponse(status_code=200, content={"ok": True, **status})

    @app.post("/map/sync")
    def post_map_sync(
        request: Request, body: Dict[str, Any] = Body(default_factory=dict)
    ) -> JSONResponse:
        logged = dict(body)
        if logged.get("resource_token") or logged.get("resource-token"):
            logged["resource_token"] = "***"
            logged.pop("resource-token", None)
        LOGGER.info(
            "HTTP POST /map/sync from %s body=%s",
            request.client.host if request.client else "-",
            logged,
        )
        service = _map_sync()
        if service is None:
            return JSONResponse(
                status_code=503,
                content={"ok": False, "error": "MAP_SYNC_UNAVAILABLE"},
            )
        try:
            result = service.sync(
                source_map_id=str(body.get("source_map_id") or ""),
                resource=str(body.get("resource") or ""),
                resource_token=str(body.get("resource_token") or body.get("resource-token") or ""),
            )
        except MapSyncError as exc:
            LOGGER.warning(
                "HTTP POST /map/sync -> %s %s body=%s",
                exc.http_status,
                exc,
                logged,
            )
            return JSONResponse(
                status_code=exc.http_status,
                content={"ok": False, "error": exc.code, "message": str(exc)},
            )
        LOGGER.info(
            "map synced via HTTP: source=%s map=%s size=%sx%s; "
            "next OSD cycle will publish UUID map_id",
            result.get("source_map_id"),
            result.get("map_id"),
            result.get("width"),
            result.get("height"),
        )
        return JSONResponse(status_code=200, content={"ok": True, **result})

    def _media_push():
        return getattr(gateway, "media_push", None)

    @app.get("/media/push")
    def get_media_push() -> JSONResponse:
        controller = _media_push()
        if controller is None:
            return JSONResponse(
                status_code=503,
                content={"ok": False, "error": "MEDIA_PUSH_UNAVAILABLE"},
            )
        result = controller.status()
        status = 200 if result.get("ok") else 502
        return JSONResponse(status_code=status, content=result)

    @app.post("/media/push/start")
    def start_media_push(
        request: Request, body: Dict[str, Any] = Body(default_factory=dict)
    ) -> JSONResponse:
        logged = dict(body)
        if logged.get("stream_key"):
            logged["stream_key"] = "***"
        LOGGER.info(
            "HTTP POST /media/push/start from %s body=%s",
            request.client.host if request.client else "-",
            logged,
        )
        controller = _media_push()
        if controller is None:
            return JSONResponse(
                status_code=503,
                content={"ok": False, "error": "MEDIA_PUSH_UNAVAILABLE"},
            )
        result = controller.start(
            camera_id=str(body.get("camera_id") or ""),
            stream_key=str(body.get("stream_key") or ""),
        )
        status = 200 if result.get("ok") else 502
        return JSONResponse(status_code=status, content=result)

    @app.post("/media/push/stop")
    def stop_media_push(
        request: Request, body: Dict[str, Any] = Body(default_factory=dict)
    ) -> JSONResponse:
        LOGGER.info(
            "HTTP POST /media/push/stop from %s body=%s",
            request.client.host if request.client else "-",
            body,
        )
        controller = _media_push()
        if controller is None:
            return JSONResponse(
                status_code=503,
                content={"ok": False, "error": "MEDIA_PUSH_UNAVAILABLE"},
            )
        result = controller.stop(camera_id=str(body.get("camera_id") or ""))
        status = 200 if result.get("ok") else 502
        return JSONResponse(status_code=status, content=result)

    return app


def listen_ports_from_config(debug_http: Mapping[str, Any] | None) -> List[int]:
    cfg = debug_http if isinstance(debug_http, Mapping) else {}
    ports: list[int] = []
    primary = int(cfg.get("port") or 8088)
    ports.append(primary)
    extra = cfg.get("fallback_ports")
    if extra is None and primary == 8088:
        extra = [8089]
    if isinstance(extra, int):
        extra = [extra]
    if isinstance(extra, Sequence) and not isinstance(extra, (str, bytes)):
        for item in extra:
            try:
                port = int(item)
            except (TypeError, ValueError):
                continue
            if port not in ports:
                ports.append(port)
    return ports


def first_free_port(host: str, ports: Iterable[int]) -> int:
    bind_host = "0.0.0.0" if str(host or "").strip() in {"", "0.0.0.0"} else str(host)
    last_error: Exception | None = None
    tried: list[int] = []
    for raw in ports:
        port = int(raw)
        tried.append(port)
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        try:
            sock.bind((bind_host, port))
            return port
        except OSError as exc:
            last_error = exc
            LOGGER.warning("debug HTTP port busy: %s:%s (%s)", bind_host, port, exc)
        finally:
            sock.close()
    raise OSError(f"no free debug HTTP port: {tried}") from last_error


class OsdTaskHttpServer:
    def __init__(self, app, *, host: str, port: int) -> None:
        self.host = host
        self.port = int(port)
        self.api = create_app(app)
        self._http = UvicornHttpServer(self.api, host=host, port=self.port)

    def start(self) -> None:
        self._http.start_background(name="osd-task-http")
        LOGGER.info(
            "debug HTTP listening on %s:%s /osd/task /events /results /map/sync /media/push",
            self.host,
            self.port,
        )

    def stop(self) -> None:
        self._http.shutdown()
        LOGGER.info("temporary OSD task HTTP stopped")
