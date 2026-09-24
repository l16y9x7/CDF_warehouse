"""中免给 Agent 看的 HTTP 皮（:8081）。

只负责拆请求、转业务；真正逻辑在 service.py。
合同：成功 200 {"status":"SUCCEEDED"}；失败非 2xx {"error_code":"EXECUTION_FAILED"}。
这和能力层 :8001 不同——那边拒绝多数仍是 HTTP 200 + error_code。
"""

from __future__ import annotations

import logging
from typing import Any, Dict, Optional

from fastapi import Body, FastAPI, Header, Request
from fastapi.responses import JSONResponse

from zhongmian.service import CapabilityError, NavigationFacade

LOGGER = logging.getLogger(__name__)


def create_app(facade: NavigationFacade) -> FastAPI:
    app = FastAPI(title="Zhongmian navigation", version="0.1", docs_url="/docs", redoc_url=None)

    @app.exception_handler(CapabilityError)
    async def capability_error(_request: Request, exc: CapabilityError) -> JSONResponse:
        # 合同要求失败体只有 error_code，不把内部原因透给 Agent。
        return JSONResponse(status_code=exc.status_code, content={"error_code": exc.error_code})

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

    @app.get("/navigation/health")
    def health() -> Dict[str, str]:
        # 下游 nav 没起来也返回 200，靠 body 的 READY / ERROR 区分。
        return facade.health()

    @app.post("/navigation/navigate")
    def navigate(
        body: Dict[str, Any] = Body(default_factory=dict),
        # 幂等键必须来自 Agent Header，这里不自动生成。
        idempotency_key: Optional[str] = Header(default=None, alias="Idempotency-Key"),
    ) -> Dict[str, str]:
        # 文档后半用过 target_id，和 nav_id 当同一个点位名。
        nav_id = str(body.get("nav_id") or body.get("target_id") or "").strip()
        return facade.navigate(nav_id=nav_id, idempotency_key=str(idempotency_key or ""))

    return app
