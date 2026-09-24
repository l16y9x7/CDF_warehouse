import logging
import time
import uuid
from typing import Any

import httpx

from agent.capabilities.common import CapabilityError, CapabilityUnavailableError, ErrorSource
from agent.observability import log_event

logger = logging.getLogger(__name__)

IDEMPOTENCY_HEADER = "Idempotency-Key"
_NO_BODY = object()


def resolve_idempotency_key(idempotency_key: str | None = None) -> str:
    key = (idempotency_key or "").strip()
    return key or uuid.uuid4().hex


class HttpCapabilityClient:
    """Shared transport behavior for all HTTP capability adapters."""

    def __init__(
        self,
        base_url: str,
        *,
        timeout: float = 30.0,
        client: httpx.Client | None = None,
        capability: str = "unknown",
    ) -> None:
        self._owns_client = client is None
        self._client = client or httpx.Client(base_url=base_url, timeout=timeout)
        self.capability = capability

    def get(
        self,
        path: str,
        *,
        params: dict[str, Any] | None = None,
        timeout: float | None = None,
    ) -> Any:
        kwargs: dict[str, Any] = {}
        if timeout is not None:
            kwargs["timeout"] = timeout
        return self._request("GET", path, params=params, **kwargs)

    def post(
        self,
        path: str,
        payload: dict[str, Any] | object = _NO_BODY,
        *,
        headers: dict[str, str] | None = None,
        timeout: float | None = None,
    ) -> Any:
        kwargs: dict[str, Any] = {}
        if payload is not _NO_BODY:
            kwargs["json"] = payload
        if headers:
            kwargs["headers"] = headers
        if timeout is not None:
            kwargs["timeout"] = timeout
        return self._request("POST", path, **kwargs)

    def post_action(
        self,
        path: str,
        payload: dict[str, Any],
        *,
        idempotency_key: str | None = None,
        timeout: float | None = None,
    ) -> Any:
        """Execute a physical action whose outcome is uncertain after a timeout."""
        headers = {IDEMPOTENCY_HEADER: resolve_idempotency_key(idempotency_key)}
        try:
            return self.post(path, payload, headers=headers, timeout=timeout)
        except CapabilityUnavailableError as exc:
            if isinstance(exc.__cause__, (httpx.ReadTimeout, httpx.WriteTimeout)):
                log_event(
                    logger, logging.ERROR, "capability.action_result_unknown",
                    f"{self.capability} 物理动作调用超时，动作结果未知",  # noqa: RUF001
                    capability=self.capability, operation=f"POST {path}", status="FAILED",
                    error_code="ACTION_RESULT_UNKNOWN", error_message=str(exc),
                    suggestion="请人工确认机器人和物品状态后再继续任务",
                )
                raise CapabilityError(
                    "ACTION_RESULT_UNKNOWN",
                    "physical action result is unknown after transport timeout",
                    source=ErrorSource.TRANSPORT,
                    operation=exc.operation or f"POST {path}",
                ) from exc
            raise

    def get_bytes(self, path: str, *, params: dict[str, Any] | None = None) -> bytes:
        response = self.request_response("GET", path, params=params)
        return response.content

    def build_url(self, path: str, *, params: dict[str, Any] | None = None) -> str:
        request = self._client.build_request("GET", path, params=params)
        return str(request.url)

    def close(self) -> None:
        if self._owns_client:
            self._client.close()

    def _request(self, method: str, path: str, **kwargs: Any) -> Any:
        return self.request_response(method, path, **kwargs).json()

    def request_response(self, method: str, path: str, **kwargs: Any) -> httpx.Response:
        started = time.monotonic()
        try:
            response = self._client.request(method, path, **kwargs)
        except httpx.HTTPError as exc:
            duration = round((time.monotonic() - started) * 1000, 3)
            suggestion = (
                "请人工确认机器人和物品状态后再继续任务"
                if isinstance(exc, (httpx.ReadTimeout, httpx.WriteTimeout)) and method == "POST"
                else "请检查外部模块进程、地址和网络连接"
            )
            log_event(
                logger, logging.ERROR, "capability.failed",
                f"{self.capability} 能力调用失败：外部模块不可用",  # noqa: RUF001
                capability=self.capability, operation=f"{method} {path}", status="FAILED",
                duration_ms=duration, error_code="MODULE_UNAVAILABLE",
                error_message=str(exc), suggestion=suggestion, exception_type=type(exc).__name__,
            )
            raise CapabilityUnavailableError(
                "MODULE_UNAVAILABLE",
                str(exc),
                source=ErrorSource.TRANSPORT,
                operation=f"{method} {path}",
            ) from exc

        if response.is_success:
            log_event(
                logger, logging.INFO, "capability.succeeded",
                f"{self.capability} 能力调用成功", capability=self.capability,
                operation=f"{method} {path}", status="SUCCEEDED",
                http_status=response.status_code,
                duration_ms=round((time.monotonic() - started) * 1000, 3),
            )
            return response

        try:
            body = response.json()
        except ValueError:
            body = {}
        error_code = body.get("error_code", "CAPABILITY_REQUEST_FAILED")
        message = body.get(
            "message",
            body.get("error", f"capability returned HTTP {response.status_code}"),
        )
        log_event(
            logger, logging.ERROR, "capability.failed",
            f"{self.capability} 能力调用失败：外部模块返回错误",  # noqa: RUF001
            capability=self.capability, operation=f"{method} {path}", status="FAILED",
            http_status=response.status_code,
            duration_ms=round((time.monotonic() - started) * 1000, 3),
            error_code=error_code, error_message=message,
            suggestion="请检查外部模块状态和本次操作参数",
        )
        raise CapabilityError(
            error_code,
            message,
            status_code=response.status_code,
            source=ErrorSource.REMOTE,
            operation=f"{method} {path}",
        )
