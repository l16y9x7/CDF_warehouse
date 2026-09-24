"""场景 HTTP 客户端。Gateway 只转发，不解释 payload。"""

from __future__ import annotations

import json
import logging
from dataclasses import dataclass, field
from typing import Any, Callable, Dict, Mapping, Optional
from urllib.error import HTTPError, URLError
from urllib.request import Request, urlopen

LOGGER = logging.getLogger(__name__)


@dataclass(frozen=True)
class ScenarioResponse:
    reached: bool
    accepted: bool
    status_code: int
    body: Dict[str, Any] = field(default_factory=dict)
    error: str = ""
    error_code: str = ""


class ScenarioClient:
    def __init__(
        self,
        *,
        timeout_sec: float = 8.0,
        poll_timeout_sec: float = 0.8,
        opener: Optional[Callable[..., Any]] = None,
    ) -> None:
        self.timeout_sec = float(timeout_sec)
        self.poll_timeout_sec = max(float(poll_timeout_sec), 0.05)
        self._opener = opener or urlopen

    def post(self, base_url: str, path: str, body: Mapping[str, Any]) -> ScenarioResponse:
        url = f"{base_url.rstrip('/')}{path}"
        payload = json.dumps(
            dict(body),
            ensure_ascii=False,
            allow_nan=False,
            separators=(",", ":"),
        ).encode("utf-8")
        request = Request(
            url,
            data=payload,
            method="POST",
            headers={
                "Content-Type": "application/json; charset=utf-8",
                "Accept": "application/json",
            },
        )
        try:
            with self._opener(request, timeout=self.timeout_sec) as response:
                status = int(getattr(response, "status", 200) or 200)
                raw = response.read()
        except HTTPError as exc:
            raw = exc.read() if exc.fp is not None else b""
            status = int(exc.code)
            parsed = _parse_json(raw)
            error_code, error = _error_from_body(parsed, fallback=str(exc))
            LOGGER.warning(
                "scenario HTTP error: url=%s status=%s error_code=%s error=%s",
                url,
                status,
                error_code,
                error,
            )
            return ScenarioResponse(
                reached=True,
                accepted=False,
                status_code=status,
                body=parsed,
                error=error,
                error_code=error_code or "GATEWAY_SCENARIO_HTTP_ERROR",
            )
        except (URLError, TimeoutError, OSError) as exc:
            LOGGER.warning("scenario unreachable: url=%s error=%s", url, exc)
            return ScenarioResponse(
                reached=False,
                accepted=False,
                status_code=0,
                error=str(exc) or "scenario unreachable",
                error_code="GATEWAY_SCENARIO_UNREACHABLE",
            )

        parsed = _parse_json(raw)
        if status >= 400:
            error_code, error = _error_from_body(parsed, fallback=f"HTTP {status}")
            LOGGER.warning(
                "scenario HTTP error: url=%s status=%s error_code=%s error=%s",
                url,
                status,
                error_code,
                error,
            )
            return ScenarioResponse(
                reached=True,
                accepted=False,
                status_code=status,
                body=parsed,
                error=error,
                error_code=error_code or "GATEWAY_SCENARIO_HTTP_ERROR",
            )
        accepted, error_code, error = _acceptance(parsed)
        if accepted:
            LOGGER.info("scenario accepted: url=%s status=%s", url, status)
        else:
            LOGGER.warning(
                "scenario rejected: url=%s status=%s error_code=%s error=%s",
                url,
                status,
                error_code,
                error,
            )
        return ScenarioResponse(
            reached=True,
            accepted=accepted,
            status_code=status,
            body=parsed,
            error="" if accepted else (error or "scenario rejected"),
            error_code="" if accepted else (error_code or "GATEWAY_SCENARIO_REJECTED"),
        )

    def get_json(self, url: str) -> Optional[Dict[str, Any]]:
        request = Request(
            url,
            method="GET",
            headers={"Accept": "application/json"},
        )
        return self._read_json(request, url, "GET")

    def post_json(
        self, url: str, body: Optional[Mapping[str, Any]] = None
    ) -> Optional[Dict[str, Any]]:
        payload = json.dumps(
            dict(body or {}),
            ensure_ascii=False,
            allow_nan=False,
            separators=(",", ":"),
        ).encode("utf-8")
        request = Request(
            url,
            data=payload,
            method="POST",
            headers={
                "Content-Type": "application/json; charset=utf-8",
                "Accept": "application/json",
            },
        )
        return self._read_json(request, url, "POST")

    def _read_json(self, request: Request, url: str, method: str) -> Optional[Dict[str, Any]]:
        try:
            with self._opener(request, timeout=self.poll_timeout_sec) as response:
                status = int(getattr(response, "status", 200) or 200)
                raw = response.read()
        except HTTPError as exc:
            # mui-sdk upper-body：部分模块失效时 HTTP 503，但 body 仍含有效模块
            raw = exc.read() if exc.fp is not None else b""
            status = int(exc.code)
            if status == 503:
                parsed = _parse_json(raw)
                return parsed or None
            LOGGER.debug("%s failed: url=%s status=%s", method, url, status)
            return None
        except Exception as exc:
            LOGGER.debug("%s failed: url=%s error=%s", method, url, exc)
            return None
        if status >= 400:
            return None
        parsed = _parse_json(raw)
        return parsed or None


def _parse_json(raw: bytes) -> Dict[str, Any]:
    if not raw:
        return {}
    try:
        value = json.loads(raw.decode("utf-8"))
    except (UnicodeDecodeError, json.JSONDecodeError):
        return {}
    return value if isinstance(value, dict) else {}


def _error_from_body(body: Mapping[str, Any], *, fallback: str) -> tuple[str, str]:
    error_code = str(body.get("error_code") or "").strip()
    message = str(body.get("message") or body.get("error") or fallback).strip()
    return error_code, message


def _acceptance(body: Mapping[str, Any]) -> tuple[bool, str, str]:
    error_code = str(body.get("error_code") or "").strip()
    message = str(body.get("message") or body.get("error") or "").strip()
    if "accepted" in body:
        accepted = bool(body.get("accepted"))
        return accepted, error_code, message
    if error_code:
        return False, error_code, message
    code = body.get("code")
    if isinstance(code, int) and code != 0:
        return False, error_code, message or "failed"
    result = body.get("result")
    if isinstance(result, int) and result != 0:
        return False, error_code, message or "failed"
    return True, "", message
