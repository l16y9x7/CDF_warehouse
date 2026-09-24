"""打 nav 能力层 :8001 的 HTTP 客户端。路径来自 zhongmian.json 的 nav 段。"""

from __future__ import annotations

import logging
from typing import Any, Dict, Optional
from urllib.parse import urljoin

import httpx

LOGGER = logging.getLogger(__name__)


class NavClient:
    def __init__(self, config: Dict[str, Any], *, client: Optional[httpx.Client] = None) -> None:
        nav = config.get("nav") if isinstance(config.get("nav"), dict) else {}
        self.base_url = str(nav.get("base_url") or "http://127.0.0.1:8001").rstrip("/") + "/"
        self.health_path = str(nav.get("health_path") or "/health")
        self.goto_path = str(nav.get("goto_path") or "/goto")
        self.status_path = str(nav.get("status_path") or "/status")
        timeout = float(nav.get("timeout_sec", 90))
        # 比业务 timeout 稍长，避免 HTTP 先断、轮询还没结束。
        self._client = client or httpx.Client(timeout=timeout + 5.0)

    def _url(self, path: str) -> str:
        return urljoin(self.base_url, path.lstrip("/"))

    def ready(self) -> bool:
        # 能力层 health 是 {"ok": true/false}，车未定位时 ok=false。
        try:
            response = self._client.get(self._url(self.health_path))
            body = response.json() if response.content else {}
            return response.status_code == 200 and bool(body.get("ok"))
        except Exception as exc:
            LOGGER.warning("nav health failed: %s", exc)
            return False

    def goto(self, body: Dict[str, Any]) -> Dict[str, Any]:
        # 立刻返回 ACCEPTED，车还在走。终态看 status()。
        response = self._client.post(self._url(self.goto_path), json=body)
        response.raise_for_status()
        data = response.json()
        if not isinstance(data, dict):
            raise RuntimeError("nav goto returned non-object")
        return data

    def status(self, request_id: str) -> Dict[str, Any]:
        response = self._client.get(self._url(f"{self.status_path.rstrip('/')}/{request_id}"))
        response.raise_for_status()
        data = response.json()
        if not isinstance(data, dict):
            raise RuntimeError("nav status returned non-object")
        return data
