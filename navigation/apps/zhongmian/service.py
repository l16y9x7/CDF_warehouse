"""中免导航业务：把 Agent 的 nav_id 译成站点，再同步等到 nav :8001 跑完。

Agent 调一次 /navigation/navigate 会一直阻塞到车到站或失败。
下游能力层 /goto 是异步的（先回 ACCEPTED），所以这里还要轮询 /status。
"""

from __future__ import annotations

import logging
import threading
import time
from typing import Any, Dict, Optional, Tuple

from zhongmian.nav_client import NavClient

LOGGER = logging.getLogger(__name__)

# nav /status 里这些算走完了，不再轮询。
_TERMINAL = {"SUCCEEDED", "FAILED", "CANCELLED", "TIMED_OUT", "REJECTED"}


class CapabilityError(Exception):
    """转成 HTTP 非 2xx。对外 error_code 固定 EXECUTION_FAILED，不暴露内部原因。"""

    def __init__(self, status_code: int, error_code: str = "EXECUTION_FAILED") -> None:
        self.status_code = int(status_code)
        self.error_code = str(error_code)
        super().__init__(error_code)


class NavigationFacade:
    def __init__(self, config: Dict[str, Any], *, nav: Any = None) -> None:
        self.config = config
        nav_cfg = config.get("nav") if isinstance(config.get("nav"), dict) else {}
        self.timeout_sec = float(nav_cfg.get("timeout_sec", 90))
        self.poll_sec = float(nav_cfg.get("poll_sec", 0.3))
        self.task_id = str(nav_cfg.get("task_id") or "ZHONGMIAN")
        # 业务名 → 能力层 station_id，同名。改点位只改 apps/zhongmian/zhongmian.json。
        raw_points = config.get("waypoints") if isinstance(config.get("waypoints"), dict) else {}
        self.waypoints = {str(key): str(value) for key, value in raw_points.items()}
        self.nav = nav or NavClient(config)
        self._lock = threading.Lock()
        # 幂等缓存：Idempotency-Key → (nav_id, 上次成功结果)。失败不缓存，方便换原因重试。
        self._by_key: Dict[str, Tuple[str, Dict[str, Any]]] = {}

    def health(self) -> Dict[str, str]:
        if self.nav.ready():
            return {"status": "READY"}
        return {"status": "ERROR"}

    def navigate(self, *, nav_id: str, idempotency_key: str) -> Dict[str, str]:
        nav_id = str(nav_id or "").strip()
        idempotency_key = str(idempotency_key or "").strip()
        if not idempotency_key or not nav_id:
            raise CapabilityError(400)
        station_id = self.waypoints.get(nav_id)
        if not station_id:
            LOGGER.warning("unknown nav_id=%s", nav_id)
            raise CapabilityError(400)

        with self._lock:
            cached = self._by_key.get(idempotency_key)
            if cached is not None:
                old_id, result = cached
                # 同一把 key 不能改目的地（防重放打错站）。
                if old_id != nav_id:
                    raise CapabilityError(400)
                return dict(result)

        if not self.nav.ready():
            raise CapabilityError(503)

        request_id = _request_id(idempotency_key)
        try:
            # 幂等键原样传给 :8001，能力层自己也会再去重一次。
            accepted = self.nav.goto(
                {
                    "task_id": self.task_id,
                    "request_id": request_id,
                    "timeout_sec": self.timeout_sec,
                    "idempotency_key": idempotency_key,
                    "station_id": station_id,
                }
            )
        except Exception as exc:
            LOGGER.error("nav goto failed: nav_id=%s error=%s", nav_id, exc)
            raise CapabilityError(500) from exc

        # 能力层拒绝是 HTTP 200 + accepted=false，这里要当成失败。
        if accepted.get("accepted") is False:
            LOGGER.warning("nav rejected goto: %s", accepted)
            raise CapabilityError(500)

        terminal = self._wait(request_id)
        if terminal != "SUCCEEDED":
            LOGGER.warning("nav finished %s: nav_id=%s", terminal, nav_id)
            raise CapabilityError(500)

        result = {"status": "SUCCEEDED"}
        with self._lock:
            self._by_key[idempotency_key] = (nav_id, dict(result))
        return result

    def _wait(self, request_id: str) -> str:
        """阻塞到这次 goto 终态，或超时。"""
        deadline = time.monotonic() + max(0.1, self.timeout_sec)
        last = "RUNNING"
        while time.monotonic() < deadline:
            try:
                body = self.nav.status(request_id)
            except Exception as exc:
                LOGGER.error("nav status failed: %s", exc)
                return "FAILED"
            last = str(body.get("terminal_state") or body.get("state") or "")
            if last in _TERMINAL:
                return last
            time.sleep(self.poll_sec)
        return "TIMED_OUT"


def _request_id(idempotency_key: str) -> str:
    # :8001 要求每次 goto 有 request_id；用幂等键派生，避免自己再发明一套 ID。
    return "ZM-" + "".join(ch if ch.isalnum() else "-" for ch in idempotency_key)[:80]
