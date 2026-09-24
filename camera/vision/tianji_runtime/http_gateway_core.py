"""HTTP gateway primitives that do not depend on ROS."""

from __future__ import annotations

import logging
import threading
from dataclasses import dataclass
from typing import Any, Callable, Optional

_LOG = logging.getLogger(__name__)


@dataclass(frozen=True)
class NavigationCapabilities:
    response_ok: bool
    runtime_ready: bool
    navigation_ready: bool
    localization_ready: bool
    motion_ready: bool


def evaluate_navigation_health(
    *,
    action_server_ready: bool,
    health_service_ready: bool,
    nav_state: Optional[str],
    nav_status_age_sec: float,
    capabilities: Optional[NavigationCapabilities],
    capabilities_age_sec: float,
    stale_after_sec: float,
) -> str:
    """Combine gateway, bridge state and vendor capabilities."""
    if not action_server_ready or not health_service_ready:
        return "ERROR" if capabilities is not None else "STARTING"
    if nav_state is None or capabilities is None:
        return "STARTING"
    if (
        nav_status_age_sec > stale_after_sec
        or capabilities_age_sec > stale_after_sec
    ):
        return "ERROR"
    if nav_state == "ERROR":
        return "ERROR"
    if nav_state not in ("READY", "ARRIVED", "NAVIGATING"):
        return "STARTING"
    if (
        capabilities.response_ok
        and capabilities.runtime_ready
        and capabilities.navigation_ready
        and capabilities.localization_ready
        and capabilities.motion_ready
    ):
        return "READY"
    return "ERROR"


@dataclass(frozen=True)
class HttpResult:
    status_code: int
    body: dict[str, Any]


def overlay_nudge_health(base: str, *, busy: bool, count: int) -> str:
    """HTTP-only overlay. Does not change READY-during-NAVIGATING behavior."""
    if base == "ERROR":
        return "ERROR"
    if busy:
        return "NUDGING"
    if int(count) > 0:
        return "APPROACHED"
    return base


def http_from_nudge_result(
    error_code: int,
    error_msg: str,
    station_id: str,
    nudge_count: int,
) -> HttpResult:
    station_id = str(station_id or "")
    count = int(nudge_count)
    if int(error_code) == 0:
        return HttpResult(
            200,
            {
                "status": "SUCCEEDED",
                "station_id": station_id,
                "nudge_count": count,
            },
        )
    message = str(error_msg or "").lower()
    if int(error_code) == 2:
        if "limit" in message:
            return HttpResult(409, {"error_code": "NUDGE_LIMIT_EXCEEDED"})
        if "off station" in message or "nudge in progress" in message:
            return HttpResult(409, {"error_code": "OFF_STATION"})
        return HttpResult(409, {"error_code": "NOT_AT_STATION"})
    if int(error_code) in (1, 4):
        return HttpResult(503, {"error_code": "MODULE_NOT_READY"})
    if int(error_code) == 6:
        return HttpResult(504, {"error_code": "EXECUTION_FAILED"})
    return HttpResult(500, {"error_code": "EXECUTION_FAILED"})


@dataclass
class _IdempotencyEntry:
    target_id: str
    completed: threading.Event
    result: Optional[HttpResult] = None


class IdempotencyRegistry:
    """Ensure one physical action is executed for each idempotency key."""

    def __init__(self) -> None:
        self._entries: dict[str, _IdempotencyEntry] = {}
        self._lock = threading.Lock()

    def execute(
        self,
        key: str,
        target_id: str,
        operation: Callable[[], HttpResult],
        wait_timeout_sec: float,
    ) -> HttpResult:
        with self._lock:
            entry = self._entries.get(key)
            if entry is None:
                entry = _IdempotencyEntry(
                    target_id=target_id,
                    completed=threading.Event(),
                )
                self._entries[key] = entry
                is_owner = True
            else:
                is_owner = False

        if entry.target_id != target_id:
            return HttpResult(409, {"error_code": "IDEMPOTENCY_KEY_CONFLICT"})

        if is_owner:
            try:
                result = operation()
            except Exception:
                _LOG.exception("idempotent operation failed key=%s target=%s", key, target_id)
                result = HttpResult(500, {"error_code": "EXECUTION_FAILED"})
            with self._lock:
                entry.result = result
                entry.completed.set()
            return result

        if not entry.completed.wait(timeout=max(0.0, wait_timeout_sec)):
            return HttpResult(504, {"error_code": "EXECUTION_FAILED"})

        with self._lock:
            return entry.result or HttpResult(
                500, {"error_code": "EXECUTION_FAILED"}
            )
