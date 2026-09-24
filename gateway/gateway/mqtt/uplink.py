"""场景上行：TaskEvent / TaskResult 转发到 MQTT event Topic。"""

from __future__ import annotations

import logging
import threading
from collections import deque
from typing import Any, Deque, Dict, Optional

from gateway.mqtt.envelopes import (
    normalize_task_event,
    normalize_task_result,
    task_status_code,
)
from gateway.records import TaskStateStore

LOGGER = logging.getLogger(__name__)


class UplinkPublisher:
    def __init__(
        self,
        mqtt_client: Any = None,
        *,
        task_state: Optional[TaskStateStore] = None,
        pending_max: int = 200,
    ) -> None:
        self.mqtt_client = mqtt_client
        self.task_state = task_state or TaskStateStore()
        self._pending: Deque[Dict[str, Any]] = deque(maxlen=max(1, int(pending_max)))
        self._lock = threading.Lock()

    def publish_task_event(self, data: Dict[str, Any]) -> Dict[str, Any]:
        event = normalize_task_event(data)
        if not self.task_state.forced:
            self.task_state.on_accepted(event["task_id"])
        LOGGER.info(
            "publish task event: task_id=%s stage=%s step_index=%s",
            event.get("task_id"),
            event.get("stage"),
            event.get("step_index"),
        )
        ok = self._emit(event)
        if ok:
            self.flush_pending()
        if not ok:
            LOGGER.warning(
                "task event not published: task_id=%s stage=%s queued=%s",
                event.get("task_id"),
                event.get("stage"),
                True,
            )
        return {
            "accepted": True,
            "published": ok,
            "data": event,
        }

    def publish_task_result(self, data: Dict[str, Any]) -> Dict[str, Any]:
        event = normalize_task_result(data)
        status = task_status_code(
            str(data.get("terminal_state") or data.get("stage") or event["stage"])
        )
        self.task_state.on_result(
            event["task_id"],
            status=status,
            error=str(data.get("error") or data.get("description") or ""),
        )
        LOGGER.info(
            "publish task result: task_id=%s stage=%s status=%s",
            event["task_id"],
            event["stage"],
            status,
        )
        ok = self._emit(event)
        if ok:
            self.flush_pending()
        if not ok:
            LOGGER.warning(
                "task result not published: task_id=%s stage=%s queued=%s",
                event["task_id"],
                event["stage"],
                True,
            )
        return {
            "accepted": True,
            "published": ok,
            "data": event,
        }

    def flush_pending(self) -> int:
        flushed = 0
        while True:
            with self._lock:
                if not self._pending:
                    return flushed
                event = self._pending[0]
            if not self._emit(event, queued=True):
                return flushed
            with self._lock:
                if self._pending and self._pending[0] is event:
                    self._pending.popleft()
                elif self._pending:
                    try:
                        self._pending.remove(event)
                    except ValueError:
                        pass
            flushed += 1
        return flushed

    def _emit(self, event: Dict[str, Any], *, queued: bool = False) -> bool:
        client = self.mqtt_client
        if client is None:
            LOGGER.warning("MQTT client missing; skip event publish")
            if not queued:
                self._enqueue(event)
            return False
        if not bool(getattr(client, "is_connected", False)):
            if not queued:
                self._enqueue(event)
            return False
        try:
            ok = bool(client.publish_event("task_event", event))
        except Exception as exc:
            LOGGER.warning("failed to publish task event: %s", exc, exc_info=True)
            ok = False
        if not ok and not queued:
            self._enqueue(event)
        return ok

    def _enqueue(self, event: Dict[str, Any]) -> None:
        with self._lock:
            self._pending.append(dict(event))
