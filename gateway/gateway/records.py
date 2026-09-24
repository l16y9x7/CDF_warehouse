"""Gateway 操作记录：只记接没接住，不补硬件成功。"""

from __future__ import annotations

import threading
import time
from collections import deque
from dataclasses import asdict, dataclass
from typing import Any, Dict, List


@dataclass(frozen=True)
class CommandRecord:
    timestamp_ms: int
    method: str
    bid: str
    tid: str
    task_id: str
    scenario: str
    accepted: bool
    code: int
    message: str
    error_code: str


class CommandLog:
    def __init__(self, *, max_entries: int = 2000) -> None:
        self._lock = threading.Lock()
        self._items: deque[CommandRecord] = deque(maxlen=max(1, int(max_entries)))

    def add(self, record: CommandRecord) -> None:
        with self._lock:
            self._items.append(record)

    def list(self, *, task_id: str = "", limit: int = 100) -> List[Dict[str, Any]]:
        with self._lock:
            items = list(self._items)
        if task_id:
            items = [item for item in items if item.task_id == task_id]
        limit = max(1, min(int(limit), 1000))
        return [asdict(item) for item in items[-limit:]]


class TaskStateStore:
    """OSD 任务摘要的回退源。场景 /state 优先；手动强制覆盖时除外。"""

    def __init__(self) -> None:
        self._lock = threading.Lock()
        self._task_id = ""
        self._status = 0
        self._error = ""
        self._forced = False

    @property
    def forced(self) -> bool:
        with self._lock:
            return self._forced

    def on_accepted(self, task_id: str) -> None:
        task_id = str(task_id or "").strip()
        if not task_id:
            return
        with self._lock:
            self._task_id = task_id
            self._status = 1
            self._error = ""

    def on_stop_accepted(self, task_id: str) -> None:
        task_id = str(task_id or "").strip()
        with self._lock:
            if task_id and self._task_id and task_id != self._task_id:
                return
            if self._status == 1:
                self._status = 1

    def on_result(
        self,
        task_id: str,
        *,
        status: int,
        error: str = "",
    ) -> None:
        task_id = str(task_id or "").strip()
        if not task_id:
            return
        with self._lock:
            self._task_id = task_id
            self._status = int(status)
            self._error = str(error or "")

    def set(
        self,
        *,
        task_id: str = "",
        status: int = 0,
        error: str = "",
        forced: bool = True,
    ) -> Dict[str, Any]:
        task_id = str(task_id or "").strip()
        status = int(status)
        if status not in {0, 1, 2, 3}:
            raise ValueError("status must be 0, 1, 2 or 3")
        with self._lock:
            self._forced = bool(forced)
            if status == 0:
                self._task_id = ""
                self._status = 0
                self._error = ""
            else:
                if not task_id:
                    raise ValueError("task_id is required when status is not idle")
                self._task_id = task_id
                self._status = status
                self._error = str(error or "")
        return self.snapshot()

    def clear(self) -> Dict[str, Any]:
        return self.set(status=0, forced=False)

    def snapshot(self) -> Dict[str, Any]:
        with self._lock:
            payload: Dict[str, Any] = {
                "task_id": self._task_id if self._status != 0 else "",
                "status": self._status,
            }
            if self._status == 3 and self._error:
                payload["error"] = self._error[:300]
            return payload


def now_ms() -> int:
    return int(time.time() * 1000)
