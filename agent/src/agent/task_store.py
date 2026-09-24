import json
import sqlite3
import threading
from contextlib import contextmanager
from dataclasses import asdict, is_dataclass
from enum import Enum
from pathlib import Path
from typing import Any

TERMINAL_STATUSES = frozenset({"SUCCEEDED", "FAILED", "WAITING_CONFIRMATION", "CANCELLED"})


class SqliteTaskStore:
    def __init__(self, path: str | Path = "agent-tasks.db") -> None:
        self.path = str(path)
        self._lock = threading.RLock()
        self._initialize()

    def _connect(self) -> sqlite3.Connection:
        connection = sqlite3.connect(self.path, timeout=30)
        connection.row_factory = sqlite3.Row
        return connection

    @contextmanager
    def _connection(self):
        connection = self._connect()
        try:
            with connection:
                yield connection
        finally:
            connection.close()

    def _initialize(self) -> None:
        with self._connection() as connection:
            connection.executescript(
                """
                CREATE TABLE IF NOT EXISTS tasks (
                    task_id TEXT PRIMARY KEY,
                    task_type TEXT NOT NULL,
                    callback_url TEXT NOT NULL,
                    request_json TEXT NOT NULL,
                    status TEXT NOT NULL,
                    result_json TEXT,
                    error_code TEXT,
                    snapshot_json TEXT,
                    created_at TEXT NOT NULL DEFAULT CURRENT_TIMESTAMP,
                    updated_at TEXT NOT NULL DEFAULT CURRENT_TIMESTAMP
                );
                CREATE TABLE IF NOT EXISTS task_nodes (
                    id INTEGER PRIMARY KEY AUTOINCREMENT,
                    task_id TEXT NOT NULL,
                    node_id TEXT NOT NULL,
                    status TEXT NOT NULL,
                    record_json TEXT NOT NULL,
                    created_at TEXT NOT NULL DEFAULT CURRENT_TIMESTAMP
                );
                CREATE TABLE IF NOT EXISTS callback_attempts (
                    id INTEGER PRIMARY KEY AUTOINCREMENT,
                    task_id TEXT NOT NULL,
                    attempt INTEGER NOT NULL,
                    succeeded INTEGER NOT NULL,
                    detail TEXT,
                    created_at TEXT NOT NULL DEFAULT CURRENT_TIMESTAMP
                );
                CREATE TABLE IF NOT EXISTS callback_events (
                    id INTEGER PRIMARY KEY AUTOINCREMENT,
                    task_id TEXT NOT NULL,
                    callback_url TEXT NOT NULL,
                    payload_json TEXT NOT NULL,
                    state TEXT NOT NULL DEFAULT 'PENDING',
                    attempts INTEGER NOT NULL DEFAULT 0,
                    last_error TEXT,
                    created_at TEXT NOT NULL DEFAULT CURRENT_TIMESTAMP,
                    sent_at TEXT
                );
                CREATE INDEX IF NOT EXISTS callback_events_pending
                    ON callback_events(task_id, id, state);
                """
            )

    def create(
        self, task_id: str, task_type: str, callback_url: str, request: dict[str, Any]
    ) -> bool:
        with self._lock, self._connection() as connection:
            cursor = connection.execute(
                "INSERT OR IGNORE INTO tasks(task_id, task_type, callback_url, request_json, status) VALUES (?, ?, ?, ?, 'ACCEPTED')",
                (task_id, task_type, callback_url, _dumps(request)),
            )
            return cursor.rowcount == 1

    def get(self, task_id: str) -> dict[str, Any] | None:
        with self._connection() as connection:
            row = connection.execute("SELECT * FROM tasks WHERE task_id = ?", (task_id,)).fetchone()
        if row is None:
            return None
        result = dict(row)
        for key in ("request_json", "result_json", "snapshot_json"):
            result[key.removesuffix("_json")] = json.loads(result[key]) if result[key] else None
        return result

    def set_status(
        self, task_id: str, status: str, *, result: Any = None, error_code: str | None = None
    ) -> None:
        with self._lock, self._connection() as connection:
            connection.execute(
                "UPDATE tasks SET status = ?, result_json = ?, error_code = ?, updated_at = CURRENT_TIMESTAMP WHERE task_id = ?",
                (status, _dumps(result) if result is not None else None, error_code, task_id),
            )

    def save(self, task_id: str, snapshot: Any) -> None:
        with self._lock, self._connection() as connection:
            connection.execute(
                "UPDATE tasks SET snapshot_json = ?, updated_at = CURRENT_TIMESTAMP WHERE task_id = ?",
                (_dumps(snapshot), task_id),
            )

    def load(self, task_id: str) -> Any | None:
        task = self.get(task_id)
        return None if task is None else task["snapshot"]

    def record_node(self, task_id: str, record: dict[str, Any]) -> None:
        with self._lock, self._connection() as connection:
            connection.execute(
                "INSERT INTO task_nodes(task_id, node_id, status, record_json) VALUES (?, ?, ?, ?)",
                (task_id, record["node_id"], record["status"], _dumps(record)),
            )

    def record_callback(
        self, task_id: str, attempt: int, succeeded: bool, detail: str = ""
    ) -> None:
        with self._lock, self._connection() as connection:
            connection.execute(
                "INSERT INTO callback_attempts(task_id, attempt, succeeded, detail) VALUES (?, ?, ?, ?)",
                (task_id, attempt, int(succeeded), detail),
            )

    def enqueue_callback(
        self, task_id: str, callback_url: str, payload: dict[str, Any]
    ) -> int:
        with self._lock, self._connection() as connection:
            cursor = connection.execute(
                "INSERT INTO callback_events(task_id, callback_url, payload_json) VALUES (?, ?, ?)",
                (task_id, callback_url, _dumps(payload)),
            )
            return int(cursor.lastrowid)

    def next_callback(self, task_id: str) -> dict[str, Any] | None:
        with self._connection() as connection:
            row = connection.execute(
                "SELECT * FROM callback_events WHERE task_id = ? AND state = 'PENDING' ORDER BY id LIMIT 1",
                (task_id,),
            ).fetchone()
        if row is None:
            return None
        result = dict(row)
        result["payload"] = json.loads(result.pop("payload_json"))
        return result

    def pending_callback_tasks(self) -> list[str]:
        with self._connection() as connection:
            rows = connection.execute(
                "SELECT DISTINCT task_id FROM callback_events WHERE state = 'PENDING'"
            ).fetchall()
        return [str(row[0]) for row in rows]

    def record_callback_event_attempt(
        self, event_id: int, task_id: str, attempt: int, succeeded: bool, detail: str = ""
    ) -> None:
        with self._lock, self._connection() as connection:
            connection.execute(
                "UPDATE callback_events SET attempts = ?, last_error = ? WHERE id = ?",
                (attempt, None if succeeded else detail, event_id),
            )
            connection.execute(
                "INSERT INTO callback_attempts(task_id, attempt, succeeded, detail) VALUES (?, ?, ?, ?)",
                (task_id, attempt, int(succeeded), detail),
            )

    def finish_callback_event(self, event_id: int, *, sent: bool, detail: str = "") -> None:
        with self._lock, self._connection() as connection:
            connection.execute(
                "UPDATE callback_events SET state = ?, last_error = ?, sent_at = CASE WHEN ? THEN CURRENT_TIMESTAMP ELSE sent_at END WHERE id = ?",
                ("SENT" if sent else "EXHAUSTED", None if sent else detail, int(sent), event_id),
            )

    def mark_interrupted(self) -> list[dict[str, Any]]:
        with self._lock, self._connection() as connection:
            rows = connection.execute(
                "SELECT * FROM tasks WHERE status IN ('ACCEPTED', 'RUNNING')"
            ).fetchall()
            connection.execute(
                "UPDATE tasks SET status = 'WAITING_CONFIRMATION', error_code = 'PROCESS_INTERRUPTED', updated_at = CURRENT_TIMESTAMP WHERE status IN ('ACCEPTED', 'RUNNING')"
            )
        return [dict(row) for row in rows]


def _dumps(value: Any) -> str:
    def encode(item: Any) -> Any:
        if is_dataclass(item):
            return asdict(item)
        if isinstance(item, Enum):
            return item.value
        raise TypeError(f"cannot serialize {type(item).__name__}")

    return json.dumps(value, ensure_ascii=False, separators=(",", ":"), default=encode)
