import hashlib
import json
import os
import secrets
import sqlite3
import threading
import time
from contextlib import contextmanager
from pathlib import Path
from typing import Any


def canonical_json(value: Any) -> str:
    return json.dumps(value, ensure_ascii=False, separators=(",", ":"), sort_keys=True)


def request_digest(target: str, layer: str, operation: str, payload: dict[str, Any]) -> str:
    raw = canonical_json(
        {"target": target, "layer": layer, "operation": operation, "payload": payload}
    )
    return hashlib.sha256(raw.encode("utf-8")).hexdigest()


class DebugStore:
    def __init__(self, path: str | Path) -> None:
        self.path = str(path)
        self._lock = threading.RLock()
        self._last_trace_cleanup = 0.0
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
                CREATE TABLE IF NOT EXISTS debug_runs (
                    run_id TEXT PRIMARY KEY,
                    target TEXT NOT NULL,
                    layer TEXT NOT NULL,
                    operation TEXT NOT NULL,
                    status TEXT NOT NULL,
                    request_json TEXT NOT NULL,
                    result_json TEXT,
                    events_json TEXT,
                    error_code TEXT,
                    message TEXT,
                    info_json TEXT,
                    task_id TEXT,
                    started_at REAL NOT NULL,
                    finished_at REAL,
                    duration_ms REAL
                );
                CREATE INDEX IF NOT EXISTS debug_runs_started_at ON debug_runs(started_at DESC);
                CREATE TABLE IF NOT EXISTS debug_confirmations (
                    token TEXT PRIMARY KEY,
                    request_hash TEXT NOT NULL,
                    expires_at REAL NOT NULL,
                    used_at REAL
                );
                CREATE TABLE IF NOT EXISTS execution_spans (
                    span_id TEXT PRIMARY KEY,
                    run_id TEXT NOT NULL,
                    parent_span_id TEXT,
                    sequence INTEGER NOT NULL,
                    kind TEXT NOT NULL,
                    name TEXT NOT NULL,
                    operation TEXT NOT NULL,
                    node_id TEXT,
                    status TEXT NOT NULL,
                    started_at REAL NOT NULL,
                    finished_at REAL,
                    duration_ms REAL,
                    input_json TEXT,
                    output_json TEXT,
                    error_code TEXT,
                    error_type TEXT,
                    error_message TEXT,
                    error_stack TEXT,
                    fields_json TEXT
                );
                CREATE INDEX IF NOT EXISTS execution_spans_run
                    ON execution_spans(run_id, started_at, sequence);
                CREATE INDEX IF NOT EXISTS execution_spans_finished
                    ON execution_spans(finished_at);
                """
            )
            columns = {row[1] for row in connection.execute("PRAGMA table_info(debug_runs)")}
            if "info_json" not in columns:
                connection.execute("ALTER TABLE debug_runs ADD COLUMN info_json TEXT")
            span_columns = {
                row[1] for row in connection.execute("PRAGMA table_info(execution_spans)")
            }
            if "error_stack" not in span_columns:
                connection.execute("ALTER TABLE execution_spans ADD COLUMN error_stack TEXT")

    def create_run(
        self,
        run_id: str,
        target: str,
        layer: str,
        operation: str,
        payload: dict[str, Any],
        *,
        task_id: str | None = None,
    ) -> None:
        with self._lock, self._connection() as connection:
            connection.execute(
                """INSERT INTO debug_runs
                   (run_id, target, layer, operation, status, request_json, task_id, started_at)
                   VALUES (?, ?, ?, ?, 'RUNNING', ?, ?, ?)""",
                (run_id, target, layer, operation, canonical_json(payload), task_id, time.time()),
            )

    def set_status(
        self,
        run_id: str,
        status: str,
        *,
        result: Any = None,
        events: list[Any] | None = None,
        error_code: str | None = None,
        message: str | None = None,
        info: dict[str, Any] | None = None,
        finished: bool = True,
    ) -> None:
        now = time.time()
        with self._lock, self._connection() as connection:
            row = connection.execute(
                "SELECT started_at FROM debug_runs WHERE run_id = ?", (run_id,)
            ).fetchone()
            if row is None:
                raise KeyError(run_id)
            connection.execute(
                """UPDATE debug_runs SET status = ?, result_json = ?,
                   events_json = COALESCE(?, events_json),
                   error_code = ?, message = ?, info_json = ?, finished_at = ?, duration_ms = ? WHERE run_id = ?""",
                (
                    status,
                    canonical_json(result) if result is not None else None,
                    canonical_json(events) if events is not None else None,
                    error_code,
                    message,
                    canonical_json(info) if info is not None else None,
                    now if finished else None,
                    round((now - row["started_at"]) * 1000, 3) if finished else None,
                    run_id,
                ),
            )

    def attach_events(self, task_id: str, events: list[Any]) -> None:
        with self._lock, self._connection() as connection:
            connection.execute(
                "UPDATE debug_runs SET events_json = ? WHERE task_id = ?",
                (canonical_json(events), task_id),
            )

    def attach_span(self, run_id: str, span: dict[str, Any]) -> None:
        known = {
            "span_id", "parent_span_id", "sequence", "kind", "name", "operation",
            "node_id", "status", "started_at", "finished_at", "duration_ms", "input",
            "output", "error_code", "error_type", "error_message",
            "error_stack",
        }
        fields = {key: value for key, value in span.items() if key not in known}
        try:
            with self._lock, self._connection() as connection:
                connection.execute(
                    """INSERT INTO execution_spans
                       (span_id, run_id, parent_span_id, sequence, kind, name, operation,
                        node_id, status, started_at, finished_at, duration_ms, input_json,
                        output_json, error_code, error_type, error_message, error_stack, fields_json)
                       VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?)
                       ON CONFLICT(span_id) DO UPDATE SET
                        status=excluded.status, finished_at=excluded.finished_at,
                        duration_ms=excluded.duration_ms, output_json=excluded.output_json,
                        error_code=excluded.error_code, error_type=excluded.error_type,
                        error_message=excluded.error_message, error_stack=excluded.error_stack,
                        fields_json=excluded.fields_json""",
                    (
                        span["span_id"], run_id, span.get("parent_span_id"), span["sequence"],
                        span["kind"], span["name"], span["operation"], span.get("node_id"),
                        span["status"], span["started_at"], span.get("finished_at"),
                        span.get("duration_ms"), canonical_json(span.get("input")),
                        canonical_json(span.get("output")), span.get("error_code"),
                        span.get("error_type"), span.get("error_message"),
                        span.get("error_stack"), canonical_json(fields),
                    ),
                )
        except (OSError, sqlite3.Error, TypeError, ValueError):
            return
        if time.monotonic() - self._last_trace_cleanup >= 3600:
            self.cleanup_traces()
            self._last_trace_cleanup = time.monotonic()

    def run_id_for_task(self, task_id: str) -> str | None:
        with self._connection() as connection:
            row = connection.execute(
                "SELECT run_id FROM debug_runs WHERE task_id = ? ORDER BY started_at DESC LIMIT 1",
                (task_id,),
            ).fetchone()
        return None if row is None else str(row["run_id"])

    def attach_task_span(self, task_id: str, span: dict[str, Any]) -> None:
        run_id = self.run_id_for_task(task_id)
        if run_id is not None:
            self.attach_span(run_id, span)

    def trace(self, run_id: str) -> dict[str, Any]:
        with self._connection() as connection:
            rows = connection.execute(
                "SELECT * FROM execution_spans WHERE run_id = ? "
                "ORDER BY started_at, sequence, rowid", (run_id,)
            ).fetchall()
            run = connection.execute(
                "SELECT started_at FROM debug_runs WHERE run_id = ?", (run_id,)
            ).fetchone()
        spans = []
        for row in rows:
            item = dict(row)
            for key in ("input_json", "output_json", "fields_json"):
                value = json.loads(item.pop(key)) if item.get(key) else None
                if key == "fields_json":
                    item.update(value or {})
                else:
                    item[key.removesuffix("_json")] = value
            spans.append(item)
        retention = max(1, int(os.getenv("AGENT_TRACE_RETENTION_DAYS", "7")))
        expired = bool(run and not spans and run["started_at"] < time.time() - retention * 86400)
        return {"trace_id": run_id, "expired": expired, "spans": spans}

    def cleanup_traces(self) -> None:
        retention = max(1, int(os.getenv("AGENT_TRACE_RETENTION_DAYS", "7")))
        max_bytes = max(1, int(os.getenv("AGENT_TRACE_MAX_MB", "200"))) << 20
        cutoff = time.time() - retention * 86400
        with self._lock, self._connection() as connection:
            connection.execute(
                "DELETE FROM execution_spans WHERE finished_at IS NOT NULL AND finished_at < ?",
                (cutoff,),
            )
            size = connection.execute(
                """SELECT COALESCE(SUM(
                    LENGTH(COALESCE(input_json, '')) + LENGTH(COALESCE(output_json, '')) +
                    LENGTH(COALESCE(fields_json, '')) + LENGTH(COALESCE(error_message, ''))
                ), 0) FROM execution_spans"""
            ).fetchone()[0]
            while size > max_bytes:
                oldest = connection.execute(
                    "SELECT run_id FROM execution_spans WHERE finished_at IS NOT NULL "
                    "GROUP BY run_id ORDER BY MIN(started_at) LIMIT 1"
                ).fetchone()
                if oldest is None:
                    break
                connection.execute(
                    "DELETE FROM execution_spans WHERE run_id = ?", (oldest["run_id"],)
                )
                size = connection.execute(
                    """SELECT COALESCE(SUM(
                        LENGTH(COALESCE(input_json, '')) + LENGTH(COALESCE(output_json, '')) +
                        LENGTH(COALESCE(fields_json, '')) + LENGTH(COALESCE(error_message, ''))
                    ), 0) FROM execution_spans"""
                ).fetchone()[0]

    def attach_run_events(self, run_id: str, events: list[Any]) -> None:
        with self._lock, self._connection() as connection:
            connection.execute(
                "UPDATE debug_runs SET events_json = ? WHERE run_id = ?",
                (canonical_json(events), run_id),
            )

    def complete_callback(self, run_id: str, payload: dict[str, Any]) -> None:
        status = str(payload.get("status", "FAILED"))
        info = payload.get("info")
        terminal = status in {"SUCCEEDED", "FAILED", "CANCELLED"}
        self.set_status(
            run_id,
            status,
            result=(info.get("result") if isinstance(info, dict) else payload.get("result")),
            error_code=payload.get("error_code"),
            message=(info or {}).get("message") if isinstance(info, dict) else payload.get("message"),
            info=info if isinstance(info, dict) else None,
            finished=terminal,
        )

    def get_run(self, run_id: str) -> dict[str, Any] | None:
        with self._connection() as connection:
            row = connection.execute(
                "SELECT * FROM debug_runs WHERE run_id = ?", (run_id,)
            ).fetchone()
        return None if row is None else self._decode_run(row)

    def list_runs(
        self,
        *,
        target: str | None = None,
        layer: str | None = None,
        status: str | None = None,
        limit: int = 100,
    ) -> list[dict[str, Any]]:
        clauses: list[str] = []
        values: list[Any] = []
        for key, value in (("target", target), ("layer", layer), ("status", status)):
            if value:
                clauses.append(f"{key} = ?")
                values.append(value)
        where = f" WHERE {' AND '.join(clauses)}" if clauses else ""
        values.append(max(1, min(limit, 500)))
        with self._connection() as connection:
            rows = connection.execute(
                f"SELECT * FROM debug_runs{where} ORDER BY started_at DESC LIMIT ?", values
            ).fetchall()
        return [self._decode_run(row) for row in rows]

    def create_confirmation(
        self,
        target: str,
        layer: str,
        operation: str,
        payload: dict[str, Any],
        *,
        ttl_seconds: int = 60,
    ) -> tuple[str, float]:
        token = secrets.token_urlsafe(24)
        expires_at = time.time() + ttl_seconds
        digest = request_digest(target, layer, operation, payload)
        with self._lock, self._connection() as connection:
            connection.execute(
                "INSERT INTO debug_confirmations(token, request_hash, expires_at) VALUES (?, ?, ?)",
                (token, digest, expires_at),
            )
        return token, expires_at

    def consume_confirmation(
        self,
        token: str,
        target: str,
        layer: str,
        operation: str,
        payload: dict[str, Any],
    ) -> None:
        digest = request_digest(target, layer, operation, payload)
        now = time.time()
        with self._lock, self._connection() as connection:
            row = connection.execute(
                "SELECT request_hash, expires_at, used_at FROM debug_confirmations WHERE token = ?",
                (token,),
            ).fetchone()
            if row is None:
                raise ValueError("confirmation token is invalid")
            if row["used_at"] is not None:
                raise ValueError("confirmation token was already used")
            if row["expires_at"] < now:
                raise ValueError("confirmation token has expired")
            if row["request_hash"] != digest:
                raise ValueError("confirmation token does not match this request")
            connection.execute(
                "UPDATE debug_confirmations SET used_at = ? WHERE token = ?", (now, token)
            )

    @staticmethod
    def _decode_run(row: sqlite3.Row) -> dict[str, Any]:
        result = dict(row)
        for key in ("request_json", "result_json", "events_json", "info_json"):
            result[key.removesuffix("_json")] = json.loads(result[key]) if result[key] else None
            result.pop(key, None)
        return result
