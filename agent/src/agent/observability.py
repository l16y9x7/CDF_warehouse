"""中文结构化日志、日志上下文和本地查询索引。"""

from __future__ import annotations

import atexit
import contextvars
import json
import logging
import logging.handlers
import os
import queue
import re
import sqlite3
import threading
import time
from collections.abc import Iterator, Mapping
from contextlib import contextmanager
from datetime import datetime, timezone
from pathlib import Path
from typing import Any

_CONTEXT: contextvars.ContextVar[dict[str, Any] | None] = contextvars.ContextVar(
    "agent_log_context", default=None
)
_RESERVED = set(logging.makeLogRecord({}).__dict__) | {"message", "asctime"}
_SENSITIVE = re.compile(
    r"authorization|token|secret|password|cookie|callback_url|confirmation", re.IGNORECASE
)
_URL_CREDENTIALS = re.compile(r"(?i)(https?://)([^/@\s]+)@")
_URL_QUERY = re.compile(r"(https?://[^?\s]+)\?[^\s]+", re.IGNORECASE)
_URL = re.compile(r"https?://[^\s]+", re.IGNORECASE)
_MAX_STRING = 2_000

STATUS_TEXT = {
    "ACCEPTED": "已受理",
    "RUNNING": "执行中",
    "SUCCEEDED": "执行成功",
    "FAILED": "执行失败",
    "WAITING_CONFIRMATION": "等待人工确认",
    "CANCELLED": "已取消",
}

EVENT_MESSAGES = {
    "workflow.started": "工作流开始执行",
    "workflow.succeeded": "工作流执行成功",
    "workflow.failed": "工作流执行失败",
    "workflow.cancelled": "工作流已取消",
    "workflow.cancellation_requested": "已收到工作流终止请求",
    "workflow.node.started": "工作流节点开始执行",
    "workflow.node.succeeded": "工作流节点执行成功",
    "workflow.node.failed": "工作流节点执行失败",
    "skill.started": "技能开始执行",
    "skill.succeeded": "技能执行成功",
    "skill.failed": "技能执行失败",
    "camera.captured": "相机拍摄完成",
}


def _safe(value: Any, *, key: str = "") -> Any:
    if _SENSITIVE.search(key):
        return "[已脱敏]"
    if value is None or isinstance(value, (bool, int, float)):
        return value
    if isinstance(value, str):
        value = _URL_CREDENTIALS.sub(r"\1[已脱敏]@", value)
        value = _URL_QUERY.sub(r"\1?[查询参数已脱敏]", value)
        value = _URL.sub("[URL已脱敏]", value)
        return value if len(value) <= _MAX_STRING else value[:_MAX_STRING] + "…[已截断]"
    if isinstance(value, Mapping):
        return {str(k): _safe(v, key=str(k)) for k, v in value.items()}
    if isinstance(value, (list, tuple, set)):
        return [_safe(item) for item in list(value)[:100]]
    return _safe(str(value), key=key)


def status_text(status: Any) -> str | None:
    return STATUS_TEXT.get(str(status)) if status is not None else None


@contextmanager
def log_context(**fields: Any) -> Iterator[None]:
    merged = {
        **(_CONTEXT.get() or {}),
        **{key: value for key, value in fields.items() if value is not None},
    }
    token = _CONTEXT.set(merged)
    try:
        yield
    finally:
        _CONTEXT.reset(token)


def current_log_context() -> dict[str, Any]:
    return dict(_CONTEXT.get() or {})


def event_message(event: str, fields: Mapping[str, Any]) -> str:
    base = EVENT_MESSAGES.get(event, "Agent 运行事件")
    target = fields.get("skill") or fields.get("workflow_node") or fields.get("workflow")
    return f"{target}：{base}" if target else base  # noqa: RUF001


def log_event(
    logger: logging.Logger,
    log_level: int,
    event: str,
    message: str,
    *,
    exc_info: Any = None,
    **fields: Any,
) -> None:
    extra = {"event": event, **(_CONTEXT.get() or {}), **fields}
    if "status" in extra and "status_text" not in extra:
        extra["status_text"] = status_text(extra["status"])
    try:
        logger.log(
            log_level, message,
            extra={key: _safe(value, key=key) for key, value in extra.items()},
            exc_info=exc_info,
        )
    except Exception:
        # 可观测性属于辅助能力, 任何格式或字段问题都不能打断机器人业务。
        return


class JsonFormatter(logging.Formatter):
    def format_record(self, record: logging.LogRecord) -> dict[str, Any]:
        timestamp = datetime.fromtimestamp(record.created, timezone.utc).isoformat(
            timespec="milliseconds"
        ).replace("+00:00", "Z")
        payload: dict[str, Any] = {
            "timestamp": timestamp,
            "level": record.levelname,
            "logger": record.name,
            "event": getattr(record, "event", "application.message"),
            "message": _safe(record.getMessage()),
            "service": "robot-agent",
            "version": "0.1.0",
            "pid": record.process,
            "thread": record.threadName,
        }
        for key, value in record.__dict__.items():
            if key not in _RESERVED and key not in payload and not key.startswith("_"):
                payload[key] = _safe(value, key=key)
        if record.exc_info:
            payload["exception_type"] = record.exc_info[0].__name__
            payload["exception"] = _safe(self.formatException(record.exc_info))
        return payload

    def format(self, record: logging.LogRecord) -> str:
        return json.dumps(self.format_record(record), ensure_ascii=False, separators=(",", ":"))


class LogStore:
    """独立于业务库的可查询日志索引。"""

    def __init__(self, path: str | Path, *, retention_days: int = 7, max_bytes: int = 100 << 20):
        self.path = str(path)
        self.retention_days = retention_days
        self.max_bytes = max_bytes
        self._lock = threading.RLock()
        Path(self.path).parent.mkdir(parents=True, exist_ok=True)
        self._initialize()

    def _connect(self) -> sqlite3.Connection:
        connection = sqlite3.connect(self.path, timeout=5)
        connection.row_factory = sqlite3.Row
        connection.execute("PRAGMA busy_timeout=5000")
        return connection

    @contextmanager
    def _connection(self) -> Iterator[sqlite3.Connection]:
        connection = self._connect()
        try:
            with connection:
                yield connection
        finally:
            connection.close()

    def _initialize(self) -> None:
        with self._connection() as connection:
            connection.execute("PRAGMA journal_mode=WAL")
            connection.execute("PRAGMA auto_vacuum=INCREMENTAL")
            connection.executescript(
                """
                CREATE TABLE IF NOT EXISTS logs (
                    id INTEGER PRIMARY KEY AUTOINCREMENT,
                    timestamp TEXT NOT NULL, created_at REAL NOT NULL,
                    level TEXT NOT NULL, logger TEXT NOT NULL, event TEXT NOT NULL,
                    message TEXT NOT NULL, request_id TEXT, task_id TEXT, run_id TEXT,
                    workflow TEXT, node_id TEXT, skill TEXT, capability TEXT,
                    span_id TEXT,
                    status TEXT, error_code TEXT, error_message TEXT, suggestion TEXT,
                    record_json TEXT NOT NULL
                );
                CREATE INDEX IF NOT EXISTS logs_created_at ON logs(created_at DESC, id DESC);
                CREATE INDEX IF NOT EXISTS logs_task_id ON logs(task_id, created_at DESC);
                CREATE INDEX IF NOT EXISTS logs_run_id ON logs(run_id, created_at DESC);
                CREATE INDEX IF NOT EXISTS logs_request_id ON logs(request_id, created_at DESC);
                """
            )
            columns = {row[1] for row in connection.execute("PRAGMA table_info(logs)")}
            for column in ("error_message", "suggestion", "span_id"):
                if column not in columns:
                    connection.execute(f"ALTER TABLE logs ADD COLUMN {column} TEXT")
            connection.execute(
                "CREATE INDEX IF NOT EXISTS logs_span_id ON logs(span_id, created_at DESC)"
            )

    def append(self, payload: Mapping[str, Any], created_at: float) -> None:
        with self._lock, self._connection() as connection:
            connection.execute(
                """INSERT INTO logs
                (timestamp, created_at, level, logger, event, message, request_id, task_id,
                 run_id, workflow, node_id, skill, capability, status, error_code,
                 span_id, error_message, suggestion, record_json)
                VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?)""",
                (
                    payload["timestamp"], created_at, payload["level"], payload["logger"],
                    payload["event"], payload["message"], payload.get("request_id"),
                    payload.get("task_id"), payload.get("run_id"), payload.get("workflow"),
                    payload.get("node_id") or payload.get("workflow_node"), payload.get("skill"),
                    payload.get("capability"), payload.get("status"), payload.get("error_code"),
                    payload.get("span_id"),
                    payload.get("error_message"), payload.get("suggestion"),
                    json.dumps(payload, ensure_ascii=False, separators=(",", ":")),
                ),
            )

    def query(
        self, *, limit: int = 100, cursor: int | None = None, from_time: float | None = None,
        to_time: float | None = None, search: str | None = None, **filters: Any,
    ) -> tuple[list[dict[str, Any]], str | None]:
        clauses: list[str] = []
        values: list[Any] = []
        allowed = {"level", "event", "task_id", "request_id", "run_id", "span_id",
                   "workflow", "skill", "capability"}
        for key, value in filters.items():
            if key == "component" and value:
                clauses.append("(workflow = ? OR skill = ? OR capability = ?)")
                values.extend([str(value)] * 3)
                continue
            if key in allowed and value:
                clauses.append(f"{key} = ?")
                values.append(str(value))
        if cursor is not None:
            clauses.append("id < ?")
            values.append(cursor)
        if from_time is not None:
            clauses.append("created_at >= ?")
            values.append(from_time)
        if to_time is not None:
            clauses.append("created_at <= ?")
            values.append(to_time)
        if search:
            clauses.append(
                "(message LIKE ? ESCAPE '\\' OR error_message LIKE ? ESCAPE '\\' "
                "OR suggestion LIKE ? ESCAPE '\\' OR error_code LIKE ? ESCAPE '\\')"
            )
            escaped = search.replace("\\", "\\\\").replace("%", "\\%").replace("_", "\\_")
            values.extend([f"%{escaped}%"] * 4)
        where = " WHERE " + " AND ".join(clauses) if clauses else ""
        size = max(1, min(int(limit), 500))
        with self._connection() as connection:
            rows = connection.execute(
                f"SELECT id, record_json FROM logs{where} ORDER BY id DESC LIMIT ?",
                [*values, size + 1],
            ).fetchall()
        more = len(rows) > size
        rows = rows[:size]
        records = [json.loads(row["record_json"]) for row in rows]
        return records, str(rows[-1]["id"]) if more and rows else None

    def cleanup(self) -> None:
        cutoff = time.time() - self.retention_days * 86400
        with self._lock, self._connection() as connection:
            connection.execute(
                "DELETE FROM logs WHERE id IN "
                "(SELECT id FROM logs WHERE created_at < ? LIMIT 5000)",
                (cutoff,),
            )
            connection.execute("PRAGMA incremental_vacuum(100)")
        try:
            size = Path(self.path).stat().st_size
        except OSError:
            return
        while size > self.max_bytes:
            with self._lock, self._connection() as connection:
                changed = connection.execute(
                    "DELETE FROM logs WHERE id IN "
                    "(SELECT id FROM logs ORDER BY id LIMIT 5000)"
                ).rowcount
                connection.execute("PRAGMA incremental_vacuum(500)")
            if not changed:
                break
            try:
                size = Path(self.path).stat().st_size
            except OSError:
                break


class _SQLiteHandler(logging.Handler):
    def __init__(self, store: LogStore, formatter: JsonFormatter):
        super().__init__(logging.INFO)
        self.store = store
        self.json_formatter = formatter
        self.last_cleanup = 0.0

    def emit(self, record: logging.LogRecord) -> None:
        try:
            payload = self.json_formatter.format_record(record)
            self.store.append(payload, record.created)
            if time.monotonic() - self.last_cleanup >= 3600:
                self.store.cleanup()
                self.last_cleanup = time.monotonic()
        except Exception:
            # 原始 JSONL 仍然可用; 索引故障不能影响业务或制造递归日志。
            return


class _SafeQueueHandler(logging.handlers.QueueHandler):
    def __init__(self, log_queue: queue.Queue[logging.LogRecord]):
        super().__init__(log_queue)
        self._dropped = 0
        self._last_report = 0.0
        self._drop_lock = threading.Lock()

    def enqueue(self, record: logging.LogRecord) -> None:
        try:
            self.queue.put_nowait(record)
        except queue.Full:
            now = time.monotonic()
            with self._drop_lock:
                self._dropped += 1
                dropped = self._dropped
                should_report = now - self._last_report >= 60
                if should_report:
                    self._dropped = 0
                    self._last_report = now
            if should_report:
                try:
                    notice = f"日志队列已满，本周期丢弃 {dropped} 条日志\n"  # noqa: RUF001
                    os.write(2, notice.encode())
                except OSError:
                    pass
            if record.levelno >= logging.ERROR:
                try:
                    warning = f"日志队列已满，重要日志未入队：{record.getMessage()}\n"  # noqa: RUF001
                    os.write(2, warning.encode())
                except OSError:
                    pass


class LoggingManager:
    def __init__(
        self, log_dir: Path, store: LogStore, *, level: int, queue_size: int,
        file_budget_mb: int,
    ):
        self.log_dir = log_dir
        self.store = store
        log_dir.mkdir(parents=True, exist_ok=True)
        formatter = JsonFormatter()
        segment_mb = min(50, max(1, file_budget_mb // 2))
        backup_count = max(1, file_budget_mb // segment_mb - 1)
        file_handler = logging.handlers.RotatingFileHandler(
            log_dir / "agent.jsonl", maxBytes=segment_mb << 20,
            backupCount=backup_count, encoding="utf-8",
        )
        file_handler.setFormatter(formatter)
        handlers: list[logging.Handler] = [file_handler, _SQLiteHandler(store, formatter)]
        if os.getenv("AGENT_LOG_CONSOLE", "true").lower() in {"1", "true", "yes"}:
            console = logging.StreamHandler()
            console.setFormatter(formatter)
            handlers.append(console)
        self.queue: queue.Queue[logging.LogRecord] = queue.Queue(maxsize=queue_size)
        self.handlers = handlers
        self.queue_handler = _SafeQueueHandler(self.queue)
        self.listener = logging.handlers.QueueListener(self.queue, *handlers, respect_handler_level=True)
        root = logging.getLogger()
        root.handlers.clear()
        root.addHandler(self.queue_handler)
        root.setLevel(level)
        logging.getLogger("httpx").setLevel(logging.WARNING)
        logging.getLogger("httpcore").setLevel(logging.WARNING)
        self.listener.start()
        self.active = True
        self._cleanup_files()

    def _cleanup_files(self) -> None:
        cutoff = time.time() - int(os.getenv("AGENT_LOG_RETENTION_DAYS", "7")) * 86400
        for path in self.log_dir.glob("agent.jsonl.*"):
            try:
                if path.stat().st_mtime < cutoff:
                    path.unlink()
            except OSError:
                continue
        self.store.cleanup()

    def shutdown(self) -> None:
        if not self.active:
            return
        self.active = False
        self.listener.stop()
        root = logging.getLogger()
        root.removeHandler(self.queue_handler)
        if not root.handlers:
            root.addHandler(logging.NullHandler())
        for handler in self.handlers:
            handler.close()


_manager: LoggingManager | None = None
_manager_lock = threading.Lock()


def configure_logging(*, database_path: str | Path, log_dir: str | Path | None = None) -> LoggingManager:
    global _manager
    db_path = Path(database_path)
    directory = Path(log_dir or os.getenv("AGENT_LOG_DIR", str(db_path.parent / "logs")))
    index_path = Path(os.getenv("AGENT_LOG_DATABASE_PATH", str(db_path.with_name("agent-logs.db"))))
    level = getattr(logging, os.getenv("AGENT_LOG_LEVEL", "INFO").upper(), logging.INFO)
    total_mb = max(100, int(os.getenv("AGENT_LOG_MAX_TOTAL_MB", "500")))
    retention = max(1, int(os.getenv("AGENT_LOG_RETENTION_DAYS", "7")))
    with _manager_lock:
        if (
            _manager is not None and _manager.active
            and Path(_manager.store.path) == index_path
        ):
            return _manager
        if _manager is not None:
            _manager.shutdown()
        store = LogStore(index_path, retention_days=retention, max_bytes=(total_mb // 5) << 20)
        _manager = LoggingManager(
            directory, store, level=level,
            queue_size=max(100, int(os.getenv("AGENT_LOG_QUEUE_SIZE", "10000"))),
            file_budget_mb=total_mb * 4 // 5,
        )
        return _manager


def shutdown_logging() -> None:
    global _manager
    with _manager_lock:
        if _manager is not None:
            _manager.shutdown()
            _manager = None


atexit.register(shutdown_logging)
