import json
import logging
import re
import time
import traceback
import uuid
from collections.abc import Callable, Iterator, Mapping
from contextlib import contextmanager
from contextvars import ContextVar
from dataclasses import asdict, dataclass, field, is_dataclass
from enum import Enum
from threading import Event
from typing import Any

from agent.observability import event_message, log_event

_ACTIVE_CONTEXT: ContextVar["ExecutionContext | None"] = ContextVar(
    "agent_execution_context", default=None
)
_TRACE_SENSITIVE = re.compile(
    r"authorization|token|secret|password|cookie|callback_url|confirmation", re.IGNORECASE
)
_TRACE_MAX_STRING = 1 << 20
_TRACE_URL_CREDENTIALS = re.compile(r"(?i)(https?://)([^/@\s]+)@")
_TRACE_URL_QUERY = re.compile(r"(https?://[^?\s]+)\?[^\s]+", re.IGNORECASE)


def _trace_value(value: Any, *, key: str = "") -> Any:
    if _TRACE_SENSITIVE.search(key):
        return "[已脱敏]"
    if value is None or isinstance(value, (bool, int, float, str)):
        if isinstance(value, str) and len(value) > _TRACE_MAX_STRING:
            value = value[:_TRACE_MAX_STRING] + "…[已截断]"
        if isinstance(value, str):
            value = _TRACE_URL_CREDENTIALS.sub(r"\1[已脱敏]@", value)
            value = _TRACE_URL_QUERY.sub(r"\1?[查询参数已脱敏]", value)
        return value
    if isinstance(value, Enum):
        return _trace_value(value.value, key=key)
    if is_dataclass(value):
        return _trace_value(asdict(value), key=key)
    if isinstance(value, Mapping):
        return {str(k): _trace_value(v, key=str(k)) for k, v in value.items()}
    if isinstance(value, (list, tuple, set)):
        return [_trace_value(item) for item in list(value)[:1000]]
    if isinstance(value, bytes):
        return {"type": "bytes", "size": len(value)}
    return str(value)


def trace_value(value: Any, *, key: str = "") -> Any:
    """Make a bounded, JSON-safe trace payload while retaining business data."""
    result = _trace_value(value, key=key)
    encoded = json.dumps(result, ensure_ascii=False, separators=(",", ":")).encode()
    if len(encoded) <= _TRACE_MAX_STRING:
        return result
    return {
        "truncated": True,
        "original_size_bytes": len(encoded),
        "preview": encoded[:_TRACE_MAX_STRING].decode("utf-8", errors="ignore"),
    }


def current_execution_context() -> "ExecutionContext | None":
    return _ACTIVE_CONTEXT.get()


def reportable_error_code(error: BaseException) -> str:
    return str(
        getattr(error, "error_code", None) or getattr(error, "code", None) or type(error).__name__
    )


def _trace_error_fields(error: BaseException) -> dict[str, Any]:
    fields: dict[str, Any] = {
        "error_code": reportable_error_code(error),
        "error_type": type(error).__name__,
        "error_message": trace_value(str(error)),
        "error_stack": trace_value("".join(traceback.format_exception(error))),
    }
    source = getattr(error, "source", None)
    if source is not None:
        fields["error_source"] = getattr(source, "value", source)
    status_code = getattr(error, "status_code", None)
    if status_code is not None:
        fields["http_status"] = status_code
    operation = getattr(error, "operation", None)
    if operation:
        fields["error_operation"] = operation
    return fields

logger = logging.getLogger(__name__)


class AgentError(Exception):
    """An expected, reportable agent failure."""

    def __init__(self, code: str, message: str):
        super().__init__(message)
        self.code = code
        self.message = message


@dataclass
class ExecutionContext:
    task_id: str
    metadata: dict[str, Any] = field(default_factory=dict)
    events: list[Mapping[str, Any]] = field(default_factory=list)
    cancelled: bool = False
    deadline: float | None = None
    event_handler: Callable[[Mapping[str, Any]], None] | None = None
    trace_handler: Callable[[Mapping[str, Any]], None] | None = None
    trace_spans: list[dict[str, Any]] = field(default_factory=list)
    _timers: dict[str, float] = field(default_factory=dict, init=False, repr=False)
    _cancel_event: Event = field(default_factory=Event, init=False, repr=False)
    _trace_stack: list[str] = field(default_factory=list, init=False, repr=False)
    _trace_counter: int = field(default=0, init=False, repr=False)

    def __post_init__(self) -> None:
        if self.cancelled:
            self._cancel_event.set()

    def cancel(self) -> None:
        self.cancelled = True
        self._cancel_event.set()

    @property
    def is_cancelled(self) -> bool:
        return self.cancelled or self._cancel_event.is_set()

    def emit(self, event: str, **data: Any) -> None:
        record = {"event": event, "task_id": self.task_id, **self.metadata, **data}
        self.events.append(record)
        level = logging.ERROR if event.endswith(".failed") else logging.INFO
        fields = {key: value for key, value in record.items() if key != "event"}
        if event == "camera.captured":
            fields.pop("color", None)
            fields.pop("depth", None)
        if "message" in fields:
            fields["error_message"] = fields.pop("message")
        log_event(logger, level, event, event_message(event, fields), **fields)
        if self.event_handler is not None:
            self.event_handler(record)

    @contextmanager
    def activate(self) -> Iterator[None]:
        token = _ACTIVE_CONTEXT.set(self)
        try:
            yield
        finally:
            _ACTIVE_CONTEXT.reset(token)

    def trace_start(
        self, kind: str, name: str, operation: str, *, input: Any = None, **fields: Any
    ) -> str:
        self._trace_counter += 1
        span_id = f"{self.task_id}:{self._trace_counter}:{uuid.uuid4().hex[:8]}"
        span = {
            "span_id": span_id,
            "parent_span_id": self._trace_stack[-1] if self._trace_stack else None,
            "sequence": self._trace_counter,
            "kind": kind,
            "name": name,
            "operation": operation,
            "status": "RUNNING",
            "started_at": time.time(),
            "input": trace_value(input),
            **{key: trace_value(value, key=key) for key, value in fields.items()},
        }
        self.trace_spans.append(span)
        self._trace_stack.append(span_id)
        if self.trace_handler is not None:
            try:
                self.trace_handler(span)
            except Exception:
                pass
        return span_id

    def trace_end(
        self,
        span_id: str,
        status: str,
        *,
        output: Any = None,
        error: BaseException | None = None,
        **fields: Any,
    ) -> None:
        span = next((item for item in reversed(self.trace_spans) if item["span_id"] == span_id), None)
        if span is None:
            return
        finished = time.time()
        span.update(
            status=status,
            finished_at=finished,
            duration_ms=round((finished - span["started_at"]) * 1000, 3),
            output=trace_value(output),
            **{key: trace_value(value, key=key) for key, value in fields.items()},
        )
        if error is not None:
            span.update(_trace_error_fields(error))
        if self._trace_stack and self._trace_stack[-1] == span_id:
            self._trace_stack.pop()
        elif span_id in self._trace_stack:
            self._trace_stack.remove(span_id)
        if self.trace_handler is not None:
            try:
                self.trace_handler(span)
            except Exception:
                pass
