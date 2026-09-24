"""Structured control traces; never issue extra SDK or motion calls."""
from __future__ import annotations

import contextvars
import functools
import threading
import time
import uuid
from contextlib import contextmanager


context = contextvars.ContextVar("control_trace", default={})


def fields():
    return dict(context.get())


def update(**values):
    context.set({**context.get(), **values})


def worker_thread(*, target, args=(), **kwargs):
    captured = contextvars.copy_context()
    return threading.Thread(target=captured.run, args=(target, *args), **kwargs)


def operation(kind):
    """An operation keeps one ID across HTTP acceptance and worker completion."""
    def decorate(method):
        @functools.wraps(method)
        def wrapped(self, *args, **kwargs):
            token = context.set({**fields(), "operation_id": uuid.uuid4().hex, "operation": kind})
            try:
                self.service.audit_event("operation_requested", args=args, kwargs=kwargs)
                return method(self, *args, **kwargs)
            except Exception as exc:
                self.service.audit_event("operation_rejected", error=str(exc))
                raise
            finally:
                context.reset(token)
        return wrapped
    return decorate


@contextmanager
def span(emit, name, **data):
    started = time.monotonic_ns()
    emit("phase_started", phase_name=name, **data)
    result = {}
    try:
        yield result
    except BaseException as exc:
        result.update(ok=False, error_type=type(exc).__name__, error=str(exc))
        raise
    else:
        result["ok"] = True
    finally:
        elapsed = (time.monotonic_ns() - started) / 1e6
        result["duration_ms"] = elapsed
        emit("phase_finished", phase_name=name, **data, **result)


def plain(value, depth=0):
    """Freeze SDK arguments, including mutable out-parameters, as JSON data."""
    if value is None or isinstance(value, (str, int, float, bool)):
        return value
    if depth > 12:
        return {"type": type(value).__name__, "serialization_limit": True}
    if isinstance(value, dict):
        return {str(k): plain(v, depth + 1) for k, v in value.items()}
    if isinstance(value, (list, tuple)):
        return [plain(v, depth + 1) for v in value]
    if hasattr(value, "tolist"):
        return plain(value.tolist(), depth + 1)
    result = {"type": type(value).__name__}
    # Attribute reads only: do not call robot/model methods while logging.
    for name in ("trans", "rpy", "elbow", "hasElbow", "confData", "external",
                 "end", "ref", "speed", "zone", "rotSpeed", "target", "cart",
                 "joints", "joint", "pose", "cartesian", "values", "name", "value", "blend",
                 "jointSpeed", "offset", "customInfo"):
        try:
            item = getattr(value, name)
            if not callable(item):
                result[name] = plain(item, depth + 1)
        except Exception:
            pass
    if type(value).__name__.startswith(("PyType", "PyString")):
        try:
            result["content"] = plain(value.content(), depth + 1)
        except Exception:
            pass
    if len(result) == 1:
        result["repr"] = str(value)
    return result


def invoke(emit, action, function, args, ec):
    started = time.monotonic_ns()
    before = plain(args) if emit else None
    result, error = None, None
    try:
        result = function(*args, ec)
        return result
    except BaseException as exc:
        error = str(exc)
        raise
    finally:
        if emit:
            emit("sdk_call", action=action, function=getattr(function, "__name__", type(function).__name__),
                 started_monotonic_ns=started, duration_ms=(time.monotonic_ns() - started) / 1e6,
                 args=before, args_after=plain(args), result=plain(result), ec=dict(ec), error=error,
                 sdk_units="m,rad; command speed mm/s; external units depend on SDK interface")
