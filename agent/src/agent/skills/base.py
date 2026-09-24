import time
from typing import Any, Protocol, TypeVar

from agent.capabilities.common import CapabilityError
from agent.contracts import AgentError, ExecutionContext

InputT = TypeVar("InputT")
ResultT = TypeVar("ResultT")


class Skill(Protocol[InputT, ResultT]):
    name: str
    version: str

    def execute(self, context: ExecutionContext, data: InputT) -> ResultT: ...


class SkillError(AgentError):
    """Expected failure raised at the Skill boundary."""


def normalize_error(error: BaseException) -> SkillError | AgentError:
    """Map adapter failures to the error vocabulary exposed by Skills."""
    if isinstance(error, AgentError):
        return error
    if isinstance(error, CapabilityError):
        if error.error_code in {
            "CAPABILITY_UNAVAILABLE",
            "MODULE_UNAVAILABLE",
            "UNAVAILABLE",
            "CONNECTION_FAILED",
        }:
            return SkillError("CAPABILITY_UNAVAILABLE", error.message)
        if error.error_code in {"TIMEOUT", "ACTION_RESULT_UNKNOWN"}:
            return SkillError("ACTION_RESULT_UNKNOWN", "physical action result is unknown")
        return SkillError("CAPABILITY_EXECUTION_FAILED", error.message)
    if isinstance(error, TimeoutError):
        return SkillError("ACTION_RESULT_UNKNOWN", "physical action result is unknown")
    return SkillError("CAPABILITY_EXECUTION_FAILED", str(error))


def fail(error: BaseException) -> SkillError | AgentError:
    return normalize_error(error)


def action_id(context: ExecutionContext, skill: str, *parts: object) -> str:
    node_id = context.metadata.get("node_id")
    values = [context.task_id]
    if node_id:
        values.append(str(node_id))
    values.extend([skill, *(str(part) for part in parts)])
    return ":".join(value.replace(":", "_") for value in values)


def emit_started(context: ExecutionContext, skill: str, **data: Any) -> None:
    context._timers[f"skill:{skill}"] = time.monotonic()
    context.emit("skill.started", skill=skill, **data)


def emit_succeeded(context: ExecutionContext, skill: str, **data: Any) -> None:
    started = context._timers.pop(f"skill:{skill}", None)
    if started is not None:
        data.setdefault("duration_ms", round((time.monotonic() - started) * 1000, 3))
    context.emit("skill.succeeded", skill=skill, status="SUCCEEDED", **data)


def emit_failed(context: ExecutionContext, skill: str, error: BaseException, **data: Any) -> None:
    normalized = normalize_error(error)
    started = context._timers.pop(f"skill:{skill}", None)
    if started is not None:
        data.setdefault("duration_ms", round((time.monotonic() - started) * 1000, 3))
    context.emit(
        "skill.failed",
        skill=skill,
        status="FAILED",
        error_code=getattr(normalized, "code", type(normalized).__name__),
        message=str(normalized),
        **data,
    )


def require(value: Any, message: str, *, code: str = "INVALID_INPUT") -> Any:
    if value is None or (isinstance(value, str) and not value.strip()):
        raise SkillError(code, message)
    return value
