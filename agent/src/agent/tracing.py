from __future__ import annotations

import inspect
from functools import wraps
from typing import Any

from agent.contracts import ExecutionContext, current_execution_context
from agent.observability import log_context


def _call_input(function: Any, args: tuple[Any, ...], kwargs: dict[str, Any]) -> Any:
    try:
        bound = inspect.signature(function).bind(*args, **kwargs)
    except (TypeError, ValueError):
        return {"args": list(args), "kwargs": kwargs}
    return dict(bound.arguments)


class TracedCapability:
    """Record capability calls only while an execution context is active."""

    def __init__(self, name: str, capability: Any):
        self._trace_name = name
        self._trace_target = capability

    def __getattr__(self, name: str) -> Any:
        attribute = getattr(self._trace_target, name)
        if not callable(attribute) or name.startswith("_"):
            return attribute

        @wraps(attribute)
        def traced(*args: Any, **kwargs: Any) -> Any:
            context = current_execution_context()
            if context is None:
                return attribute(*args, **kwargs)
            span_id = context.trace_start(
                "capability",
                self._trace_name,
                name,
                input=_call_input(attribute, args, kwargs),
                node_id=context.metadata.get("node_id"),
            )
            with log_context(span_id=span_id):
                try:
                    result = attribute(*args, **kwargs)
                except Exception as exc:
                    context.trace_end(span_id, "FAILED", error=exc)
                    raise
                context.trace_end(span_id, "SUCCEEDED", output=result)
                return result

        return traced


class TracedSkill:
    """Record a Skill execute boundary and retain the Skill's public surface."""

    def __init__(self, skill: Any):
        self._trace_target = skill
        self.name = skill.name
        self.version = skill.version

    def __getattr__(self, name: str) -> Any:
        return getattr(self._trace_target, name)

    def execute(self, context: ExecutionContext, data: Any) -> Any:
        span_id = context.trace_start(
            "skill",
            self.name,
            "execute",
            input=data,
            node_id=context.metadata.get("node_id"),
            version=self.version,
        )
        with context.activate(), log_context(span_id=span_id):
            try:
                result = self._trace_target.execute(context, data)
            except Exception as exc:
                status = "CANCELLED" if getattr(exc, "code", None) == "CANCELLED" else "FAILED"
                context.trace_end(span_id, status, error=exc)
                raise
            context.trace_end(span_id, "SUCCEEDED", output=result)
            return result


def traced_capabilities(capabilities: dict[str, Any]) -> dict[str, Any]:
    return {
        name: value if isinstance(value, TracedCapability) else TracedCapability(name, value)
        for name, value in capabilities.items()
    }


def traced_skills(skills: dict[str, Any]) -> dict[str, Any]:
    return {
        name: value if isinstance(value, TracedSkill) else TracedSkill(value)
        for name, value in skills.items()
    }
