from dataclasses import dataclass

from agent.capabilities.navigation import NAVIGATION_TARGETS, NavigationCapability
from agent.contracts import ExecutionContext
from agent.skills.base import (
    SkillError,
    action_id,
    emit_failed,
    emit_started,
    emit_succeeded,
    fail,
    require,
)


@dataclass(frozen=True)
class NavigateInput:
    target_id: str


@dataclass(frozen=True)
class NavigateResult:
    target_id: str
    status: str = "SUCCEEDED"


class NavigateSkill:
    name = "navigate"
    version = "1"

    def __init__(self, navigation: NavigationCapability):
        self.navigation = navigation

    def execute(self, context: ExecutionContext, data: NavigateInput) -> NavigateResult:
        target = require(data.target_id, "target_id is required")
        if target not in NAVIGATION_TARGETS:
            raise SkillError("INVALID_INPUT", f"unsupported target_id: {target}")
        key = action_id(context, self.name, target)
        emit_started(context, self.name, target_id=target, action_id=key)
        try:
            receipt = self.navigation.navigate(target, idempotency_key=key)
        except Exception as exc:
            emit_failed(context, self.name, exc, action_id=key)
            raise fail(exc) from exc
        emit_succeeded(context, self.name, target_id=target, action_id=key)
        return NavigateResult(target, receipt.status.value)
