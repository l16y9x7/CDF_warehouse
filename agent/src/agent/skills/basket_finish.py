from dataclasses import dataclass

from agent.capabilities.common import Hand
from agent.capabilities.manipulation import ManipulationCapability, PushRequest
from agent.contracts import ExecutionContext
from agent.skills.base import action_id, emit_failed, emit_started, emit_succeeded, fail


@dataclass(frozen=True)
class PushBasketInput:
    hand: Hand


@dataclass(frozen=True)
class PushBasketResult:
    status: str = "SUCCEEDED"


class PushBasketSkill:
    name = "push_basket"
    version = "5"

    def __init__(self, manipulation: ManipulationCapability):
        self.manipulation = manipulation

    def execute(self, context: ExecutionContext, data: PushBasketInput) -> PushBasketResult:
        key = action_id(context, self.name)
        emit_started(context, self.name, action_id=key)
        try:
            self.manipulation.push(PushRequest(data.hand), idempotency_key=key)
        except Exception as exc:
            emit_failed(context, self.name, exc, action_id=key)
            raise fail(exc) from exc
        emit_succeeded(context, self.name, action_id=key)
        return PushBasketResult()


__all__ = ["PushBasketInput", "PushBasketResult", "PushBasketSkill"]
