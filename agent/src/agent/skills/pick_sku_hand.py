from dataclasses import dataclass

from agent.capabilities.common import Hand, TargetType, TaskType
from agent.capabilities.hand import HandCapability, HandPickRequest
from agent.contracts import ExecutionContext
from agent.skills.base import action_id, emit_failed, emit_started, emit_succeeded, fail, require
from agent.skills.pick_sku_standard import PickSkuResult


@dataclass(frozen=True)
class PickSkuHandInput:
    sku_id: str
    hand: Hand


class PickSkuHandSkill:
    name = "pick_sku_hand"
    version = "2"

    def __init__(self, hand_capability: HandCapability) -> None:
        self.hand_capability = hand_capability

    def execute(self, context: ExecutionContext, data: PickSkuHandInput) -> PickSkuResult:
        require(data.sku_id, "sku_id is required")
        key = action_id(context, self.name, data.sku_id)
        emit_started(context, self.name, sku_id=data.sku_id, action_id=key)
        try:
            self.hand_capability.pick(
                HandPickRequest(TaskType.SORTING, TargetType.SKU, data.sku_id, data.hand),
                idempotency_key=key,
            )
        except Exception as exc:
            emit_failed(context, self.name, exc, action_id=key)
            raise fail(exc) from exc
        emit_succeeded(context, self.name, sku_id=data.sku_id, backend="HAND", action_id=key)
        return PickSkuResult(data.sku_id, "HAND")
