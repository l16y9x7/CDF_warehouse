from dataclasses import dataclass

from agent.capabilities.common import DestinationType, Hand, TargetType, TaskType
from agent.capabilities.manipulation import ManipulationCapability, PlaceRequest
from agent.contracts import ExecutionContext
from agent.skills.base import (
    action_id,
    emit_failed,
    emit_started,
    emit_succeeded,
    fail,
)


@dataclass(frozen=True)
class PlaceSkuInBasketInput:
    sku_typ: str
    hand: Hand


@dataclass(frozen=True)
class PlaceSkuResult:
    destination_type: DestinationType
    status: str = "SUCCEEDED"


class PlaceSkuInBasketSkill:
    name = "place_sku_in_basket"
    version = "3"

    def __init__(
        self,
        manipulation: ManipulationCapability,
    ):
        self.manipulation = manipulation

    def execute(self, context: ExecutionContext, data: PlaceSkuInBasketInput) -> PlaceSkuResult:
        key = action_id(context, self.name)
        emit_started(context, self.name, action_id=key)
        try:
            self.manipulation.place(
                PlaceRequest(
                    task_type=TaskType.SORTING,
                    target_type=TargetType.SKU,
                    destination_type=DestinationType.BASKET,
                    hand=data.hand,
                    sku_typ=data.sku_typ,
                ),
                idempotency_key=key,
            )
        except Exception as exc:
            emit_failed(context, self.name, exc, action_id=key)
            raise fail(exc) from exc
        emit_succeeded(context, self.name, action_id=key)
        return PlaceSkuResult(DestinationType.BASKET)
