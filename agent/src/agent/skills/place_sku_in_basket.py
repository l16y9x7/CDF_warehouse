from dataclasses import dataclass

from agent.capabilities.camera import CameraCapability
from agent.capabilities.common import DestinationType, Hand, TargetType, TaskType
from agent.capabilities.estimation import EstimationCapability
from agent.capabilities.manipulation import ManipulationCapability, PlaceRequest
from agent.capabilities.pose import BodyPoseCapability
from agent.contracts import ExecutionContext
from agent.skills.base import (
    action_id,
    emit_failed,
    emit_started,
    emit_succeeded,
    fail,
)
from agent.skills.basket_infer import infer_head_basket_pose


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
        camera: CameraCapability,
        estimation: EstimationCapability,
        manipulation: ManipulationCapability,
        pose: BodyPoseCapability,
    ):
        self.camera = camera
        self.estimation = estimation
        self.manipulation = manipulation
        self.pose = pose

    def execute(self, context: ExecutionContext, data: PlaceSkuInBasketInput) -> PlaceSkuResult:
        key = action_id(context, self.name)
        emit_started(context, self.name, action_id=key)
        try:
            result = infer_head_basket_pose(
                context, self.camera, self.estimation, self.pose, self.name
            )
            self.manipulation.place(
                PlaceRequest(
                    task_type=TaskType.SORTING,
                    target_type=TargetType.SKU,
                    destination_type=DestinationType.BASKET,
                    hand=data.hand,
                    sku_typ=data.sku_typ,
                    localization_result=result.localization_result,
                ),
                idempotency_key=key,
            )
        except Exception as exc:
            emit_failed(context, self.name, exc, action_id=key)
            raise fail(exc) from exc
        emit_succeeded(context, self.name, action_id=key)
        return PlaceSkuResult(DestinationType.BASKET)
