from collections.abc import Mapping
from dataclasses import dataclass
from typing import Any

from agent.capabilities.camera import CameraCapability
from agent.capabilities.common import Hand
from agent.capabilities.estimation import EstimationCapability
from agent.capabilities.manipulation import ManipulationCapability, PushRequest
from agent.capabilities.pose import BodyPoseCapability
from agent.contracts import ExecutionContext
from agent.skills.base import SkillError, action_id, emit_failed, emit_started, emit_succeeded, fail
from agent.skills.basket_infer import infer_head_basket_pose


@dataclass(frozen=True)
class PushBasketInput:
    hand: Hand
    localization_result: Mapping[str, Any] | None = None


@dataclass(frozen=True)
class PushBasketResult:
    status: str = "SUCCEEDED"


class PushBasketSkill:
    name = "push_basket"
    version = "4"

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

    def execute(self, context: ExecutionContext, data: PushBasketInput) -> PushBasketResult:
        key = action_id(context, self.name)
        emit_started(context, self.name, action_id=key)
        try:
            localization = data.localization_result
            if localization is None:
                localization = infer_head_basket_pose(
                    context, self.camera, self.estimation, self.pose, self.name
                ).localization_result
            elif not isinstance(localization, Mapping):
                raise SkillError("INVALID_INPUT", "localization_result must be an object")
            self.manipulation.push(
                PushRequest(data.hand, localization),
                idempotency_key=key,
            )
        except Exception as exc:
            emit_failed(context, self.name, exc, action_id=key)
            raise fail(exc) from exc
        emit_succeeded(context, self.name, action_id=key)
        return PushBasketResult()


__all__ = ["PushBasketInput", "PushBasketResult", "PushBasketSkill"]
