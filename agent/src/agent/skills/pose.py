from dataclasses import dataclass
from typing import ClassVar

from agent.capabilities.pose import BodyPoseCapability
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
class PreparePoseInput:
    pose_type: str
    level: str | None = None


@dataclass(frozen=True)
class PreparePoseResult:
    pose_type: str
    level: str | None
    status: str = "SUCCEEDED"


class PreparePoseSkill:
    name = "prepare_pose"
    version = "1"

    _POSE_TYPES: ClassVar[frozenset[str]] = frozenset(
        {
            "AGV_carton_item_inspect",
            "AGV_item_barcode_scan",
            "basket_item_place_prepare",
            "basket_item_pick_inspect",
            "basket_item_pick_prepare",
            "basket_item_barcode_scan",
            "basket_place",
            "review_item_place",
            "basket_pick_prepare",
            "basket_push",
        }
    )
    _LEVEL_POSES: ClassVar[frozenset[str]] = frozenset(
        {
            "AGV_carton_item_inspect",
            "basket_item_place_prepare",
            "basket_pick_prepare",
            "basket_push",
        }
    )

    def __init__(self, pose: BodyPoseCapability):
        self.pose = pose

    def execute(self, context: ExecutionContext, data: PreparePoseInput) -> PreparePoseResult:
        require(data.pose_type, "pose_type is required")
        if data.pose_type not in self._POSE_TYPES:
            raise SkillError("INVALID_INPUT", f"unsupported pose_type: {data.pose_type}")
        level_required = data.pose_type in self._LEVEL_POSES
        if level_required and data.level is None:
            raise SkillError("INVALID_INPUT", f"{data.pose_type} requires level")
        if not level_required and data.level is not None:
            raise SkillError("INVALID_INPUT", f"{data.pose_type} must not include level")
        max_level = 5 if data.pose_type.startswith("AGV_") else 4
        if data.level is not None and data.level not in {f"L{i}" for i in range(1, max_level + 1)}:
            raise SkillError("INVALID_INPUT", f"level must be L1-L{max_level}")
        key = action_id(context, self.name, data.pose_type, data.level or "none")
        emit_started(context, self.name, pose_type=data.pose_type, level=data.level, action_id=key)
        try:
            receipt = self.pose.prepare(data.pose_type, data.level, idempotency_key=key)
        except Exception as exc:
            emit_failed(context, self.name, exc, action_id=key)
            raise fail(exc) from exc
        emit_succeeded(context, self.name, action_id=key)
        return PreparePoseResult(data.pose_type, data.level, receipt.status.value)
