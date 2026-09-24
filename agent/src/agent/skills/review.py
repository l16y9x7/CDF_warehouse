from collections import Counter
from dataclasses import dataclass
from enum import Enum

from agent.capabilities.camera import CameraCapability, CameraStream, capture_with_event
from agent.capabilities.common import DestinationType, Hand, Pose6D, TargetType, TaskType
from agent.capabilities.estimation import EstimationCapability
from agent.capabilities.manipulation import (
    BasketPickRequest,
    ManipulationCapability,
    PlaceRequest,
)
from agent.capabilities.perception import ImageRequest, LocateStatus, PerceptionCapability
from agent.capabilities.pose import BodyPoseCapability
from agent.capabilities.vla import VlaCapability, VlaPickRequest, VlaPickStatus
from agent.contracts import ExecutionContext
from agent.models import InspectedItem, ReviewItemCount, ReviewSummary
from agent.skills.base import (
    SkillError,
    action_id,
    emit_failed,
    emit_started,
    emit_succeeded,
    fail,
)
from agent.skills.basket_infer import infer_head_basket_pose
from agent.skills.pick_sku_standard import _rgbd_paths, _wrist_camera


@dataclass(frozen=True)
class HandOnlyInput:
    hand: Hand


@dataclass(frozen=True)
class PhysicalResult:
    status: str = "SUCCEEDED"
    pose: Pose6D | None = None


class PickReviewBasketSkill:
    name = "pick_review_basket"
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

    def execute(self, context: ExecutionContext, data: HandOnlyInput) -> PhysicalResult:
        key = action_id(context, self.name)
        emit_started(context, self.name, action_id=key)
        try:
            result = infer_head_basket_pose(
                context, self.camera, self.estimation, self.pose, self.name
            )
            self.manipulation.pick_basket(
                BasketPickRequest(data.hand, result.localization_result),
                idempotency_key=key,
            )
        except Exception as exc:
            emit_failed(context, self.name, exc, action_id=key)
            raise fail(exc) from exc
        emit_succeeded(context, self.name, action_id=key)
        return PhysicalResult()


class PlaceReviewBasketSkill:
    name = "place_review_basket"
    version = "2"

    def __init__(self, manipulation: ManipulationCapability):
        self.manipulation = manipulation

    def execute(self, context: ExecutionContext, data: HandOnlyInput) -> PhysicalResult:
        return _fixed_place(self.manipulation, TargetType.BASKET, data.hand, context, self.name)


class ReviewPickStatus(str, Enum):
    PICKED = "PICKED"
    NOT_FOUND = "NOT_FOUND"


@dataclass(frozen=True)
class ReviewPickResult:
    status: ReviewPickStatus
    pose: Pose6D | None = None


class PickReviewItemStandardSkill:
    name = "pick_review_item_standard"
    version = "2"

    def __init__(
        self,
        perception: PerceptionCapability,
        camera: CameraCapability,
        estimation: EstimationCapability,
        manipulation: ManipulationCapability,
    ):
        self.perception, self.camera, self.estimation, self.manipulation = (
            perception,
            camera,
            estimation,
            manipulation,
        )

    def execute(self, context: ExecutionContext, data: HandOnlyInput) -> ReviewPickResult:
        # TODO 旧 pick_pose 迁移待引入：本链路依赖旧 /estimation/pick_pose
        # （mask + sku），已随 /infer 切换移除；待服务端 Review 链路接入后
        # 重新实现。见 tmp/PICK_POSE_HANDOFF_BRIEF.md 待办清单。
        raise SkillError(
            "PICK_POSE_LEGACY_MIGRATION_PENDING",
            "pick_review_item_standard 依赖旧 pick_pose，迁移待引入",
        )


class PickReviewItemVlaSkill:
    name = "pick_review_item_vla"
    version = "2"

    def __init__(self, vla: VlaCapability):
        self.vla = vla

    def execute(self, context: ExecutionContext, data: HandOnlyInput) -> ReviewPickResult:
        try:
            result = self.vla.pick_review_item(
                VlaPickRequest(data.hand), idempotency_key=action_id(context, self.name)
            )
        except Exception as exc:
            raise fail(exc) from exc
        status = (
            ReviewPickStatus.PICKED
            if result.status is VlaPickStatus.PICKED
            else ReviewPickStatus.NOT_FOUND
        )
        return ReviewPickResult(status)


class PlaceReviewItemSkill:
    name = "place_review_item"
    version = "2"

    def __init__(self, manipulation: ManipulationCapability):
        self.manipulation = manipulation

    def execute(self, context: ExecutionContext, data: HandOnlyInput) -> PhysicalResult:
        return _fixed_place(self.manipulation, TargetType.SKU, data.hand, context, self.name)


class EmptyStatus(str, Enum):
    FOUND = "FOUND"
    NOT_FOUND = "NOT_FOUND"


@dataclass(frozen=True)
class EmptyResult:
    status: EmptyStatus


class ConfirmReviewBasketEmptySkill:
    name = "confirm_review_basket_empty"
    version = "2"

    def __init__(self, perception: PerceptionCapability, camera: CameraCapability):
        self.perception, self.camera = perception, camera

    def execute(self, context: ExecutionContext, data: HandOnlyInput) -> EmptyResult:
        try:
            capture = capture_with_event(
                context,
                self.camera,
                _wrist_camera(data.hand),
                (CameraStream.COLOR, CameraStream.DEPTH),
                skill=self.name,
            )
            rgb, _ = _rgbd_paths(capture)
            result = self.perception.locate_basket_item(ImageRequest(rgb))
        except Exception as exc:
            raise fail(exc) from exc
        return EmptyResult(
            EmptyStatus.NOT_FOUND if result.status is LocateStatus.NOT_FOUND else EmptyStatus.FOUND
        )


@dataclass(frozen=True)
class SummarizeReviewInput:
    expected_items: tuple[ReviewItemCount, ...]
    inspected_items: tuple[InspectedItem, ...]


class SummarizeReviewResultSkill:
    name = "summarize_review_result"
    version = "2"

    def execute(self, context: ExecutionContext, data: SummarizeReviewInput) -> ReviewSummary:
        emit_started(context, self.name)
        expected = Counter()
        for item in data.expected_items:
            if not item.sku_id or item.count <= 0:
                raise SkillError(
                    "INVALID_INPUT", "expected item requires sku_id and positive count"
                )
            expected[item.sku_id] += item.count
        actual = Counter(item.actual_sku_id for item in data.inspected_items)
        missing = tuple(
            ReviewItemCount(sku, expected[sku] - actual[sku])
            for sku in sorted(expected)
            if expected[sku] > actual[sku]
        )
        extra = tuple(
            ReviewItemCount(sku, actual[sku] - expected[sku])
            for sku in sorted(expected)
            if expected[sku] and actual[sku] > expected[sku]
        )
        wrong = tuple(
            ReviewItemCount(sku, count)
            for sku, count in sorted(actual.items())
            if sku not in expected
        )
        status = "PASS" if not (missing or extra or wrong) else "DISCREPANCY"
        result = ReviewSummary(status, missing, extra, wrong)
        emit_succeeded(context, self.name, review_status=status)
        return result


def _fixed_place(
    manipulation: ManipulationCapability,
    target: TargetType,
    hand: Hand,
    context: ExecutionContext,
    skill_name: str,
) -> PhysicalResult:
    try:
        manipulation.place(
            PlaceRequest(TaskType.REVIEW, target, DestinationType.TABLE, hand),
            idempotency_key=action_id(context, skill_name),
        )
    except Exception as exc:
        raise fail(exc) from exc
    return PhysicalResult()


__all__ = [
    name
    for name in globals()
    if name.endswith(("Input", "Result", "Skill", "Status", "Summary", "ItemCount", "Item"))
]
