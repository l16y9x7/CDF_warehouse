from collections.abc import Mapping
from dataclasses import dataclass, field
from typing import Any, Protocol

from agent.capabilities.common import (
    ActionResult,
    ActionStatus,
    DestinationType,
    Hand,
    HealthResult,
    Pose6D,
    TargetType,
    TaskType,
)

PICK_LEVELS = frozenset(f"L{i}" for i in range(1, 6))


@dataclass(frozen=True)
class PickRequest:
    task_type: TaskType
    target_type: TargetType
    sku_typ: str
    hand: Hand
    level: str
    localization_result: Mapping[str, Any]

    def __post_init__(self) -> None:
        if self.task_type is not TaskType.SORTING or self.target_type is not TargetType.SKU:
            raise ValueError("standard pick requires SORTING sku")
        if not str(self.sku_typ).strip():
            raise ValueError("standard pick requires sku_typ")
        if self.level not in PICK_LEVELS:
            raise ValueError("standard pick level must be L1-L5")
        if not isinstance(self.localization_result, Mapping):
            raise ValueError("standard pick requires localization_result object")


@dataclass(frozen=True)
class PickActionResult:
    status: ActionStatus
    box_clearance: dict[str, Any] | None = None
    completed_moves: int | None = None
    raw: dict[str, Any] = field(default_factory=dict)


@dataclass(frozen=True)
class RotateActionResult:
    status: ActionStatus
    camera: str
    image_paths: tuple[str, ...]


@dataclass(frozen=True)
class ReviewItemPickRequest:
    pose: Pose6D
    hand: Hand


@dataclass(frozen=True)
class BasketPickRequest:
    hand: Hand
    localization_result: Mapping[str, Any]
    task_type: TaskType = TaskType.REVIEW
    target_type: TargetType = TargetType.BASKET

    def __post_init__(self) -> None:
        if self.task_type is not TaskType.REVIEW or self.target_type is not TargetType.BASKET:
            raise ValueError("basket pick requires REVIEW basket")
        if not isinstance(self.localization_result, Mapping):
            raise ValueError("basket pick requires localization_result object")


@dataclass(frozen=True)
class PlaceRequest:
    task_type: TaskType
    target_type: TargetType
    destination_type: DestinationType
    hand: Hand
    pose: Pose6D | None = None
    sku_id: str | None = None
    sku_typ: str | None = None
    localization_result: Mapping[str, Any] | None = None

    def __post_init__(self) -> None:
        if self.destination_type is DestinationType.BASKET:
            if (
                self.task_type is not TaskType.SORTING
                or self.target_type is not TargetType.SKU
                or not isinstance(self.sku_typ, str)
                or not self.sku_typ.strip()
                or not isinstance(self.localization_result, Mapping)
                or self.pose is not None
                or self.sku_id is not None
            ):
                raise ValueError("basket place requires SORTING sku_typ and basket localization")
        elif (
            self.task_type is not TaskType.REVIEW
            or self.pose is not None
            or self.sku_id is not None
            or self.sku_typ is not None
            or self.localization_result is not None
        ):
            raise ValueError("table place requires REVIEW without dynamic localization fields")


@dataclass(frozen=True)
class PushRequest:
    hand: Hand
    localization_result: Mapping[str, Any]

    def __post_init__(self) -> None:
        if not isinstance(self.localization_result, Mapping):
            raise ValueError("push requires basket localization")


class ManipulationCapability(Protocol):
    def health(self) -> HealthResult: ...
    def pick(
        self, request: PickRequest, *, idempotency_key: str | None = None
    ) -> PickActionResult: ...
    def pick_review_item(
        self, request: ReviewItemPickRequest, *, idempotency_key: str | None = None
    ) -> ActionResult: ...
    def place(
        self, request: PlaceRequest, *, idempotency_key: str | None = None
    ) -> ActionResult: ...
    def rotate(
        self, hand: Hand, sku_typ: str, *, idempotency_key: str | None = None
    ) -> RotateActionResult: ...
    def push(self, request: PushRequest, *, idempotency_key: str | None = None) -> ActionResult: ...
    def pick_basket(
        self, request: BasketPickRequest, *, idempotency_key: str | None = None
    ) -> ActionResult: ...
