from collections.abc import Mapping
from dataclasses import dataclass
from enum import Enum
from typing import Any

from agent.capabilities.common import Hand, TaskType


class PickBackend(str, Enum):
    """Legacy policy values retained for config/API compatibility."""

    STANDARD = "STANDARD"
    VLA = "VLA"
    HAND = "HAND"


class BarcodeMismatchMode(str, Enum):
    STOP = "STOP"
    CONTINUE = "CONTINUE"


@dataclass(frozen=True)
class PickSelection:
    hand: Hand = Hand.RIGHT


class PickPolicy:
    def __init__(
        self,
        *,
        sorting_backend: PickBackend | None = None,
        review_backend: PickBackend | None = None,
        sorting_hand: Hand = Hand.RIGHT,
        review_hand: Hand = Hand.RIGHT,
        barcode_mismatch: BarcodeMismatchMode = BarcodeMismatchMode.STOP,
    ) -> None:
        _reject_unconnected_backend(TaskType.SORTING, sorting_backend)
        _reject_unconnected_backend(TaskType.REVIEW, review_backend)
        self._hands = {
            TaskType.SORTING: sorting_hand,
            TaskType.REVIEW: review_hand,
        }
        self.barcode_mismatch = barcode_mismatch

    @classmethod
    def from_config(cls, config: Mapping[str, Any]) -> "PickPolicy":
        workflows = config.get("workflows", {})
        sorting = workflows.get("sorting", {})
        review = workflows.get("review", {})
        _reject_unconnected_backend(TaskType.SORTING, sorting.get("pick_backend"))
        _reject_unconnected_backend(TaskType.REVIEW, review.get("pick_backend"))
        return cls(
            sorting_hand=_hand(sorting.get("hand")),
            review_hand=_hand(review.get("hand")),
            barcode_mismatch=_barcode_mismatch(sorting.get("barcode_mismatch")),
        )

    def select(self, task_type: TaskType) -> PickSelection:
        return PickSelection(self._hands[task_type])


def _reject_unconnected_backend(task_type: TaskType, value: object) -> None:
    normalized = value.value if isinstance(value, PickBackend) else value
    if normalized is not None and str(normalized).upper() != PickBackend.STANDARD.value:
        raise ValueError(
            f"{task_type.value} pick backend {normalized} is not connected; "
            "only standard grasping is currently supported"
        )


def _hand(value: object) -> Hand:
    if value is None:
        return Hand.RIGHT
    try:
        return Hand(str(value).upper())
    except ValueError as exc:
        raise ValueError(f"hand must be LEFT or RIGHT, got {value!r}") from exc


def _barcode_mismatch(value: object) -> BarcodeMismatchMode:
    if value is None:
        return BarcodeMismatchMode.STOP
    try:
        return BarcodeMismatchMode(str(value).upper())
    except ValueError as exc:
        raise ValueError(
            f"barcode_mismatch must be STOP or CONTINUE, got {value!r}"
        ) from exc


__all__ = ["BarcodeMismatchMode", "PickBackend", "PickPolicy", "PickSelection"]
