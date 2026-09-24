from dataclasses import dataclass
from enum import Enum
from typing import Any, cast


class HealthStatus(str, Enum):
    STARTING = "STARTING"
    READY = "READY"
    ERROR = "ERROR"


class ActionStatus(str, Enum):
    SUCCEEDED = "SUCCEEDED"


class TaskType(str, Enum):
    SORTING = "SORTING"
    REVIEW = "REVIEW"


class TargetType(str, Enum):
    SKU = "sku"
    BASKET = "basket"


class DestinationType(str, Enum):
    BASKET = "basket"
    TABLE = "table"


class Hand(str, Enum):
    LEFT = "LEFT"
    RIGHT = "RIGHT"


@dataclass(frozen=True)
class HealthResult:
    status: HealthStatus


@dataclass(frozen=True)
class ActionResult:
    status: ActionStatus
    action_id: str | None = None


@dataclass(frozen=True)
class BoundingBox:
    x1: int
    y1: int
    x2: int
    y2: int

    def as_list(self) -> list[int]:
        return [self.x1, self.y1, self.x2, self.y2]

    @classmethod
    def from_value(cls, value: list[int]) -> "BoundingBox":
        if len(value) != 4:
            raise ValueError("bbox must contain exactly four integers")
        return cls(*value)


@dataclass(frozen=True)
class Detection:
    bbox: BoundingBox
    mask: str


@dataclass(frozen=True)
class Pose6D:
    values: tuple[float, float, float, float, float, float]
    corners_mm: tuple[tuple[float, float, float], ...]
    frame: str = "camera"
    pose_unit: str = "mm_rad"
    rotation_order: str = "zyx"

    @classmethod
    def from_payload(cls, payload: dict[str, Any]) -> "Pose6D":
        pose = tuple(float(value) for value in payload["pose"])
        if len(pose) != 6:
            raise ValueError("pose must contain exactly six values")
        corners = tuple(tuple(float(value) for value in point) for point in payload["corners_mm"])
        return cls(
            values=cast(tuple[float, float, float, float, float, float], pose),
            corners_mm=corners,
            frame=payload["frame"],
            pose_unit=payload["pose_unit"],
            rotation_order=payload["rotation_order"],
        )

    def as_payload(self) -> dict[str, Any]:
        return {
            "pose": list(self.values),
            "corners_mm": [list(point) for point in self.corners_mm],
            "frame": self.frame,
            "pose_unit": self.pose_unit,
            "rotation_order": self.rotation_order,
        }


class ErrorSource(str, Enum):
    REMOTE = "remote"
    TRANSPORT = "transport"
    LOCAL = "local"


class CapabilityError(RuntimeError):
    def __init__(
        self,
        error_code: str,
        message: str,
        *,
        status_code: int | None = None,
        source: ErrorSource = ErrorSource.LOCAL,
        operation: str | None = None,
    ):
        super().__init__(message)
        self.error_code = error_code
        self.message = message
        self.status_code = status_code
        self.source = ErrorSource(source)
        self.operation = operation

    @property
    def code(self) -> str:
        return self.error_code


class CapabilityUnavailableError(CapabilityError):
    def __init__(
        self,
        error_code: str,
        message: str,
        *,
        status_code: int | None = None,
        source: ErrorSource = ErrorSource.TRANSPORT,
        operation: str | None = None,
    ):
        super().__init__(
            error_code,
            message,
            status_code=status_code,
            source=source,
            operation=operation,
        )
