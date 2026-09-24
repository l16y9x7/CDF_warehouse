import math
from dataclasses import dataclass
from typing import Any, Protocol

from agent.capabilities.common import ActionResult, HealthResult

HEAD_CAMERA = "head"


class BodyPoseCapability(Protocol):
    def health(self) -> HealthResult: ...

    def prepare(
        self,
        pose_type: str,
        level: str | None = None,
        *,
        idempotency_key: str | None = None,
    ) -> ActionResult: ...

    def camera_transform(self, camera: str = HEAD_CAMERA) -> "CameraTransform": ...


@dataclass(frozen=True)
class CameraTransform:
    """Camera extrinsics and their frame metadata, as returned by BodyPose 8082."""

    T_chassis_camera: tuple[tuple[float, ...], ...]
    t_unit: str
    camera: str
    camera_frame: str
    base_frame: str
    sampled_at_unix_s: float | None = None
    upper_body_joints_deg: tuple[float, ...] | None = None

    @classmethod
    def from_payload(
        cls, payload: Any, *, expected_camera: str = HEAD_CAMERA
    ) -> "CameraTransform":
        if not isinstance(payload, dict):
            raise ValueError("camera transform response must be an object")
        camera = _nonempty_string(payload.get("camera"), "camera")
        if camera != expected_camera:
            raise ValueError(
                f"camera transform response camera {camera!r} does not match "
                f"requested camera {expected_camera!r}"
            )
        camera_frame = _nonempty_string(payload.get("camera_frame"), "camera_frame")
        base_frame = _nonempty_string(payload.get("base_frame"), "base_frame")
        t_unit = payload.get("t_unit")
        if t_unit not in {"m", "mm"}:
            raise ValueError("camera transform t_unit must be m or mm")
        return cls(
            _matrix4(payload.get("T_chassis_camera")),
            t_unit,
            camera,
            camera_frame,
            base_frame,
            _optional_float(payload.get("sampled_at_unix_s"), "sampled_at_unix_s"),
            _optional_vector(payload.get("upper_body_joints_deg"), "upper_body_joints_deg", 6),
        )


def _nonempty_string(value: Any, name: str) -> str:
    if not isinstance(value, str) or not value.strip():
        raise ValueError(f"camera transform {name} must be a non-empty string")
    return value


def _finite(value: Any) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ValueError("expected a finite number")
    number = float(value)
    if not math.isfinite(number):
        raise ValueError("expected a finite number")
    return number


def _matrix4(value: Any) -> tuple[tuple[float, ...], ...]:
    if not isinstance(value, list) or len(value) != 4:
        raise ValueError("T_chassis_camera must be a 4x4 numeric matrix")
    rows: list[tuple[float, ...]] = []
    for row in value:
        if not isinstance(row, list) or len(row) != 4:
            raise ValueError("T_chassis_camera must be a 4x4 numeric matrix")
        try:
            rows.append(tuple(_finite(item) for item in row))
        except ValueError as exc:
            raise ValueError("T_chassis_camera must be a 4x4 numeric matrix") from exc
    return tuple(rows)


def _optional_float(value: Any, name: str) -> float | None:
    if value is None:
        return None
    try:
        return _finite(value)
    except ValueError as exc:
        raise ValueError(f"{name} must be a finite number") from exc


def _optional_vector(value: Any, name: str, size: int) -> tuple[float, ...] | None:
    if value is None:
        return None
    if not isinstance(value, list) or len(value) != size:
        raise ValueError(f"{name} must be an array of {size} finite numbers")
    try:
        return tuple(_finite(item) for item in value)
    except ValueError as exc:
        raise ValueError(f"{name} must be an array of {size} finite numbers") from exc
