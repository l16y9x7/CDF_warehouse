from __future__ import annotations

import math
from collections.abc import Sequence


# The two SDK worlds have separate shoulder origins, not a shared chest origin.
POSE_FRAMES = {
    "left_arm": "left_arm_sdk_world",
    "right_arm": "right_arm_sdk_world",
    "trunk": "trunk_controller_ref",
}


def _pose(values: Sequence[float]) -> list[float]:
    result = [float(value) for value in values]
    if len(result) != 6 or not all(math.isfinite(value) for value in result):
        raise ValueError("坐标变换需要 6 个有限数值（m/rad）")
    return result


def _rotation(rpy: Sequence[float]) -> list[list[float]]:
    roll, pitch, yaw = rpy
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    # SDK RPY convention: Rz(yaw) @ Ry(pitch) @ Rx(roll).
    return [
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
        [-sp, cp * sr, cp * cr],
    ]


def _transform_pose(
    pose: Sequence[float], world_from_ref: Sequence[float], *, inverse: bool
) -> list[float]:
    source = _pose(pose)
    frame = _pose(world_from_ref)
    rotation = _rotation(frame[3:])
    translation = frame[:3]
    if inverse:
        rotation = [list(row) for row in zip(*rotation)]
        translation = [
            -sum(rotation[i][j] * frame[j] for j in range(3)) for i in range(3)
        ]
    xyz = [
        translation[i] + sum(rotation[i][j] * source[j] for j in range(3))
        for i in range(3)
    ]
    if all(abs(value) < 1e-14 for value in frame[3:]):
        return xyz + source[3:]
    source_rotation = _rotation(source[3:])
    result = [
        [sum(rotation[i][k] * source_rotation[k][j] for k in range(3)) for j in range(3)]
        for i in range(3)
    ]
    horizontal = math.hypot(result[0][0], result[1][0])
    pitch = math.atan2(-result[2][0], horizontal)
    if horizontal > 1e-9:
        roll = math.atan2(result[2][1], result[2][2])
        yaw = math.atan2(result[1][0], result[0][0])
    else:
        roll = math.atan2(-result[1][2], result[1][1])
        yaw = 0.0
    return xyz + [roll, pitch, yaw]


def ref_pose_to_world(pose: Sequence[float], world_from_ref: Sequence[float]) -> list[float]:
    """T_world_tcp = T_world_ref @ T_ref_tcp; all values are m/rad."""
    return _transform_pose(pose, world_from_ref, inverse=False)


def world_pose_to_ref(pose: Sequence[float], world_from_ref: Sequence[float]) -> list[float]:
    """T_ref_tcp = inverse(T_world_ref) @ T_world_tcp; all values are m/rad."""
    return _transform_pose(pose, world_from_ref, inverse=True)
