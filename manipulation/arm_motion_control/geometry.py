"""Coordinate transforms for the grasp workflow. All lengths are millimetres.

The only world frame is chassis_link.  A frame name is never used as evidence
that two origins coincide: the caller must supply their measured transform.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np


def rigid(value: object, name: str) -> np.ndarray:
    matrix = np.asarray(value, dtype=float)
    if matrix.shape != (4, 4) or not np.isfinite(matrix).all():
        raise ValueError(f"{name}: expected finite 4x4 transform")
    if not np.allclose(matrix[3], [0, 0, 0, 1], atol=1e-8):
        raise ValueError(f"{name}: invalid homogeneous row")
    rotation = matrix[:3, :3]
    if not np.allclose(rotation.T @ rotation, np.eye(3), atol=2e-3):
        raise ValueError(f"{name}: rotation is not orthonormal")
    if not math.isclose(float(np.linalg.det(rotation)), 1.0, abs_tol=2e-3):
        raise ValueError(f"{name}: rotation is reflected or scaled")
    return matrix


def inverse(transform: np.ndarray) -> np.ndarray:
    transform = rigid(transform, "transform")
    result = np.eye(4)
    result[:3, :3] = transform[:3, :3].T
    result[:3, 3] = -result[:3, :3] @ transform[:3, 3]
    return result


def point(transform: np.ndarray, xyz_mm: object) -> np.ndarray:
    xyz = np.asarray(xyz_mm, dtype=float)
    if xyz.shape != (3,) or not np.isfinite(xyz).all():
        raise ValueError("point must contain three finite coordinates")
    return transform[:3, :3] @ xyz + transform[:3, 3]


def direction(transform: np.ndarray, xyz: object) -> np.ndarray:
    vector = np.asarray(xyz, dtype=float)
    if vector.shape != (3,) or not np.isfinite(vector).all():
        raise ValueError("direction must contain three finite coordinates")
    norm = float(np.linalg.norm(vector))
    if norm < 1e-9:
        raise ValueError("zero direction")
    return transform[:3, :3] @ (vector / norm)


@dataclass(frozen=True)
class GraspGeometry:
    visible_top_world_mm: np.ndarray
    grasp_world_mm: np.ndarray
    axis_up_world: np.ndarray
    flange_grasp_world: np.ndarray
    flange_pregrasp_world: np.ndarray


def flange_orientation_world() -> np.ndarray:
    """Front grasp: flange +Z points chassis +X, flange +X points +Z."""
    return np.array([[0., 0., 1.], [0., -1., 0.], [1., 0., 0.]])


def grasp_geometry(
    visible_top_world_mm: object,
    axis_up_world: object,
    *,
    below_top_mm: float = 20.0,
    virtual_tool_mm: float = 255.0,
    pregrasp_extra_mm: float = 100.0,
) -> GraspGeometry:
    top = np.asarray(visible_top_world_mm, dtype=float)
    axis = np.asarray(axis_up_world, dtype=float)
    if top.shape != (3,) or axis.shape != (3,) or not np.isfinite(top).all() or not np.isfinite(axis).all():
        raise ValueError("top and axis must be finite 3-vectors")
    axis_length = float(np.linalg.norm(axis))
    if axis_length < 1e-9:
        raise ValueError("zero bottle axis")
    axis = axis / axis_length
    if any(not math.isfinite(v) or v < 0 for v in (below_top_mm, virtual_tool_mm, pregrasp_extra_mm)):
        raise ValueError("grasp offsets must be nonnegative and finite")
    grasp = top - below_top_mm * axis
    rotation = flange_orientation_world()
    flange = np.eye(4)
    flange[:3, :3] = rotation
    flange[:3, 3] = grasp - virtual_tool_mm * rotation[:, 2]
    pregrasp = flange.copy()
    pregrasp[:3, 3] -= pregrasp_extra_mm * rotation[:, 2]
    if not np.allclose(flange[:3, 3] + virtual_tool_mm * rotation[:, 2],
                       pregrasp[:3, 3] + (virtual_tool_mm + pregrasp_extra_mm) * rotation[:, 2]):
        raise AssertionError("virtual pregrasp tip changed")
    return GraspGeometry(top, grasp, axis, flange, pregrasp)


def world_goal_in_arm_reference(flange_world: np.ndarray, world_from_arm_reference: np.ndarray) -> np.ndarray:
    """Reproject an unchanged world goal after reading the new torso pose."""
    return inverse(world_from_arm_reference) @ rigid(flange_world, "flange_world")
