"""Fail-closed MOVE L *candidate* sampler, not a collision-certified controller.

The caller supplies IK/FK for the controller's current TCP and reference.  The
J4 elbow plane is a required filter, but a passing result is not proof that the
forearm, gripper, box or shelf is collision-free.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Protocol

import numpy as np

if __package__:
    from .geometry import rigid
else:
    from geometry import rigid


class Kinematics(Protocol):
    def ik(self, world_flange_mm: np.ndarray, elbow_angle_deg: float, seed_deg: np.ndarray) -> np.ndarray | None: ...
    def fk(self, joint_deg: np.ndarray) -> tuple[np.ndarray, np.ndarray]: ...
    def within_limits(self, joint_deg: np.ndarray) -> bool: ...


@dataclass(frozen=True)
class Waypoint:
    flange_world_mm: np.ndarray
    joints_deg: np.ndarray
    elbow_world_mm: np.ndarray
    arm_angle_deg: float


@dataclass(frozen=True)
class CandidatePlan:
    approach: tuple[Waypoint, ...]
    grasp: tuple[Waypoint, ...]
    selected_arm_angle_deg: float
    plane_y_mm: float


def rotation_log(matrix: np.ndarray) -> np.ndarray:
    cosine = float(np.clip((np.trace(matrix) - 1) / 2, -1, 1))
    angle = math.acos(cosine)
    skew = np.array([matrix[2, 1] - matrix[1, 2], matrix[0, 2] - matrix[2, 0], matrix[1, 0] - matrix[0, 1]]) / 2
    sine = float(np.linalg.norm(skew))
    if angle < 1e-9:
        return skew
    if sine > 1e-8:
        return skew * (angle / sine)
    values, vectors = np.linalg.eigh((matrix + np.eye(3)) / 2)
    return angle * vectors[:, int(np.argmax(values))]


def rotation_exp(vector: np.ndarray) -> np.ndarray:
    angle = float(np.linalg.norm(vector))
    if angle < 1e-12:
        return np.eye(3)
    x, y, z = vector / angle
    c, s, v = math.cos(angle), math.sin(angle), 1 - math.cos(angle)
    return np.array([
        [c + x*x*v, x*y*v - z*s, x*z*v + y*s],
        [y*x*v + z*s, c + y*y*v, y*z*v - x*s],
        [z*x*v - y*s, z*y*v + x*s, c + z*z*v],
    ])


def interpolate(start: np.ndarray, goal: np.ndarray, fraction: float) -> np.ndarray:
    start, goal = rigid(start, "start"), rigid(goal, "goal")
    if not 0 <= fraction <= 1:
        raise ValueError("fraction outside [0, 1]")
    result = np.eye(4)
    result[:3, 3] = start[:3, 3] * (1 - fraction) + goal[:3, 3] * fraction
    rotation_delta = rotation_log(goal[:3, :3] @ start[:3, :3].T)
    result[:3, :3] = rotation_exp(fraction * rotation_delta) @ start[:3, :3]
    return result


def arm_angle_candidates(current_deg: float, max_change_deg: float = 90.0) -> tuple[float, ...]:
    if not math.isfinite(current_deg) or max_change_deg < 0 or max_change_deg > 180:
        raise ValueError("invalid arm angle search range")
    values = [current_deg]
    for delta in range(5, int(max_change_deg) + 1, 5):
        for sign in (1, -1):
            candidate = current_deg + sign * delta
            if -180 <= candidate <= 180:
                values.append(candidate)
    return tuple(values)


def _sample_segment(
    kinematics: Kinematics,
    start: np.ndarray,
    goal: np.ndarray,
    start_joints_deg: np.ndarray,
    start_angle_deg: float,
    final_angle_deg: float,
    *,
    plane_y_mm: float,
    elbow_margin_mm: float,
    sample_mm: float,
    max_joint_step_deg: float,
) -> tuple[Waypoint, ...] | None:
    distance = float(np.linalg.norm(goal[:3, 3] - start[:3, 3]))
    rotation_distance_deg = math.degrees(float(np.linalg.norm(rotation_log(goal[:3, :3] @ start[:3, :3].T))))
    count = max(1, math.ceil(distance / sample_mm), math.ceil(rotation_distance_deg / 2),
                math.ceil(abs(final_angle_deg - start_angle_deg) / 5))
    previous = np.asarray(start_joints_deg, dtype=float)
    waypoints = []
    for index in range(1, count + 1):
        fraction = index / count
        target = interpolate(start, goal, fraction)
        arm_angle = start_angle_deg + fraction * (final_angle_deg - start_angle_deg)
        solved = kinematics.ik(target, arm_angle, previous)
        if solved is None:
            return None
        joints = np.asarray(solved, dtype=float)
        if (joints.shape != previous.shape or not np.isfinite(joints).all()
                or not kinematics.within_limits(joints)
                or float(np.max(np.abs(joints - previous))) > max_joint_step_deg):
            return None
        actual, elbow = kinematics.fk(joints)
        actual = rigid(actual, "FK result")
        elbow = np.asarray(elbow, dtype=float)
        if elbow.shape != (3,) or not np.isfinite(elbow).all():
            return None
        position_error = float(np.linalg.norm(actual[:3, 3] - target[:3, 3]))
        orientation_error_deg = math.degrees(float(np.linalg.norm(rotation_log(target[:3, :3] @ actual[:3, :3].T))))
        if position_error > 3.0 or orientation_error_deg > 1.0:
            return None
        if elbow[1] > plane_y_mm - elbow_margin_mm:
            return None
        # A midpoint check prevents a narrow plane crossing between samples.
        midpoint = (previous + joints) / 2
        _, elbow_mid = kinematics.fk(midpoint)
        if float(elbow_mid[1]) > plane_y_mm - elbow_margin_mm:
            return None
        waypoints.append(Waypoint(target, joints, elbow, arm_angle))
        previous = joints
    return tuple(waypoints)


def plan(
    kinematics: Kinematics,
    current_flange_world_mm: np.ndarray,
    pregrasp_flange_world_mm: np.ndarray,
    grasp_flange_world_mm: np.ndarray,
    current_joints_deg: np.ndarray,
    current_arm_angle_deg: float,
    *,
    plane_y_mm: float,
    elbow_margin_mm: float = 30.0,
    sample_mm: float = 5.0,
    max_joint_step_deg: float = 5.0,
    max_arm_angle_change_deg: float = 90.0,
) -> CandidatePlan | None:
    """Try current arm angle, then ±5°, ±10° ...; never sends motion."""
    if (not math.isfinite(plane_y_mm) or elbow_margin_mm < 0 or sample_mm <= 0
            or max_joint_step_deg <= 0):
        raise ValueError("invalid protection-plane or sampling configuration")
    current = rigid(current_flange_world_mm, "current flange")
    pregrasp = rigid(pregrasp_flange_world_mm, "pregrasp flange")
    grasp = rigid(grasp_flange_world_mm, "grasp flange")
    current_joints = np.asarray(current_joints_deg, dtype=float)
    actual, elbow = kinematics.fk(current_joints)
    if (not kinematics.within_limits(current_joints)
            or np.linalg.norm(actual[:3, 3] - current[:3, 3]) > 3.0
            or math.degrees(float(np.linalg.norm(rotation_log(current[:3, :3] @ actual[:3, :3].T)))) > 1.0
            or float(elbow[1]) > plane_y_mm - elbow_margin_mm):
        return None
    for candidate in arm_angle_candidates(current_arm_angle_deg, max_arm_angle_change_deg):
        approach = _sample_segment(
            kinematics, current, pregrasp, current_joints, current_arm_angle_deg, candidate,
            plane_y_mm=plane_y_mm, elbow_margin_mm=elbow_margin_mm,
            sample_mm=sample_mm, max_joint_step_deg=max_joint_step_deg,
        )
        if approach is None:
            continue
        grasp_segment = _sample_segment(
            kinematics, pregrasp, grasp, approach[-1].joints_deg, candidate, candidate,
            plane_y_mm=plane_y_mm, elbow_margin_mm=elbow_margin_mm,
            sample_mm=sample_mm, max_joint_step_deg=max_joint_step_deg,
        )
        if grasp_segment is not None:
            return CandidatePlan(approach, grasp_segment, candidate, plane_y_mm)
    return None
