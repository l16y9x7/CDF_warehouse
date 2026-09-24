"""Read state and ask the AR SDK model for IK; no motion/power API is used."""

from __future__ import annotations

import math
import sys
from pathlib import Path

import numpy as np

from geometry import inverse, rigid
from model import RobotModel
from planner import rotation_log


def pose_matrix(values: list[float], *, translation_scale: float = 1000.0) -> np.ndarray:
    if len(values) != 6 or not np.isfinite(values).all():
        raise ValueError("invalid six-dimensional SDK pose")
    x, y, z, roll, pitch, yaw = (float(value) for value in values)
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    result = np.eye(4)
    result[:3, :3] = np.array([
        [cy*cp, cy*sp*sr-sy*cr, cy*sp*cr+sy*sr],
        [sy*cp, sy*sp*sr+cy*cr, sy*sp*cr-cy*sr],
        [-sp, cp*sr, cp*cr],
    ])
    result[:3, 3] = [x * translation_scale, y * translation_scale, z * translation_scale]
    return rigid(result, "SDK pose")


def matrix_pose(transform: np.ndarray) -> list[float]:
    transform = rigid(transform, "target pose")
    r = transform[:3, :3]
    horizontal = math.hypot(float(r[0, 0]), float(r[1, 0]))
    pitch = math.atan2(-float(r[2, 0]), horizontal)
    if horizontal > 1e-9:
        roll, yaw = math.atan2(float(r[2, 1]), float(r[2, 2])), math.atan2(float(r[1, 0]), float(r[0, 0]))
    else:
        roll, yaw = math.atan2(-float(r[1, 2]), float(r[1, 1])), 0.0
    return list(transform[:3, 3] / 1000) + [roll, pitch, yaw]


def _call(name: str, function: object, *args: object) -> object:
    ec: dict = {}
    result = function(*args, ec)
    if ec.get("ec", 0):
        raise RuntimeError(f"{name}: {ec.get('message', 'SDK error')} ({ec['ec']})")
    return result


class RightArmReadOnly:
    def __init__(self, sdk_root: str, urdf_zip: str, local_ip: str, right_ip: str, trunk_ip: str):
        sys.path.insert(0, str(Path(sdk_root) / "rokae_xcore"))
        import xCoreSDK_python as sdk

        self.sdk = sdk
        self.urdf = RobotModel(urdf_zip)
        self.right = sdk.ArRobot(right_ip, local_ip)
        self.trunk = sdk.PCB4Robot(trunk_ip)
        self.trunk_joints_deg = np.degrees(list(_call("read trunk joints", self.trunk.jointPos))[:4])
        self.current_joints_deg = np.degrees(list(_call("read arm joints", self.right.jointPos))[:7])
        self.current_cart = _call("read arm pose", self.right.cartPosture, sdk.CoordinateType.endInRef)
        self.toolset = _call("read calibrated toolset", self.right.toolset)
        self.arm_angle_deg = math.degrees(float(self.current_cart.elbow))
        self.world_from_shoulder = self.urdf.right_shoulder_world(self.trunk_joints_deg.tolist())
        self.shoulder_from_ref = pose_matrix(list(self.toolset.ref.trans) + list(self.toolset.ref.rpy))
        self.flange_from_end = pose_matrix(list(self.toolset.end.trans) + list(self.toolset.end.rpy))
        self.current_flange_world, self.current_elbow_world = self.fk(self.current_joints_deg)
        current_tcp_ref = pose_matrix(list(self.current_cart.trans) + list(self.current_cart.rpy))
        sdk_tcp_world = self.world_from_shoulder @ self.shoulder_from_ref @ current_tcp_ref
        model_tcp_world = self.current_flange_world @ self.flange_from_end
        position_delta = float(np.linalg.norm(sdk_tcp_world[:3, 3] - model_tcp_world[:3, 3]))
        rotation_delta = math.degrees(float(np.linalg.norm(rotation_log(sdk_tcp_world[:3, :3] @ model_tcp_world[:3, :3].T))))
        if position_delta > 5.0 or rotation_delta > 2.0:
            self.close()
            raise RuntimeError(f"SDK/URDF TCP frame mismatch: {position_delta:.2f} mm, {rotation_delta:.2f} deg")

    def ik(self, world_flange_mm: np.ndarray, elbow_angle_deg: float, seed_deg: np.ndarray) -> np.ndarray | None:
        del seed_deg  # SDK uses its own configuration descriptor from live readback.
        target = self.target_cartesian(world_flange_mm, elbow_angle_deg)
        try:
            joints_rad = _call("SDK calcIk", self.right.model().calcIk, target, self.toolset)
        except RuntimeError:
            return None  # Controller model reported this arm-angle/pose candidate infeasible.
        joints = np.degrees(list(joints_rad)[:7])
        return joints if len(joints) == 7 and np.isfinite(joints).all() else None

    def target_cartesian(self, world_flange_mm: np.ndarray, elbow_angle_deg: float):
        shoulder_from_tcp = inverse(self.world_from_shoulder) @ world_flange_mm @ self.flange_from_end
        ref_from_tcp = inverse(self.shoulder_from_ref) @ shoulder_from_tcp
        target = self.sdk.CartesianPosition(matrix_pose(ref_from_tcp))
        target.elbow = math.radians(float(elbow_angle_deg))
        target.hasElbow = True
        target.confData = list(self.current_cart.confData)
        return target

    def fk(self, joint_deg: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
        return self.urdf.arm_frames_world(self.trunk_joints_deg.tolist(), list(joint_deg))

    def within_limits(self, joint_deg: np.ndarray) -> bool:
        return self.urdf.arm_joints_within_limits(list(joint_deg))

    def close(self) -> None:
        for robot in (getattr(self, "right", None), getattr(self, "trunk", None)):
            if robot is not None:
                try:
                    _call("disconnect", robot.disconnectFromRobot)
                except Exception:
                    pass
