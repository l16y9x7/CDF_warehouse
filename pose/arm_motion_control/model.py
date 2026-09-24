"""Read-only URDF kinematics.  Units: millimetres and radians.

The physical arm mounting frame is tilted; the SDK arm-world frame instead
uses that mount's *origin* with chassis/torso-aligned axes.  Never use the
physical base rotation as an SDK-world rotation.
"""

from __future__ import annotations

import math
import xml.etree.ElementTree as ET
import zipfile
from pathlib import Path

import numpy as np

if __package__:
    from .geometry import rigid
else:
    from geometry import rigid


RIGHT_BASE = "AR5-5_08R-W4C1C9-ZY2_base"
RIGHT_FLANGE = "AR5-5_08R-W4C1C9-ZY2_flan_link"


def rotation(axis: np.ndarray, angle: float) -> np.ndarray:
    axis = np.asarray(axis, dtype=float)
    axis /= np.linalg.norm(axis)
    x, y, z = axis
    c, s, v = math.cos(angle), math.sin(angle), 1 - math.cos(angle)
    return np.array([
        [c + x*x*v, x*y*v - z*s, x*z*v + y*s],
        [y*x*v + z*s, c + y*y*v, y*z*v - x*s],
        [z*x*v - y*s, z*y*v + x*s, c + z*z*v],
    ])


def origin_matrix(element: ET.Element | None) -> np.ndarray:
    result = np.eye(4)
    if element is None:
        return result
    result[:3, 3] = np.fromstring(element.attrib.get("xyz", "0 0 0"), sep=" ") * 1000
    roll, pitch, yaw = np.fromstring(element.attrib.get("rpy", "0 0 0"), sep=" ")
    result[:3, :3] = (rotation(np.array([0., 0., 1.]), yaw)
                       @ rotation(np.array([0., 1., 0.]), pitch)
                       @ rotation(np.array([1., 0., 0.]), roll))
    return result


class RobotModel:
    def __init__(self, urdf_zip: str | Path, side: str = "right"):
        if side not in ("right", "left"):
            raise ValueError("side must be right or left")
        with zipfile.ZipFile(urdf_zip) as archive:
            name = next((name for name in archive.namelist() if name.endswith(".urdf")), None)
            if name is None:
                raise ValueError("URDF archive has no .urdf")
            root = ET.fromstring(archive.read(name))
        self.by_child = {joint.find("child").attrib["link"]: joint for joint in root.findall("joint")}
        self.trunk_chain = self.chain("chassis_link", "Chest_link")
        prefix = "AR5-5_08R-W4C1C9-ZY2" if side == "right" else "AR5-5_08L-W4C1C9-ZY2"
        self.arm_chain = self.chain("Chest_link", prefix + "_flan_link")
        self.mount_chain = self.chain("Chest_link", prefix + "_base")
        self.mount_chest = self.forward(self.mount_chain, [])
        self.actuated_arm = [joint for joint in self.arm_chain if joint.attrib["type"] in ("revolute", "continuous")]
        if len(self.actuated_arm) != 7:
            raise ValueError("expected a seven-joint right arm")
        self.lower = np.array([float(joint.find("limit").attrib["lower"]) for joint in self.actuated_arm])
        self.upper = np.array([float(joint.find("limit").attrib["upper"]) for joint in self.actuated_arm])

    def chain(self, parent: str, child: str) -> list[ET.Element]:
        path = []
        while child != parent:
            joint = self.by_child[child]
            path.append(joint)
            child = joint.find("parent").attrib["link"]
        return path[::-1]

    @staticmethod
    def forward(chain: list[ET.Element], joint_rad: list[float] | np.ndarray) -> np.ndarray:
        matrix = np.eye(4)
        index = 0
        for joint in chain:
            matrix = matrix @ origin_matrix(joint.find("origin"))
            if joint.attrib["type"] in ("revolute", "continuous"):
                if index >= len(joint_rad):
                    raise ValueError("too few joint angles")
                axis = np.fromstring(joint.find("axis").attrib["xyz"], sep=" ")
                turn = np.eye(4)
                turn[:3, :3] = rotation(axis, float(joint_rad[index]))
                matrix = matrix @ turn
                index += 1
        if index != len(joint_rad):
            raise ValueError("too many joint angles")
        return rigid(matrix, "URDF forward result")

    def torso_world(self, trunk_joints_deg: list[float]) -> np.ndarray:
        if len(trunk_joints_deg) != 4 or not np.isfinite(trunk_joints_deg).all():
            raise ValueError("expected four finite trunk joints")
        return self.forward(self.trunk_chain, np.radians(trunk_joints_deg))

    def right_shoulder_world(self, trunk_joints_deg: list[float]) -> np.ndarray:
        """T_chassis_right_arm_sdk_world, not the tilted physical arm base."""
        torso = self.torso_world(trunk_joints_deg)
        shoulder = torso.copy()
        shoulder[:3, 3] += torso[:3, :3] @ self.mount_chest[:3, 3]
        return shoulder

    shoulder_world = right_shoulder_world

    def arm_frames_world(self, trunk_joints_deg: list[float], arm_joints_deg: list[float]) -> tuple[np.ndarray, np.ndarray]:
        """Return world flange and J4-origin elbow; J4 origin is elbow proxy."""
        if len(arm_joints_deg) != 7 or not np.isfinite(arm_joints_deg).all():
            raise ValueError("expected seven finite right-arm joints")
        frame = self.torso_world(trunk_joints_deg)
        elbow = None
        index = 0
        for joint in self.arm_chain:
            frame = frame @ origin_matrix(joint.find("origin"))
            if joint.attrib["type"] in ("revolute", "continuous"):
                index += 1
                if index == 4:
                    elbow = frame[:3, 3].copy()
                axis = np.fromstring(joint.find("axis").attrib["xyz"], sep=" ")
                turn = np.eye(4)
                turn[:3, :3] = rotation(axis, math.radians(float(arm_joints_deg[index - 1])))
                frame = frame @ turn
        if elbow is None:
            raise ValueError("J4 elbow origin missing")
        return rigid(frame, "flange"), elbow

    def arm_joints_within_limits(self, arm_joints_deg: list[float], margin_deg: float = 1.0) -> bool:
        values = np.radians(arm_joints_deg)
        return bool(np.all(values >= self.lower + math.radians(margin_deg))
                    and np.all(values <= self.upper - math.radians(margin_deg)))
