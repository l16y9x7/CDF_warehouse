from __future__ import annotations

import math
import zipfile
from dataclasses import dataclass
from pathlib import Path
from typing import Iterable
from xml.etree import ElementTree

import numpy as np


def _numbers(text: str | None, count: int, default: float = 0.0) -> np.ndarray:
    if not text:
        return np.full(count, default, dtype=np.float64)
    values = np.asarray([float(value) for value in text.split()], dtype=np.float64)
    if values.size != count:
        raise ValueError(f"期望 {count} 个数值，实际得到 {values.size}: {text!r}")
    return values


def rotation_about_axis(axis: Iterable[float], angle_rad: float) -> np.ndarray:
    vector = np.asarray(list(axis), dtype=np.float64)
    length = float(np.linalg.norm(vector))
    if length <= 0.0:
        raise ValueError("关节旋转轴不能为零向量")
    x, y, z = vector / length
    c = math.cos(float(angle_rad))
    s = math.sin(float(angle_rad))
    one_minus_c = 1.0 - c
    return np.asarray(
        [
            [c + x * x * one_minus_c, x * y * one_minus_c - z * s, x * z * one_minus_c + y * s],
            [y * x * one_minus_c + z * s, c + y * y * one_minus_c, y * z * one_minus_c - x * s],
            [z * x * one_minus_c - y * s, z * y * one_minus_c + x * s, c + z * z * one_minus_c],
        ],
        dtype=np.float64,
    )


def rpy_rotation(rpy_rad: Iterable[float]) -> np.ndarray:
    roll, pitch, yaw = [float(value) for value in rpy_rad]
    return (
        rotation_about_axis((0.0, 0.0, 1.0), yaw)
        @ rotation_about_axis((0.0, 1.0, 0.0), pitch)
        @ rotation_about_axis((1.0, 0.0, 0.0), roll)
    )


def make_transform(rotation: np.ndarray | None = None, translation: Iterable[float] | None = None) -> np.ndarray:
    transform = np.eye(4, dtype=np.float64)
    if rotation is not None:
        transform[:3, :3] = np.asarray(rotation, dtype=np.float64).reshape(3, 3)
    if translation is not None:
        transform[:3, 3] = np.asarray(list(translation), dtype=np.float64).reshape(3)
    return transform


def invert_transform(transform: np.ndarray) -> np.ndarray:
    source = np.asarray(transform, dtype=np.float64).reshape(4, 4)
    rotation = source[:3, :3]
    translation = source[:3, 3]
    return make_transform(rotation.T, -(rotation.T @ translation))


def rotation_to_rpy_deg(rotation: np.ndarray) -> list[float]:
    matrix = np.asarray(rotation, dtype=np.float64).reshape(3, 3)
    pitch = math.asin(float(np.clip(-matrix[2, 0], -1.0, 1.0)))
    if abs(math.cos(pitch)) > 1e-9:
        roll = math.atan2(float(matrix[2, 1]), float(matrix[2, 2]))
        yaw = math.atan2(float(matrix[1, 0]), float(matrix[0, 0]))
    else:
        roll = math.atan2(float(-matrix[1, 2]), float(matrix[1, 1]))
        yaw = 0.0
    return [math.degrees(roll), math.degrees(pitch), math.degrees(yaw)]


@dataclass(frozen=True)
class URDFJoint:
    name: str
    joint_type: str
    parent: str
    child: str
    origin_xyz_m: np.ndarray
    origin_rpy_rad: np.ndarray
    axis: np.ndarray


def _read_urdf(path: str | Path) -> tuple[str, str]:
    source = Path(path).expanduser().resolve()
    if source.suffix.lower() == ".zip":
        with zipfile.ZipFile(source) as archive:
            members = [name for name in archive.namelist() if name.lower().endswith(".urdf")]
            if len(members) != 1:
                raise ValueError(f"URDF 压缩包内应恰好有一个 .urdf，实际为 {members}")
            return archive.read(members[0]).decode("utf-8"), f"{source}!/{members[0]}"
    return source.read_text(encoding="utf-8"), str(source)


class UpperBodySixDofKinematics:
    """URDF-based chassis_link -> Head_link forward kinematics for trunk 4 + head 2."""

    EXPECTED_JOINTS = (
        "Calf_joint",
        "Thigh_joint",
        "Waist_joint",
        "Chest_joint",
        "Neck_joint",
        "Head_joint",
    )
    LEFT_ARM_BASE_LINK = "AR5-5_08L-W4C1C9-ZY2_base"
    RIGHT_ARM_BASE_LINK = "AR5-5_08R-W4C1C9-ZY2_base"

    def __init__(
        self,
        urdf_path: str | Path,
        base_link: str = "chassis_link",
        tip_link: str = "Head_link",
    ) -> None:
        xml_text, source_name = _read_urdf(urdf_path)
        self.urdf_source = source_name
        self.base_link = base_link
        self.tip_link = tip_link
        root = ElementTree.fromstring(xml_text)
        joints_by_child: dict[str, URDFJoint] = {}
        for element in root.findall("joint"):
            parent = element.find("parent")
            child = element.find("child")
            if parent is None or child is None:
                continue
            origin = element.find("origin")
            axis = element.find("axis")
            joint = URDFJoint(
                name=str(element.attrib["name"]),
                joint_type=str(element.attrib.get("type", "fixed")),
                parent=str(parent.attrib["link"]),
                child=str(child.attrib["link"]),
                origin_xyz_m=_numbers(None if origin is None else origin.attrib.get("xyz"), 3),
                origin_rpy_rad=_numbers(None if origin is None else origin.attrib.get("rpy"), 3),
                axis=_numbers(None if axis is None else axis.attrib.get("xyz"), 3, default=0.0),
            )
            joints_by_child[joint.child] = joint
        self._joints_by_child = joints_by_child

        reverse_chain: list[URDFJoint] = []
        current = tip_link
        visited: set[str] = set()
        while current != base_link:
            if current in visited or current not in joints_by_child:
                raise ValueError(f"URDF 中找不到从 {base_link} 到 {tip_link} 的唯一关节链")
            visited.add(current)
            joint = joints_by_child[current]
            reverse_chain.append(joint)
            current = joint.parent
        self.joints = tuple(reversed(reverse_chain))
        self.actuated_joints = tuple(
            joint for joint in self.joints if joint.joint_type in ("revolute", "continuous", "prismatic")
        )
        names = tuple(joint.name for joint in self.actuated_joints)
        if names != self.EXPECTED_JOINTS:
            raise ValueError(f"躯干+头部关节链与预期不符: {names}")

    @property
    def joint_names(self) -> tuple[str, ...]:
        return tuple(joint.name for joint in self.actuated_joints)

    def forward_rad(self, joint_values_rad: Iterable[float], tip_link: str | None = None) -> np.ndarray:
        values = np.asarray(list(joint_values_rad), dtype=np.float64)
        if values.size != len(self.actuated_joints):
            raise ValueError(f"6 自由度关节值应有 6 个，实际得到 {values.size}")
        positions = dict(zip(self.joint_names, values.tolist()))
        requested_tip = tip_link or self.tip_link
        transform = np.eye(4, dtype=np.float64)
        reached = requested_tip == self.base_link
        for joint in self.joints:
            transform = transform @ make_transform(
                rpy_rotation(joint.origin_rpy_rad),
                joint.origin_xyz_m,
            )
            if joint.joint_type in ("revolute", "continuous"):
                transform = transform @ make_transform(
                    rotation_about_axis(joint.axis, positions[joint.name])
                )
            elif joint.joint_type == "prismatic":
                transform = transform @ make_transform(
                    translation=joint.axis * positions[joint.name]
                )
            elif joint.joint_type != "fixed":
                raise ValueError(f"暂不支持 URDF 关节类型: {joint.joint_type}")
            if joint.child == requested_tip:
                reached = True
                break
        if not reached:
            raise ValueError(f"{requested_tip} 不在 {self.base_link} -> {self.tip_link} 链上")
        return transform

    def forward_deg(self, joint_values_deg: Iterable[float], tip_link: str | None = None) -> np.ndarray:
        return self.forward_rad(np.radians(list(joint_values_deg)), tip_link=tip_link)

    def right_shoulder_sdk_world(self, trunk_joints_deg: Iterable[float]) -> np.ndarray:
        """chassis_link -> SDK right-shoulder world, in metres.

        The SDK world uses the physical mount's origin but chest-aligned axes;
        the tilted mounting rotation is deliberately not applied.
        """
        return self._shoulder_sdk_world(trunk_joints_deg, self.RIGHT_ARM_BASE_LINK)

    def left_shoulder_sdk_world(self, trunk_joints_deg: Iterable[float]) -> np.ndarray:
        return self._shoulder_sdk_world(trunk_joints_deg, self.LEFT_ARM_BASE_LINK)

    def _shoulder_sdk_world(self, trunk_joints_deg, base_link):
        trunk = np.asarray(list(trunk_joints_deg), dtype=np.float64)
        if trunk.shape != (4,) or not np.isfinite(trunk).all():
            raise ValueError("肩部坐标转换需要四个有效躯干关节角")
        chest = self.forward_deg([*trunk.tolist(), 0.0, 0.0], tip_link="Chest_link")
        reverse_chain: list[URDFJoint] = []
        current = base_link
        visited: set[str] = set()
        while current != "Chest_link":
            if current in visited or current not in self._joints_by_child:
                raise ValueError("URDF 中找不到胸部到手臂安装基座的固定链")
            visited.add(current)
            joint = self._joints_by_child[current]
            if joint.joint_type != "fixed":
                raise ValueError("胸部到手臂安装基座的链不是固定连接")
            reverse_chain.append(joint)
            current = joint.parent
        mount = np.eye(4, dtype=np.float64)
        for joint in reversed(reverse_chain):
            mount = mount @ make_transform(
                rpy_rotation(joint.origin_rpy_rad), joint.origin_xyz_m
            )
        shoulder = chest.copy()
        shoulder[:3, 3] += chest[:3, :3] @ mount[:3, 3]
        return shoulder

    def camera_to_base(
        self,
        joint_values_deg: Iterable[float],
        transform_head_from_camera: np.ndarray,
    ) -> np.ndarray:
        return self.forward_deg(joint_values_deg) @ np.asarray(
            transform_head_from_camera,
            dtype=np.float64,
        ).reshape(4, 4)

    def point_camera_to_base(
        self,
        point_camera_m: Iterable[float],
        joint_values_deg: Iterable[float],
        transform_head_from_camera: np.ndarray,
    ) -> np.ndarray:
        point = np.ones(4, dtype=np.float64)
        point[:3] = np.asarray(list(point_camera_m), dtype=np.float64).reshape(3)
        return (self.camera_to_base(joint_values_deg, transform_head_from_camera) @ point)[:3]
