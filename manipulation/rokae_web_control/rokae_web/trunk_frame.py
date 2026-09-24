"""Live bridge from the URDF world to the PCB4 SDK endInRef frame.

Chest_link is the PCB4 output flange in this robot's URDF. It is *not* the
controller reference frame: T_world_ref = T_world_flange T_flange_tcp T_tcp_ref.
All matrices here use metres. No controller calls or fixed-world assumptions.
"""
from __future__ import annotations

import numpy as np

from .head_kinematics import make_transform, rpy_rotation
from .pose_frames import POSE_FRAMES


TRUNK_FRAME = POSE_FRAMES["trunk"]


def pose_transform(values, *, mm_deg):
    v = np.asarray(values, dtype=float)
    if v.shape != (6,) or not np.isfinite(v).all():
        raise ValueError("躯干坐标变换需要六个有限位姿数值")
    return make_transform(rpy_rotation(np.radians(v[3:]) if mm_deg else v[3:]),
                          v[:3] / 1000.0 if mm_deg else v[:3])


def trunk_reference_in_chassis(state, kinematics):
    """Use the same readback's trunk joints, TCP pose and actual tool offset."""
    try:
        if state["pose_frames"]["trunk"] != TRUNK_FRAME:
            raise ValueError("躯干 Pose 不是 SDK endInRef 参考系")
        q = np.asarray(state["joints_deg"]["trunk"], dtype=float)
        if q.shape != (4,) or not np.isfinite(q).all():
            raise ValueError("躯干关节角无效")
        ref_tcp = pose_transform(state["poses"]["trunk"], mm_deg=True)
        flange_tcp = pose_transform(state["toolsets"]["trunk"]["end"], mm_deg=False)
    except (KeyError, TypeError) as exc:
        raise ValueError("缺少同次回读的躯干 SDK 位姿或工具变换，请重启新版后端并重新回读") from exc
    world_flange = kinematics.forward_deg([*q, 0.0, 0.0], tip_link="Chest_link")
    return world_flange @ flange_tcp @ np.linalg.inv(ref_tcp)


def stable_trunk_reference(before, after, kinematics):
    if before.get("toolsets", {}).get("trunk") != after.get("toolsets", {}).get("trunk"):
        raise ValueError("回读期间躯干工具或参考系改变")
    first = trunk_reference_in_chassis(before, kinematics)
    last = trunk_reference_in_chassis(after, kinematics)
    delta = np.linalg.inv(first) @ last
    angle = np.degrees(np.arccos(np.clip((np.trace(delta[:3, :3]) - 1) / 2, -1, 1)))
    if np.linalg.norm(delta[:3, 3]) > .001 or angle > .05:
        raise ValueError("躯干 SDK 与 URDF 变换回读不一致，请保持静止后重试")
    return last
