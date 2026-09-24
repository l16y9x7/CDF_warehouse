"""Read-only bridge for an ALREADY constructed RightArmReadOnly snapshot.

Importing this module and constructing this wrapper cannot connect, power or
move a robot. The injected snapshot owns SDK connections and their lifetime.
"""
from __future__ import annotations

import numpy as np

from geometry import rigid


IK_INFEASIBLE_CODES = {-50102, -50114, -50519, -50002}


class RightArmGuardKinematics:
    def __init__(self, readonly_arm):
        self._arm = readonly_arm
        self.world_from_torso_mm = rigid(
            readonly_arm.urdf.torso_world(list(readonly_arm.trunk_joints_deg)),
            'world_from_Chest_link',
        ).copy()

    def ik(self, flange_world_mm, arm_angle_deg, seed_deg):
        del seed_deg  # SDK uses confData; the planner independently checks joint continuity.
        target = self._arm.target_cartesian(flange_world_mm, arm_angle_deg)
        ec = {}
        joints = self._arm.right.model().calcIk(target, self._arm.toolset, ec)
        if ec.get('ec', 0) in IK_INFEASIBLE_CODES:
            return None
        if ec.get('ec', 0):
            raise RuntimeError(f"SDK IK error: {ec.get('message', '')} (ec={ec['ec']})")
        radians = np.asarray(joints, dtype=float)
        if radians.shape != (7,) or not np.isfinite(radians).all():
            raise ValueError('SDK IK returned invalid seven-axis data')
        return np.degrees(radians)

    def fk(self, joints_deg):
        return self._arm.fk(joints_deg)

    def within_limits(self, joints_deg):
        return self._arm.within_limits(joints_deg)
