"""Mirror the placement position and point the flange horizontally forward."""
import math

import numpy as np

from .arm_movel import pose_values, transform
from .grasp_test import flange_to_tcp
from .memory_points import vector


def mirrored_preplacement(left_tcp, toolsets, y=None):
    # Both SDK shoulder worlds have chest-aligned axes and mirrored origins.
    # Mirror the flange, then apply the right TCP, never mirror joint values or
    # Euler angles directly. S R S remains a proper rotation, including at gimbal lock.
    end = vector(toolsets['left_arm']['end'], 6, '左臂工具 TCP')
    end_pose = [v * 1000 for v in end[:3]] + [math.degrees(v) for v in end[3:]]
    left = transform(vector(left_tcp, 6, '左臂预放置位姿')) @ np.linalg.inv(transform(end_pose))
    reflection = np.diag([1., -1., 1., 1.])
    right_flange = pose_values(reflection @ left @ reflection)
    # Shoulder SDK axes are chest-aligned. Keep the mirrored flange XYZ,
    # replace its attitude, then apply the actual right TCP.
    right_flange[3:] = [180., -90., 0.]
    if y is not None:right_flange[1] = float(y)
    return flange_to_tcp(right_flange, toolsets['right_arm'])
