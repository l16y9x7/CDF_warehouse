from __future__ import annotations

import json
import sys
import unittest
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
from geometry import grasp_geometry, world_goal_in_arm_reference
from localization import from_4090
from model import RobotModel
from planner import arm_angle_candidates, plan
from sdk_readonly import matrix_pose, pose_matrix


PARENT = Path(__file__).resolve().parents[1]
ROOT = next((path for path in (PARENT / "offline_movel", PARENT / "offline_movel_20260917") if path.exists()), PARENT / "offline_movel")


class FakeKinematics:
    def ik(self, world_flange_mm, elbow_angle_deg, seed_deg):
        return np.array([world_flange_mm[0, 3], elbow_angle_deg], dtype=float)

    def fk(self, joint_deg):
        frame = np.eye(4)
        frame[0, 3] = joint_deg[0]
        elbow_y = 5.0 if joint_deg[1] == 0.0 and joint_deg[0] > 0 else -50.0
        return frame, np.array([0., elbow_y, 0.])

    def within_limits(self, joint_deg):
        return True


class MotionTests(unittest.TestCase):
    def test_4090_visible_top_is_not_diagnostic_reference(self):
        request = json.loads((ROOT / "pose_estimation_request.json").read_text())
        response = json.loads((ROOT / "pose_estimation_response.json").read_text())
        localization = from_4090(request, response)
        np.testing.assert_allclose(localization.geometry.grasp_world_mm, [577.73, -58.46, 852.76], atol=.05)
        self.assertGreater(localization.cylinder_band_points, 1000)
        self.assertGreater(abs(localization.geometry.grasp_world_mm[2] - response["reference_point_chassis_mm"][2]), 300)
        response["ok"] = False
        with self.assertRaises(ValueError):
            from_4090(request, response)

    def test_virtual_tool_and_reprojection(self):
        geometry = grasp_geometry([600, -60, 900], [0, 0, 1])
        np.testing.assert_allclose(geometry.flange_grasp_world[:3, 3], [345, -60, 880])
        np.testing.assert_allclose(geometry.flange_pregrasp_world[:3, 3], [245, -60, 880])
        shoulder = np.eye(4)
        shoulder[:3, 3] = [-100, -77.5, 1143]
        reprojected = world_goal_in_arm_reference(geometry.flange_grasp_world, shoulder)
        np.testing.assert_allclose(reprojected[:3, 3], [445, 17.5, -263])

    def test_sdk_pose_conversion_at_gimbal_lock(self):
        geometry = grasp_geometry([600, -60, 900], [0, 0, 1])
        flange = geometry.flange_grasp_world
        np.testing.assert_allclose(pose_matrix(matrix_pose(flange)), flange, atol=1e-8)

    def test_live_shoulder_frame_matches_recorded_readback(self):
        model = RobotModel(ROOT / "robot_urdf.zip")
        trunk = [-36.445782, -36.319393, -17.329571, .67516]
        arm = [-28.435341, 83.361385, 88.096921, 109.750424, 8.467685, 7.99015, -3.932556]
        shoulder = model.right_shoulder_world(trunk)
        flange, elbow = model.arm_frames_world(trunk, arm)
        self.assertLess(np.linalg.norm((np.linalg.inv(shoulder) @ flange)[:3, 3]
                                       - [214.68008, -193.264732, -349.459629]), 2.0)
        self.assertLess(elbow[1], 0)

    def test_elbow_plane_then_five_degree_search(self):
        current, pregrasp, grasp = (np.eye(4) for _ in range(3))
        pregrasp[0, 3], grasp[0, 3] = 10., 20.
        self.assertEqual(arm_angle_candidates(0, 10), (0, 5, -5, 10, -10))
        result = plan(FakeKinematics(), current, pregrasp, grasp,
                      np.array([0., 0.]), 0., plane_y_mm=0., elbow_margin_mm=30.)
        self.assertIsNotNone(result)
        self.assertEqual(result.selected_arm_angle_deg, 5.)
        self.assertIsNone(plan(FakeKinematics(), current, pregrasp, grasp,
                               np.array([0., 0.]), 0., plane_y_mm=-100., elbow_margin_mm=30.))


if __name__ == "__main__":
    unittest.main()
