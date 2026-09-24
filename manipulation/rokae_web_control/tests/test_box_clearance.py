"""Offline plane/frame math; no camera, service or robot connections."""
import copy
import unittest

import numpy as np

from rokae_web.box_clearance import front_panel_from_response, clearance_in_trunk
from rokae_web.grasp_pose import world_grasp_to_right_shoulder
from rokae_web.grasp_test import flange_targets
from rokae_web.head_kinematics import rpy_rotation


class BoxClearanceTests(unittest.TestCase):
    def setUp(self):
        self.request = {"base_frame": "chassis_link", "camera_frame": "camera",
                        "T_unit": "m", "T_chassis_camera": np.eye(4).tolist()}
        self.response = {"front_panel_valid": True, "output_frame": "camera", "output_unit": "mm",
                         "front_panel_plane_point_camera_mm": [400, 0, 700],
                         "front_panel_plane_normal_camera": [-1, 0, 0],
                         "front_panel_top_edge_midpoint_camera_mm": [400, 0, 750]}
        self.world = {"frame": "chassis_link", "grasp_pose_world_mm_deg": [500, -100, 720, 180, -90, 0]}

    def calculate(self):
        return clearance_in_trunk(front_panel_from_response(self.request, self.response), self.world, np.eye(4))

    def test_axis_distance_drives_pregrasp_and_both_added_arm_targets(self):
        box = self.calculate()
        self.assertEqual(box["D_mm"], 400)
        self.assertEqual(box["d_mm"], 100)
        self.assertEqual(box["pregrasp_virtual_length_mm"], 350)
        shoulder = world_grasp_to_right_shoulder(self.world, np.eye(4), box, np.eye(4))
        targets = {k: p for k, _, p in flange_targets(shoulder)}
        self.assertEqual(targets["pregrasp"], [150, -100, 720, 180, -90, 0])
        self.assertEqual(targets["grasp"], [330, -100, 720, 180, -90, 0])
        self.assertEqual(targets["lift"], [330, -100, 760, 180, -90, 0])
        self.assertEqual(targets["arm_retreat"], [210, -100, 760, 180, -90, 0])
        self.assertEqual(targets["clearance_lift"], [210, -100, 835, 180, -90, 0])

    def test_tilted_panel_uses_intersection_at_object_not_plane_point_or_origin(self):
        # x + 0.2y = 400. Object y=-100 crosses the panel at x=420.
        self.response["front_panel_plane_normal_camera"] = (np.array([-1, -.2, 0]) / np.sqrt(1.04)).tolist()
        box = self.calculate()
        self.assertAlmostEqual(box["D_mm"], 400)
        self.assertAlmostEqual(box["d_mm"], 80)
        self.assertAlmostEqual(box["edge_intersection_mm"][0], 420)
        # Changing the arbitrary point used to describe the same plane changes nothing.
        self.response["front_panel_plane_point_camera_mm"] = [380, 100, 1000]
        self.assertAlmostEqual(self.calculate()["d_mm"], 80)

    def test_rotated_translated_chest_and_camera_preserve_local_distances(self):
        baseline = self.calculate()
        chest = np.eye(4)
        chest[:3, :3] = rpy_rotation(np.radians([8, -12, 42]))
        chest[:3, 3] = [.2, -.3, .5]
        camera = np.eye(4)
        camera[:3, :3] = rpy_rotation(np.radians([-20, 18, 91]))
        camera[:3, 3] = [.17, -.1, .8]
        for key in ("front_panel_plane_point_camera_mm", "front_panel_top_edge_midpoint_camera_mm"):
            world_point = chest[:3, :3] @ self.response[key] + 1000 * chest[:3, 3]
            self.response[key] = (camera[:3, :3].T @ (world_point - 1000 * camera[:3, 3])).tolist()
        self.response["front_panel_plane_normal_camera"] = (camera[:3, :3].T @ chest[:3, :3] @ np.array([-1, 0, 0])).tolist()
        self.request["T_chassis_camera"] = camera.tolist()
        self.world["grasp_pose_world_mm_deg"][:3] = (chest[:3, :3] @ np.array([500, -100, 720]) + 1000 * chest[:3, 3]).tolist()
        box = clearance_in_trunk(front_panel_from_response(self.request, self.response), self.world, chest)
        self.assertAlmostEqual(box["D_mm"], baseline["D_mm"])
        self.assertAlmostEqual(box["d_mm"], baseline["d_mm"])
        shoulder = world_grasp_to_right_shoulder(self.world, chest, box, chest)
        np.testing.assert_allclose(shoulder["grasp_pose_right_shoulder_mm_deg"], [330, -100, 720, 180, -90, 0], atol=1e-7)

    def test_normal_sign_does_not_change_distances(self):
        expected = self.calculate()
        self.response["front_panel_plane_normal_camera"] = [1, 0, 0]
        box = self.calculate()
        self.assertEqual(box["D_mm"], expected["D_mm"])
        self.assertEqual(box["d_mm"], expected["d_mm"])

    def test_missing_bad_or_inconsistent_panel_is_rejected(self):
        cases = [{"front_panel_plane_normal_camera": [0, 0, 0]},
                 {"front_panel_plane_normal_camera": [float('nan'), 0, 0]},
                 {"front_panel_plane_point_camera_mm": None},
                 {"front_panel_top_edge_midpoint_camera_mm": None}]
        for override in cases:
            with self.subTest(override=override), self.assertRaises(ValueError):
                front_panel_from_response(self.request, {**self.response, **override})

    def test_parallel_plane_negative_d_and_plane_behind_robot_rejected(self):
        for normal, point, obj in [([0, 1, 0], [400, 0, 700], [500, -100, 720]),
                                    ([-1, 0, 0], [400, 0, 700], [399, -100, 720]),
                                    ([-1, 0, 0], [-400, 0, 700], [500, -100, 720])]:
            self.response.update(front_panel_plane_normal_camera=normal, front_panel_plane_point_camera_mm=point)
            self.world["grasp_pose_world_mm_deg"][:3] = obj
            with self.subTest(normal=normal, point=point, obj=obj), self.assertRaises(ValueError):
                self.calculate()

    def test_new_trunk_origin_reprojects_saved_world_plane_and_object(self):
        panel = front_panel_from_response(self.request, self.response)
        chest = np.eye(4)
        chest[0, 3] = .08
        box = clearance_in_trunk(panel, self.world, chest)
        self.assertAlmostEqual(box["D_mm"], 320)
        self.assertAlmostEqual(box["d_mm"], 100)
        shoulder = world_grasp_to_right_shoulder(self.world, chest, box, chest)
        self.assertAlmostEqual(shoulder["pregrasp_pose_right_shoulder_mm_deg"][0], 70)


if __name__ == '__main__':
    unittest.main()
