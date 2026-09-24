from __future__ import annotations

import copy
import base64
import json
import tempfile
import unittest
from pathlib import Path
from types import SimpleNamespace

import numpy as np

from rokae_web.camera import CameraSnapshot
from rokae_web.config import DEFAULT_CONFIG
from rokae_web.pose_estimation import PoseEstimationClient, PoseEstimationError


class PoseEstimationClientTests(unittest.TestCase):
    def test_successful_result_is_saved_with_overlay(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            config = copy.deepcopy(DEFAULT_CONFIG)
            config["camera"]["data_directory"] = temporary
            config["pose_estimation"]["grasp_height_trunk_mm_by_sku"] = {"Avene": 1025.0}
            client = PoseEstimationClient(config)
            chest = np.eye(4)
            chest[0, 3] = -.5
            client._kinematics = SimpleNamespace(forward_deg=lambda q, tip_link: chest)
            camera_matrix = np.asarray(
                [[612.0, 0.0, 640.0], [0.0, 612.0, 360.0], [0.0, 0.0, 1.0]],
                dtype=np.float64,
            )
            client._load_calibration = lambda: (
                camera_matrix,
                np.eye(4, dtype=np.float64),
                [1280, 720],
            )
            client._synchronized_transform = lambda before, after, transform: (
                np.eye(4, dtype=np.float64),
                [0.0] * 6,
                0.0,
            )
            client.health = lambda: {"available": True, "response": {"ok": True}}
            captured_request = {}

            def fake_request(url, *, method, payload, timeout):
                del url, method, timeout
                captured_request.update(payload)
                cloud = []
                for index in range(240):
                    angle = 2.0 * np.pi * (index % 24) / 24.0
                    height = 1000.0 + 10.0 * (index // 24)
                    cloud.append(f"{28.5 * np.cos(angle):.6f} {28.5 * np.sin(angle):.6f} {height:.6f}")
                ply = ("ply\nformat ascii 1.0\n"
                       f"element vertex {len(cloud)}\n"
                       "property float x\nproperty float y\nproperty float z\n"
                       "end_header\n" + "\n".join(cloud) + "\n")
                return {
                    "ok": True,
                    "sku_typ": "bottle",
                    "output_frame": client.CAMERA_FRAME,
                    "output_unit": "mm",
                    "body_radius_mm": 28.5,
                    "selected_instance_id": 1,
                    "sam3_score": 0.9,
                    "axis_fit_valid": True,
                    "reference_point_valid": True,
                    "axis_point_camera_mm": [0.0, 0.0, 1000.0],
                    "axis_direction_camera_up": [0.0, 0.0, 1.0],
                    "reference_point_camera_mm": [0.0, 0.0, 1000.0],
                    "reference_point_chassis_mm": [0.0, 0.0, 1000.0],
                    "front_panel_valid": True,
                    "front_panel_plane_point_camera_mm": [-100, 0, 1000],
                    "front_panel_plane_normal_camera": [-1, 0, 0],
                    "front_panel_top_edge_midpoint_camera_mm": [-100, 0, 1100],
                    "artifacts": {"point_cloud_ply_base64": base64.b64encode(
                        ply.encode("ascii")
                    ).decode("ascii")},
                }

            client._request_json = fake_request
            current_camera_matrix = camera_matrix.copy()
            current_camera_matrix[0, 0] = 610.0
            snapshot = CameraSnapshot(
                rgb_bgr=np.zeros((720, 1280, 3), dtype=np.uint8),
                depth_aligned_mm=np.full((720, 1280), 1000.0, dtype=np.float32),
                captured_at="2026-09-16T12:00:00.000+08:00",
                color_timestamp_ms=1.0,
                depth_timestamp_ms=1.0,
                sequence=10,
                camera_info={
                    "source": "ros2",
                    "color_intrinsics": {"width": 1280, "height": 720,
                                         "camera_matrix": current_camera_matrix.tolist()},
                },
            )
            state = {
                "joints_deg": {"trunk": [0.0] * 4, "head": [0.0] * 2},
                "operation_state": {"trunk": "idle"},
                "pose_frames": {"trunk": "trunk_controller_ref"},
                "poses": {"trunk": [0.0] * 6},
                "toolsets": {"trunk": {"end": [0.0] * 6, "ref": [0.0] * 6}},
            }

            result = client.estimate(snapshot, state, state)

            self.assertTrue(result["usable"])
            self.assertTrue(result["grasp_test_usable"])
            self.assertAlmostEqual(result["box_clearance"]["D_mm"], 400)
            self.assertAlmostEqual(result["box_clearance"]["d_mm"], 100)
            self.assertEqual(captured_request["target_type"], "sku")
            self.assertEqual(captured_request["sku_typ"], "bottle")
            self.assertNotIn("sku_id", captured_request)
            self.assertNotIn("class_name", captured_request)
            self.assertNotIn("z_ref_mm", captured_request)
            self.assertIs(captured_request["return_visualizations"], True)
            self.assertEqual(captured_request["side"], "RIGHT")
            self.assertEqual(captured_request["front_rule"]["front_axis_chassis"], [1, 0, 0])
            self.assertEqual(result["world_grasp"]["frame"], "chassis_link")
            self.assertEqual(result["world_grasp"]["grasp_pose_world_mm_deg"][3:],
                             [180.0, -90.0, 0.0])
            self.assertAlmostEqual(result["world_grasp"]["grasp_pose_world_mm_deg"][2],
                                   1025.0, delta=1e-6)
            self.assertEqual(result["world_grasp"]["grasp_point_rule"],
                             "fixed_trunk_height_vertical_reference_v2")
            self.assertEqual(captured_request["depth_unit"], "mm")
            self.assertEqual(captured_request["camera_frame"], client.CAMERA_FRAME)
            self.assertEqual(captured_request["base_frame"], client.BASE_FRAME)
            self.assertEqual(captured_request["K"], current_camera_matrix.tolist())
            self.assertNotEqual(captured_request["rgb_base64"], "")
            target = Path(result["directory"])
            self.assertTrue((target / "head_rgb.jpg").is_file())
            self.assertTrue((target / "head_depth_aligned.npy").is_file())
            self.assertTrue((target / "pose_estimation_overlay.jpg").is_file())
            stored = json.loads(
                (target / "pose_estimation_summary.json").read_text(encoding="utf-8")
            )
            self.assertTrue(stored["usable"])
            self.assertEqual(stored["world_grasp"], result["world_grasp"])
            self.assertEqual(client.latest_world_grasp()["world_grasp"], result["world_grasp"])
            # Reprojection must not reuse a saved point from the previous rule.
            stored["world_grasp"]["grasp_pose_world_mm_deg"][2] = 1069.5
            stored["world_grasp"]["grasp_pose_world_mm_deg"][1] = 1015.0
            (target / "pose_estimation_summary.json").write_text(
                json.dumps(stored), encoding="utf-8"
            )
            self.assertEqual(client.latest_world_grasp()["world_grasp"], result["world_grasp"])
            # Historical successful results have raw request/response files
            # but no precomputed world_grasp in their summary.
            stored.pop("world_grasp")
            (target / "pose_estimation_summary.json").write_text(
                json.dumps(stored), encoding="utf-8"
            )
            self.assertEqual(client.latest_world_grasp()["world_grasp"], result["world_grasp"])
            shoulder = np.eye(4)
            shoulder[:3, 3] = [0.1, 0.0, 0.0]
            client._kinematics = SimpleNamespace(
                right_shoulder_sdk_world=lambda trunk: shoulder,
                forward_deg=lambda q, tip_link: chest,
            )
            projected = client.reproject_latest_to_right_shoulder(state, state)
            self.assertEqual(projected["source_result_id"], result["result_id"])
            self.assertAlmostEqual(projected["world_grasp"]["grasp_point_trunk_mm"][1],10.0)
            self.assertAlmostEqual(projected["shoulder_grasp"]["grasp_pose_right_shoulder_mm_deg"][1],10.0)
            self.assertAlmostEqual(projected["shoulder_grasp"]["pregrasp_pose_right_shoulder_mm_deg"][1],10.0)
            self.assertAlmostEqual(
                projected["shoulder_grasp"]["grasp_pose_right_shoulder_mm_deg"][0],
                -270.0,
            )
            self.assertAlmostEqual(
                projected["shoulder_grasp"]["pregrasp_pose_right_shoulder_mm_deg"][0],
                -450.0,
            )
            # Reapply the configured height to raw historical axes, never reuse their old midpoint.
            client.config["grasp_height_trunk_mm_by_sku"]["Avene"] = 1035.0
            projected = client.reproject_latest_to_right_shoulder(state, state)
            self.assertAlmostEqual(projected["world_grasp"]["grasp_point_trunk_mm"][2], 1035.0)
            # A live reference translation changes world Z, while SDK Z stays at the fixed height.
            shifted = copy.deepcopy(state)
            shifted["poses"]["trunk"][2] = -50.0
            projected = client.reproject_latest_to_right_shoulder(shifted, shifted)
            self.assertAlmostEqual(projected["world_grasp"]["grasp_point_trunk_mm"][2], 1035.0)
            self.assertAlmostEqual(projected["world_grasp"]["grasp_pose_world_mm_deg"][2], 1085.0)
            # Results missing a historical trunk transform can still be converted with the current readback.
            request_path = target / "pose_estimation_request.json"
            saved_request = json.loads(request_path.read_text(encoding="utf-8"))
            saved_request.pop("T_chassis_trunk_ref_m")
            saved_request.pop("grasp_height_trunk_mm")
            request_path.write_text(json.dumps(saved_request), encoding="utf-8")
            self.assertAlmostEqual(client.reproject_latest_to_right_shoulder(state,state)["world_grasp"]["grasp_point_trunk_mm"][2],1035.0)
            client.config['sku_typ'] = 'box'
            self.assertAlmostEqual(client.reproject_latest_to_right_shoulder(state,state)['world_grasp']['grasp_point_trunk_mm'][2],1035.0)
            # Missing/invalid panel in this result must not reuse an older plane.
            response_path = target / "pose_estimation_response.json"
            response = json.loads(response_path.read_text(encoding="utf-8"))
            response["front_panel_plane_point_camera_mm"] = None
            response_path.write_text(json.dumps(response), encoding="utf-8")
            with self.assertRaisesRegex(PoseEstimationError, "前挡板"):
                client.reproject_latest_to_right_shoulder(state, state)


if __name__ == "__main__":
    unittest.main()
