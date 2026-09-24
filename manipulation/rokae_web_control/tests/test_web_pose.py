from __future__ import annotations

import copy
import http.client
import json
import threading
import unittest
from pathlib import Path

from rokae_web.backends import MockChassisBackend, MockRobotBackend
from rokae_web.config import DEFAULT_CONFIG
from rokae_web.pose_estimation import PoseEstimationError
from rokae_web.service import ControlService
from rokae_web.web import make_server


class SavedWorldPose:
    fresh_frame_timeout = 0.1

    def __init__(self) -> None:
        self.available = True

    def reproject_latest_to_left_shoulder(self, before, after):
        result = self.reproject_latest_to_right_shoulder(before, after)
        result['sku_typ'] = 'box'
        result['shoulder_grasp'] = {'frame': 'left_arm_sdk_world'}
        return result

    def reproject_latest_to_right_shoulder(self, before, after):
        if not self.available:
            raise PoseEstimationError("尚无已保存的有效世界抓取位姿")
        return {
            "source_result_id": "20260917/pose_120000000_abcdef",
            "current_trunk_joints_deg": after["joints_deg"]["trunk"],
            "world_grasp": {
                "frame": "chassis_link",
                "grasp_pose_world_mm_deg": [500.0, 0.0, 800.0, 180.0, -90.0, 0.0],
            },
            "shoulder_grasp": {
                "frame": "right_arm_sdk_world",
                "grasp_pose_right_shoulder_mm_deg": [600.0, 50.0, -300.0, 180.0, -90.0, 0.0],
                "pregrasp_pose_right_shoulder_mm_deg": [490.0, 50.0, -300.0, 180.0, -90.0, 0.0],
            },
        }


class PoseHTTPTests(unittest.TestCase):
    def setUp(self) -> None:
        self.estimator = SavedWorldPose()
        self.service = ControlService(
            copy.deepcopy(DEFAULT_CONFIG), MockRobotBackend(), MockChassisBackend(),
            False, pose_estimator=self.estimator,
        )
        static = Path(__file__).resolve().parents[1] / "static"
        self.server = make_server("127.0.0.1", 0, self.service, static)
        self.thread = threading.Thread(target=self.server.serve_forever, daemon=True)
        self.thread.start()

    def tearDown(self) -> None:
        self.server.shutdown()
        self.server.server_close()
        self.thread.join(timeout=2)
        self.service.close()

    def request(self, arm='right', body=None):
        connection = http.client.HTTPConnection("127.0.0.1", self.server.server_port, timeout=3)
        connection.request("POST", f"/api/pose-estimation/reproject-{arm}-shoulder",
                           body=json.dumps(body or {}), headers={"Content-Type": "application/json"})
        response = connection.getresponse()
        data = json.loads(response.read())
        connection.close()
        return response.status, data

    def test_reprojection_route_is_read_only_and_works_while_locked(self) -> None:
        status, result = self.request()
        self.assertEqual(status, 200)
        self.assertEqual(result["data"]["shoulder_grasp"]["frame"], "right_arm_sdk_world")
        self.assertEqual(
            result["data"]["shoulder_grasp"]["pregrasp_pose_right_shoulder_mm_deg"][0],
            490.0,
        )
        self.assertFalse(self.service.status()["armed"])

    def test_missing_saved_pose_returns_validation_error(self) -> None:
        self.estimator.available = False
        status, result = self.request()
        self.assertEqual(status, 400)
        self.assertIn("尚无已保存", result["error"])

    def test_left_conversion_reads_only_while_locked(self):
        before = self.service.robot.read_state()
        status, result = self.request('left')
        self.assertEqual(status, 200)
        self.assertEqual(result['data']['sku_typ'], 'box')
        self.assertEqual(result['data']['shoulder_grasp']['frame'], 'left_arm_sdk_world')
        self.assertEqual(self.service.robot.read_state(), before)
        self.assertFalse(self.service.armed)

    def test_tube_conversion_uses_tube_saved_result_and_remains_read_only(self):
        def project(before,after):
            return dict(self.estimator.reproject_latest_to_right_shoulder(before,after),sku_typ='tube')
        self.estimator.reproject_latest_tube_to_right_shoulder=project
        before=self.service.robot.read_state()
        status,result=self.request(body={'sku_typ':'tube'})
        self.assertEqual(status,200)
        self.assertEqual(result['data']['sku_typ'],'tube')
        self.assertEqual(self.service.robot.read_state(),before)
        self.assertFalse(self.service.armed)


if __name__ == "__main__":
    unittest.main()
