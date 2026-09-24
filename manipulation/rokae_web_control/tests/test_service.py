from __future__ import annotations

import copy
import time
import unittest
from unittest.mock import Mock
from types import SimpleNamespace

from rokae_web.audit import MemoryAuditLogger
from rokae_web.backends import BackendError, MockChassisBackend, MockRobotBackend
from rokae_web.config import DEFAULT_CONFIG
from rokae_web.pose_frames import POSE_FRAMES
from rokae_web.service import ControlService, ValidationError


class FakeCameraManager:
    def __init__(self) -> None:
        self.enabled = {"head": False, "left_wrist": False, "right_wrist": False}
        self.closed = False
        self.saved_state = None
        self.saved_camera_id = None
        self.sequence = 2

    def status(self):
        return {
            camera_id: {
                "available": True,
                "enabled": enabled,
                "starting": False,
                "camera_id": camera_id,
            }
            for camera_id, enabled in self.enabled.items()
        }

    def set_enabled(self, camera_id, enabled):
        self.enabled[camera_id] = enabled
        return self.status()[camera_id]

    def frame_jpeg(self, camera_id, kind, after_sequence, timeout):
        del kind, timeout
        self.sequence = max(self.sequence, after_sequence + 1)
        return b"jpeg", self.sequence, self.enabled[camera_id]

    def snapshot(self, camera_id):
        if not self.enabled[camera_id]:
            raise RuntimeError("camera disabled")
        return SimpleNamespace(sequence=self.sequence)

    def record(self, camera_id, robot_state):
        self.saved_camera_id = camera_id
        self.saved_state = robot_state
        return {
            "directory": "/tmp/data/20260828/210000000",
            "relative_directory": "data/20260828/210000000",
            "active_cameras": [camera_id],
        }

    def close(self):
        self.closed = True


class FakePoseEstimator:
    fresh_frame_timeout = 0.1

    def __init__(self) -> None:
        self.calls = []
        self.reprojects = []

    def health(self):
        return {"available": True}

    def estimate(self, snapshot, state_before, state_after):
        self.calls.append((snapshot, state_before, state_after))
        return {
            "status": "success",
            "usable": True,
            "result_id": "20260916/pose_120000000_abcdef",
            "relative_directory": "data/20260916/pose_120000000_abcdef",
            "elapsed_seconds": 0.1,
        }

    def result_image(self, day, leaf):
        return f"{day}/{leaf}".encode()

    def reproject_latest_to_right_shoulder(self, before, after):
        self.reprojects.append((before, after))
        return {
            "source_result_id": "20260916/pose_120000000_abcdef",
            "current_trunk_joints_deg": after["joints_deg"]["trunk"],
            "shoulder_grasp": {"grasp_pose_right_shoulder_mm_deg": [1.0] * 6},
        }


class ControlServiceTests(unittest.TestCase):
    def setUp(self) -> None:
        self.config = copy.deepcopy(DEFAULT_CONFIG)
        self.robot = MockRobotBackend()
        self.chassis = MockChassisBackend()
        self.audit = MemoryAuditLogger()
        self.service = ControlService(self.config, self.robot, self.chassis, False, self.audit)

    def arm(self) -> None:
        self.service.arm({})

    def test_speed_pair_defaults_validation_and_legacy_translation_save(self):
        self.assertEqual(self.service.status()["rotation_deg_s"], 6)
        result = self.service.set_speed({"speed_mm_s": 80, "rotation_deg_s": 9})
        self.assertEqual(result, {"speed_mm_s": 80, "rotation_deg_s": 9})
        for invalid in (0, -1, 201, float("nan"), float("inf"), None, True):
            with self.subTest(rotation=invalid), self.assertRaises(ValidationError):
                self.service.set_speed({"speed_mm_s": 90, "rotation_deg_s": invalid})
            self.assertEqual((self.service.speed_mm_s, self.service.rotation_deg_s), (80, 9))
        with self.assertRaises(ValidationError):
            self.service.set_speed({"speed_mm_s": 0, "rotation_deg_s": 12})
        self.assertEqual(self.service.rotation_deg_s, 9)
        self.assertEqual(self.service.set_speed({"speed_mm_s": 100}),
                         {"speed_mm_s": 100, "rotation_deg_s": 9})
        changed = [r for r in self.audit.records if r["event"] == "speed_changed"]
        self.assertEqual(changed[-1]["rotation_deg_s"], 9)

    def test_mock_default_is_locked(self) -> None:
        status = self.service.status()
        self.assertEqual(status["mode"], "mock")
        self.assertFalse(status["armed"])
        self.assertFalse(status["gripper"]["unlocked"])
        self.assertNotIn("armed_remaining_seconds", status)

    def test_gripper_follows_global_control_and_still_requires_initialization(self) -> None:
        with self.assertRaisesRegex(ValidationError, "未解锁"):
            self.service.set_gripper_unlocked({"unlocked": True})
        self.arm()
        self.assertTrue(self.service.status()['gripper']['unlocked'])
        self.assertEqual(self.service.gripper_status()["activation_state"], 0)
        with self.assertRaisesRegex(BackendError, "初始化"):
            self.service.move_gripper({"position": 30})
        self.service.activate_gripper()
        self.assertEqual(self.service.move_gripper({"position": 30})["measured_position"], 30)
        self.assertEqual(self.service.move_gripper({"position": 0})["measured_position"], 0)
        self.service.disarm()
        self.assertFalse(self.service.status()["gripper"]["unlocked"])
        with self.assertRaisesRegex(ValidationError, "未解锁"):
            self.service.move_gripper({"position": 30})

    def test_global_unlock_does_not_activate_or_move_gripper(self) -> None:
        self.service.robot.gripper_activate = Mock(side_effect=AssertionError('unexpected activation'))
        self.service.robot.gripper_move = Mock(side_effect=AssertionError('unexpected gripper motion'))
        self.service.arm({})
        self.assertTrue(self.service.gripper_unlocked)
        self.service.robot.gripper_activate.assert_not_called()
        self.service.robot.gripper_move.assert_not_called()

    def test_gripper_rejects_invalid_positions(self) -> None:
        self.arm()
        self.service.set_gripper_unlocked({"unlocked": True})
        self.service.activate_gripper()
        for invalid in (-1, 256, True, 1.5, "30"):
            with self.subTest(position=invalid):
                with self.assertRaisesRegex(ValidationError, "0–255"):
                    self.service.move_gripper({"position": invalid})

    def test_unlock_requires_no_confirmation_and_has_no_timeout(self) -> None:
        status = self.service.arm({})

        self.assertTrue(status["armed"])

    def test_status_exposes_all_visible_limits(self) -> None:
        status = self.service.status()
        limits = status["limits"]
        self.assertEqual(len(limits["joint_limits_deg"]["left_arm"]), 7)
        self.assertEqual(len(limits["joint_limits_deg"]["right_arm"]), 7)
        self.assertEqual(len(limits["joint_limits_deg"]["trunk"]), 4)
        self.assertEqual(len(limits["joint_limits_deg"]["head"]), 2)
        self.assertEqual(limits["joint_limits_deg"]["left_arm"][3], [-60.0, 145.0])
        self.assertEqual(limits["joint_limits_deg"]["trunk"][1], [-175.0, 49.0])
        self.assertEqual(limits["joint_limit_source"], "配置中的控制器软限位备用快照")
        self.assertEqual(status["chassis"]["limits"]["max_linear_m_s"], 0.5)

    def test_joint_motion_requires_unlock(self) -> None:
        with self.assertRaisesRegex(ValidationError, "未解锁"):
            self.service.move_joints(
                "head", {"values": [1, 1]}
            )

    def test_arm_joint_step_guard_is_disabled(self) -> None:
        self.arm()
        target = [90, -80, 70, -60, 50, -40, 30]

        self.service.move_joints("left_arm", {"values": target})

        self.assertEqual(self.service.readback()["joints_deg"]["left_arm"], target)

    def test_readback_includes_arm_elbow_angles(self) -> None:
        state = self.service.readback()

        self.assertEqual(state["arm_elbow_deg"], {"left_arm": 0.0, "right_arm": 0.0})

    def test_joint_motion_updates_mock(self) -> None:
        self.arm()
        target = [1, 2, 3, 4, 5, 6, 7]
        self.service.move_joints(
            "right_arm", {"values": target}
        )
        self.assertEqual(self.service.readback()["joints_deg"]["right_arm"], target)
        events = [record["event"] for record in self.audit.records]
        self.assertIn("joint_motion_requested", events)
        self.assertIn("joint_motion_accepted", events)

    def test_trunk_ten_degree_guard_is_disabled(self) -> None:
        self.arm()
        target = [30, -20, 15, -15]

        self.service.move_joints("trunk", {"values": target})

        self.assertEqual(self.service.readback()["joints_deg"]["trunk"], target)

    def test_hardware_motion_starts_full_state_sampling(self) -> None:
        config = copy.deepcopy(DEFAULT_CONFIG)
        config["logging"]["state_sample_interval_seconds"] = 0.05
        config["logging"]["state_sample_max_seconds"] = 1.0
        audit = MemoryAuditLogger()
        service = ControlService(
            config,
            MockRobotBackend(),
            MockChassisBackend(),
            True,
            audit,
        )
        try:
            service.arm({})
            service.move_joints("right_arm", {"values": [1, 2, 3, 4, 5, 6, 7]})
            deadline = time.monotonic() + 1.0
            while time.monotonic() < deadline:
                if any(record["event"] == "robot_state_sample" for record in audit.records):
                    break
                time.sleep(0.01)
            self.assertTrue(
                any(record["event"] == "robot_state_sample" for record in audit.records)
            )
        finally:
            service.close()

    def test_pose_step_guard_is_disabled(self) -> None:
        self.arm()
        target = [500, -250, 125, 180, -90, 45]

        self.service.move_pose("trunk", {"values": target})

        self.assertEqual(self.service.readback()["poses"]["trunk"], target)

    def test_arm_pose_accepts_and_updates_elbow_angle(self) -> None:
        self.arm()
        target = [300, -200, -250, 80, -70, 100]

        result = self.service.move_pose(
            "right_arm",
            {"values": target, "elbow_deg": 85.0, "frame": POSE_FRAMES["right_arm"]},
        )

        state = self.service.readback()
        self.assertEqual(result["elbow_deg"], 85.0)
        self.assertEqual(state["poses"]["right_arm"], target)
        self.assertEqual(state["arm_elbow_deg"]["right_arm"], 85.0)
        self.assertEqual(result["frame"], POSE_FRAMES["right_arm"])
        self.assertEqual(state["pose_frames"], POSE_FRAMES)

    def test_arm_pose_rejects_old_or_wrong_coordinate_frame(self) -> None:
        self.arm()
        for frame in (None, "chest_common", "flangeInBase", POSE_FRAMES["left_arm"]):
            with self.subTest(frame=frame):
                with self.assertRaisesRegex(ValidationError, "世界坐标系"):
                    self.service.move_pose(
                        "right_arm", {"values": [0] * 6, "elbow_deg": 0, "frame": frame}
                    )
        self.assertEqual(self.service.readback()["poses"]["right_arm"], [0.0] * 6)

    def test_status_advertises_the_pose_coordinate_frames(self) -> None:
        self.assertEqual(self.service.status()["pose_frames"], POSE_FRAMES)

    def test_arm_pose_requires_numeric_elbow_angle(self) -> None:
        self.arm()

        with self.assertRaisesRegex(ValidationError, "臂角"):
            self.service.move_pose(
                "left_arm",
                {"values": [0, 0, 0, 0, 0, 0], "elbow_deg": ""},
            )

    def test_drag_blocks_motion(self) -> None:
        self.arm()
        self.service.set_drag(
            "left_arm", {"enabled": True}
        )
        with self.assertRaisesRegex(ValidationError, "拖拽状态"):
            self.service.move_joints(
                "head", {"values": [1, 1]}
            )
        self.service.set_drag("left_arm", {"enabled": False})

    def test_chassis_limits_and_stop(self) -> None:
        self.arm()
        self.service.set_chassis_enabled(
            {"enabled": True}
        )
        self.service.chassis_command(
            {"linear_x": 0.1, "linear_y": 0.0, "angular_z": 0.2}
        )
        self.assertEqual(self.chassis.last_velocity, [0.1, 0.0, 0.2])
        self.service.chassis_stop()
        self.assertEqual(self.chassis.last_velocity, [0.0, 0.0, 0.0])

    def test_disarm_stops_chassis(self) -> None:
        self.arm()
        self.service.set_chassis_enabled(
            {"enabled": True}
        )
        self.service.chassis_command(
            {"linear_x": 0.1, "linear_y": 0.0, "angular_z": 0.0}
        )
        self.service.disarm()
        self.assertEqual(self.chassis.last_velocity, [0.0, 0.0, 0.0])
        self.assertFalse(self.service.status()["armed"])

    def test_release_chassis_emergency_stop_requires_unlock_and_stops_remote_control(self) -> None:
        with self.assertRaisesRegex(ValidationError, "未解锁"):
            self.service.release_chassis_emergency_stop()

        self.arm()
        self.service.set_chassis_enabled({"enabled": True})
        self.service.chassis_command(
            {"linear_x": 0.1, "linear_y": 0.0, "angular_z": 0.0}
        )
        result = self.service.release_chassis_emergency_stop()

        self.assertTrue(result["released"])
        self.assertFalse(self.chassis.enabled)
        self.assertEqual(self.chassis.last_velocity, [0.0, 0.0, 0.0])
        self.assertTrue(
            any(
                record["event"] == "chassis_emergency_stop_released"
                for record in self.audit.records
            )
        )

    def test_camera_can_start_record_full_state_and_stop_without_unlock(self) -> None:
        camera = FakeCameraManager()
        service = ControlService(
            self.config,
            self.robot,
            self.chassis,
            False,
            self.audit,
            camera_backend=camera,
        )
        try:
            self.assertFalse(service.status()["cameras"]["right_wrist"]["enabled"])
            self.assertTrue(
                service.set_camera_enabled(
                    {"camera": "right_wrist", "enabled": True}
                )["enabled"]
            )
            self.assertTrue(
                service.set_camera_enabled(
                    {"camera": "head", "enabled": True}
                )["enabled"]
            )
            result = service.record_camera_snapshot("right_wrist")
            self.assertEqual(result["relative_directory"], "data/20260828/210000000")
            self.assertEqual(result["active_cameras"], ["right_wrist"])
            self.assertEqual(camera.saved_camera_id, "right_wrist")
            self.assertIn("joints_deg", camera.saved_state)
            self.assertIn("poses", camera.saved_state)
            self.assertIn("arm_elbow_deg", camera.saved_state)
            self.assertFalse(
                service.set_camera_enabled(
                    {"camera": "right_wrist", "enabled": False}
                )["enabled"]
            )
        finally:
            service.close()
        self.assertTrue(camera.closed)

    def test_pose_estimation_uses_fresh_head_frame_without_unlock(self) -> None:
        camera = FakeCameraManager()
        estimator = FakePoseEstimator()
        service = ControlService(
            self.config,
            self.robot,
            self.chassis,
            False,
            self.audit,
            camera_backend=camera,
            pose_estimator=estimator,
        )
        try:
            result = service.estimate_grasp_object_pose()
            self.assertTrue(result["usable"])
            self.assertTrue(camera.enabled["head"])
            self.assertEqual(len(estimator.calls), 1)
            snapshot, before, after = estimator.calls[0]
            self.assertGreater(snapshot.sequence, 2)
            self.assertIn("joints_deg", before)
            self.assertIn("joints_deg", after)
            self.assertFalse(service.status()["armed"])
            self.assertEqual(
                service.pose_estimation_image("20260916", "pose_120000000_abcdef"),
                b"20260916/pose_120000000_abcdef",
            )
        finally:
            service.close()

    def test_reproject_latest_pose_only_reads_state_without_unlock(self) -> None:
        estimator = FakePoseEstimator()
        service = ControlService(
            self.config, self.robot, self.chassis, False, self.audit,
            camera_backend=FakeCameraManager(), pose_estimator=estimator,
        )
        try:
            result = service.reproject_latest_grasp_to_right_shoulder()
            self.assertEqual(result["shoulder_grasp"]["grasp_pose_right_shoulder_mm_deg"],
                             [1.0] * 6)
            self.assertEqual(len(estimator.reprojects), 1)
            self.assertFalse(service.status()["armed"])
        finally:
            service.close()


if __name__ == "__main__":
    unittest.main()
