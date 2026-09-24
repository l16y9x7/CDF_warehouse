from __future__ import annotations

import copy
import json
import tempfile
import threading
import time
import unittest
from pathlib import Path
from unittest.mock import patch

from rokae_web.backends import BackendError, JOINT_COUNTS, MockChassisBackend, MockRobotBackend
from rokae_web.config import DEFAULT_CONFIG
from rokae_web.memory_points import MemoryPointStore
from rokae_web.service import ControlService, ValidationError


class SequencedRobot(MockRobotBackend):
    def __init__(self):
        super().__init__()
        self.calls = []
        self.hold_arms = False
        self.fail_arms = False
        self.arm_started = threading.Event()
        self.arm_plan = None

    def start_memory_arms(self, plan, cancel):
        self.calls.append("arms")
        self.arm_started.set()
        if self.fail_arms:
            raise BackendError("right start failed after left accepted")
        self.arm_plan = plan
        if self.hold_arms:
            self._state["operation_state"].update(left_arm="moving", right_arm="moving")
        else:
            super().start_memory_arms(plan, cancel)

    def arrive(self, side):
        with self._lock:
            for field in ("joints_deg", "poses"):
                self._state[field][side] = list(self.arm_plan["target"][field][side])
            self._state["operation_state"][side] = "mock-idle"

    def start_memory_head_trunk(self, plan, cancel):
        self.calls.append("head_trunk")
        super().start_memory_head_trunk(plan, cancel)

    def stop_memory_motion(self):
        self.calls.append("stop")
        for name in self._state["operation_state"]:
            self._state["operation_state"][name] = "mock-idle"
        return []


class MemoryPointsTests(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.config = copy.deepcopy(DEFAULT_CONFIG)
        self.config["memory_points"] = {"file": str(Path(self.tmp.name) / "points.json")}
        self.robot = SequencedRobot()
        self.service = ControlService(self.config, self.robot, MockChassisBackend(), False)
        self.service.memory.poll_seconds = 0.005
        self.service.memory.timeout_seconds = 0.5

    def tearDown(self):
        self.service.close()
        self.tmp.cleanup()

    def point(self):
        for name, count in JOINT_COUNTS.items():
            self.robot.move_joints(name, [1.0] * count, 5)
        result = self.service.memory.save({"name": "收纳 姿态"})["point"]
        for name, count in JOINT_COUNTS.items():
            self.robot.move_joints(name, [0.0] * count, 5)
        return result

    def run_point(self, point):
        self.service.arm({})
        return self.service.memory.execute({"id": point["id"], "revision": point["revision"]})

    def finish(self):
        self.service.memory.thread.join(timeout=2)
        self.assertFalse(self.service.memory.thread.is_alive())
        return self.service.memory.status()

    def test_create_persists_all_twenty_joints_poses_and_frames(self):
        point = self.point()
        saved = MemoryPointStore(self.config["memory_points"]["file"]).get(point["id"])
        self.assertEqual(sum(map(len, saved["state"]["joints_deg"].values())), 20)
        self.assertEqual(set(saved["state"]["poses"]), set(JOINT_COUNTS))
        self.assertEqual(saved["state"]["joints_deg"]["left_arm"], [1.0] * 7)
        self.assertEqual(saved["state"]["pose_frames"]["head"], "chassis_link")

    def test_overwrite_reads_fresh_hardware_and_retains_name_and_id(self):
        point = self.point()
        self.robot.move_joints("head", [12.0, 4.0], 5)
        changed = self.service.memory.save({"id": point["id"], "revision": 1,
                                           "state": point["state"]}, overwrite=True)["point"]
        self.assertEqual(changed["id"], point["id"])
        self.assertEqual(changed["name"], point["name"])
        self.assertEqual(changed["revision"], 2)
        self.assertEqual(changed["state"]["joints_deg"]["head"], [12.0, 4.0])
        with self.assertRaisesRegex(BackendError, "已被更新"):
            self.service.memory.save({"id": point["id"], "revision": 1}, overwrite=True)

    def test_duplicate_empty_and_nonfinite_data_rejected(self):
        point = self.point()
        with self.assertRaisesRegex(BackendError, "已存在"):
            self.service.memory.save({"name": point["name"]})
        with self.assertRaisesRegex(BackendError, "名称"):
            self.service.memory.save({"name": "  "})
        self.robot._state["joints_deg"]["head"][0] = float("nan")
        with self.assertRaisesRegex(BackendError, "有限数值"):
            self.service.memory.save({"name": "坏数据"})

    def test_delete_selected_persists_and_never_reads_robot(self):
        selected = self.point()
        keep = self.service.memory.save({"name": "保留点"})["point"]
        with patch.object(self.robot, "read_state", side_effect=AssertionError("unexpected read")), \
                patch.object(self.robot, "read_memory_state", side_effect=AssertionError("unexpected capture")), \
                patch.object(self.service, "audit_event") as audit:
            result = self.service.memory.delete({"id": selected["id"], "revision": selected["revision"]})
        self.assertFalse(self.service.armed)
        self.assertEqual([p["id"] for p in result["points"]], [keep["id"]])
        self.assertEqual(result["point"], selected)
        audit.assert_called_once_with("memory_point_deleted", point=selected)
        reopened = MemoryPointStore(self.config["memory_points"]["file"])
        self.assertEqual(reopened.list(), [keep])
        reopened.delete(keep["id"], keep["revision"])
        self.assertEqual(MemoryPointStore(reopened.path).list(), [])

    def test_delete_stale_missing_or_invalid_selection_preserves_file(self):
        point = self.point()
        self.service.memory.save({"id": point["id"], "revision": 1}, overwrite=True)
        path = self.service.memory.store.path
        original = path.read_bytes()
        for payload in ({}, {"id": point["id"]}, {"id": "missing", "revision": 1},
                        {"id": point["id"], "revision": 1},
                        {"id": point["id"], "revision": True}):
            with self.subTest(payload=payload), self.assertRaises(BackendError):
                self.service.memory.delete(payload)
            self.assertEqual(path.read_bytes(), original)

    def test_failed_delete_write_keeps_point_and_does_not_log_success(self):
        point = self.point()
        original = self.service.memory.store.path.read_bytes()
        with patch("rokae_web.memory_points.os.replace", side_effect=OSError("disk full")), \
                patch.object(self.service, "audit_event") as audit:
            with self.assertRaises(OSError):
                self.service.memory.delete({"id": point["id"], "revision": 1})
        self.assertEqual(self.service.memory.store.path.read_bytes(), original)
        self.assertEqual(list(Path(self.tmp.name).glob("*.tmp")), [])
        audit.assert_not_called()

    def test_corrupt_file_is_not_silently_replaced(self):
        path = Path(self.config["memory_points"]["file"])
        path.write_text("broken-json", encoding="utf-8")
        with self.assertRaisesRegex(BackendError, "原文件已保留"):
            self.service.memory.save({"name": "新点"})
        self.assertEqual(path.read_text(), "broken-json")

    def test_failed_atomic_write_keeps_previous_point(self):
        point = self.point()
        with patch("rokae_web.memory_points.os.replace", side_effect=OSError("disk full")):
            with self.assertRaises(OSError):
                self.service.memory.save({"id": point["id"]}, overwrite=True)
        self.assertEqual(self.service.memory.store.get(point["id"])["revision"], 1)

    def test_requires_unlock_and_stopped_controllers_and_matching_mode(self):
        point = self.point()
        with self.assertRaises(ValidationError):
            self.service.memory.execute({"id": point["id"]})
        self.service.arm({})
        for state in ("moving", "unknown", "drag"):
            self.robot._state["operation_state"]["left_arm"] = state
            with self.assertRaises(BackendError):
                self.service.memory.execute({"id": point["id"]})
        self.robot._state["operation_state"]["left_arm"] = "mock-idle"
        self.service.memory.mode = "hardware"
        with self.assertRaisesRegex(BackendError, "不能混用"):
            self.service.memory.execute({"id": point["id"]})
        self.assertEqual(self.robot.calls, [])

    def test_sequence_finishes_only_after_both_arms_then_head_trunk(self):
        point = self.point()
        self.robot.hold_arms = True
        self.run_point(point)
        self.assertTrue(self.robot.arm_started.wait(1))
        self.robot.arrive("left_arm")
        time.sleep(0.03)
        self.assertEqual(self.robot.calls, ["arms"])
        self.robot.arrive("right_arm")
        self.assertEqual(self.finish()["phase"], "completed")
        self.assertEqual(self.robot.calls, ["arms", "head_trunk"])
        self.assertEqual(self.robot.read_state()["joints_deg"], point["state"]["joints_deg"])

    def test_memory_speed_follows_saved_default_on_next_execution(self):
        point = self.point()
        selected_speeds = []
        original = self.robot.prepare_memory_motion

        def capture_speed(target, speeds, cancel):
            selected_speeds.append(dict(speeds))
            return original(target, speeds, cancel)

        self.robot.prepare_memory_motion = capture_speed
        self.service.set_speed({"speed_mm_s": 80.0, "rotation_deg_s": 8.0})
        self.assertEqual(self.service.memory.status()["speed"], {"linear_mm_s": 80.0, "rotation_deg_s": 8.0})
        self.robot.hold_arms = True
        self.run_point(point)
        self.assertTrue(self.robot.arm_started.wait(1))
        self.service.set_speed({"speed_mm_s": 120.0, "rotation_deg_s": 12.0})
        self.assertEqual(self.service.memory.status()["speed"], {"linear_mm_s": 80.0, "rotation_deg_s": 8.0})
        self.service.memory.stop()
        self.assertEqual(self.finish()["phase"], "cancelled")

        self.robot.hold_arms = False
        self.run_point(point)
        self.assertEqual(self.finish()["phase"], "completed")
        self.assertEqual(selected_speeds, [
            {"linear_mm_s": 80.0, "rotation_deg_s": 8.0}, {"linear_mm_s": 120.0, "rotation_deg_s": 12.0},
        ])

    def test_failed_arm_start_stops_without_body_motion_or_retry(self):
        point = self.point()
        self.robot.fail_arms = True
        self.run_point(point)
        self.assertEqual(self.finish()["phase"], "failed")
        self.assertEqual(self.robot.calls, ["arms", "stop"])

    def test_manual_commands_duplicate_run_and_overwrite_blocked_during_motion(self):
        point = self.point()
        self.robot.hold_arms = True
        self.run_point(point)
        self.assertTrue(self.robot.arm_started.wait(1))
        for action in (
            lambda: self.service.move_joints("head", {"values": [0, 0]}),
            lambda: self.service.set_drag("left_arm", {"enabled": False}),
            lambda: self.service.memory.execute({"id": point["id"]}),
            lambda: self.service.memory.save({"id": point["id"]}, overwrite=True),
            lambda: self.service.memory.delete({"id": point["id"], "revision": point["revision"]}),
        ):
            with self.assertRaisesRegex(BackendError, "正在执行"):
                action()
        self.service.memory.stop()
        self.assertEqual(self.finish()["phase"], "cancelled")
        self.assertEqual(self.robot.calls, ["arms", "stop"])

    def test_disarm_cancels_motion_before_next_stage(self):
        self.robot.hold_arms = True
        self.run_point(self.point())
        self.assertTrue(self.robot.arm_started.wait(1))
        self.service.disarm()
        self.assertEqual(self.finish()["phase"], "cancelled")
        self.assertFalse(self.service.armed)
        self.assertNotIn("head_trunk", self.robot.calls)

    def test_unknown_controller_state_and_timeout_stop_sequence(self):
        self.robot.hold_arms = True
        self.run_point(self.point())
        self.assertTrue(self.robot.arm_started.wait(1))
        self.robot._state["operation_state"]["right_arm"] = "unknown"
        self.assertEqual(self.finish()["phase"], "failed")
        self.assertNotIn("head_trunk", self.robot.calls)
        self.robot.calls.clear()
        self.run_point(self.service.memory.store.list()[0])
        self.assertEqual(self.finish()["phase"], "failed")
        self.assertIn("超时", self.service.memory.status()["message"])
        self.assertNotIn("head_trunk", self.robot.calls)

    def test_unexpected_trunk_movement_aborts_arm_stage(self):
        self.robot.hold_arms = True
        self.run_point(self.point())
        self.assertTrue(self.robot.arm_started.wait(1))
        self.robot.move_joints("trunk", [10.0] * 4, 5)
        self.assertEqual(self.finish()["phase"], "failed")
        self.assertNotIn("head_trunk", self.robot.calls)

    def test_cancellation_during_preflight_sends_no_motion(self):
        entered = threading.Event()
        release = threading.Event()
        original = self.robot.prepare_memory_motion
        def prepare(*args):
            entered.set()
            release.wait(1)
            return original(*args)
        self.robot.prepare_memory_motion = prepare
        self.run_point(self.point())
        self.assertTrue(entered.wait(1))
        self.service.memory.stop()
        release.set()
        self.assertEqual(self.finish()["phase"], "cancelled")
        self.assertEqual(self.robot.calls, [])

    def test_pose_rotation_wrap_is_not_a_false_arrival_failure(self):
        from rokae_web.memory_motion import poses_match
        self.assertTrue(poses_match([0, 0, 0, 0, 0, 180], [0, 0, 0, 0, 0, -180]))
        self.assertFalse(poses_match([0, 0, 0, 0, 0, 0], [0, 0, 0, 0, 0, 10]))

    def test_stop_error_locks_controls_and_preserves_failure_message(self):
        self.robot.fail_arms = True
        self.robot.stop_memory_motion = lambda: ["left controller stop rejected"]
        self.run_point(self.point())
        status = self.finish()
        self.assertEqual(status["phase"], "failed")
        self.assertIn("stop rejected", status["message"])
        self.assertFalse(self.service.armed)


if __name__ == "__main__":
    unittest.main()
