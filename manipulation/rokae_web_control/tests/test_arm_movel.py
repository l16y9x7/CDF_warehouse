"""Offline SDK doubles only: importing this test never connects to hardware."""
import copy
import json
import math
from pathlib import Path
import sys
import tempfile
import threading
import time
from types import SimpleNamespace
import unittest
from unittest.mock import patch

import numpy as np

sys.path.append(str(Path(__file__).resolve().parents[2]))
from arm_motion_control.model import RobotModel
from rokae_web.arm_movel import HardwareMoveL, pose_values, transform
from rokae_web.backends import BackendError, MockRobotBackend, MockChassisBackend
from rokae_web.config import DEFAULT_CONFIG
from rokae_web.pose_frames import POSE_FRAMES, ref_pose_to_world, world_pose_to_ref
from rokae_web.service import ControlService
import test_memory_motion as fixtures


class ServiceTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        config = copy.deepcopy(DEFAULT_CONFIG)
        config["memory_points"] = {"file": str(Path(self.temp.name) / "memory.json")}
        self.path = Path(self.temp.name) / "guard.json"
        config["torso_guard_file"] = str(self.path)
        self.write_config(20)
        self.service = ControlService(config, MockRobotBackend(), MockChassisBackend(), False)
        self.payload = {"values": [10, 0, 0, 0, 0, 0], "frame": POSE_FRAMES["right_arm"], "elbow_deg": 0}

    def tearDown(self):
        self.service.close()
        self.temp.cleanup()

    def write_config(self, offset):
        self.path.write_text(json.dumps({"plane_offset_mm": offset, "elbow_radius_mm": 65, "margin_mm": 10}))

    def execute(self, **changes):
        result = self.service.arm_movel.execute("right_arm", {**self.payload, **changes}, True)
        self.service.arm_movel.thread.join(2)
        return result, self.service.arm_movel.status()

    def test_lock_gate_config_null_and_invalid_fields(self):
        with self.assertRaisesRegex(ValueError, "解锁"):
            self.execute()
        self.service.arm({})
        for invalid in (None, -1, float("nan"), True, ""):
            self.write_config(invalid)
            with self.subTest(invalid=invalid), self.assertRaises(BackendError):
                self.execute()
        self.write_config(20)
        for changes in ({"elbow_deg": 181}, {"elbow_deg": False}, {"frame": "Chest_link"}, {"values": [0]*5}):
            with self.subTest(changes=changes), self.assertRaises(BackendError):
                self.execute(**changes)

    def test_shared_config_reloads_and_uses_web_speed_and_zero_seed(self):
        self.service.arm({})
        self.service.set_speed({"speed_mm_s": 87})
        self.service.robot._state["arm_elbow_deg"]["right_arm"] = 33
        accepted, finished = self.execute()
        self.assertEqual(accepted["speed_mm_s"], 87)
        self.assertEqual(accepted["plane"], {"offset_mm": 20, "elbow_radius_mm": 65, "margin_mm": 10})
        self.assertEqual(finished["selected_arm_angle_deg"], 0)
        self.write_config(45)
        accepted, finished = self.execute(elbow_deg=None)
        self.assertEqual(accepted["plane"]["offset_mm"], 45)
        self.assertIn("MOCK", finished["message"])
        payload = {**self.payload, "frame": POSE_FRAMES["left_arm"]}
        accepted = self.service.arm_movel.execute("left_arm", payload, True)
        self.service.arm_movel.thread.join(2)
        self.assertEqual(accepted["plane"]["offset_mm"], 45)

    def test_blank_seed_uses_measured_angle(self):
        self.service.arm({})
        self.service.robot._state["arm_elbow_deg"]["right_arm"] = 33
        _, result = self.execute(elbow_deg=None)
        self.assertEqual(result["selected_arm_angle_deg"], 33)

    def test_mutual_exclusion_and_disarm_cancels_before_dispatch(self):
        entered = threading.Event()
        class WaitingExecutor:
            def __init__(self, backend, module, config, cancel):
                self.cancel = cancel
            def plan(self, *args):
                entered.set()
                self.cancel.wait(2)
                raise BackendError("cancelled")
        self.service.hardware_enabled = True  # Backend remains Mock; executor is a double.
        self.service.arm({})
        with patch("rokae_web.arm_movel.HardwareMoveL", WaitingExecutor):
            self.service.arm_movel.execute("right_arm", self.payload, True)
            self.assertTrue(entered.wait(1))
            for action in (
                lambda: self.service.move_joints("head", {"values": [1, 1]}),
                lambda: self.service.move_pose("right_arm", self.payload),
                lambda: self.service.memory.execute({"id": "unused"}),
                lambda: self.service.memory.save({"name": "blocked"}),
                lambda: self.service.set_drag("left_arm", {"enabled": False}),
                lambda: self.service.arm_movel.execute("left_arm", {**self.payload, "frame": POSE_FRAMES["left_arm"]}, False),
            ):
                with self.assertRaises(BackendError):
                    action()
            self.service.disarm()
            self.service.arm_movel.thread.join(2)
        self.assertFalse(self.service.armed)
        self.assertEqual(self.service.arm_movel.status()["phase"], "cancelled")

    def test_failed_start_stops_and_unconfirmed_stop_latches_motion_lock(self):
        stopped = []
        class FailingExecutor:
            steps = [object()]
            def __init__(self, *args):
                pass
            def plan(self, *args):
                return {"angle": 0}
            def check_fresh(self):
                pass
            def start_step(self, *args):
                raise BackendError("start communication error")
            def stop_and_verify(self):
                stopped.append(True)
                return ["stop unavailable"]
        self.service.hardware_enabled = True
        self.service.arm({})
        with patch("rokae_web.arm_movel.HardwareMoveL", FailingExecutor):
            self.execute()
        self.assertEqual(stopped, [True])
        self.assertFalse(self.service.armed)
        self.assertTrue(self.service.arm_movel.status()["stop_unconfirmed"])
        self.service.arm({})
        with self.assertRaisesRegex(BackendError, "停止未确认"):
            self.service.move_joints("head", {"values": [1, 1]})


class AnalyticModel:
    """Test-only seven-variable pose model, not a robot approximation."""
    elbow_override = None

    def __init__(self, path, side="right"):
        self.sign = 1 if side == "left" else -1

    def torso_world(self, trunk):
        return transform([trunk[0], 0, 0, 0, trunk[1], trunk[3]])

    def shoulder_world(self, trunk):
        return self.torso_world(trunk) @ transform([0, self.sign * 200, 0, 0, 0, 0])

    def arm_frames_world(self, trunk, q):
        shoulder = self.shoulder_world(trunk)
        elbow = shoulder[:3, 3].copy()
        if self.elbow_override is not None:
            override = type(self).elbow_override
            elbow[1] = override(q) if callable(override) else override
        return shoulder @ transform(q[:6]), elbow

    def arm_joints_within_limits(self, q):
        return True


class AdapterTests(unittest.TestCase):
    def setUp(self):
        self.fixture = fixtures.MemoryMotionTests()
        self.fixture.setUp()
        self.backend = self.fixture.backend
        self.config = copy.deepcopy(DEFAULT_CONFIG)
        self.cancel = threading.Event()
        self.q = {side: [0.0]*7 for side in ("left_arm", "right_arm")}
        self.events = self.fixture.events
        self.path_fail = None
        self.path_mismatch = False
        self.current_command = None
        self.model_patch = patch("rokae_web.arm_movel.arm_model", side_effect=AnalyticModel)
        self.model_patch.start()
        self.addCleanup(self.model_patch.stop)
        AnalyticModel.elbow_override = None
        for side in self.q:
            robot = self.backend._robot(side)
            tool = robot.toolset({})
            tool.ref.trans = [.1, -.04, .03]
            tool.ref.rpy = [.1, -.1, .2]
            robot.jointPos = lambda ec, side=side: np.radians(self.q[side]).tolist()
            def cart(ec=None, side=side, tool=tool):
                q = self.q[side]
                tcp = transform(q[:6]) @ transform([0, 0, 100, 0, 0, 0])
                values = self.backend._pose_to_sdk(pose_values(tcp))
                values = world_pose_to_ref(values, self.backend._world_from_ref(tool))
                return SimpleNamespace(trans=values[:3], rpy=values[3:], elbow=math.radians(q[6]), confData=[0]*8)
            robot.cartPosture = lambda coordinate, ec, cart=cart: cart()
            def solve(target, tool=tool):
                pose = ref_pose_to_world(target.values, self.backend._world_from_ref(tool))
                tcp = transform([v * 1000 for v in pose[:3]] + [math.degrees(v) for v in pose[3:]])
                flange = tcp @ np.linalg.inv(transform([0, 0, 100, 0, 0, 0]))
                return np.radians(pose_values(flange) + [math.degrees(target.elbow)]).tolist()
            robot.model = lambda solve=solve: SimpleNamespace(calcIk=lambda target, tool, ec: solve(target))
            def check_path(start, joints, goal, ec, side=side, solve=solve):
                self.events.append((side, "checkPath"))
                if self.path_fail:
                    ec["ec"] = self.path_fail
                result = solve(goal)
                if self.path_mismatch:
                    result[0] += .5
                return result
            robot.checkPath = check_path
            def append(commands, identifier, ec, side=side):
                self.events.append((side, "append"))
                self.current_command = commands[0]
            def start(ec, side=side, solve=solve):
                self.events.append((side, "start"))
                self.q[side] = np.degrees(solve(self.current_command.target)).tolist()
            robot.moveAppend, robot.moveStart = append, start
        self.backend.soft_limit_status = lambda: {"joint_limits_deg": {side: [[-1000, 1000]]*7 for side in self.q}}
        self.executor = HardwareMoveL(self.backend, "right_arm", self.config, self.cancel)
        self.plane = {"offset_mm": 20, "elbow_radius_mm": 65, "margin_mm": 10}
        self.events.clear()

    def test_protected_tcp_line_with_offset_tool_and_rotated_reference_and_seed(self):
        self.backend._robot("right_arm").model = lambda: self.fail("must not call calcIk")
        result = self.executor.plan([10, 0, 100, 0, 10, 0], 5, self.plane)
        self.assertEqual(result["angle"], 5)
        self.assertEqual(self.events, [("right_arm", "checkPath")])
        self.assertEqual(result["check_path_calls"], 1)
        self.assertEqual(len(self.executor.steps), 1)
        for step in self.executor.steps:
            self.assertAlmostEqual(step["pose"][1], 0, places=7)
            self.assertAlmostEqual(step["pose"][2], 100, places=7)
        self.executor.check_fresh()
        self.executor.start_step(0, 87, 9)
        self.executor.wait_step(0)
        self.assertIs(type(self.current_command), fixtures.MoveL)
        self.assertEqual(self.current_command.speed, 87)
        self.assertEqual(self.current_command.blend, 0)
        self.assertAlmostEqual(self.current_command.rotSpeed, math.radians(9))
        self.assertEqual(self.events.count(("right_arm", "append")), 1)
        self.assertEqual(self.events.count(("right_arm", "start")), 1)

    def test_left_arm_uses_same_plane_and_mirrored_model(self):
        executor = HardwareMoveL(self.backend, "left_arm", self.config, self.cancel)
        result = executor.plan([10, 0, 100, 0, 0, 0], None, self.plane)
        self.assertEqual(result["angle"], 0)
        self.assertAlmostEqual(result["clearance"], 105)

    def test_unsafe_endpoint_rejects_all_candidates_without_motion(self):
        AnalyticModel.elbow_override = -70
        with self.assertRaisesRegex(BackendError, "37 个臂角"):
            self.executor.plan([10, 0, 100, 0, 0, 0], None, self.plane)
        self.assertEqual(self.events, [("right_arm", "checkPath")] * 37)
        self.assertEqual(self.executor.steps, [])

    def test_native_unreachable_never_falls_back_to_movej(self):
        self.path_fail = -50102
        with self.assertRaises(BackendError):
            self.executor.plan([2, 0, 100, 0, 0, 0], None, self.plane)
        self.assertEqual(len(self.events), 37)
        self.assertFalse(any(event in ("prepare", "append", "start") for _, event in self.events))

    def test_scan_uses_native_endpoint_and_stops_at_first_safe_angle(self):
        AnalyticModel.elbow_override = lambda q: -100 if q[6] <= 15 else -70
        result = self.executor.plan([2, 0, 100, 0, 0, 0], 20, self.plane)
        self.assertEqual([a["angle"] for a in result["attempts"]], [20, 15])
        self.assertEqual(result["angle"], 15)
        self.assertEqual(result["check_path_calls"], 2)
        self.assertEqual(len(self.executor.steps), 1)
        self.assertAlmostEqual(self.executor.steps[0]["cart"].elbow, math.radians(15))

    def test_start_or_interior_plane_violation_is_not_sampled(self):
        # All intermediate positions and the start violate the plane; ONLY the
        # endpoint is accepted, matching the deliberately narrower requirement.
        AnalyticModel.elbow_override = lambda q: -70 if q[0] < 9.9 else -200
        result = self.executor.plan([10, 0, 100, 0, 0, 0], None, self.plane)
        self.assertEqual(result["check_path_calls"], 1)
        self.assertEqual(len(self.executor.steps), 1)

    def test_transport_error_aborts_after_one_native_call(self):
        self.path_fail = 10001
        with self.assertRaisesRegex(BackendError, "10001"):
            self.executor.plan([2, 0, 100, 0, 0, 0], None, self.plane)
        self.assertEqual(self.events, [("right_arm", "checkPath")])

    def test_stale_start_or_changed_tool_never_starts(self):
        self.executor.plan([2, 0, 100, 0, 0, 0], None, self.plane)
        self.q["right_arm"][0] = 1
        with self.assertRaisesRegex(BackendError, "位置改变"):
            self.executor.start_step(0, 87, 9)
        self.q["right_arm"][0] = 0
        self.backend._robot("right_arm").toolset({}).end.trans[0] = .2
        with self.assertRaisesRegex(BackendError, "工具"):
            self.executor.start_step(0, 87, 9)
        self.assertFalse(any(event == "start" for _, event in self.events))

    def test_runtime_does_not_reintroduce_plane_sampling_and_stop_error_attempts_reset(self):
        self.executor.plan([2, 0, 100, 0, 0, 0], None, self.plane)
        self.executor.start_step(0, 87, 9)
        AnalyticModel.elbow_override = -70
        self.executor.wait_step(0)
        self.backend._robot("right_arm").stop = lambda ec: ec.update(ec=10001)
        errors = self.executor.stop_and_verify()
        self.assertEqual(len(errors), 1)
        self.assertIn(("right_arm", "reset"), self.events)

    def test_cancel_before_dispatch_and_between_append_start(self):
        self.executor.plan([2, 0, 100, 0, 0, 0], None, None)
        self.cancel.set()
        with self.assertRaisesRegex(BackendError, "取消"):
            self.executor.start_step(0, 87, 9)
        self.cancel.clear()
        self.backend._robot("right_arm").moveAppend = lambda *args: self.cancel.set()
        with self.assertRaisesRegex(BackendError, "取消"):
            self.executor.start_step(0, 87, 9)
        self.assertFalse(any(event == "start" for _, event in self.events))

    @unittest.skipUnless(Path(DEFAULT_CONFIG["pose_estimation"]["urdf_file"]).exists(), "URDF is on rokae")
    def test_actual_left_and_right_urdf_under_torso_rotations(self):
        self.model_patch.stop()
        for side in ("left", "right"):
            model = RobotModel(DEFAULT_CONFIG["pose_estimation"]["urdf_file"], side)
            local = []
            for trunk in ([0, 0, 0, 0], [-10, 20, 5, 70]):
                _, elbow = model.arm_frames_world(trunk, [0, 30, 20, 50, 0, 0, 0])
                local.append(np.linalg.inv(model.torso_world(trunk)) @ np.r_[elbow, 1])
            np.testing.assert_allclose(local[0], local[1], atol=1e-7)
            self.assertEqual(len(model.actuated_arm), 7)


if __name__ == "__main__":
    unittest.main()
