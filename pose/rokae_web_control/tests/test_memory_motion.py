from __future__ import annotations

import copy
import math
import threading
import unittest
from types import SimpleNamespace

from rokae_web.backends import BackendError, XCoreRobotBackend
from rokae_web.memory_motion import capture, prepare, start_arms, start_head_trunk
from test_ar_backend import _FakeArRobot, _FakePCB4Robot, _FakeCartesianPosition, _FakeSdk


class JointPosition:
    def __init__(self, values):
        self.values = values
        self.external = []


class MoveL:
    def __init__(self, target, speed, blend):
        self.target, self.speed, self.blend = target, speed, blend


class MoveJ(MoveL):
    pass


class MoveAbsJ(MoveL):
    pass


class MemoryMotionTests(unittest.TestCase):
    def setUp(self):
        self.backend = XCoreRobotBackend({"sdk_root": "/unused", "local_ip": "local",
                                          "left_arm_ip": "161", "right_arm_ip": "160", "trunk_ip": "162"})
        self.sdk = _FakeSdk()
        self.sdk.JointPosition = JointPosition
        self.sdk.MoveLCommand, self.sdk.MoveJCommand, self.sdk.MoveAbsJCommand = MoveL, MoveJ, MoveAbsJ
        self.sdk.PyString = lambda: None
        self.backend._sdk = self.sdk
        self.events = []
        self.errors = {}
        self.path_results = {}
        self.endpoint_results = {}
        self.fail_start = None
        self.cancel = threading.Event()
        self.backend._prepare_motion = lambda robot, name, speed: self.events.append((name, "prepare"))
        self.backend.soft_limit_status = lambda: {"joint_limits_deg": {
            "left_arm": [[-180, 180]] * 7, "right_arm": [[-180, 180]] * 7,
            "trunk": [[-180, 180]] * 4, "head": [[-180, 180]] * 2}}
        for name in ("left_arm", "right_arm", "trunk"):
            robot = self.backend._robot(name)
            tool = SimpleNamespace(ref=SimpleNamespace(trans=[0., 0., 0.], rpy=[0., 0., 0.]),
                                   end=SimpleNamespace(trans=[0., 0., .1], rpy=[0., 0., 0.]))
            robot.toolset = lambda ec, tool=tool: tool
            robot.moveAppend = lambda commands, command_id, ec, name=name: self.events.append((name, "append"))
            def move_start(ec, name=name):
                self.events.append((name, "start"))
                if name == self.fail_start:
                    ec["ec"] = 10001
                    ec["message"] = "communication failure"
            robot.moveStart = move_start
            robot.stop = lambda ec, name=name: self.events.append((name, "stop"))
            robot.moveReset = lambda ec, name=name: self.events.append((name, "reset"))
            if name != "trunk":
                def ik(target, tool, ec, name=name):
                    ec["ec"] = self.errors.get((name, "ik"), 0)
                    return self.endpoint_results.get(name, [math.radians(1)] * 7)
                robot.model = lambda ik=ik: SimpleNamespace(calcIk=ik)
                def path(start, joints, target, ec, name=name):
                    self.events.append((name, "checkPath"))
                    ec["ec"] = self.errors.get((name, "path"), 0)
                    return self.path_results.get(name, [math.radians(1)] * 7)
                robot.checkPath = path
        self.target = capture(self.backend)
        for name in self.target["joints_deg"]:
            self.target["joints_deg"][name] = [1.0] * len(self.target["joints_deg"][name])
        for name in ("left_arm", "right_arm"):
            self.target["poses"][name][0] += 10.0
        self.events.clear()

    def plan(self):
        return prepare(self.backend, self.target, {"linear_mm_s": 50.0, "rotation_deg_s": 9.0}, self.cancel)

    def test_success_uses_full_check_path_and_web_default_speed(self):
        plan = self.plan()
        self.assertEqual(self.events, [("left_arm", "checkPath"), ("right_arm", "checkPath")])
        for name in ("left_arm", "right_arm"):
            command = plan["commands"][name]
            self.assertIsInstance(command, MoveL)
            self.assertNotIsInstance(command, MoveJ)
            self.assertEqual(command.speed, 50)
            self.assertAlmostEqual(command.rotSpeed, math.radians(9))
            self.assertEqual(command.blend, 0)
            self.assertEqual(command.target.confData, self.target["arm_conf_data"][name])
        self.assertEqual(plan["body_command"].speed, 50)
        self.assertFalse(hasattr(plan["body_command"], "jointSpeed"))
        self.assertEqual(plan["body_command"].target.external, [math.radians(1)] * 2)

    def test_only_unreachable_path_falls_back_to_movej_for_that_arm(self):
        self.errors[("right_arm", "path")] = -50102
        plan = self.plan()
        self.assertIs(type(plan["commands"]["left_arm"]), MoveL)
        self.assertIs(type(plan["commands"]["right_arm"]), MoveJ)
        self.assertEqual(plan["commands"]["right_arm"].speed, 50)
        self.assertFalse(hasattr(plan["commands"]["right_arm"], "jointSpeed"))
        self.assertEqual(plan["modes"]["right_arm"]["motion"], "MoveJ")

    def test_unreachable_endpoint_replays_saved_seven_joints_at_web_speed(self):
        self.errors[("left_arm", "ik")] = -50002
        plan = self.plan()
        command = plan["commands"]["left_arm"]
        self.assertIs(type(command), MoveAbsJ)
        self.assertEqual(command.target.values, [math.radians(1)] * 7)
        self.assertEqual(command.speed, 50)
        self.assertFalse(hasattr(command, "jointSpeed"))

    def test_unknown_or_communication_error_cannot_fall_back_or_start(self):
        self.errors[("right_arm", "path")] = 10001
        with self.assertRaises(BackendError):
            self.plan()
        self.assertFalse(any(event in ("prepare", "append", "start") for _, event in self.events))

    def test_wrong_ik_branch_uses_recorded_joint_angles(self):
        self.endpoint_results["right_arm"] = [math.radians(20)] * 7
        plan = self.plan()
        self.assertIs(type(plan["commands"]["right_arm"]), MoveAbsJ)
        self.assertEqual(plan["commands"]["right_arm"].target.values, [math.radians(1)] * 7)

    def test_changed_tool_or_out_of_limits_is_rejected_before_motion(self):
        self.target["toolsets"]["left_arm"]["end"][0] = .3
        with self.assertRaisesRegex(BackendError, "TCP"):
            self.plan()
        self.target["toolsets"] = capture(self.backend)["toolsets"]
        self.target["joints_deg"]["head"][0] = 900
        with self.assertRaisesRegex(BackendError, "软限位"):
            self.plan()
        self.assertEqual(self.events, [])

    def test_both_arms_enqueued_before_either_start(self):
        plan = self.plan()
        self.events.clear()
        start_arms(self.backend, plan, self.cancel)
        self.assertEqual(self.events, [("left_arm", "prepare"), ("left_arm", "append"),
                                       ("right_arm", "prepare"), ("right_arm", "append"),
                                       ("left_arm", "start"), ("right_arm", "start")])

    def test_joint_change_after_preflight_prevents_both_starts(self):
        plan = self.plan()
        self.backend._robot("left_arm").jointPos = lambda ec: [math.radians(10)] * 7
        self.events.clear()
        with self.assertRaisesRegex(BackendError, "状态改变"):
            start_arms(self.backend, plan, self.cancel)
        self.assertEqual(self.events, [])

    def test_head_and_trunk_share_one_command_after_arm_arrival(self):
        plan = self.plan()
        with self.assertRaisesRegex(BackendError, "尚未到位"):
            start_head_trunk(self.backend, plan, self.cancel)
        for name in ("left_arm", "right_arm"):
            self.backend._robot(name).jointPos = lambda ec: [math.radians(1)] * 7
        self.events.clear()
        start_head_trunk(self.backend, plan, self.cancel)
        self.assertEqual(self.events, [("trunk", "prepare"), ("trunk", "append"), ("trunk", "start")])

    def test_stop_attempts_all_controllers_when_one_fails(self):
        def fail(ec):
            ec["ec"] = 10001
        self.backend._robot("left_arm").stop = fail
        errors = self.backend.stop_memory_motion()
        self.assertEqual(len(errors), 1)
        self.assertIn(("right_arm", "stop"), self.events)
        self.assertIn(("trunk", "stop"), self.events)


if __name__ == "__main__":
    unittest.main()
