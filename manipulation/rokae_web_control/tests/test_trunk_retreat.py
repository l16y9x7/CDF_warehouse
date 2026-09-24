"""Synthetic PCB4 tests; deliberately no SDK import or real connection."""
import copy
import math
import threading
from types import SimpleNamespace
import unittest

import numpy as np

from rokae_web.arm_movel import pose_values, transform
from rokae_web.backends import BackendError, MockRobotBackend, XCoreRobotBackend
from rokae_web.config import DEFAULT_CONFIG
from rokae_web.trunk_retreat import HardwareTrunkRetreat


class Cart:
    def __init__(self, values):
        self.trans, self.rpy = list(values[:3]), list(values[3:])
        self.external = [.1, -.2, 0, 0, 0, 0]
        self.confData = [0]*8


def chest(q):
    return transform([q[0]*1000, 0, q[1]*1000, 0, math.degrees(q[2]), math.degrees(q[3])])


class TrunkTests(unittest.TestCase):
    def setUp(self):
        self.config = copy.deepcopy(DEFAULT_CONFIG)
        self.cancel = threading.Event()
        self.events = []
        self.q = [.1, 1., 0., 0.]
        self.error = 0
        self.joint_jump = 0
        self.tool = SimpleNamespace(
            end=SimpleNamespace(trans=[.04, -.03, .12], rpy=[.2, -.1, .3]),
            ref=SimpleNamespace(trans=[.1, .2, -.05], rpy=[0, .2, 0]))
        self.ref = transform([100, 200, -50, *np.degrees([0, .2, 0])])
        self.end = transform([40, -30, 120, *np.degrees([.2, -.1, .3])])
        self.backend = MockRobotBackend()
        self.backend._pose_to_sdk = XCoreRobotBackend._pose_to_sdk
        self.backend._call = XCoreRobotBackend._call.__get__(self.backend)
        self.backend._check_ec = XCoreRobotBackend._check_ec
        self.backend.soft_limit_status = lambda: {"joint_limits_deg": {"trunk": [[-180,180]]*4}}
        self.sdk = SimpleNamespace(CartesianPosition=Cart, CoordinateType=SimpleNamespace(endInRef=0),
            PyString=lambda: None, MoveLCommand=lambda target, speed, zone: SimpleNamespace(target=target, speed=speed, zone=zone))
        self.robot = SimpleNamespace(toolset=lambda ec: self.tool,
            cartPosture=lambda coordinate, ec: self.fk(self.q),
            checkPath=lambda *args: self.fail("PCB4 checkPath is not used"),
            model=lambda: SimpleNamespace(calcFk=lambda q, tool, ec: self.fk(q), calcIk=self.ik),
            moveAppend=self.append, moveStart=self.start,
            stop=lambda ec: self.events.append("stop"), moveReset=lambda ec: self.events.append("reset"))
        self.backend._load_sdk = lambda: self.sdk
        self.backend._robot = lambda module: self.robot
        self.backend._operation_name = lambda robot, module: "idle"
        self.backend._prepare_motion = lambda robot, module, speed: self.events.append("prepare")
        read = self.backend.read_state
        def read_state():
            result = read()
            result["joints_deg"]["trunk"] = np.degrees(self.q).tolist()
            result["joints_deg"]["head"] = np.degrees(self.fk(self.q).external[:2]).tolist()
            c = self.fk(self.q)
            result["poses"]["trunk"] = [v*1000 for v in c.trans] + np.degrees(c.rpy).tolist()
            result["operation_state"] = {m: "idle" for m in ("left_arm", "right_arm", "trunk")}
            return result
        self.backend.read_state = read_state

    def fk(self, q):
        matrix = np.linalg.inv(self.ref) @ chest(q) @ self.end
        return Cart(self.backend._pose_to_sdk(pose_values(matrix)))

    def solve(self, goal):
        values = [v*1000 for v in goal.trans] + np.degrees(goal.rpy).tolist()
        target = self.ref @ transform(values) @ np.linalg.inv(self.end)
        pose = pose_values(target)
        return [pose[0]/1000, pose[2]/1000, math.radians(pose[4]), math.radians(pose[5])]

    def ik(self, goal, tool, ec):
        self.events.append("calcIk")
        ec["ec"] = self.error
        values = self.solve(goal)
        values[0] += self.joint_jump
        return values

    def append(self, commands, identifier, ec):
        self.events.append("append")
        self.assertEqual(len(commands), 1)
        self.command = commands[0]

    def start(self, ec):
        self.events.append("start")
        self.q = self.solve(self.command.target)

    def executor(self):
        return HardwareTrunkRetreat(self.backend, self.config, self.cancel, self.backend.read_state(), 220)

    def test_preflight_is_readonly_and_two_movel_commands_restore_original_joints(self):
        q0 = list(self.q)
        executor = self.executor()
        self.assertEqual(self.events.count("calcIk"), 0)
        self.assertEqual(executor.preflight_stats["fk_calls"], 0)
        self.assertEqual(executor.preflight_stats["path_samples"], 0)
        self.assertFalse(executor.preflight_stats["return_preflight"])
        self.assertEqual(self.events, [])
        for key in ("trunk_retreat", "trunk_return"):
            before = self.backend.read_state()
            executor.start_move(key, before, 66, 9)
            after = executor.wait_move()
            self.assertEqual(self.command.speed, 66)
            self.assertAlmostEqual(self.command.rotSpeed, math.radians(9))
            self.assertEqual(self.command.target.external, [.1, -.2, 0, 0, 0, 0])
            for module in ("left_arm", "right_arm", "head"):
                self.assertEqual(before["joints_deg"][module], after["joints_deg"][module])
            if key == "trunk_retreat":
                np.testing.assert_allclose(chest(self.q)[:3, 3],
                    np.array([100, 0, 1000]) + self.ref[:3, :3] @ [-220, 0, 0], atol=1e-8)
        np.testing.assert_allclose(self.q, q0, atol=1e-8)
        self.assertEqual(self.events.count("append"), 2)
        self.assertEqual(self.events.count("start"), 2)
        self.assertEqual(self.events.count("calcIk"), 0)  # Neither dispatch re-plans.

    def test_failing_model_ik_is_not_used(self):
        self.error = -50002
        executor = self.executor()
        self.assertEqual(executor.preflight_stats["ik_calls"], 0)
        self.assertEqual(self.events, [])

    def test_retreat_changes_only_sdk_reference_x_at_rotated_flange(self):
        self.q[2:] = [.25, .8]
        executor = self.executor()
        start = executor.current
        goal = executor.moves["trunk_retreat"]["cart"]
        np.testing.assert_allclose(np.asarray(goal.trans) - start.trans, [-.22, 0, 0], atol=1e-9)
        np.testing.assert_allclose(transform([0, 0, 0, *np.degrees(goal.rpy)])[:3, :3],
                                   transform([0, 0, 0, *np.degrees(start.rpy)])[:3, :3], atol=1e-9)
        self.assertEqual(goal.external, start.external)
        self.assertEqual(self.events, [])

    def test_return_uses_saved_start_without_solving_again(self):
        executor = self.executor()
        self.assertIsNone(executor.moves["trunk_return"]["from_q"])
        np.testing.assert_allclose(executor.moves["trunk_return"]["q"], np.degrees(self.q), atol=1e-8)
        executor.start_move("trunk_retreat", self.backend.read_state(), 66, 9)
        executor.wait_move()
        np.testing.assert_allclose(executor.moves["trunk_return"]["from_q"], np.degrees(self.q), atol=1e-8)
        self.assertEqual(self.events.count("calcIk"), 0)
        self.assertEqual(self.events.count("start"), 1)

    def test_return_requires_confirmed_retreat(self):
        executor = self.executor()
        with self.assertRaisesRegex(BackendError, "尚未确认到位"):
            executor.start_move("trunk_return", self.backend.read_state(), 66, 9)
        self.assertNotIn("start", self.events)

    def test_live_feedback_checks_configured_joint_step_limit(self):
        self.config["motion"]["max_joint_step_deg"]["trunk"] = 1
        executor = self.executor()
        executor.start_move("trunk_retreat", self.backend.read_state(), 66, 9)
        with self.assertRaisesRegex(BackendError, "单次关节变化"):
            executor.wait_move()

    def test_different_sdk_euler_representation_does_not_call_ik(self):
        def current(coordinate, ec):
            cart = self.fk(self.q)
            cart.rpy[2] += 2 * math.pi
            return cart
        self.robot.cartPosture = current
        executor = self.executor()
        self.assertEqual(self.events.count("calcIk"), 0)
        self.assertEqual(executor.preflight_stats["fk_calls"], 0)

    def test_start_soft_limit_rejected_before_command(self):
        self.backend.soft_limit_status = lambda: {"joint_limits_deg": {"trunk": [[10, 180]] * 4}}
        with self.assertRaisesRegex(BackendError, "软限位"):
            self.executor()
        self.assertNotIn("start", self.events)

    def test_sdk_fk_is_not_called(self):
        def forbidden(*args):
            self.fail("躯干规划不得调用 SDK 模型")
        self.robot.model = lambda: SimpleNamespace(calcFk=forbidden, calcIk=self.ik)
        self.executor()
        self.assertEqual(self.events, [])

    def test_stale_state_or_changed_tool_prevents_dispatch(self):
        executor = self.executor()
        state = self.backend.read_state()
        self.backend._state["joints_deg"]["right_arm"][0] = 1
        with self.assertRaisesRegex(BackendError, "位置改变"):
            executor.start_move("trunk_retreat", state, 66, 9)
        self.backend._state["joints_deg"]["right_arm"][0] = 0
        self.tool.end.trans[0] += .01
        with self.assertRaisesRegex(BackendError, "工具"):
            executor.start_move("trunk_retreat", state, 66, 9)
        self.assertNotIn("start", self.events)

    def test_arm_change_during_trunk_wait_is_detected(self):
        executor = self.executor()
        executor.start_move("trunk_retreat", self.backend.read_state(), 66, 9)
        self.backend._state["joints_deg"]["left_arm"][2] = .2
        with self.assertRaisesRegex(BackendError, "双臂或头部"):
            executor.wait_move()
        self.assertEqual(executor.stop_and_verify(), [])
        self.assertEqual(self.events[-2:], ["stop", "reset"])


if __name__ == "__main__":
    unittest.main()
