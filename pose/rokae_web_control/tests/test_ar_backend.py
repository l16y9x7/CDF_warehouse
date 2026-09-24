from __future__ import annotations

import math
import unittest
from types import SimpleNamespace

from rokae_web.backends import Ros2ChassisBackend, XCoreRobotBackend
from rokae_web.pose_frames import POSE_FRAMES, ref_pose_to_world


class _FakeRobot:
    def __init__(self, *args: str) -> None:
        self.args = args
        self.connect_calls = 0
        self.disconnect_calls = 0
        self.calls: list[tuple] = []

    def connectToRobot(self, ec: dict) -> None:
        self.connect_calls += 1
        ec["ec"] = 0

    def disconnectFromRobot(self, ec: dict) -> None:
        self.disconnect_calls += 1
        ec["ec"] = 0

    def setPowerState(self, enabled: bool, ec: dict) -> None:
        self.calls.append(("setPowerState", enabled))
        ec["ec"] = 0

    def setOperateMode(self, mode: object, ec: dict) -> None:
        self.calls.append(("setOperateMode", mode))
        ec["ec"] = 0

    def moveReset(self, ec: dict) -> None:
        self.calls.append(("moveReset",))
        ec["ec"] = 0

    def enableDrag(self, space: object, kind: object, ec: dict, no_button: bool) -> None:
        self.calls.append(("enableDrag", space, kind, no_button))
        ec["ec"] = 0

    def disableDrag(self, ec: dict) -> None:
        self.calls.append(("disableDrag",))
        ec["ec"] = 0

    def operationState(self, ec: dict) -> str:
        ec["ec"] = 0
        return "idle"


class _FakeCartesianState:
    trans = [0.1, 0.2, 0.3]
    rpy = [0.4, 0.5, 0.6]
    elbow = 1.25
    confData = [-1, 0, 0, 1, 0, -1, 0, 1]


class _FakeModel:
    def __init__(self, robot: _FakeRobot) -> None:
        self.robot = robot

    def calcIk(self, target: object, toolset: object, ec: dict) -> list[float]:
        self.robot.calls.append(("calcIk", target, toolset))
        ec["ec"] = 0
        return [0.0] * 7


class _FakeCartesianPosition:
    def __init__(self, values: list[float]) -> None:
        self.values = list(values)
        self.elbow = 0.0
        self.hasElbow = False
        self.confData: list[int] = []


class _FakeMoveJCommand:
    def __init__(self, target: object, speed: float, blend: float) -> None:
        self.target = target
        self.speed = speed
        self.blend = blend


class _FakeArRobot(_FakeRobot):
    instances: list[_FakeArRobot] = []

    def __init__(self, remote_ip: str, local_ip: str) -> None:
        super().__init__(remote_ip, local_ip)
        self.active_toolset = SimpleNamespace(
            ref=SimpleNamespace(
                trans=[0.0, -0.0775 if remote_ip.endswith("161") else 0.0775, -0.1429],
                rpy=[0.0, 0.0, 0.0],
            ),
            end="unchanged-tcp",
            load="unchanged-load",
        )
        self.instances.append(self)

    def cartPosture(self, coordinate: object, ec: dict) -> _FakeCartesianState:
        self.calls.append(("cartPosture", coordinate))
        ec["ec"] = 0
        return _FakeCartesianState()

    def jointPos(self, ec: dict) -> list[float]:
        ec["ec"] = 0
        return [0.0] * 7

    def toolset(self, ec: dict) -> object:
        self.calls.append(("toolset",))
        ec["ec"] = 0
        return self.active_toolset

    def model(self) -> _FakeModel:
        return _FakeModel(self)


class _FakePCB4Robot(_FakeRobot):
    instances: list[_FakePCB4Robot] = []

    def toolset(self, ec):
        ec["ec"] = 0
        return SimpleNamespace(end=SimpleNamespace(trans=[0.0]*3, rpy=[0.0]*3),
                               ref=SimpleNamespace(trans=[0.0]*3, rpy=[0.0]*3))

    def __init__(self, remote_ip: str) -> None:
        super().__init__(remote_ip)
        self.instances.append(self)

    def jointPos(self, ec: dict) -> list[float]:
        ec["ec"] = 0
        return [0.0] * 6

    def posture(self, coordinate: object, ec: dict) -> list[float]:
        ec["ec"] = 0
        return [0.0] * 6


class _FakeSdk:
    ArRobot = _FakeArRobot
    PCB4Robot = _FakePCB4Robot
    OperateMode = type("OperateMode", (), {"manual": "manual"})
    DragParameterSpace = type("DragParameterSpace", (), {"cartesianSpace": 1})
    DragParameterType = type("DragParameterType", (), {"freely": 2})
    CoordinateType = type("CoordinateType", (), {"endInRef": "endInRef"})
    CartesianPosition = _FakeCartesianPosition
    MoveJCommand = _FakeMoveJCommand


class ArBackendConstructionTests(unittest.TestCase):
    def setUp(self) -> None:
        _FakeArRobot.instances.clear()
        _FakePCB4Robot.instances.clear()
        self.backend = XCoreRobotBackend(
            {
                "sdk_root": "/unused/in/unit/test",
                "local_ip": "192.168.71.51",
                "left_arm_ip": "192.168.71.161",
                "right_arm_ip": "192.168.71.160",
                "trunk_ip": "192.168.71.162",
            }
        )
        self.backend._sdk = _FakeSdk()

    def test_arm_uses_ar_constructor_without_second_connect(self) -> None:
        robot = self.backend._robot("left_arm")

        self.assertEqual(robot.args, ("192.168.71.161", "192.168.71.51"))
        self.assertEqual(robot.connect_calls, 0)
        self.assertIs(self.backend._robot("left_arm"), robot)
        self.assertEqual(len(_FakeArRobot.instances), 1)

    def test_head_and_trunk_share_pcb4_constructor(self) -> None:
        trunk = self.backend._robot("trunk")
        head = self.backend._robot("head")

        self.assertIs(trunk, head)
        self.assertEqual(trunk.args, ("192.168.71.162",))
        self.assertEqual(trunk.connect_calls, 0)
        self.assertEqual(len(_FakePCB4Robot.instances), 1)

    def test_close_disconnects_each_constructed_controller_once(self) -> None:
        arm = self.backend._robot("right_arm")
        trunk = self.backend._robot("trunk")

        self.backend.close()

        self.assertEqual(arm.disconnect_calls, 1)
        self.assertEqual(trunk.disconnect_calls, 1)

    def test_drag_requires_holding_end_button(self) -> None:
        self.backend.set_drag("left_arm", True)

        robot = _FakeArRobot.instances[0]
        self.assertEqual(
            robot.calls,
            [
                ("setPowerState", False),
                ("setOperateMode", "manual"),
                ("moveReset",),
                ("enableDrag", 1, 2, False),
            ],
        )

    def test_arm_pose_uses_requested_elbow_and_preserves_configuration(self) -> None:
        captured: dict[str, object] = {}
        self.backend._prepare_motion = lambda *args: None
        self.backend._send_command = lambda robot, module, command: captured.update(
            robot=robot,
            module=module,
            command=command,
        )

        self.backend.move_pose("right_arm", [100, 200, 300, 10, 20, 30], 50, 35.0)

        command = captured["command"]
        target = command.target
        self.assertEqual(captured["module"], "right_arm")
        self.assertTrue(target.hasElbow)
        self.assertAlmostEqual(target.elbow, math.radians(35.0))
        self.assertEqual(target.confData, _FakeCartesianState.confData)
        for actual, expected in zip(target.values, [0.1, 0.1225, 0.4429]):
            self.assertAlmostEqual(actual, expected)
        for actual, expected in zip(target.values[3:], [10, 20, 30]):
            self.assertAlmostEqual(actual, math.radians(expected))
        robot = captured["robot"]
        self.assertTrue(any(call[0] == "calcIk" for call in robot.calls))
        ik_call = next(call for call in robot.calls if call[0] == "calcIk")
        self.assertIs(ik_call[1], target)
        self.assertIs(ik_call[2], robot.active_toolset)
        self.assertEqual(robot.active_toolset.end, "unchanged-tcp")
        self.assertEqual(robot.active_toolset.load, "unchanged-load")

    def test_read_state_uses_each_shoulder_world_and_keeps_trunk_reference(self) -> None:
        state = self.backend.read_state()

        self.assertEqual(state["pose_frames"], POSE_FRAMES)
        self.assertEqual(state["poses"]["left_arm"][:3], [100.0, 122.5, 157.1])
        self.assertEqual(state["poses"]["right_arm"][:3], [100.0, 277.5, 157.1])
        self.assertEqual(state["poses"]["trunk"], [0.0] * 6)
        self.assertAlmostEqual(state["arm_elbow_deg"]["left_arm"], math.degrees(1.25), places=5)
        for robot in _FakeArRobot.instances:
            self.assertEqual(robot.active_toolset.ref.rpy, [0.0, 0.0, 0.0])
            self.assertFalse(any(call[0].startswith("set") for call in robot.calls))

    def test_readback_and_command_are_inverse_even_for_rotated_work_object(self) -> None:
        robot = self.backend._robot("left_arm")
        robot.active_toolset.ref.trans = [0.1, -0.2, 0.3]
        robot.active_toolset.ref.rpy = [0.2, -0.3, 0.4]
        frame = robot.active_toolset.ref.trans + robot.active_toolset.ref.rpy
        reference_pose = _FakeCartesianState.trans + _FakeCartesianState.rpy
        world_pose = ref_pose_to_world(reference_pose, frame)
        captured = {}
        self.backend._prepare_motion = lambda *args: None
        self.backend._send_command = lambda robot, module, command: captured.update(command=command)

        readback = self.backend.read_state()
        expected_ui = self.backend._pose_to_ui(world_pose)
        self.assertEqual(readback["poses"]["left_arm"], expected_ui)
        self.backend.move_pose("left_arm", expected_ui, 50)

        for actual, expected in zip(captured["command"].target.values, reference_pose):
            self.assertAlmostEqual(actual, expected, places=7)
        self.assertAlmostEqual(captured["command"].target.elbow, 1.25)

    def test_soft_limit_cache_has_service_lifetime(self) -> None:
        expected = {
            "joint_limits_deg": {
                "left_arm": [[-1.0, 1.0]] * 7,
                "right_arm": [[-1.0, 1.0]] * 7,
                "trunk": [[-1.0, 1.0]] * 4,
                "head": [[-1.0, 1.0]] * 2,
            },
            "joint_soft_limit_enabled": {
                "left_arm": True,
                "right_arm": True,
                "trunk": True,
                "head": True,
            },
            "joint_limit_source": "once",
        }
        self.backend._soft_limits_cache = expected
        self.backend._load_sdk = lambda: self.fail("cached limits must not reload the SDK")

        first = self.backend.soft_limit_status()
        first["joint_limits_deg"]["left_arm"][0][0] = -99.0
        second = self.backend.soft_limit_status()

        self.assertEqual(second, expected)


class Ros2ChassisBackendTests(unittest.TestCase):
    def test_remote_control_error_70002_explains_estop(self) -> None:
        message = Ros2ChassisBackend._remote_control_error("70002")

        self.assertIn("急停", message)
        self.assertIn("70002", message)


if __name__ == "__main__":
    unittest.main()
