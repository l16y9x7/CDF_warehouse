"""Offline sequence/SDK doubles; no real controller connections."""
import copy
import http.client
import json
import math
from pathlib import Path
import tempfile
import threading
from types import SimpleNamespace
import unittest
from unittest.mock import patch

import numpy as np

import test_arm_movel as arm_fixtures
from rokae_web.arm_movel import transform
from rokae_web.audit import MemoryAuditLogger
from rokae_web.control_trace import context
from rokae_web.backends import BackendError, MockChassisBackend, MockRobotBackend
from rokae_web.config import DEFAULT_CONFIG
from rokae_web.grasp_test import flange_targets, flange_to_tcp
from rokae_web.gripper import Robotiq2F85
from rokae_web.head_kinematics import UpperBodySixDofKinematics
from rokae_web.pose_frames import POSE_FRAMES
from rokae_web.service import ControlService
from rokae_web.web import make_server
import test_gripper as gripper_fixtures


SOURCE = "20260917/pose_120000000_abcdef"
SHOULDER = {"frame": POSE_FRAMES["right_arm"],
            "R_right_shoulder_from_trunk_ref": np.eye(3).tolist(),
            "pregrasp_pose_right_shoulder_mm_deg": [10, 0, 0, 0, 0, 0],
            "grasp_pose_right_shoulder_mm_deg": [20, 0, 0, 0, 0, 0],
            "box_clearance": {"valid": True, "frame": "trunk_controller_ref", "D_mm": 400, "d_mm": 80}}


class GeometryTests(unittest.TestCase):
    def test_offsets_use_trunk_axes_and_preserve_orientation(self):
        shoulder = copy.deepcopy(SHOULDER)
        shoulder["grasp_pose_right_shoulder_mm_deg"] = [200, -100, 300, 25, 40, 15]
        targets = dict((key, pose) for key, _, pose in flange_targets(shoulder))
        self.assertEqual(targets["lift"], [200, -100, 340, 25, 40, 15])
        self.assertEqual(targets["arm_retreat"], [100, -100, 340, 25, 40, 15])
        self.assertEqual(targets["clearance_lift"], [100, -100, 415, 25, 40, 15])
        signature = {"end": [.02, -.03, .17, .3, -.2, .1]}
        tcp = flange_to_tcp(targets["grasp"], signature)
        expected = transform(targets["grasp"]) @ transform([20, -30, 170, *np.degrees([.3, -.2, .1])])
        np.testing.assert_allclose(transform(tcp), expected, atol=1e-9)
        # The offset is in trunk SDK axes, independent of flange and tool rotation.
        lift_tcp = flange_to_tcp(targets["lift"], signature)
        np.testing.assert_allclose(np.array(lift_tcp[:3]) - tcp[:3], [0, 0, 40], atol=1e-9)

    def test_real_urdf_chest_axes_match_sdk_shoulder_at_rotated_trunk(self):
        path = DEFAULT_CONFIG["pose_estimation"]["urdf_file"]
        if not Path(path).is_file():
            self.skipTest("现场 URDF 不在此离线主机")
        model = UpperBodySixDofKinematics(path)
        for trunk in ([0, 0, 0, 0], [-10, 15, -20, 35]):
            chest = model.forward_deg([*trunk, 0, 0], tip_link="Chest_link")
            shoulder = model.right_shoulder_sdk_world(trunk)
            np.testing.assert_allclose(chest[:3, :3], shoulder[:3, :3], atol=1e-10)

    def test_gripper_stop_keeps_activation_and_never_auto_releases(self):
        arm = gripper_fixtures.FakeArm()
        arm.status_words = [0xF100, 0x00FF, 0xB000]
        result = Robotiq2F85(gripper_fixtures.SDK, arm).stop()
        self.assertTrue(result["activated"])
        self.assertFalse(result["going_to_position"])
        self.assertIn((9, 0x10, 0x03E8, "uint16", 3, [0x0100, 0, 0], False), arm.requests)


class SequenceTests(unittest.TestCase):
    def setUp(self):
        self.fixture = arm_fixtures.AdapterTests()
        self.fixture.setUp()
        self.addCleanup(self.fixture.doCleanups)
        self.addCleanup(self.fixture.tearDown)
        self.backend = self.fixture.backend
        self.events = self.fixture.events
        owner = self
        class TrunkDouble:
            def __init__(self, backend, config, cancel, start, distance):
                self.backend, self.cancel = backend, cancel
                self.q0 = list(start["joints_deg"]["trunk"])
                self.q = list(self.q0)
                backend._robot("trunk").jointPos = lambda ec: np.radians(self.q + start["joints_deg"]["head"]).tolist()
                owner.assertEqual(distance, 100)
                owner.events.append(("trunk", "preflight"))
            def start_move(self, key, expected, speed, rotation):
                owner.assertEqual(speed, 77)
                owner.assertEqual(rotation, 9)
                owner.assertIn(key, ("trunk_retreat", "trunk_return"))
                self.key = key
                self.q = [-10, 5, 5, 0] if key == "trunk_retreat" else list(self.q0)
                owner.events.append((key, "start"))
            def wait_move(self):
                return self.backend.read_state()
            def stop_and_verify(self):
                owner.events.append(("trunk", "stop"))
                return []
        self.trunk_class = TrunkDouble
        body_patch = patch("rokae_web.grasp_test.HardwareTrunkRetreat", TrunkDouble)
        body_patch.start()
        self.addCleanup(body_patch.stop)
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        config = copy.deepcopy(DEFAULT_CONFIG)
        config["torso_guard_file"] = str(Path(self.temp.name) / "guard.json")
        Path(config["torso_guard_file"]).write_text(json.dumps({
            "plane_offset_mm": 20, "elbow_radius_mm": 65, "margin_mm": 10}))
        config["memory_points"] = {"file": str(Path(self.temp.name) / "memory.json")}
        self.gripper = {"fault_code": 0, "activation_state": 3, "going_to_position": True,
                        "object_state": 3, "requested_position": 0, "measured_position": 3}
        self.backend.gripper_status = lambda: dict(self.gripper)
        self.backend.gripper_start_move = self.close_gripper
        self.backend.gripper_stop = lambda: self.events.append(("gripper", "stop")) or {"going_to_position": False}
        self.estimator = SimpleNamespace(reproject_latest_to_right_shoulder=lambda a, b: {
            "source_result_id": SOURCE, "sku_typ": "bottle", "world_grasp": {"sku_typ": "bottle"}, "shoulder_grasp": copy.deepcopy(SHOULDER),
            "current_trunk_joints_deg": list(a["joints_deg"]["trunk"])})
        self.service = ControlService(config, self.backend, MockChassisBackend(), True,
                                      pose_estimator=self.estimator)
        self.addCleanup(self.service.close)
        self.service.arm({})
        self.service.set_gripper_unlocked({"unlocked": True})
        self.service.set_speed({"speed_mm_s": 77, "rotation_deg_s": 9})
        self.origin = self.backend.read_state()["poses"]["right_arm"]
        self.events.clear()

    def close_gripper(self, position):
        self.events.append(("gripper", "close" if position else "open"))
        self.gripper.update(requested_position=position, object_state=2 if position else 3, measured_position=position)

    def execute(self):
        self.service.grasp_test.execute({"sku_typ": "bottle", "source_result_id": SOURCE, "elbow_deg": 0})
        self.service.grasp_test.thread.join(5)
        self.assertFalse(self.service.grasp_test.thread.is_alive())
        return self.service.grasp_test.status()

    def test_arm_and_trunk_order_stops_after_retreat_without_either_return(self):
        status = self.execute()
        self.assertEqual(status["phase"], "completed", status)
        self.assertEqual(status["completed_moves"], 6)
        self.assertEqual(status["speed_mm_s"], 77)
        motions = [e for e in self.events if e[1] in ("start", "close")]
        self.assertEqual(motions, [("right_arm", "start")]*2 + [("gripper", "close"),
            *( [("right_arm", "start")] * 3 ), ("trunk_retreat", "start")])
        self.assertEqual(self.events.count(("right_arm", "checkPath")), 5)
        self.assertEqual(self.events.count(("right_arm", "append")), 5)
        self.assertLess(self.events.index(("trunk", "preflight")), self.events.index(("right_arm", "start")))
        self.assertEqual(self.fixture.current_command.speed, 77)
        self.assertAlmostEqual(self.fixture.current_command.rotSpeed, math.radians(9))
        self.assertEqual(status["total_moves"], 6)
        np.testing.assert_allclose(self.backend.read_state()["poses"]["right_arm"], status["targets_tcp_mm_deg"]["clearance_lift"], atol=1e-7)
        targets = status["targets_tcp_mm_deg"]
        np.testing.assert_allclose(targets["grasp"][:3], [20, 0, 100], atol=1e-7)
        np.testing.assert_allclose(targets["lift"][:3], [20, 0, 140], atol=1e-7)
        np.testing.assert_allclose(targets["arm_retreat"][:3], [-80, 0, 140], atol=1e-7)
        np.testing.assert_allclose(targets["clearance_lift"][:3], [-80, 0, 215], atol=1e-7)
        self.assertEqual(self.backend.read_state()["joints_deg"]["trunk"], [-10, 5, 5, 0])
        self.assertNotIn("return", targets)
        self.assertNotIn(("trunk_return", "start"), self.events)
        self.assertEqual(self.gripper["requested_position"], 255)

    def test_return_stages_remain_available_behind_disabled_switch(self):
        with patch("rokae_web.grasp_test.RUN_RETURN_STAGES", True):
            status = self.execute()
        self.assertEqual(status["phase"], "completed", status)
        self.assertEqual(status["completed_moves"], 8)
        self.assertEqual(status["total_moves"], 8)
        self.assertIn(("trunk_return", "start"), self.events)
        np.testing.assert_allclose(self.backend.read_state()["poses"]["right_arm"], self.origin, atol=1e-7)

    def test_grasp_lift_use_fresh_angles_and_never_restore_start(self):
        original_plan = self.fixture.executor.__class__.plan
        original_wait = self.fixture.executor.__class__.wait_step
        self.fixture.q["right_arm"][6] = 23
        seeds = []
        guarded = []
        def plan(executor, pose, seed, plane):
            seeds.append(seed)
            guarded.append(plane is not None)
            return original_plan(executor, pose, seed + 5 if plane is not None else seed, plane)
        def wait(executor, index):
            original_wait(executor, index)
            if guarded[-1]:
                self.fixture.q["right_arm"][6] += .07
        with patch("rokae_web.grasp_test.HardwareMoveL.plan", plan), \
             patch("rokae_web.grasp_test.HardwareMoveL.wait_step", wait):
            status = self.execute()
        self.assertEqual(status["phase"], "completed", status)
        np.testing.assert_allclose(seeds, [0, 5.07, 10.14, 15.21, 20.28], atol=1e-5)
        self.assertEqual(guarded, [True] * 5)
        self.assertAlmostEqual(status["start_arm_angle_deg"], 23)
        self.assertAlmostEqual(self.backend.read_state()["arm_elbow_deg"]["right_arm"], 25.35)
        self.assertEqual(status["protected_arm_stages"], ["pregrasp", "grasp", "lift", "arm_retreat", "clearance_lift"])

    def test_speed_pair_is_frozen_even_when_changed_after_first_segment(self):
        start_step = self.fixture.executor.__class__.start_step
        used = []
        def dispatch(executor, index, speed, rotation):
            used.append((speed, rotation))
            start_step(executor, index, speed, rotation)
            self.service.set_speed({"speed_mm_s": 123, "rotation_deg_s": 15})
        with patch("rokae_web.grasp_test.HardwareMoveL.start_step", dispatch):
            status = self.execute()
        self.assertEqual(status["phase"], "completed", status)
        self.assertEqual(used, [(77, 9)] * 5)
        self.assertEqual(status["rotation_deg_s"], 9)
        self.assertEqual(self.service.rotation_deg_s, 15)

    def test_full_trace_keeps_request_id_inputs_states_and_phase_timings(self):
        audit = self.service.audit = MemoryAuditLogger()
        token = context.set({"request_id": "grasp-click-123"})
        try:
            status = self.execute()
        finally:
            context.reset(token)
        self.assertEqual(status["phase"], "completed", status)
        self.assertGreaterEqual(status["first_dispatch_ms"], status["timings_ms"]["pregrasp_plan"])
        records = [r for r in audit.records if r.get("operation_id") == status["operation_id"]]
        self.assertTrue(records)
        self.assertTrue(all(r["request_id"] == "grasp-click-123" for r in records))
        phases = {r["phase_name"] for r in records if r["event"] == "phase_finished"}
        self.assertTrue({"initial_snapshot", "gripper_ready", "reproject_pose", "trunk_endpoint_preflight",
                         "pregrasp_plan", "pregrasp_dispatch", "lift_plan", "trunk_retreat_dispatch"} <= phases)
        paths = [r for r in records if r["event"] == "sdk_call" and r["action"] == "MoveL checkPath"]
        self.assertEqual(len(paths), 5)
        self.assertEqual(len(paths[0]["args"][1]), 7)
        self.assertEqual(len(paths[0]["args"][2]["values"]), 6)  # Fake SDK stores trans+rpy together.
        states = [r for r in records if r["event"] == "robot_state_observed"]
        self.assertGreater(len(states), 4)
        self.assertEqual(sum(map(len, states[-1]["state"]["joints_deg"].values())), 20)
        self.assertTrue(any(r["event"] == "motion_first_observed" for r in records))
        self.assertNotIn("return_plan", phases)
        self.assertNotIn("trunk_return_dispatch", phases)
        stages = [r["stage"] for r in records if r["event"] == "grasp_test_stage_completed"]
        self.assertEqual(stages, ["pregrasp", "grasp", "lift", "arm_retreat", "clearance_lift", "trunk_retreat"])
        started = next(r for r in records if r["event"] == "grasp_test_start")
        self.assertEqual(started["rotation_deg_s"], 9)

    def test_mock_stops_at_lift_and_body_retreat_without_restoring_origin(self):
        self.service.hardware_enabled = False
        self.service.robot = MockRobotBackend()
        self.service.robot.gripper_activate()
        origin = self.service.robot.read_state()["poses"]["right_arm"]
        self.service.robot.move_pose("right_arm", origin, 77, 23)
        status = self.execute()
        self.assertEqual(status["phase"], "completed", status)
        self.assertAlmostEqual(status["start_arm_angle_deg"], 23)
        self.assertAlmostEqual(self.service.robot.read_state()["arm_elbow_deg"]["right_arm"], 0)
        self.assertEqual(self.service.robot.read_state()["poses"]["right_arm"], status["targets_tcp_mm_deg"]["clearance_lift"])
        self.assertEqual(status["completed_moves"], 6)

    def test_unreachable_trunk_rejects_before_any_arm_or_gripper_command(self):
        with patch("rokae_web.grasp_test.HardwareTrunkRetreat", side_effect=BackendError("躯干不可达")):
            status = self.execute()
        self.assertEqual(status["phase"], "failed")
        self.assertNotIn(("right_arm", "start"), self.events)
        self.assertNotIn(("gripper", "close"), self.events)

    def test_cancel_trunk_stops_trunk_and_does_not_retract_arm_or_return_body(self):
        def wait(trunk):
            self.service.grasp_test.cancel.set()
            raise BackendError("cancelled")
        with patch.object(self.trunk_class, "wait_move", wait):
            status = self.execute()
        self.assertEqual(status["phase"], "cancelled")
        self.assertIn(("trunk", "stop"), self.events)
        self.assertEqual(self.events.count(("right_arm", "start")), 5)
        self.assertNotIn(("trunk_return", "start"), self.events)

    def test_unreachable_lift_cancels_retreat_and_return_without_movej(self):
        close = self.backend.gripper_start_move
        def unreachable(position):
            close(position)
            self.fixture.path_fail = -50102
        self.backend.gripper_start_move = unreachable
        status = self.execute()
        self.assertEqual(status["phase"], "failed")
        self.assertEqual(status["completed_moves"], 2)
        self.assertEqual(self.events.count(("right_arm", "start")), 2)

    def test_result_changed_rejects_before_motion(self):
        self.estimator.reproject_latest_to_right_shoulder = lambda a, b: {"source_result_id": "new"}
        status = self.execute()
        self.assertIn("结果已更新", status["message"])
        self.assertNotIn(("right_arm", "start"), self.events)
        self.assertNotIn(("gripper", "close"), self.events)

    def test_missing_or_invalid_box_distance_rejects_before_motion(self):
        for box in ({}, {"valid": False}, {"valid": True, "frame": "trunk_controller_ref", "d_mm": -1},
                    {"valid": True, "frame": "trunk_controller_ref", "d_mm": float("nan")}):
            shoulder = {**SHOULDER, "box_clearance": box}
            self.estimator.reproject_latest_to_right_shoulder = lambda a, b: {
                "source_result_id": SOURCE, "sku_typ": "bottle", "world_grasp": {"sku_typ": "bottle"}, "shoulder_grasp": shoulder,
                "current_trunk_joints_deg": [0] * 4}
            with self.subTest(box=box):
                status = self.execute()
                self.assertEqual(status["phase"], "failed")
                self.assertNotIn(("right_arm", "start"), self.events)
                self.assertNotIn(("gripper", "close"), self.events)

    def test_added_arm_stages_failure_prevents_remaining_motion(self):
        plan = self.fixture.executor.__class__.plan
        for failed_stage in ("arm_retreat", "clearance_lift"):
            with self.subTest(failed_stage=failed_stage):
                self.events.clear()
                def fail(executor, *args):
                    if self.service.grasp_test.job["stage"] == failed_stage:
                        raise BackendError("不可达")
                    return plan(executor, *args)
                with patch("rokae_web.grasp_test.HardwareMoveL.plan", fail):
                    status = self.execute()
                self.assertEqual(status["phase"], "failed")
                self.assertEqual(status["stage"], failed_stage)
                self.assertNotIn(("trunk_retreat", "start"), self.events)

    def test_gripper_not_ready_rejects_before_arm_motion(self):
        self.gripper["activation_state"] = 0
        self.assertEqual(self.execute()["phase"], "failed")
        self.assertNotIn(("right_arm", "checkPath"), self.events)

    def test_gripper_locked_prevents_planning(self):
        self.service.set_gripper_unlocked({"unlocked": False})
        self.backend.gripper_status = lambda: self.fail("locked grasp must not probe gripper")
        status = self.execute()
        self.assertEqual(status["phase"], "failed", status)
        self.assertEqual(status["completed_moves"], 0)
        self.assertNotIn(("right_arm", "checkPath"), self.events)
        self.assertNotIn(("gripper", "close"), self.events)

    def test_unrecognized_gripper_prevents_planning(self):
        def unavailable():
            raise BackendError("Modbus timeout")
        self.backend.gripper_status = unavailable
        status = self.execute()
        self.assertEqual(status["phase"], "failed", status)
        self.assertNotIn(("right_arm", "checkPath"), self.events)
        self.assertNotIn(("right_arm", "start"), self.events)
        self.assertNotIn(("gripper", "close"), self.events)

    def test_tool_change_between_steps_aborts(self):
        wait = self.fixture.executor.__class__.wait_step
        def changed(executor, index):
            wait(executor, index)
            executor.robot.toolset({}).end.trans[2] += .01
        with patch("rokae_web.grasp_test.HardwareMoveL.wait_step", changed):
            status = self.execute()
        self.assertIn("工具或工件坐标系改变", status["message"])
        self.assertEqual(self.events.count(("right_arm", "start")), 1)

    def test_cancel_while_closing_stops_gripper_and_skips_lift(self):
        entered = threading.Event()
        def closing(position):
            self.gripper.update(requested_position=position, object_state=0)
            entered.set()
        self.backend.gripper_start_move = closing
        self.service.grasp_test.execute({"sku_typ": "bottle", "source_result_id": SOURCE})
        self.assertTrue(entered.wait(2))
        self.service.grasp_test.stop()
        self.service.grasp_test.thread.join(3)
        self.assertEqual(self.service.grasp_test.status()["phase"], "cancelled")
        self.assertIn(("gripper", "stop"), self.events)
        self.assertEqual(self.events.count(("right_arm", "start")), 2)

    def test_cancel_planning_and_motion_mutual_exclusion(self):
        entered = threading.Event()
        def hold(executor, *args):
            entered.set()
            executor.cancel.wait(3)
            executor.check()
        with patch("rokae_web.grasp_test.HardwareMoveL.plan", hold):
            self.service.grasp_test.execute({"sku_typ": "bottle", "source_result_id": SOURCE})
            self.assertTrue(entered.wait(1))
            for action in (
                lambda: self.service.move_joints("head", {"values": [0, 0]}),
                lambda: self.service.move_gripper({"position": 0}),
                lambda: self.service.memory.save({"name": "blocked"}),
                lambda: self.service.set_drag("right_arm", {"enabled": False}),
                lambda: self.service.chassis_command({"linear_x": 0, "linear_y": 0, "angular_z": 0}),
                lambda: self.service.arm_movel.execute("right_arm", {"values": [0]*6, "frame": POSE_FRAMES["right_arm"]}, True),
                lambda: self.service.grasp_test.execute({"sku_typ": "bottle", "source_result_id": SOURCE}),
            ):
                with self.assertRaises(BackendError):
                    action()
            self.service.disarm()
            self.service.grasp_test.thread.join(3)
        self.assertNotIn(("right_arm", "start"), self.events)
        self.assertEqual(self.service.grasp_test.status()["phase"], "cancelled")

    def test_failed_start_and_stop_latches_lock(self):
        with patch("rokae_web.grasp_test.HardwareMoveL.start_step", side_effect=BackendError("start failed")), \
             patch("rokae_web.grasp_test.HardwareMoveL.stop_and_verify", return_value=["stop failed"]):
            status = self.execute()
        self.assertTrue(status["stop_unconfirmed"])
        self.assertFalse(self.service.armed)
        self.service.arm({})
        with self.assertRaisesRegex(BackendError, "停止未确认"):
            self.service.move_joints("head", {"values": [0, 0]})

    def test_http_routes_require_unlock_and_run_mock_only(self):
        audit = self.service.audit = MemoryAuditLogger()
        # Explicitly substitute a Mock backend before exposing this local server.
        self.service.hardware_enabled = False
        self.service.robot = MockRobotBackend()
        self.service.robot.gripper_activate()
        self.service.armed = False
        static = Path(__file__).resolve().parents[1] / "static"
        server = make_server("127.0.0.1", 0, self.service, static)
        thread = threading.Thread(target=server.serve_forever, daemon=True)
        thread.start()
        def request(path, body):
            conn = http.client.HTTPConnection("127.0.0.1", server.server_port, timeout=3)
            conn.request("POST", path, body=json.dumps(body), headers={"Content-Type": "application/json",
                         "X-Control-Request-Id": "mock-http-click", "X-Control-Sent-At": "2026-09-17T13:00:00Z"})
            response = conn.getresponse()
            self.assertEqual(response.getheader("X-Control-Request-Id"), "mock-http-click")
            result = response.status, json.loads(response.read())
            conn.close()
            return result
        try:
            route = "/api/pose-estimation/grasp-test"
            self.assertEqual(request(route, {"sku_typ": "bottle", "source_result_id": SOURCE})[0], 400)
            self.service.armed = True
            self.assertEqual(request(route, {"sku_typ": "bottle", "source_result_id": SOURCE})[0], 200)
            self.service.grasp_test.thread.join(2)
            self.assertIn("MOCK", self.service.grasp_test.status()["message"])
            self.assertEqual(self.service.grasp_test.status()["request_id"], "mock-http-click")
            self.assertTrue(any(r["event"] == "grasp_test_progress" and r.get("request_id") == "mock-http-click"
                                for r in audit.records))
            self.assertEqual(request(route + "/stop", {})[0], 200)
        finally:
            server.shutdown()
            server.server_close()
            thread.join(2)


if __name__ == "__main__":
    unittest.main()
