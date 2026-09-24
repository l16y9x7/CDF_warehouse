"""Offline tests: no SDK, robot connection, power or motion calls."""
from __future__ import annotations

import math
import unittest
from dataclasses import replace
from pathlib import Path

import numpy as np

from planner import rotation_exp, rotation_log
from torso_guard import (SearchOptions, TorsoPlane, arm_angle_scan, inspect_runtime_sample,
                         plan_guarded_movel, start_matches)


def pose(x=0.0, y=0.0, z=0.0):
    value = np.eye(4)
    value[:3, 3] = [x, y, z]
    return value


class FakeKinematics:
    """Seven-value analytic test model; deliberately not a physical robot."""
    def __init__(self, elbow=None):
        self.elbow = elbow or (lambda q: np.array([q[0], -30.0, 0.0]))
        self.ik_requests = []
        self.unreachable = lambda target, angle: False
        self.joint_limit = lambda joints: True
        self.fk_error = False
        self.ik_result = None

    def ik(self, target, angle, seed):
        self.ik_requests.append((target.copy(), angle, seed.copy()))
        if self.unreachable(target, angle):
            return None
        if self.ik_result is not None:
            return self.ik_result
        return np.r_[target[:3, 3], np.degrees(rotation_log(target[:3, :3])), angle]

    def fk(self, joints):
        frame = pose(*joints[:3])
        frame[:3, :3] = rotation_exp(np.radians(joints[3:6]))
        if self.fk_error and joints[0] > 0:
            frame[0, 3] += 100
        return frame, self.elbow(joints)

    def within_limits(self, joints):
        return self.joint_limit(joints)


class PlaneTests(unittest.TestCase):
    def test_right_negative_y_and_offset_direction(self):
        plane = TorsoPlane("right", 100, margin_mm=10)
        self.assertEqual(plane.plane_y_mm, -100)
        self.assertTrue(plane.measure([0, -120, 0], np.eye(4)).safe)
        self.assertFalse(plane.measure([0, -105, 0], np.eye(4)).safe)
        self.assertFalse(TorsoPlane("right", 120, margin_mm=10).measure([0, -120, 0], np.eye(4)).safe)

    def test_left_is_mirrored(self):
        plane = TorsoPlane("left", 100, margin_mm=10)
        self.assertEqual(plane.plane_y_mm, 100)
        self.assertTrue(plane.measure([0, 120, 0], np.eye(4)).safe)
        self.assertFalse(plane.measure([0, -120, 0], np.eye(4)).safe)

    def test_plane_follows_torso_yaw_pitch_translation(self):
        torso = pose(500, -400, 700)
        torso[:3, :3] = rotation_exp(np.radians([25., -30., 90.]))
        local = np.array([60., -140., 45.])
        world = torso[:3, :3] @ local + torso[:3, 3]
        result = TorsoPlane("right", 100, 10, 20).measure(world, torso)
        np.testing.assert_allclose(result.elbow_torso_mm, local, atol=1e-10)
        self.assertAlmostEqual(result.clearance_mm, 10)

    def test_radius_margin_and_plane_boundary(self):
        result = TorsoPlane("right", 100, 10, 20).measure([0, -130, 0], np.eye(4))
        self.assertTrue(result.safe)
        self.assertEqual(result.clearance_mm, 0)
        self.assertFalse(TorsoPlane("right", 100, 10, 21).measure([0, -130, 0], np.eye(4)).safe)

    def test_invalid_values_are_rejected(self):
        for value in (float("nan"), float("inf"), -1, True):
            with self.subTest(value=value), self.assertRaises(ValueError):
                TorsoPlane("right", value)
        with self.assertRaises(ValueError):
            TorsoPlane("back", 0)
        with self.assertRaises(ValueError):
            TorsoPlane("right", 0).measure([0, float("nan"), 0], np.eye(4))
        with self.assertRaises(ValueError):
            TorsoPlane("right", 0).measure([0, -20, 0], np.zeros((4, 4)))


class PlanningTests(unittest.TestCase):
    def setUp(self):
        self.options = SearchOptions(max_arm_angle_change_deg=10, linear_sample_mm=2,
                                     fk_position_tolerance_mm=0.001, fk_rotation_tolerance_deg=0.001)
        self.plane = TorsoPlane("right", 0, margin_mm=5)

    def run_plan(self, model=None, targets=None, options=None, joints=None, current_angle=0, **kwargs):
        return plan_guarded_movel(model or FakeKinematics(), np.zeros(7) if joints is None else joints,
                                  current_angle, [pose(10)] if targets is None else targets,
                                  world_from_torso_mm=np.eye(4), plane=self.plane,
                                  options=options or self.options, **kwargs)

    def test_scan_exact_order_relative_to_current_without_wrap(self):
        self.assertEqual(arm_angle_scan(20, self.options), (20, 15, 25, 10, 30))
        self.assertEqual(arm_angle_scan(179, self.options), (179, 174, 169))
        self.assertEqual(arm_angle_scan(-179, self.options), (-179, -174, -169))
        self.assertEqual(arm_angle_scan(0, replace(self.options, max_arm_angle_change_deg=0)), (0,))

    def test_current_angle_passes_and_returns_without_scanning_others(self):
        result = self.run_plan()
        self.assertEqual(result.status, "candidate_found")
        self.assertEqual([a.arm_angle_deg for a in result.attempts], [0])
        self.assertEqual(result.plan.selected_arm_angle_deg, 0)
        self.assertAlmostEqual(result.plan.minimum_sampled_clearance_mm, 25)

    def test_explicit_seed_does_not_replace_actual_start_and_scans_negative_first(self):
        checked = []
        def validate(plan):
            checked.append(plan.selected_arm_angle_deg)
            return None if plan.selected_arm_angle_deg == 15 else "native_rejected"
        result = self.run_plan(search_seed_arm_angle_deg=20, validate_candidate=validate)
        self.assertEqual(checked, [20, 15])
        self.assertEqual(result.plan.start_arm_angle_deg, 0)
        self.assertEqual(result.plan.selected_arm_angle_deg, 15)
        self.assertLessEqual(result.plan.waypoints[0].arm_angle_deg, 1)

    def test_native_candidate_error_aborts_instead_of_scanning(self):
        def validate(plan):
            raise RuntimeError("network failure")
        with self.assertRaisesRegex(RuntimeError, "network failure"):
            self.run_plan(validate_candidate=validate)

    def test_native_rejection_of_all_candidates_never_returns_partial_plan(self):
        result = self.run_plan(validate_candidate=lambda plan: "native_rejected")
        self.assertEqual(result.status, "no_solution")
        self.assertIsNone(result.plan)

    def test_radius_65_and_margin_10_apply_symmetrically(self):
        for side, sign in (("right", -1), ("left", 1)):
            plane = TorsoPlane(side, 100, 10, 65)
            self.assertTrue(plane.measure([0, sign * 175, 0], np.eye(4)).safe)
            self.assertFalse(plane.measure([0, sign * 174.9, 0], np.eye(4)).safe)
    def test_zero_hits_plane_negative_five_passes(self):
        model = FakeKinematics(lambda q: np.array([q[0], -20 + 3*q[0] + 5*q[6], 0]))
        result = self.run_plan(model)
        self.assertEqual(result.plan.selected_arm_angle_deg, -5)
        self.assertEqual([a.reason for a in result.attempts], ["elbow_plane", "passed"])
        self.assertTrue(all(w.clearance_mm >= 0 for w in result.plan.waypoints))

    def test_negative_five_rejected_then_positive_five_passes(self):
        model = FakeKinematics(lambda q: np.array([q[0], -20 + 3*q[0] - 5*q[6], 0]))
        result = self.run_plan(model)
        self.assertEqual(result.plan.selected_arm_angle_deg, 5)
        self.assertEqual([a.arm_angle_deg for a in result.attempts], [0, -5, 5])

    def test_negative_ten_found_in_requested_order(self):
        model = FakeKinematics()
        model.unreachable = lambda target, angle: target[0, 3] >= 9.99 and angle > -9
        result = self.run_plan(model)
        self.assertEqual(result.plan.selected_arm_angle_deg, -10)
        self.assertEqual([a.arm_angle_deg for a in result.attempts], [0, -5, 5, -10])

    def test_safe_endpoint_but_interior_path_crossing_rejected(self):
        model = FakeKinematics(lambda q: np.array([q[0], 0 if 3 < q[0] < 7 else -30, 0]))
        result = self.run_plan(model)
        self.assertIsNone(result.plan)
        self.assertEqual(result.status, "no_solution")
        self.assertEqual(len(result.attempts), 5)

    def test_quarter_interval_crossing_is_found_by_joint_probes(self):
        model = FakeKinematics(lambda q: np.array([q[0], 0 if .49 < q[0] < .51 else -30, 0]))
        result = self.run_plan(model, targets=[pose(2)], options=replace(self.options, max_arm_angle_change_deg=0))
        self.assertEqual(result.status, "no_solution")
        self.assertEqual(result.attempts[0].reason, "intermediate_elbow_plane")

    def test_arm_angle_transition_is_checked_not_teleported(self):
        # Candidate -5 has a safe endpoint, but the self-motion crosses near -2.5.
        model = FakeKinematics(lambda q: np.array([q[0], 0 if -3 < q[6] < -2 else -30, 0]))
        model.unreachable = lambda target, angle: target[0, 3] >= 9.99 and angle > -4
        result = self.run_plan(model, options=replace(self.options, max_arm_angle_change_deg=5))
        self.assertEqual(result.status, "no_solution")
        self.assertEqual(result.attempts[1].reason, "intermediate_elbow_plane")

    def test_all_segments_are_checked_before_acceptance(self):
        model = FakeKinematics(lambda q: np.array([q[0], 0 if q[0] > 12 and q[6] >= 0 else -30, 0]))
        result = self.run_plan(model, targets=[pose(10), pose(20)])
        self.assertEqual(result.plan.selected_arm_angle_deg, -5)
        self.assertEqual(result.attempts[0].segment_index, 1)
        self.assertEqual(result.plan.waypoints[-1].segment_index, 1)

    def test_arm_angle_kept_after_first_segment(self):
        model = FakeKinematics(lambda q: np.array([q[0], -20 + 3*q[0] + 5*q[6], 0]))
        result = self.run_plan(model, targets=[pose(10), pose(11)])
        first = [w.arm_angle_deg for w in result.plan.waypoints if w.segment_index == 0]
        later = [w.arm_angle_deg for w in result.plan.waypoints if w.segment_index == 1]
        self.assertTrue(-5 < first[0] < 0)
        self.assertEqual(first[-1], -5)
        self.assertTrue(all(angle == -5 for angle in later))

    def test_hold_pose_arm_angle_change_is_supported(self):
        model = FakeKinematics()
        model.unreachable = lambda target, angle: target[0, 3] > 0 and angle > -4.9
        result = self.run_plan(model, targets=[pose(), pose(2)])
        self.assertEqual(result.plan.selected_arm_angle_deg, -5)
        hold = [w for w in result.plan.waypoints if w.segment_index == 0]
        self.assertEqual(len(hold), 5)
        self.assertTrue(all(np.allclose(w.flange_world_mm, np.eye(4)) for w in hold))

    def test_start_inside_plane_stops_without_scanning_ik(self):
        model = FakeKinematics(lambda q: np.array([0, -1, 0]))
        result = self.run_plan(model)
        self.assertEqual(result.status, "start_inside_protection")
        self.assertEqual(result.ik_calls, 0)
        self.assertEqual(model.ik_requests, [])

    def test_joint_limits_discontinuities_fk_mismatch_are_not_accepted(self):
        model = FakeKinematics()
        model.joint_limit = lambda q: q[0] < 9
        self.assertEqual(self.run_plan(model).attempts[0].reason, "joint_limit")
        model = FakeKinematics()
        model.ik_result = np.array([100., 0, 0, 0, 0, 0, 0])
        self.assertEqual(self.run_plan(model).attempts[0].reason, "joint_discontinuity")
        model = FakeKinematics()
        model.fk_error = True
        self.assertEqual(self.run_plan(model).attempts[0].reason, "fk_pose_mismatch")

    def test_malformed_ik_or_fk_aborts_instead_of_scanning_other_angles(self):
        model = FakeKinematics()
        model.ik_result = np.array([float("nan")] * 7)
        with self.assertRaises(ValueError):
            self.run_plan(model)
        self.assertEqual(len(model.ik_requests), 1)
        model = FakeKinematics(lambda q: np.array([1., float("nan"), 0]))
        with self.assertRaises(ValueError):
            self.run_plan(model)

    def test_transport_error_aborts_instead_of_automatic_fallback(self):
        model = FakeKinematics()
        def failed(*args):
            raise RuntimeError("controller communication lost")
        model.ik = failed
        with self.assertRaisesRegex(RuntimeError, "communication"):
            self.run_plan(model)

    def test_cancellation_and_budget_cannot_return_a_partial_plan(self):
        result = self.run_plan(cancelled=lambda: True)
        self.assertEqual(result.status, "cancelled")
        self.assertEqual(result.ik_calls, 0)
        limited = self.run_plan(options=replace(self.options, max_ik_calls=1))
        self.assertEqual(limited.status, "computation_limit")
        self.assertIsNone(limited.plan)
        limited = self.run_plan(options=replace(self.options, max_fk_calls=1))
        self.assertEqual(limited.status, "computation_limit")
        self.assertIsNone(limited.plan)

    def test_zero_distance_rotation_is_sampled(self):
        goal = pose()
        goal[:3, :3] = rotation_exp(np.radians([0., 0., 10.]))
        result = self.run_plan(targets=[goal])
        self.assertEqual(result.status, "candidate_found")
        self.assertGreaterEqual(len(result.plan.waypoints), 10)

    def test_time_limit_returns_no_plan(self):
        from unittest.mock import patch
        with patch('torso_guard.time.monotonic', side_effect=[0.0, 1000.0]):
            result = self.run_plan()
        self.assertEqual(result.status, 'time_limit')
        self.assertIsNone(result.plan)

    def test_returned_plan_is_independent_of_mutable_inputs(self):
        target = pose(10)
        joints = np.zeros(7)
        result = self.run_plan(targets=[target], joints=joints)
        target[0, 3] = 1000
        joints[:] = 100
        self.assertEqual(result.plan.waypoints[-1].flange_world_mm[0][3], 10)
        self.assertEqual(result.plan.start_joints_deg, (0,) * 7)

    def test_invalid_search_configuration_is_rejected(self):
        for updates in ({"scan_step_deg": 0}, {"max_arm_angle_change_deg": float("nan")},
                        {"linear_sample_mm": float("inf")}, {"joint_probe_step_deg": -1},
                        {"max_ik_calls": True}, {"arm_angle_min_deg": 90, "arm_angle_max_deg": 0}):
            with self.subTest(updates=updates), self.assertRaises(ValueError):
                replace(self.options, **updates)
        with self.assertRaises(ValueError):
            self.run_plan(targets=[])
        with self.assertRaises(ValueError):
            self.run_plan(joints=[False] * 7)

    def test_start_check_and_runtime_guard_are_read_only(self):
        model = FakeKinematics()
        plan = self.run_plan(model).plan
        self.assertTrue(start_matches(plan, np.zeros(7), np.eye(4)))
        self.assertFalse(start_matches(plan, np.ones(7), np.eye(4)))
        self.assertFalse(start_matches(plan, np.zeros(7), pose(1)))
        self.assertFalse(inspect_runtime_sample(plan, model, np.zeros(7), np.eye(4)).stop_required)
        self.assertEqual(inspect_runtime_sample(plan, model, np.zeros(7), pose(1)).reason,
                         "torso_changed_replan_required")
        model.elbow = lambda q: np.array([0., -1., 0.])
        decision = inspect_runtime_sample(plan, model, np.zeros(7), np.eye(4))
        self.assertTrue(decision.stop_required)
        self.assertEqual(decision.reason, "elbow_protection_margin")


class ActualURDFTests(unittest.TestCase):
    @unittest.skipUnless(Path('/home/admin/mui/1.5整机urdf-0.8AR5-20260520.zip').exists(), 'robot URDF is on rokae')
    def test_real_j4_elbow_clearance_is_invariant_under_torso_motion(self):
        from model import RobotModel
        model = RobotModel('/home/admin/mui/1.5整机urdf-0.8AR5-20260520.zip')
        joints = [-28.435341, 83.361385, 88.096921, 109.750424, 8.467685, 7.99015, -3.932556]
        plane = TorsoPlane('right', 0)
        results = []
        for trunk in ([0, 0, 0, 0], [-36.445782, -36.319393, -17.329571, .67516], [-10, 20, 5, 70]):
            _, elbow = model.arm_frames_world(trunk, joints)
            results.append(plane.measure(elbow, model.torso_world(trunk)))
        for measured in results[1:]:
            np.testing.assert_allclose(measured.elbow_torso_mm, results[0].elbow_torso_mm, atol=1e-8)
            self.assertAlmostEqual(measured.clearance_mm, results[0].clearance_mm, places=8)
        self.assertLess(results[0].elbow_torso_mm[1], 0)


class SDKBridgeTests(unittest.TestCase):
    def test_only_known_ik_errors_are_infeasible_and_units_are_degrees(self):
        from types import SimpleNamespace
        from torso_guard_adapter import RightArmGuardKinematics
        code = [0]
        def calc_ik(target, toolset, ec):
            ec['ec'] = code[0]
            return [math.pi / 2] * 7
        arm = SimpleNamespace(
            urdf=SimpleNamespace(torso_world=lambda joints: np.eye(4)), trunk_joints_deg=[0]*4,
            right=SimpleNamespace(model=lambda: SimpleNamespace(calcIk=calc_ik)), toolset=object(),
            target_cartesian=lambda p, a: (p, a), fk=lambda q: (np.eye(4), [0, -50, 0]),
            within_limits=lambda q: True,
        )
        adapter = RightArmGuardKinematics(arm)
        np.testing.assert_allclose(adapter.ik(np.eye(4), 5, np.zeros(7)), [90]*7)
        code[0] = -50102
        self.assertIsNone(adapter.ik(np.eye(4), 5, np.zeros(7)))
        code[0] = 10001
        with self.assertRaisesRegex(RuntimeError, '10001'):
            adapter.ik(np.eye(4), 5, np.zeros(7))


if __name__ == '__main__':
    unittest.main()
