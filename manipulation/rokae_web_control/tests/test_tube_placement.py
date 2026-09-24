"""Offline tube placement mirror and dispatch/release ordering."""
import copy
import json
from pathlib import Path
import unittest
from unittest.mock import patch

import numpy as np

from rokae_web.arm_movel import transform
from rokae_web.backends import BackendError
from rokae_web.grasp_test import flange_to_tcp
from rokae_web.placement_planning import preflight_tube
from rokae_web.tube_placement import mirrored_preplacement
import test_box_placement
import test_placement_preflight


class MirrorTests(unittest.TestCase):
    def test_mirrors_flange_xyz_and_points_horizontal_with_different_tcps(self):
        tools = {'left_arm': {'end': [.01, -.02, .165, .2, -.1, .3]},
                 'right_arm': {'end': [-.03, .04, .175, -.1, .3, -.2]}}
        reflection = np.diag([1., -1., 1., 1.])
        for attitude in ([0, -70, -180], [180, -90, 0], [23, 14, -54]):
            flange = [400, 40, -340, *attitude]
            result = mirrored_preplacement(flange_to_tcp(flange, tools['left_arm']), tools, -110)
            right_tool = transform(flange_to_tcp([0]*6, tools['right_arm']))
            actual = transform(result) @ np.linalg.inv(right_tool)
            expected = transform([400,-110,-340,180,-90,0])
            np.testing.assert_allclose(actual, expected, atol=1e-9)
            self.assertAlmostEqual(np.linalg.det(actual[:3, :3]), 1.)


class TubePlacementTests(unittest.TestCase):
    def setUp(self):
        self.f = test_box_placement.BoxPlacementTests(); self.f.setUp()
        self.addCleanup(self.f.doCleanups)
        self.service, self.robot, self.job = self.f.service, self.f.robot, self.f.job
        self.service.gripper_unlocked = True
        self.robot.suction_set = lambda *a: self.fail('tube must not change suction')
        tools = {arm:dict(end=[0.]*6, ref=[0.]*6) for arm in ('left_arm','right_arm','trunk')}
        read = self.robot.read_state
        self.robot.read_memory_state = lambda: dict(read(), toolsets=copy.deepcopy(tools))
        path=Path(self.service.config['action_poses_file'])
        data=json.loads(path.read_text())
        for point in data['points']: point['state']['toolsets']=copy.deepcopy(tools)
        path.write_text(json.dumps(data))

    def run_tube(self):
        self.job.execute({'source_result_id':self.f.fixture.source, 'sku_typ':'tube'})
        self.job.thread.join(3)
        self.assertFalse(self.job.thread.is_alive())
        return self.job.status()

    def test_horizontal_pre_trunk_joint6_minus30_open_restore_then_both_arms_and_body(self):
        with patch('rokae_web.placement_planning.preflight_tube', wraps=preflight_tube) as preflight:
            result = self.run_tube()
        self.assertEqual(result['phase'], 'completed', result)
        preflight.assert_called_once()
        np.testing.assert_allclose(preflight.call_args.args[3][:3], [400,20,-340])
        self.assertEqual([e[0] for e in self.robot.events], ['pose','pose','joint','open','joint','memory','body'])
        self.assertEqual(self.robot.events[0][1], 'right_arm')
        np.testing.assert_allclose(self.robot.events[0][2][:3], [400,20,-340])
        expected = transform([0,0,0,180,-90,0])[:3,:3]
        np.testing.assert_allclose(transform(self.robot.events[0][2])[:3,:3], expected, atol=1e-9)
        self.assertEqual(self.robot.events[1], ('pose','trunk',[200,0,600,0,0,0]))
        original = self.f.fixture.start['joints_deg']['right_arm']
        lower = list(original);lower[5] -= 30
        self.assertEqual(self.robot.events[2], ('joint','right_arm',lower))
        self.assertEqual(self.robot.events[4], ('joint','right_arm',original))
        self.assertEqual(self.robot.events[3], ('open',0))
        self.assertEqual(result['gripper_result']['measured_position'], 0)
        self.assertEqual(result['final_state']['joints_deg'], self.f.returned['joints_deg'])
        self.assertEqual(result['completed_moves'], 6)
        self.assertEqual(result['total_moves'], 6)
        self.assertEqual(result['preplacement_seed_deg'], 50)

    def test_missing_right_reference_never_uses_left_or_releases(self):
        data=json.loads(self.f.summary_path.read_text());data['localization'].pop('right_shoulder_frame')
        self.f.summary_path.write_text(json.dumps(data))
        with self.assertRaisesRegex(BackendError, '右肩'): self.run_tube()
        self.assertEqual(self.robot.events, [])

    def test_preflight_failure_and_lock_never_move_or_release(self):
        with patch('rokae_web.placement_planning.preflight_tube', side_effect=BackendError('return path failed')):
            self.assertEqual(self.run_tube()['phase'], 'failed')
        self.assertEqual(self.robot.events, [])
        self.service.gripper_unlocked=False
        with self.assertRaisesRegex(BackendError, '解锁'): self.run_tube()
        self.assertEqual(self.robot.events, [])

    def test_trunk_failure_never_descends_releases_or_returns(self):
        with patch.object(self.job, '_trunk_linear', side_effect=BackendError('trunk failed')):
            self.assertEqual(self.run_tube()['phase'], 'failed')
        self.assertEqual([e[0] for e in self.robot.events], ['pose','stop'])

    def test_release_failure_never_returns(self):
        self.robot.fail_open=True
        self.assertEqual(self.run_tube()['phase'], 'failed')
        self.assertEqual([e[0] for e in self.robot.events], ['pose','pose','joint','open','stop'])

    def test_joint6_failure_never_opens_or_returns(self):
        with patch.object(self.job, '_joint6', side_effect=BackendError('J6 not reached')):
            self.assertEqual(self.run_tube()['phase'],'failed')
        self.assertEqual([e[0] for e in self.robot.events],['pose','pose','stop'])

    def test_restore_failure_after_release_never_returns_to_l2(self):
        original=self.job._joint6;calls=[]
        def joint6(*args):
            calls.append(args[0])
            if len(calls)==2:raise BackendError('restore failed')
            return original(*args)
        with patch.object(self.job,'_joint6',side_effect=joint6):
            self.assertEqual(self.run_tube()['phase'],'failed')
        self.assertEqual([e[0] for e in self.robot.events],['pose','pose','joint','open','stop'])

    def test_cancel_after_joint6_arrival_does_not_release_or_restore(self):
        original=self.job._joint6
        def cancel(*args):
            reached=original(*args);self.job.cancelled.set();return reached
        with patch.object(self.job,'_joint6',side_effect=cancel):
            self.assertEqual(self.run_tube()['phase'],'cancelled')
        self.assertEqual([e[0] for e in self.robot.events],['pose','pose','joint','stop'])

    def test_gripper_lock_or_cancel_before_lowering_keeps_object(self):
        def lock(*a, **kw):
            self.service.gripper_unlocked=False
            self.job._check_cancel()
        with patch.object(self.job, '_trunk_linear', side_effect=lock):
            self.assertEqual(self.run_tube()['phase'], 'failed')
        self.assertEqual([e[0] for e in self.robot.events], ['pose','stop'])


class TubePreflightTests(unittest.TestCase):
    def test_native_geometry_queries_cover_joint6_swing_restore_and_return_without_dispatch(self):
        f=test_placement_preflight.PlannerTests();f.setUp();self.addCleanup(f.doCleanups)
        points=[{'state':copy.deepcopy(f.start)}, {'state':copy.deepcopy(f.start)}, {'state':copy.deepcopy(f.saved)}]
        points[0]['state']['arm_elbow_deg']['left_arm']=-5
        preflight_tube(f.job, points, f.start, [40,-10,0,0,0,0], f.fixture.plane, f.speeds)
        record=next(c.kwargs for c in f.service.audit_event.call_args_list if c.args[0]=='placement_preflight_passed')
        self.assertEqual(record['sku_typ'],'tube')
        self.assertEqual([s['stage'] for s in record['steps']],
                         ['tube_preplacement','tube_advance','tube_joint6','tube_joint6_return','tube_return_arms','tube_return_body'])
        lower=next(s for s in record['steps'] if s['stage']=='tube_joint6')
        restored=next(s for s in record['steps'] if s['stage']=='tube_joint6_return')
        self.assertEqual(lower['module'],'right_arm')
        delta=np.array(lower['joints_deg'])-np.array(restored['joints_deg'])
        np.testing.assert_allclose(delta,[0,0,0,0,0,-30,0])
        f.assert_no_motion()

    def test_joint6_limit_failure_rejects_whole_sequence_before_motion(self):
        f=test_placement_preflight.PlannerTests();f.setUp();self.addCleanup(f.doCleanups)
        points=[{'state':copy.deepcopy(f.start)}, {'state':copy.deepcopy(f.start)}, {'state':copy.deepcopy(f.saved)}]
        original=f.job._limits
        def limits(*args):
            if abs(args[0]['joints_deg']['right_arm'][5] - args[1]['joints_deg']['right_arm'][5] + 30) < 1e-7:raise BackendError('J6 limit')
            return original(*args)
        with patch.object(f.job,'_limits',side_effect=limits),self.assertRaisesRegex(BackendError,'tube_joint6.*整套动作未启动'):
            preflight_tube(f.job,points,f.start,[40,-10,0,0,0,0],f.fixture.plane,f.speeds)
        f.assert_no_motion()
