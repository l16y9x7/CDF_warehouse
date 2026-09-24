"""No hardware calls: exercise the actual endpoint planner with SDK doubles."""
import copy
import json
import math
from pathlib import Path
import tempfile
from types import SimpleNamespace
import unittest
from unittest.mock import patch

import numpy as np
import test_arm_movel
from arm_motion_control.model import RobotModel
from rokae_web.arm_movel import HardwareMoveL, load_guard_plane
from rokae_web.backends import BackendError
from rokae_web.config import DEFAULT_CONFIG


class FrontPlaneTests(unittest.TestCase):
    def setUp(self):
        self.f = test_arm_movel.AdapterTests(); self.f.setUp()
        self.addCleanup(self.f.doCleanups)
        self.x, self.y = 223.0, 70.0
        original = test_arm_movel.AnalyticModel.arm_frames_world
        test = self
        def frames(model, trunk, q):
            flange, elbow = original(model, trunk, q)
            elbow[0], elbow[1] = test.x, model.sign * test.y
            return flange, elbow
        p = patch.object(test_arm_movel.AnalyticModel, 'arm_frames_world', frames)
        p.start(); self.addCleanup(p.stop)
        self.plane = dict(offset_mm=130,elbow_radius_mm=65,margin_mm=10,front_plane_x_mm=220)

    def run_plan(self, side='right', plane=None):
        move = HardwareMoveL(self.f.backend,side+'_arm',self.f.config,self.f.cancel)
        result = move.plan([10,0,100,0,0,0],0,self.plane if plane is None else plane)
        self.assertEqual(len(move.steps),1)
        self.assertFalse(any(e[1] in ('prepare','append','start') for e in self.f.events))
        return result

    def test_both_arms_center_past_front_plane_accepts_despite_negative_y_clearance(self):
        for side in ('left','right'):
            with self.subTest(side=side):
                result=self.run_plan(side)
                self.assertEqual(result['accepted_by'],'front_plane')
                self.assertEqual(result['front_clearance_mm'],3)
                self.assertLess(result['clearance'],0)
                self.assertEqual(result['check_path_calls'],1)

    def test_at_or_behind_x_plane_y_protection_still_required(self):
        for x in (219.9,220.0):
            self.x=x
            with self.subTest(x=x),self.assertRaisesRegex(BackendError,'37 个臂角'):
                self.run_plan()
        self.x=220.0001
        self.assertEqual(self.run_plan()['accepted_by'],'front_plane')

    def test_y_safe_is_enough_when_behind_front_plane(self):
        self.x,self.y=100,220
        self.assertEqual(self.run_plan()['accepted_by'],'y_plane')

    def test_missing_or_null_front_plane_preserves_original_rule(self):
        for plane in ({k:v for k,v in self.plane.items() if k!='front_plane_x_mm'},
                      dict(self.plane,front_plane_x_mm=None)):
            with self.assertRaisesRegex(BackendError,'37 个臂角'):
                self.run_plan(plane=plane)

    def test_new_plane_does_not_override_native_path_or_joint_limits(self):
        self.f.path_fail=-50102
        with self.assertRaisesRegex(BackendError,'37 个臂角'):self.run_plan()
        self.f.path_fail=None
        self.f.backend.soft_limit_status()['joint_limits_deg']['right_arm'][0][1]=5
        # The fixture builds a fresh list on every lookup; constrain this lookup.
        limits=self.f.backend.soft_limit_status()['joint_limits_deg']
        limits['right_arm'][0]=[-1000,5]
        self.f.backend.soft_limit_status=lambda:{'joint_limits_deg':limits}
        with self.assertRaisesRegex(BackendError,'37 个臂角'):self.run_plan()


class ConfigTests(unittest.TestCase):
    def test_reload_and_invalid_x_threshold(self):
        with tempfile.TemporaryDirectory() as temp:
            p=Path(temp)/'torso_guard.json';cfg={'torso_guard_file':str(p)}
            base=dict(plane_offset_mm=130,elbow_radius_mm=65,margin_mm=10)
            for value in (220,240):
                p.write_text(json.dumps(dict(base,front_plane_x_mm=value)))
                self.assertEqual(load_guard_plane(cfg)['front_plane_x_mm'],value)
            for value in (None,):
                p.write_text(json.dumps(dict(base,front_plane_x_mm=value)))
                self.assertNotIn('front_plane_x_mm',load_guard_plane(cfg))
            for value in (False,-1,1001,'NaN','bad'):
                p.write_text(json.dumps(dict(base,front_plane_x_mm=value)))
                with self.subTest(value=value),self.assertRaises(BackendError):load_guard_plane(cfg)


class RecordedFailureTests(unittest.TestCase):
    @unittest.skipUnless(Path(DEFAULT_CONFIG['pose_estimation']['urdf_file']).is_file(),'robot URDF archive required for recorded FK replay')
    def test_recorded_37_candidates_accept_sixth_at_15_degrees_with_new_rule(self):
        data=json.loads((Path(__file__).parent/'fixtures/elbow_x220_20260922.json').read_text())
        f=test_arm_movel.AdapterTests();f.setUp();self.addCleanup(f.doCleanups)
        inp=data['plan'];move=f.executor
        move.start=copy.deepcopy(inp['start'])
        move.conf=list(move.start['arm_conf_data']['right_arm']) if 'arm_conf_data' in move.start else [0,1,0,0,1,0,0,0]
        move.limits=inp['soft_limits_deg']
        move.signature=inp['toolset']
        move.toolset=SimpleNamespace(**{k:SimpleNamespace(trans=v[:3],rpy=v[3:]) for k,v in inp['toolset'].items()})
        def replay(start,joints,goal):
            row=next(c for c in data['calls'] if abs(c['angle_rad']-goal.elbow)<1e-8)
            return (np.degrees(row['q_rad']),None) if row['ec']==0 else (None,'recorded unreachable')
        move._path=replay
        model=RobotModel(DEFAULT_CONFIG['pose_estimation']['urdf_file'],'right')
        with patch('rokae_web.arm_movel.arm_model',return_value=model):
            with self.assertRaisesRegex(BackendError,'37 个臂角'):
                move.plan(inp['target_pose_mm_deg'],inp['seed_arm_angle_deg'],inp['plane'])
            result=move.plan(inp['target_pose_mm_deg'],inp['seed_arm_angle_deg'],dict(inp['plane'],front_plane_x_mm=220))
        self.assertEqual(result['check_path_calls'],6)
        self.assertAlmostEqual(result['angle'],14.999942)
        self.assertEqual(result['accepted_by'],'front_plane')
        np.testing.assert_allclose(result['elbow_chest_mm'],[223.02257115054,-177.73922496447,-91.92427250771],atol=1e-6)
        self.assertFalse(any(e[1] in ('prepare','append','start') for e in f.events))
