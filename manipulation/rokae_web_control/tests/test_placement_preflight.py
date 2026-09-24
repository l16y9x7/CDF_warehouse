"""Offline only: real planner logic with analytic SDK doubles; never dispatch."""
import copy
import math
import unittest
from types import SimpleNamespace
from unittest.mock import Mock, patch

import numpy as np
import test_arm_movel
import test_placement_sequence
from rokae_web.backends import BackendError, JOINT_COUNTS
from rokae_web.memory_motion import capture
from rokae_web.placement_sequence import PlacementSequence, ARM
from rokae_web.placement_planning import preflight_bottle, PlanningBackend
from rokae_web.pose_frames import ref_pose_to_world


class PlannerTests(unittest.TestCase):
    def setUp(self):
        f = self.fixture = test_arm_movel.AdapterTests(); f.setUp()
        self.addCleanup(f.doCleanups)
        limits = {name:[[-1000,1000]]*count for name,count in JOINT_COUNTS.items()}
        f.backend.soft_limit_status = lambda: {'joint_limits_deg':limits}
        f.config['motion']['max_joint_step_deg'] = {name:None for name in JOINT_COUNTS}
        self.start = capture(f.backend)
        trunk = f.backend._robot('trunk')
        def posture(*args):
            c = f.backend._load_sdk().CartesianPosition(f.backend._pose_to_sdk(self.start['poses']['trunk']))
            c.confData = [0]*8
            c.external = np.radians(self.start['joints_deg']['head']).tolist()
            return c
        trunk.cartPosture = posture
        def ik(cart, tool, ec):
            f.events.append(('trunk','calcIk'))
            q = list(self.start['joints_deg']['trunk'])
            q[0] += (cart.values[0]*1000-self.start['poses']['trunk'][0])*.01
            return np.radians(q).tolist()
        trunk.model = lambda: SimpleNamespace(calcIk=ik)
        f.q[ARM] = [50,0,0,0,0,0,10]
        self.saved = capture(f.backend)
        f.q[ARM] = [0]*7
        self.service = SimpleNamespace(robot=f.backend, config=f.config, hardware_enabled=True, audit_event=Mock())
        self.job = SimpleNamespace(service=self.service, cancelled=f.cancel, _check_cancel=lambda:None)
        self.job._limits = lambda *a:PlacementSequence._limits(self.job,*a)
        self.speeds = dict(linear_mm_s=73,rotation_deg_s=8)
        self.reference = [430,50,-300]
        f.events.clear()

    def run_plan(self):
        return preflight_bottle(self.job,self.saved,self.start,self.reference,self.fixture.plane,self.speeds)

    def assert_no_motion(self):
        self.assertFalse(any(e[1] in ('prepare','append','start','stop','reset') for e in self.fixture.events))
        self.assertEqual(capture(self.fixture.backend),self.start)

    def test_all_eight_segments_and_return_paths_are_preflighted_without_motion(self):
        self.run_plan()
        completed = [c.kwargs for c in self.service.audit_event.call_args_list if c.args[0]=='placement_preflight_passed'][0]
        stages = list(dict.fromkeys(s['stage'] for s in completed['steps']))
        self.assertEqual(stages,['memory','align_y','advance','joint7','joint7_return','retreat','unalign_y','return_start'])
        self.assertEqual(self.fixture.events.count(('right_arm','checkPath')),6)
        self.assertEqual(self.fixture.events.count(('trunk','calcIk')),2)
        self.assertEqual(completed['speed'],self.speeds)
        arm_advance = next(s for s in completed['steps'] if s['stage']=='advance' and 'arm_plan' in s)
        self.assertEqual(arm_advance['target'][:2],[80,20])
        self.assert_no_motion()

    def test_advance_unreachable_rejected_before_first_motion(self):
        robot = self.fixture.backend._robot(ARM)
        old = robot.checkPath
        tool = robot.toolset({})
        def path(start,joints,goal,ec):
            target = ref_pose_to_world(goal.values,self.fixture.backend._world_from_ref(tool))
            result = old(start,joints,goal,ec)
            if abs(target[0]*1000-80)<.01:ec['ec']=-50102
            return result
        robot.checkPath = path
        with self.assertRaisesRegex(BackendError,'advance.*整套动作未启动'):
            self.run_plan()
        self.assert_no_motion()

    def test_j7_limit_failure_before_motion(self):
        self.fixture.backend.soft_limit_status()['joint_limits_deg'][ARM][6] = [-10,1000]
        with self.assertRaisesRegex(BackendError,'advance.*软限位'):
            self.run_plan()
        self.assert_no_motion()

    def test_return_path_failure_before_motion(self):
        robot = self.fixture.backend._robot(ARM)
        old = robot.checkPath
        tool = robot.toolset({})
        def path(start,joints,goal,ec):
            result = old(start,joints,goal,ec)
            goal_world = ref_pose_to_world(goal.values,self.fixture.backend._world_from_ref(tool))
            if math.degrees(joints[0]) > 79 and abs(goal_world[0]*1000-50)<.01:ec['ec']=-50102
            return result
        robot.checkPath = path
        with self.assertRaisesRegex(BackendError,'retreat.*整套动作未启动'):
            self.run_plan()
        self.assert_no_motion()

    def test_planning_backend_has_no_motion_methods(self):
        view=PlanningBackend(self.fixture.backend,self.start)
        for name in ('move_pose','move_joints','start_memory_arms','_prepare_motion'):
            with self.assertRaises(AttributeError):getattr(view,name)
        for name in ('moveStart','moveAppend','setPowerState','stop'):
            with self.assertRaises(AttributeError):getattr(view._robot(ARM),name)


class DispatchGateTests(unittest.TestCase):
    def setUp(self):
        self.fixture=test_placement_sequence.PlacementTests();self.fixture.setUp()
        self.addCleanup(self.fixture.tearDown)
        self.f=self.fixture

    def test_late_preflight_failure_sends_no_motion_or_gripper_command(self):
        with patch('rokae_web.placement_planning.preflight_bottle',side_effect=BackendError('return path failed')) as p:
            self.f.job.execute({'source_result_id':self.f.source},preflight_all=True)
            self.f.job.thread.join(2)
        self.assertEqual(self.f.job.status()['phase'],'failed')
        self.assertEqual(self.f.job.status()['completed_moves'],0)
        self.assertEqual(self.f.robot.events,[])
        p.assert_called_once()

    def test_preflight_completes_before_first_dispatch(self):
        def checked(*args):
            self.assertEqual(self.f.robot.events,[])
        with patch('rokae_web.placement_planning.preflight_bottle',side_effect=checked) as p:
            self.f.job.execute({'source_result_id':self.f.source},preflight_all=True)
            self.f.job.thread.join(3)
        self.assertEqual(self.f.job.status()['phase'],'completed',self.f.job.status())
        p.assert_called_once()

    def test_start_change_during_preflight_prevents_dispatch(self):
        def changed(*args):
            self.f.robot._state['joints_deg'][ARM][0] += 1
        with patch('rokae_web.placement_planning.preflight_bottle',side_effect=changed):
            self.f.job.execute({'source_result_id':self.f.source},preflight_all=True)
            self.f.job.thread.join(2)
        self.assertEqual(self.f.job.status()['phase'],'failed')
        self.assertEqual(self.f.robot.events,[])

    def test_cancel_during_preflight_prevents_dispatch(self):
        with patch('rokae_web.placement_planning.preflight_bottle',side_effect=lambda *a:self.f.job.cancelled.set()):
            self.f.job.execute({'source_result_id':self.f.source},preflight_all=True)
            self.f.job.thread.join(2)
        self.assertEqual(self.f.job.status()['phase'],'cancelled')
        self.assertEqual(self.f.robot.events,[])
