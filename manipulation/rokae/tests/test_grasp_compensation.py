"""Synthetic motion only; never imports the native SDK or connects to hardware."""
import copy
import math
import threading
import unittest
from types import SimpleNamespace
from unittest.mock import Mock, patch

import numpy as np

from rokae_web.arm_movel import HardwareMoveL, MotionPlanUnavailable, transform
from rokae_web.backends import BackendError, JOINT_COUNTS
from rokae_web.grasp_compensation import (ADVANCE_OPTIONS_MM, PairedGraspMove, plan_approach,
                                         plan_trunk, shifted, shifted_targets)
from rokae_web.grasp_test import flange_targets, left_flange_targets
from rokae_web.head_kinematics import rpy_rotation
from rokae_web.memory_motion import capture
from rokae_web.pose_frames import ref_pose_to_world
from rokae_web.trunk_retreat import HardwareTrunkRetreat
import test_grasp_test as fixtures
import test_left_box_grasp as left_fixtures
import test_placement_preflight as plan_fixtures


class GeometryTests(unittest.TestCase):
    def test_both_sides_rotated_axes_and_negative_relative_retreat_preserve_world_target(self):
        rotation = rpy_rotation(np.radians([15, -20, 40]))
        for advance, retreat_extra in ((a, e) for a in ADVANCE_OPTIONS_MM for e in (20, 30)):
            d = 35.0  # A >= 100 produces a negative arm retreat.
            lift = [500, 60, 800, 20, 30, 10]
            original_retreat = shifted(lift, rotation, -(d+retreat_extra))
            goals = shifted_targets([('lift','',lift),('arm_retreat','',original_retreat)], rotation, advance)
            approach_world = np.asarray(goals[0][2][:3]) + rotation[:,0]*advance
            np.testing.assert_allclose(approach_world, lift[:3])
            delta = np.asarray(goals[1][2][:3])-goals[0][2][:3]
            np.testing.assert_allclose(rotation.T @ delta, [-(d+retreat_extra-advance),0,0], atol=1e-9)
            for body_back, extra in ((advance, 0), (advance+100, 100)):
                world_delta = delta - rotation[:,0]*body_back
                np.testing.assert_allclose(rotation.T @ world_delta, [-(d+retreat_extra+extra),0,0], atol=1e-9)
            self.assertEqual(goals[0][2][3:], lift[3:])


class SelectionTests(unittest.TestCase):
    def arm(self, results):
        return SimpleNamespace(backend=SimpleNamespace(audit_callback=Mock()), module='left_arm', config={},
                               cancel=threading.Event(), start={'poses':{'trunk':[100,0,900,0,0,0]},
                               'arm_elbow_deg':{'left_arm':23}}, check=Mock(), plan=Mock(side_effect=results))

    def test_reachable_does_not_plan_or_move_body(self):
        arm = self.arm([{'angle':23}])
        with patch('rokae_web.grasp_compensation.plan_trunk') as trunk:
            result, advance, body = plan_approach(arm,[300,0,0,0,0,0],np.eye(3),{})
        self.assertEqual(advance,0); self.assertIsNone(body); trunk.assert_not_called()

    def test_fixed_order_first_success_and_fresh_seed(self):
        arm = self.arm([MotionPlanUnavailable('unreachable')]*3 + [{'angle':28}])
        with patch('rokae_web.grasp_compensation.plan_trunk',return_value={'q':[0]*4}) as trunk:
            result, advance, body = plan_approach(arm,[300,0,0,0,0,0],np.eye(3),{})
        self.assertEqual(advance,150)
        self.assertEqual([c.args[4][0] for c in trunk.call_args_list],[150,200,250])
        self.assertEqual([c.args[0][0] for c in arm.plan.call_args_list],[300,250,200,150])
        self.assertEqual([c.args[1] for c in arm.plan.call_args_list],[23]*4)

    def test_all_four_fail_and_unknown_errors_do_not_fallback(self):
        arm=self.arm([MotionPlanUnavailable('no')]*5)
        with patch('rokae_web.grasp_compensation.plan_trunk',return_value={}) as trunk:
            with self.assertRaisesRegex(MotionPlanUnavailable,'200mm'):
                plan_approach(arm,[300,0,0,0,0,0],np.eye(3),{})
        self.assertEqual(trunk.call_count,4)
        arm=self.arm([BackendError('network offline')])
        with patch('rokae_web.grasp_compensation.plan_trunk') as trunk:
            with self.assertRaisesRegex(BackendError,'network'):
                plan_approach(arm,[300,0,0,0,0,0],np.eye(3),{})
        trunk.assert_not_called()

    def test_preflight_choice_is_reused_without_trying_another_branch(self):
        arm=self.arm([{'angle':23}])
        with patch('rokae_web.grasp_compensation.plan_trunk',return_value={'q':[0]*4}) as trunk:
            _, advance, _=plan_approach(arm,[300,0,0,0,0,0],np.eye(3),{},preselected=100)
        self.assertEqual(advance,100);self.assertEqual(arm.plan.call_count,1)
        self.assertEqual(arm.plan.call_args.args[0][0],200)
        arm=self.arm([MotionPlanUnavailable('changed')])
        with patch('rokae_web.grasp_compensation.plan_trunk') as trunk:
            with self.assertRaises(MotionPlanUnavailable):
                plan_approach(arm,[300,0,0,0,0,0],np.eye(3),{},preselected=0)
        trunk.assert_not_called()


class ExecutionTests(unittest.TestCase):
    def setup_robot(self, kind):
        if kind=='box':
            left=left_fixtures.LeftSequenceTests();left.setUp();self.addCleanup(left.doCleanups)
            self.f, self.left=left.f,left
        else:
            self.f=fixtures.SequenceTests();self.f.setUp();self.addCleanup(self.f.doCleanups)
        f=self.f; self.kind=kind;self.module='left_arm' if kind=='box' else 'right_arm'
        self.backend,self.service=f.backend,f.service
        self.backend.soft_limit_status=lambda:{'joint_limits_deg':{m:[[-10000,10000]]*n for m,n in JOINT_COUNTS.items()}}
        self.service.config['motion']['max_joint_step_deg']={m:None for m in JOINT_COUNTS}
        initial=capture(self.backend)
        self.trunk_origin=list(initial['poses']['trunk']);self.trunk_pose=list(self.trunk_origin)
        self.trunk_q0=list(initial['joints_deg']['trunk']);self.trunk_q=list(self.trunk_q0)
        self.head=list(initial['joints_deg']['head'])
        sdk=self.backend._load_sdk()
        base_cart=sdk.CartesianPosition
        class Cart(base_cart):
            def __init__(self, values):
                super().__init__(values)
                self.trans=list(values[:3]);self.rpy=list(values[3:])
                self.external=np.radians(self_outer.head).tolist()
        self_outer=self
        sdk.CartesianPosition=Cart
        trunk=self.backend._robot('trunk')
        trunk.cartPosture=lambda *a:Cart(self.backend._pose_to_sdk(self.trunk_pose))
        trunk.posture=lambda *a:self.backend._pose_to_sdk(self.trunk_pose)
        trunk.jointPos=lambda ec:np.radians(self.trunk_q+self.head).tolist()
        def solve(cart):
            q=list(self.trunk_q0);q[0]+=(cart.values[0]*1000-self.trunk_origin[0])*.01
            return q
        def ik(cart,tool,ec):
            f.events.append(('trunk','calcIk'))
            return np.radians(solve(cart)).tolist()
        trunk.model=lambda:SimpleNamespace(calcIk=ik)
        self.commands=[]
        def append(commands,identifier,ec):
            f.events.append(('trunk','append'));self.pending=commands[0]
            self.commands.append(commands[0])
        def start(ec):
            f.events.append(('trunk','start'))
            self.trunk_pose=[v*1000 for v in self.pending.target.values[:3]]+np.degrees(self.pending.target.values[3:]).tolist()
            self.trunk_q=solve(self.pending.target)
        trunk.moveAppend=append;trunk.moveStart=start
        p=patch('rokae_web.grasp_test.HardwareTrunkRetreat',HardwareTrunkRetreat);p.start();self.addCleanup(p.stop)
        self.audit=[]
        original=self.service.audit_event
        def record(event,**data):
            self.audit.append((event,copy.deepcopy(data)));original(event,**data)
        self.service.audit_event=record;self.backend.audit_callback=record
        f.events.clear()

    def constrain_approach(self, first_feasible):
        robot=self.backend._robot(self.module);old=robot.checkPath;tool=robot.toolset({})
        source=self.left.shoulder if self.kind=='box' else fixtures.SHOULDER
        name='grasp_pose_left_shoulder_mm_deg' if self.kind=='box' else 'grasp_pose_right_shoulder_mm_deg'
        grasp=source[name]
        def path(start,joints,goal,ec):
            result=old(start,joints,goal,ec)
            world=ref_pose_to_world(goal.values,self.backend._world_from_ref(tool))
            flange=(transform([v*1000 for v in world[:3]]+np.degrees(world[3:]).tolist())
                    @ transform([0,0,-100,0,0,0]))[:3,3]
            # Trigger only after the pregrasp and only on the grasp-height leg.
            moves=self.f.events.count((self.module,'start'))
            if moves==1 and abs(flange[2]-grasp[2])<.01 and flange[0] > grasp[0]-first_feasible+.01:
                ec['ec']=-50102
            return result
        robot.checkPath=path

    def execute(self):
        self.service.grasp_test.execute({'sku_typ':self.kind,'source_result_id':fixtures.SOURCE,'elbow_deg':0})
        self.service.grasp_test.thread.join(12)
        self.assertFalse(self.service.grasp_test.thread.is_alive())
        return self.service.grasp_test.status()

    def test_right_pair_returns_only_A_then_clearance_then_separate_100(self):
        self.setup_robot('bottle');self.constrain_approach(150)
        original=HardwareTrunkRetreat.start_move
        def final_start(move,*args):
            stages=[d['stage'] for e,d in self.audit if e=='grasp_test_stage_completed']
            self.assertEqual(stages[-2:],['arm_retreat','clearance_lift'])
            self.assertAlmostEqual(move.start['poses']['trunk'][0],self.trunk_origin[0])
            self.assertEqual(move.start['joints_deg']['right_arm'],self.backend.read_state()['joints_deg']['right_arm'])
            return original(move,*args)
        with patch.object(HardwareTrunkRetreat,'start_move',final_start):result=self.execute()
        self.assertEqual(result['phase'],'completed',result)
        self.assertEqual(result['advance_mm'],150)
        self.assertEqual(result['arm_retreat_mm'],-50)  # d=80; signed, never clamped.
        self.assertEqual(result['completed_moves'],6)
        self.assertEqual(result['total_moves'],6)
        self.assertEqual(result['paired_trunk_retreat_mm'],150)
        self.assertEqual([round(c.target.values[0]*1000-self.trunk_origin[0]) for c in self.commands],[150,0,-100])
        stages=[d['stage'] for e,d in self.audit if e=='grasp_test_stage_completed']
        self.assertEqual(stages,['pregrasp','grasp','lift','arm_retreat','clearance_lift','trunk_retreat'])
        np.testing.assert_allclose(self.backend.read_state()['poses']['right_arm'][:3],[-80,0,215],atol=1e-7)
        self.assertEqual(self.f.gripper['requested_position'],255)
        self.assertEqual([c.speed for c in self.commands],[77,77,77])
        self.assertTrue(all(abs(c.rotSpeed-math.radians(9))<1e-10 for c in self.commands))
        self.assertEqual(len([e for e,d in self.audit if e=='grasp_synchronized_dispatch']),2)

    def test_right_clearance_lift_failure_never_starts_final_100(self):
        self.setup_robot('bottle');self.constrain_approach(150)
        original=HardwareMoveL.wait_step
        def wait(move,index):
            if self.service.grasp_test.job['stage']=='clearance_lift':
                raise BackendError('clearance lift not reached')
            return original(move,index)
        with patch.object(HardwareMoveL,'wait_step',wait):result=self.execute()
        self.assertEqual(result['phase'],'failed',result)
        self.assertEqual([round(c.target.values[0]*1000-self.trunk_origin[0]) for c in self.commands],[150,0])
        self.assertEqual(self.f.gripper['requested_position'],255)
        self.assertIn(('right_arm','stop'),self.f.events)

    def test_right_cancel_after_clearance_arrival_never_starts_final_100(self):
        self.setup_robot('bottle');self.constrain_approach(150)
        original=HardwareMoveL.wait_step
        def wait(move,index):
            original(move,index)
            if self.service.grasp_test.job['stage']=='clearance_lift':
                self.service.grasp_test.cancel.set()
        with patch.object(HardwareMoveL,'wait_step',wait):result=self.execute()
        self.assertEqual(result['phase'],'cancelled',result)
        self.assertEqual([round(c.target.values[0]*1000-self.trunk_origin[0]) for c in self.commands],[150,0])
        self.assertEqual(self.f.gripper['requested_position'],255)

    def test_left_lift_then_pair_return_A_then_body_100_preserves_suction(self):
        self.setup_robot('box');self.constrain_approach(100)
        original_wait=PairedGraspMove.wait_move
        def measured(pair):
            original_wait(pair)
            # Simulate a valid 0.2deg measured residual after the paired arrival.
            # The subsequent trunk-only move must use this actual joint state.
            self.f.fixture.q['left_arm'][6]+=.2
            return self.backend.read_state()
        with patch.object(PairedGraspMove,'wait_move',measured):result=self.execute()
        self.assertEqual(result['phase'],'completed',result)
        self.assertEqual(result['advance_mm'],100)
        self.assertEqual(result['arm_retreat_mm'],130)  # d=200, extra=30, A=100.
        self.assertEqual(result['completed_moves'],6)
        self.assertEqual([round(c.target.values[0]*1000-self.trunk_origin[0]) for c in self.commands],[100,0,-100])
        stages=[d['stage'] for e,d in self.audit if e=='grasp_test_stage_completed']
        self.assertEqual(stages,['pregrasp','grasp','descend','lift','arm_retreat','trunk_retreat'])
        self.assertEqual(self.left.suction,[(False,0),(True,3)])
        np.testing.assert_allclose(self.backend.read_state()['poses']['left_arm'][:3],[205,100,890],atol=1e-7)

    def test_all_candidates_unreachable_never_dispatch_trunk_or_close_gripper(self):
        self.setup_robot('bottle');self.constrain_approach(250)
        result=self.execute()
        self.assertEqual(result['phase'],'failed',result)
        self.assertEqual(self.commands,[])
        self.assertNotIn(('gripper','close'),self.f.events)
        self.assertEqual(self.f.events.count(('right_arm','start')),1)  # pregrasp only
        candidates=[d['advance_mm'] for e,d in self.audit if e=='grasp_advance_candidate']
        self.assertEqual(candidates,list(ADVANCE_OPTIONS_MM))

    def test_partial_pair_start_failure_stops_both_and_no_gripper_close(self):
        self.setup_robot('bottle');self.constrain_approach(50)
        def fail(ec):
            self.f.events.append(('trunk','start'));ec['ec']=10001;ec['message']='lost reply'
        self.backend._robot('trunk').moveStart=fail
        result=self.execute()
        self.assertEqual(result['phase'],'failed',result)
        for module in ('right_arm','trunk'):
            self.assertIn((module,'stop'),self.f.events)
            self.assertIn((module,'reset'),self.f.events)
        self.assertNotIn(('gripper','close'),self.f.events)

    def test_pair_wait_requires_both_arrived_before_next_stage(self):
        self.setup_robot('bottle');self.constrain_approach(50)
        old_start=self.backend._robot('trunk').moveStart
        self.backend._robot('trunk').moveStart=lambda ec:self.f.events.append(('trunk','start'))
        # Hold the trunk short of target; cancel after the arm has arrived.
        old_wait=PairedGraspMove.wait_move
        def wait(pair):
            timer=threading.Timer(.25,self.service.grasp_test.cancel.set);timer.start()
            try:return old_wait(pair)
            finally:timer.cancel()
        with patch.object(PairedGraspMove,'wait_move',wait):result=self.execute()
        self.assertEqual(result['phase'],'cancelled',result)
        self.assertNotIn(('gripper','close'),self.f.events)
        for module in ('right_arm','trunk'):self.assertIn((module,'stop'),self.f.events)

    def test_stop_confirmation_failure_latches_control_lock(self):
        self.setup_robot('bottle');self.constrain_approach(50)
        def fail_start(ec):ec.update(ec=10001,message='start reply lost')
        def fail_stop(ec):
            self.f.events.append(('trunk','stop'));ec.update(ec=10001,message='stop reply lost')
        trunk=self.backend._robot('trunk');trunk.moveStart=fail_start;trunk.stop=fail_stop
        result=self.execute()
        self.assertTrue(result['stop_unconfirmed'],result)
        self.assertFalse(self.service.armed)
        self.assertIn(('right_arm','stop'),self.f.events)
        self.assertIn(('trunk','reset'),self.f.events)
        self.assertNotIn(('gripper','close'),self.f.events)


class FullPreflightTests(unittest.TestCase):
    def test_box_and_bottle_compensation_are_planned_before_any_dispatch(self):
        from rokae_web.agent_planning import preflight_pick
        for kind in ('box','bottle'):
            with self.subTest(kind=kind):
                f=plan_fixtures.PlannerTests();f.setUp()
                try:
                    module='left_arm' if kind=='box' else 'right_arm'
                    shoulder=left_fixtures.box_geometry()[1] if kind=='box' else copy.deepcopy(fixtures.SHOULDER)
                    key='grasp_pose_left_shoulder_mm_deg' if kind=='box' else 'grasp_pose_right_shoulder_mm_deg'
                    pose=shoulder[key]
                    old=f.fixture.backend._robot(module).checkPath
                    tool=f.fixture.backend._robot(module).toolset({})
                    def path(start,joints,goal,ec):
                        result=old(start,joints,goal,ec)
                        world=ref_pose_to_world(goal.values,f.fixture.backend._world_from_ref(tool))
                        flange=(transform([v*1000 for v in world[:3]]+np.degrees(world[3:]).tolist())
                                @ transform([0,0,-100,0,0,0]))[:3,3]
                        if abs(flange[2]-pose[2])<.01 and abs(flange[0]-pose[0])<.01:ec['ec']=-50102
                        return result
                    f.fixture.backend._robot(module).checkPath=path
                    geometry=SimpleNamespace(project=lambda *a:{'shoulder_grasp':shoulder})
                    body=SimpleNamespace(sdk=f.fixture.backend._load_sdk(),current=SimpleNamespace(confData=[],external=[]))
                    planned_starts=[];original_plan=HardwareMoveL.plan
                    def plan(arm,*args,**kwargs):
                        planned_starts.append(copy.deepcopy(arm.start))
                        return original_plan(arm,*args,**kwargs)
                    with patch('rokae_web.agent_planning.prepare_trunk',return_value=(body,f.start)), \
                         patch('rokae_web.agent_planning.trunk_ik',return_value=[0]*4) as final_ik, \
                         patch.object(HardwareMoveL,'plan',plan), \
                         patch('rokae_web.agent_planning.load_guard_plane',return_value=f.fixture.plane):
                        _,_,projected=preflight_pick(f.service,{'sku_typ':kind},f.start,geometry,f.fixture.cancel)
                        final_ik.assert_called_once()
                        if kind=='bottle':
                            self.assertEqual(planned_starts[-1]['poses']['trunk'],f.start['poses']['trunk'])
                        final_ik.side_effect=BackendError('final retreat unreachable')
                        with self.assertRaisesRegex(BackendError,'整套动作未启动.*final retreat unreachable'):
                            preflight_pick(f.service,{'sku_typ':kind},f.start,geometry,f.fixture.cancel)
                    self.assertEqual(projected['preplanned_advance_mm'],50)
                    record=next(c.kwargs for c in f.service.audit_event.call_args_list if c.args[0]=='agent_pick_preflight')
                    pair=next(p for p in record['arm_plans'] if p['stage']=='arm_retreat')
                    self.assertEqual(pair['trunk_plan']['pose'],f.start['poses']['trunk'])
                    self.assertEqual(record['retreat_pose'][0],f.start['poses']['trunk'][0]-100)
                    f.assert_no_motion()
                finally:f.doCleanups()


if __name__=='__main__':unittest.main()
