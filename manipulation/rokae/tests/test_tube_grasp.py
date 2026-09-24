"""Offline controller doubles only: no robot connection or native SDK."""
import copy
import math
import unittest
from unittest.mock import patch

import numpy as np

from rokae_web.arm_movel import HardwareMoveL, MotionPlanUnavailable, transform
from rokae_web.backends import BackendError
from rokae_web.grasp_compensation import PairedGraspMove
from rokae_web.grasp_test import flange_to_tcp
from rokae_web.head_kinematics import rpy_rotation, rotation_to_rpy_deg
from rokae_web.pose_targets import selection
from rokae_web.action_poses import tube_height
from rokae_web.tube_grasp import tube_targets, compensated_targets, STAGES
import test_grasp_compensation as motion_fixtures
import test_grasp_test as fixtures
import test_pose_targets as pose_fixtures


def shoulder():
    result=copy.deepcopy(fixtures.SHOULDER)
    for key in ('pregrasp','grasp'):
        result[key+'_pose_right_shoulder_mm_deg'][3:]=[180.,-90.,0.]
    return result


class GeometryTests(unittest.TestCase):
    def test_rotated_frame_flange_center_and_signed_recovery_with_nonzero_tcp(self):
        rotation=rpy_rotation(np.radians([15,-20,40]))
        data=shoulder();data['R_right_shoulder_from_trunk_ref']=rotation.tolist()
        for key in ('grasp','pregrasp'):
            data[key+'_pose_right_shoulder_mm_deg'][3:]=rotation_to_rpy_deg(rotation @ rpy_rotation(np.radians([180,-90,0])))
        for d in (10,80,300):
            data['box_clearance']['d_mm']=d
            for advance in (0,50,100,150,200):
                goals=dict((k,p) for k,_,p in compensated_targets(tube_targets(data),rotation,advance))
                g,down,recover,final=(goals[k] for k in ('grasp','tilt_down','recover_retreat','final_retreat'))
                np.testing.assert_allclose(down[:3],g[:3])
                np.testing.assert_allclose(rotation.T @ (np.array(recover[:3])-down[:3]),[-(d-20-advance),0,0],atol=1e-8)
                np.testing.assert_allclose(rotation.T @ (np.array(final[:3])-recover[:3]),[-40,0,0],atol=1e-8)
                np.testing.assert_allclose(transform(recover)[:3,:3], transform(g)[:3,:3],atol=1e-8)
                rel=transform(g)[:3,:3].T @ transform(down)[:3,:3]
                self.assertAlmostEqual(math.degrees(math.acos(np.clip((np.trace(rel)-1)/2,-1,1))),30)
                end=[20,-30,170,10,-5,20]
                tcp=flange_to_tcp(down,{'end':[.02,-.03,.17,*np.radians(end[3:])]})
                np.testing.assert_allclose(transform(tcp) @ np.linalg.inv(transform(end)),transform(down),atol=1e-8)


class SequenceTests(unittest.TestCase):
    def setUp(self):
        self.fixture=motion_fixtures.ExecutionTests();self.fixture.setup_robot('bottle')
        self.addCleanup(self.fixture.doCleanups)
        self.service=self.fixture.service;self.backend=self.fixture.backend
        self.events=self.fixture.f.events;self.audit=self.fixture.audit
        close=self.backend.gripper_start_move
        def preopen(position):
            if position==130:
                self.events.append(('gripper','preopen130'))
                self.fixture.f.gripper.update(requested_position=130,measured_position=130,object_state=3)
            else:close(position)
        self.backend.gripper_start_move=preopen
        self.service.pose_estimator.reproject_latest_tube_to_right_shoulder=lambda a,b:{
            'source_result_id':fixtures.SOURCE,'sku_typ':'tube','shoulder_grasp':shoulder(),
            'world_grasp':{'sku_typ':'tube','grasp_height_trunk_mm':812.9056959008101}}

    def execute(self):
        self.service.grasp_test.execute({'sku_typ':'tube','source_result_id':fixtures.SOURCE,'elbow_deg':0})
        self.service.grasp_test.thread.join(15)
        self.assertFalse(self.service.grasp_test.thread.is_alive())
        return self.service.grasp_test.status()

    def assert_completed(self,result,advance):
        self.assertEqual(result['phase'],'completed',result)
        self.assertEqual(result['advance_mm'],advance)
        self.assertEqual(result['completed_moves'],5)
        self.assertEqual(result['total_moves'],5)
        self.assertEqual(result['arm_retreat_mm'],60-advance)
        stages=[d['stage'] for e,d in self.audit if e=='grasp_test_stage_completed']
        self.assertEqual(stages,STAGES)
        close=self.events.index(('gripper','close'))
        self.assertEqual(self.events[:close].count(('right_arm','start')),3)
        self.assertEqual(self.events.count(('right_arm','start')),5)
        preflight=next(i for i,(e,d) in enumerate(self.audit) if e=='tube_full_preflight_passed')
        dispatch=next(i for i,(e,d) in enumerate(self.audit) if e=='grasp_test_preflight')
        self.assertLess(preflight,dispatch)
        goals=result['targets_flange_mm_deg']
        np.testing.assert_allclose([p[2] for p in goals.values()],0,atol=1e-7)
        np.testing.assert_allclose(goals['recover_retreat'],[-40,0,0,180,-90,0],atol=1e-7)
        np.testing.assert_allclose(goals['final_retreat'],[-80,0,0,180,-90,0],atol=1e-7)
        pairs=[d for e,d in self.audit if e=='grasp_pair_preflight']
        self.assertEqual([p['stage'] for p in pairs],['grasp','recover_retreat','final_retreat'] if advance else ['final_retreat'])
        commands=self.fixture.commands
        self.assertEqual([round(c.target.values[0]*1000-self.fixture.trunk_origin[0]) for c in commands],
                         [advance,0,-100] if advance else [-100])
        self.assertTrue(all(c.speed==77 and abs(c.rotSpeed-math.radians(9))<1e-10 for c in commands))
        self.assertEqual(self.fixture.f.gripper['requested_position'],255)

    def test_no_advance_five_moves_close_after_tilt_final_pair(self):
        self.assert_completed(self.execute(),0)

    def test_A150_selected_before_dispatch_and_returned_during_recovery(self):
        original=HardwareMoveL.plan
        rejected=[]
        def plan(arm,pose,seed,plane):
            # Pose targets are TCP; remove the fake tool's 100mm end.
            flange=transform(pose) @ transform([0,0,-100,0,0,0])
            start_flange=transform(arm.start['poses']['right_arm']) @ transform([0,0,-100,0,0,0])
            if abs(start_flange[0,3]-10)<.01 and any(abs(flange[0,3]-(20-a))<.01 for a in (0,50,100)) and abs(flange[2,3])<.01:
                # Only horizontal grasp candidates (tilt has the same center).
                if np.allclose(flange[:3,:3],transform([0,0,0,180,-90,0])[:3,:3]):
                    rejected.append(flange[0,3]);raise MotionPlanUnavailable('synthetic reach limit')
            return original(arm,pose,seed,plane)
        with patch.object(HardwareMoveL,'plan',plan):result=self.execute()
        self.assert_completed(result,150)
        self.assertEqual(len(rejected),3)  # execution reuses A directly

    def test_final_unreachable_preflight_sends_no_arm_or_body_commands(self):
        original=HardwareMoveL.plan
        def plan(arm,pose,*args):
            flange=transform(pose) @ transform([0,0,-100,0,0,0])
            if abs(flange[0,3]+80)<.01:raise MotionPlanUnavailable('final unreachable')
            return original(arm,pose,*args)
        with patch.object(HardwareMoveL,'plan',plan):result=self.execute()
        self.assertEqual(result['phase'],'failed',result)
        self.assertNotIn(('right_arm','start'),self.events)
        self.assertEqual(self.fixture.commands,[])
        self.assertNotIn(('gripper','close'),self.events)

    def test_preopen_130_before_any_path_planning(self):
        self.fixture.f.gripper.update(requested_position=255,measured_position=200,object_state=2)
        original=HardwareMoveL.plan
        def plan(arm,*args):
            self.assertIn(('gripper','preopen130'),self.events)
            self.assertEqual(self.fixture.f.gripper['requested_position'],130)
            self.assertEqual(self.fixture.f.gripper['measured_position'],130)
            return original(arm,*args)
        # Inspect only the full preflight; the execution necessarily closes later.
        from rokae_web import tube_grasp
        original_preflight=tube_grasp.preflight
        def checked(*args):
            with patch.object(HardwareMoveL,'plan',plan):return original_preflight(*args)
        with patch.object(tube_grasp,'preflight',checked):result=self.execute()
        self.assert_completed(result,0)

    def test_already_at_130_no_repeated_preopen(self):
        self.fixture.f.gripper.update(requested_position=130,measured_position=130,object_state=3)
        result=self.execute()
        self.assert_completed(result,0)
        self.assertNotIn(('gripper','preopen130'),self.events)
        self.assertNotIn(('gripper','open'),self.events)

    def test_blocked_at_130_never_plans_and_unconfirmed_stop_locks(self):
        def blocked(position):
            self.assertEqual(position,130)
            self.fixture.f.gripper.update(requested_position=130,measured_position=180,object_state=1)
        self.backend.gripper_start_move=blocked
        self.backend.gripper_stop=lambda:{'going_to_position':True}
        result=self.execute()
        self.assertEqual(result['phase'],'failed',result)
        self.assertTrue(result['stop_unconfirmed'])
        self.assertFalse(self.service.armed)
        self.assertNotIn(('right_arm','checkPath'),self.events)
        self.assertEqual(self.fixture.commands,[])

    def test_partial_final_pair_failure_stops_both_and_holds_gripper(self):
        def fail(ec):ec.update(ec=10001,message='lost start reply')
        self.backend._robot('trunk').moveStart=fail
        result=self.execute()
        self.assertEqual(result['phase'],'failed',result)
        for module in ('right_arm','trunk'):
            self.assertIn((module,'stop'),self.events)
            self.assertIn((module,'reset'),self.events)
        self.assertEqual(self.fixture.f.gripper['requested_position'],255)

    def test_cancel_at_tilt_arrival_does_not_close_or_retreat(self):
        original=HardwareMoveL.wait_step
        def wait(arm,index):
            original(arm,index)
            if self.service.grasp_test.job['stage']=='tilt_down':self.service.grasp_test.cancel.set()
        with patch.object(HardwareMoveL,'wait_step',wait):result=self.execute()
        self.assertEqual(result['phase'],'cancelled',result)
        self.assertNotIn(('gripper','close'),self.events)
        self.assertEqual(self.fixture.commands,[])


class PoseTests(unittest.TestCase):
    def setUp(self):
        self.f=pose_fixtures.TargetClientTests();self.f.setUp();self.addCleanup(self.f.tearDown)

    def test_tube_geometry_saved_reproject_no_y_offset_quality_flags_ignored(self):
        response=dict(ok=False,sku_typ='tube',edge_valid=False,point_valid=False,
            output_frame='head_camera_color_optical_frame',output_unit='mm',
            top_edge_center_camera_mm=[500,2,500],top_edge_center_chassis_mm=[900,900,900],
            front_panel_valid=False,front_panel_plane_point_camera_mm=[300,0,500],
            front_panel_plane_normal_camera=[-1,0,0],front_panel_top_edge_midpoint_camera_mm=[300,0,600])
        self.f.client._request_json=lambda *a,**k:response
        result=self.f.client.estimate(self.f.snapshot,self.f.state,self.f.state,selected=selection({'target':'tube'}))
        self.assertTrue(result['grasp_test_usable'],result)
        world=result['world_grasp'];height=tube_height({})['pregrasp_flange_height_mm']
        np.testing.assert_allclose(world['grasp_point_trunk_mm'],[500,2,height])
        np.testing.assert_allclose(world['grasp_offset_trunk_mm'],[0,0,0])
        projected=self.f.client.reproject_latest_tube_to_right_shoulder(self.f.state,self.f.state)
        self.assertEqual(projected['sku_typ'],'tube')
        np.testing.assert_allclose(projected['shoulder_grasp']['grasp_pose_right_shoulder_mm_deg'][:3],[325,2,height])
        self.assertEqual(projected['world_grasp']['tube_height_calibration'],tube_height({}))
        with self.assertRaisesRegex(Exception,'其他类别'):
            self.f.client.reproject_latest_to_right_shoulder(self.f.state,self.f.state)

    def test_tube_height_required_and_right_side_fixed(self):
        self.f.client.config['tube_grasp_file']='/nonexistent/tube-height.json'
        with self.assertRaisesRegex(Exception,'软管高度标定'):
            self.f.client._height_request({'sku_typ':'tube'})
        with self.assertRaisesRegex(ValueError,'RIGHT'):
            selection({'target':'tube','side':'LEFT'})


if __name__=='__main__':unittest.main()
