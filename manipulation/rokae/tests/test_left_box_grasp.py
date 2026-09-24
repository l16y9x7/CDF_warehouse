"""Left-box geometry and motion routing, using SDK doubles only."""
import copy
import json
from pathlib import Path
import unittest
from unittest.mock import patch
import numpy as np
from rokae_web.backends import BackendError
from rokae_web.grasp_pose import world_grasp_from_4090, world_grasp_to_left_shoulder
from rokae_web.grasp_test import left_flange_targets
from rokae_web.head_kinematics import rpy_rotation, UpperBodySixDofKinematics
from rokae_web.box_clearance import clearance_in_trunk
from rokae_web.arm_movel import transform
from rokae_web.config import DEFAULT_CONFIG
from rokae_web.pose_estimation import PoseEstimationError
from rokae_web.pose_targets import selection
import test_grasp_pose as geometry_fixtures
import test_grasp_test as sequence_fixtures
import test_pose_targets as target_fixtures
SOURCE = sequence_fixtures.SOURCE
CALIBRATION = dict(version=1, frame='trunk_controller_ref', unit='mm', heights_mm=dict(grasp=800., descend=770., lift=890.))


def box_geometry():
    request = dict(sku_typ='box', base_frame='chassis_link', camera_frame='camera', T_unit='m',
                   T_chassis_camera=np.eye(4).tolist(), T_chassis_trunk_ref_m=np.eye(4).tolist(),
                   grasp_height_trunk_mm=800, box_height_calibration=copy.deepcopy(CALIBRATION))
    raw = dict(ok=True, top_plane_valid=True, top_point_valid=True,
               top_point_camera_mm=[500, 100, 700], output_frame='camera', output_unit='mm')
    world = world_grasp_from_4090(request, raw)
    panel = dict(valid=True, frame='chassis_link', point_mm=[300,0,0], normal=[1,0,0])
    clearance = clearance_in_trunk(panel, world, np.eye(4))
    shoulder = world_grasp_to_left_shoulder(world, np.eye(4), clearance, np.eye(4))
    return world, shoulder


class LeftGeometryTests(unittest.TestCase):
    def test_rotated_trunk_camera_and_shoulder_only_change_trunk_z_on_descent(self):
        request, raw, trunk, camera = geometry_fixtures.GraspPoseTests().axis_fixture()
        request['sku_typ'] = 'box'
        request['box_height_calibration'] = copy.deepcopy(CALIBRATION)
        raw.update(top_plane_valid=True, top_point_valid=True,
                   top_point_camera_mm=raw['reference_point_camera_mm'])
        # Decoy axis and reference fields must not affect box top-point geometry.
        raw['axis_point_camera_mm'] = [9999,9999,9999]
        raw['reference_point_camera_mm'] = [-9999,-9999,-9999]
        raw['axis_direction_camera_up'] = [float('nan')]*3
        world = world_grasp_from_4090(request, raw)
        np.testing.assert_allclose(world['grasp_point_trunk_mm'], [500,-100,800], atol=1e-8)
        np.testing.assert_allclose(world['axis_direction_trunk'], [0,0,1])
        self.assertAlmostEqual(world['recognition_height_trunk_mm'],720)
        panel = dict(valid=True,frame='chassis_link',
                     point_mm=(trunk[:3,:3] @ np.array([400,0,0])+trunk[:3,3]*1000).tolist(),
                     normal=trunk[:3,0].tolist())
        clearance = clearance_in_trunk(panel,world,trunk)
        self.assertAlmostEqual(clearance['d_mm'],100)
        self.assertAlmostEqual(clearance['pregrasp_virtual_length_mm'],330)
        shoulder = np.eye(4)
        shoulder[:3,:3]=rpy_rotation([.2,-.4,-.8]);shoulder[:3,3]=[.13,.24,.9]
        result = world_grasp_to_left_shoulder(world,shoulder,clearance,trunk)
        expected={'pregrasp':[170,-100,800],'grasp':[335,-100,800],'descend':[335,-100,770],'lift':[335,-100,890]}
        for key, xyz in expected.items():
            pose=result[key+'_pose_left_shoulder_mm_deg']
            in_world=shoulder[:3,:3]@np.array(pose[:3])+shoulder[:3,3]*1000
            in_trunk=trunk[:3,:3].T@(in_world-trunk[:3,3]*1000)
            np.testing.assert_allclose(in_trunk,xyz,atol=1e-8)
            np.testing.assert_allclose(shoulder[:3,:3]@rpy_rotation(np.radians(pose[3:])),
                trunk[:3,:3]@rpy_rotation(np.radians([180,-90,0])),atol=1e-8)
        self.assertEqual(result['gripper_length_mm'],165)
        targets={key:pose for key,_,pose in left_flange_targets(result)}
        delta=np.asarray(targets['arm_retreat'][:3])-targets['lift'][:3]
        np.testing.assert_allclose(trunk[:3,:3].T @ shoulder[:3,:3] @ delta,[-130,0,0],atol=1e-8)

    def test_reference_point_used_for_bottle_even_if_axis_point_differs(self):
        request,raw,_,_=geometry_fixtures.GraspPoseTests().axis_fixture()
        raw['axis_point_camera_mm']=[1e6]*3
        raw['axis_direction_camera_up']=None
        point=world_grasp_from_4090(request,raw)
        np.testing.assert_allclose(point['grasp_point_trunk_mm'],[500,-90,800],atol=1e-8)

    def test_recognition_height_does_not_change_fixed_descent_or_lift(self):
        world, shoulder=box_geometry()
        world['recognition_height_trunk_mm']=99999
        result=world_grasp_to_left_shoulder(world,np.eye(4),shoulder['box_clearance'],np.eye(4))
        self.assertEqual(result['descend_height_trunk_mm'],770)
        self.assertEqual(result['lift_height_trunk_mm'],890)
        world['box_height_calibration']['heights_mm']['descend']=900
        with self.assertRaisesRegex(ValueError,'顺序'):
            world_grasp_to_left_shoulder(world,np.eye(4),shoulder['box_clearance'],np.eye(4))

    def test_real_left_sdk_shoulder_uses_left_origin_and_chest_axes(self):
        path=DEFAULT_CONFIG['pose_estimation']['urdf_file']
        if not Path(path).is_file():self.skipTest('URDF only on robot')
        kin=UpperBodySixDofKinematics(path)
        for q in ([0]*4,[-10,15,-20,35]):
            chest=kin.forward_deg([*q,0,0],tip_link='Chest_link')
            left=kin.left_shoulder_sdk_world(q); right=kin.right_shoulder_sdk_world(q)
            np.testing.assert_allclose(left[:3,:3],chest[:3,:3],atol=1e-9)
            np.testing.assert_allclose(chest[:3,:3].T@(left[:3,3]-right[:3,3]),[0,.155,0],atol=1e-9)


class LeftSequenceTests(unittest.TestCase):
    def setUp(self):
        self.f=sequence_fixtures.SequenceTests(); self.f.setUp(); self.addCleanup(self.f.doCleanups)
        self.service=self.f.service
        self.world,self.shoulder=box_geometry()
        self.f.estimator.reproject_latest_to_left_shoulder=lambda a,b:dict(
            sku_typ='box',source_result_id=SOURCE,world_grasp=copy.deepcopy(self.world),
            shoulder_grasp=copy.deepcopy(self.shoulder),current_trunk_joints_deg=a['joints_deg']['trunk'])
        self.service.robot.gripper_status=lambda: self.fail('left flow must not read gripper')
        self.service.robot.gripper_start_move=lambda *a:self.fail('left flow must not close gripper')
        self.service.config['suction'] = dict(enabled=True,slave_id=1,open_outputs=[dict(address=0,value=True)],close_outputs=[dict(address=0,value=False)])
        self.suction=[]
        def suction(opened,cfg):
            self.suction.append((opened,self.f.events.count(('left_arm','start'))))
            return dict(commanded_open=opened,confirmed=True)
        self.service.robot.suction_set=suction
        self.before=copy.deepcopy(self.f.backend.read_state())
        self.f.events.clear()

    def execute(self):
        self.service.grasp_test.execute(dict(sku_typ='box',source_result_id=SOURCE,elbow_deg=0))
        self.service.grasp_test.thread.join(5)
        self.assertFalse(self.service.grasp_test.thread.is_alive())
        return self.service.grasp_test.status()

    def test_five_left_moves_suction_then_trunk_fresh_angles_speed_and_actual_tcp(self):
        cls=self.f.fixture.executor.__class__;original_plan=cls.plan;original_wait=cls.wait_step;dispatch=cls.start_step
        seeds=[];speeds=[]
        def plan(executor,pose,seed,plane):
            self.assertEqual(executor.module,'left_arm');self.assertIsNotNone(plane)
            seeds.append(seed)
            return original_plan(executor,pose,seed+5,plane)
        def wait(executor,index):
            original_wait(executor,index)
            self.f.fixture.q['left_arm'][6]+=.07
        def start(executor,index,speed,rotation):
            speeds.append((speed,rotation));dispatch(executor,index,speed,rotation)
            self.service.set_speed(dict(speed_mm_s=100,rotation_deg_s=12))
        with patch.object(cls,'plan',plan),patch.object(cls,'wait_step',wait),patch.object(cls,'start_step',start):
            status=self.execute()
        self.assertEqual(status['phase'],'completed',status)
        self.assertEqual(status['total_moves'],6);self.assertEqual(status['completed_moves'],6)
        self.assertEqual(self.suction,[(False,0),(True,3)])
        np.testing.assert_allclose(seeds,[0,5.07,10.14,15.21,20.28],atol=1e-5)
        self.assertEqual(speeds,[(77,9)]*5)
        self.assertEqual([e for e in self.f.events if e[1] in ('start','close','preflight')],[('trunk','preflight')]+[('left_arm','start')]*5+[('trunk_retreat','start')])
        self.assertEqual(self.f.events.count(('left_arm','checkPath')),5)
        after=self.f.backend.read_state()
        for module in ('right_arm','head'):
            self.assertEqual(after['joints_deg'][module],self.before['joints_deg'][module])
        # A real configured tool offset means flange height != TCP height.
        tcp=transform(after['poses']['left_arm'])
        flange=tcp@np.linalg.inv(transform([0,0,100,0,0,0]))
        np.testing.assert_allclose(flange[:3,3],[105,100,890],atol=1e-6)
        self.assertEqual(status['arm_retreat_mm'],230)

    def test_wrong_category_or_changed_source_never_moves(self):
        original=self.f.estimator.reproject_latest_to_left_shoulder
        for change in (dict(sku_typ='bottle'),dict(source_result_id='new')):
            self.f.estimator.reproject_latest_to_left_shoulder=lambda a,b:dict(original(a,b),**change)
            result=self.execute()
            self.assertEqual(result['phase'],'failed',result)
            self.assertFalse(any(e[1]=='start' for e in self.f.events))

    def test_cancel_stops_left_and_does_not_start_next_stage(self):
        def wait(executor,index):
            self.service.grasp_test.cancel.set();raise BackendError('cancelled')
        with patch('rokae_web.grasp_test.HardwareMoveL.wait_step',wait):status=self.execute()
        self.assertEqual(status['phase'],'cancelled',status)
        self.assertEqual(self.f.events.count(('left_arm','start')),1)
        self.assertIn(('left_arm','stop'),self.f.events)
        self.assertNotIn(('right_arm','stop'),self.f.events)

    def test_descent_failure_or_other_arm_drift_stops_later_commands(self):
        cls=self.f.fixture.executor.__class__;original=cls.wait_step
        def wait(executor,index):
            original(executor,index);self.f.fixture.q['right_arm'][0]+=1
        with patch.object(cls,'wait_step',wait):status=self.execute()
        self.assertEqual(status['phase'],'failed',status)
        self.assertEqual(self.f.events.count(('left_arm','start')),1)

    def test_missing_or_unsupported_type_rejected_before_dispatch(self):
        for kind in (None,'Avene','estee','basket'):
            with self.assertRaises(BackendError):
                self.service.grasp_test.execute(dict(sku_typ=kind,source_result_id=SOURCE))
        self.assertFalse(any(e[1]=='start' for e in self.f.events))


class LeftSavedResultTests(unittest.TestCase):
    def test_saved_box_reprojects_latest_with_shared_plane_and_no_stale_fallback(self):
        f=target_fixtures.TargetClientTests();f.setUp();self.addCleanup(f.tearDown)
        f.client.config['grasp_height_trunk_mm']=500.0  # bottle plane must not leak into box
        calibration_path=f.client.data_root.parent/'box_grasp.json'
        calibration_path.write_text(json.dumps(CALIBRATION))
        f.client.config['box_grasp_file']=str(calibration_path)
        f.client._kinematics.left_shoulder_sdk_world=lambda q:np.eye(4)
        raw=target_fixtures.response('box')
        raw.update(output_frame=f.client.CAMERA_FRAME,output_unit='mm',top_point_camera_mm=[500,100,700],
                   front_panel_valid=True,front_panel_plane_point_camera_mm=[300,0,500],
                   front_panel_plane_normal_camera=[-1,0,0],front_panel_top_edge_midpoint_camera_mm=[300,0,600])
        f.client._request_json=lambda *a,**k:raw
        out=f.client.estimate(f.snapshot,f.state,f.state,selected=selection(dict(sku_typ='box')))
        self.assertTrue(out['grasp_test_usable'],out)
        projected=f.client.reproject_latest_to_left_shoulder(f.state,f.state)
        self.assertEqual(projected['source_result_id'],out['result_id'])
        self.assertEqual(projected['sku_typ'],'box')
        np.testing.assert_allclose(projected['shoulder_grasp']['descend_pose_left_shoulder_mm_deg'][:3],[335,100,770])
        with self.assertRaisesRegex(PoseEstimationError,'其他类别'):
            f.client.reproject_latest_to_right_shoulder(f.state,f.state)
        raw['top_point_valid']=False
        invalid=f.client.estimate(f.snapshot,f.state,f.state,selected=selection(dict(sku_typ='box')))
        self.assertTrue(invalid['grasp_test_usable'])
        self.assertEqual(f.client.reproject_latest_to_left_shoulder(f.state,f.state)['source_result_id'], invalid['result_id'])


if __name__=='__main__':unittest.main()
