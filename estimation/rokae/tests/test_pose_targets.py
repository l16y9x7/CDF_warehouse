import copy
import json
from pathlib import Path
import tempfile
from types import SimpleNamespace
import unittest
import numpy as np
from rokae_web.backends import MockRobotBackend, MockChassisBackend
from rokae_web.camera import CameraSnapshot
from rokae_web.config import DEFAULT_CONFIG
from rokae_web.pose_targets import interpret, selection, basket_right_shoulder, basket_left_shoulder, TARGETS
from rokae_web.pose_protocol import sku_type
from rokae_web.pose_estimation import PoseEstimationClient, PoseEstimationError
from rokae_web.service import ControlService, ValidationError


def response(target):
    if target == 'bottle':
        return dict(ok=True, sku_typ='bottle', axis_fit_valid=True, reference_point_valid=True,
                    reference_mode='visible_axis_midpoint', reference_point_camera_mm=[1, 2, 500],
                    axis_point_camera_mm=[1, 2, 490], axis_direction_camera_up=[0, 0, 1])
    if target == 'box':
        return dict(ok=True, sku_typ='box', top_plane_valid=True, top_point_valid=True, top_point_camera_mm=[1, 2, 500])
    if target == 'tube':
        return dict(ok=True, sku_typ='tube', edge_valid=True, point_valid=True, point_semantics='visible_top_edge_midpoint',
                    top_edge_center_camera_mm=[1, 2, 500], top_edge_endpoints_camera_mm=[[0,2,500],[2,2,500]],
                    edge_direction_camera=[1,0,0])
    matrix = np.eye(4)
    matrix[:3,3] = [100,200,500]
    return dict(ok=True, pose_valid=True, point_semantics='basket_model_center',
                model_center_camera_mm=[1,2,500], pose_4x4=matrix.tolist(), rotation_euler_zyx_rad=[0,0,0])


class TargetContractTests(unittest.TestCase):
    def test_basket_center_shoulder_rotation_translation_and_units(self):
        localization=interpret(response('basket'),'basket',np.eye(4))
        shoulder=np.array([[0,-1,0,.1],[1,0,0,.2],[0,0,1,.3],[0,0,0,1.]])
        out=basket_right_shoulder(localization,shoulder)
        np.testing.assert_allclose(out['point_right_shoulder_mm'],[-198,99,200])
        self.assertEqual(out['right_shoulder_frame'],'right_arm_sdk_world')
        # Round trip recovers the center, not the CAD origin or a flange offset.
        point=shoulder[:3,:3] @ out['point_right_shoulder_mm'] + shoulder[:3,3]*1000
        np.testing.assert_allclose(point,[1,2,500])
        for bad in (np.zeros((4,4)),np.diag([-1,1,1,1]),np.full((4,4),np.nan)):
            with self.assertRaises(ValueError):basket_right_shoulder(localization,bad)
        for target in TARGETS:
            other=interpret(response(target),target,np.eye(4))
            if target=='basket':other['valid']=False
            with self.assertRaises(ValueError):basket_right_shoulder(other,shoulder)

    def test_box_failure_explains_missing_pair_without_coordinates(self):
        out=interpret(dict(ok=False,box_selection={'reason':'no_separable_left_right_box_pair'}),'box',np.eye(4))
        self.assertFalse(out['valid'])
        self.assertIsNone(out['point_camera_mm'])

    def test_each_class_has_its_own_gate_and_point(self):
        transform=np.eye(4)
        transform[:3,3]=[.1,.2,.3]
        for target in TARGETS:
            with self.subTest(target=target):
                out=interpret(response(target),target,transform)
                self.assertTrue(out['valid'], out)
                self.assertEqual(out['point_camera_mm'],[1,2,500])
                self.assertEqual(out['point_chassis_mm'],[101,202,800])
        basket=interpret(response('basket'),'basket',transform)
        self.assertNotEqual(basket['point_camera_mm'],basket['cad_origin_camera_mm'])

    def test_invalid_and_cross_class_results_never_reuse_points(self):
        cases=[('bottle',{'reference_point_camera_mm':None}),
               ('basket',{'pose_4x4':[[0]*4]*4}),('box',{'top_point_camera_mm':[float('nan'),2,3]}),
               ('bottle',{'sku_typ':'box'}),('bottle',{'output_unit':'m'})]
        for target, change in cases:
            raw=response(target);raw.update(change)
            out=interpret(raw,target,np.eye(4))
            self.assertFalse(out['valid'],out)
            self.assertIsNone(out['point_camera_mm'])
        self.assertFalse(interpret(response('bottle'),'box',np.eye(4))['valid'])

    def test_selection_rejects_arbitrary_inputs(self):
        for payload in ({'target':'other'},{'target':'bottle','side':'front'},
                        {'target':'basket','url':'http://other'}, {'target':'bottle','camera':'left_wrist'}):
            with self.assertRaises(ValueError): selection(payload)


class TargetClientTests(unittest.TestCase):
    def setUp(self):
        self.tmp=tempfile.TemporaryDirectory()
        cfg=copy.deepcopy(DEFAULT_CONFIG)
        cfg['camera']['data_directory']=self.tmp.name
        self.client=PoseEstimationClient(cfg)
        self.client._kinematics=SimpleNamespace(forward_deg=lambda *a,**k:np.eye(4),
                                               right_shoulder_sdk_world=lambda *a:np.eye(4),
                                               left_shoulder_sdk_world=lambda *a:np.eye(4))
        self.client._load_calibration=lambda:(np.array([[2,0,1],[0,2,1],[0,0,1]]),np.eye(4),[3,2])
        self.client._synchronized_transform=lambda *a:(np.eye(4),[0.]*6,0.)
        self.client.health=lambda *a:{'available':True}
        self.snapshot=CameraSnapshot(rgb_bgr=np.zeros((2,3,3),dtype=np.uint8),
              depth_aligned_mm=np.full((2,3),500,dtype=np.float32), captured_at='test',
              color_timestamp_ms=1,depth_timestamp_ms=1,sequence=7,camera_info={})
        self.state=MockRobotBackend().read_state()

    def tearDown(self):self.tmp.cleanup()

    def test_basket_uses_capture_trunk_and_persists_shoulder_point(self):
        captured=[10.,20.,30.,40.,50.,60.]
        self.client._synchronized_transform=lambda *a:(np.eye(4),captured,0.)
        seen=[]
        def shoulder(joints):
            seen.append(joints)
            return np.array([[0,-1,0,.1],[1,0,0,.2],[0,0,1,.3],[0,0,0,1.]])
        self.client._kinematics.right_shoulder_sdk_world=shoulder
        left_seen=[]
        def left_shoulder(joints):
            left_seen.append(joints)
            transform=np.array([[0,-1,0,.1],[1,0,0,-.2],[0,0,1,.3],[0,0,0,1.]])
            return transform
        self.client._kinematics.left_shoulder_sdk_world=left_shoulder
        self.client._request_json=lambda *a,**k:response('basket')
        out=self.client.estimate(self.snapshot,self.state,self.state,selected=selection({'target':'basket'}))
        self.assertTrue(out['usable'],out)
        self.assertEqual(seen,[captured[:4]])
        np.testing.assert_allclose(out['localization']['point_right_shoulder_mm'],[-198,99,200])
        self.assertEqual(left_seen,[captured[:4]])
        np.testing.assert_allclose(out['localization']['point_left_shoulder_mm'],[202,99,200])
        self.assertEqual(out['localization']['left_shoulder_frame'],'left_arm_sdk_world')
        saved=json.loads((Path(out['directory'])/'pose_estimation_summary.json').read_text())
        self.assertEqual(saved['localization'],out['localization'])
        self.client._request_json=lambda *a,**k:dict(ok=True,pose_valid=False)
        out=self.client.estimate(self.snapshot,self.state,self.state,selected=selection({'target':'basket'}))
        self.assertFalse(out['usable'])
        self.assertNotIn('point_right_shoulder_mm',out['localization'])
        self.assertNotIn('point_left_shoulder_mm',out['localization'])
        self.assertEqual(len(left_seen),1)
        self.assertEqual(len(seen),1)

    def test_requests_and_saved_results_for_all_four_targets(self):
        before=copy.deepcopy(self.client.config)
        for target in TARGETS:
            sent={}
            def request(url,**kwargs):
                sent.update(kwargs['payload'])
                return response(target)
            self.client._request_json=request
            result=self.client.estimate(self.snapshot,self.state,self.state,selected=selection({'sku_typ':target,'side':'RIGHT' if target in ('bottle','tube') else 'LEFT'}))
            self.assertTrue(result['usable'],result['message'])
            self.assertEqual(result['localization']['target'],target)
            self.assertEqual(sent['camera_frame'],'head_camera_color_optical_frame')
            self.assertEqual(sent['depth_unit'],'mm')
            self.assertTrue(sent['rgb_base64'] and sent['depth_npy_base64'])
            self.assertNotIn('body_radius_mm',sent)
            self.assertNotIn('z_ref_mm',sent)
            if target=='basket':
                self.assertEqual(sent['target_type'],'basket')
                for key in ('sku_typ','sku_id','side','class_name','front_rule'):self.assertNotIn(key,sent)
            else:
                self.assertEqual(sent['target_type'],'sku')
                self.assertEqual(sent['sku_typ'],sku_type(target))
                self.assertNotIn('sku_id',sent)
                self.assertNotIn('class_name',sent)
                self.assertEqual(sent['side'],'RIGHT' if target in ('bottle','tube') else 'LEFT')
            if target!='bottle':
                self.assertIsNone(result['world_grasp'])
                self.assertFalse(result['grasp_test_usable'])
                with self.assertRaises(PoseEstimationError):self.client.latest_world_grasp()
            summary=json.loads((Path(result['directory'])/'pose_estimation_summary.json').read_text())
            self.assertEqual(summary['selected_target'],target)
            self.assertEqual(summary['localization'],result['localization'])
        self.assertEqual(self.client.config,before)

    def test_selected_avene_grasp_and_saved_replay_use_axis_without_radius(self):
        self.client.config['grasp_height_trunk_mm_by_sku']={'bottle':812.0}
        reply=dict(ok=True,sku_typ='bottle',axis_fit_valid=True,reference_point_valid=True,
            output_frame='head_camera_color_optical_frame',output_unit='mm',
            reference_mode='visible_axis_midpoint',axis_point_camera_mm=[500,2,500],
            axis_direction_camera_up=[0,0,1],reference_point_camera_mm=[500,2,500],
            reference_point_chassis_mm=[500,2,500],front_panel_valid=True,
            front_panel_plane_point_camera_mm=[300,0,500],front_panel_plane_normal_camera=[-1,0,0],
            front_panel_top_edge_midpoint_camera_mm=[300,0,600])
        sent={}
        def infer(url,**kwargs):
            sent.update(kwargs['payload'])
            return reply
        self.client._request_json=infer
        result=self.client.estimate(self.snapshot,self.state,self.state,selected=selection({'target':'bottle'}))
        self.assertTrue(result['grasp_test_usable'],result['message'])
        self.assertNotIn('body_radius_mm',sent)
        self.assertNotIn('body_radius_mm',reply)
        np.testing.assert_allclose(result['world_grasp']['axis_intersection_trunk_mm'],[500,2,812])
        np.testing.assert_allclose(result['world_grasp']['grasp_point_trunk_mm'],[500,12,812])
        self.assertEqual(self.client.latest_world_grasp()['world_grasp'],result['world_grasp'])

    def test_invalid_response_stays_invalid_and_saved(self):
        self.client._request_json=lambda *a,**k:dict(ok=True,top_plane_valid=False,top_point_valid=False)
        result=self.client.estimate(self.snapshot,self.state,self.state,selected=selection({'target':'box'}))
        self.assertFalse(result['usable'])
        self.assertFalse(result['grasp_test_usable'])
        self.assertTrue(Path(result['directory']).exists())

    def test_service_uses_head_for_every_button_and_does_not_move(self):
        captured=[]
        class Camera:
            externally_managed=True
            def status(self):return {'head':{'available':True,'enabled':True}}
            def fresh_snapshot(inner,name,timeout):
                captured.append(name);return self.snapshot
            def close(self):pass
        robot=MockRobotBackend()
        start=robot.read_state()
        seen=[]
        def estimate(*args,selected):
            seen.append(selected)
            return dict(status='invalid',usable=False,result_id='test',relative_directory='test',elapsed_seconds=0)
        estimator=SimpleNamespace(fresh_frame_timeout=1,estimate=estimate)
        service=ControlService(copy.deepcopy(DEFAULT_CONFIG),robot,MockChassisBackend(),False,
                               camera_backend=Camera(),pose_estimator=estimator)
        try:
            for target in TARGETS:service.estimate_grasp_object_pose({'target':target})
            self.assertEqual(captured,['head']*4)
            self.assertEqual([s['target'] for s in seen],list(TARGETS))
            self.assertEqual(robot.read_state(),start)
            with self.assertRaises(ValidationError):service.estimate_grasp_object_pose({'target':'bad'})
        finally:service.close()
