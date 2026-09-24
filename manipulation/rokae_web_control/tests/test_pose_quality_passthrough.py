"""Returned coordinates remain usable independently of upstream quality flags."""
import copy
import unittest
import numpy as np

from rokae_web.pose_targets import interpret, TARGETS, selection
from rokae_web.grasp_pose import world_grasp_from_4090
from rokae_web.box_clearance import front_panel_from_response
import test_pose_targets as target_fixtures
import test_grasp_pose as grasp_fixtures
import test_box_clearance as panel_fixtures


class QualityPassthroughTests(unittest.TestCase):
    def test_all_categories_use_coordinates_despite_false_or_missing_quality_flags(self):
        for target in TARGETS:
            raw=target_fixtures.response(target)
            expected=interpret(raw,target,np.eye(4))
            for remove in [False,True]:
                candidate=copy.deepcopy(raw)
                for key in list(candidate):
                    if key=='ok' or key.endswith('_valid'):
                        if remove:candidate.pop(key)
                        else:candidate[key]=False
                candidate['rejection_reasons']=['deliberate upstream rejection']
                candidate['sam3_score']=0
                with self.subTest(target=target,remove=remove):
                    self.assertEqual(interpret(candidate,target,np.eye(4)),expected)

    def test_grasp_ignores_quality_and_duplicate_world_point_difference(self):
        request,raw,_,_=grasp_fixtures.GraspPoseTests().axis_fixture()
        expected=world_grasp_from_4090(request,raw)['grasp_pose_world_mm_deg']
        raw.update(ok=False,axis_fit_valid=False,reference_point_valid=False,
                   reference_point_chassis_mm=[0,0,0])
        result=world_grasp_from_4090(request,raw)
        self.assertEqual(result['grasp_pose_world_mm_deg'],expected)
        self.assertGreater(result['reference_consistency_error_mm'],5)
        raw['reference_point_camera_mm']=None
        with self.assertRaises(ValueError):world_grasp_from_4090(request,raw)

    def test_panel_ignores_quality_and_duplicate_coordinates_and_normalizes_normal(self):
        # Standalone geometry fixture needs no hardware.
        request=dict(base_frame='chassis_link',T_unit='m',camera_frame='camera',T_chassis_camera=np.eye(4).tolist())
        raw=dict(output_frame='camera',output_unit='mm',front_panel_valid=False,
                 front_panel_plane_point_camera_mm=[400,0,0],front_panel_plane_normal_camera=[-2,0,0],
                 front_panel_top_edge_midpoint_camera_mm=[400,0,700],
                 front_panel_plane_point_chassis_mm=[9999,9999,9999])
        result=front_panel_from_response(request,raw)
        self.assertEqual(result['point_mm'],[400,0,0]);self.assertEqual(result['normal'],[-1,0,0])
        raw['front_panel_plane_point_camera_mm']=None
        with self.assertRaises(ValueError):front_panel_from_response(request,raw)

    def test_web_estimation_and_reload_accept_false_bottle_flags(self):
        f=target_fixtures.TargetClientTests();f.setUp();self.addCleanup(f.tearDown)
        f.client.config['grasp_height_trunk_mm']=800.0
        raw=target_fixtures.response('bottle')
        raw.update(ok=False,axis_fit_valid=False,reference_point_valid=False,
                   reference_point_camera_mm=[500,0,700],
                   output_frame=f.client.CAMERA_FRAME,output_unit='mm',front_panel_valid=False,
                   front_panel_plane_point_camera_mm=[300,0,0],front_panel_plane_normal_camera=[1,0,0],
                   front_panel_top_edge_midpoint_camera_mm=[300,0,600])
        f.client._request_json=lambda *a,**k:copy.deepcopy(raw)
        out=f.client.estimate(f.snapshot,f.state,f.state,selected=selection({'sku_typ':'bottle'}))
        self.assertTrue(out['grasp_test_usable'],out)
        saved=f.client.reproject_latest_to_right_shoulder(f.state,f.state)
        self.assertEqual(saved['source_result_id'],out['result_id'])
        # The legacy estimate route also consumes the same raw point.
        legacy=f.client.estimate(f.snapshot,f.state,f.state)
        self.assertTrue(legacy['grasp_test_usable'],legacy)
