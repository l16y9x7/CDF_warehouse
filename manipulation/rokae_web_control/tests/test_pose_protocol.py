"""New vision wire contract, with historical replay isolated from fresh input."""
import copy
import json
from pathlib import Path
import unittest
from unittest.mock import Mock
from types import SimpleNamespace
import numpy as np
from rokae_web.pose_protocol import sku_type, local_target, validate_response_type
from rokae_web.pose_targets import selection, interpret
from rokae_web.agent_geometry import AgentGeometry
from rokae_web.backends import BackendError
from rokae_web.pose_estimation import PoseEstimationError
import test_pose_targets
from test_pose_targets import response


class ProtocolTests(unittest.TestCase):
    def client_fixture(self):
        case=test_pose_targets.TargetClientTests()
        case.setUp()
        self.addCleanup(case.tearDown)
        return case

    def test_categories_are_canonical_and_old_public_names_rejected(self):
        for old,new in [('Avene','bottle'),('estee','box'),('origins','tube')]:
            with self.assertRaises(ValueError): sku_type(old)
            with self.assertRaises(ValueError): selection({'target':old})
            self.assertEqual(sku_type(new),new)
            self.assertEqual(local_target(new),new)
            self.assertEqual(selection({'sku_typ':new})['target'],new)
        self.assertEqual(selection({'sku_typ':'box'})['side'],'LEFT')
        self.assertEqual(selection({'sku_typ':'bottle'})['side'],'RIGHT')
        for payload in ({'sku_typ':'bottle','side':'LEFT'}, {'sku_typ':'box','side':'RIGHT'},
                        {'sku_id':'Avene'}, {'target':'box','sku_typ':'bottle'}):
            with self.assertRaises(ValueError): selection(payload)
        for unknown in (None,{},'Bottle','not-a-category'):
            with self.assertRaises(ValueError):sku_type(unknown)

    def test_health_uses_new_supported_types_and_rejects_missing_or_mismatch(self):
        f=self.client_fixture()
        # setUp stubs health; bind the actual implementation for this test.
        from rokae_web.pose_estimation import PoseEstimationClient
        health=PoseEstimationClient.health.__get__(f.client)
        f.client._request_json=lambda *a,**k:dict(ok=True,supported_sku_types=['bottle','box','tube'])
        for target in ('bottle','box','tube','basket'):
            self.assertTrue(health(target)['available'])
        for body in (dict(ok=True,supported_skus=['Avene']),dict(ok=True,supported_sku_types=['box']),dict(ok=False,supported_sku_types=['bottle'])):
            f.client._request_json=lambda *a,**k:body
            self.assertFalse(health('bottle')['available'])

    def test_response_requires_new_field_and_consistent_redundant_name(self):
        good=dict(ok=True,sku_typ='bottle',class_name='bottle')
        validate_response_type(good,'bottle')
        for bad in (dict(ok=True),dict(ok=True,sku_id='Avene'),
                    dict(good,sku_typ='box'),dict(good,class_name='Avene'),
                    dict(good,sku_id='Avene'),dict(good,target_type='basket')):
            with self.subTest(bad=bad),self.assertRaises(ValueError):
                validate_response_type(bad,'bottle')
        validate_response_type({'sku_id':'Avene'},'bottle',historical=True)
        validate_response_type({},'bottle',historical=True)
        with self.assertRaises(ValueError):
            validate_response_type({'sku_id':'estee'},'bottle',historical=True)

    def test_agent_rejects_old_or_wrong_visual_type_before_any_state_read(self):
        robot=SimpleNamespace(read_memory_state=Mock(side_effect=AssertionError('must not read/move')))
        geo=AgentGeometry(SimpleNamespace(robot=robot,pose_estimator=SimpleNamespace()))
        for raw in ({'sku_id':'Avene'}, {'sku_typ':'box'}, {'sku_typ':'bottle','class_name':'tube'}):
            with self.assertRaises(BackendError):geo.freeze(raw,{})
        robot.read_memory_state.assert_not_called()

    def test_no_old_wire_fields_and_visualization_opt_out(self):
        f=self.client_fixture()
        f.client.config['return_visualizations']=False
        for target in ('bottle','box','tube','basket'):
            sent={}
            def infer(url,**kw):
                sent.update(kw['payload'])
                return response(target)
            f.client._request_json=infer
            result=f.client.estimate(f.snapshot,f.state,f.state,selected=selection({'target':target}))
            self.assertTrue(result['usable'],result)
            for key in ('sku_id','class_name','z_ref_mm'):
                self.assertNotIn(key,sent)
            self.assertIs(sent['return_visualizations'],False)
            if target=='basket':self.assertNotIn('sku_typ',sent)
            else:self.assertEqual(sent['sku_typ'],sku_type(target))

    def test_mislabeled_response_never_produces_coordinates(self):
        for target, wrong in [('bottle','box'),('box','tube'),('tube','bottle')]:
            raw=response(target);raw['sku_typ']=wrong
            out=interpret(raw,target,np.eye(4))
            self.assertFalse(out['valid'])
            self.assertIsNone(out['point_camera_mm'])

    def test_new_and_historical_height_metadata_use_existing_calibration(self):
        f=self.client_fixture()
        f.client.config['grasp_height_trunk_mm_by_sku']={'Avene':792.90086}
        requests=({'sku_typ':'bottle'},{'sku_id':'Avene'},{'class_name':'Avene'})
        for request in requests:
            original=copy.deepcopy(request)
            out=f.client._height_request(request,np.eye(4))
            self.assertEqual(out['grasp_height_trunk_mm'],792.90086)
            self.assertEqual(out['grasp_height_sku_typ'],'bottle')
            self.assertEqual(request,original)
        with self.assertRaises(PoseEstimationError):f.client._height_request({'sku_typ':'tube'})

    def test_saved_v1_record_replays_without_rewriting_and_new_records_stay_strict(self):
        f=self.client_fixture()
        f.client.config['grasp_height_trunk_mm_by_sku']={'Avene':792.90086}
        raw=response('bottle')
        raw.update(output_frame='head_camera_color_optical_frame',output_unit='mm',
            axis_point_camera_mm=[500,2,500],reference_point_camera_mm=[500,2,500],
            reference_point_chassis_mm=[500,2,500],front_panel_valid=True,
            front_panel_plane_point_camera_mm=[300,0,500],front_panel_plane_normal_camera=[-1,0,0],
            front_panel_top_edge_midpoint_camera_mm=[300,0,600])
        f.client._request_json=lambda *a,**k:raw
        out=f.client.estimate(f.snapshot,f.state,f.state,selected=selection({'target':'bottle'}))
        self.assertTrue(out['grasp_test_usable'],out)
        directory=Path(out['directory'])
        request_path=directory/'pose_estimation_request.json'
        response_path=directory/'pose_estimation_response.json'
        new_request=request_path.read_bytes()
        raw['sku_typ']='box'
        response_path.write_text(json.dumps(raw),encoding='utf8')
        with self.assertRaises(PoseEstimationError):f.client.latest_world_grasp()
        historical=json.loads(new_request)
        historical.pop('sku_typ')
        historical.update(sku_id='Avene',class_name='Avene')
        raw.pop('sku_typ');raw.update(sku_id='Avene',class_name='Avene')
        request_path.write_text(json.dumps(historical),encoding='utf8')
        response_path.write_text(json.dumps(raw),encoding='utf8')
        before={p.name:p.read_bytes() for p in (request_path,response_path)}
        replay=f.client.latest_world_grasp()
        self.assertEqual(replay['world_grasp'],out['world_grasp'])
        self.assertEqual(before,{p.name:p.read_bytes() for p in (request_path,response_path)})


if __name__=='__main__':unittest.main()
