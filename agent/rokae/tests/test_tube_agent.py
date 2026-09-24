"""Tube public API contracts and shared motion, using fake hardware only."""
import copy
import http.client
import json
from pathlib import Path
import threading
from types import SimpleNamespace
import unittest
from unittest.mock import Mock, patch

import numpy as np
from rokae_web.agent_actions import AgentActions
from rokae_web.agent_http import make_agent_server
from rokae_web.agent_planning import preflight_pick
from rokae_web.arm_movel import HardwareMoveL, MotionPlanUnavailable
from rokae_web.backends import BackendError
import test_agent_interfaces
import test_box_agent
import test_tube_grasp
import test_tube_scan
import test_tube_placement


def payload():
    return dict(task_type='SORTING', target_type='sku', sku_typ='tube', hand='RIGHT',
                level='L2', localization_result={'sku_typ':'tube'})


class ContractTests(unittest.TestCase):
    def setUp(self):
        self.f=test_agent_interfaces.ActionsTests();self.f.setUp();self.addCleanup(self.f.doCleanups)
        self.actions,self.service=self.f.actions,self.f.service

    def test_wrong_hand_or_level_never_opens(self):
        self.service.robot.gripper_start_move=Mock()
        for i, change in enumerate(({'hand':'LEFT'},{'level':'L3'},{'sku_id':'tube'})):
            p=payload();p.update(change)
            self.assertGreaterEqual(self.actions.run('/manipulation/pick',p,str(i))[0],400)
        self.service.robot.gripper_start_move.assert_not_called()

    def test_preopen130_before_freeze_and_late_preflight_failure_never_moves(self):
        self.actions.point=Mock(return_value={'state':{}})
        def freeze(*a):
            g=self.service.robot.gripper_status()
            self.assertEqual(g['requested_position'],130);self.assertEqual(g['measured_position'],130)
            return {'sku_typ':'tube'}
        self.actions.geometry.freeze=freeze
        self.service.grasp_test.execute=Mock()
        with patch('rokae_web.agent_actions.preflight_pick',side_effect=BackendError('late path failed')), \
             patch('rokae_web.agent_actions.execute_trunk') as move:
            response=self.actions.run('/manipulation/pick',payload(),'tube-fail')
        self.assertIn('late path failed',response[1]['message'])
        move.assert_not_called();self.service.grasp_test.execute.assert_not_called()

    def test_place_requires_right_reference_and_full_preflight(self):
        self.actions.geometry.basket=Mock(return_value=[800,80,-500])
        self.service.placement.execute=Mock();self.actions.wait_job=Mock(return_value={})
        p=payload();p.pop('level');p['destination_type']='basket'
        self.actions.place(p)
        self.assertEqual(self.actions.geometry.basket.call_args.args[2],'right_arm')
        self.service.placement.execute.assert_called_once_with(
            {'source_result_id':'agent-basket','sku_typ':'tube'},prepared_reference=[800,80,-500],preflight_all=True)

    def test_scan_requires_exactly_two_existing_images(self):
        self.service.scan_sequence.execute=Mock()
        a=self.f.root/'1.jpg';a.write_bytes(b'jpg')
        self.actions.wait_job=Mock(return_value={'photos':[{'path':str(a)}]})
        with self.assertRaisesRegex(BackendError,'2张'):self.actions.rotate({'sku_typ':'tube','hand':'RIGHT'})


class ExecutionTests(unittest.TestCase):
    def test_prepared_pick_preserves_A150_and_never_reads_latest_web_pose(self):
        f=test_tube_grasp.SequenceTests();f.setUp();self.addCleanup(f.doCleanups)
        start=f.backend.read_memory_state()
        result=f.service.pose_estimator.reproject_latest_tube_to_right_shoulder(start,start)
        result.update(preplanned_advance_mm=150.,current_trunk_joints_deg=start['joints_deg']['trunk'])
        f.service.pose_estimator.reproject_latest_tube_to_right_shoulder=Mock(side_effect=AssertionError('latest web pose'))
        f.service.grasp_test.execute({'sku_typ':'tube','source_result_id':result['source_result_id'],'elbow_deg':0},
                                    prepared_result=result,require_gripper=True)
        f.service.grasp_test.thread.join(15)
        f.assert_completed(f.service.grasp_test.status(),150)
        choices=[d['advance_mm'] for e,d in f.audit if e=='grasp_advance_candidate']
        self.assertTrue(choices);self.assertEqual(set(choices),{150})

    def test_body_change_since_frozen_projection_prevents_arm_dispatch(self):
        f=test_tube_grasp.SequenceTests();f.setUp();self.addCleanup(f.doCleanups)
        start=f.backend.read_memory_state()
        result=f.service.pose_estimator.reproject_latest_tube_to_right_shoulder(start,start)
        result.update(preplanned_advance_mm=0.,current_trunk_joints_deg=[q+1 for q in start['joints_deg']['trunk']])
        f.service.grasp_test.execute({'sku_typ':'tube','source_result_id':result['source_result_id']},prepared_result=result,require_gripper=True)
        f.service.grasp_test.thread.join(15)
        self.assertEqual(f.service.grasp_test.status()['phase'],'failed')
        self.assertNotIn(('right_arm','start'),f.events);self.assertEqual(f.fixture.commands,[])

    def test_http_pick_frozen_result_shared_sequence_and_replay_once(self):
        f=test_tube_grasp.SequenceTests();f.setUp();self.addCleanup(f.doCleanups)
        service=f.service
        actions=AgentActions(service,Path(f.fixture.f.temp.name)/'tube-agent');self.addCleanup(actions.ledger.db.close)
        start=service.robot.read_memory_state()
        result=service.pose_estimator.reproject_latest_tube_to_right_shoulder(start,start)
        result.update(box_clearance=result['shoulder_grasp']['box_clearance'],preplanned_advance_mm=0.,
                      current_trunk_joints_deg=start['joints_deg']['trunk'])
        service.pose_estimator.reproject_latest_tube_to_right_shoulder=Mock(side_effect=AssertionError('latest web result'))
        actions.point=Mock(return_value={'state':start})
        actions.geometry.freeze=Mock(return_value={'sku_typ':'tube'})
        actions.geometry.project=Mock(return_value=result)
        def preflight(*args):
            self.assertEqual(f.fixture.f.gripper['requested_position'],130)
            self.assertNotIn(('right_arm','start'),f.events)
            return SimpleNamespace(start=start),start,result
        server=make_agent_server('127.0.0.1',0,'/manipulation',actions)
        thread=threading.Thread(target=server.serve_forever,daemon=True);thread.start()
        try:
            results=[]
            with patch('rokae_web.agent_actions.preflight_pick',side_effect=preflight), \
                 patch('rokae_web.agent_actions.execute_trunk',return_value=start):
                for _ in range(2):
                    conn=http.client.HTTPConnection(*server.server_address,timeout=20)
                    conn.request('POST','/manipulation/pick',json.dumps(payload()),{'Idempotency-Key':'tube-pick'})
                    response=conn.getresponse();body=json.loads(response.read());conn.close()
                    self.assertEqual(response.status,200,body);results.append(body)
            self.assertEqual(results[0],results[1]);self.assertEqual(results[0]['completed_moves'],5)
            f.assert_completed(service.grasp_test.status(),0)
            self.assertEqual(f.events.count(('gripper','preopen130')),1)
            actions.geometry.freeze.assert_called_once()
        finally:
            server.shutdown();server.server_close();thread.join(2)

    def test_rotate_reuses_tube_points_both_photos_and_offsets(self):
        f=test_tube_scan.TubeScanTests();f.setUp();self.addCleanup(f.doCleanups)
        actions=AgentActions(f.service,Path(f.f.tmp.name)/'agent');self.addCleanup(actions.ledger.db.close)
        actions._gripper_ready=Mock()
        response=actions.run('/manipulation/rotate',{'sku_typ':'tube','hand':'RIGHT'},'scan-tube')
        self.assertEqual(response[0],200,response)
        self.assertEqual(len(response[1]['image_paths']),2);self.assertEqual(response[1]['camera'],'left_wrist')
        offsets=[e[2][1] for e in f.events if e[:2]==('pose','right_arm')]
        self.assertEqual(offsets,[-40,50,-50])
        turn=next(e[1] for e in f.events[1:] if e[0]=='arms')
        self.assertEqual(turn['joints_deg']['right_arm'],[31,32,33,34,35,36,37])
        before=len(f.events)
        self.assertEqual(actions.run('/manipulation/rotate',{'sku_typ':'tube','hand':'RIGHT'},'scan-tube'),response)
        self.assertEqual(len(f.events),before)

    def test_place_reuses_web_release_and_return(self):
        f=test_tube_placement.TubePlacementTests();f.setUp();self.addCleanup(f.doCleanups)
        actions=AgentActions(f.service,Path(f.service.config['action_poses_file']).parent/'agent')
        self.addCleanup(actions.ledger.db.close)
        actions.geometry.basket=Mock(return_value=[800,100,-500])
        p=payload();p.pop('level');p['destination_type']='basket'
        response=actions.run('/manipulation/place',p,'place-tube')
        self.assertEqual(response[0],200,response)
        self.assertEqual(f.robot.events[0][2][:3], [400,20,-340])
        self.assertEqual(f.robot.events[-2][0],'memory');self.assertEqual(f.robot.events[-1][0],'body')
        self.assertIn(('open',0),f.robot.events)
        self.assertEqual(f.job.status()['final_state']['joints_deg'],f.f.returned['joints_deg'])


class GeometryTests(unittest.TestCase):
    def test_freezes_tube_midpoint_no_y_offset_175mm_and_saved_height(self):
        f=test_box_agent.BoxGeometryTests();f.setUp();self.addCleanup(f.doCleanups)
        raw=dict(f.raw,sku_typ='tube',ok=False,edge_valid=False,point_valid=False,
                 top_edge_center_camera_mm=[500,2,700],top_edge_center_chassis_mm=[1,1,1])
        frozen=f.geo.freeze(raw,f.state,'tube');result=f.geo.project(frozen,f.state)
        self.assertEqual(frozen['world_grasp']['grasp_point_trunk_mm'][:2],[500,2])
        self.assertAlmostEqual(frozen['world_grasp']['grasp_point_trunk_mm'][2],812.9056959008101)
        self.assertAlmostEqual(result['shoulder_grasp']['grasp_pose_right_shoulder_mm_deg'][0],325)


class PreflightTests(unittest.TestCase):
    def test_all_five_arm_paths_and_final_trunk_planned_before_any_dispatch(self):
        f=test_tube_grasp.SequenceTests();f.setUp();self.addCleanup(f.doCleanups)
        start=f.backend.read_memory_state()
        geometry=SimpleNamespace(project=lambda *a:{'shoulder_grasp':test_tube_grasp.shoulder()})
        with patch('rokae_web.agent_planning.prepare_trunk',return_value=(SimpleNamespace(),start)), \
             patch('rokae_web.agent_planning.load_guard_plane',return_value=None):
            _,_,result=preflight_pick(f.service,{'sku_typ':'tube'},start,geometry,threading.Event())
        self.assertEqual(result['preplanned_advance_mm'],0)
        record=next(d for e,d in f.audit if e=='agent_pick_preflight')
        self.assertEqual([p['stage'] for p in record['arm_plans']],test_tube_grasp.STAGES)
        self.assertEqual(record['retreat_pose'][0],start['poses']['trunk'][0]-100)
        self.assertNotIn(('right_arm','start'),f.events);self.assertEqual(f.fixture.commands,[])

    def test_final_path_rejection_happens_before_initial_body_or_arm_dispatch(self):
        f=test_tube_grasp.SequenceTests();f.setUp();self.addCleanup(f.doCleanups)
        start=f.backend.read_memory_state();geometry=SimpleNamespace(project=lambda *a:{'shoulder_grasp':test_tube_grasp.shoulder()})
        original=HardwareMoveL.plan;calls=[]
        def plan(arm,*args):
            calls.append(1)
            if len(calls)==5:raise MotionPlanUnavailable('last path blocked')
            return original(arm,*args)
        with patch('rokae_web.agent_planning.prepare_trunk',return_value=(SimpleNamespace(),start)), \
             patch('rokae_web.agent_planning.load_guard_plane',return_value=None),patch.object(HardwareMoveL,'plan',plan):
            with self.assertRaisesRegex(BackendError,'last path blocked'):
                preflight_pick(f.service,{'sku_typ':'tube'},start,geometry,threading.Event())
        self.assertNotIn(('right_arm','start'),f.events);self.assertEqual(f.fixture.commands,[])
