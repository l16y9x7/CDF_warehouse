"""Box Agent contracts and complete preflight using only fake hardware."""
import copy
import http.client
import json
import tempfile
import threading
import unittest
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import Mock, patch

import numpy as np
from rokae_web.agent_actions import AgentActions, AgentError
from rokae_web.agent_geometry import AgentGeometry
from rokae_web.agent_http import make_agent_server
from rokae_web.agent_planning import preflight_pick
from rokae_web.backends import BackendError, MockRobotBackend
from rokae_web.memory_motion import capture
from rokae_web.placement_planning import preflight_box
import test_agent_interfaces as agent_fixtures
import test_box_placement as place_fixtures
import test_left_box_grasp as grasp_fixtures
import test_placement_preflight as plan_fixtures
import test_pose_targets as vision_fixtures


def pick_payload(**extra):
    return dict(task_type='SORTING', target_type='sku', sku_typ='box', hand='LEFT',
                level='L2', localization_result={'sku_typ':'box'}, **extra)


class BoxActionTests(unittest.TestCase):
    def setUp(self):
        self.f = agent_fixtures.ActionsTests(); self.f.setUp()
        self.addCleanup(self.f.doCleanups)
        self.actions, self.service = self.f.actions, self.f.service
        self.service.config['suction'] = dict(enabled=True, slave_id=1,
            open_outputs=[dict(address=0,value=True)], close_outputs=[dict(address=0,value=False)])
        self.actions._gripper_ready = Mock(side_effect=AssertionError('box must not read right gripper'))

    def test_wrong_arm_type_or_level_rejected_before_geometry_or_motion(self):
        self.actions.geometry.freeze = Mock(side_effect=AssertionError('no geometry expected'))
        self.service.robot.suction_set = Mock()
        for index, change in enumerate(({'hand':'RIGHT'}, {'sku_typ':'tube'}, {'level':'L3'}, {'sku_id':'estee'})):
            payload = pick_payload(); payload.update(change)
            status, _ = self.actions.run('/manipulation/pick', payload, 'bad-box-'+str(index))
            self.assertGreaterEqual(status,400)
        self.actions.geometry.freeze.assert_not_called()
        self.service.robot.suction_set.assert_not_called()

    def test_box_pick_preflight_failure_leaves_suction_closed_without_motion(self):
        self.actions.point = Mock(return_value={'state':{}})
        self.actions.geometry.freeze = Mock(return_value={'sku_typ':'box'})
        self.service.grasp_test.execute = Mock()
        self.service.robot.suction_set = Mock(return_value={'commanded_open':False,'confirmed':True})
        with patch('rokae_web.agent_actions.preflight_pick',side_effect=BackendError('no box path')), \
             patch('rokae_web.agent_actions.execute_trunk') as move:
            result = self.actions.run('/manipulation/pick',pick_payload(),'bad-plan')
        self.assertGreaterEqual(result[0],400)
        move.assert_not_called(); self.service.grasp_test.execute.assert_not_called()
        self.service.robot.suction_set.assert_called_once_with(False,self.service.config['suction'])
        self.assertEqual(self.service._suction_result,{'commanded_open':False,'confirmed':True})

    def test_failed_or_unconfirmed_close_prevents_geometry_planning_and_motion(self):
        self.actions.geometry.freeze = Mock()
        self.service.grasp_test.execute = Mock()
        outcomes = [BackendError('relay timeout'), {'commanded_open':False,'confirmed':False},
                    {'commanded_open':True,'confirmed':True}]
        with patch('rokae_web.agent_actions.preflight_pick') as plan, \
             patch('rokae_web.agent_actions.execute_trunk') as move:
            for i,outcome in enumerate(outcomes):
                self.service.robot.suction_set = Mock(side_effect=[outcome])
                response = self.actions.run('/manipulation/pick',pick_payload(),'close-failed-'+str(i))
                self.assertGreaterEqual(response[0],400,response)
                self.service.robot.suction_set.assert_called_once()
                self.assertFalse(self.service._suction_result['confirmed'])
                self.assertIn('error',self.service._suction_result)
        self.actions.geometry.freeze.assert_not_called()
        plan.assert_not_called(); move.assert_not_called()
        self.service.grasp_test.execute.assert_not_called()

    def test_cancel_during_close_does_not_plan_or_reopen(self):
        def close(*args):
            self.actions.cancelled.set()
            return {'commanded_open':False,'confirmed':True}
        self.service.robot.suction_set = Mock(side_effect=close)
        self.actions.geometry.freeze = Mock()
        with patch('rokae_web.agent_actions.preflight_pick') as plan, \
             patch('rokae_web.agent_actions.execute_trunk') as move:
            response = self.actions.run('/manipulation/pick',pick_payload(),'close-cancelled')
        self.assertEqual(response[1]['error_code'],'CANCELLED')
        self.service.robot.suction_set.assert_called_once()
        self.actions.geometry.freeze.assert_not_called()
        plan.assert_not_called(); move.assert_not_called()

    def test_already_cancelled_action_does_not_close_suction(self):
        self.actions.cancelled.set()
        self.service.robot.suction_set = Mock()
        with self.assertRaises(AgentError):
            self.actions.pick(pick_payload())
        self.service.robot.suction_set.assert_not_called()

    def test_bottle_pick_does_not_switch_left_suction(self):
        self.service.robot.suction_set = Mock(side_effect=AssertionError('right pick must not use suction'))
        self.f.test_pick_order_and_frozen_projection_after_body()
        self.service.robot.suction_set.assert_not_called()

    def test_web_payload_cannot_skip_initial_suction_close(self):
        with self.assertRaisesRegex(BackendError,'仅接受'):
            self.service.grasp_test.execute({'sku_typ':'box','source_result_id':'id',
                                            'prepared_suction_config':self.service.config['suction']})

    def test_box_pick_requires_valid_suction_configuration_before_body_motion(self):
        self.service.config['suction'] = {'enabled':False}
        with patch('rokae_web.agent_actions.execute_trunk') as move:
            result = self.actions.run('/manipulation/pick',pick_payload(),'no-suction')
        self.assertGreaterEqual(result[0],400); move.assert_not_called()

    def test_box_place_passes_left_reference_and_requires_full_preflight(self):
        self.actions.geometry.basket = Mock(return_value=[800,-80,-500])
        self.service.placement.execute = Mock()
        self.actions.wait_job = Mock(return_value={})
        payload=pick_payload(); payload.pop('level'); payload['destination_type']='basket'
        self.actions.place(payload)
        self.assertEqual(self.actions.geometry.basket.call_args.args[2],'left_arm')
        self.service.placement.execute.assert_called_once_with(
            {'source_result_id':'agent-basket','sku_typ':'box'},
            prepared_reference=[800,-80,-500],preflight_all=True)


class BoxPickExecutionTests(unittest.TestCase):
    def test_http_pick_uses_prepared_geometry_and_web_sequence_and_replays_once(self):
        f=grasp_fixtures.LeftSequenceTests(); f.setUp(); self.addCleanup(f.doCleanups)
        service=f.service
        actions=AgentActions(service,Path(f.f.temp.name)/'agent')
        self.addCleanup(actions.ledger.db.close)
        start=service.robot.read_memory_state()
        result=f.f.estimator.reproject_latest_to_left_shoulder(start,start)
        result['box_clearance']=result['shoulder_grasp']['box_clearance']
        result['preplanned_advance_mm']=0.0
        f.f.estimator.reproject_latest_to_left_shoulder=Mock(side_effect=AssertionError('must not read latest web result'))
        actions.point=Mock(return_value={'state':start})
        events=[]
        original_suction=service.robot.suction_set
        suction_configs=[]
        def suction(opened,config):
            events.append(('suction',opened))
            suction_configs.append(copy.deepcopy(config))
            return original_suction(opened,config)
        service.robot.suction_set=suction
        initial_suction_config=copy.deepcopy(service.config['suction'])
        actions.geometry.freeze=lambda response,state,kind: events.append(('freeze',kind)) or {'sku_typ':kind}
        actions.geometry.project=lambda *a: events.append(('project',)) or copy.deepcopy(result)
        def preflight(*args):
            self.assertEqual(f.suction,[(False,0)])
            self.assertFalse(any(e[1]=='start' for e in f.f.events))
            events.append(('preflight',))
            return SimpleNamespace(start=start),start,result
        def move(*args):
            self.assertEqual(f.suction,[(False,0)])
            # The confirmed command's configuration remains the task snapshot.
            service.config['suction']['slave_id']=2
            events.append(('trunk',)); return start
        server=make_agent_server('127.0.0.1',0,'/manipulation',actions)
        thread=threading.Thread(target=server.serve_forever,daemon=True);thread.start()
        try:
            responses=[]
            with patch('rokae_web.agent_actions.preflight_pick',side_effect=preflight), \
                 patch('rokae_web.agent_actions.execute_trunk',side_effect=move):
                for _ in range(2):
                    conn=http.client.HTTPConnection(*server.server_address,timeout=10)
                    conn.request('POST','/manipulation/pick',json.dumps(pick_payload()),
                                 {'Content-Type':'application/json','Idempotency-Key':'box-http-pick'})
                    response=conn.getresponse(); body=json.loads(response.read());conn.close()
                    self.assertEqual(response.status,200,body);responses.append(body)
            self.assertEqual(responses[0],responses[1])
            self.assertEqual(responses[0]['completed_moves'],6)
            self.assertEqual(events,[('suction',False),('freeze','box'),('preflight',),
                                     ('trunk',),('project',),('suction',True)])
            self.assertEqual(f.suction,[(False,0),(True,3)])
            self.assertEqual(f.service.grasp_test.status()['arm_retreat_mm'],230)
            self.assertEqual(suction_configs,[initial_suction_config,initial_suction_config])
            self.assertEqual(f.f.events.count(('left_arm','start')),5)
            self.assertEqual(f.f.events.count(('trunk_retreat','start')),1)
            self.assertFalse(any(e[0]=='right_arm' and e[1]=='start' for e in f.f.events))
        finally:
            server.shutdown();server.server_close();thread.join(2)


class BoxGeometryTests(unittest.TestCase):
    def setUp(self):
        temp=tempfile.TemporaryDirectory();self.addCleanup(temp.cleanup)
        path=Path(temp.name)/'box.json';path.write_text(json.dumps(grasp_fixtures.CALIBRATION))
        class Kin:
            def camera_to_base(self,*args):return np.eye(4)
            def forward_deg(self,q,tip_link=None):
                t=np.eye(4);t[0,3]=q[0]/1000;return t
            def left_shoulder_sdk_world(self,q):
                t=self.forward_deg(q);t[1,3]=.2;return t
            def right_shoulder_sdk_world(self,q):
                t=self.forward_deg(q);t[1,3]=-.2;return t
        estimator=SimpleNamespace(_load_calibration=lambda:(None,np.eye(4),None),_kinematics=Kin(),
            _upper_body_joints=lambda s:np.array(s['joints_deg']['trunk']+[0,0]),
            CAMERA_FRAME='head_camera_color_optical_frame',config={'box_grasp_file':str(path),'grasp_height_trunk_mm':123})
        self.service=SimpleNamespace(pose_estimator=estimator,robot=MockRobotBackend(),audit_event=Mock())
        self.geo=AgentGeometry(self.service)
        self.state=self.service.robot.read_memory_state()
        self.state['joints_deg']['trunk']=[0]*4;self.state['poses']['trunk']=[0]*6
        self.state['toolsets']['trunk']={'end':[0]*6,'ref':[0]*6}
        self.raw=dict(ok=True,sku_typ='box',top_plane_valid=True,top_point_valid=True,
            output_frame=estimator.CAMERA_FRAME,output_unit='mm',top_point_camera_mm=[500,100,700],
            front_panel_valid=True,front_panel_plane_point_camera_mm=[300,0,0],
            front_panel_plane_normal_camera=[1,0,0],front_panel_top_edge_midpoint_camera_mm=[300,0,600])

    def test_frozen_box_uses_independent_heights_and_left_shoulder_after_body(self):
        frozen=self.geo.freeze(self.raw,self.state,'box')
        first=self.geo.project(frozen,self.state)
        self.state['joints_deg']['trunk'][0]=100;self.state['poses']['trunk'][0]=100
        second=self.geo.project(frozen,self.state)
        self.assertEqual(frozen['world_grasp']['grasp_point_trunk_mm'],[500,100,800])
        self.assertEqual(second['box_clearance']['pregrasp_virtual_length_mm'],430)
        a=first['shoulder_grasp'];b=second['shoulder_grasp']
        for stage in ('pregrasp','grasp','descend','lift'):
            key=stage+'_pose_left_shoulder_mm_deg'
            self.assertAlmostEqual(a[key][0]-b[key][0],100)
            self.assertAlmostEqual(b[key][1],-100)
        self.assertEqual([b[k+'_pose_left_shoulder_mm_deg'][2] for k in ('grasp','descend','lift')],[800,770,890])
        self.assertEqual(b['frame'],'left_arm_sdk_world')

    def test_missing_top_or_panel_or_wrong_kind_rejected(self):
        for change in ({'sku_typ':'bottle'},
                       {'top_point_camera_mm':None},{'front_panel_plane_normal_camera':None}):
            with self.subTest(change=change),self.assertRaises((BackendError,ValueError)):
                self.geo.freeze(dict(self.raw,**change),self.state,'box')

    def test_basket_projects_to_requested_shoulder_not_other_arm(self):
        raw=vision_fixtures.response('basket')
        self.assertEqual(self.geo.basket(raw,self.state,'left_arm'),[1,-198,500])
        self.assertEqual(self.geo.basket(raw,self.state,'right_arm'),[1,202,500])


class BoxPlannerTests(unittest.TestCase):
    def setUp(self):
        f=self.f=plan_fixtures.PlannerTests();f.setUp();self.addCleanup(f.doCleanups)
        self.points=[{'state':copy.deepcopy(f.start)},{'state':copy.deepcopy(f.start)},{'state':copy.deepcopy(f.saved)}]
        self.points[0]['state']['arm_elbow_deg']['left_arm']=5
        self.pose=[40,10,0,0,0,0]

    def test_box_place_checks_trunk_before_left_descent_and_full_return_without_dispatch(self):
        f=self.f
        preflight_box(f.job,self.points,f.start,self.pose,f.fixture.plane,f.speeds)
        record=next(c.kwargs for c in f.service.audit_event.call_args_list if c.args[0]=='placement_preflight_passed')
        self.assertEqual(record['sku_typ'],'box')
        self.assertEqual(list(dict.fromkeys(s['stage'] for s in record['steps'])),
                         ['box_preplacement','box_advance','box_lower','box_return_arms','box_return_body'])
        target=next(s['target'] for s in record['steps'] if s['stage']=='box_lower' and s['module']=='left_arm')
        self.assertEqual(target,[40,10,-100,0,0,0])
        self.assertEqual(f.fixture.events.count(('trunk','calcIk')),1)
        self.assertEqual(f.fixture.events.count(('left_arm','checkPath')),3)
        self.assertEqual(f.fixture.events.count(('right_arm','checkPath')),1)
        f.assert_no_motion()

    def test_left_descent_preflight_uses_predicted_advanced_trunk(self):
        from rokae_web.placement_planning import BoxPreflight
        f=self.f
        planner=BoxPreflight(f.job,f.start,f.fixture.plane,f.speeds)
        original=planner.linear
        snapshots=[]
        def linear(*args,**kwargs):
            snapshots.append((planner.stage,planner.view.read_state()))
            return original(*args,**kwargs)
        with patch.object(planner,'linear',side_effect=linear):planner.run(self.points,self.pose)
        self.assertEqual([s[0] for s in snapshots],['box_preplacement','box_lower'])
        after=snapshots[1][1]
        self.assertEqual(after['poses']['trunk'][0],f.start['poses']['trunk'][0]+100)
        self.assertAlmostEqual(after['joints_deg']['trunk'][0],f.start['joints_deg']['trunk'][0]+1)
        self.assertEqual(after['poses']['left_arm'],self.pose)
        f.assert_no_motion()

    def test_late_body_return_limit_failure_rejects_before_any_dispatch(self):
        self.points[2]['state']['joints_deg']['head'][0]=2000
        with self.assertRaisesRegex(BackendError,'box_return_arms.*整套动作未启动'):
            preflight_box(self.f.job,self.points,self.f.start,self.pose,self.f.fixture.plane,self.f.speeds)
        self.f.assert_no_motion()

    def test_five_box_pick_paths_use_left_arm_before_retreat_ik(self):
        f=self.f
        _,shoulder=grasp_fixtures.box_geometry()
        geometry=SimpleNamespace(project=lambda *a:{'shoulder_grasp':shoulder})
        body=SimpleNamespace(sdk=f.fixture.backend._load_sdk(),current=SimpleNamespace(confData=[],external=[]))
        with patch('rokae_web.agent_planning.prepare_trunk',return_value=(body,f.start)), \
             patch('rokae_web.agent_planning.trunk_ik',return_value=[0]*4) as ik, \
             patch('rokae_web.agent_planning.load_guard_plane',return_value=f.fixture.plane):
            preflight_pick(f.service,{'sku_typ':'box'},f.start,geometry,f.fixture.cancel)
        self.assertEqual(f.fixture.events.count(('left_arm','checkPath')),5)
        self.assertEqual(f.fixture.events.count(('right_arm','checkPath')),0)
        record=next(c.kwargs for c in f.service.audit_event.call_args_list if c.args[0]=='agent_pick_preflight')
        plans={p['stage']:p['target'] for p in record['arm_plans']}
        self.assertAlmostEqual(plans['lift'][0]-plans['arm_retreat'][0],230)
        ik.assert_called_once();f.assert_no_motion()

    def test_box_pick_final_retreat_failure_is_logged_without_dispatch(self):
        f=self.f
        _,shoulder=grasp_fixtures.box_geometry()
        geometry=SimpleNamespace(project=lambda *a:{'shoulder_grasp':shoulder})
        body=SimpleNamespace(sdk=f.fixture.backend._load_sdk(),current=SimpleNamespace(confData=[],external=[]))
        with patch('rokae_web.agent_planning.prepare_trunk',return_value=(body,f.start)), \
             patch('rokae_web.agent_planning.trunk_ik',side_effect=BackendError('retreat unreachable')), \
             patch('rokae_web.agent_planning.load_guard_plane',return_value=f.fixture.plane):
            with self.assertRaisesRegex(BackendError,'整套动作未启动.*retreat unreachable'):
                preflight_pick(f.service,{'sku_typ':'box'},f.start,geometry,f.fixture.cancel)
        self.assertEqual(f.fixture.events.count(('left_arm','checkPath')),5)
        failed=next(c.kwargs for c in f.service.audit_event.call_args_list if c.args[0]=='agent_pick_preflight_failed')
        self.assertEqual(failed['module'],'left_arm');self.assertFalse(failed['motion_started'])
        f.assert_no_motion()

    def test_box_place_trunk_ik_failure_prevents_first_left_motion(self):
        f=self.f
        f.fixture.backend._robot('trunk').model=lambda:SimpleNamespace(calcIk=Mock(side_effect=BackendError('body unreachable')))
        with self.assertRaisesRegex(BackendError,'box_advance.*整套动作未启动'):
            preflight_box(f.job,self.points,f.start,self.pose,f.fixture.plane,f.speeds)
        f.assert_no_motion()


class BoxPlaceDispatchTests(unittest.TestCase):
    def setUp(self):
        self.f=place_fixtures.BoxPlacementTests();self.f.setUp();self.addCleanup(self.f.doCleanups)

    def test_external_reference_needs_no_saved_web_result_and_runs_full_flow(self):
        f=self.f;f.summary_path.unlink()
        with patch('rokae_web.placement_planning.preflight_box',side_effect=lambda *a:self.assertEqual(f.robot.events,[])) as preflight:
            f.job.execute({'source_result_id':'external-basket','sku_typ':'box'},
                          prepared_reference=[800,-80,-500],preflight_all=True)
            f.job.thread.join(3)
        self.assertEqual(f.job.status()['phase'],'completed',f.job.status())
        preflight.assert_called_once()
        self.assertEqual(preflight.call_args.args[3],[400,30,-340,0,-70,-180])
        self.assertEqual([e[0] for e in f.robot.events],['pose','pose','pose','suction','memory','body'])

    def test_preflight_failure_does_not_move_or_release_box(self):
        f=self.f
        with patch('rokae_web.placement_planning.preflight_box',side_effect=BackendError('return failed')):
            f.job.execute({'source_result_id':'external-basket','sku_typ':'box'},
                          prepared_reference=[800,-80,-500],preflight_all=True)
            f.job.thread.join(3)
        self.assertEqual(f.job.status()['phase'],'failed')
        self.assertEqual(f.robot.events,[])

    def test_changed_start_during_preflight_does_not_dispatch(self):
        f=self.f
        def drift(*args):f.robot._state['joints_deg']['left_arm'][0]+=1
        with patch('rokae_web.placement_planning.preflight_box',side_effect=drift):
            f.job.execute({'source_result_id':'external-basket','sku_typ':'box'},
                          prepared_reference=[800,-80,-500],preflight_all=True)
            f.job.thread.join(3)
        self.assertEqual(f.job.status()['phase'],'failed');self.assertEqual(f.robot.events,[])

    def test_http_place_runs_complete_box_sequence_once_without_web_snapshot(self):
        f=self.f;f.summary_path.unlink()
        actions=AgentActions(f.service,Path(f.fixture.tmp.name)/'agent')
        self.addCleanup(actions.ledger.db.close)
        actions.geometry.basket=Mock(return_value=[800,-80,-500])
        f.robot.gripper_status=Mock(side_effect=AssertionError('box cannot use right gripper'))
        payload=pick_payload();payload.pop('level');payload['destination_type']='basket'
        server=make_agent_server('127.0.0.1',0,'/manipulation',actions)
        thread=threading.Thread(target=server.serve_forever,daemon=True);thread.start()
        try:
            with patch('rokae_web.placement_planning.preflight_box',side_effect=lambda *a:self.assertEqual(f.robot.events,[])) as preflight:
                for _ in range(2):
                    conn=http.client.HTTPConnection(*server.server_address,timeout=10)
                    conn.request('POST','/manipulation/place',json.dumps(payload),
                                 {'Content-Type':'application/json','Idempotency-Key':'box-http-place'})
                    response=conn.getresponse();body=json.loads(response.read());conn.close()
                    self.assertEqual(response.status,200,body)
                preflight.assert_called_once()
            self.assertEqual([e[0] for e in f.robot.events],['pose','pose','pose','suction','memory','body'])
            self.assertEqual([e[1] for e in f.robot.events[:3]],['left_arm','trunk','left_arm'])
            self.assertEqual(f.robot.events[0][2],[400,30,-340,0,-70,-180])
            self.assertEqual(f.robot.events[2][2],[400,30,-440,0,-70,-180])
            self.assertEqual(f.job.status()['completed_moves'],5)
            actions.geometry.basket.assert_called_once()
        finally:
            server.shutdown();server.server_close();thread.join(2)


if __name__=='__main__':unittest.main()
