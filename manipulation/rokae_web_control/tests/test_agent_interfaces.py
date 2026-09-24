"""Offline tests: no real controllers, cameras, mode changes or network calls."""
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
from rokae_web.agent_actions import ActionLedger, AgentActions, AgentError
from rokae_web.agent_geometry import AgentGeometry
from rokae_web.agent_http import make_agent_server
from rokae_web.agent_planning import prepare_trunk, execute_trunk, preflight_pick
from rokae_web.backends import MockRobotBackend, MockChassisBackend, BackendError
from rokae_web.config import DEFAULT_CONFIG
from rokae_web.service import ControlService
from rokae_web.audit import MemoryAuditLogger
from rokae_web.arm_movel import transform
import test_trunk_retreat


class ActionsTests(unittest.TestCase):
    def setUp(self):
        temp = tempfile.TemporaryDirectory()
        self.addCleanup(temp.cleanup)
        self.root = Path(temp.name)
        config = copy.deepcopy(DEFAULT_CONFIG)
        config['memory_points'] = {'file': str(self.root/'memory.json')}
        self.service = ControlService(config, MockRobotBackend(), MockChassisBackend(), False, MemoryAuditLogger())
        self.addCleanup(self.service.close)
        self.service.robot.gripper_activate()
        self.actions = AgentActions(self.service, self.root/'agent')
        self.addCleanup(self.actions.ledger.db.close)

    def test_health_no_motion_and_pose_activates_once(self):
        robot = self.service.robot
        robot.gripper_status = Mock(return_value=dict(activation_state=0, fault_code=0))
        robot.gripper_activate = Mock(return_value=dict(activation_state=3, fault_code=0))
        robot.move_pose = Mock(side_effect=AssertionError('motion'))
        with self.assertRaises(AgentError):
            self.actions.health()
        robot.gripper_activate.assert_not_called()
        self.assertEqual(self.actions.health(pose=True)['status'], 'READY')
        robot.gripper_status.return_value = dict(activation_state=3, fault_code=0)
        self.actions.health(pose=True)
        robot.gripper_activate.assert_called_once()
        robot.move_pose.assert_not_called()

    def test_health_does_not_activate_while_busy(self):
        self.actions.active = {'route':'test'}
        self.service.robot.gripper_status = Mock(return_value={'activation_state':0})
        self.service.robot.gripper_activate = Mock()
        with self.assertRaises(AgentError):
            self.actions.health(pose=True)
        self.service.robot.gripper_activate.assert_not_called()

    def test_idempotency_replay_conflict_unknown_and_restart(self):
        ledger = self.actions.ledger
        self.assertIsNone(ledger.begin('one', '/pose/prepare', {}))
        with self.assertRaisesRegex(AgentError, '执行中或结果未知'):
            ledger.begin('one', '/pose/prepare', {})
        other = ActionLedger(self.root/'agent/actions.sqlite3')
        self.addCleanup(other.db.close)
        with self.assertRaises(AgentError):
            other.begin('one', '/pose/prepare', {})
        ledger.finish('one',[200,{'status':'SUCCEEDED'}])
        self.assertEqual(other.begin('one','/pose/prepare',{})[0],200)
        with self.assertRaises(AgentError):
            ledger.begin('one','/pose/prepare',{'different':True})

    def test_repeated_action_does_not_dispatch_twice(self):
        self.actions.prepare = Mock(return_value={})
        for _ in range(2):
            self.assertEqual(self.actions.run('/pose/prepare',{},'retry')[0],200)
        self.actions.prepare.assert_called_once()
        self.assertFalse(self.service.armed)

    def test_unsupported_before_motion(self):
        self.service.memory.execute = Mock(side_effect=AssertionError('motion'))
        for i, payload in enumerate(({'pose_type':'review_item_place'},
                {'pose_type':'AGV_carton_item_inspect','level':'L9'})):
            self.assertGreaterEqual(self.actions.run('/pose/prepare',payload,'reject'+str(i))[0],400)
        self.service.memory.execute.assert_not_called()
        payload=dict(task_type='SORTING',target_type='sku',sku_typ='bottle',hand='RIGHT',level='L3')
        self.assertEqual(self.actions.run('/manipulation/pick',payload,'unsupported')[1]['error_code'], 'LEVEL_NOT_CONFIGURED')

    def test_web_gate_blocks_mutations_but_stop_cancels_agent(self):
        self.actions.active={'route':'/manipulation/pick'}
        with self.assertRaises(AgentError):
            with self.actions.web_request('/api/speed',{}):
                self.fail('must block')
        with self.actions.web_request('/api/control/disarm',{}):
            self.assertTrue(self.actions.cancelled.is_set())

    def test_failure_in_preflight_never_executes_trunk_or_grasp(self):
        payload=dict(task_type='SORTING',target_type='sku',sku_typ='bottle',hand='RIGHT',level='L2',localization_result={})
        self.actions._gripper_ready=Mock()
        self.actions.point=Mock(return_value={'state':{}})
        self.actions.geometry.freeze=Mock(return_value={})
        self.service.grasp_test.execute=Mock()
        with patch('rokae_web.agent_actions.preflight_pick',side_effect=BackendError('no path')), \
             patch('rokae_web.agent_actions.execute_trunk') as move:
            self.assertGreaterEqual(self.actions.run('/manipulation/pick',payload,'no-plan')[0],400)
            move.assert_not_called()
            self.service.grasp_test.execute.assert_not_called()

    def test_pick_order_and_frozen_projection_after_body(self):
        events=[]
        payload=dict(task_type='SORTING',target_type='sku',sku_typ='bottle',hand='RIGHT',level='L2',localization_result={})
        self.actions._gripper_ready=Mock()
        self.actions.point=Mock(return_value={'state':{}})
        self.actions.geometry.freeze=lambda *_: events.append('freeze') or {'frozen':True}
        result={'source_result_id':'id','box_clearance':{'d_mm':50}}
        self.actions.geometry.project=lambda *_: events.append('project-actual') or result
        self.service.grasp_test.execute=lambda *a,**k: events.append('grasp')
        self.actions.wait_job=lambda _: {'completed_moves':6}
        with patch('rokae_web.agent_actions.preflight_pick',side_effect=lambda *a: events.append('all-plans') or (SimpleNamespace(start={}),{},{'preplanned_advance_mm':0.0})), \
             patch('rokae_web.agent_actions.execute_trunk',side_effect=lambda *a: events.append('trunk') or {}):
            self.assertEqual(self.actions.run('/manipulation/pick',payload,'plan-ok')[0],200)
        self.assertEqual(events,['freeze','all-plans','trunk','project-actual','grasp'])
        self.assertTrue((self.root/'agent/last_pick_trunk.json').is_file())
        self.assertFalse((self.root/'agent/barcode_trunk.json').exists())

    def test_scan_pose_uses_fixed_log_calibration_and_only_trunk_native_movel(self):
        from rokae_web.barcode_pose import load_barcode_pose
        calibration = load_barcode_pose({})
        self.assertEqual(calibration['source']['operation_id'], 'dfa42df502de4e01bc61861ef74b8977')
        target = calibration['state']['poses']['trunk']
        np.testing.assert_allclose(target, [-200.019101,0,999.997921,0,-.000726,-.000772])
        f = test_trunk_retreat.TrunkTests(); f.setUp(); self.addCleanup(f.doCleanups)
        f.ref = np.eye(4); f.end = np.eye(4)
        for part in (f.tool.end, f.tool.ref):
            part.trans = [0.0]*3; part.rpy = [0.0]*3
        old_read = f.backend.read_state
        def read():
            state = old_read()
            state['toolsets'] = copy.deepcopy(calibration['state']['toolsets'])
            return state
        f.backend.read_state = read
        f.backend.read_memory_state = read
        accessed = []
        def robot(module):
            accessed.append(module)
            self.assertEqual(module,'trunk')
            return f.robot
        f.backend._robot = robot
        self.service.robot = f.backend
        self.service.memory.execute = Mock(side_effect=AssertionError('must not replay whole-body memory'))
        before = read()
        result = self.actions.prepare({'pose_type':'AGV_item_barcode_scan'})
        after = read()
        self.assertEqual(result['moved_modules'],['trunk'])
        self.assertEqual(f.events.count('start'),1)
        self.assertEqual(f.events.count('calcIk'),1)
        np.testing.assert_allclose(after['poses']['trunk'],target,atol=1e-7)
        for module in ('left_arm','right_arm','head'):
            self.assertEqual(after['joints_deg'][module],before['joints_deg'][module])
        self.assertEqual(f.command.target.external,[.1,-.2,0,0,0,0])
        self.assertEqual(f.command.speed,self.service.speed_mm_s)
        self.assertAlmostEqual(f.command.rotSpeed,np.radians(self.service.rotation_deg_s))
        self.service.memory.execute.assert_not_called()
        self.assertEqual(accessed,['trunk'])

    def test_bad_scan_calibration_rejected_without_any_motion(self):
        self.service.config['barcode_pose_file'] = str(self.root/'bad.json')
        (self.root/'bad.json').write_text('{"schema_version": 1, "frame": "Chest_link"}')
        with patch('rokae_web.agent_actions.execute_trunk') as execute:
            with self.assertRaises(BackendError):
                self.actions.prepare({'pose_type':'AGV_item_barcode_scan'})
            execute.assert_not_called()

    def test_place_interface_requires_full_preflight(self):
        self.actions._gripper_ready=Mock()
        self.actions.geometry.basket=Mock(return_value=[800,100,-400])
        self.service.placement.execute=Mock()
        self.actions.wait_job=Mock(return_value={})
        response={'request_id':'basket-test'}
        self.actions.place(dict(task_type='SORTING',target_type='sku',destination_type='basket',
                                sku_typ='bottle',hand='RIGHT',localization_result=response))
        self.service.placement.execute.assert_called_once_with(
            {'source_result_id':'basket-test','sku_typ':'bottle'},prepared_reference=[800,100,-400],preflight_all=True)

    def test_public_manipulation_rejects_old_product_fields(self):
        payload = dict(task_type='SORTING',target_type='sku',hand='RIGHT',level='L2',
                       sku_id='Avene',localization_result={})
        with self.assertRaisesRegex(AgentError,'sku_typ'):
            self.actions._sorting(payload,pick=True)

    def test_prepare_mapping_and_review_rejected(self):
        self.actions.point=Mock(return_value=dict(id='point',revision=1,name='L2观察'))
        self.service.memory.execute=Mock()
        self.actions.wait_job=Mock(return_value={})
        for level in ('L1','L2','L3','L4','L5'):
            self.actions.prepare(dict(pose_type='AGV_carton_item_inspect',level=level))
            self.actions.point.assert_called_with('L2观察')
        self.actions.prepare(dict(pose_type='basket_push',level='L4'))
        self.actions.point.assert_called_with('L2抓取')

    def test_rotate_returns_five_ordered_file_paths(self):
        photos=[]
        for i in range(1,6):
            p=self.root/f'{i}.jpg';p.write_bytes(b'jpg');photos.append({'path':str(p)})
        self.actions._gripper_ready=Mock()
        self.service.scan_sequence.execute=Mock()
        self.actions.wait_job=Mock(return_value={'photos':photos})
        result=self.actions.rotate({'hand':'RIGHT'})
        self.assertEqual(result['image_paths'],[p['path'] for p in photos])
        self.actions.wait_job.assert_called_once_with(self.service.scan_sequence)

    def test_box_rotate_returns_one_right_wrist_image_without_gripper_or_suction_command(self):
        image=self.root/'box.jpg';image.write_bytes(b'jpg')
        self.actions._gripper_ready=Mock(side_effect=AssertionError('box does not use right gripper'))
        self.service.robot.suction_set=Mock(side_effect=AssertionError('scan must preserve suction'))
        self.service.scan_sequence.execute=Mock()
        self.actions.wait_job=Mock(return_value={'photos':[{'path':str(image)}]})
        result=self.actions.rotate({'hand':'LEFT','sku_typ':'box'})
        self.assertEqual(result,{'image_paths':[str(image)],'camera':'right_wrist'})
        self.service.scan_sequence.execute.assert_called_once_with({'sku_typ':'box'})
        self.service.robot.suction_set.assert_not_called()

    def test_http_success_and_missing_key_rejection(self):
        self.actions.prepare=Mock(return_value={})
        server=make_agent_server('127.0.0.1',0,'/pose',self.actions)
        thread=threading.Thread(target=server.serve_forever,daemon=True);thread.start()
        try:
            c=http.client.HTTPConnection(*server.server_address)
            c.request('POST','/pose/prepare','{}',{'Content-Type':'application/json'})
            response=c.getresponse();self.assertEqual(response.status,400);response.read();c.close()
            self.actions.prepare.assert_not_called()
            c=http.client.HTTPConnection(*server.server_address)
            c.request('POST','/pose/prepare','{}',{'Idempotency-Key':'http-test'})
            response=c.getresponse();self.assertEqual(response.status,200)
            self.assertEqual(json.loads(response.read())['status'],'SUCCEEDED');c.close()
        finally:
            server.shutdown();server.server_close();thread.join(2)


class GeometryTests(unittest.TestCase):
    def test_freeze_trunk_height_then_reproject_after_body_translation(self):
        class Kin:
            def camera_to_base(self,*_):return np.eye(4)
            def forward_deg(self,q,tip_link=None):
                t=np.eye(4);t[0,3]=q[0]/1000;return t
            def right_shoulder_sdk_world(self,q):return self.forward_deg(q)
        estimator=SimpleNamespace(_load_calibration=lambda:(None,np.eye(4),None),
            _kinematics=Kin(),_upper_body_joints=lambda s:np.array(s['joints_deg']['trunk']+[0,0]),
            CAMERA_FRAME='head_camera_color_optical_frame',
            config={'grasp_height_trunk_mm_by_sku':{'Avene':792.90086}})
        robot=MockRobotBackend()
        s=robot.read_memory_state()
        s['joints_deg']['trunk']=[0]*4;s['poses']['trunk']=[0]*6
        s['toolsets']['trunk']={'end':[0]*6,'ref':[0]*6}
        geo=AgentGeometry(SimpleNamespace(pose_estimator=estimator,robot=robot))
        response=dict(ok=True,sku_typ='bottle',class_name='bottle',output_frame=estimator.CAMERA_FRAME,output_unit='mm',
            axis_fit_valid=True,reference_point_valid=True,reference_point_camera_mm=[500,0,800],
            reference_point_chassis_mm=[500,0,800],axis_point_camera_mm=[500,0,800],axis_direction_camera_up=[0,0,1],
            front_panel_valid=True,front_panel_plane_point_camera_mm=[400,0,0],
            front_panel_plane_normal_camera=[1,0,0],front_panel_top_edge_midpoint_camera_mm=[400,0,700])
        frozen=geo.freeze(response,s)
        before=geo.project(frozen,s)
        s['joints_deg']['trunk'][0]=100;s['poses']['trunk'][0]=100
        after=geo.project(frozen,s)
        self.assertEqual(after['box_clearance']['d_mm'],100)
        self.assertAlmostEqual(frozen['world_grasp']['grasp_point_trunk_mm'][2],792.90086)
        self.assertEqual(frozen['world_grasp']['grasp_point_trunk_mm'][1],10)
        self.assertAlmostEqual(before['shoulder_grasp']['grasp_pose_right_shoulder_mm_deg'][0]-
                               after['shoulder_grasp']['grasp_pose_right_shoulder_mm_deg'][0],100)


class TrunkPlanningTests(unittest.TestCase):
    def test_all_five_arm_paths_checked_before_any_dispatch(self):
        import test_arm_movel
        from test_grasp_test import SHOULDER
        f=test_arm_movel.AdapterTests();f.setUp()
        self.addCleanup(f.doCleanups);self.addCleanup(f.tearDown)
        state=f.backend.read_memory_state()
        service=SimpleNamespace(robot=f.backend,config=f.config,audit_event=Mock())
        result={'shoulder_grasp':copy.deepcopy(SHOULDER)}
        geometry=SimpleNamespace(project=lambda *args:result)
        body=SimpleNamespace(sdk=f.backend._load_sdk(),current=SimpleNamespace(confData=[],external=[]))
        with patch('rokae_web.agent_planning.prepare_trunk',return_value=(body,state)), \
             patch('rokae_web.agent_planning.trunk_ik',return_value=[0]*4), \
             patch('rokae_web.agent_planning.load_guard_plane',return_value=None):
            preflight_pick(service,{},state,geometry,f.cancel)
        self.assertEqual(f.events.count(('right_arm','checkPath')),5)
        self.assertFalse(any('start' in str(e).lower() or 'append' in str(e).lower() for e in f.events))

    def test_preflight_only_then_trunk_moves_arms_head_fixed(self):
        f=test_trunk_retreat.TrunkTests();f.setUp()
        self.addCleanup(f.doCleanups)
        state=f.backend.read_state()
        state['toolsets']={'trunk':{'end':list(f.tool.end.trans)+list(f.tool.end.rpy),'ref':list(f.tool.ref.trans)+list(f.tool.ref.rpy)}}
        read=f.backend.read_state
        def read_with_tools():
            current=read();current['toolsets']=copy.deepcopy(state['toolsets']);return current
        f.backend.read_state=read_with_tools
        saved=copy.deepcopy(state);saved['poses']['trunk'][0]-=30
        service=SimpleNamespace(robot=f.backend,config=f.config,speed_mm_s=33,rotation_deg_s=7)
        move,predicted=prepare_trunk(service,saved,f.cancel,state)
        self.assertEqual(f.events,['calcIk'])
        actual=execute_trunk(service,move,state)
        self.assertEqual(f.events.count('start'),1)
        self.assertEqual(f.command.speed,33)
        self.assertAlmostEqual(f.command.rotSpeed,np.radians(7))
        self.assertEqual(actual['joints_deg']['head'],state['joints_deg']['head'])

    def test_uncertain_stop_latches(self):
        service=SimpleNamespace(speed_mm_s=1,rotation_deg_s=2,agent_actions=SimpleNamespace(stop_unconfirmed=False))
        move=SimpleNamespace(check=lambda:None,start_move=Mock(side_effect=BackendError('lost')),
                             stop_and_verify=lambda:['no acknowledgement'])
        with self.assertRaises(BackendError):execute_trunk(service,move,{})
        self.assertTrue(service.agent_actions.stop_unconfirmed)
