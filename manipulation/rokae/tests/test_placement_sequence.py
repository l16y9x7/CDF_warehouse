import copy
import json
from pathlib import Path
import tempfile
import threading
import time
import unittest
from unittest.mock import patch

from rokae_web.backends import BackendError, MockRobotBackend, MockChassisBackend
from rokae_web.config import DEFAULT_CONFIG
from rokae_web.placement_sequence import ARM, basket_reference
from rokae_web.service import ControlService


class Robot(MockRobotBackend):
    def __init__(self):
        super().__init__()
        self.events=[]
        self.fail_open=False
    def start_memory_arms(self,plan,cancel):
        self.events.append(('memory',copy.deepcopy(plan['target'])))
        super().start_memory_arms(plan,cancel)
    def move_pose(self,module,pose,speed,elbow_deg=None):
        self.events.append(('pose',module,list(pose)))
        super().move_pose(module,pose,speed,elbow_deg)
    def move_joints(self,module,q,speed):
        self.events.append(('joint',module,list(q)))
        super().move_joints(module,q,speed)
    def gripper_start_move(self,value):
        self.events.append(('open',value))
        if self.fail_open:raise BackendError('open failed')
        super().gripper_start_move(value)
    def stop_memory_motion(self):
        self.events.append(('stop',))
        return []


class PlacementTests(unittest.TestCase):
    def setUp(self):
        self.tmp=tempfile.TemporaryDirectory();root=Path(self.tmp.name)
        cfg=copy.deepcopy(DEFAULT_CONFIG)
        cfg['memory_points']={'file':str(root/'memory.json')}
        cfg['camera']['data_directory']=str(root/'data')
        cfg['torso_guard_file']=str(root/'guard.json')
        (root/'guard.json').write_text(json.dumps(dict(plane_offset_mm=20,elbow_radius_mm=65,margin_mm=10)))
        self.robot=Robot()
        self.robot._state['poses'][ARM]=[200,50,400,180,0,0]
        self.robot._state['poses']['trunk']=[100,0,600,0,0,0]
        self.robot._state['joints_deg'][ARM]=[1,2,3,4,5,6,20]
        self.service=ControlService(cfg,self.robot,MockChassisBackend(),False)
        saved=self.robot.read_state();saved['poses']['head']=[0]*6
        saved['joints_deg'][ARM]=[11,12,13,14,15,16,40]
        saved['poses'][ARM]=[400,100,300,180,-90,0]
        # The whole point contains deliberately different head, left arm and torso.
        for m in ('left_arm','head','trunk'):saved['joints_deg'][m]=[50]*len(saved['joints_deg'][m])
        self.service.memory.store.save('L2放置1',saved,'mock')
        self.memory=self.service.memory.store.path.read_bytes()
        cfg['action_poses_file']=str(root/'actions.json')
        self.service.config['action_poses_file']=cfg['action_poses_file']
        Path(cfg['action_poses_file']).write_bytes(self.memory)
        self.source='20260920/pose_120000000_abcdef'
        folder=self.service.pose_estimator.data_root/self.source;folder.mkdir(parents=True)
        (folder/'pose_estimation_summary.json').write_text(json.dumps(dict(selected_target='basket',usable=True,
            localization=dict(target='basket',valid=True,right_shoulder_frame='right_arm_sdk_world',
                              point_right_shoulder_mm=[800,100,-500]))))
        (folder/'pose_estimation_request.json').write_text(json.dumps(dict(upper_body_joints_deg=[0]*6)))
        self.service.arm({});self.service.gripper_unlocked=True;self.robot.gripper_activate()
        self.robot.gripper_move(200)
        self.job=self.service.placement;self.job.poll_seconds=.001;self.job.timeout_seconds=.15
        self.start=self.robot.read_state()
    def tearDown(self):self.service.close();self.tmp.cleanup()
    def run_job(self):
        self.job.execute({'source_result_id':self.source});self.job.thread.join(3)
        self.assertFalse(self.job.thread.is_alive())
        return self.job.status()
    def test_full_sequence_exact_axes_reverse_order_and_original_start(self):
        with patch.object(self.job,'_pair',wraps=self.job._pair) as paired:
            result=self.run_job()
        self.assertEqual(result['phase'],'completed',result)
        self.assertEqual(paired.call_count,2)
        self.assertEqual([x[0] for x in self.robot.events],['memory','pose','pose','pose','joint','open','joint','pose','pose','pose','memory'])
        self.assertEqual(result['completed_moves'],8)
        self.assertEqual(result['total_moves'],8)
        poses=[x for x in self.robot.events if x[0]=='pose']
        self.assertEqual(poses[0],('pose',ARM,[400,70,300,180,-90,0]))
        self.assertEqual(poses[1],('pose',ARM,[450,70,300,180,-90,0]))
        self.assertEqual(poses[2],('pose','trunk',[300,0,600,0,0,0]))
        self.assertEqual(poses[3],poses[0])
        self.assertEqual(poses[4],('pose','trunk',self.start['poses']['trunk']))
        self.assertEqual(poses[5],('pose',ARM,[400,100,300,180,-90,0]))
        joint=[x[2] for x in self.robot.events if x[0]=='joint']
        self.assertEqual(joint,[[11,12,13,14,15,16,10],[11,12,13,14,15,16,40]])
        self.assertEqual(self.robot.read_state(),self.start)
        self.assertEqual(self.robot.gripper_status()['measured_position'],0)
        self.assertEqual(self.service.memory.store.path.read_bytes(),self.memory)
        self.assertEqual(self.robot.events[-1][1]['poses'][ARM],self.start['poses'][ARM])
    def test_reverse_translation_failure_prevents_direct_return(self):
        linear=self.job._linear
        calls=[]
        def fail_return(*args):
            calls.append(args)
            if len(calls)==2:raise BackendError('reverse translation failed')
            return linear(*args)
        with patch.object(self.job,'_linear',side_effect=fail_return):
            result=self.run_job()
        self.assertEqual(result['phase'],'failed')
        self.assertEqual(len(calls),2)
        self.assertEqual(sum(e[0]=='memory' for e in self.robot.events),1)
        self.assertEqual(self.robot.events[-1],('stop',))

    def test_bad_or_stale_basket_and_locked_gripper_never_start(self):
        for source in ('../escape','missing',self.source):
            if source==self.source:self.robot._state['joints_deg']['trunk'][0]=1
            with self.assertRaises(BackendError):self.job.execute({'source_result_id':source})
        self.robot._state['joints_deg']['trunk'][0]=0;self.service.gripper_unlocked=False
        with self.assertRaises(BackendError):self.job.execute({'source_result_id':self.source})
        self.assertEqual(self.robot.events,[])

    def test_open_failure_stops_and_does_not_retreat(self):
        self.robot.fail_open=True
        self.assertEqual(self.run_job()['phase'],'failed')
        self.assertEqual([e[0] for e in self.robot.events],['memory','pose','pose','pose','joint','open','stop'])
    def test_cancel_and_interlocks_prevent_following_moves(self):
        entered=threading.Event()
        def hold(*args,**kwargs):
            entered.set();self.job.cancelled.wait(1);self.job._check_cancel()
        with patch.object(self.job,'_pair',side_effect=hold):
            self.job.execute({'source_result_id':self.source});self.assertTrue(entered.wait(1))
            for action in (self.service._require_armed,lambda:self.service.scan_sequence.execute({}),
                           lambda:self.service.move_gripper({'position':0})):
                with self.assertRaises(BackendError):action()
            self.job.stop();self.job.thread.join(2)
        self.assertEqual(self.job.status()['phase'],'cancelled')
        self.assertNotIn('open',[e[0] for e in self.robot.events])
    def test_wait_requires_both_arrivals_and_fixed_head(self):
        target=copy.deepcopy(self.start['poses']);target[ARM][0]+=10;target['trunk'][0]+=10
        self.robot._state['poses'][ARM]=target[ARM]
        with self.assertRaisesRegex(BackendError,'超时'):
            self.job._wait(self.start,(ARM,'trunk'),{}, {ARM:target[ARM],'trunk':target['trunk']})
        self.robot._state['joints_deg']['head'][0]+=1
        with self.assertRaisesRegex(BackendError,'非本阶段'):
            self.job._wait(self.start,(ARM,'trunk'),{}, {})
    def test_locking_gripper_cancels_active_placement(self):
        self.job.job['active']=True
        self.service.gripper_unlocked=False
        with self.assertRaises(BackendError):self.job._check_cancel()
        self.job.job['active']=False

    def test_joint_limit_configuration_checked_before_joint_move(self):
        self.service.config['motion']['max_joint_step_deg'][ARM]=20
        with self.assertRaisesRegex(BackendError,'上限'):
            self.job._joint7(-30,self.start,dict(linear_mm_s=50,rotation_deg_s=6))
        self.assertFalse(self.robot.events)
    def test_http_start_status_stop_uses_saved_basket_source(self):
        import http.client
        from rokae_web.web import make_server
        entered=threading.Event()
        def hold(*args,**kwargs):
            entered.set();self.job.cancelled.wait(2);self.job._check_cancel()
        server=make_server('127.0.0.1',0,self.service,Path(__file__).resolve().parents[1]/'static')
        thread=threading.Thread(target=server.serve_forever,daemon=True);thread.start()
        def request(method,route,payload=None):
            connection=http.client.HTTPConnection('127.0.0.1',server.server_port,timeout=3)
            connection.request(method,route,body=json.dumps(payload or {}) if method=='POST' else None,
                               headers={'Content-Type':'application/json'})
            response=connection.getresponse();body=json.loads(response.read());connection.close()
            self.assertEqual(response.status,200,body);return body['data']
        try:
            with patch.object(self.job,'_pair',side_effect=hold):
                self.assertTrue(request('POST','/api/placement/start',{'source_result_id':self.source})['active'])
                self.assertTrue(entered.wait(1))
                self.assertTrue(request('GET','/api/status')['placement']['active'])
                request('POST','/api/placement/stop');self.job.thread.join(2)
                self.assertEqual(self.job.status()['phase'],'cancelled')
        finally:
            server.shutdown();server.server_close();thread.join(2)

    def test_stop_failure_latches_and_disarms(self):
        self.robot.fail_open=True
        with patch.object(self.robot,'stop_memory_motion',return_value=['stop failed']):
            result=self.run_job()
        self.assertTrue(result['stop_unconfirmed']);self.assertFalse(self.service.armed)
        with self.assertRaises(BackendError):self.job.ensure_idle()
        self.job.stop();self.job.ensure_idle()


class HardwarePairTests(unittest.TestCase):
    def setUp(self):
        from types import SimpleNamespace
        from rokae_web.sdk_wire import SDK
        from rokae_web.backends import XCoreRobotBackend
        self.fixture=PlacementTests();self.fixture.setUp();self.addCleanup(self.fixture.tearDown)
        self.job=self.fixture.job;self.backend=self.fixture.robot
        self.fixture.service.hardware_enabled=True
        self.events=[];self.commands={};self.cancel_append=False
        self.backend.soft_limit_status=lambda:{'joint_limits_deg':{m:[[-180,180]]*len(q) for m,q in self.fixture.start['joints_deg'].items()}}
        self.backend._load_sdk=lambda:SDK
        self.backend._pose_to_sdk=XCoreRobotBackend._pose_to_sdk
        current=SDK.CartesianPosition([.1,0,.6,0,0,0]);current.external=[0,0];current.confData=[1]*8
        def append(m,commands,identifier,ec):
            self.events.append(('append',m));self.commands[m]=commands[0]
            if self.cancel_append:self.job.cancelled.set()
        controllers={m:SimpleNamespace(cartPosture=lambda coordinate,ec:current,
                         moveAppend=lambda commands,identifier,ec,m=m:append(m,commands,identifier,ec)) for m in (ARM,'trunk')}
        self.backend._robot=lambda m:controllers[m]
        self.backend._call=lambda action,fn,*args:fn(*args,{})
        self.backend._prepare_motion=lambda robot,m,speed:self.events.append(('prepare',m))
        def dispatch(modules):
            self.assertEqual(set(self.commands),{ARM,'trunk'})
            self.events.append(('start',tuple(modules)))
            return dict(modules=modules,dispatch_skew_ms=0.1)
        self.backend.start_arms_synchronized=dispatch
        self.move=SimpleNamespace(steps=[dict(q=[1,2,3,4,5,6,20],angle=0,cart=SDK.CartesianPosition([.45,.07,.3,0,0,0]))],
                                 plan=lambda *args:None,check_fresh=lambda:None)
        self.pose=[450,70,300,180,-90,0];self.trunk=[300,0,600,0,0,0]
        self.speeds=dict(linear_mm_s=50,rotation_deg_s=6)
    def invoke(self):
        with patch('rokae_web.placement_sequence.HardwareMoveL',return_value=self.move),patch.object(self.job,'_wait',return_value=self.fixture.start) as wait:
            result=self.job._pair(self.pose,self.trunk,self.fixture.start,{},self.speeds)
        return wait
    def test_queues_native_movel_targets_then_starts_both_and_waits_both(self):
        import math
        wait=self.invoke()
        self.assertEqual(self.events,[('prepare',ARM),('append',ARM),('prepare','trunk'),('append','trunk'),('start',(ARM,'trunk'))])
        self.assertEqual(self.commands['trunk'].target.trans,[.3,0,.6])
        self.assertEqual(self.commands['trunk'].target.external,[0,0])
        self.assertEqual(self.commands['trunk'].target.confData,[1]*8)
        for command in self.commands.values():
            self.assertEqual(command.speed,50);self.assertAlmostEqual(command.rotSpeed,math.radians(6))
        self.assertEqual(wait.call_args.args[1],(ARM,'trunk'))
    def test_cancel_while_queueing_never_starts_either_controller(self):
        self.cancel_append=True
        with self.assertRaises(BackendError):self.invoke()
        self.assertEqual(self.events,[('prepare',ARM),('append',ARM)])
    def test_planning_failure_and_j7_limit_send_no_commands(self):
        def fail(*args):raise BackendError('unreachable')
        self.move.plan=fail
        with self.assertRaises(BackendError):self.invoke()
        self.assertFalse(self.events)
        self.move.plan=lambda *args:None;self.move.steps[0]['q'][6]=-160
        with self.assertRaisesRegex(BackendError,'软限位'):self.invoke()
        self.assertFalse(self.events)
