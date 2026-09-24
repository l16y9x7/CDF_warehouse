import copy
import json
import math
import threading
from pathlib import Path
from types import SimpleNamespace
import unittest
from unittest.mock import patch

from rokae_web.backends import BackendError, XCoreRobotBackend
from rokae_web.memory_points import MemoryPointStore
from rokae_web.placement_sequence import LEFT_ARM
from rokae_web.sdk_wire import SDK
import test_placement_sequence as fixtures


class BoxPlacementTests(unittest.TestCase):
    def setUp(self):
        self.fixture = fixtures.PlacementTests(); self.fixture.setUp()
        self.addCleanup(self.fixture.tearDown)
        self.service, self.robot, self.job = self.fixture.service, self.fixture.robot, self.fixture.job
        self.service.gripper_unlocked = False
        self.service.config['suction'] = dict(enabled=True, slave_id=1,
            open_outputs=[dict(address=0, value=True)], close_outputs=[dict(address=0, value=False)])
        self.pre = copy.deepcopy(self.fixture.start)
        self.pre['poses']['head'] = [0]*6
        self.pre['poses'][LEFT_ARM] = [400,40,-340,0,-70,-180]
        self.pre['arm_elbow_deg'][LEFT_ARM] = -50
        placed = copy.deepcopy(self.pre)
        placed['poses'][LEFT_ARM][2] -= 100
        placed['poses']['trunk'][0] += 100
        self.returned = copy.deepcopy(self.fixture.start)
        self.returned['poses']['head'] = [0]*6
        self.returned['poses'][LEFT_ARM] = [15,210,-367,-161,-67,-8]
        self.returned['joints_deg']['head'] = [1,2]
        store = MemoryPointStore(Path(self.service.config['action_poses_file']))
        for name,state in [('L2盒子预放置',self.pre),('L2盒子放置点',placed),('L2抓取',self.returned)]:
            store.save(name,state,'mock')
        self.fixture.service.memory.store.path.unlink()  # Business poses must survive teaching-point deletion.
        path = self.service.pose_estimator.data_root/self.fixture.source/'pose_estimation_summary.json'
        summary=json.loads(path.read_text())
        summary['localization'].update(left_shoulder_frame='left_arm_sdk_world',point_left_shoulder_mm=[800,-80,-500])
        path.write_text(json.dumps(summary))
        self.summary_path=path
        self.suction_failure=None
        def release(enabled, config):
            self.robot.events.append(('suction',enabled))
            self.assertEqual(self.robot.read_state()['poses'][LEFT_ARM][2],-440)
            self.assertEqual(self.robot.read_state()['poses']['trunk'][0],200)
            if self.suction_failure: return self.suction_failure
            return dict(commanded_open=enabled,confirmed=True)
        self.robot.suction_set=release
        body=self.robot.start_memory_head_trunk
        def body_start(plan,cancel):
            self.robot.events.append(('body',copy.deepcopy(plan['target'])))
            return body(plan,cancel)
        self.robot.start_memory_head_trunk=body_start

    def run_box(self):
        self.job.execute(dict(source_result_id=self.fixture.source,sku_typ='box'))
        self.job.thread.join(3)
        self.assertFalse(self.job.thread.is_alive())
        return self.job.status()

    def test_dynamic_left_y_trunk_then_lower_release_and_full_l2_return_without_gripper(self):
        with patch.object(self.robot,'gripper_status',side_effect=AssertionError('right gripper must not be read')), \
             patch.object(self.job,'_pair',side_effect=AssertionError('box must not dispatch a pair')):
            result=self.run_box()
        self.assertEqual(result['phase'],'completed',result)
        self.assertEqual([x[0] for x in self.robot.events],['pose','pose','pose','suction','memory','body'])
        self.assertEqual(self.robot.events[0],('pose',LEFT_ARM,[400,30,-340,0,-70,-180]))
        self.assertEqual(self.robot.events[1],('pose','trunk',[200,0,600,0,0,0]))
        self.assertEqual(self.robot.events[2],('pose',LEFT_ARM,[400,30,-440,0,-70,-180]))
        self.assertEqual(self.robot.events[3],('suction',False))
        self.assertEqual(result['completed_moves'],5)
        self.assertEqual(result['total_moves'],5)
        self.assertEqual(result['final_state']['poses']['left_arm'],self.returned['poses']['left_arm'])
        self.assertEqual(result['final_state']['joints_deg'],self.returned['joints_deg'])
        self.assertTrue(self.service._suction_result['confirmed'])

    def test_reference_y_is_not_hardcoded_and_speeds_and_seed_are_snapshotted(self):
        summary=json.loads(self.summary_path.read_text());summary['localization']['point_left_shoulder_mm'][1]=-33
        self.summary_path.write_text(json.dumps(summary))
        self.service.speed_mm_s,self.service.rotation_deg_s=72,9
        def first(pose,state,plane,speeds,**kw):
            self.assertEqual(pose,[400,77,-340,0,-70,-180])
            self.assertEqual(speeds,dict(linear_mm_s=72,rotation_deg_s=9))
            self.assertEqual(kw,dict(arm_module=LEFT_ARM,seed=-50))
            raise BackendError('dry planning failure')
        with patch.object(self.job,'_linear',side_effect=first):r=self.run_box()
        self.assertEqual(r['phase'],'failed');self.assertEqual(self.robot.events,[])

    def test_unconfirmed_release_never_returns_or_retries(self):
        self.suction_failure=dict(confirmed=False,commanded_open=False)
        r=self.run_box()
        self.assertEqual(r['phase'],'failed')
        self.assertEqual([x[0] for x in self.robot.events],['pose','pose','pose','suction','stop'])
        self.assertIn('未确认',r['message'])

    def test_cancel_before_trunk_does_not_lower_release_or_return(self):
        def cancel(*args,**kw):self.job.cancelled.set();self.job._check_cancel()
        with patch.object(self.job,'_trunk_linear',side_effect=cancel):r=self.run_box()
        self.assertEqual(r['phase'],'cancelled')
        self.assertEqual([x[0] for x in self.robot.events],['pose','stop'])

    def test_trunk_failure_does_not_lower_release_or_return(self):
        with patch.object(self.job,'_trunk_linear',side_effect=BackendError('trunk not reached')):r=self.run_box()
        self.assertEqual(r['phase'],'failed')
        self.assertEqual([x[0] for x in self.robot.events],['pose','stop'])

    def test_descent_failure_does_not_release_or_return(self):
        original=self.job._linear
        def lower(pose,*args,**kwargs):
            if pose[2] == -440:raise BackendError('descent failed')
            return original(pose,*args,**kwargs)
        with patch.object(self.job,'_linear',side_effect=lower):r=self.run_box()
        self.assertEqual(r['phase'],'failed')
        self.assertEqual(self.robot.events[:2],[('pose',LEFT_ARM,[400,30,-340,0,-70,-180]),
                                               ('pose','trunk',[200,0,600,0,0,0])])
        self.assertEqual([x[0] for x in self.robot.events],['pose','pose','stop'])

    def test_web_http_waits_for_trunk_arrival_before_left_descent(self):
        import http.client
        from rokae_web.web import make_server
        entered,allow_arrival=threading.Event(),threading.Event()
        original=self.job._wait
        def wait(start,moving,*args,**kwargs):
            if moving == ('trunk',):
                entered.set()
                if not allow_arrival.wait(2):raise BackendError('test arrival timeout')
            return original(start,moving,*args,**kwargs)
        server=make_server('127.0.0.1',0,self.service,Path(__file__).resolve().parents[1]/'static')
        thread=threading.Thread(target=server.serve_forever,daemon=True);thread.start()
        try:
            with patch.object(self.job,'_wait',side_effect=wait):
                conn=http.client.HTTPConnection(*server.server_address,timeout=3)
                conn.request('POST','/api/placement/start',json.dumps(
                    dict(source_result_id=self.fixture.source,sku_typ='box')),{'Content-Type':'application/json'})
                response=conn.getresponse();body=json.loads(response.read());conn.close()
                self.assertEqual(response.status,200,body)
                self.assertTrue(entered.wait(1))
                self.assertEqual(self.robot.events,[('pose',LEFT_ARM,[400,30,-340,0,-70,-180]),
                                                    ('pose','trunk',[200,0,600,0,0,0])])
                self.assertEqual(self.job.status()['stage'],'box_advance')
                allow_arrival.set();self.job.thread.join(3)
                self.assertEqual(self.job.status()['phase'],'completed',self.job.status())
                self.assertEqual(self.robot.events[2],('pose',LEFT_ARM,[400,30,-440,0,-70,-180]))
        finally:
            allow_arrival.set()
            if self.job.thread:self.job.thread.join(3)
            server.shutdown();server.server_close();thread.join(2)

    def test_missing_left_reference_never_substitutes_right(self):
        summary=json.loads(self.summary_path.read_text());summary['localization'].pop('left_shoulder_frame')
        self.summary_path.write_text(json.dumps(summary))
        with self.assertRaisesRegex(BackendError,'左肩'):self.run_box()
        self.assertFalse(self.robot.events)

    def test_inconsistent_calibration_and_stale_capture_never_move(self):
        path=Path(self.service.config['action_poses_file']);data=json.loads(path.read_text())
        point=next(x for x in data['points'] if x['name']=='L2盒子放置点')
        point['state']['poses'][LEFT_ARM][2]+=25;path.write_text(json.dumps(data))
        with self.assertRaisesRegex(BackendError,'标定不一致'):self.run_box()
        point['state']['poses'][LEFT_ARM][2]-=25;path.write_text(json.dumps(data))
        self.robot._state['joints_deg']['trunk'][0]+=1
        with self.assertRaisesRegex(BackendError,'躯干已偏离'):self.run_box()
        self.assertFalse(self.robot.events)

    def native_trunk_fixture(self, cancel_on_append=False):
        self.service.hardware_enabled=True
        self.robot.soft_limit_status=lambda:{'joint_limits_deg':{m:[[-180,180]]*len(q) for m,q in self.fixture.start['joints_deg'].items()}}
        self.robot._load_sdk=lambda:SDK;self.robot._pose_to_sdk=XCoreRobotBackend._pose_to_sdk
        self.robot._state['joints_deg']['head']=[2,-3]
        current=SDK.CartesianPosition([.1,0,.6,0,0,0]);current.external=[math.radians(2),math.radians(-3)];current.confData=[1]*8
        events=[];commands={}
        def append(items,identifier,ec):
            events.append(('append','trunk'));commands['trunk']=items[0]
            if cancel_on_append:self.job.cancelled.set()
        trunk=SimpleNamespace(cartPosture=lambda coord,ec:current,moveAppend=append,
                              moveStart=lambda ec:events.append(('start','trunk')))
        def controller(module):
            self.assertEqual(module,'trunk');return trunk
        self.robot._robot=controller
        self.robot._call=lambda label,fn,*args:fn(*args,{})
        self.robot._prepare_motion=lambda robot,module,speed:events.append(('prepare',module))
        return events,commands,current

    def test_native_trunk_only_movel_preserves_head_and_saved_speeds(self):
        events,commands,current=self.native_trunk_fixture()
        start=self.robot.read_memory_state()
        with patch('rokae_web.placement_sequence.HardwareMoveL',side_effect=AssertionError('no arm planning')), \
             patch.object(self.job,'_wait',return_value=start) as wait:
            self.job._trunk_linear([200,0,600,0,0,0],start,dict(linear_mm_s=72,rotation_deg_s=9))
        self.assertEqual(events,[('prepare','trunk'),('append','trunk'),('start','trunk')])
        self.assertEqual(wait.call_args.args[1],('trunk',))
        self.assertEqual(wait.call_args.args[3],{'trunk':[200,0,600,0,0,0]})
        self.assertEqual(wait.call_args.kwargs['arm'],LEFT_ARM)
        command=commands['trunk']
        self.assertEqual(command.speed,72);self.assertAlmostEqual(command.rotSpeed,math.radians(9))
        self.assertEqual(command.target.external,current.external)
        self.assertEqual(command.target.confData,current.confData)
        self.assertAlmostEqual(command.target.trans[0],.2)

    def test_cancel_after_trunk_append_never_starts(self):
        events,_,_=self.native_trunk_fixture(cancel_on_append=True)
        with self.assertRaisesRegex(BackendError,'已停止'):
            self.job._trunk_linear([200,0,600,0,0,0],self.robot.read_memory_state(),
                                   dict(linear_mm_s=72,rotation_deg_s=9))
        self.assertEqual(events,[('prepare','trunk'),('append','trunk')])

    def test_trunk_wait_rejects_arm_drift_or_incomplete_arrival(self):
        start=self.robot.read_memory_state()
        with self.assertRaisesRegex(BackendError,'超时'):
            self.job._wait(start,('trunk',),{}, {'trunk':[200,0,600,0,0,0]},arm=LEFT_ARM)
        self.robot._state['joints_deg'][LEFT_ARM][0]+=1
        with self.assertRaisesRegex(BackendError,'非本阶段'):
            self.job._wait(start,('trunk',),{}, {},arm=LEFT_ARM)


if __name__=='__main__':unittest.main()
