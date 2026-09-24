"""Offline tube-specific points, two photos, right-only moves and body sequencing."""
import copy
import json
from pathlib import Path
import threading
import unittest
from unittest.mock import patch

from rokae_web.backends import BackendError
from rokae_web.action_poses import tube_scan_turn_point
from rokae_web.memory_points import MemoryPointStore
from rokae_web.tube_scan import TubeScanMotion
import test_scan_sequence


class TubeScanTests(unittest.TestCase):
    def setUp(self):
        self.f=test_scan_sequence.ScanSequenceTests();self.f.setUp();self.addCleanup(self.f.tearDown)
        self.service,self.robot,self.scan=self.f.service,self.f.robot,self.f.scan
        root=Path(self.f.tmp.name)
        tools={arm:dict(end=[0.]*6,ref=[0.]*6) for arm in ('left_arm','right_arm','trunk')}
        conf={arm:[0]*8 for arm in ('left_arm','right_arm')}
        read=self.robot.read_state
        self.robot.read_memory_state=lambda:dict(read(),toolsets=copy.deepcopy(tools),arm_conf_data=copy.deepcopy(conf))
        self.f.body=self.robot.read_memory_state()
        actions=Path(self.service.config['action_poses_file'])
        data=json.loads(actions.read_text())
        for point in data['points']:
            point['state']['toolsets']=copy.deepcopy(tools)
            point['state']['arm_conf_data']=copy.deepcopy(conf)
        actions.write_text(json.dumps(data))
        self.service.config['torso_guard_file']=str(root/'guard.json')
        (root/'guard.json').write_text(json.dumps(dict(plane_offset_mm=20, elbow_radius_mm=65, margin_mm=10)))
        self.service.config['tube_scan_poses_file']=str(root/'tube.json')
        store=MemoryPointStore(root/'tube.json')
        for i in (1,2):
            state=copy.deepcopy(self.f.body);state['poses']['head']=[0]*6
            for arm in ('left_arm','right_arm'):
                state['joints_deg'][arm]=[float(i*20)]*7
                state['poses'][arm]=[200+i*10, 70-i*10, -300, 180, -60, 0]
            state['joints_deg']['trunk']=[90.]*4
            state['poses']['trunk']=[900,0,800,0,0,0]
            store.save(f'软管扫码{i}',state,'mock')
        # A is a distinct historical arrival, including all seven joints.
        # Keeping the original scan2 deliberately different prevents accidental replay.
        a=copy.deepcopy(state)
        a['poses']['right_arm']=[220,-50,-300,15,20,30]
        a['joints_deg']['right_arm']=[31,32,33,34,35,36,37]
        self.turn_state=copy.deepcopy(a)
        self.service.config['tube_scan_turn_file']=str(root/'turn.json')
        (root/'turn.json').write_text(json.dumps(dict(version=1,name='软管扫码翻转A',mode='mock',state=a)),encoding='utf8')
        self.events=[]
        old_arms=self.robot.start_memory_arms
        def arms(plan,cancel):
            self.events.append(('arms',copy.deepcopy(plan['target'])))
            plan.setdefault('synchronized', True)  # One-arm helper does not need a start barrier.
            return old_arms(plan,cancel)
        self.robot.start_memory_arms=arms
        old_pose=self.robot.move_pose
        def pose(module,target,speed,*args):
            self.events.append(('pose',module,list(target)))
            return old_pose(module,target,speed,*args)
        self.robot.move_pose=pose
        old_photo=self.f.camera.record_scan_rgb
        def photo(*args,**kw):
            self.events.append(('photo',args[1]))
            return old_photo(*args,**kw)
        self.f.camera.record_scan_rgb=photo
        self.robot.gripper_start_move=lambda *a:self.fail('scan must preserve gripper')
        self.robot.suction_set=lambda *a:self.fail('scan must preserve suction')
        self.service.memory.store.path.unlink()

    def run_tube(self):
        self.scan.execute({'sku_typ':'tube'})
        return self.f.finish()

    def test_scan1_right100_full_A_left100_photo_right100_return_and_body_plus100(self):
        result=self.run_tube();self.assertEqual(result['phase'],'completed',result)
        self.assertEqual([e[0] for e in self.events],['arms','photo','pose','arms','pose','photo','pose','arms','pose'])
        first,second=self.f.camera.events
        self.assertEqual(first[2]['joints_deg']['right_arm'],[20.]*7)
        self.assertEqual(second[2]['joints_deg']['right_arm'],[31,32,33,34,35,36,37])
        self.assertEqual(second[2]['joints_deg']['left_arm'],[20.]*7)
        self.assertEqual(self.events[2],('pose','right_arm',[210,-40,-300,180,-60,0]))
        self.assertEqual(self.events[3][1]['joints_deg']['right_arm'],[31,32,33,34,35,36,37])
        self.assertEqual(self.events[3][1]['poses']['right_arm'],[220,-50,-300,15,20,30])
        self.assertEqual(self.events[4],('pose','right_arm',[220,50,-300,15,20,30]))
        self.assertEqual(self.events[6],('pose','right_arm',[220,-50,-300,15,20,30]))
        target=list(self.f.body['poses']['trunk']);target[0]+=100
        self.assertEqual(self.events[-1],('pose','trunk',target))
        for photo in self.f.camera.events:
            self.assertEqual(photo[2]['joints_deg']['head'],self.f.body['joints_deg']['head'])
            self.assertEqual(photo[2]['poses']['trunk'],self.f.body['poses']['trunk'])
        self.assertEqual(result['final_state']['joints_deg']['left_arm'],[6.]*7)
        self.assertEqual(result['final_state']['joints_deg']['right_arm'],[6.]*7)
        self.assertEqual(result['final_state']['joints_deg']['head'],self.f.body['joints_deg']['head'])
        self.assertEqual(len({p['directory'] for p in result['photos']}),1)
        self.assertEqual(result['total_photos'],2)

    def test_missing_tube_calibration_does_not_fall_back_to_bottle_scan_points(self):
        Path(self.service.config['tube_scan_poses_file']).unlink()
        with self.assertRaisesRegex(BackendError,'软管扫码1'):self.run_tube()
        self.assertEqual(self.events,[])

    def test_arrival_telemetry_without_arm_tools_is_recaptured_before_right_motion(self):
        tools={arm:dict(end=[0.]*6,ref=[0.]*6) for arm in ('left_arm','right_arm','trunk')}
        read=self.robot.read_state
        self.robot.read_memory_state=lambda:dict(read(),toolsets=copy.deepcopy(tools))
        for key in ('tube_scan_poses_file','action_poses_file','tube_scan_turn_file'):
            path=Path(self.service.config[key]);data=json.loads(path.read_text())
            for point in data.get('points',[data]):point['state']['toolsets']=copy.deepcopy(tools)
            path.write_text(json.dumps(data))
        result=self.run_tube()
        self.assertEqual(result['phase'],'completed',result)
        self.assertEqual(result['completed_points'],2)

    def test_first_offset_failure_keeps_first_photo_and_never_moves_scan2_or_body(self):
        with patch.object(TubeScanMotion,'_linear',side_effect=BackendError('unreachable')):
            result=self.run_tube()
        self.assertEqual(result['phase'],'failed')
        self.assertEqual([e[0] for e in self.events],['arms','photo'])
        self.assertEqual(result['completed_points'],1)

    def test_second_capture_failure_never_offsets_returns_or_advances(self):
        self.f.camera.fail_index=2
        result=self.run_tube();self.assertEqual(result['phase'],'failed')
        self.assertEqual([e[0] for e in self.events],['arms','photo','pose','arms','pose','photo'])
        self.assertEqual(result['completed_points'],1)

    def test_waits_for_both_arms_before_first_photo_and_cancel_stops(self):
        self.robot.hold_arms=True
        self.scan.execute({'sku_typ':'tube'});self.assertTrue(self.robot.arm_started.wait(1))
        self.robot.arrive('right_arm')
        self.assertEqual(self.f.camera.events,[])
        self.scan.stop();result=self.f.finish()
        self.assertEqual(result['phase'],'cancelled')
        self.assertEqual(self.f.camera.events,[])

    def test_return_wait_must_complete_before_body_advance(self):
        original=self.scan._arms
        def fail_return(target,speeds,label,body,*args,**kw):
            if label.startswith('返回'):raise BackendError('return not reached')
            return original(target,speeds,label,body,*args,**kw)
        with patch.object(self.scan,'_arms',side_effect=fail_return):result=self.run_tube()
        self.assertEqual(result['phase'],'failed')
        self.assertFalse(any(e[:2]==('pose','trunk') for e in self.events))

    def test_A_failure_stops_before_left_shift_second_photo_or_return(self):
        with patch.object(TubeScanMotion,'_memory_right',side_effect=BackendError('A failed')):
            result=self.run_tube()
        self.assertEqual(result['phase'],'failed')
        self.assertEqual([e[0] for e in self.events],['arms','photo','pose'])
        self.assertEqual(result['completed_points'],1)

    def test_left_shift_failure_never_takes_second_photo_or_returns(self):
        original=TubeScanMotion._linear
        def move(motion,pose,*args,**kwargs):
            if self.scan.job['stage']=='tube_scan2_approach':raise BackendError('approach failed')
            return original(motion,pose,*args,**kwargs)
        with patch.object(TubeScanMotion,'_linear',move):result=self.run_tube()
        self.assertEqual(result['phase'],'failed')
        self.assertEqual([e[0] for e in self.events],['arms','photo','pose','arms'])

    def test_missing_or_bad_A_rejected_before_first_motion(self):
        path=Path(self.service.config['tube_scan_turn_file']);data=json.loads(path.read_text())
        for field in ('joints_deg','arm_conf_data','pose_frames','toolsets'):
            changed=copy.deepcopy(data);changed['state'][field].pop('right_arm')
            path.write_text(json.dumps(changed))
            with self.assertRaisesRegex(BackendError,'翻转A'):self.run_tube()
        path.unlink()
        with self.assertRaisesRegex(BackendError,'翻转A'):self.run_tube()
        self.assertEqual(self.events,[])

    def test_A_load_returns_independent_state_and_original_scan2_is_unused(self):
        point=tube_scan_turn_point(self.service.config,'mock')
        point['state']['joints_deg']['right_arm'][0]=999
        self.assertEqual(tube_scan_turn_point(self.service.config,'mock')['state']['joints_deg']['right_arm'][0],31)
        path=Path(self.service.config['tube_scan_poses_file']);data=json.loads(path.read_text())
        data['points']=[p for p in data['points'] if p['name']=='软管扫码1'];path.write_text(json.dumps(data))
        self.assertEqual(self.run_tube()['phase'],'completed')
