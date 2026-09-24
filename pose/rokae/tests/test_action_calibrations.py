"""Offline action independence, suction ordering and one-image box scan."""
import copy
import json
import tempfile
import threading
from pathlib import Path
import unittest
from unittest.mock import patch

from rokae_web.action_poses import action_point, box_heights
from rokae_web.backends import BackendError
import test_scan_sequence
import test_left_box_grasp


class IndependentScanTests(unittest.TestCase):
    def setUp(self):
        self.f=test_scan_sequence.ScanSequenceTests();self.f.setUp()
        self.addCleanup(self.f.tearDown)
        self.service=self.f.service

    def test_box_uses_fixed_arms_right_camera_then_arms_then_trunk_after_teaching_deletion(self):
        self.service.memory.store.path.unlink()
        self.service.robot.suction_set=lambda *a:self.fail('scan must never change suction')
        self.service._suction_result=dict(commanded_open=True,confirmed=True)
        self.f.scan.execute({'sku_typ':'box'})
        result=self.f.finish()
        self.assertEqual(result['phase'],'completed',result)
        self.assertEqual(result['total_photos'],1)
        self.assertEqual(len(result['photos']),1)
        self.assertEqual(self.f.camera.last_camera,'right_wrist')
        self.assertEqual(self.f.robot.calls,['arms','arms','head_trunk'])
        self.assertEqual(self.f.camera.events[0][2]['joints_deg']['left_arm'],[9.]*7)
        self.assertEqual(self.f.camera.events[0][2]['joints_deg']['right_arm'],[8.]*7)
        for target in self.f.robot.targets:
            for module in ('head','trunk'):
                self.assertEqual(target['joints_deg'][module],self.f.body['joints_deg'][module])
        for arm in ('left_arm','right_arm'):
            self.assertEqual(self.f.robot.targets[-1]['joints_deg'][arm],[6.]*7)
        self.assertEqual(self.f.robot.body_target['joints_deg']['trunk'],[6.]*4)
        self.assertTrue(self.service._suction_result['commanded_open'])

    def test_bottle_still_has_five_photos_after_teaching_deletion(self):
        self.service.memory.store.path.unlink()
        self.f.scan.execute({'sku_typ':'bottle'})
        result=self.f.finish()
        self.assertEqual(result['phase'],'completed',result)
        self.assertEqual(len(result['photos']),5)
        self.assertEqual(self.f.camera.last_camera,'left_wrist')

    def test_fixed_prepare_executes_without_teaching_store_and_missing_fixed_file_fails(self):
        point=action_point(self.service.config,'L2抓取','mock')
        self.service.memory.store.path.unlink()
        self.service.memory.poll_seconds=.003
        self.service.robot.start_memory_arms=lambda plan,cancel: test_scan_sequence.SequencedRobot.start_memory_arms(self.f.robot,plan,cancel)
        self.service.memory.execute({},prepared_point=point)
        self.service.memory.thread.join(2)
        self.assertEqual(self.service.memory.status()['phase'],'completed',self.service.memory.status())
        with self.assertRaises(BackendError):action_point(self.service.config,'L2抓取','hardware')
        Path(self.service.config['action_poses_file']).unlink()
        with self.assertRaises(BackendError):action_point(self.service.config,'L2抓取','mock')


class BoxSuctionFailures(unittest.TestCase):
    def setUp(self):
        self.f=test_left_box_grasp.LeftSequenceTests();self.f.setUp()
        self.addCleanup(self.f.doCleanups)

    def test_close_not_confirmed_prevents_all_movement(self):
        self.f.service.robot.suction_set=lambda *a:dict(commanded_open=False,confirmed=False)
        result=self.f.execute()
        self.assertEqual(result['phase'],'failed',result)
        self.assertFalse(any(e[1]=='start' for e in self.f.f.events))

    def test_open_failure_prevents_lift_and_retreat_without_automatic_close(self):
        def suction(opened,config):
            self.f.suction.append(opened)
            if opened:raise BackendError('open reply lost')
            return dict(commanded_open=False,confirmed=True)
        self.f.service.robot.suction_set=suction
        result=self.f.execute()
        self.assertEqual(result['phase'],'failed',result)
        self.assertEqual(self.f.suction,[False,True])
        self.assertEqual(self.f.f.events.count(('left_arm','start')),3)
        self.assertNotIn(('trunk_retreat','start'),self.f.f.events)

    def test_cancel_after_open_keeps_suction_on(self):
        original=self.f.service.robot.suction_set
        def suction(opened,config):
            result=original(opened,config)
            if opened:self.f.service.grasp_test.cancel.set()
            return result
        self.f.service.robot.suction_set=suction
        result=self.f.execute()
        self.assertEqual(result['phase'],'cancelled',result)
        self.assertEqual(self.f.suction,[(False,0),(True,3)])
        self.assertNotIn(('trunk_retreat','start'),self.f.f.events)
        self.assertTrue(self.f.service._suction_result['commanded_open'])

    def test_heights_fail_closed_for_wrong_frame_order_and_missing_file(self):
        path=Path(self.f.f.temp.name)/'box.json'
        config={'box_grasp_file':str(path)}
        with self.assertRaises(BackendError):box_heights(config)
        data=copy.deepcopy(test_left_box_grasp.CALIBRATION)
        path.write_text(json.dumps(data));self.assertEqual(box_heights(config),data)
        data['heights_mm']['lift']=750
        path.write_text(json.dumps(data))
        with self.assertRaises(BackendError):box_heights(config)


class RightWristCaptureTests(unittest.TestCase):
    def test_right_scan_captures_right_wrist_and_preserves_shared_path(self):
        from rokae_web.http_camera import HttpCameraManager
        from rokae_web.rgb_recording import RgbRecordingStore
        with tempfile.TemporaryDirectory() as tmp:
            camera=HttpCameraManager.__new__(HttpCameraManager)
            camera.rgb_store=RgbRecordingStore(Path(tmp)/'left')
            camera.right_rgb_store=RgbRecordingStore(Path(tmp)/'right')
            group=camera.begin_scan_recording(camera_id='right_wrist')
            metadata={'color':{'path':'/shared/frames/example/rgb.jpg'},'capture_id':'example'}
            with patch('rokae_web.http_camera.capture_rgb',return_value=(b'jpeg',metadata)) as capture:
                result=camera.record_scan_rgb(group,1,threading.Event(),camera_id='right_wrist')
            capture.assert_called_once_with('right_wrist',5,return_metadata=True)
            self.assertEqual(result['camera_id'],'right_wrist')
            self.assertEqual(result['image_path'],metadata['color']['path'])
            self.assertTrue(Path(result['path']).is_relative_to(Path(tmp)/'right'))
            self.assertFalse((Path(tmp)/'left').exists())


if __name__=='__main__':unittest.main()
