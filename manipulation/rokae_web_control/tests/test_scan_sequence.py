import copy
import http.client
import json
from pathlib import Path
import tempfile
import threading
import time
import unittest

from rokae_web.backends import BackendError, JOINT_COUNTS, MockChassisBackend
from rokae_web.config import DEFAULT_CONFIG
from rokae_web.rgb_recording import RgbRecordingStore
from rokae_web.ros_camera import RosColorStream
from rokae_web.scan_sequence import POINT_NAMES
from rokae_web.service import ControlService
from test_memory_points import SequencedRobot
from test_ros_camera import image, config as camera_config


class ScanCamera:
    def __init__(self, root, robot):
        self.store, self.robot = RgbRecordingStore(root), robot
        self.events = []
        self.fail_index = None

    def status(self):
        return {name: {'enabled': True, 'available': True} for name in ('left_wrist','right_wrist')}

    def begin_scan_recording(self, camera_id='left_wrist'):
        return self.store.begin_scan()

    def record_scan_rgb(self, group, index, cancel, camera_id='left_wrist'):
        if cancel.is_set():
            raise BackendError('cancelled')
        if index == self.fail_index:
            raise BackendError('camera failed')
        state = self.robot.read_state()
        self.events.append(('photo', index, copy.deepcopy(state)))
        self.last_camera=camera_id
        assert all(v == 'mock-idle' for v in state['operation_state'].values())
        return self.store.save_scan(group, index, b'fake-jpeg')

    def close(self):
        pass


class ScanRobot(SequencedRobot):
    def __init__(self):
        super().__init__()
        self.targets = []
        self.stop_failure = False

    def start_memory_arms(self, plan, cancel):
        assert plan['synchronized'] is True
        self.targets.append(copy.deepcopy(plan['target']))
        super().start_memory_arms(plan, cancel)

    def start_memory_head_trunk(self, plan, cancel):
        self.body_target = copy.deepcopy(plan['target'])
        super().start_memory_head_trunk(plan, cancel)

    def stop_memory_motion(self):
        result = super().stop_memory_motion()
        return ['stop failed'] if self.stop_failure else result


class ScanSequenceTests(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.robot = ScanRobot()
        self.camera = ScanCamera(Path(self.tmp.name) / 'left_wrist_rgb', self.robot)
        cfg = copy.deepcopy(DEFAULT_CONFIG)
        cfg['memory_points'] = {'file': str(Path(self.tmp.name) / 'points.json')}
        self.service = ControlService(cfg, self.robot, MockChassisBackend(), False, camera_backend=self.camera)
        for i, name in enumerate(POINT_NAMES, 1):
            state = self.robot.read_state()
            state['poses']['head'] = [99] * 6
            for module, count in JOINT_COUNTS.items():
                state['joints_deg'][module] = [float(i)] * count
                state['poses'][module] = [float(i * 10)] * 6
            self.service.memory.store.save(name, state, 'mock')
        box = self.robot.read_state();box['poses']['head']=[0]*6
        box['joints_deg']['left_arm']=[9.]*7;box['joints_deg']['right_arm']=[8.]*7
        self.service.memory.store.save('盒子扫码',box,'mock')
        self.service.config['action_poses_file']=str(Path(self.tmp.name)/'actions.json')
        Path(self.service.config['action_poses_file']).write_bytes(self.service.memory.store.path.read_bytes())
        self.before = self.service.memory.store.path.read_bytes()
        self.body = self.robot.read_state()
        self.scan = self.service.scan_sequence
        self.scan.poll_seconds, self.scan.timeout_seconds = .003, .5
        self.service.arm({})

    def tearDown(self):
        self.service.close()
        self.tmp.cleanup()

    def finish(self):
        self.scan.thread.join(3)
        self.assertFalse(self.scan.thread.is_alive())
        return self.scan.status()

    def test_five_points_same_folder_absolute_y_then_arms_then_trunk(self):
        self.scan.execute({})
        job = self.finish()
        self.assertEqual(job['phase'], 'completed', job)
        self.assertEqual(self.robot.calls, ['arms'] * 7 + ['head_trunk'])
        self.assertEqual([p[1] for p in self.camera.events], [1, 2, 3, 4, 5])
        for i, event in enumerate(self.camera.events, 1):
            for arm in ('left_arm', 'right_arm'):
                self.assertEqual(event[2]['joints_deg'][arm], [float(i)] * 7)
        for target in self.robot.targets:
            for module in ('head', 'trunk'):
                self.assertEqual(target['joints_deg'][module], self.body['joints_deg'][module])
        for arm, y in [('left_arm', 50), ('right_arm', -50)]:
            self.assertEqual(self.robot.targets[5]['poses'][arm], [50., y, 50., 50., 50., 50.])
            self.assertEqual(self.robot.targets[6]['joints_deg'][arm], [6.] * 7)
        self.assertEqual(self.robot.body_target['joints_deg']['head'], self.body['joints_deg']['head'])
        self.assertEqual(self.robot.body_target['joints_deg']['trunk'], [6.] * 4)
        folder = Path(job['directory'])
        self.assertEqual(folder.parent.parent, self.camera.store.root)
        self.assertEqual(sorted(p.name for p in folder.iterdir()), [f'扫码{i}.jpg' for i in range(1, 6)])
        self.assertEqual(len({p['directory'] for p in job['photos']}), 1)
        self.assertEqual(self.service.memory.store.path.read_bytes(), self.before)
        self.scan.execute({})
        again = self.finish()
        self.assertEqual(again['phase'], 'completed')
        self.assertNotEqual(again['directory'], job['directory'])

    def test_waits_for_both_arms_and_cancel_prevents_capture_and_next_stage(self):
        self.robot.hold_arms = True
        self.scan.execute({})
        self.assertTrue(self.robot.arm_started.wait(1))
        self.robot.arrive('left_arm')
        time.sleep(.04)
        self.assertEqual(self.camera.events, [])
        self.scan.stop()
        job = self.finish()
        self.assertEqual(job['phase'], 'cancelled')
        self.assertEqual(self.robot.calls, ['arms', 'stop'])
        self.assertEqual(self.camera.events, [])

    def test_capture_failure_stops_without_return_motion_and_retains_partial_set(self):
        self.camera.fail_index = 3
        self.scan.execute({})
        job = self.finish()
        self.assertEqual(job['phase'], 'failed')
        self.assertEqual(job['completed_points'], 2)
        self.assertEqual(self.robot.calls, ['arms'] * 3 + ['stop'])
        self.assertEqual(sorted(p.name for p in Path(job['directory']).iterdir()), ['扫码1.jpg', '扫码2.jpg'])

    def test_timeout_does_not_take_photo_and_blocks_other_controls_while_active(self):
        self.robot.hold_arms = True
        self.scan.timeout_seconds = .08
        self.scan.execute({})
        self.assertTrue(self.robot.arm_started.wait(1))
        for action in (lambda: self.scan.execute({}), lambda: self.service._require_armed(),
                       lambda: self.service.memory.save({'name': 'new'})):
            with self.assertRaises(BackendError):
                action()
        self.assertEqual(self.finish()['phase'], 'failed')
        self.assertEqual(self.camera.events, [])

    def test_failed_start_stops_both_and_unconfirmed_stop_latches_interlock(self):
        self.robot.fail_arms, self.robot.stop_failure = True, True
        self.scan.execute({})
        job = self.finish()
        self.assertTrue(job['stop_unconfirmed'])
        self.assertFalse(self.service.armed)
        with self.assertRaises(BackendError):
            self.scan.ensure_idle()
        self.robot.stop_failure = False
        self.scan.stop()
        self.scan.ensure_idle()

    def test_missing_point_or_locked_control_cannot_start(self):
        with self.assertRaises(BackendError):
            self.scan.execute({'target': 'external'})
        self.service.disarm()
        with self.assertRaises(ValueError):
            self.scan.execute({})
        self.service.arm({})
        point = self.service.memory.store.list()[0]
        from rokae_web.memory_points import MemoryPointStore
        MemoryPointStore(self.service.config['action_poses_file']).delete(point['id'], point['revision'])
        with self.assertRaisesRegex(BackendError, '标定位姿'):
            self.scan.execute({})
        self.assertEqual(self.robot.calls, [])

    def test_fixed_body_movement_aborts(self):
        self.robot.hold_arms = True
        self.scan.execute({})
        self.assertTrue(self.robot.arm_started.wait(1))
        self.robot._state['joints_deg']['head'][0] += 1
        self.assertEqual(self.finish()['phase'], 'failed')
        self.assertEqual(self.camera.events, [])

    def test_http_start_status_stop_with_fake_robot(self):
        from rokae_web.web import make_server
        self.robot.hold_arms = True
        server = make_server('127.0.0.1', 0, self.service, Path(__file__).resolve().parents[1] / 'static')
        thread = threading.Thread(target=server.serve_forever, daemon=True)
        thread.start()
        def request(method, route):
            connection = http.client.HTTPConnection('127.0.0.1', server.server_port, timeout=3)
            connection.request(method, route, body='{}' if method == 'POST' else None,
                               headers={'Content-Type': 'application/json'})
            response = connection.getresponse()
            data = json.loads(response.read())
            connection.close()
            self.assertEqual(response.status, 200, data)
            return data['data']
        try:
            self.assertTrue(request('POST', '/api/scan-sequence/start')['active'])
            self.assertTrue(request('GET', '/api/status')['scan_sequence']['active'])
            request('POST', '/api/scan-sequence/stop')
            self.assertEqual(self.finish()['phase'], 'cancelled')
        finally:
            server.shutdown()
            server.server_close()
            thread.join(2)


class OffsetTests(unittest.TestCase):
    def test_hardware_offset_preserves_tcp_axes_elbow_and_configuration(self):
        import math
        from test_memory_motion import MemoryMotionTests
        from rokae_web.scan_sequence import offset_target
        fixture = MemoryMotionTests()
        fixture.setUp()
        saved = copy.deepcopy(fixture.target)
        seen = {}
        for name in ('left_arm', 'right_arm'):
            robot = fixture.backend._robot(name)
            def ik(cart, tool, ec, name=name):
                seen[name] = cart
                return [math.radians(2)] * 7
            from types import SimpleNamespace
            robot.model = lambda ik=ik: SimpleNamespace(calcIk=ik)
        result = offset_target(fixture.backend, saved, saved, fixture.cancel)
        for name, y in [('left_arm', 50), ('right_arm', -50)]:
            expected = list(saved['poses'][name])
            expected[1] = y
            self.assertEqual(result['poses'][name], expected)
            self.assertAlmostEqual(seen[name].values[1], y / 1000)
            self.assertEqual(seen[name].confData, saved['arm_conf_data'][name])
            self.assertAlmostEqual(seen[name].elbow, math.radians(saved['arm_elbow_deg'][name]))
            self.assertEqual(result['joints_deg'][name], [2.] * 7)
        self.assertEqual(saved, fixture.target)
        self.assertFalse(fixture.events)


class ScanFrameTests(unittest.TestCase):
    def test_cached_frame_is_rejected_and_fresh_frame_is_encoded(self):
        stream = RosColorStream('left_wrist', camera_config())
        old = image()
        stream.receive('color', old)
        cancel = threading.Event()
        with self.assertRaisesRegex(BackendError, '超时'):
            stream.fresh_jpeg(.03, 80, cancel)
        timer = threading.Timer(.03, lambda: stream.receive('color', image()))
        timer.start()
        jpeg, metadata = stream.fresh_jpeg(.5, 80, cancel)
        timer.join()
        self.assertTrue(jpeg.startswith(b'\xff\xd8'))
        self.assertEqual(metadata['sequence'], 2)
        cancel.set()
        with self.assertRaisesRegex(BackendError, '停止'):
            stream.fresh_jpeg(.5, 80, cancel)

    def test_group_cannot_escape_root_or_overwrite_and_normal_recording_unchanged(self):
        with tempfile.TemporaryDirectory() as root:
            store = RgbRecordingStore(Path(root) / 'left_wrist_rgb')
            group = store.begin_scan()
            self.assertNotEqual(group, store.begin_scan())
            store.save_scan(group, 1, b'first')
            with self.assertRaises(FileExistsError):
                store.save_scan(group, 1, b'overwrite')
            with self.assertRaises(ValueError):
                store.save_scan({'directory': root}, 1, b'outside')
            normal = store.save(b'normal')
            self.assertEqual(Path(normal['path']).parent.parent, store.root)


if __name__ == '__main__':
    unittest.main()
