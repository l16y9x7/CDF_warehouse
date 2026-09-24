from __future__ import annotations

import copy
import json
import tempfile
import threading
import time
import unittest
from pathlib import Path
from types import SimpleNamespace as NS
from unittest.mock import patch

import numpy as np

from rokae_web.backends import BackendError, MockChassisBackend, MockRobotBackend
from rokae_web.config import DEFAULT_CONFIG
from rokae_web.ros_camera import (
    ExistingCameraRecovery, RosCameraManager, RosCameraStream, decode_color, decode_depth_mm,
)
from rokae_web.service import ControlService, ValidationError


def header(timestamp: int, frame: str = 'head_color_optical_frame'):
    return NS(stamp=NS(sec=timestamp // 1_000_000_000, nanosec=timestamp % 1_000_000_000), frame_id=frame)


def image(encoding='rgb8', values=None, timestamp=None, padding=0, bigendian=False):
    if values is None:
        values = np.full((2, 3, 3), [10, 20, 30], dtype=np.uint8)
    dtype = values.dtype.newbyteorder('>' if bigendian else '<')
    array = values.astype(dtype)
    rows = [row.tobytes() + b'\x00' * padding for row in array]
    return NS(header=header(time.time_ns() if timestamp is None else timestamp), height=array.shape[0],
              width=array.shape[1], encoding=encoding, is_bigendian=bigendian,
              step=len(rows[0]), data=b''.join(rows))


def info(timestamp):
    return NS(header=header(timestamp), width=3, height=2,
              k=[600., 0., 1., 0., 601., 1., 0., 0., 1.], d=[0.] * 5,
              distortion_model='plumb_bob')


def config():
    return dict(width=3, height=2, fps=15,
                topics=dict(color='/color', depth='/aligned_depth', info='/camera_info'))


def feed(stream, timestamp=None, depth_frame=None):
    timestamp = time.time_ns() if timestamp is None else timestamp
    color = image(timestamp=timestamp)
    depth = image('16UC1', np.full((2, 3), 1234, np.uint16), timestamp=timestamp)
    if depth_frame is not None:
        depth.header.frame_id = depth_frame
    for name, msg in [('depth', depth), ('info', info(timestamp)), ('color', color)]:
        stream.receive(name, msg)


class RosImageTests(unittest.TestCase):
    def test_rgb_to_bgr_with_row_padding(self):
        result = decode_color(image(padding=7))
        np.testing.assert_array_equal(result[1, 2], [30, 20, 10])
        self.assertEqual(result.shape, (2, 3, 3))

    def test_bgr_rgba_and_mono(self):
        np.testing.assert_array_equal(decode_color(image('bgr8'))[0, 0], [10, 20, 30])
        rgba = image('rgba8', np.full((2, 3, 4), [1, 2, 3, 255], np.uint8))
        np.testing.assert_array_equal(decode_color(rgba)[0, 0], [3, 2, 1])
        mono = image('mono8', np.full((2, 3), 7, np.uint8))
        np.testing.assert_array_equal(decode_color(mono)[0, 0], [7, 7, 7])

    def test_depth_u16_mm_big_endian_padding(self):
        source = np.asarray([[0, 500, 3000], [1234, 65535, 42]], np.uint16)
        result = decode_depth_mm(image('16UC1', source, padding=2, bigendian=True))
        np.testing.assert_array_equal(result, source)
        self.assertEqual(result.dtype, np.dtype('float32'))

    def test_depth_float_meters_invalid_values(self):
        source = np.asarray([[0, 1.25, np.nan], [-1, np.inf, 2.5]], np.float32)
        result = decode_depth_mm(image('32FC1', source, bigendian=True))
        np.testing.assert_array_equal(result, [[0, 1250, 0], [0, 0, 2500]])

    def test_rejects_invalid_encoding_and_payload(self):
        for msg in (image('yuyv'), image('16UC3')):
            with self.assertRaises(BackendError):
                decode_color(msg)
        msg = image()
        msg.step -= 1
        with self.assertRaises(BackendError):
            decode_color(msg)
        with self.assertRaises(BackendError):
            decode_depth_mm(image('8UC1', np.zeros((2, 3), np.uint8)))


class RosPairTests(unittest.TestCase):
    def setUp(self):
        self.stream = RosCameraStream('head', config(), stale_seconds=2.)

    def tearDown(self):
        self.stream.close()

    def test_exact_three_way_pair_and_metadata(self):
        feed(self.stream)
        snapshot = self.stream.snapshot(timeout=.01)
        self.assertEqual(snapshot.sequence, 1)
        self.assertEqual(snapshot.color_timestamp_ms, snapshot.depth_timestamp_ms)
        self.assertEqual(snapshot.camera_info['source'], 'ros2')
        self.assertFalse(snapshot.camera_info['original_depth_timestamp_available'])
        self.assertEqual(snapshot.camera_info['color_intrinsics']['fx'], 600.)
        self.assertTrue(self.stream.status()['enabled'])
        np.testing.assert_array_equal(snapshot.rgb_bgr[0, 0], [30, 20, 10])
        self.assertEqual(snapshot.depth_aligned_mm[0, 0], 1234.)

    def test_different_timestamps_do_not_pair(self):
        timestamp = time.time_ns()
        self.stream.receive('color', image(timestamp=timestamp))
        self.stream.receive('depth', image('16UC1', np.zeros((2, 3), np.uint16), timestamp=timestamp+1))
        self.stream.receive('info', info(timestamp))
        with self.assertRaises(BackendError):
            self.stream.snapshot(timeout=.01)

    def test_same_timestamp_but_unaligned_frame_is_rejected(self):
        feed(self.stream, depth_frame='depth_optical_frame')
        self.assertFalse(self.stream.status()['enabled'])
        with self.assertRaisesRegex(BackendError, '光学坐标系'):
            self.stream.snapshot(timeout=.01)

    def test_bad_dimensions_and_intrinsics_are_rejected(self):
        timestamp = time.time_ns()
        feed(self.stream, timestamp)
        for bad_info in (info(timestamp+1), info(timestamp+2)):
            if bad_info.header.stamp.nanosec == (timestamp+1) % 1_000_000_000:
                bad_info.width = 9
            else:
                bad_info.k[0] = 0
            t = bad_info.header.stamp.sec*1_000_000_000 + bad_info.header.stamp.nanosec
            self.stream.receive('info', bad_info)
            self.stream.receive('color', image(timestamp=t))
            self.stream.receive('depth', image('16UC1', np.zeros((2, 3), np.uint16), timestamp=t))
            self.assertTrue(self.stream.error)
        self.assertEqual(self.stream.sequence, 1)

    def test_stale_and_far_future_messages_are_rejected(self):
        for timestamp in (time.time_ns()-3_000_000_000, time.time_ns()+1_000_000_000):
            feed(self.stream, timestamp)
        self.assertFalse(self.stream.status()['enabled'])

    def test_pair_becomes_stale_even_if_previously_ready(self):
        feed(self.stream)
        self.stream.received_monotonic -= 3
        self.assertFalse(self.stream.status()['enabled'])
        with self.assertRaises(BackendError):
            self.stream.snapshot(timeout=.01)

    def test_fresh_capture_waits_for_post_request_pair(self):
        feed(self.stream)
        def later():
            time.sleep(.03)
            feed(self.stream)
        thread = threading.Thread(target=later)
        thread.start()
        try:
            snapshot = self.stream.snapshot(fresh=True, timeout=.5)
            self.assertEqual(snapshot.sequence, 2)
        finally:
            thread.join()

    def test_newly_received_pre_request_pair_is_not_current(self):
        timestamp = time.time_ns()-500_000_000
        def later():
            time.sleep(.02)
            feed(self.stream, timestamp)
        thread = threading.Thread(target=later)
        thread.start()
        try:
            with self.assertRaises(BackendError):
                self.stream.snapshot(fresh=True, timeout=.08)
        finally:
            thread.join()

    def test_bounded_cache_and_closed_waiters(self):
        for offset in range(30):
            self.stream.receive('color', image(timestamp=time.time_ns()+offset))
        self.assertLessEqual(len(self.stream.pending['color']), 8)
        self.stream.close()
        with self.assertRaisesRegex(BackendError, '关闭'):
            self.stream.snapshot(timeout=.1)

    def test_out_of_order_old_pair_cannot_replace_latest(self):
        timestamp = time.time_ns()
        feed(self.stream, timestamp)
        feed(self.stream, timestamp-1)
        self.assertEqual(self.stream.sequence, 1)


class RosCameraServiceTests(unittest.TestCase):
    def manager(self, temporary):
        with patch.object(RosCameraManager, '_subscribe'):
            return RosCameraManager(dict(cameras={'head': dict(config(), enabled=True)}), temporary)

    def test_record_saves_exact_pair_full_state_and_readback_interval(self):
        with tempfile.TemporaryDirectory() as temporary:
            manager = self.manager(temporary)
            feed(manager.streams['head'])
            def later():
                time.sleep(.03)
                feed(manager.streams['head'])
            thread = threading.Thread(target=later)
            thread.start()
            service = ControlService(copy.deepcopy(DEFAULT_CONFIG), MockRobotBackend(),
                                     MockChassisBackend(), False, camera_backend=manager)
            try:
                result = service.record_camera_snapshot('head')
                target = Path(result['directory'])
                metadata = json.loads((target/'head_camera_metadata.json').read_text())
                state = json.loads((target/'robot_state.json').read_text())
                self.assertEqual(metadata['camera_sequence'], 2)
                self.assertEqual(state['capture_synchronization']['camera_sequence'], 2)
                self.assertIn('arm_elbow_deg', state)
                self.assertEqual(len(state['joints_deg']['head']), 2)
                self.assertEqual(len(state['joints_deg']['trunk']), 4)
                self.assertEqual(len(state['joints_deg']['left_arm']), 7)
                self.assertEqual(len(state['joints_deg']['right_arm']), 7)
                np.testing.assert_array_equal(np.load(target/'head_depth_aligned.npy'), np.full((2, 3), 1234.))
            finally:
                thread.join()
                service.close()

    def test_hardware_enable_endpoint_no_longer_opens_camera(self):
        with tempfile.TemporaryDirectory() as temporary:
            manager = self.manager(temporary)
            try:
                with self.assertRaisesRegex(BackendError, '开机服务'):
                    manager.set_enabled('head', True)
                with self.assertRaisesRegex(BackendError, '未配置'):
                    manager.snapshot('left_wrist')
            finally:
                manager.close()

    def test_mock_cannot_restart_and_commands_cannot_be_supplied(self):
        service = ControlService(copy.deepcopy(DEFAULT_CONFIG), MockRobotBackend(), MockChassisBackend(), False)
        try:
            with self.assertRaises(BackendError):
                service.restart_camera({'camera': 'all'})
            with self.assertRaises(ValidationError):
                service.restart_camera({'camera': 'head', 'command': 'anything'})
        finally:
            service.close()

    def test_only_all_camera_restart_can_be_triggered(self):
        with tempfile.TemporaryDirectory() as temporary:
            manager = self.manager(temporary)
            try:
                with patch.object(manager.recovery, 'trigger', return_value={'accepted': True}) as trigger:
                    self.assertTrue(manager.restart('all')['accepted'])
                    trigger.assert_called_once_with()
                    with self.assertRaisesRegex(BackendError, '刷新网页'):
                        manager.restart('right_wrist')
            finally:
                manager.close()

    def test_service_all_scope_and_payload_validation(self):
        with tempfile.TemporaryDirectory() as temporary:
            manager=self.manager(temporary)
            service=ControlService(copy.deepcopy(DEFAULT_CONFIG),MockRobotBackend(),MockChassisBackend(),
                                   True,camera_backend=manager)
            try:
                with patch.object(manager,'restart',return_value={'accepted':True}) as restart:
                    self.assertTrue(service.restart_camera({'camera':'all'})['accepted'])
                    restart.assert_called_once_with('all')
                    for bad in ({'camera':'head'},{'camera':'unknown'},{'camera':'all','command':'anything'},None):
                        with self.assertRaises(ValidationError):service.restart_camera(bad)
                    restart.assert_called_once()
            finally:service.close()

    def test_pose_estimation_uses_ros_pair_without_starting_camera(self):
        class FakeRosCamera:
            externally_managed = True
            def status(self):
                return dict(head=dict(available=True, enabled=False))
            def fresh_snapshot(self, camera_id, timeout):
                return NS(sequence=2)
            def set_enabled(self, *args):
                raise AssertionError('must never start a ROS camera')
            def close(self):
                pass
        estimator = NS(fresh_frame_timeout=.1, estimate=lambda *args: dict(
            status='success', usable=True, result_id='test', relative_directory='data/test', elapsed_seconds=.1))
        service = ControlService(copy.deepcopy(DEFAULT_CONFIG), MockRobotBackend(), MockChassisBackend(), False,
                                 camera_backend=FakeRosCamera(), pose_estimator=estimator)
        try:
            self.assertTrue(service.estimate_grasp_object_pose()['usable'])
        finally:
            service.close()


if __name__ == '__main__':
    unittest.main()
