import time
import unittest
from types import SimpleNamespace
from unittest.mock import patch

import numpy as np

from vision.rokae_runtime.ros_bridge import RosImageWorker, image_to_bgr, RosCameraOwner, RosOwnerLiveness
from vision.rokae_runtime.driver_supervisor import (
    RgbdLiveness,
    validate_realsense_color_profile,
    validate_unique_realsense_owner,
)
from vision.rokae_runtime.ros_contract import ros_camera_id


def message(data=b'\x01\x02\x03\x04\x05\x06\xff\xff', stamp=None, encoding='rgb8'):
    stamp = time.time() if stamp is None else stamp
    return SimpleNamespace(width=2, height=1, step=8, data=data, encoding=encoding,
                           header=SimpleNamespace(stamp=SimpleNamespace(
                               sec=int(stamp), nanosec=int((stamp % 1) * 1e9))))


class TestRosOwner(unittest.TestCase):
    def test_realsense_profile_validation_matches_serial_and_exact_profile(self):
        class Profile:
            def __init__(self, width, height, fps, stream, pixel_format='rgb8'):
                self._width, self._height, self._fps = width, height, fps
                self._stream, self._format = stream, pixel_format
            def as_video_stream_profile(self):
                return self
            def width(self): return self._width
            def height(self): return self._height
            def fps(self): return self._fps
            def stream_type(self): return self._stream
            def format(self): return self._format
        class Device:
            def get_info(self, _key): return 'test-serial'
            def query_sensors(self):
                return [SimpleNamespace(get_stream_profiles=lambda: [
                    Profile(1280, 720, 15, 'color'), Profile(640, 480, 30, 'depth'),
                ])]
        rs = SimpleNamespace(
            context=lambda: SimpleNamespace(query_devices=lambda: [Device()]),
            camera_info=SimpleNamespace(serial_number='serial'),
            stream=SimpleNamespace(color='color'),
            format=SimpleNamespace(rgb8='rgb8'),
        )
        cfg = {
            'match': {'type': 'realsense_serial', 'value': 'test-serial'},
            'width': 1280, 'height': 720, 'fps': 15,
        }
        self.assertEqual(validate_realsense_color_profile(cfg, rs), (1280, 720, 15))
        yuyv_device = Device()
        yuyv_device.query_sensors = lambda: [SimpleNamespace(
            get_stream_profiles=lambda: [Profile(
                1280, 720, 15, 'color', 'yuyv'
            )]
        )]
        yuyv_rs = SimpleNamespace(
            context=lambda: SimpleNamespace(query_devices=lambda: [yuyv_device]),
            camera_info=rs.camera_info,
            stream=rs.stream,
            format=rs.format,
        )
        with self.assertRaisesRegex(RuntimeError, 'Unsupported.*1280x720@15'):
            validate_realsense_color_profile(cfg, yuyv_rs)
        cfg['fps'] = 30
        with self.assertRaisesRegex(RuntimeError, 'Unsupported.*1280x720@30'):
            validate_realsense_color_profile(cfg, rs)
        cfg['match']['value'] = 'other'
        with self.assertRaisesRegex(RuntimeError, 'serial not found'):
            validate_realsense_color_profile(cfg, rs)
        cfg['match'] = {'type': 'usb_path', 'value': 'test-serial'}
        with self.assertRaisesRegex(ValueError, 'realsense_serial'):
            validate_realsense_color_profile(cfg, rs)
        cfg['match'] = {'type': 'realsense_serial', 'value': 'test-serial'}
        cfg['backend'] = 'v4l2'
        with self.assertRaisesRegex(ValueError, 'backend=realsense'):
            validate_realsense_color_profile(cfg, rs)

    def test_realsense_profile_prefers_canonical_fps_over_legacy_ros_fps(self):
        class Profile:
            def as_video_stream_profile(self): return self
            def width(self): return 1280
            def height(self): return 720
            def fps(self): return 15
            def stream_type(self): return 'color'
            def format(self): return 'rgb8'
        device = SimpleNamespace(
            get_info=lambda _key: 'test-serial',
            query_sensors=lambda: [SimpleNamespace(get_stream_profiles=lambda: [Profile()])],
        )
        rs = SimpleNamespace(
            context=lambda: SimpleNamespace(query_devices=lambda: [device]),
            camera_info=SimpleNamespace(serial_number='serial'),
            stream=SimpleNamespace(color='color'),
            format=SimpleNamespace(rgb8='rgb8'),
        )
        cfg = {
            'backend': 'realsense',
            'match': {'type': 'realsense_serial', 'value': 'test-serial'},
            'width': 1280, 'height': 720, 'fps': 15, 'ros_fps': 30,
        }
        self.assertEqual(validate_realsense_color_profile(cfg, rs), (1280, 720, 15))

    def test_duplicate_enabled_realsense_serial_is_rejected(self):
        camera = {
            'enabled': True, 'backend': 'realsense',
            'match': {'type': 'realsense_serial', 'value': 'same-serial'},
        }
        data = {'rokae': {'owner': {'cameras': {
            'hand_left': dict(camera), 'hand_right': dict(camera),
        }}}}
        # Legacy wrist configurations implied the RealSense backend.
        data['rokae']['owner']['cameras']['hand_right'].pop('backend')
        with self.assertRaisesRegex(ValueError, 'multiple enabled roles'):
            validate_unique_realsense_owner(data, 'hand_left')
        data['rokae']['owner']['cameras']['hand_right']['enabled'] = False
        validate_unique_realsense_owner(data, 'hand_left')

        data['rokae']['owner']['cameras']['head'] = dict(camera)
        with self.assertRaisesRegex(ValueError, 'multiple enabled roles'):
            validate_unique_realsense_owner(data, 'hand_left')

    def test_owner_liveness_startup_stall_and_recovery(self):
        live = RosOwnerLiveness(0, startup_timeout=60, stale_timeout=15)
        self.assertFalse(live.expired(60, False))
        self.assertTrue(live.expired(61, False))
        self.assertFalse(live.expired(62, True))
        self.assertFalse(live.expired(77, False))
        self.assertTrue(live.expired(78, False))
        self.assertFalse(live.expired(79, True))
        self.assertFalse(live.expired(94, False))

    def test_role_mapping_preserves_http_roles_with_tianji_ros_names(self):
        self.assertEqual([ros_camera_id(role) for role in ('head', 'hand_left', 'hand_right')],
                         ['head', 'left_wrist', 'right_wrist'])
        with self.assertRaises(KeyError):
            ros_camera_id('unknown')

    def test_clock_regression_recovers_only_after_previous_stream_stales(self):
        worker = RosImageWorker(max_age=2.0)
        with patch('vision.rokae_runtime.ros_bridge.time.time', return_value=1000), \
             patch('vision.rokae_runtime.ros_bridge.time.monotonic', return_value=100):
            worker.receive(message(stamp=1000))
            first = worker.get_frame()
        # The wall clock moved backwards; a fresh previous stream still rejects
        # out-of-order frames rather than replacing its accepted frame.
        with patch('vision.rokae_runtime.ros_bridge.time.time', return_value=999), \
             patch('vision.rokae_runtime.ros_bridge.time.monotonic', return_value=101):
            worker.receive(message(data=bytes(8), stamp=999))
            np.testing.assert_array_equal(worker._frame, first)
        # Once stale, accept a new wall-clock-fresh stream even with lower stamps.
        with patch('vision.rokae_runtime.ros_bridge.time.time', return_value=999), \
             patch('vision.rokae_runtime.ros_bridge.time.monotonic', return_value=103):
            worker.receive(message(data=bytes(8), stamp=999))
            self.assertTrue(worker.ready())
            self.assertFalse(worker.get_frame().any())
        # Device stamps far behind wall clock are remapped to arrival time, so a
        # still-publishing Orbbec stream is accepted instead of staying stale.
        with patch('vision.rokae_runtime.ros_bridge.time.time', return_value=1005), \
             patch('vision.rokae_runtime.ros_bridge.time.monotonic', return_value=110):
            worker.receive(message(stamp=999))
            self.assertTrue(worker.ready())

    def test_supervisor_requires_full_rgbd_startup_but_survives_depth_only_stall(self):
        live = RgbdLiveness(0, startup_timeout=60, stale_timeout=15)
        live.record('color', 50)
        self.assertEqual(live.failure(59), '')
        self.assertTrue(live.was_ready)
        # Depth/info never arrive: color-only ready still survives.
        live.record('color', 80)
        self.assertEqual(live.failure(80), '')
        live.record('color', 100)
        self.assertEqual(live.failure(116), 'RGB-D stream stalled')

    def test_padded_rgb_is_bgr_without_padding(self):
        image = image_to_bgr(message())
        self.assertEqual(image.tolist(), [[[3, 2, 1], [6, 5, 4]]])
        self.assertEqual(image_to_bgr(message(encoding='bgr8')).tolist(), [[[1, 2, 3], [4, 5, 6]]])

    def test_invalid_payload_and_depth_rejected(self):
        for msg in [message(data=b'bad'), message(encoding='16UC1')]:
            with self.assertRaises(ValueError):
                image_to_bgr(msg)
        msg = message()
        msg.step = 5
        with self.assertRaises(ValueError):
            image_to_bgr(msg)

    def test_freshness_errors_recovery_and_stop(self):
        worker = RosImageWorker(max_age=1.0)
        self.assertFalse(worker.ready())
        worker.receive(message())
        self.assertTrue(worker.ready())
        # Skewed device stamps are remapped to wall clock so USB Orbbec skew
        # cannot permanently starve Owner while frames keep arriving.
        worker.receive(message(stamp=time.time() - 5))
        self.assertTrue(worker.ready())
        worker.receive(message(stamp=time.time() + 5))
        self.assertTrue(worker.ready())
        worker.receive(message())
        self.assertTrue(worker.ready())
        with patch('vision.rokae_runtime.ros_bridge.time.monotonic', return_value=time.monotonic() + 5):
            self.assertFalse(worker.ready())
        worker.stop()
        worker.receive(message())
        self.assertFalse(worker.ready())

    def test_duplicate_stamp_does_not_replace_source(self):
        worker = RosImageWorker()
        msg = message()
        worker.receive(msg)
        first = worker.get_frame()
        msg.data = bytes(8)
        worker.receive(msg)
        np.testing.assert_array_equal(worker.get_frame(), first)

    def test_http_owner_uses_rgb_only_and_preserves_disabled_role(self):
        import sys
        from unittest.mock import MagicMock

        owner = RosCameraOwner({'rokae': {'owner': {'cameras': {'hand_left': {'enabled': False}}}}})
        worker = RosImageWorker()
        worker.receive(message())
        owner._workers['head'] = {'worker': worker, 'stale': 2.0}
        owner._meta = {'head': {'enabled': True, 'backend': 'ros'}, 'hand_left': {'enabled': False}}
        mock_cv2 = MagicMock()
        mock_cv2.imencode.return_value = (True, np.array([0xff, 0xd8], dtype=np.uint8))
        mock_cv2.IMWRITE_JPEG_QUALITY = 1
        with patch.dict(sys.modules, {'cv2': mock_cv2}):
            self.assertTrue(owner.get_jpeg('head').startswith(b'\xff\xd8'))
        self.assertIsNone(owner.get_jpeg('hand_left'))
        self.assertFalse(next(c for c in owner.listing()['cameras'] if c['id'] == 'left_wrist')['enabled'])
        owner.stop()
        self.assertIsNone(owner.get_jpeg('head'))


if __name__ == '__main__':
    unittest.main()
