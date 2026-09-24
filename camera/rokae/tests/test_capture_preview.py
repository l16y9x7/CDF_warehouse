"""Capture API contract checks, with no camera or controller connections."""
import copy
import io
import json
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch
from urllib.error import HTTPError
from urllib.parse import parse_qs, urlparse

import cv2
import numpy as np

from rokae_web.backends import BackendError
from rokae_web.capture_preview import CAPTURE_URL, capture_rgb
from rokae_web.ros_camera import RosCameraManager


class CapturePreviewTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.root = Path(self.temp.name)
        self.capture_dir = self.root / 'capture-002'
        self.capture_dir.mkdir()
        self.path = self.capture_dir / 'rgb.jpg'
        ok, encoded = cv2.imencode('.jpg', np.zeros((2, 3, 3), dtype=np.uint8))
        self.assertTrue(ok)
        self.jpeg = encoded.tobytes()
        self.path.write_bytes(self.jpeg)
        self.payload = dict(ok=True, capture_id='capture-002', camera='head',
                            same_shot=True, depth=None,
                            color=dict(path=str(self.path), format='jpeg', width=3, height=2))

    def capture(self, payload=None, camera='head'):
        with patch('rokae_web.capture_preview.urlopen',
                   return_value=io.BytesIO(json.dumps(payload or self.payload).encode())) as request:
            frame = capture_rgb(camera, 3, frames_root=self.root)
        return frame, request

    def test_all_camera_ids_request_only_color_and_read_returned_file(self):
        for camera in ('head', 'left_wrist', 'right_wrist'):
            with self.subTest(camera=camera):
                payload = dict(self.payload, camera=camera)
                frame, request = self.capture(payload, camera)
                self.assertEqual(frame, self.jpeg)
                target = urlparse(request.call_args.args[0])
                self.assertEqual(target.path, '/camera/capture')
                self.assertEqual(target.netloc, '127.0.0.1:8085')
                self.assertEqual(parse_qs(target.query), {'camera': [camera], 'streams': ['color']})
                request.assert_called_once()

    def test_json_and_http_failures_preserve_camera_error(self):
        for code in ('CAMERA_NOT_FOUND', 'INVALID_STREAMS', 'INVALID_FORMAT',
                     'CAMERA_NOT_READY', 'DEPTH_NOT_ALIGNED', 'CAPTURE_FAILED'):
            payload = dict(ok=False, error_code=code, message='camera not ready', camera='head')
            with self.subTest(code=code), self.assertRaisesRegex(BackendError, code):
                self.capture(payload)
        error = HTTPError(CAPTURE_URL, 404, 'Not Found', {},
                          io.BytesIO(json.dumps(payload).encode()))
        with patch('rokae_web.capture_preview.urlopen', side_effect=error), \
                self.assertRaisesRegex(BackendError, 'CAPTURE_FAILED: camera not ready'):
            capture_rgb('head', 3, frames_root=self.root)

    def test_rejects_wrong_camera_session_and_unrequested_depth(self):
        for changes in ({'camera': 'right_wrist'}, {'capture_id': '../escape'},
                        {'capture_id': 'different-capture'}, {'depth': {}}, {'color': None}):
            with self.subTest(changes=changes), self.assertRaises(BackendError):
                self.capture(dict(self.payload, **changes))

    def test_rejects_outside_paths_and_missing_files(self):
        for path in (self.root / 'rgb.jpg', self.capture_dir / 'missing.jpg', Path('rgb.jpg')):
            payload = copy.deepcopy(self.payload)
            payload['color']['path'] = str(path)
            with self.subTest(path=path), self.assertRaises(BackendError):
                self.capture(payload)

    def test_rejects_malformed_json_and_invalid_jpeg_or_dimensions(self):
        with patch('rokae_web.capture_preview.urlopen', return_value=io.BytesIO(b'not-json')), \
                self.assertRaises(BackendError):
            capture_rgb('head', 3, frames_root=self.root)
        self.path.write_bytes(b'\xff\xd8bad\xff\xd9')
        with self.assertRaises(BackendError):
            self.capture()
        self.path.write_bytes(self.jpeg)
        payload = copy.deepcopy(self.payload)
        payload['color']['width'] = 4
        with self.assertRaises(BackendError):
            self.capture(payload)

    def test_timeout_is_reported_without_fallback(self):
        with patch('rokae_web.capture_preview.urlopen', side_effect=TimeoutError('timed out')) as request, \
                self.assertRaisesRegex(BackendError, 'timed out'):
            capture_rgb('head', 3, frames_root=self.root)
        request.assert_called_once()

    def test_preview_uses_capture_even_when_ros_is_unavailable(self):
        camera_config = dict(enabled=True, width=3, height=2, topics={})
        config = dict(cameras={name: camera_config for name in ('head', 'left_wrist', 'right_wrist')})
        with patch.object(RosCameraManager, '_subscribe'):
            manager = RosCameraManager(config, self.root)
        self.addCleanup(manager.close)
        manager.init_error = 'ROS unavailable'
        with patch('rokae_web.ros_camera.capture_rgb', return_value=self.jpeg) as capture:
            for camera in config['cameras']:
                self.assertTrue(manager.status()[camera]['frame_available'])
                frame, sequence, enabled = manager.frame_jpeg(camera, 'rgb', 0, 3)
                self.assertEqual(frame, self.jpeg)
                self.assertTrue(enabled)
                self.assertGreater(sequence, 0)
                capture.assert_called_with(camera, 3, url=CAPTURE_URL)


if __name__ == '__main__':
    unittest.main()
