import concurrent.futures
from datetime import datetime
import http.client
import json
from pathlib import Path
import tempfile
import threading
import unittest
from unittest.mock import Mock, patch

from test_ros_camera import image, config
from rokae_web.backends import BackendError
from rokae_web.rgb_recording import RgbRecordingStore
from rokae_web.ros_camera import RosCameraManager, RosColorStream
from rokae_web.service import ControlService
from rokae_web.web import make_server


class LeftWristRGBTests(unittest.TestCase):
    def test_rgb_ready_without_depth_and_stale_rejected(self):
        stream = RosColorStream('left_wrist', dict(config(), color_only=True))
        stream.receive('color', image())
        status = stream.status()
        self.assertTrue(status['enabled'])
        self.assertTrue(status['color_only'])
        self.assertEqual(status['alignment'], 'none')
        with self.assertRaises(BackendError):
            stream.snapshot(timeout=.01)
        stream.received_monotonic -= 3
        self.assertFalse(stream.status()['enabled'])
        stream.close()

    def test_millisecond_names_never_overwrite_even_concurrently(self):
        with tempfile.TemporaryDirectory() as root:
            store = RgbRecordingStore(root)
            with patch('rokae_web.rgb_recording.datetime') as clock:
                clock.now.return_value = datetime(2026, 9, 19, 23, 59, 59, 999000)
                with concurrent.futures.ThreadPoolExecutor(max_workers=4) as executor:
                    rows = list(executor.map(store.save, [b'jpeg-one', b'jpeg-two', b'jpeg-three']))
            paths = [Path(row['path']) for row in rows]
            self.assertEqual(len(set(paths)), 3)
            self.assertEqual({p.read_bytes() for p in paths}, {b'jpeg-one', b'jpeg-two', b'jpeg-three'})
            self.assertTrue((Path(root)/'2026-09-19/235959999.jpg').is_file())
            self.assertTrue((Path(root)/'2026-09-20/000000000.jpg').is_file())
            self.assertEqual(len(list(Path(root).rglob('*.jpg'))), 3)
            self.assertEqual(len([p for p in Path(root).rglob('*') if p.is_file()]), 3)

    def test_http_rgb_record_saves_only_jpeg_without_robot_reads(self):
        with tempfile.TemporaryDirectory() as root, patch.object(RosCameraManager, '_subscribe'):
            manager = RosCameraManager(dict(cameras={'left_wrist': dict(config(), enabled=True, color_only=True)},
                                            left_wrist_rgb_directory=root), Path(root)/'head_data')
            service = ControlService.__new__(ControlService)
            service.camera = manager
            service.robot = Mock()
            service.robot.read_state.side_effect = AssertionError('RGB save must not read robot')
            service.audit_event = Mock()
            server = make_server('127.0.0.1', 0, service, Path(root))
            thread = threading.Thread(target=server.serve_forever, daemon=True)
            thread.start()
            try:
                with patch('rokae_web.ros_camera.capture_rgb', return_value=b'jpeg') as capture:
                    connection = http.client.HTTPConnection('127.0.0.1', server.server_port)
                    connection.request('POST','/api/cameras/left_wrist/record-rgb',body='{}',
                                       headers={'Content-Type':'application/json'})
                    response = connection.getresponse()
                    result = json.loads(response.read())
                    self.assertEqual(response.status, 200, result)
                    connection.close()
                    capture.assert_called_once_with('left_wrist', 3.0, url='http://127.0.0.1:8085/camera/capture')
                    path = Path(result['data']['path'])
                    self.assertRegex(path.name, r'^\d{9}\.jpg$')
                    self.assertEqual(path.read_bytes(), b'jpeg')
                    service.robot.read_state.assert_not_called()
                    self.assertEqual([p for p in Path(root).rglob('*') if p.is_file()], [path])
            finally:
                server.shutdown()
                server.server_close()
                thread.join(timeout=2)
                manager.close()


if __name__ == '__main__':
    unittest.main()
