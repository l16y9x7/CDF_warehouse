import threading
import time
import unittest
from unittest.mock import Mock, patch

from vision.adapters.rokae import RokaeCameraAdapter
from vision.service import CameraService, MediaService


class TestRokaeSourceWait(unittest.TestCase):
    def test_missing_reason_reaches_camera_api_without_changing_error_contract(self):
        from fastapi.testclient import TestClient
        from vision.http_app import create_camera_app

        cfg = {"adapter": "rokae"}
        rows = [{"id": "head", "enabled": True, "ready": True},
                {"id": "hand_right", "enabled": True, "ready": False, "error": "DEVICE_NOT_FOUND"},
                {"id": "hand_left", "enabled": False, "ready": False}]
        with patch('vision.adapters.rokae._http_json', return_value=(200, {"cameras": rows})):
            client = TestClient(create_camera_app(CameraService(cfg)))
            state = client.get('/state').json()
            by_id = {r['camera_id']: r for r in state['cameras']}
            self.assertEqual(by_id['hand_right']['error'], 'DEVICE_NOT_FOUND')
            self.assertEqual(by_id['hand_left']['error'], 'CAMERA_DISABLED')
            result = client.get('/frame/hand_right')
            self.assertEqual(result.status_code, 404)
            self.assertEqual(result.json()['error_code'], 'CAMERA_NOT_READY')
            self.assertEqual(result.json()['reason'], 'DEVICE_NOT_FOUND')
            self.assertEqual(client.get('/frame/head').status_code, 200)

    def test_missing_right_waits_then_autostarts_without_accumulating_network_retries(self):
        cfg = {"adapter": "rokae", "media": {"push": {
            "device_sn": "TEST", "stream_server_url": "rtmp://localhost/live",
            "streams": [{"camera_id": "hand_right", "stream_slot": 3,
                         "restart_initial_sec": .1}]}}}
        present = threading.Event()
        checks = []
        def listing():
            checks.append(time.monotonic())
            return {"ok": True, "cameras": [
                {"camera_id": "head", "enabled": True, "ready": True},
                {"camera_id": "hand_right", "enabled": True, "ready": present.is_set()}]}
        service = MediaService(CameraService(cfg), cfg)
        process = Mock()
        process.poll.return_value = None
        process.stderr = None
        worker = service.push._workers['hand_right']
        worker._popen_factory = Mock(return_value=process)
        with patch.object(service.camera.adapter, 'listing', side_effect=listing):
            try:
                service.start_push(camera_id='hand_right')
                deadline = time.monotonic() + 3
                while len(checks) < 3 and time.monotonic() < deadline:
                    time.sleep(.02)
                self.assertGreaterEqual(len(checks), 3)
                worker._popen_factory.assert_not_called()
                self.assertEqual(worker._restart_attempt, 0)
                self.assertFalse(worker.snapshot()['online'])
                present.set()
                deadline = time.monotonic() + 2
                while not worker.snapshot()['online'] and time.monotonic() < deadline:
                    time.sleep(.02)
                self.assertTrue(worker.snapshot()['online'])
                worker._popen_factory.assert_called_once()
                cmd = worker._popen_factory.call_args.args[0]
                self.assertIn('camera=hand_wrist', cmd[cmd.index('-i') + 1])
                self.assertNotIn('/dev/video', ' '.join(cmd))
            finally:
                service.push.stop_all()
        self.assertFalse(worker._watchdog and worker._watchdog.is_alive())

    def test_owner_offline_has_explicit_reason(self):
        with patch('vision.adapters.rokae._http_json', return_value=(0, None)):
            rows = RokaeCameraAdapter({}).listing()['cameras']
            self.assertTrue(all(not r['ready'] and r['error'] == 'OWNER_UNAVAILABLE' for r in rows))
