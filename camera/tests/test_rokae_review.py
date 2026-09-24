"""Regressions discovered by reviewing the real SDK and lifecycle contracts."""
import threading
import unittest
from unittest.mock import Mock, patch

from vision.push import PushRuntime
from vision.rokae_runtime.capture import OrbbecSdkCaptureWorker, V4L2CaptureWorker
from vision.rokae_runtime.owner import RokaeCameraOwner
from test_rokae_resilience import configuration
from test_vision import FakeProcess


class TestReviewRegressions(unittest.TestCase):
    def test_unknown_id_cannot_start_or_stop_all_streams(self):
        runtime = PushRuntime({"media": {"push": {"streams": [
            {"camera_id": "head", "enabled": True},
            {"camera_id": "hand_left", "enabled": True},
        ]}}}, source_uri=lambda cid: "unused")
        for worker in runtime._workers.values():
            worker.start = Mock()
            worker.stop = Mock()
        self.assertEqual(runtime.start(camera_id="typo").get("error_code"), "CAMERA_NOT_FOUND")
        self.assertEqual(runtime.stop(camera_id="typo").get("error_code"), "CAMERA_NOT_FOUND")
        self.assertEqual(runtime.status(camera_id="typo")["streams"], [])
        for worker in runtime._workers.values():
            worker.start.assert_not_called()
            worker.stop.assert_not_called()

    def test_explicit_disabled_stream_cannot_start(self):
        runtime = PushRuntime({"media": {"push": {"streams": [
            {"camera_id": "head", "enabled": False},
        ]}}}, source_uri=lambda cid: "unused")
        worker = runtime._workers["head"]
        worker.start = Mock()
        self.assertFalse(runtime.start(camera_id="head")["ok"])
        worker.start.assert_not_called()

    def test_ephemeral_owner_port_is_preserved(self):
        self.assertEqual(RokaeCameraOwner(configuration()).port, 0)

    def test_zero_capture_dimensions_are_not_silently_defaulted(self):
        owner = RokaeCameraOwner(configuration(head={"backend": "fake", "width": 0}))
        try:
            owner.start()
            self.assertFalse(owner.camera_ready("head"))
            self.assertTrue(owner.camera_ready("hand_left"))
        finally:
            owner.stop()

    def test_blocked_worker_keeps_handle_and_prevents_reopen(self):
        for worker in (V4L2CaptureWorker(camera_id="head", device="unused"),
                       OrbbecSdkCaptureWorker(camera_id="head")):
            with self.subTest(worker=type(worker).__name__):
                thread = Mock()
                thread.is_alive.return_value = True
                worker._thread = thread
                resource = Mock()
                if isinstance(worker, V4L2CaptureWorker):
                    worker._cap = resource
                else:
                    worker._pipeline = resource
                    worker._needs_stop = True
                with self.assertRaises(RuntimeError):
                    worker.stop()
                self.assertIs(worker._thread, thread)
                resource.release.assert_not_called()
                resource.stop.assert_not_called()
                self.assertFalse(worker.start())
                thread.is_alive.return_value = False
                worker.stop()

    def test_owner_retains_worker_if_stop_fails(self):
        owner = RokaeCameraOwner(configuration())
        worker = Mock()
        worker.stop.side_effect = RuntimeError("capture read is still blocked")
        owner._workers["head"] = {"worker": worker, "stale": 2.0}
        owner.stop()
        self.assertIn("head", owner._workers)
        worker.stop.side_effect = None
        owner.stop()
        self.assertEqual(owner._workers, {})

    def test_owner_releases_resources_when_http_bind_fails(self):
        from vision.rokae_runtime import __main__ as entry
        owner = Mock()
        with patch.object(entry, "load_config", return_value={}), patch.object(
            entry, "RokaeCameraOwner", return_value=owner
        ), patch.object(entry, "serve_owner", side_effect=OSError("address in use")):
            with self.assertRaises(OSError):
                entry.main([])
            owner.stop.assert_called_once()
            owner.start.assert_not_called()


class TestV4L2Review(unittest.TestCase):
    def test_configuration_failure_releases_open_device(self):
        worker = V4L2CaptureWorker(camera_id="head", device="unused")
        cap = Mock()
        cap.isOpened.return_value = True
        cap.set.side_effect = RuntimeError("driver configuration failed")
        with patch("cv2.VideoCapture", return_value=cap):
            with self.assertRaises(RuntimeError):
                worker.start()
        cap.release.assert_called_once()
        self.assertIsNone(worker._cap)

    def test_transient_read_exception_recovers_without_second_reader(self):
        import time
        import numpy as np

        worker = V4L2CaptureWorker(camera_id="head", device="unused", warmup_frames=1, fps=100)
        cap = Mock()
        cap.isOpened.return_value = True
        image = np.full((2, 2, 3), 127, dtype=np.uint8)
        calls = []
        def read():
            calls.append(threading.get_ident())
            if len(calls) == 2:
                raise RuntimeError("temporary read error")
            return True, image
        cap.read.side_effect = read
        with patch("cv2.VideoCapture", return_value=cap) as opened:
            try:
                self.assertTrue(worker.start())
                self.assertTrue(worker.start())
                deadline = time.monotonic() + 2
                while (len(calls) < 3 or not worker.ready()) and time.monotonic() < deadline:
                    time.sleep(0.01)
                self.assertTrue(worker.ready())
                np.testing.assert_array_equal(worker.get_frame(), image)
                self.assertEqual(len(set(calls[1:])), 1)
                opened.assert_called_once()
            finally:
                worker.stop()
        cap.release.assert_called_once()
        self.assertFalse(worker.ready())
