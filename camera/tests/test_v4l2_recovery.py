import threading
import time
import unittest
from unittest.mock import Mock, patch

import numpy as np

from vision.rokae_runtime.capture import V4L2CaptureWorker
from vision.rokae_runtime.owner import RokaeCameraOwner
from vision.rokae_runtime import devices
from test_rokae_resilience import configuration


def wait_for(predicate, timeout=2):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if predicate():
            return True
        time.sleep(0.01)
    return False


def capture(read):
    cap = Mock()
    cap.isOpened.return_value = True
    cap.read.side_effect = read
    return cap


class TestV4L2Recovery(unittest.TestCase):
    def test_initial_open_failure_retries_original_binding(self):
        failed = Mock()
        failed.isOpened.return_value = False
        healthy = capture(lambda: (True, np.ones((4, 4, 3), dtype=np.uint8)))
        resolver = Mock(return_value='/dev/video18')
        worker = V4L2CaptureWorker(camera_id='hand_right', device='/dev/video6',
                                  warmup_frames=1, device_resolver=resolver)
        worker.reconnect_delay_sec = .02
        with patch('cv2.VideoCapture', side_effect=[failed, healthy]) as opened:
            try:
                self.assertTrue(worker.start())
                self.assertTrue(wait_for(worker.ready))
                self.assertEqual([c.args[0] for c in opened.call_args_list], ['/dev/video6', '/dev/video18'])
                failed.release.assert_called_once()
            finally:
                worker.stop()

    def test_owner_waits_for_missing_serial_then_recovers_without_wrong_node(self):
        present = threading.Event()
        cfg = configuration(hand_left={'enabled':False}, hand_right={
            'backend':'v4l2', 'match':{'type':'usb_serial','value':'right-sn'}, 'warmup_frames':1})
        picture = np.ones((4, 4, 3), dtype=np.uint8)
        cap = capture(lambda: (True, picture))
        def resolve(match, **kwargs):
            self.assertEqual(match, {'type':'usb_serial','value':'right-sn'})
            return '/dev/video18' if present.is_set() else None
        with patch('vision.rokae_runtime.owner.resolve_device', side_effect=resolve), patch('cv2.VideoCapture', return_value=cap) as opened:
            owner = RokaeCameraOwner(cfg)
            try:
                owner.start()
                owner.start()
                worker = owner._workers['hand_right']['worker']
                worker.reconnect_delay_sec = .02
                self.assertFalse(owner.camera_ready('hand_right'))
                self.assertIsNone(owner.get_jpeg('hand_right'))
                self.assertTrue(owner.camera_ready('head'))
                opened.assert_not_called()
                present.set()
                self.assertTrue(wait_for(lambda: owner.camera_ready('hand_right'), timeout=3))
                self.assertEqual(owner._meta['hand_right']['device'], '/dev/video18')
                self.assertNotIn('error', owner._meta['hand_right'])
                opened.assert_called_once()
                self.assertEqual(opened.call_args.args[0], '/dev/video18')
            finally:
                owner.stop()

    def test_missing_duplicate_binding_is_reserved(self):
        missing = {'backend':'v4l2','match':{'type':'usb_serial','value':'same'}}
        with patch('vision.rokae_runtime.owner.resolve_device', return_value=None), patch('cv2.VideoCapture') as opened:
            owner = RokaeCameraOwner(configuration(head=missing, hand_left=missing, hand_right={'enabled':False}))
            try:
                owner.start()
                self.assertIn('head', owner._workers)
                self.assertNotIn('hand_left', owner._workers)
                self.assertEqual(owner._meta['hand_left']['error'], 'DEVICE_ALREADY_ASSIGNED')
                opened.assert_not_called()
            finally:
                owner.stop()

    def test_start_does_not_wait_for_first_native_read(self):
        entered = threading.Event()
        release = threading.Event()
        started = threading.Event()
        def read():
            entered.set()
            release.wait(3)
            return False, None
        cap = capture(read)
        worker = V4L2CaptureWorker(camera_id='head', device='unused')
        with patch('cv2.VideoCapture', return_value=cap):
            starter = threading.Thread(target=lambda: (worker.start(), started.set()))
            starter.start()
            try:
                self.assertTrue(entered.wait(1))
                self.assertTrue(started.wait(0.2), 'warmup blocked start')
                self.assertFalse(worker.ready())
            finally:
                release.set()
                starter.join(6)
                worker.stop()

    def test_failed_reads_reopen_and_recover_frame(self):
        first = capture(lambda: (False, None))
        picture = np.ones((4, 4, 3), dtype=np.uint8)
        second = capture(lambda: (True, picture))
        worker = V4L2CaptureWorker(camera_id='head', device='original', warmup_frames=1)
        worker.reconnect_delay_sec = 0.02
        with patch('cv2.VideoCapture', side_effect=[first, second]) as opened:
            try:
                self.assertTrue(worker.start())
                self.assertTrue(wait_for(worker.ready))
                self.assertEqual(opened.call_count, 2)
                first.release.assert_called_once()
                np.testing.assert_array_equal(worker.get_frame(), picture)
            finally:
                worker.stop()
        second.release.assert_called_once()

    def test_rediscovery_does_not_reuse_old_node_when_binding_missing(self):
        first = capture(lambda: (False, None))
        picture = np.ones((4, 4, 3), dtype=np.uint8)
        second = capture(lambda: (True, picture))
        resolver = Mock(side_effect=[None, '/dev/video20'])
        worker = V4L2CaptureWorker(camera_id='head', device='/dev/video4', warmup_frames=1,
                                  device_resolver=resolver)
        worker.reconnect_delay_sec = 0.02
        with patch('cv2.VideoCapture', side_effect=[first, second]) as opened:
            try:
                worker.start()
                self.assertTrue(wait_for(worker.ready))
                self.assertEqual([call.args[0] for call in opened.call_args_list], ['/dev/video4', '/dev/video20'])
                self.assertEqual(resolver.call_count, 2)
            finally:
                worker.stop()

    def test_stop_interrupts_reconnect_backoff(self):
        first = capture(lambda: (False, None))
        worker = V4L2CaptureWorker(camera_id='head', device='unused')
        worker.reconnect_delay_sec = 30
        with patch('cv2.VideoCapture', return_value=first) as opened:
            worker.start()
            try:
                self.assertTrue(wait_for(lambda: first.release.called))
                begin = time.monotonic()
                worker.stop()
                self.assertLess(time.monotonic() - begin, 1)
                self.assertEqual(opened.call_count, 1)
            finally:
                worker.stop()

    def test_owner_rediscovery_excludes_other_camera_node(self):
        config = configuration(head={'backend':'v4l2', 'match':{'type':'usb_path','value':'head-usb'}},
                               hand_left={'enabled':False},
                               hand_right={'backend':'v4l2','match':{'type':'usb_path','value':'right-usb'}})
        with patch('vision.rokae_runtime.owner.resolve_device', side_effect=['/dev/video4','/dev/video2','/dev/video20']) as resolve, patch(
            'vision.rokae_runtime.owner.V4L2CaptureWorker'
        ) as factory:
            owner = RokaeCameraOwner(config)
            try:
                owner.start()
                callback = factory.call_args_list[0].kwargs['device_resolver']
                self.assertEqual(callback(), '/dev/video20')
                self.assertEqual(resolve.call_args.kwargs['exclude'], {'/dev/video2'})
                self.assertEqual(owner._meta['head']['device'], '/dev/video20')
            finally:
                owner.stop()

    def test_probe_sets_requested_fourcc_before_read(self):
        import cv2
        cap = capture(lambda: (True, np.ones((2, 2, 3), dtype=np.uint8)))
        cap.get.return_value = int.from_bytes(b'MJPG', 'little')
        with patch('cv2.VideoCapture', return_value=cap):
            self.assertTrue(devices._probe_color_node('unused', 640, 480, fourcc='MJPG'))
        self.assertEqual(cap.set.call_args_list[0].args,
                         (cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG')))
        cap.release.assert_called_once()

    def test_probe_error_releases_handle(self):
        cap = capture(lambda: (_ for _ in ()).throw(RuntimeError('read failed')))
        cap.get.return_value = int.from_bytes(b'YUYV', 'little')
        with patch('cv2.VideoCapture', return_value=cap):
            self.assertFalse(devices._probe_color_node('unused',640,480))
        cap.release.assert_called_once()

    def test_preview_uses_local_owner_urls(self):
        from vision.rokae_runtime.preview import PREVIEW_HTML
        self.assertIn("fetch('/camera/list'", PREVIEW_HTML)
        self.assertIn("'/camera/stream?camera='", PREVIEW_HTML)
        self.assertNotIn('127.0.0.1', PREVIEW_HTML)
