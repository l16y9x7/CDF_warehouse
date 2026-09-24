import threading
import time
import unittest
from types import SimpleNamespace
from unittest.mock import Mock, patch

# Import native OpenCV before patch.dict snapshots sys.modules. Reloading it
# after that patch removes newly imported modules is unsupported by cv2.
import cv2
import numpy as np

from vision.rokae_runtime.capture import OrbbecSdkCaptureWorker
from vision.rokae_runtime.owner import RokaeCameraOwner
from test_rokae_resilience import configuration, eventually, sdk_fixture


def color_frames(value=90):
    color = Mock()
    color.get_width.return_value = 2
    color.get_height.return_value = 2
    color.get_format.return_value = 'BGR'
    color.get_data.return_value = np.full(12, value, dtype=np.uint8)
    return SimpleNamespace(get_color_frame=lambda: color)


class TestOrbbecRecovery(unittest.TestCase):
    def test_failed_pipeline_start_is_released_before_retry(self):
        sdk, first = sdk_fixture()
        _, second = sdk_fixture()
        events = []
        first.start.side_effect = RuntimeError('temporarily busy')
        first.stop.side_effect = lambda: events.append('failed pipeline stopped')
        second.start.side_effect = lambda cfg: events.append('next pipeline started')
        second.wait_for_frames.return_value = color_frames()
        sdk.Pipeline.side_effect = [first, second]
        worker = OrbbecSdkCaptureWorker(camera_id='head', serial='A')
        worker.reconnect_delay_sec = .02
        with patch.dict('sys.modules', pyorbbecsdk=sdk):
            try:
                worker.start()
                self.assertTrue(eventually(worker.ready))
                self.assertEqual(events, ['failed pipeline stopped', 'next pipeline started'])
            finally:
                worker.stop()

    def test_boot_usb_failure_retries_and_becomes_ready(self):
        sdk, pipeline = sdk_fixture()
        context = sdk.Context.return_value
        sdk.Context.side_effect = [RuntimeError('USB not ready'), context]
        pipeline.wait_for_frames.return_value = color_frames()
        worker = OrbbecSdkCaptureWorker(camera_id='head', serial='A')
        worker.reconnect_delay_sec = .02
        with patch.dict('sys.modules', pyorbbecsdk=sdk):
            try:
                self.assertTrue(worker.start())
                self.assertTrue(eventually(worker.ready))
                self.assertEqual(worker.last_error, '')
                self.assertEqual(worker.serial, 'A')
                pipeline.start.assert_called_once()
            finally:
                worker.stop()
        pipeline.stop.assert_called_once()

    def test_missing_device_arrival_updates_owner_without_manual_restart(self):
        sdk, pipeline = sdk_fixture()
        listing = sdk.Context.return_value.query_devices.return_value
        present = threading.Event()
        listing.get_count.side_effect = lambda: int(present.is_set())
        pipeline.wait_for_frames.return_value = color_frames()
        owner = RokaeCameraOwner(configuration(head={'backend':'orbbec', 'serial':'A'}))
        with patch.dict('sys.modules', pyorbbecsdk=sdk):
            try:
                owner.start()
                worker = owner._workers['head']['worker']
                worker.reconnect_delay_sec = .02
                self.assertTrue(eventually(lambda: worker.last_error == 'DEVICE_NOT_FOUND'))
                self.assertFalse(owner.camera_ready('head'))
                self.assertEqual(owner.listing()['cameras'][0]['error'], 'DEVICE_NOT_FOUND')
                sdk.Pipeline.assert_not_called()
                present.set()
                self.assertTrue(eventually(lambda: owner.camera_ready('head'), timeout=3))
                self.assertEqual(owner.listing()['cameras'][0]['error'], '')
            finally:
                owner.stop()

    def test_no_frames_rebuilds_after_stopping_old_pipeline_and_clears_cache(self):
        sdk, first = sdk_fixture()
        _, second = sdk_fixture()
        first.wait_for_frames.side_effect = [color_frames(10)] + [None] * 100
        second.wait_for_frames.return_value = color_frames(200)
        events = []
        first.stop.side_effect = lambda: events.append('old stopped')
        second.start.side_effect = lambda cfg: events.append('new started')
        sdk.Pipeline.side_effect = [first, second]
        worker = OrbbecSdkCaptureWorker(camera_id='head', serial='A', fps=100)
        worker.frame_timeout_sec = .08
        worker.reconnect_delay_sec = .15
        with patch.dict('sys.modules', pyorbbecsdk=sdk):
            try:
                worker.start()
                self.assertTrue(eventually(lambda: first.stop.called))
                self.assertIsNone(worker.get_frame(max_stale_sec=0))
                self.assertTrue(eventually(worker.ready))
                self.assertEqual(events[:2], ['old stopped', 'new started'])
                np.testing.assert_array_equal(worker.get_frame(), np.full((2, 2, 3), 200))
                first.stop.assert_called_once()
            finally:
                worker.stop()

    def test_slow_sdk_open_does_not_block_other_roles_or_start_after_stop(self):
        sdk, pipeline = sdk_fixture()
        context = sdk.Context.return_value
        entered, release = threading.Event(), threading.Event()
        def open_context():
            entered.set()
            release.wait(4)
            return context
        sdk.Context.side_effect = open_context
        owner = RokaeCameraOwner(configuration(head={'backend':'orbbec','serial':'A'}))
        with patch.dict('sys.modules', pyorbbecsdk=sdk):
            try:
                before = time.monotonic()
                owner.start()
                self.assertLess(time.monotonic() - before, .5)
                self.assertTrue(entered.wait(1))
                self.assertTrue(owner.camera_ready('hand_right'))
                worker = owner._workers['head']['worker']
                worker._stop.set()
                release.set()
            finally:
                release.set()
                owner.stop()
            pipeline.start.assert_not_called()

    def test_recovery_never_switches_from_initially_unique_serial(self):
        sdk, pipeline = sdk_fixture()
        listing = sdk.Context.return_value.query_devices.return_value
        info = listing.get_device_by_index(0).get_device_info()
        pipeline.wait_for_frames.return_value = None
        pipeline.stop.side_effect = lambda: setattr(info.get_serial_number, 'return_value', 'B')
        worker = OrbbecSdkCaptureWorker(camera_id='head', fps=100)
        worker.frame_timeout_sec = .04
        worker.reconnect_delay_sec = .02
        with patch.dict('sys.modules', pyorbbecsdk=sdk):
            try:
                worker.start()
                self.assertTrue(eventually(lambda: worker.last_error == 'DEVICE_NOT_FOUND'))
                self.assertEqual(worker.serial, 'A')
                pipeline.start.assert_called_once()
                self.assertFalse(worker.ready())
            finally:
                worker.stop()

    def test_failed_native_stop_retains_handle_and_prevents_new_pipeline(self):
        sdk, pipeline = sdk_fixture()
        pipeline.stop.side_effect = RuntimeError('native stop failed')
        worker = OrbbecSdkCaptureWorker(camera_id='head')
        worker.frame_timeout_sec = .03
        with patch.dict('sys.modules', pyorbbecsdk=sdk):
            try:
                worker.start()
                self.assertTrue(eventually(lambda: not worker._thread.is_alive()))
                self.assertEqual(worker.last_error, 'CAPTURE_STOP_FAILED')
                self.assertIs(worker._pipeline, pipeline)
                self.assertFalse(worker.start())
                sdk.Pipeline.assert_called_once()
            finally:
                pipeline.stop.side_effect = None
                worker.stop()
