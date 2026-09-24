import os
import threading
import time
import unittest
from types import SimpleNamespace
from unittest.mock import Mock, patch

import numpy as np

from vision.push import (
    PushRuntime,
    _ffmpeg_command,
    _gstreamer_command,
    nvidia_gstreamer_available,
    resolve_encoder_backend,
)
from vision.ros_push import LatestRgb, RawRgbFrame, RosPushSource, feed_frames, write_frame


def image(stamp=100, value=1):
    return SimpleNamespace(width=2, height=1, step=6, data=bytes([value] * 6), encoding='rgb8',
                           header=SimpleNamespace(stamp=SimpleNamespace(sec=stamp, nanosec=0)))


class CacheTests(unittest.TestCase):
    def test_latest_keeps_host_fresh_frame_and_rejects_empty_cache_when_monotonic_stale(self):
        cache = LatestRgb()
        with patch('vision.ros_push.time.time', return_value=100), \
             patch('vision.ros_push.time.monotonic', return_value=10):
            cache.receive(image(99, 1))
            cache.receive(image(100, 2))
            # Skewed device stamps are remapped to host time and still accepted.
            cache.receive(image(90, 4))
            cache.receive(image(105, 5))
            bad = image(); bad.encoding = '16UC1'
            cache.receive(bad)
            frame, seq = cache.latest()
            self.assertEqual(seq, 4)
            self.assertIsInstance(frame, RawRgbFrame)
            self.assertEqual(frame.pixel_format, 'rgb24')
            self.assertEqual(bytes(frame.wire_payload()), bytes([5] * 6))
        with patch('vision.ros_push.time.time', return_value=103), \
             patch('vision.ros_push.time.monotonic', return_value=13):
            self.assertIsNone(cache.latest())

    def test_timestamp_regression_recovers_after_stale_stream(self):
        cache = LatestRgb()
        with patch('vision.ros_push.time.time', return_value=100), \
             patch('vision.ros_push.time.monotonic', return_value=10):
            cache.receive(image())
        with patch('vision.ros_push.time.time', return_value=99), \
             patch('vision.ros_push.time.monotonic', return_value=13):
            cache.receive(image(99, 2))
            self.assertEqual(cache.latest()[1], 2)

    def test_rgb_payload_is_preserved_and_padded_rows_are_packed(self):
        cache = LatestRgb()
        msg = image(); msg.data = bytes([1, 2, 3, 4, 5, 6])
        with patch('vision.ros_push.time.time', return_value=100):
            cache.receive(msg)
            frame, sequence = cache.latest()
            self.assertEqual(frame.pixel_format, 'rgb24')
            self.assertEqual(bytes(frame.wire_payload()), msg.data)
        padded = image(stamp=101, value=0)
        padded.step = 8
        padded.data = bytes([1, 2, 3, 4, 5, 6, 99, 99])
        with patch('vision.ros_push.time.time', return_value=101):
            cache.receive(padded)
            frame, sequence = cache.latest()
            self.assertEqual(bytes(frame.wire_payload()), bytes([1, 2, 3, 4, 5, 6]))
        msg.data = Mock(); msg.data.__len__ = Mock(return_value=33 * 1024 * 1024)
        cache.receive(msg)
        self.assertEqual(cache.latest()[1], sequence)


class PipeTests(unittest.TestCase):
    def test_partial_writes_preserve_whole_frame(self):
        writes = []
        def write(fd, data):
            writes.append(bytes(data)); return min(2, len(data))
        with patch('vision.ros_push.select.select', return_value=([], [1], [])), \
             patch('vision.ros_push.os.write', side_effect=write):
            write_frame(1, b'abcdef', threading.Event())
        self.assertEqual(writes, [b'abcdef', b'cdef', b'ef'])

    def test_real_full_pipe_times_out_and_stop_interrupts(self):
        read_fd, write_fd = os.pipe()
        try:
            os.set_blocking(write_fd, False)
            while True:
                try:
                    os.write(write_fd, bytes(4096))
                except BlockingIOError:
                    break
            start = time.monotonic()
            with self.assertRaises(TimeoutError):
                write_frame(write_fd, b'x', threading.Event(), timeout=0.08)
            self.assertLess(time.monotonic() - start, 0.5)
            stop = threading.Event(); stop.set()
            with self.assertRaises(InterruptedError):
                write_frame(write_fd, b'x', stop)
        finally:
            os.close(read_fd); os.close(write_fd)

    def test_delayed_pipe_reader_startup_over_half_second_succeeds(self):
        read_fd, write_fd = os.pipe()
        stop = threading.Event()
        payload = bytes(256 * 1024)
        received = bytearray()
        def reader():
            time.sleep(0.7)
            while len(received) < len(payload):
                received.extend(os.read(read_fd, 65536))
        thread = threading.Thread(target=reader)
        try:
            os.set_blocking(write_fd, False)
            thread.start()
            write_frame(write_fd, payload, stop, timeout=10.0)
            thread.join(2)
            self.assertFalse(thread.is_alive())
            self.assertEqual(bytes(received), payload)
        finally:
            os.close(write_fd)
            thread.join(2)
            os.close(read_fd)

    def test_source_expiry_interrupts_blocked_startup_write(self):
        read_fd, write_fd = os.pipe()
        try:
            os.set_blocking(write_fd, False)
            while True:
                try:
                    os.write(write_fd, bytes(4096))
                except BlockingIOError:
                    break
            started = time.monotonic()
            fresh = lambda: time.monotonic() - started < 0.1
            with self.assertRaisesRegex(TimeoutError, 'ROS color source stale'):
                write_frame(write_fd, b'x', threading.Event(), timeout=10, source_fresh=fresh)
            self.assertLess(time.monotonic() - started, 0.5)
        finally:
            os.close(read_fd); os.close(write_fd)

    def test_feed_startup_grace_expires_without_per_frame_reset(self):
        now = [0.0]
        class Stop:
            def is_set(self): return False
            def wait(self, delay): now[0] += delay; return False
        frame = np.zeros((1, 2, 3), dtype=np.uint8)
        source = Mock(); source.latest.side_effect = [(frame, 1), (frame, 2), (frame, 3), None]
        process = Mock(); process.poll.return_value = None
        budgets = []
        def write(*args, timeout, **kwargs):
            budgets.append(timeout)
            now[0] += 5
        with patch('vision.ros_push.os.set_blocking'), \
             patch('vision.ros_push.time.monotonic', side_effect=lambda: now[0]), \
             patch('vision.ros_push.write_frame', side_effect=write):
            with self.assertRaises(TimeoutError):
                feed_frames(process, source, 'head', frame.shape, 8, Stop())
        self.assertEqual(budgets[0], 10.0)
        self.assertLess(budgets[1], 5.0)
        self.assertEqual(budgets[2], 2.0)

    def test_pacing_latest_only_no_duplicate_and_stale_exit(self):
        now = [0.0]
        class Stop:
            def is_set(self): return False
            def wait(self, delay): now[0] += delay; return False
        frame = np.zeros((1, 2, 3), dtype=np.uint8)
        source = Mock()
        source.latest.side_effect = [(frame, 1), (frame, 1), (frame, 7), None]
        process = Mock(); process.poll.return_value = None
        sent = []
        with patch('vision.ros_push.os.set_blocking'), \
             patch('vision.ros_push.time.monotonic', side_effect=lambda: now[0]), \
             patch('vision.ros_push.write_frame', side_effect=lambda *a, **kw: (sent.append(now[0]), now.__setitem__(0, now[0] + 0.03))):
            with self.assertRaisesRegex(TimeoutError, 'stale'):
                feed_frames(process, source, 'head', frame.shape, 8, Stop())
        self.assertEqual(sent, [0.0, 0.25])

    def test_feed_preserves_rgb_order_strips_padding_and_counts_progress(self):
        now = [0.0]
        class Stop:
            def is_set(self): return False
            def wait(self, delay): now[0] += delay; return False
        frame = RawRgbFrame(
            data=bytes([1, 2, 3, 4, 5, 6, 99, 99]),
            width=2, height=1, step=8, pixel_format='rgb24',
        )
        source = Mock(); source.latest.side_effect = [(frame, 1), None]
        process = Mock(); process.poll.return_value = None
        payloads = []
        progress = Mock()
        with patch('vision.ros_push.os.set_blocking'), \
             patch('vision.ros_push.time.monotonic', side_effect=lambda: now[0]), \
             patch('vision.ros_push.write_frame', side_effect=lambda fd, payload, *a, **kw: payloads.append(bytes(payload))):
            with self.assertRaisesRegex(TimeoutError, 'stale'):
                feed_frames(
                    process, source, 'head', frame.shape, 8, Stop(),
                    on_frame=progress, pixel_format='rgb24',
                )
        self.assertEqual(payloads, [bytes([1, 2, 3, 4, 5, 6])])
        progress.assert_called_once_with()


class FakeProcess:
    def __init__(self):
        self.read_fd, write_fd = os.pipe()
        self.stdin = os.fdopen(write_fd, 'wb', buffering=0)
        self.stderr = None
        self.code = None
    def poll(self): return self.code
    def kill(self): self.code = -9
    def terminate(self): self.code = -15
    def wait(self, timeout=None): return self.code
    def close(self):
        self.stdin.close(); os.close(self.read_fd)


class RuntimeTests(unittest.TestCase):
    def setUp(self):
        # Other discovery-order tests intentionally import device-owner modules.
        # PushRuntime's ownership guard has its own focused tests; isolate these
        # encoder/lifecycle tests from process-global sys.modules state.
        self.ownership_guard = patch('vision.ownership.ensure_ros_media_subscriber_only')
        self.ownership_guard.start()
        self.addCleanup(self.ownership_guard.stop)

    def config(self):
        return {'media': {'push': {'source': 'ros', 'encoder_backend': 'ffmpeg', 'device_sn': 'TEST',
            'streams': [{'camera_id': 'head', 'width': 640, 'height': 480, 'fps': 8,
                         'restart_initial_sec': 0.1}, {'camera_id': 'hand_left', 'enabled': False}]}}}

    def test_rawvideo_args_keep_filter_and_encoder(self):
        cmd = _ffmpeg_command(ffmpeg_bin='ffmpeg', source='pipe:0', target='rtmp://test/live',
                              width=640, height=480, fps=8, bitrate='1200k', preset='ultrafast',
                              raw_shape=(720, 1280, 3))
        for flag, value in [('-pixel_format', 'bgr24'), ('-video_size', '1280x720'),
                            ('-framerate', '8'), ('-i', 'pipe:0'), ('-c:v', 'libx264')]:
            self.assertEqual(cmd[cmd.index(flag) + 1], value)
        self.assertIn('force_original_aspect_ratio=decrease', cmd[cmd.index('-vf') + 1])
        self.assertNotIn('-reconnect', cmd)
        self.assertEqual(cmd[cmd.index('-g') + 1], '8')
        self.assertEqual(cmd[cmd.index('-bf') + 1], '0')

    def test_nvidia_command_is_bounded_low_latency_h264(self):
        cmd = _gstreamer_command(
            gst_launch_bin='gst-launch-1.0', target='rtmp://test/live',
            width=640, height=480, fps=8, bitrate='1200k', raw_shape=(720, 1280, 3),
            raw_pixel_format='rgb24',
        )
        joined = ' '.join(cmd)
        self.assertIn('queue max-size-buffers=1', joined)
        self.assertIn('leaky=downstream', joined)
        self.assertIn('format=rgb', cmd)
        self.assertTrue(any(value.startswith('video/x-raw(memory:NVMM),format=NV12') for value in cmd))
        self.assertIn('nvv4l2h264enc', cmd)
        self.assertIn('bitrate=1200000', cmd)
        self.assertIn('idrinterval=8', cmd)
        self.assertIn('num-B-Frames=0', cmd)
        self.assertIn('rtmpsink', cmd)

    def test_capability_detection_requires_every_plugin(self):
        completed = SimpleNamespace(returncode=0)
        with patch('vision.push.os.access', return_value=True):
            run = Mock(return_value=completed)
            self.assertTrue(nvidia_gstreamer_available(
                '/usr/bin/gst-launch-1.0', '/usr/bin/gst-inspect-1.0', run=run))
            self.assertGreater(run.call_count, 1)
            run.reset_mock()
            run.side_effect = [completed, SimpleNamespace(returncode=1)]
            self.assertFalse(nvidia_gstreamer_available(
                '/usr/bin/gst-launch-1.0', '/usr/bin/gst-inspect-1.0', run=run))

    def test_backend_auto_falls_back_but_explicit_nvidia_fails_closed(self):
        with patch('vision.push.nvidia_gstreamer_available', return_value=False):
            selected, reason = resolve_encoder_backend('auto', source='ros')
            self.assertEqual(selected, 'ffmpeg')
            self.assertIn('unavailable', reason)
            with self.assertRaisesRegex(ValueError, 'required executables/plugins'):
                resolve_encoder_backend('nvidia_gstreamer', source='ros')
        with patch('vision.push.nvidia_gstreamer_available', return_value=True):
            self.assertEqual(resolve_encoder_backend('auto', source='ros')[0], 'nvidia_gstreamer')

    def test_ros_isolated_from_http_and_single_feeder_on_repeated_start_stop(self):
        source = Mock(); source.latest.return_value = (np.zeros((1, 2, 3), dtype=np.uint8), 1)
        processes = []
        def spawn(*args, **kwargs):
            process = FakeProcess(); processes.append(process); return process
        http = Mock(side_effect=AssertionError('HTTP must not be called'))
        entered = threading.Event()
        def feed(process, source, camera, shape, fps, stop, on_frame=None, pixel_format='bgr24'):
            if on_frame:
                on_frame()
            entered.set(); stop.wait(2)
        with patch('vision.ros_push.RosPushSource', return_value=source) as factory, \
             patch('vision.ros_push.feed_frames', side_effect=feed):
            runtime = PushRuntime(self.config(), source_uri=http, popen_factory=spawn)
            factory.assert_called_once_with(['head'])
            try:
                runtime.start(); self.assertTrue(entered.wait(1))
                runtime.start()
                self.assertEqual(len(processes), 1)
                feeder = runtime._workers['head']._feeder
                runtime.stop_all()
                self.assertFalse(feeder.is_alive())
                runtime.start()
                self.assertEqual(len(processes), 2)
                self.assertEqual(runtime.status()['source'], 'ros')
                stream = runtime.status()['streams'][0]
                self.assertEqual(stream['frames_sent'], 2)
                self.assertIsNotNone(stream['progress_age_sec'])
                http.assert_not_called()
            finally:
                runtime.stop_all()
                for process in processes: process.close()

    def test_runtime_start_waits_for_shared_source_shutdown(self):
        source = Mock(); source.latest.return_value = None
        stopping, release, started = threading.Event(), threading.Event(), threading.Event()
        source.stop.side_effect = lambda: (stopping.set(), release.wait(2))
        source.start.side_effect = started.set
        with patch('vision.ros_push.RosPushSource', return_value=source):
            runtime = PushRuntime(self.config(), source_uri=Mock(side_effect=AssertionError))
            shutdown = threading.Thread(target=runtime.stop_all)
            restart = threading.Thread(target=runtime.start)
            try:
                shutdown.start(); self.assertTrue(stopping.wait(1))
                restart.start()
                self.assertFalse(started.wait(0.05))
                release.set()
                shutdown.join(1); restart.join(1)
                self.assertTrue(started.is_set())
                self.assertFalse(restart.is_alive())
                self.assertTrue(runtime.status()['streams'][0]['desired'])
            finally:
                release.set(); shutdown.join(1); restart.join(1)
                runtime.stop_all()

    def test_stop_while_encoder_pipe_is_blocked_is_bounded(self):
        source = Mock()
        source.latest.return_value = (np.zeros((480, 640, 3), dtype=np.uint8), 1)
        processes = []
        def spawn(*args, **kwargs):
            process = FakeProcess(); processes.append(process); return process
        with patch('vision.ros_push.RosPushSource', return_value=source):
            runtime = PushRuntime(self.config(), source_uri=Mock(side_effect=AssertionError), popen_factory=spawn)
            try:
                runtime.start()
                feeder = runtime._workers['head']._feeder
                time.sleep(0.05)
                start = time.monotonic()
                runtime.stop_all()
                self.assertLess(time.monotonic() - start, 1.5)
                self.assertFalse(feeder.is_alive())
                self.assertIsNotNone(processes[0].poll())
            finally:
                runtime.stop_all()
                for process in processes: process.close()

    def test_stale_encoder_killed_then_source_reconnect_starts_new_encoder(self):
        source = Mock(); frame = np.zeros((1, 2, 3), dtype=np.uint8)
        source.latest.return_value = (frame, 1)
        processes = []
        def spawn(*args, **kwargs):
            process = FakeProcess(); processes.append(process); return process
        failed = threading.Event()
        def feed(process, source, camera, shape, fps, stop, on_frame=None, pixel_format='bgr24'):
            if len(processes) == 1:
                source.latest.return_value = None
                failed.set()
                raise TimeoutError('ROS color source stale or unavailable')
            stop.wait(3)
        with patch('vision.ros_push.RosPushSource', return_value=source), \
             patch('vision.ros_push.feed_frames', side_effect=feed):
            runtime = PushRuntime(self.config(), source_uri=Mock(side_effect=AssertionError), popen_factory=spawn)
            try:
                runtime.start(); self.assertTrue(failed.wait(1))
                deadline = time.monotonic() + 2
                while processes[0].poll() is None and time.monotonic() < deadline:
                    time.sleep(0.01)
                self.assertEqual(processes[0].poll(), -9)
                source.latest.return_value = (frame, 2)
                while len(processes) < 2 and time.monotonic() < deadline:
                    time.sleep(0.02)
                self.assertEqual(len(processes), 2)
            finally:
                runtime.stop_all()
                for process in processes: process.close()

    def test_explicit_nvidia_backend_spawns_gstreamer_only(self):
        cfg = self.config()
        cfg['media']['push']['encoder_backend'] = 'nvidia_gstreamer'
        source = Mock(); source.latest.return_value = (np.zeros((1, 2, 3), dtype=np.uint8), 1)
        processes = []
        def spawn(command, **kwargs):
            process = FakeProcess(); processes.append(process)
            self.assertEqual(command[0], 'gst-launch-1.0')
            self.assertIn('nvv4l2h264enc', command)
            return process
        def feed(*args, **kwargs):
            args[5].wait(2)
        with patch('vision.push.nvidia_gstreamer_available', return_value=True), \
             patch('vision.ros_push.RosPushSource', return_value=source), \
             patch('vision.ros_push.feed_frames', side_effect=feed):
            runtime = PushRuntime(cfg, source_uri=Mock(side_effect=AssertionError), popen_factory=spawn)
            try:
                runtime.start()
                self.assertEqual(runtime.status()['encoder_backend'], 'nvidia_gstreamer')
                self.assertEqual(runtime.status()['streams'][0]['encoder_backend'], 'nvidia_gstreamer')
            finally:
                runtime.stop_all()
                for process in processes: process.close()

    def test_feeder_fault_reason_survives_encoder_exit_and_watchdog_cleanup(self):
        source = Mock(); source.latest.return_value = (np.zeros((1, 2, 3), dtype=np.uint8), 1)
        processes = []
        def spawn(*args, **kwargs):
            process = FakeProcess(); processes.append(process); return process
        cfg = self.config()
        cfg['media']['push']['streams'][0]['restart_initial_sec'] = 5
        with patch('vision.ros_push.RosPushSource', return_value=source), \
             patch('vision.ros_push.feed_frames', side_effect=TimeoutError('rawvideo pipe blocked')):
            runtime = PushRuntime(cfg, source_uri=Mock(side_effect=AssertionError), popen_factory=spawn)
            try:
                runtime.start()
                deadline = time.monotonic() + 2
                while runtime._workers['head']._process is not None and time.monotonic() < deadline:
                    if processes[0].poll() is not None:
                        self.assertEqual(runtime.status()['streams'][0]['reason'], 'rawvideo pipe blocked')
                    time.sleep(0.02)
                self.assertIsNone(runtime._workers['head']._process)
                self.assertEqual(runtime.status()['streams'][0]['reason'], 'rawvideo pipe blocked')
            finally:
                runtime.stop_all()
                for process in processes: process.close()

    def test_invalid_source_is_explicit_error(self):
        cfg = self.config(); cfg['media']['push']['source'] = 'typo'
        with self.assertRaises(ValueError):
            PushRuntime(cfg, source_uri=Mock())


if __name__ == '__main__':
    unittest.main()
