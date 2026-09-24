"""Bounded ROS RGB cache and nonblocking rawvideo pipe input; no device access."""
from __future__ import annotations

import os
import select
import threading
import time
from dataclasses import dataclass

from vision.rokae_runtime.ros_contract import ros_camera_id


@dataclass(frozen=True)
class RawRgbFrame:
    """A ROS RGB payload retained without channel reversal or ndarray copying."""
    data: object
    width: int
    height: int
    step: int
    pixel_format: str

    @property
    def shape(self):
        return (self.height, self.width, 3)

    def wire_payload(self):
        view = memoryview(self.data).cast('B')
        tight_step = self.width * 3
        if self.step == tight_step:
            return view
        # rawvideo has no row-stride metadata. Strip padding row by row while
        # preserving pixel order and encoding.
        packed = bytearray(tight_step * self.height)
        for row in range(self.height):
            source = row * self.step
            target = row * tight_step
            packed[target:target + tight_step] = view[source:source + tight_step]
        return packed


class LatestRgb:
    def __init__(self, max_age=2.0):
        self.max_age = max_age
        self._lock = threading.Lock()
        self._value = None
        self._sequence = 0

    def receive(self, msg):
        # Retain at most one bounded ROS payload; the encoder handles RGB/BGR.
        try:
            stamp = msg.header.stamp.sec + msg.header.stamp.nanosec / 1e9
            width, height, step = int(msg.width), int(msg.height), int(msg.step)
            encoding = str(msg.encoding).lower()
            if encoding not in {'rgb8', 'bgr8'}:
                raise ValueError(f'unsupported RGB encoding: {encoding}')
            if min(width, height) <= 0 or step < width * 3:
                raise ValueError('invalid RGB image dimensions/stride')
            if not 0 < len(msg.data) <= 32 * 1024 * 1024 or len(msg.data) != step * height:
                raise ValueError('invalid RGB payload size')
            now_wall = time.time()
            # Orbbec USB2 heads can publish with host-skewed stamps; dropping them
            # leaves Media forever "ROS color source stale" while the driver lives.
            remapped = False
            if stamp <= 0 or not -0.25 <= now_wall - stamp <= self.max_age:
                stamp = now_wall
                remapped = True
            frame = RawRgbFrame(
                data=msg.data,
                width=width,
                height=height,
                step=step,
                pixel_format='rgb24' if encoding == 'rgb8' else 'bgr24',
            )
            now = time.monotonic()
            with self._lock:
                if self._value and now - self._value[2] <= self.max_age and stamp <= self._value[1]:
                    if not remapped:
                        return
                    # Remapped arrivals share the same wall second; keep replacing so
                    # a live skewed publisher still advances the Media encode pipe.
                    stamp = max(stamp, self._value[1] + 1e-6)
                self._sequence += 1
                self._value = (frame, stamp, now, self._sequence)
        except (ValueError, TypeError, AttributeError, OverflowError):
            return

    def latest(self):
        with self._lock:
            value = self._value
            if value is None:
                return None
            if time.monotonic() - value[2] > self.max_age:
                return None
            return value[0], value[3]


class RosPushSource:
    def __init__(self, cameras):
        self.caches = {camera: LatestRgb() for camera in cameras}
        self._context = self._node = self._executor = self._thread = None
        self._lock = threading.Lock()

    def latest(self, camera):
        cache = self.caches.get(camera)
        return cache.latest() if cache else None

    def start(self):
        with self._lock:
            if self._node is not None:
                return
            # Lazy imports keep HTTP deployments and unit tests independent of ROS.
            import rclpy
            from rclpy.context import Context
            from rclpy.executors import SingleThreadedExecutor
            from rclpy.node import Node
            from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
            from rclpy.signals import SignalHandlerOptions
            from sensor_msgs.msg import Image
            context = Context()
            node = executor = None
            try:
                rclpy.init(context=context, signal_handler_options=SignalHandlerOptions.NO)
                node = Node('vision_media_rgb', context=context)
                executor = SingleThreadedExecutor(context=context)
                qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT,
                                 history=HistoryPolicy.KEEP_LAST)
                for camera, cache in self.caches.items():
                    node.create_subscription(Image, f'/camera/{ros_camera_id(camera)}/color/image_raw',
                                             cache.receive, qos)
                executor.add_node(node)
                thread = threading.Thread(target=executor.spin, name='media-ros-rgb', daemon=True)
                self._context, self._node, self._executor, self._thread = context, node, executor, thread
                thread.start()
            except Exception:
                if executor is not None:
                    executor.shutdown(timeout_sec=1.0)
                if node is not None:
                    node.destroy_node()
                if context.ok():
                    context.shutdown()
                self._context = self._node = self._executor = self._thread = None
                raise

    def stop(self):
        with self._lock:
            if self._executor is not None:
                self._executor.shutdown(timeout_sec=2.0)
            if self._thread is not None:
                self._thread.join(timeout=2.0)
                if self._thread.is_alive():
                    raise RuntimeError('ROS media executor did not stop')
            if self._node is not None:
                self._node.destroy_node()
            if self._context is not None and self._context.ok():
                self._context.shutdown()
            self._context = self._node = self._executor = self._thread = None
            self.caches = {camera: LatestRgb() for camera in self.caches}


def write_frame(fd, payload, stop, timeout=0.5, source_fresh=None):
    """Finish one raw frame or fail; dropping a partial frame corrupts framing."""
    pending = memoryview(payload)
    deadline = time.monotonic() + timeout
    while pending:
        if stop.is_set():
            raise InterruptedError('rawvideo stopped')
        if source_fresh is not None and not source_fresh():
            raise TimeoutError('ROS color source stale or unavailable')
        remaining = deadline - time.monotonic()
        if remaining <= 0:
            raise TimeoutError('rawvideo pipe blocked')
        if not select.select([], [fd], [], min(remaining, 0.05))[1]:
            continue
        try:
            written = os.write(fd, pending)
        except BlockingIOError:
            continue
        if written <= 0:
            raise BrokenPipeError('rawvideo pipe closed')
        pending = pending[written:]


def feed_frames(process, source, camera, shape, fps, stop, on_frame=None, pixel_format='bgr24'):
    """Only this thread writes this encoder; latest-only sampling drops old frames."""
    fd = process.stdin.fileno()
    os.set_blocking(fd, False)
    last_sequence = None
    started = time.monotonic()
    deadline = started
    while not stop.is_set() and process.poll() is None:
        if stop.wait(max(0, deadline - time.monotonic())):
            return
        latest = source.latest(camera)
        if latest is None:
            raise TimeoutError('ROS color source stale or unavailable')
        frame, sequence = latest
        if frame.shape != shape:
            raise ValueError('ROS color dimensions changed')
        current_format = getattr(frame, 'pixel_format', 'bgr24')
        if current_format != pixel_format:
            raise ValueError('ROS color encoding changed')
        if sequence != last_sequence:
            # FFmpeg can consume a header/frame then pause while connecting RTMP.
            # Startup allowance decreases with elapsed time; steady writes get 2s.
            write_timeout = max(2.0, 10.0 - (time.monotonic() - started))
            payload = frame.wire_payload() if isinstance(frame, RawRgbFrame) else frame.tobytes()
            write_frame(fd, payload, stop, timeout=write_timeout,
                        source_fresh=lambda: source.latest(camera) is not None)
            last_sequence = sequence
            if on_frame is not None:
                on_frame()
        # Do not burst to catch up after a blocked write.
        period = 1.0 / fps
        deadline += period
        now = time.monotonic()
        if deadline < now:
            deadline += (int((now - deadline) / period) + 1) * period
