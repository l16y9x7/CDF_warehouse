"""Read externally managed RGB-D cameras. Never open USB or launch a driver."""
from __future__ import annotations

import copy
import json
import math
import os
import subprocess
import threading
import time
from collections import OrderedDict, deque
from datetime import datetime
from pathlib import Path
from typing import Any

from .rgb_recording import RgbRecordingStore
from .backends import BackendError
from .camera import CameraSnapshot
from .capture_preview import CAPTURE_URL, capture_rgb
from .camera_manager import CAMERA_LABELS, MultiCameraRecordingStore


def stamp_ns(message: Any) -> int:
    return int(message.header.stamp.sec) * 1_000_000_000 + int(message.header.stamp.nanosec)


def _image_array(message: Any, dtype: str, channels: int = 1) -> Any:
    import numpy as np

    width, height, step = int(message.width), int(message.height), int(message.step)
    item = np.dtype(dtype).newbyteorder('>' if message.is_bigendian else '<')
    row_bytes = width * channels * item.itemsize
    if min(width, height) <= 0 or step < row_bytes or len(message.data) != step * height:
        raise BackendError('ROS 图像的尺寸、步长或数据长度无效')
    shape = (height, width, channels) if channels > 1 else (height, width)
    strides = (step, channels * item.itemsize, item.itemsize) if channels > 1 else (step, item.itemsize)
    return np.ndarray(shape, dtype=item, buffer=memoryview(message.data), strides=strides).copy()


def decode_color(message: Any) -> Any:
    import numpy as np

    encoding = str(message.encoding).lower()
    if encoding in ('bgr8', 'rgb8'):
        result = _image_array(message, 'u1', 3)
        return result if encoding == 'bgr8' else result[:, :, ::-1].copy()
    if encoding in ('bgra8', 'rgba8'):
        result = _image_array(message, 'u1', 4)[:, :, :3]
        return result.copy() if encoding == 'bgra8' else result[:, :, ::-1].copy()
    if encoding in ('mono8', '8uc1'):
        return np.repeat(_image_array(message, 'u1')[:, :, None], 3, axis=2)
    raise BackendError(f'不支持的 ROS RGB 编码: {message.encoding}')


def decode_depth_mm(message: Any) -> Any:
    import numpy as np

    encoding = str(message.encoding).lower()
    if encoding in ('16uc1', 'mono16'):
        depth = _image_array(message, 'u2').astype(np.float32)
    elif encoding == '32fc1':
        depth = _image_array(message, 'f4').astype(np.float32) * 1000.0
    else:
        raise BackendError(f'不支持的 ROS 深度编码: {message.encoding}')
    depth[~np.isfinite(depth) | (depth <= 0)] = 0.0
    return depth


def color_intrinsics(info: Any) -> dict[str, Any]:
    k, d = [float(x) for x in info.k], [float(x) for x in info.d]
    if len(k) != 9 or not all(math.isfinite(x) for x in k + d) or k[0] <= 0 or k[4] <= 0:
        raise BackendError('ROS CameraInfo 内参无效')
    return dict(width=int(info.width), height=int(info.height), fx=k[0], fy=k[4],
                cx=k[2], cy=k[5], camera_matrix=[k[:3], k[3:6], k[6:]],
                distortion_model=info.distortion_model, distortion_coefficients=d)


class RosCameraStream:
    """Bounded, exact-stamp pairing of upstream synced RGB/depth/CameraInfo."""
    def __init__(self, camera_id: str, config: dict[str, Any], stale_seconds: float = 2.0) -> None:
        self.camera_id, self.config = camera_id, dict(config)
        self.stale_seconds = stale_seconds
        self.condition = threading.Condition()
        self.pending: dict[str, OrderedDict[int, Any]] = {
            name: OrderedDict() for name in ('color', 'depth', 'info')
        }
        self.latest: tuple[Any, Any, Any] | None = None
        self.sequence = 0
        self.received_monotonic = 0.0
        self.receipt_times: deque[float] = deque(maxlen=31)
        self.error = ''
        self.closed = False

    def receive(self, name: str, message: Any) -> None:
        timestamp = stamp_ns(message)
        age = time.time() - timestamp / 1e9
        if timestamp <= 0 or not -0.25 <= age <= self.stale_seconds:
            return
        with self.condition:
            if self.closed:
                return
            queue = self.pending[name]
            queue[timestamp] = message
            while len(queue) > 8:
                queue.popitem(last=False)
            if not all(timestamp in cache for cache in self.pending.values()):
                return
            color, depth, info = (self.pending[key][timestamp] for key in ('color', 'depth', 'info'))
            try:
                dimensions = {(int(m.width), int(m.height)) for m in (color, depth, info)}
                if len(dimensions) != 1 or min(dimensions.pop()) <= 0:
                    raise BackendError('RGB、对齐深度与 CameraInfo 尺寸不一致')
                expected = (int(self.config['width']), int(self.config['height']))
                if (color.width, color.height) != expected:
                    raise BackendError(f'ROS RGB 分辨率与配置不符，期望 {expected[0]}×{expected[1]}')
                if not color.header.frame_id or len({m.header.frame_id for m in (color, depth, info)}) != 1:
                    raise BackendError('RGB、深度与 CameraInfo 不是同一个光学坐标系')
                color_intrinsics(info)
                for message, supported in (
                    (color, {'rgb8': 3, 'bgr8': 3, 'rgba8': 4, 'bgra8': 4, 'mono8': 1, '8uc1': 1}),
                    (depth, {'16uc1': 2, 'mono16': 2, '32fc1': 4}),
                ):
                    pixel_bytes = supported.get(str(message.encoding).lower())
                    if pixel_bytes is None:
                        raise BackendError(f'不支持的 ROS 图像编码: {message.encoding}')
                    if (message.step < message.width * pixel_bytes
                            or len(message.data) != message.step * message.height):
                        raise BackendError('ROS RGB-D 数据长度或步长无效')
            except BackendError as exc:
                self.error = str(exc)
                self.condition.notify_all()
                return
            if self.latest is not None and timestamp <= stamp_ns(self.latest[0]):
                return
            self.latest = (color, depth, info)
            self.sequence += 1
            self.received_monotonic = time.monotonic()
            self.receipt_times.append(self.received_monotonic)
            self.error = ''
            for cache in self.pending.values():
                for old_stamp in list(cache):
                    if old_stamp <= timestamp:
                        del cache[old_stamp]
            self.condition.notify_all()

    def _fresh_locked(self) -> bool:
        return bool(not self.closed and self.latest is not None
                    and time.monotonic() - self.received_monotonic <= self.stale_seconds
                    and -0.25 <= time.time() - stamp_ns(self.latest[0]) / 1e9 <= self.stale_seconds)

    def status(self) -> dict[str, Any]:
        with self.condition:
            enabled = self._fresh_locked()
            fps = ((len(self.receipt_times) - 1) / (self.receipt_times[-1] - self.receipt_times[0])
                   if len(self.receipt_times) > 1 and self.receipt_times[-1] > self.receipt_times[0] else 0.0)
            return dict(enabled=enabled, starting=False, source='ros2', externally_managed=True,
                        model=self.config.get('model', CAMERA_LABELS[self.camera_id]),
                        sequence=self.sequence, actual_capture_fps=fps if enabled else 0.0,
                        rgb_profile=dict(width=self.config['width'], height=self.config['height'],
                                         fps=self.config.get('fps', 15)),
                        alignment='depth_to_color', topics=dict(self.config['topics']),
                        error=self.error or ('' if enabled else '等待 ROS2 新的同步 RGB-D 数据'))

    def snapshot(self, after_sequence: int | None = None, timeout: float = 3.0,
                 fresh: bool = False) -> CameraSnapshot:
        requested_at = time.time() if fresh else 0.0
        deadline = time.monotonic() + timeout
        with self.condition:
            floor = self.sequence if fresh and after_sequence is None else (after_sequence or 0)
            while True:
                if self.closed:
                    raise BackendError('ROS2 摄像头订阅已关闭')
                if (self._fresh_locked() and self.sequence > floor
                        and stamp_ns(self.latest[0]) / 1e9 >= requested_at):
                    messages, sequence = self.latest, self.sequence
                    break
                remaining = deadline - time.monotonic()
                if remaining <= 0:
                    raise BackendError(self.error or '等待 ROS2 新的同步 RGB-D 帧超时；可点击重启摄像头服务（三路）')
                self.condition.wait(remaining)
        color, depth, info = messages
        rgb, depth_mm = decode_color(color), decode_depth_mm(depth)
        timestamp = stamp_ns(color)
        metadata = dict(source='ros2', model=self.config.get('model', CAMERA_LABELS[self.camera_id]),
                        serial_number=self.config.get('serial_number', ''),
                        alignment='depth_to_color', depth_unit='millimeter',
                        rgb_profile=dict(width=color.width, height=color.height, fps=self.config.get('fps', 15)),
                        depth_profile=dict(width=depth.width, height=depth.height, fps=self.config.get('fps', 15)),
                        color_intrinsics=color_intrinsics(info),
                        ros_topics=dict(self.config['topics']), ros_frame_id=color.header.frame_id,
                        color_encoding=color.encoding, depth_encoding=depth.encoding,
                        ros_color_stamp_ns=timestamp, ros_depth_stamp_ns=stamp_ns(depth),
                        ros_camera_info_stamp_ns=stamp_ns(info),
                        synchronization='upstream_synced_rgbd_retimestamped_to_color',
                        original_depth_timestamp_available=False,
                        fetched_at=datetime.now().astimezone().isoformat(timespec='milliseconds'))
        return CameraSnapshot(rgb_bgr=rgb, depth_aligned_mm=depth_mm,
                              captured_at=datetime.fromtimestamp(timestamp / 1e9).astimezone().isoformat(timespec='milliseconds'),
                              color_timestamp_ms=timestamp / 1e6, depth_timestamp_ms=stamp_ns(depth) / 1e6,
                              sequence=sequence, camera_info=metadata)

    def close(self) -> None:
        with self.condition:
            self.closed = True
            self.latest = None
            for cache in self.pending.values():
                cache.clear()
            self.condition.notify_all()


class RosColorStream(RosCameraStream):
    """Track RGB freshness independently; this camera intentionally has no depth."""
    def receive(self, name: str, message: Any) -> None:
        if name != 'color':
            return
        timestamp = stamp_ns(message)
        if timestamp <= 0 or not -0.25 <= time.time() - timestamp / 1e9 <= self.stale_seconds:
            return
        with self.condition:
            if self.closed:
                return
            if self.latest is not None and timestamp <= stamp_ns(self.latest[0]):
                return
            if ((message.width, message.height) != (self.config['width'], self.config['height'])
                    or message.encoding not in ('rgb8', 'bgr8')
                    or message.step < message.width * 3
                    or len(message.data) != message.step * message.height):
                self.error = 'ROS RGB 图像尺寸、编码或数据长度无效'
                return
            self.latest = (message, None, None)
            self.sequence += 1
            self.received_monotonic = time.monotonic()
            self.receipt_times.append(self.received_monotonic)
            self.error = ''
            self.condition.notify_all()

    def status(self) -> dict[str, Any]:
        status = super().status()
        status.update(color_only=True, alignment='none')
        if status['error'] == '等待 ROS2 新的同步 RGB-D 数据':
            status['error'] = '等待 ROS2 新的 RGB 数据'
        return status

    def snapshot(self, **kwargs) -> CameraSnapshot:
        raise BackendError('左手相机仅提供 RGB；请使用采集左手 RGB 画面按钮')

    def fresh_jpeg(self, timeout, quality, cancelled):
        import cv2
        requested_at = time.time_ns()
        deadline = time.monotonic() + timeout
        with self.condition:
            floor = self.sequence
            while True:
                if cancelled.is_set():
                    raise BackendError('扫码采集已停止')
                if self.closed:
                    raise BackendError('左腕 RGB 订阅已关闭')
                if (self._fresh_locked() and self.sequence > floor
                        and stamp_ns(self.latest[0]) >= requested_at):
                    message, sequence = self.latest[0], self.sequence
                    break
                remaining = deadline - time.monotonic()
                if remaining <= 0:
                    raise BackendError(self.error or '等待到位后的左腕新 RGB 帧超时')
                self.condition.wait(min(remaining, 0.1))
        ok, encoded = cv2.imencode('.jpg', decode_color(message), [cv2.IMWRITE_JPEG_QUALITY, quality])
        if not ok:
            raise BackendError('左腕 RGB 图片编码失败')
        return encoded.tobytes(), dict(sequence=sequence, timestamp_ns=stamp_ns(message))


# Compatibility name for callers of the original recovery component.
from .camera_restart import CameraServiceRestart as ExistingCameraRecovery


class RosCameraManager:
    externally_managed = True

    def __init__(self, config: dict[str, Any], data_root: str | Path, jpeg_quality: int = 80) -> None:
        self.config = config
        self.jpeg_quality = int(jpeg_quality)
        self.streams = {name: (RosColorStream if cfg.get('color_only') else RosCameraStream)(
                        name, cfg, float(config.get('stale_seconds', 2.0)))
                        for name, cfg in config['cameras'].items() if cfg.get('enabled', False)}
        self.store = MultiCameraRecordingStore(data_root)
        self.rgb_store = RgbRecordingStore(config.get('left_wrist_rgb_directory',
                                                   str(Path(data_root).parent / 'left_wrist_rgb')))
        self.recovery = ExistingCameraRecovery()
        self.context = self.node = self.executor = self.thread = None
        self.init_error = ''
        self.domain_id: int | None = None
        self.capture_timeout = float(config.get('capture_timeout_seconds', 3.0))
        self._subscribe()

    def _subscribe(self) -> None:
        try:
            import rclpy
            from rclpy.context import Context
            from rclpy.executors import SingleThreadedExecutor
            from rclpy.node import Node
            from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
            from rclpy.signals import SignalHandlerOptions
            from sensor_msgs.msg import CameraInfo, Image

            domain = self.config.get('domain_id')
            if domain is None:
                # Source the existing resolver in a child shell only. Never change the
                # main process ROS environment/domain used by the chassis adapter.
                resolved = subprocess.run(
                    ['/bin/bash', '-c', 'source "$1" && printf "%s" "$ROS_DOMAIN_ID"',
                     'rokae-camera-domain', self.config['setup_file']],
                    capture_output=True, text=True, check=True, timeout=10)
                domain = int(resolved.stdout.strip())
            self.domain_id = int(domain)
            self.context = Context()
            rclpy.init(args=[], context=self.context, domain_id=self.domain_id,
                        signal_handler_options=SignalHandlerOptions.NO)
            self.node = Node(f'rokae_web_rgbd_{os.getpid()}', context=self.context)
            qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                             durability=DurabilityPolicy.VOLATILE,
                             history=HistoryPolicy.KEEP_LAST, depth=8)
            for stream in self.streams.values():
                for name, kind in [('color', Image), ('depth', Image), ('info', CameraInfo)]:
                    if stream.config.get('color_only') and name != 'color':
                        continue
                    self.node.create_subscription(kind, stream.config['topics'][name],
                        lambda message, stream=stream, name=name: stream.receive(name, message), qos)
            self.executor = SingleThreadedExecutor(context=self.context)
            self.executor.add_node(self.node)
            self.thread = threading.Thread(target=self._spin, name='rokae-ros-rgbd', daemon=True)
            self.thread.start()
        except Exception as exc:
            self.init_error = f'ROS2 摄像头订阅初始化失败: {exc}'
            self._close_ros()

    def _spin(self) -> None:
        try:
            self.executor.spin()
        except Exception as exc:
            self.init_error = f'ROS2 摄像头订阅异常: {exc}'
            for stream in self.streams.values():
                stream.close()

    def _stream(self, camera_id: str) -> RosCameraStream:
        if camera_id not in self.streams:
            raise BackendError('该摄像头未配置 ROS2 订阅')
        if self.init_error:
            raise BackendError(self.init_error)
        return self.streams[camera_id]

    def status(self) -> dict[str, dict[str, Any]]:
        result = {}
        for name, label in CAMERA_LABELS.items():
            configured = name in self.streams
            status = self.streams[name].status() if configured else dict(enabled=False, source='ros2')
            status.update(camera_id=name, label=label, available=configured and not self.init_error,
                          externally_managed=True, ros_domain_id=self.domain_id,
                          frame_available=configured)
            if self.init_error and configured:
                status.update(enabled=False, error=self.init_error)
            status.update(self.recovery.status())
            result[name] = status
        return result

    def set_enabled(self, camera_id: str, enabled: bool) -> dict[str, Any]:
        del camera_id, enabled
        raise BackendError('摄像头由现有开机服务管理，网页不再启停摄像头；异常时请点击重启摄像头服务（三路）')

    def restart(self, camera_id: str) -> dict[str, Any]:
        if camera_id != 'all':
            raise BackendError('请刷新网页，使用三路摄像头服务重启按钮')
        return self.recovery.trigger()

    def snapshot(self, camera_id: str) -> CameraSnapshot:
        return self._stream(camera_id).snapshot(timeout=self.capture_timeout)

    def fresh_snapshot(self, camera_id: str, timeout: float | None = None) -> CameraSnapshot:
        return self._stream(camera_id).snapshot(timeout=self.capture_timeout if timeout is None else timeout, fresh=True)

    def frame_jpeg(self, camera_id: str, kind: str, after_sequence: int,
                   timeout: float) -> tuple[bytes | None, int, bool]:
        if kind != 'rgb':
            raise BackendError('网页只展示 RGB 单帧')
        if camera_id not in self.streams:
            raise BackendError('该摄像头未配置 ROS2 订阅')
        # Preview follows GET /camera/capture; synchronized RGB-D recording and
        # pose estimation continue through their existing ROS snapshot methods.
        frame = capture_rgb(camera_id, timeout,
                            url=self.config.get('capture_url', CAPTURE_URL))
        return frame, time.time_ns(), True

    def record_rgb(self, camera_id: str) -> dict[str, Any]:
        if camera_id != 'left_wrist' or camera_id not in self.streams:
            raise BackendError('仅左手相机支持独立 RGB 采集')
        jpeg = capture_rgb(camera_id, self.capture_timeout,
                           url=self.config.get('capture_url', CAPTURE_URL))
        return self.rgb_store.save(jpeg)

    def begin_scan_recording(self):
        self._stream('left_wrist')
        return self.rgb_store.begin_scan()

    def record_scan_rgb(self, group, index, cancelled):
        jpeg, metadata = self._stream('left_wrist').fresh_jpeg(self.capture_timeout, self.jpeg_quality, cancelled)
        if cancelled.is_set():
            raise BackendError('扫码采集已停止')
        return dict(self.rgb_store.save_scan(group, index, jpeg), **metadata)

    def save_snapshot(self, camera_id: str, snapshot: CameraSnapshot,
                      robot_state: dict[str, Any]) -> dict[str, Any]:
        self._stream(camera_id)
        return self.store.save({camera_id: snapshot}, robot_state)

    def record(self, camera_id: str, robot_state: dict[str, Any]) -> dict[str, Any]:
        return self.save_snapshot(camera_id, self.fresh_snapshot(camera_id), robot_state)

    def _close_ros(self) -> None:
        if self.executor is not None:
            self.executor.shutdown(timeout_sec=2)
        if self.thread is not None and self.thread is not threading.current_thread():
            self.thread.join(timeout=2)
        if self.node is not None:
            self.node.destroy_node()
            self.node = None
        if self.context is not None and self.context.ok():
            self.context.shutdown()

    def close(self) -> None:
        for stream in self.streams.values():
            stream.close()
        self._close_ros()
        self.recovery.close()
