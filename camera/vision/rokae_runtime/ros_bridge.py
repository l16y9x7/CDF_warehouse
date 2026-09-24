"""ROS RGB-D drivers own hardware; this Owner consumes RGB for existing HTTP push."""
from __future__ import annotations

import logging
import os
import signal
import threading
import time
from collections import deque
from datetime import datetime

import numpy as np

from vision.rokae_runtime.owner import CAMERA_IDS, RokaeCameraOwner
from vision.rokae_runtime.ros_contract import ros_camera_id

LOGGER = logging.getLogger(__name__)


class RosOwnerLiveness:
    """Recreate DDS subscriptions if all enabled inputs remain unavailable."""
    def __init__(self, now, startup_timeout=60.0, stale_timeout=15.0):
        self.started = now
        self.last_ready = None
        self.startup_timeout, self.stale_timeout = startup_timeout, stale_timeout

    def expired(self, now, any_ready):
        if any_ready:
            self.last_ready = now
            return False
        baseline = self.started if self.last_ready is None else self.last_ready
        timeout = self.startup_timeout if self.last_ready is None else self.stale_timeout
        return now - baseline > timeout


def image_to_bgr(msg):
    """Decode a ROS color image, respecting padded rows; never interpret depth as RGB."""
    if msg.encoding not in {"rgb8", "bgr8"}:
        raise ValueError(f"unsupported RGB encoding: {msg.encoding}")
    width, height, step = int(msg.width), int(msg.height), int(msg.step)
    if min(width, height) <= 0 or step < width * 3 or len(msg.data) != step * height:
        raise ValueError("invalid RGB image dimensions/stride/payload")
    pixels = np.frombuffer(bytes(msg.data), dtype=np.uint8).reshape(height, step)
    frame = pixels[:, :width * 3].reshape(height, width, 3)
    if msg.encoding == "rgb8":
        frame = frame[:, :, ::-1]
    return frame.copy()


def image_to_depth_mm(msg):
    """Decode ROS depth Image (16UC1/mono16) to HxW uint16 millimeters."""
    if msg.encoding not in {"16UC1", "mono16"}:
        raise ValueError(f"unsupported depth encoding: {msg.encoding}")
    width, height, step = int(msg.width), int(msg.height), int(msg.step)
    if min(width, height) <= 0 or step < width * 2 or len(msg.data) != step * height:
        raise ValueError("invalid depth image dimensions/stride/payload")
    pixels = np.frombuffer(bytes(msg.data), dtype=np.uint16).reshape(height, step // 2)
    return pixels[:, :width].copy()


class RosImageWorker:
    def __init__(self, max_age=2.0, decode=None):
        self.max_age = max_age
        self._decode = decode or image_to_bgr
        self._lock = threading.Lock()
        self._frame = None
        self._received = 0.0
        self._source_stamp = 0.0
        self._stopped = False
        self.last_error = "WAITING_FOR_ROS_IMAGE"
        self.history = deque(maxlen=6)

    def receive(self, msg):
        try:
            stamp = msg.header.stamp.sec + msg.header.stamp.nanosec / 1e9
            raw_stamp = stamp
            now_wall = time.time()
            # Orbbec USB heads can publish host-skewed stamps; rejecting them leaves
            # Owner forever "timestamp stale or in future" while the driver lives.
            if stamp <= 0 or not -0.25 <= now_wall - stamp <= self.max_age:
                stamp = now_wall
            frame = self._decode(msg)
            with self._lock:
                if self._stopped:
                    return
                # A driver reconnect or boot-time clock correction can move source
                # timestamps backwards. Keep duplicate/out-of-order protection only
                # while the previously accepted stream is fresh; otherwise a valid
                # new stream could be rejected forever. Wall-clock freshness above
                # still prevents replaying old buffered frames.
                if stamp <= self._source_stamp and time.monotonic() - self._received <= self.max_age:
                    return
                self._frame, self._received, self._source_stamp = frame, time.monotonic(), stamp
                self.history.append((raw_stamp, self._received, frame))
                self.last_error = ""
        except Exception as exc:
            with self._lock:
                self._frame = None
                self.last_error = str(exc)

    def get_frame(self, max_stale_sec=2.0):
        with self._lock:
            if self._stopped or self._frame is None:
                return None
            if time.monotonic() - self._received > max_stale_sec:
                return None
            if not -0.25 <= time.time() - self._source_stamp <= max_stale_sec:
                return None
            return self._frame.copy()

    def peek(self, max_stale_sec=2.0):
        """Return stream liveness metadata without copying the full frame."""
        with self._lock:
            if self._stopped or self._frame is None:
                return None
            age = time.monotonic() - self._received
            if age > max_stale_sec:
                return None
            if not -0.25 <= time.time() - self._source_stamp <= max_stale_sec:
                return None
            shape = self._frame.shape
            height, width = int(shape[0]), int(shape[1])
            return {
                "online": True,
                "width": width,
                "height": height,
                "age_sec": float(age),
                "stamp": float(self._source_stamp),
            }

    def ready(self, max_stale_sec=2.0):
        return self.get_frame(max_stale_sec) is not None

    def stop(self):
        with self._lock:
            self._stopped = True
            self._frame = None


class RosCameraOwner(RokaeCameraOwner):
    """Reuse the HTTP contract with one ROS context and no device handles."""
    def __init__(self, config):
        super().__init__(config)
        self._context = self._node = self._executor = self._ros_thread = None
        self._watch_stop = threading.Event()
        self._watch_thread = None
        self._depth_workers = {}
        self._color_info = {}

    def start(self):
        if self._node is not None:
            return
        import rclpy
        from rclpy.context import Context
        from rclpy.executors import MultiThreadedExecutor
        from rclpy.node import Node
        from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
        from rclpy.signals import SignalHandlerOptions
        from sensor_msgs.msg import Image, CameraInfo

        self._context = Context()
        rclpy.init(context=self._context, signal_handler_options=SignalHandlerOptions.NO)
        self._node = Node("rokae_http_owner", context=self._context)
        self._executor = MultiThreadedExecutor(num_threads=2, context=self._context)
        qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                         history=HistoryPolicy.KEEP_LAST, depth=1)
        for camera in CAMERA_IDS:
            cfg = self._cameras_cfg.get(camera) or {}
            enabled = camera in self._cameras_cfg and cfg.get("enabled", True) is not False
            ros_id = ros_camera_id(camera)
            topic = (f"/camera/{ros_id}/synced/color/image_raw" if camera == 'head'
                     else f"/camera/{ros_id}/color/image_raw")
            self._meta[camera] = {"enabled": enabled, "ready": False, "backend": "ros",
                                  "device": topic if enabled else "", "fps": cfg.get("fps", 15)}
            if not enabled:
                continue
            stale = float(cfg.get("frame_stale_sec", 2.0))
            worker = RosImageWorker(max_age=stale)
            self._workers[camera] = {"worker": worker, "stale": stale}
            self._node.create_subscription(Image, topic, worker.receive, qos)
            def receive_info(msg, name=camera):
                k = list(msg.k)
                self._color_info[name] = dict(width=int(msg.width), height=int(msg.height),
                    camera_matrix=[k[:3], k[3:6], k[6:9]], distortion_model=msg.distortion_model,
                    distortion_coefficients=list(msg.d))
            info_topic = f'/camera/{ros_id}/synced/color/camera_info' if camera == 'head' else f'/camera/{ros_id}/color/camera_info'
            self._node.create_subscription(CameraInfo, info_topic, receive_info, qos)
            LOGGER.info("Owner subscribing %s", topic)
            if camera != 'head' or cfg.get("enable_depth", True) is False:
                LOGGER.info("Depth disabled for %s", camera)
                continue
            # Prefer aligned depth; keep native as fallback for USB2 heads without D2C.
            aligned = RosImageWorker(max_age=stale, decode=image_to_depth_mm)
            native = RosImageWorker(max_age=stale, decode=image_to_depth_mm)
            self._depth_workers[camera] = {
                "aligned": aligned, "native": native, "stale": stale,
            }
            aligned_topic = f"/camera/{ros_id}/synced/depth/image_raw"
            native_topic = f"/camera/{ros_id}/depth/image_rect_raw"
            self._node.create_subscription(Image, aligned_topic, aligned.receive, qos)
            self._node.create_subscription(Image, native_topic, native.receive, qos)
            LOGGER.info("Owner subscribing %s and %s", aligned_topic, native_topic)
        self._executor.add_node(self._node)
        self._ros_thread = threading.Thread(target=self._executor.spin, name="rokae-ros-owner", daemon=True)
        self._ros_thread.start()
        self._watch_stop.clear()
        self._watch_thread = threading.Thread(target=self._watch_inputs, name="rokae-ros-watch", daemon=True)
        self._watch_thread.start()

    def get_depth_mm(self, camera_id: str):
        """Return (depth_mm HxW uint16, aligned: bool) or None."""
        from vision.rokae_runtime.capture_api import resolve_camera

        resolved = resolve_camera(camera_id)
        internal = resolved[1] if resolved else str(camera_id or "").strip()
        entry = self._depth_workers.get(internal)
        if entry is None:
            return None
        stale = float(entry["stale"])
        aligned = entry["aligned"].get_frame(max_stale_sec=stale)
        if aligned is not None:
            return aligned, True
        native = entry["native"].get_frame(max_stale_sec=stale)
        if native is not None:
            return native, False
        return None

    def capture(self, *, contract, internal, streams, format=None):
        """Fresh frames from the running ROS drivers; aligned RGB-D stamp match."""
        from .capture_api import new_capture_id, write_capture_dir
        import cv2
        if 'depth' in streams and internal != 'head':
            return dict(ok=False, error_code='DEPTH_UNSUPPORTED', camera=contract)
        color_entry = self._workers.get(internal)
        depth_entry = self._depth_workers.get(internal)
        if not color_entry or ('depth' in streams and not depth_entry):
            return dict(ok=False, error_code='CAMERA_NOT_READY', camera=contract)
        started, deadline = time.monotonic(), time.monotonic()+3.0
        color = depth = None
        color_stamp = depth_stamp = None
        while time.monotonic() < deadline:
            worker = color_entry['worker']
            with worker._lock:
                colors = list(worker.history)
            depths = []
            if 'depth' in streams:
                aligned = depth_entry['aligned']
                with aligned._lock:
                    depths = list(aligned.history)
            if streams == {'color'}:
                fresh = [p for p in colors if p[1] >= started]
                if fresh:
                    color_stamp, _, color = fresh[-1]
                    break
            elif streams == {'depth'}:
                fresh = [p for p in depths if p[1] >= started]
                if fresh:
                    depth_stamp, _, depth = fresh[-1]
                    break
            else:
                pairs = [(c,d) for c in colors for d in depths
                         if min(c[1],d[1]) >= started and min(c[0],d[0]) > 0
                         and c[0] == d[0] and c[2].shape[:2] == d[2].shape]
                if pairs:
                    c,d = min(pairs, key=lambda p: abs(p[0][0]-p[1][0]))
                    color_stamp, _, color = c
                    depth_stamp, _, depth = d
                    break
            time.sleep(.01)
        else:
            return dict(ok=False, error_code='FRESH_FRAME_OR_RGBD_SYNC_TIMEOUT', camera=contract)
        jpeg = None
        if color is not None:
            ok, encoded = cv2.imencode('.jpg', color)
            if not ok:
                return dict(ok=False, error_code='CAPTURE_FAILED', camera=contract)
            jpeg = encoded.tobytes()
        capture_id = new_capture_id()
        try:
            written = write_capture_dir(capture_id=capture_id, color_jpeg=jpeg,
                depth_mm=depth, depth_format=format if depth is not None else None,
                depth_aligned=True if depth is not None else None)
        except Exception as exc:
            return dict(ok=False, error_code='CAPTURE_FAILED', camera=contract, message=str(exc))
        return dict(ok=True, camera=contract, capture_id=capture_id, same_shot=True,
                    color_intrinsics=self._color_info.get(internal),
                    captured_at=datetime.now().astimezone().isoformat(),
                    timestamps=dict(color_s=color_stamp, depth_s=depth_stamp, pairing='exact_ros_stamp'),
                    color=written['color'], depth=written['depth'])

    def _watch_inputs(self):
        live = RosOwnerLiveness(time.monotonic())
        while not self._watch_stop.wait(1.0):
            if not self._workers:
                continue
            ready = any(self.camera_ready(camera) for camera in self._workers)
            if live.expired(time.monotonic(), ready):
                LOGGER.error("All ROS Owner inputs stalled; exiting for service restart and DDS rediscovery")
                os.kill(os.getpid(), signal.SIGTERM)
                return

    def listing(self):
        payload = super().listing()
        by_legacy = {row["legacy_id"]: row for row in payload["cameras"]}
        for camera, entry in self._workers.items():
            row = by_legacy.get(camera)
            if row is None:
                continue
            self._meta[camera]["error"] = ("" if self.camera_ready(camera) else
                                           entry["worker"].last_error or "ROS_IMAGE_STALE")
            row["error"] = self._meta[camera]["error"]
            depth_entry = self._depth_workers.get(camera)
            if depth_entry is None:
                continue
            stale = float(depth_entry["stale"])
            aligned = depth_entry["aligned"].peek(max_stale_sec=stale)
            if aligned is not None:
                row["depth"] = {
                    "online": True,
                    "aligned": True,
                    "width": aligned["width"],
                    "height": aligned["height"],
                    "age_sec": aligned["age_sec"],
                    "stamp": aligned["stamp"],
                }
                continue
            native = depth_entry["native"].peek(max_stale_sec=stale)
            if native is not None:
                row["depth"] = {
                    "online": True,
                    "aligned": False,
                    "width": native["width"],
                    "height": native["height"],
                    "age_sec": native["age_sec"],
                    "stamp": native["stamp"],
                }
        return payload

    def stop(self):
        self._watch_stop.set()
        if self._watch_thread is not None and self._watch_thread is not threading.current_thread():
            self._watch_thread.join(timeout=2.0)
        for entry in self._workers.values():
            entry["worker"].stop()
        for entry in self._depth_workers.values():
            entry["aligned"].stop()
            entry["native"].stop()
        if self._executor is not None:
            self._executor.shutdown(timeout_sec=3.0)
        if self._ros_thread is not None:
            self._ros_thread.join(timeout=3.0)
        if self._node is not None:
            self._node.destroy_node()
        if self._context is not None and self._context.ok():
            self._context.shutdown()
        self._node = self._context = self._executor = self._ros_thread = None
        self._workers.clear()
        self._depth_workers.clear()
