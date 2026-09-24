"""HTTP gateway that exposes robot RGB-D camera streams as MJPEG / JPEG."""

from __future__ import annotations

import json
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from typing import Optional
from urllib.parse import urlsplit

import rclpy
from ament_index_python.packages import get_package_share_directory
from rcl_interfaces.srv import GetParameters
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import parameter_value_to_python
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image

from vision.tianji_runtime.camera_gateway_core import (
    STREAM_TYPES,
    CameraGatewayConfig,
    HeadResolutionProfile,
    LatestFrameStore,
    LatestRgbdPairStore,
    build_camera_list_payload,
    build_rgbd_archive,
    encode_sensor_image_to_jpeg,
    evaluate_camera_health,
    load_camera_gateway_config,
    parse_camera_query,
    parse_head_resolution_payload,
)
from vision.tianji_runtime.http_gateway_core import HttpResult
from vision.tianji_runtime.head_camera_supervisor import (
    HeadCameraProcessSupervisor,
)


_HEAD_READY_720 = "READY_720_RGBD"
_HEAD_READY_1080 = "READY_1080_RGB"
_HEAD_SWITCHING = "SWITCHING"
_HEAD_DEGRADED = "DEGRADED"


class _HeadParameterServiceUnavailable(TimeoutError):
    pass


class _HeadParameterCallTimeout(TimeoutError):
    pass


class _HeadParameterSetTimeout(_HeadParameterCallTimeout):
    pass


class _HeadParameterReadbackTimeout(_HeadParameterCallTimeout):
    pass


class _HeadProfileMismatch(RuntimeError):
    pass


def _default_config_path(filename: str) -> str:
    source_candidate = Path(__file__).resolve().parents[1] / "config" / filename
    if source_candidate.exists():
        return str(source_candidate)
    return str(
        Path(get_package_share_directory("retail_nav_bridge"))
        / "config"
        / filename
    )


class _CameraHttpServer(ThreadingHTTPServer):
    daemon_threads = True

    def __init__(self, server_address, handler_class, gateway) -> None:
        super().__init__(server_address, handler_class)
        self.gateway = gateway


class _CameraRequestHandler(BaseHTTPRequestHandler):
    protocol_version = "HTTP/1.1"
    server_version = "TianJiCameraGateway/1.0"

    @property
    def gateway(self):
        return self.server.gateway  # type: ignore[attr-defined]

    def _write_json(self, result: HttpResult) -> None:
        payload = json.dumps(
            result.body, ensure_ascii=False, separators=(",", ":")
        ).encode("utf-8")
        self.send_response(result.status_code)
        self.send_header("Content-Type", "application/json; charset=utf-8")
        self.send_header("Content-Length", str(len(payload)))
        self.send_header("Connection", "close")
        self.end_headers()
        self.wfile.write(payload)

    def _write_bytes(
        self,
        status_code: int,
        body: bytes,
        content_type: str,
        extra_headers: Optional[dict[str, str]] = None,
    ) -> None:
        self.send_response(status_code)
        self.send_header("Content-Type", content_type)
        self.send_header("Content-Length", str(len(body)))
        self.send_header("Cache-Control", "no-store")
        self.send_header("Connection", "close")
        for name, value in (extra_headers or {}).items():
            self.send_header(name, value)
        self.end_headers()
        self.wfile.write(body)

    def do_GET(self) -> None:
        path = urlsplit(self.path).path
        if path == "/camera/head/resolution":
            self._write_json(HttpResult(200, self.gateway.head_resolution_status()))
            return
        if path == "/camera/health":
            self._write_json(
                HttpResult(200, {"status": self.gateway.health_status()})
            )
            return
        if path == "/camera/list":
            self._write_json(HttpResult(200, self.gateway.list_cameras()))
            return
        if path == "/camera/rgbd":
            self._handle_rgbd()
            return
        if path == "/camera/snapshot":
            self._handle_snapshot()
            return
        if path == "/camera/stream":
            self._handle_stream()
            return
        self._write_json(HttpResult(404, {"error_code": "NOT_FOUND"}))

    def do_POST(self) -> None:
        path = urlsplit(self.path).path
        if path != "/camera/head/resolution":
            self._write_json(HttpResult(404, {"error_code": "NOT_FOUND"}))
            return
        try:
            content_length = int(self.headers.get("Content-Length", "0"))
        except ValueError:
            content_length = -1
        if content_length <= 0 or content_length > 4096:
            self._write_json(
                HttpResult(
                    400,
                    {
                        "error_code": "INVALID_REQUEST",
                        "message": "a small JSON request body is required",
                    },
                )
            )
            return
        try:
            payload = json.loads(self.rfile.read(content_length))
            resolution = parse_head_resolution_payload(payload)
        except (json.JSONDecodeError, UnicodeDecodeError, ValueError) as exc:
            self._write_json(
                HttpResult(
                    400,
                    {
                        "error_code": "INVALID_REQUEST",
                        "message": str(exc),
                    },
                )
            )
            return
        self._write_json(self.gateway.switch_head_resolution(resolution))

    def _handle_rgbd(self) -> None:
        query = parse_camera_query(self.path)
        result = self.gateway.get_rgbd_archive(query["camera"])
        if isinstance(result, HttpResult):
            self._write_json(result)
            return
        body, camera_id, stamp_sec, stamp_nanosec = result
        filename = f"{camera_id}_{stamp_sec}_{stamp_nanosec:09d}.zip"
        self._write_bytes(
            200,
            body,
            "application/zip",
            {
                "Content-Disposition": f'attachment; filename="{filename}"',
                "X-Camera-Id": camera_id,
                "X-ROS-Stamp-Sec": str(stamp_sec),
                "X-ROS-Stamp-Nanosec": str(stamp_nanosec),
            },
        )

    def _handle_snapshot(self) -> None:
        query = parse_camera_query(self.path)
        response_format = query["format"]
        if response_format not in ("", "raw", "preview", "jpeg"):
            self._write_json(HttpResult(400, {"error_code": "INVALID_REQUEST"}))
            return
        if query["type"] != "depth" and response_format == "raw":
            self._write_json(HttpResult(400, {"error_code": "INVALID_REQUEST"}))
            return

        result = self.gateway.get_snapshot_frame(query["camera"], query["type"])
        if isinstance(result, HttpResult):
            self._write_json(result)
            return

        if query["type"] == "depth" and response_format not in ("preview", "jpeg"):
            if result.raw is None:
                self._write_json(HttpResult(503, {"error_code": "STREAM_NOT_READY"}))
                return
            self._write_bytes(
                200,
                result.raw,
                "application/octet-stream",
                self._raw_image_headers(result),
            )
            return
        self._write_bytes(
            200,
            self.gateway.frame_jpeg(result, query["type"]),
            "image/jpeg",
            self._image_headers(result),
        )

    def _handle_stream(self) -> None:
        query = parse_camera_query(self.path)
        response_format = query["format"]
        if response_format not in ("", "raw", "preview", "jpeg"):
            self._write_json(HttpResult(400, {"error_code": "INVALID_REQUEST"}))
            return
        if query["type"] != "depth" and response_format == "raw":
            self._write_json(HttpResult(400, {"error_code": "INVALID_REQUEST"}))
            return

        stream = self.gateway.resolve_stream(query["camera"], query["type"])
        if isinstance(stream, HttpResult):
            self._write_json(stream)
            return

        access = self.gateway.stream_access(stream)
        if isinstance(access, HttpResult):
            self._write_json(access)
            return
        stream_generation = access

        raw_depth = stream.stream_type == "depth" and response_format not in (
            "preview",
            "jpeg",
        )
        boundary = "tianjiframe"
        self.send_response(200)
        self.send_header(
            "Content-Type",
            f"multipart/x-mixed-replace; boundary={boundary}",
        )
        self.send_header("Cache-Control", "no-store")
        self.send_header("Connection", "close")
        self.end_headers()

        interval = 1.0 / max(1.0, self.gateway.stream_fps)
        last_received_at: Optional[float] = None
        try:
            while self.gateway.stream_session_active(
                stream,
                stream_generation,
            ):
                entry = self.gateway.get_frame(stream.key)
                if entry is not None and entry.received_at != last_received_at:
                    last_received_at = entry.received_at
                    body = (
                        entry.raw
                        if raw_depth
                        else self.gateway.frame_jpeg(
                            entry,
                            stream.stream_type,
                        )
                    )
                    if body is None:
                        time.sleep(interval)
                        continue
                    # Encoding can take long enough for a profile transition to
                    # invalidate this session.  Re-check immediately before the
                    # socket write so an old head-camera generation is not sent
                    # after the switch has begun.
                    if not self.gateway.stream_session_active(
                        stream,
                        stream_generation,
                    ):
                        return
                    content_type = (
                        "application/octet-stream" if raw_depth else "image/jpeg"
                    )
                    part_headers = (
                        self._raw_image_headers(entry)
                        if raw_depth
                        else self._image_headers(entry)
                    )
                    header = (
                        f"--{boundary}\r\n"
                        f"Content-Type: {content_type}\r\n"
                        f"Content-Length: {len(body)}\r\n"
                        + "".join(
                            f"{name}: {value}\r\n"
                            for name, value in part_headers.items()
                        )
                        + "\r\n"
                    ).encode("ascii")
                    self.wfile.write(header)
                    self.wfile.write(body)
                    self.wfile.write(b"\r\n")
                    self.wfile.flush()
                time.sleep(interval)
        except (BrokenPipeError, ConnectionResetError):
            return

    @staticmethod
    def _raw_image_headers(entry) -> dict[str, str]:
        headers = _CameraRequestHandler._image_headers(entry)
        headers.update(
            {
                "X-Image-Step": str(entry.step or entry.width * 2),
                "X-Image-Is-Bigendian": str(entry.is_bigendian),
            }
        )
        return headers

    @staticmethod
    def _image_headers(entry) -> dict[str, str]:
        return {
            "X-Image-Width": str(entry.width),
            "X-Image-Height": str(entry.height),
            "X-Image-Encoding": entry.encoding,
            "X-Camera-Generation": str(entry.generation),
            "X-ROS-Stamp-Sec": str(entry.stamp_sec),
            "X-ROS-Stamp-Nanosec": str(entry.stamp_nanosec),
        }

    def log_message(self, format_string: str, *args) -> None:
        self.gateway.get_logger().debug(format_string % args)


class CameraHttpGateway(Node):
    def __init__(self) -> None:
        super().__init__("retail_camera_http_gateway")
        self.declare_parameter("http_host", "0.0.0.0")
        self.declare_parameter("http_port", 8085)
        self.declare_parameter(
            "camera_config_file",
            _default_config_path("camera_streams.yaml"),
        )

        config_file = str(self.get_parameter("camera_config_file").value)
        self._config: CameraGatewayConfig = load_camera_gateway_config(config_file)
        self._frame_store = LatestFrameStore()
        self._rgbd_store = LatestRgbdPairStore()
        self._running = True
        self._subscriptions = []
        self._head_switch_config = self._config.head_resolution_switch
        self._head_switch_lock = threading.Lock()
        self._head_state_lock = threading.Lock()
        self._head_ready_event = threading.Event()
        self._head_generation = 0
        self._head_state = _HEAD_READY_720
        self._head_resolution = 720
        self._head_target_resolution: Optional[int] = None
        self._head_readiness_started_at = float("inf")
        self._head_valid_color_frames = 0
        self._head_valid_rgbd_pairs = 0
        self._head_last_color_stamp: Optional[tuple[int, int]] = None
        self._head_last_pair_stamp: Optional[tuple[int, int]] = None
        self._head_last_error: Optional[str] = None
        self._head_last_switch_elapsed_ms: Optional[int] = None
        self._head_last_ready_stamp: Optional[tuple[int, int]] = None
        self._head_profile_verified_resolution: Optional[int] = None
        self._head_recovery_required = False
        self._head_recovery_thread = None
        self._head_get_parameters_client = None
        self._head_process_supervisor = None
        self._head_parameter_callback_group = MutuallyExclusiveCallbackGroup()
        if self._head_switch_config is not None and self._head_switch_config.enabled:
            remote = self._head_switch_config.node_name.rstrip("/")
            self._head_get_parameters_client = self.create_client(
                GetParameters,
                f"{remote}/get_parameters",
                callback_group=self._head_parameter_callback_group,
            )
            self._head_process_supervisor = HeadCameraProcessSupervisor(
                profile_configs=(
                    self._head_switch_config.process_profile_configs
                ),
                serial_no=self._head_switch_config.serial_no,
                runtime_dir=self._head_switch_config.process_runtime_dir,
                stop_timeout_sec=(
                    self._head_switch_config.process_stop_timeout_sec
                ),
                startup_probe_sec=(
                    self._head_switch_config.process_startup_probe_sec
                ),
            )

        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        for stream in self._config.streams:
            subscription = self.create_subscription(
                Image,
                stream.topic,
                self._make_image_callback(stream),
                qos,
            )
            self._subscriptions.append(subscription)
            self.get_logger().info(
                f"subscribed camera stream {stream.key} -> {stream.topic}"
            )

        host = str(self.get_parameter("http_host").value)
        port = int(self.get_parameter("http_port").value)
        self._http_server = _CameraHttpServer(
            (host, port),
            _CameraRequestHandler,
            self,
        )
        self._http_thread = threading.Thread(
            target=self._http_server.serve_forever,
            name="retail-camera-http",
            daemon=True,
        )
        self._http_thread.start()
        self.get_logger().info(
            f"camera HTTP gateway listening on {host}:{port}"
        )
        self._startup_profile_timer = None
        self._startup_profile_thread = None
        if self._head_switch_config is not None and self._head_switch_config.enabled:
            self._startup_profile_timer = self.create_timer(
                1.0,
                self._start_head_profile_initialization,
            )

    @property
    def stream_fps(self) -> float:
        return self._config.stream_fps

    def is_running(self) -> bool:
        return self._running

    def _start_head_profile_initialization(self) -> None:
        timer = self._startup_profile_timer
        if timer is None:
            return
        timer.cancel()
        self.destroy_timer(timer)
        self._startup_profile_timer = None

        self._startup_profile_thread = threading.Thread(
            target=self._initialize_standard_head_profile,
            name="head-profile-initialize",
            daemon=True,
        )
        self._startup_profile_thread.start()

    def _initialize_standard_head_profile(self) -> None:
        switch = self._head_switch_config
        deadline = time.monotonic() + switch.startup_timeout_sec
        attempt = 0
        last_result = None
        while self._running and time.monotonic() < deadline:
            with self._head_state_lock:
                state = self._head_state
                target = self._head_target_resolution
            if state == _HEAD_READY_1080 or (
                state == _HEAD_SWITCHING and target == 1080
            ):
                self.get_logger().info(
                    "startup 720 initialization yielded to a user 1080 switch"
                )
                return
            attempt += 1
            last_result = self.switch_head_resolution(
                720,
                startup_only=True,
            )
            if last_result.status_code == 200:
                return
            if bool(last_result.body.get("startup_yielded")):
                return
            if not bool(last_result.body.get("retryable")):
                self.get_logger().error(
                    "startup 720 initialization failed without a safe retry: "
                    f"{last_result.body}"
                )
                return
            retry_after = max(
                float(last_result.body.get("retry_after_sec", 1.0)),
                min(10.0, float(attempt)),
            )
            time.sleep(max(0.1, retry_after))
        self.get_logger().error(
            "failed to establish the standard 720 RGB-D profile within "
            f"{switch.startup_timeout_sec:.1f}s: "
            f"{None if last_result is None else last_result.body}"
        )

    def _make_image_callback(self, stream):
        def _callback(message: Image) -> None:
            try:
                generation = self._stream_generation(stream.camera_id)
                message_data = bytes(message.data)
                entry = self._frame_store.update(
                    stream.key,
                    None,
                    width=int(message.width),
                    height=int(message.height),
                    encoding=str(message.encoding),
                    raw=message_data,
                    step=int(message.step),
                    is_bigendian=int(message.is_bigendian),
                    stamp_sec=int(message.header.stamp.sec),
                    stamp_nanosec=int(message.header.stamp.nanosec),
                    frame_id=str(message.header.frame_id),
                    generation=generation,
                )
                pair = self._rgbd_store.update(
                    stream.camera_id,
                    stream.stream_type,
                    entry,
                )
                self._observe_head_readiness(stream, entry, pair)
            except Exception as exc:
                self.get_logger().warning(
                    f"failed to cache {stream.key}: {exc}"
                )
                return

        return _callback

    def _stream_generation(self, camera_id: str) -> int:
        switch = self._head_switch_config
        if switch is None or camera_id != switch.camera_id:
            return 0
        with self._head_state_lock:
            return self._head_generation

    def _observe_head_readiness(self, stream, entry, pair) -> None:
        switch = self._head_switch_config
        if switch is None or stream.camera_id != switch.camera_id:
            return
        with self._head_state_lock:
            if self._head_state != _HEAD_SWITCHING:
                return
            target = self._head_target_resolution
            if target is None or entry.generation != self._head_generation:
                return
            if entry.received_at < self._head_readiness_started_at:
                return
            profile = switch.profile(target)
            if target == 1080:
                if stream.stream_type != "color":
                    return
                if not self._valid_color_entry(entry, profile):
                    self._head_valid_color_frames = 0
                    self._head_last_color_stamp = None
                    return
                if not self._stamp_after(entry.stamp, self._head_last_color_stamp):
                    return
                self._head_last_color_stamp = entry.stamp
                self._head_valid_color_frames += 1
                if self._head_valid_color_frames >= profile.settle_frames:
                    self._head_last_ready_stamp = entry.stamp
                    self._head_ready_event.set()
                return

            if pair is None or pair.generation != self._head_generation:
                return
            if min(pair.color.received_at, pair.depth.received_at) < (
                self._head_readiness_started_at
            ):
                return
            if not self._valid_rgbd_pair(pair, profile):
                self._head_valid_rgbd_pairs = 0
                self._head_last_pair_stamp = None
                return
            if not self._stamp_after(pair.color.stamp, self._head_last_pair_stamp):
                return
            self._head_last_pair_stamp = pair.color.stamp
            self._head_valid_rgbd_pairs += 1
            if self._head_valid_rgbd_pairs >= profile.settle_frames:
                self._head_last_ready_stamp = pair.color.stamp
                self._head_ready_event.set()

    @staticmethod
    def _stamp_after(
        stamp: tuple[int, int],
        previous: Optional[tuple[int, int]],
    ) -> bool:
        return previous is None or stamp > previous

    @staticmethod
    def _valid_color_entry(entry, profile: HeadResolutionProfile) -> bool:
        return (
            (entry.width, entry.height) == profile.color_size
            and entry.encoding.lower() in ("rgb8", "bgr8")
            and entry.raw is not None
            and int(entry.step or 0) >= entry.width * 3
            and len(entry.raw) >= int(entry.step or 0) * entry.height
        )

    @classmethod
    def _valid_rgbd_pair(cls, pair, profile: HeadResolutionProfile) -> bool:
        return (
            cls._valid_color_entry(pair.color, profile)
            and profile.depth_size is not None
            and (pair.depth.width, pair.depth.height) == profile.depth_size
            and pair.depth.encoding.lower() in ("16uc1", "mono16")
            and pair.depth.raw is not None
            and int(pair.depth.step or 0) >= pair.depth.width * 2
            and len(pair.depth.raw)
            >= int(pair.depth.step or 0) * pair.depth.height
            and pair.color.stamp == pair.depth.stamp
        )

    def head_resolution_status(self) -> dict:
        switch = self._head_switch_config
        if switch is None or not switch.enabled:
            return {
                "camera": "head",
                "supported": False,
                "state": "UNAVAILABLE",
            }
        with self._head_state_lock:
            target = self._head_target_resolution
            resolution = self._head_resolution
            state = self._head_state
            generation = self._head_generation
            last_error = self._head_last_error
            elapsed_ms = self._head_last_switch_elapsed_ms
            ready_stamp = self._head_last_ready_stamp
            color_frames = self._head_valid_color_frames
            rgbd_pairs = self._head_valid_rgbd_pairs
            verified_resolution = self._head_profile_verified_resolution
            recovery_required = self._head_recovery_required
        active_resolution = target if state == _HEAD_SWITCHING else resolution
        profile = switch.profile(active_resolution)
        data_fresh = None
        reported_state = state
        if state in (_HEAD_READY_720, _HEAD_READY_1080):
            data_fresh = self._head_profile_data_fresh(profile, generation)
            if verified_resolution != resolution:
                reported_state = (
                    "UNVERIFIED_1080_RGB"
                    if state == _HEAD_READY_1080
                    else "UNVERIFIED_720_RGBD"
                )
            elif not data_fresh:
                reported_state = (
                    "STALE_1080_RGB"
                    if state == _HEAD_READY_1080
                    else "STALE_720_RGBD"
                )
        width, height = profile.color_size
        payload = {
            "camera": switch.camera_id,
            "supported": True,
            "state": reported_state,
            "configured_state": state,
            "profile_verified": verified_resolution == resolution,
            "recovery_to_720_required": recovery_required,
            "data_fresh": data_fresh,
            "resolution": resolution,
            "target_resolution": target,
            "generation": generation,
            "color_profile": profile.color_profile,
            "width": width,
            "height": height,
            "fps": profile.color_fps,
            "rgb_only": not profile.enable_depth,
            "depth_enabled": profile.enable_depth,
            "aligned_depth_enabled": profile.align_depth,
            "settle_frames_required": profile.settle_frames,
            "settled_color_frames": color_frames,
            "settled_rgbd_pairs": rgbd_pairs,
            "last_switch_elapsed_ms": elapsed_ms,
            "last_error": last_error,
            "snapshot_url": (
                f"/camera/snapshot?camera={switch.camera_id}&type=color"
            ),
        }
        if ready_stamp is not None:
            payload["ready_stamp_sec"] = ready_stamp[0]
            payload["ready_stamp_nanosec"] = ready_stamp[1]
        return payload

    def stream_access(self, stream):
        switch = self._head_switch_config
        if (
            switch is None
            or not switch.enabled
            or stream.camera_id != switch.camera_id
        ):
            return 0
        with self._head_state_lock:
            state = self._head_state
            generation = self._head_generation
            resolution = self._head_resolution
            verified_resolution = self._head_profile_verified_resolution
        if state == _HEAD_SWITCHING:
            return HttpResult(
                423,
                {
                    "error_code": "HEAD_PROFILE_SWITCHING",
                    "message": "head camera profile is switching",
                },
            )
        if state == _HEAD_DEGRADED:
            return HttpResult(
                503,
                {
                    "error_code": "HEAD_CAMERA_DEGRADED",
                    "message": "switch back to resolution 720 to recover",
                },
            )
        if state == _HEAD_READY_1080 and stream.stream_type == "depth":
            return HttpResult(
                409,
                {
                    "error_code": "HEAD_DEPTH_DISABLED_IN_1080_MODE",
                    "message": "switch head resolution to 720 before using depth",
                },
            )
        if state in (_HEAD_READY_720, _HEAD_READY_1080):
            if verified_resolution != resolution:
                return HttpResult(
                    503,
                    {
                        "error_code": "HEAD_PROFILE_UNVERIFIED",
                        "message": "head camera profile has not been verified yet",
                    },
                )
            profile = switch.profile(resolution)
            if not self._head_profile_data_fresh(profile, generation):
                return HttpResult(
                    503,
                    {
                        "error_code": "HEAD_PROFILE_DATA_STALE",
                        "message": (
                            "head RGB-D is not ready"
                            if state == _HEAD_READY_720
                            else "head 1080 RGB is not ready"
                        ),
                    },
                )
        return generation

    def stream_session_active(self, stream, generation: int) -> bool:
        if not self._running:
            return False
        access = self.stream_access(stream)
        return isinstance(access, int) and access == generation

    def switch_head_resolution(
        self,
        resolution: int,
        *,
        startup_only: bool = False,
        recovery_only: bool = False,
    ) -> HttpResult:
        switch = self._head_switch_config
        if switch is None or not switch.enabled:
            return HttpResult(
                501,
                {
                    "error_code": "HEAD_RESOLUTION_SWITCH_UNAVAILABLE",
                },
            )
        try:
            profile = switch.profile(resolution)
        except ValueError as exc:
            return HttpResult(
                400,
                {"error_code": "INVALID_REQUEST", "message": str(exc)},
            )
        if not self._head_switch_lock.acquire(blocking=False):
            return HttpResult(
                423,
                {
                    "error_code": "HEAD_PROFILE_BUSY",
                    "message": "another head camera profile switch is running",
                    "retryable": True,
                    "retry_after_sec": 0.5,
                },
            )

        started_at = time.monotonic()
        try:
            if startup_only:
                with self._head_state_lock:
                    state = self._head_state
                    target = self._head_target_resolution
                if state == _HEAD_READY_1080 or (
                    state == _HEAD_SWITCHING and target == 1080
                ):
                    return HttpResult(
                        409,
                        {
                            "error_code": "STARTUP_PROFILE_YIELDED",
                            "startup_yielded": True,
                            "retryable": False,
                        },
                    )
            if recovery_only:
                with self._head_state_lock:
                    recovery_required = self._head_recovery_required
                if not recovery_required:
                    return HttpResult(
                        200,
                        {
                            "state": self.head_resolution_status(),
                            "recovery_yielded": True,
                        },
                    )
            with self._head_state_lock:
                recovery_required = self._head_recovery_required
            if recovery_required and resolution != 720:
                return HttpResult(
                    423,
                    {
                        "error_code": "HEAD_720_RECOVERY_REQUIRED",
                        "message": "automatic recovery to 720 is still required",
                        "retryable": True,
                        "retry_after_sec": 1.0,
                    },
                )
            if self._head_profile_already_ready(profile):
                payload = self.head_resolution_status()
                payload["changed"] = False
                return HttpResult(200, payload)
            try:
                if startup_only and profile.resolution == 720:
                    try:
                        self._verify_head_profile_parameters(profile)
                    except _HeadProfileMismatch:
                        pass
                    else:
                        self._prepare_head_transition(profile.resolution)
                        self._start_head_readiness_window()
                        if not self._head_ready_event.wait(
                            timeout=switch.readiness_timeout_sec
                        ):
                            raise TimeoutError(
                                "existing 720 RGB-D stream did not settle"
                            )
                        self._verify_head_profile(profile)
                        elapsed_ms = round(
                            (time.monotonic() - started_at) * 1000
                        )
                        self._finish_head_transition(720, elapsed_ms)
                        payload = self.head_resolution_status()
                        payload["changed"] = False
                        payload["verified_existing_profile"] = True
                        return HttpResult(200, payload)
                self._prepare_head_transition(profile.resolution)
                self._apply_head_profile(profile)
                self._start_head_readiness_window()
                if not self._head_ready_event.wait(
                    timeout=switch.readiness_timeout_sec
                ):
                    raise TimeoutError(
                        f"no reliable {profile.resolution} frame within "
                        f"{switch.readiness_timeout_sec:.1f}s"
                    )
                self._verify_head_profile(profile)
                elapsed_ms = round((time.monotonic() - started_at) * 1000)
                self._finish_head_transition(profile.resolution, elapsed_ms)
                payload = self.head_resolution_status()
                payload["changed"] = True
                return HttpResult(200, payload)
            except Exception as exc:
                self.get_logger().error(
                    f"head resolution switch to {resolution} failed: {exc}"
                )
                recovered = False
                recovery_error = None
                if resolution == 1080:
                    try:
                        recovery = switch.profile(720)
                        self._prepare_head_transition(720)
                        self._apply_head_profile(recovery)
                        self._start_head_readiness_window()
                        if not self._head_ready_event.wait(
                            timeout=switch.readiness_timeout_sec
                        ):
                            raise TimeoutError("720 RGB-D recovery timed out")
                        self._verify_head_profile(recovery)
                        elapsed_ms = round(
                            (time.monotonic() - started_at) * 1000
                        )
                        self._finish_head_transition(720, elapsed_ms)
                        recovered = True
                    except Exception as rollback_exc:
                        recovery_error = str(rollback_exc)
                if not recovered:
                    self._mark_head_degraded(
                        str(exc),
                        recovery_error=recovery_error,
                    )
                    if resolution == 1080:
                        self._schedule_standard_head_recovery()
                return HttpResult(
                    503,
                    {
                        "error_code": "HEAD_PROFILE_SWITCH_FAILED",
                        "message": str(exc),
                        "requested_resolution": resolution,
                        "recovered_to_720": recovered,
                        "recovery_error": recovery_error,
                        "retryable": startup_only
                        or recovery_only
                        or isinstance(
                            exc,
                            (
                                _HeadParameterServiceUnavailable,
                                _HeadParameterCallTimeout,
                            ),
                        ),
                        "retry_after_sec": 1.0,
                        "state": self.head_resolution_status(),
                    },
                )
        finally:
            self._head_switch_lock.release()

    def _head_profile_already_ready(
        self,
        profile: HeadResolutionProfile,
    ) -> bool:
        expected_state = (
            _HEAD_READY_1080 if profile.resolution == 1080 else _HEAD_READY_720
        )
        with self._head_state_lock:
            if (
                self._head_state != expected_state
                or self._head_resolution != profile.resolution
                or self._head_profile_verified_resolution
                != profile.resolution
            ):
                return False
            generation = self._head_generation
        supervisor = self._head_process_supervisor
        if supervisor is None:
            return False
        process_status = supervisor.status()
        if (
            not bool(process_status.get("running"))
            or process_status.get("resolution") != profile.resolution
        ):
            return False
        return self._head_profile_data_fresh(profile, generation)

    def _head_profile_data_fresh(
        self,
        profile: HeadResolutionProfile,
        generation: int,
    ) -> bool:
        if profile.resolution == 1080:
            stream = self._config.find_stream(
                self._head_switch_config.camera_id,
                "color",
            )
            entry = None if stream is None else self._frame_store.get(stream.key)
            return (
                entry is not None
                and entry.generation == generation
                and self._valid_color_entry(entry, profile)
                and time.monotonic() - entry.received_at
                <= self._config.stale_after_sec
            )
        pair = self._rgbd_store.get(self._head_switch_config.camera_id)
        return (
            pair is not None
            and pair.generation == generation
            and self._valid_rgbd_pair(pair, profile)
            and time.monotonic()
            - min(pair.color.received_at, pair.depth.received_at)
            <= self._config.stale_after_sec
        )

    def _prepare_head_transition(self, resolution: int) -> None:
        switch = self._head_switch_config
        with self._head_state_lock:
            self._head_generation += 1
            self._head_state = _HEAD_SWITCHING
            self._head_target_resolution = resolution
            self._head_readiness_started_at = float("inf")
            self._head_valid_color_frames = 0
            self._head_valid_rgbd_pairs = 0
            self._head_last_color_stamp = None
            self._head_last_pair_stamp = None
            self._head_last_ready_stamp = None
            self._head_last_error = None
            self._head_profile_verified_resolution = None
            self._head_ready_event.clear()
        self._frame_store.clear_camera(switch.camera_id)
        self._rgbd_store.clear_camera(switch.camera_id)

    def _start_head_readiness_window(self) -> None:
        switch = self._head_switch_config
        self._frame_store.clear_camera(switch.camera_id)
        self._rgbd_store.clear_camera(switch.camera_id)
        with self._head_state_lock:
            self._head_readiness_started_at = time.monotonic()
            self._head_valid_color_frames = 0
            self._head_valid_rgbd_pairs = 0
            self._head_last_color_stamp = None
            self._head_last_pair_stamp = None
            self._head_last_ready_stamp = None
            self._head_ready_event.clear()

    def _finish_head_transition(self, resolution: int, elapsed_ms: int) -> None:
        with self._head_state_lock:
            self._head_resolution = resolution
            self._head_target_resolution = None
            self._head_state = (
                _HEAD_READY_1080 if resolution == 1080 else _HEAD_READY_720
            )
            self._head_profile_verified_resolution = resolution
            if resolution == 720:
                self._head_recovery_required = False
            self._head_last_error = None
            self._head_last_switch_elapsed_ms = elapsed_ms

    def _mark_head_degraded(
        self,
        error: str,
        *,
        recovery_error: Optional[str],
    ) -> None:
        switch = self._head_switch_config
        self._frame_store.clear_camera(switch.camera_id)
        self._rgbd_store.clear_camera(switch.camera_id)
        message = error
        if recovery_error:
            message = f"{error}; recovery failed: {recovery_error}"
        with self._head_state_lock:
            self._head_state = _HEAD_DEGRADED
            self._head_target_resolution = None
            self._head_profile_verified_resolution = None
            self._head_last_error = message
            self._head_last_switch_elapsed_ms = None

    def _apply_head_profile(self, profile: HeadResolutionProfile) -> None:
        supervisor = self._head_process_supervisor
        if supervisor is None:
            raise RuntimeError("head camera process supervisor is not configured")
        result = supervisor.apply(
            profile.resolution,
            force_restart=True,
        )
        if not bool(result.get("running")):
            raise RuntimeError(
                f"head {profile.resolution} camera process did not start"
            )

    def _get_remote_parameters(self, names: list[str]) -> dict[str, object]:
        client = self._head_get_parameters_client
        switch = self._head_switch_config
        if client is None:
            raise RuntimeError("head parameter client is not configured")
        if not client.wait_for_service(timeout_sec=switch.parameter_timeout_sec):
            raise _HeadParameterServiceUnavailable(
                "head get_parameters service unavailable"
            )
        request = GetParameters.Request()
        request.names = names
        try:
            response = self._wait_future(
                client.call_async(request),
                switch.parameter_timeout_sec,
                "get head parameters",
            )
        except _HeadParameterCallTimeout as exc:
            raise _HeadParameterReadbackTimeout(str(exc)) from exc
        if response is None or len(response.values) != len(names):
            raise RuntimeError("invalid get_parameters response")
        return {
            name: parameter_value_to_python(value)
            for name, value in zip(names, response.values)
        }

    @staticmethod
    def _wait_future(
        future,
        timeout_sec: float,
        action: str,
        *,
        cancel_on_timeout: bool = True,
    ):
        deadline = time.monotonic() + timeout_sec
        while not future.done():
            if time.monotonic() >= deadline:
                if cancel_on_timeout:
                    future.cancel()
                raise _HeadParameterCallTimeout(f"{action} timed out")
            time.sleep(0.01)
        exception = future.exception()
        if exception is not None:
            raise RuntimeError(f"{action} failed: {exception}") from exception
        return future.result()

    def _schedule_standard_head_recovery(self) -> None:
        with self._head_state_lock:
            self._head_recovery_required = True
            existing = self._head_recovery_thread
            if existing is not None and existing.is_alive():
                return
            thread = threading.Thread(
                target=self._recover_standard_head_profile,
                name="head-profile-recover-720",
                daemon=True,
            )
            self._head_recovery_thread = thread
        thread.start()

    def _recover_standard_head_profile(self) -> None:
        current_thread = threading.current_thread()
        try:
            switch = self._head_switch_config
            next_warning_at = time.monotonic() + switch.startup_timeout_sec
            attempt = 0
            last_result = None
            while self._running:
                with self._head_state_lock:
                    if not self._head_recovery_required:
                        return
                attempt += 1
                last_result = self.switch_head_resolution(
                    720,
                    recovery_only=True,
                )
                if last_result.status_code == 200:
                    if bool(last_result.body.get("recovery_yielded")):
                        return
                    self.get_logger().info(
                        "automatic head-camera recovery restored 720 RGB-D"
                    )
                    return
                retry_after = max(
                    float(last_result.body.get("retry_after_sec", 1.0)),
                    min(10.0, float(attempt)),
                )
                time.sleep(max(0.1, retry_after))
                if time.monotonic() >= next_warning_at:
                    self.get_logger().error(
                        "automatic 720 RGB-D recovery is still retrying after "
                        f"{switch.startup_timeout_sec:.1f}s: "
                        f"{None if last_result is None else last_result.body}"
                    )
                    next_warning_at = (
                        time.monotonic() + switch.startup_timeout_sec
                    )
        finally:
            replacement_thread = None
            with self._head_state_lock:
                if self._head_recovery_thread is current_thread:
                    self._head_recovery_thread = None
                    if self._running and self._head_recovery_required:
                        replacement_thread = threading.Thread(
                            target=self._recover_standard_head_profile,
                            name="head-profile-recover-720",
                            daemon=True,
                        )
                        self._head_recovery_thread = replacement_thread
            if replacement_thread is not None:
                replacement_thread.start()

    def _verify_head_profile_parameters(
        self,
        profile: HeadResolutionProfile,
    ) -> None:
        supervisor = self._head_process_supervisor
        if supervisor is None:
            raise RuntimeError("head camera process supervisor is not configured")
        process_status = supervisor.status()
        if (
            not bool(process_status.get("running"))
            or process_status.get("resolution") != profile.resolution
        ):
            raise _HeadProfileMismatch(
                "head process profile mismatch: "
                f"expected={profile.resolution}, status={process_status}"
            )
        names = [
            "rgb_camera.color_profile",
            "enable_color",
            "enable_depth",
            "enable_sync",
            "align_depth.enable",
        ]
        if profile.depth_profile is not None:
            names.append("depth_module.depth_profile")
        values = self._get_remote_parameters(names)
        expected = {
            "rgb_camera.color_profile": profile.color_profile,
            "enable_color": True,
            "enable_depth": profile.enable_depth,
            "enable_sync": profile.align_depth,
            "align_depth.enable": profile.align_depth,
        }
        if profile.depth_profile is not None:
            expected["depth_module.depth_profile"] = profile.depth_profile
        mismatches = {
            name: {"expected": value, "actual": values.get(name)}
            for name, value in expected.items()
            if values.get(name) != value
        }
        if mismatches:
            raise _HeadProfileMismatch(
                f"head parameter readback mismatch: {mismatches}"
            )

    def _verify_head_profile(self, profile: HeadResolutionProfile) -> None:
        self._verify_head_profile_parameters(profile)
        with self._head_state_lock:
            generation = self._head_generation
        if not self._head_profile_data_fresh(profile, generation):
            raise RuntimeError(
                f"reliable {profile.resolution} camera data became stale "
                "during parameter verification"
            )
        if profile.resolution == 1080:
            stream = self._config.find_stream(
                self._head_switch_config.camera_id,
                "color",
            )
            entry = None if stream is None else self._frame_store.get(stream.key)
            if (
                entry is None
                or entry.generation != generation
                or not self._valid_color_entry(entry, profile)
            ):
                raise RuntimeError("reliable 1080 color frame disappeared")
            return
        pair = self._rgbd_store.get(self._head_switch_config.camera_id)
        if (
            pair is None
            or pair.generation != generation
            or not self._valid_rgbd_pair(pair, profile)
        ):
            raise RuntimeError("reliable 720 aligned RGB-D pair disappeared")

    def frame_jpeg(self, entry, stream_type: str) -> bytes:
        if entry.jpeg is not None:
            return entry.jpeg
        if entry.raw is None:
            raise RuntimeError("frame has no raw payload")
        return encode_sensor_image_to_jpeg(
            data=entry.raw,
            height=entry.height,
            width=entry.width,
            encoding=entry.encoding,
            is_bigendian=entry.is_bigendian,
            step=int(
                entry.step
                or entry.width * (2 if stream_type == "depth" else 3)
            ),
            jpeg_quality=self._config.jpeg_quality,
            max_depth_mm=self._config.max_depth_mm,
            stream_type=stream_type,
        )

    def resolve_stream(self, camera_id: str, stream_type: str):
        stream_type = stream_type.strip().lower()
        if stream_type not in STREAM_TYPES:
            return HttpResult(400, {"error_code": "INVALID_REQUEST"})
        stream = self._config.find_stream(camera_id, stream_type)
        if stream is None:
            return HttpResult(404, {"error_code": "CAMERA_NOT_FOUND"})
        return stream

    def get_frame(self, key: str):
        entry = self._frame_store.get(key)
        switch = self._head_switch_config
        if (
            entry is None
            or switch is None
            or not key.startswith(f"{switch.camera_id}/")
        ):
            return entry
        with self._head_state_lock:
            generation = self._head_generation
        return entry if entry.generation == generation else None

    def get_snapshot_frame(self, camera_id: str, stream_type: str):
        stream = self.resolve_stream(camera_id, stream_type)
        if isinstance(stream, HttpResult):
            return stream
        access = self.stream_access(stream)
        if isinstance(access, HttpResult):
            return access
        entry = self._frame_store.get(stream.key)
        if entry is None:
            return HttpResult(503, {"error_code": "STREAM_NOT_READY"})
        if entry.generation != access:
            return HttpResult(503, {"error_code": "STREAM_NOT_READY"})
        age = time.monotonic() - entry.received_at
        if age > self._config.stale_after_sec:
            return HttpResult(503, {"error_code": "STREAM_STALE"})
        return entry

    def get_rgbd_archive(self, camera_id: str):
        resolved = self._config.resolve_camera_id(camera_id)
        if resolved is None:
            return HttpResult(404, {"error_code": "CAMERA_NOT_FOUND"})
        switch = self._head_switch_config
        if switch is not None and resolved == switch.camera_id:
            stream = self._config.find_stream(resolved, "depth")
            if stream is None:
                return HttpResult(404, {"error_code": "CAMERA_NOT_FOUND"})
            access = self.stream_access(stream)
            if isinstance(access, HttpResult):
                return access
        else:
            access = 0
        pair = self._rgbd_store.get(resolved)
        if pair is None:
            return HttpResult(503, {"error_code": "RGBD_NOT_READY"})
        if pair.generation != access:
            return HttpResult(503, {"error_code": "RGBD_NOT_READY"})
        age = time.monotonic() - min(
            pair.color.received_at,
            pair.depth.received_at,
        )
        if age > self._config.stale_after_sec:
            return HttpResult(503, {"error_code": "RGBD_STALE"})
        try:
            archive = build_rgbd_archive(
                pair,
                jpeg_quality=self._config.jpeg_quality,
            )
        except Exception as exc:
            self.get_logger().warning(
                f"failed to build RGB-D archive for {resolved}: {exc}"
            )
            return HttpResult(
                500,
                {
                    "error_code": "RGBD_ENCODE_FAILED",
                    "message": str(exc),
                },
            )
        return (
            archive,
            resolved,
            pair.color.stamp_sec,
            pair.color.stamp_nanosec,
        )

    def list_cameras(self) -> dict:
        payload = build_camera_list_payload(self._config, self._frame_store)
        payload["head_resolution"] = self.head_resolution_status()
        return payload

    def health_status(self) -> str:
        switch = self._head_switch_config
        if switch is not None and switch.enabled:
            with self._head_state_lock:
                state = self._head_state
                resolution = self._head_resolution
                verified_resolution = self._head_profile_verified_resolution
            if state == _HEAD_SWITCHING:
                return "STARTING"
            if state == _HEAD_DEGRADED:
                return "ERROR"
            if state in (_HEAD_READY_720, _HEAD_READY_1080):
                if verified_resolution != resolution:
                    return "STARTING"
                with self._head_state_lock:
                    generation = self._head_generation
                profile = switch.profile(resolution)
                if not self._head_profile_data_fresh(profile, generation):
                    return "ERROR"
        now = time.monotonic()
        head_id = None if switch is None else switch.camera_id
        wrist_ids = [
            camera_id
            for camera_id in self._config.camera_ids()
            if camera_id != head_id
        ]
        for camera_id in wrist_ids:
            pair = self._rgbd_store.get(camera_id)
            if pair is None:
                return "STARTING"
            age = now - min(
                pair.color.received_at,
                pair.depth.received_at,
            )
            if age > self._config.stale_after_sec:
                return "ERROR"
        return "READY"

    def destroy_node(self) -> bool:
        self._running = False
        try:
            self._http_server.shutdown()
            self._http_server.server_close()
        except Exception:
            pass
        if self._http_thread.is_alive():
            self._http_thread.join(timeout=2.0)
        try:
            return super().destroy_node()
        except ValueError:
            # rclpy can raise during subscription teardown on interrupt.
            return False


def main(args=None) -> None:
    rclpy.init(args=args)
    node = CameraHttpGateway()
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
