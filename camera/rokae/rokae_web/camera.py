from __future__ import annotations

import importlib.util
import json
import shutil
import threading
import time
import uuid
from dataclasses import dataclass
from datetime import datetime
from pathlib import Path
from typing import Any

from .backends import BackendError


@dataclass(frozen=True)
class CameraSnapshot:
    rgb_bgr: Any
    depth_aligned_mm: Any
    captured_at: str
    color_timestamp_ms: float
    depth_timestamp_ms: float
    sequence: int
    camera_info: dict[str, Any]


class RecordingStore:
    def __init__(self, data_root: str | Path) -> None:
        self.root = Path(data_root).expanduser().resolve()
        self.root.mkdir(parents=True, exist_ok=True)
        self._lock = threading.Lock()

    def save(self, snapshot: CameraSnapshot, robot_state: dict[str, Any]) -> dict[str, Any]:
        import cv2
        import numpy as np

        with self._lock:
            while True:
                recorded_at = datetime.now().astimezone()
                day_dir = self.root / recorded_at.strftime("%Y%m%d")
                leaf = recorded_at.strftime("%H%M%S") + f"{recorded_at.microsecond // 1000:03d}"
                target = day_dir / leaf
                if not target.exists():
                    break
                time.sleep(0.001)

            day_dir.mkdir(parents=True, exist_ok=True)
            temporary = day_dir / f".{leaf}-{uuid.uuid4().hex}.tmp"
            temporary.mkdir()
            try:
                rgb_path = temporary / "rgb.jpg"
                if not cv2.imwrite(
                    str(rgb_path),
                    snapshot.rgb_bgr,
                    [cv2.IMWRITE_JPEG_QUALITY, 95],
                ):
                    raise BackendError("保存 RGB 图像失败")
                np.save(
                    temporary / "depth_aligned.npy",
                    snapshot.depth_aligned_mm.astype(np.float32, copy=False),
                    allow_pickle=False,
                )
                (temporary / "robot_state.json").write_text(
                    json.dumps(robot_state, ensure_ascii=False, indent=2),
                    encoding="utf-8",
                )
                metadata = {
                    "recorded_at": recorded_at.isoformat(timespec="milliseconds"),
                    "camera_frame_captured_at": snapshot.captured_at,
                    "color_timestamp_ms": snapshot.color_timestamp_ms,
                    "depth_timestamp_ms": snapshot.depth_timestamp_ms,
                    "camera_sequence": snapshot.sequence,
                    "rgb_file": "rgb.jpg",
                    "depth_file": "depth_aligned.npy",
                    "depth_dtype": "float32",
                    "depth_unit": "millimeter",
                    "invalid_depth_value": 0,
                    "camera": snapshot.camera_info,
                }
                (temporary / "camera_metadata.json").write_text(
                    json.dumps(metadata, ensure_ascii=False, indent=2),
                    encoding="utf-8",
                )
                temporary.replace(target)
            except Exception:
                shutil.rmtree(temporary, ignore_errors=True)
                raise

        relative = target.relative_to(self.root)
        return {
            "directory": str(target),
            "relative_directory": f"{self.root.name}/{relative.as_posix()}",
            "files": [
                "rgb.jpg",
                "depth_aligned.npy",
                "robot_state.json",
                "camera_metadata.json",
            ],
        }


class DisabledCameraBackend:
    def status(self) -> dict[str, Any]:
        return {
            "available": False,
            "enabled": False,
            "starting": False,
            "model": "未启用",
            "error": None,
        }

    def start(self) -> dict[str, Any]:
        raise BackendError("当前服务模式未启用头部深度摄像头")

    def stop(self) -> dict[str, Any]:
        return self.status()

    def frame_jpeg(
        self,
        kind: str,
        after_sequence: int,
        timeout: float,
    ) -> tuple[bytes | None, int, bool]:
        del kind, after_sequence, timeout
        return None, 0, False

    def snapshot(self) -> CameraSnapshot:
        raise BackendError("摄像头未开启")

    def save_record(
        self,
        snapshot: CameraSnapshot,
        robot_state: dict[str, Any],
    ) -> dict[str, Any]:
        del snapshot, robot_state
        raise BackendError("摄像头未开启")

    def close(self) -> None:
        return


class OrbbecCameraBackend:
    """Gemini 335L RGB/depth capture with D2C-aligned depth and MJPEG previews."""

    def __init__(self, config: dict[str, Any]) -> None:
        self._config = config
        self._store = RecordingStore(config["data_directory"])
        self._lock = threading.RLock()
        self._condition = threading.Condition(self._lock)
        self._stop_event = threading.Event()
        self._thread: threading.Thread | None = None
        self._preview_thread: threading.Thread | None = None
        self._pipeline: Any | None = None
        self._align_filter: Any | None = None
        self._sdk: Any | None = None
        self._cv2: Any | None = None
        self._np: Any | None = None
        self._enabled = False
        self._starting = False
        self._sequence = 0
        self._raw_sequence = 0
        self._latest_rgb_bgr: Any | None = None
        self._latest_depth_mm: Any | None = None
        self._latest_rgb_jpeg: bytes | None = None
        self._latest_captured_at = ""
        self._latest_color_timestamp_ms = 0.0
        self._latest_depth_timestamp_ms = 0.0
        self._camera_info: dict[str, Any] = {
            "model": "Orbbec Gemini 335L",
            "serial_number": "",
            "firmware_version": "",
            "alignment": "",
        }
        self._error: str | None = None
        self._actual_fps = 0.0
        self._fps_started_at = 0.0
        self._fps_frames = 0
        self._actual_preview_fps = 0.0
        self._preview_fps_started_at = 0.0
        self._preview_fps_frames = 0

    @staticmethod
    def _format_name(value: Any) -> str:
        text = str(value)
        return text.rsplit(".", 1)[-1]

    @staticmethod
    def _profile_dict(profile: Any) -> dict[str, Any]:
        return {
            "width": int(profile.get_width()),
            "height": int(profile.get_height()),
            "fps": int(profile.get_fps()),
            "format": OrbbecCameraBackend._format_name(profile.get_format()),
        }

    def _load_dependencies(self) -> None:
        if self._sdk is not None:
            return
        try:
            import cv2
            import numpy as np
            import pyorbbecsdk as sdk
        except Exception as exc:
            raise BackendError(f"无法加载 Orbbec 摄像头 SDK: {exc}") from exc
        self._sdk = sdk
        self._cv2 = cv2
        self._np = np

    def _profiles(self, pipeline: Any, sensor_type: Any) -> list[Any]:
        profiles = pipeline.get_stream_profile_list(sensor_type)
        return [profiles[index] for index in range(len(profiles))]

    def _color_candidates(self, pipeline: Any) -> list[Any]:
        sdk = self._sdk
        profiles = self._profiles(pipeline, sdk.OBSensorType.COLOR_SENSOR)
        width = int(self._config["rgb_width"])
        height = int(self._config["rgb_height"])
        capture_fps = int(self._config["capture_fps"])
        exact = [
            profile
            for profile in profiles
            if int(profile.get_width()) == width and int(profile.get_height()) == height
        ]
        target_aspect = width / height
        candidates = exact or sorted(
            profiles,
            key=lambda profile: (
                -abs((int(profile.get_width()) / int(profile.get_height())) - target_aspect),
                int(profile.get_width()) * int(profile.get_height()),
            ),
            reverse=True,
        )
        format_rank = {
            sdk.OBFormat.MJPG: 4,
            sdk.OBFormat.RGB: 3,
            sdk.OBFormat.BGR: 2,
            sdk.OBFormat.YUYV: 1,
        }
        return sorted(
            candidates,
            key=lambda profile: (
                int(profile.get_width()) == width and int(profile.get_height()) == height,
                -abs(
                    (int(profile.get_width()) / int(profile.get_height()))
                    - target_aspect
                ),
                int(profile.get_width()) * int(profile.get_height()),
                int(profile.get_fps()) >= int(self._config["display_fps"]),
                -abs(int(profile.get_fps()) - capture_fps),
                format_rank.get(profile.get_format(), 0),
            ),
            reverse=True,
        )

    def _configure_pipeline(self, pipeline: Any) -> tuple[Any, Any | None, dict[str, Any]]:
        sdk = self._sdk
        color_candidates = self._color_candidates(pipeline)
        preferred_size = (
            int(color_candidates[0].get_width()),
            int(color_candidates[0].get_height()),
        )
        for color_profile in color_candidates:
            if (
                int(color_profile.get_width()),
                int(color_profile.get_height()),
            ) != preferred_size:
                continue
            try:
                depth_list = pipeline.get_d2c_depth_profile_list(
                    color_profile,
                    sdk.OBAlignMode.HW_MODE,
                )
                depth_profiles = [depth_list[index] for index in range(len(depth_list))]
            except Exception:
                depth_profiles = []
            if not depth_profiles:
                continue
            depth_profile = max(
                depth_profiles,
                key=lambda profile: (
                    int(profile.get_fps()) == int(color_profile.get_fps()),
                    int(profile.get_fps()),
                    int(profile.get_width()) * int(profile.get_height()),
                ),
            )
            config = sdk.Config()
            config.enable_stream(color_profile)
            config.enable_stream(depth_profile)
            config.set_align_mode(sdk.OBAlignMode.HW_MODE)
            config.set_frame_aggregate_output_mode(
                sdk.OBFrameAggregateOutputMode.FULL_FRAME_REQUIRE
            )
            try:
                pipeline.enable_frame_sync()
            except Exception:
                pass
            return config, None, {
                "alignment": "hardware_depth_to_color",
                "rgb_profile": self._profile_dict(color_profile),
                "depth_profile": self._profile_dict(depth_profile),
            }

        color_profile = color_candidates[0]
        depth_profiles = self._profiles(pipeline, sdk.OBSensorType.DEPTH_SENSOR)
        minimum_fps = int(self._config["display_fps"])
        depth_profile = max(
            depth_profiles,
            key=lambda profile: (
                int(profile.get_fps()) >= minimum_fps,
                -abs(int(profile.get_fps()) - int(color_profile.get_fps())),
                int(profile.get_width()) * int(profile.get_height()),
            ),
        )
        config = sdk.Config()
        config.enable_stream(color_profile)
        config.enable_stream(depth_profile)
        config.set_frame_aggregate_output_mode(sdk.OBFrameAggregateOutputMode.FULL_FRAME_REQUIRE)
        align_filter = sdk.AlignFilter(align_to_stream=sdk.OBStreamType.COLOR_STREAM)
        return config, align_filter, {
            "alignment": "software_depth_to_color",
            "rgb_profile": self._profile_dict(color_profile),
            "depth_profile": self._profile_dict(depth_profile),
        }

    def _friendly_start_error(self, exc: Exception) -> BackendError:
        message = str(exc)
        if "openUsbDevice failed" in message or "permission" in message.lower():
            return BackendError(
                "摄像头 USB 权限不足；请安装 Orbbec 99-obsensor-libusb.rules 后重试"
            )
        return BackendError(f"开启头部摄像头失败: {message}")

    def start(self) -> dict[str, Any]:
        with self._condition:
            if self._enabled:
                return self.status()
            if self._starting:
                raise BackendError("摄像头正在启动")
            self._starting = True
            self._error = None

        pipeline = None
        try:
            self._load_dependencies()
            pipeline = self._sdk.Pipeline()
            stream_config, align_filter, stream_info = self._configure_pipeline(pipeline)
            pipeline.start(stream_config)
            camera_info = dict(self._camera_info)
            camera_info.update(stream_info)
            try:
                info = pipeline.get_device().get_device_info()
                camera_info.update(
                    model=info.get_name(),
                    serial_number=info.get_serial_number(),
                    firmware_version=info.get_firmware_version(),
                )
            except Exception:
                pass

            with self._condition:
                self._pipeline = pipeline
                self._align_filter = align_filter
                self._camera_info = camera_info
                self._stop_event.clear()
                self._enabled = True
                self._starting = False
                self._sequence = 0
                self._raw_sequence = 0
                self._fps_started_at = time.monotonic()
                self._fps_frames = 0
                self._preview_fps_started_at = time.monotonic()
                self._preview_fps_frames = 0
                self._thread = threading.Thread(
                    target=self._capture_loop,
                    name="orbbec-camera",
                    daemon=True,
                )
                self._preview_thread = threading.Thread(
                    target=self._preview_loop,
                    name="orbbec-camera-preview",
                    daemon=True,
                )
                self._thread.start()
                self._preview_thread.start()

                deadline = time.monotonic() + 6.0
                while self._enabled and self._sequence == 0 and time.monotonic() < deadline:
                    self._condition.wait(timeout=0.2)
                if self._sequence == 0:
                    error = self._error or "等待首帧超时"
                else:
                    return self.status()
        except Exception as exc:
            with self._condition:
                self._starting = False
                self._error = str(exc)
                self._condition.notify_all()
            if pipeline is not None:
                try:
                    pipeline.stop()
                except Exception:
                    pass
            if isinstance(exc, BackendError):
                raise
            raise self._friendly_start_error(exc) from exc

        self.stop()
        raise BackendError(f"开启头部摄像头失败: {error}")

    def _color_to_bgr(self, frame: Any) -> Any | None:
        sdk = self._sdk
        cv2 = self._cv2
        np = self._np
        width = int(frame.get_width())
        height = int(frame.get_height())
        data = np.asanyarray(frame.get_data())
        frame_format = frame.get_format()
        if frame_format == sdk.OBFormat.MJPG:
            return cv2.imdecode(data, cv2.IMREAD_COLOR)
        if frame_format == sdk.OBFormat.RGB:
            return cv2.cvtColor(data.reshape((height, width, 3)), cv2.COLOR_RGB2BGR)
        if frame_format == sdk.OBFormat.BGR:
            return data.reshape((height, width, 3)).copy()
        if frame_format == sdk.OBFormat.YUYV:
            return cv2.cvtColor(data.reshape((height, width, 2)), cv2.COLOR_YUV2BGR_YUYV)
        if frame_format == sdk.OBFormat.UYVY:
            return cv2.cvtColor(data.reshape((height, width, 2)), cv2.COLOR_YUV2BGR_UYVY)
        if frame_format == sdk.OBFormat.NV12:
            return cv2.cvtColor(data.reshape((height * 3 // 2, width)), cv2.COLOR_YUV2BGR_NV12)
        if frame_format == sdk.OBFormat.NV21:
            return cv2.cvtColor(data.reshape((height * 3 // 2, width)), cv2.COLOR_YUV2BGR_NV21)
        return None

    def _encode_rgb_preview(self, rgb_bgr: Any) -> bytes:
        cv2 = self._cv2
        preview_width = int(self._config["preview_width"])
        preview_height = int(self._config["preview_height"])
        rgb_preview = cv2.resize(
            rgb_bgr,
            (preview_width, preview_height),
            interpolation=cv2.INTER_AREA,
        )
        quality = int(self._config["jpeg_quality"])
        rgb_ok, rgb_jpeg = cv2.imencode(
            ".jpg", rgb_preview, [cv2.IMWRITE_JPEG_QUALITY, quality]
        )
        if not rgb_ok:
            raise BackendError("编码摄像头预览帧失败")
        return rgb_jpeg.tobytes()

    def _capture_loop(self) -> None:
        try:
            while not self._stop_event.is_set():
                frames = self._pipeline.wait_for_frames(1000)
                if not frames:
                    continue
                if self._align_filter is not None:
                    frames = self._align_filter.process(frames)
                    if not frames:
                        continue
                color_frame = frames.get_color_frame()
                depth_frame = frames.get_depth_frame()
                if not color_frame or not depth_frame:
                    continue
                rgb_bgr = self._color_to_bgr(color_frame)
                if rgb_bgr is None:
                    raise BackendError(
                        f"不支持的 RGB 格式: {self._format_name(color_frame.get_format())}"
                    )
                depth_raw = self._np.frombuffer(
                    depth_frame.get_data(),
                    dtype=self._np.uint16,
                ).reshape((depth_frame.get_height(), depth_frame.get_width()))
                depth_mm = depth_raw.astype(self._np.float32) * float(
                    depth_frame.get_depth_scale()
                )
                if depth_mm.shape != rgb_bgr.shape[:2]:
                    raise BackendError(
                        "深度到 RGB 对齐后的尺寸不一致: "
                        f"RGB={rgb_bgr.shape[1]}x{rgb_bgr.shape[0]}, "
                        f"Depth={depth_mm.shape[1]}x{depth_mm.shape[0]}"
                    )
                output_size = (
                    int(self._config["rgb_width"]),
                    int(self._config["rgb_height"]),
                )
                if rgb_bgr.shape[1::-1] != output_size:
                    rgb_bgr = self._cv2.resize(
                        rgb_bgr,
                        output_size,
                        interpolation=self._cv2.INTER_LINEAR,
                    )
                    depth_mm = self._cv2.resize(
                        depth_mm,
                        output_size,
                        interpolation=self._cv2.INTER_NEAREST,
                    )

                now = time.monotonic()
                self._fps_frames += 1
                elapsed = now - self._fps_started_at
                if elapsed >= 1.0:
                    self._actual_fps = self._fps_frames / elapsed
                    self._fps_frames = 0
                    self._fps_started_at = now

                with self._condition:
                    self._latest_rgb_bgr = rgb_bgr
                    self._latest_depth_mm = depth_mm
                    self._latest_captured_at = datetime.now().astimezone().isoformat(
                        timespec="milliseconds"
                    )
                    self._latest_color_timestamp_ms = float(color_frame.get_timestamp())
                    self._latest_depth_timestamp_ms = float(depth_frame.get_timestamp())
                    self._raw_sequence += 1
                    self._condition.notify_all()
        except Exception as exc:
            with self._condition:
                self._error = str(exc)
        finally:
            try:
                if self._pipeline is not None:
                    self._pipeline.stop()
            except Exception:
                pass
            with self._condition:
                self._enabled = False
                self._pipeline = None
                self._latest_rgb_jpeg = None
                self._condition.notify_all()

    def _preview_loop(self) -> None:
        display_interval = 1.0 / float(self._config["display_fps"])
        next_preview = 0.0
        seen_raw_sequence = 0
        try:
            while not self._stop_event.is_set():
                with self._condition:
                    while (
                        self._enabled
                        and self._raw_sequence <= seen_raw_sequence
                        and not self._stop_event.is_set()
                    ):
                        self._condition.wait(timeout=0.25)
                    if not self._enabled or self._stop_event.is_set():
                        return
                    now = time.monotonic()
                    wait_seconds = next_preview - now
                    if wait_seconds > 0:
                        self._condition.wait(timeout=wait_seconds)
                        continue
                    rgb_bgr = self._latest_rgb_bgr
                    seen_raw_sequence = self._raw_sequence
                if rgb_bgr is None:
                    continue
                rgb_jpeg = self._encode_rgb_preview(rgb_bgr)
                completed_at = time.monotonic()
                next_preview = max(next_preview + display_interval, completed_at)
                self._preview_fps_frames += 1
                preview_elapsed = completed_at - self._preview_fps_started_at
                if preview_elapsed >= 1.0:
                    self._actual_preview_fps = self._preview_fps_frames / preview_elapsed
                    self._preview_fps_frames = 0
                    self._preview_fps_started_at = completed_at
                with self._condition:
                    self._latest_rgb_jpeg = rgb_jpeg
                    self._sequence += 1
                    self._condition.notify_all()
        except Exception as exc:
            with self._condition:
                self._error = str(exc)
                self._enabled = False
                self._stop_event.set()
                self._condition.notify_all()

    def stop(self) -> dict[str, Any]:
        with self._condition:
            thread = self._thread
            preview_thread = self._preview_thread
            self._stop_event.set()
            self._enabled = False
            self._condition.notify_all()
        if thread is not None and thread is not threading.current_thread():
            thread.join(timeout=2.5)
        if preview_thread is not None and preview_thread is not threading.current_thread():
            preview_thread.join(timeout=2.5)
        with self._condition:
            if self._pipeline is not None:
                try:
                    self._pipeline.stop()
                except Exception:
                    pass
            self._pipeline = None
            self._thread = None
            self._preview_thread = None
            self._starting = False
            self._latest_rgb_bgr = None
            self._latest_depth_mm = None
            self._latest_rgb_jpeg = None
            self._sequence = 0
            self._raw_sequence = 0
            self._actual_fps = 0.0
            self._actual_preview_fps = 0.0
            self._condition.notify_all()
            return self.status()

    def status(self) -> dict[str, Any]:
        with self._lock:
            available = importlib.util.find_spec("pyorbbecsdk") is not None
            return {
                "available": available,
                "enabled": self._enabled,
                "starting": self._starting,
                "model": self._camera_info.get("model", "Orbbec Gemini 335L"),
                "serial_number": self._camera_info.get("serial_number", ""),
                "firmware_version": self._camera_info.get("firmware_version", ""),
                "alignment": self._camera_info.get("alignment", ""),
                "rgb_profile": self._camera_info.get("rgb_profile"),
                "depth_profile": self._camera_info.get("depth_profile"),
                "display_fps": int(self._config["display_fps"]),
                "actual_capture_fps": round(self._actual_fps, 1),
                "actual_preview_fps": round(self._actual_preview_fps, 1),
                "preview_size": [
                    int(self._config["preview_width"]),
                    int(self._config["preview_height"]),
                ],
                "output_size": [
                    int(self._config["rgb_width"]),
                    int(self._config["rgb_height"]),
                ],
                "error": self._error,
            }

    def frame_jpeg(
        self,
        kind: str,
        after_sequence: int,
        timeout: float,
    ) -> tuple[bytes | None, int, bool]:
        if kind != "rgb":
            raise BackendError("未知摄像头流")
        deadline = time.monotonic() + timeout
        with self._condition:
            while (
                self._enabled
                and self._sequence <= after_sequence
                and time.monotonic() < deadline
            ):
                remaining = max(0.0, deadline - time.monotonic())
                self._condition.wait(timeout=min(0.5, remaining))
            return self._latest_rgb_jpeg, self._sequence, self._enabled

    def snapshot(self) -> CameraSnapshot:
        with self._condition:
            if not self._enabled or self._latest_rgb_bgr is None or self._latest_depth_mm is None:
                raise BackendError("摄像头未开启或尚未取得完整 RGB/深度帧")
            return CameraSnapshot(
                rgb_bgr=self._latest_rgb_bgr.copy(),
                depth_aligned_mm=self._latest_depth_mm.copy(),
                captured_at=self._latest_captured_at,
                color_timestamp_ms=self._latest_color_timestamp_ms,
                depth_timestamp_ms=self._latest_depth_timestamp_ms,
                sequence=self._sequence,
                camera_info=dict(self._camera_info),
            )

    def save_record(
        self,
        snapshot: CameraSnapshot,
        robot_state: dict[str, Any],
    ) -> dict[str, Any]:
        return self._store.save(snapshot, robot_state)

    def close(self) -> None:
        self.stop()
