from __future__ import annotations

import importlib.util
import threading
import time
from datetime import datetime
from typing import Any

from .backends import BackendError
from .camera import CameraSnapshot


class RealSenseCameraBackend:
    """D435i RGB/depth capture with depth aligned to the RGB viewport."""

    def __init__(self, config: dict[str, Any]) -> None:
        self._config = config
        self._camera_id = str(config.get("camera_id", "wrist"))
        self._label = str(config.get("camera_label", "腕部 RealSense"))
        self._lock = threading.RLock()
        self._condition = threading.Condition(self._lock)
        self._stop_event = threading.Event()
        self._capture_thread: threading.Thread | None = None
        self._preview_thread: threading.Thread | None = None
        self._pipeline: Any | None = None
        self._align: Any | None = None
        self._rs: Any | None = None
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
            "model": "Intel RealSense D435i",
            "serial_number": str(config.get("serial_number", "")),
            "firmware_version": "",
            "alignment": "software_depth_to_color",
        }
        self._error: str | None = None
        self._actual_fps = 0.0
        self._fps_started_at = 0.0
        self._fps_frames = 0
        self._actual_preview_fps = 0.0
        self._preview_fps_started_at = 0.0
        self._preview_fps_frames = 0

    def _load_dependencies(self) -> None:
        if self._rs is not None:
            return
        try:
            import cv2
            import numpy as np
            import pyrealsense2 as rs
        except Exception as exc:
            raise BackendError(f"无法加载 RealSense 摄像头 SDK: {exc}") from exc
        self._rs = rs
        self._cv2 = cv2
        self._np = np

    def _friendly_start_error(self, exc: Exception) -> BackendError:
        message = str(exc)
        lowered = message.lower()
        if "permission" in lowered or "access denied" in lowered:
            return BackendError("RealSense USB 权限不足，请安装 librealsense udev 规则后重试")
        if "no device" in lowered or "not found" in lowered:
            return BackendError(f"未找到{self._label} D435i，请确认接线和腕部选择")
        return BackendError(f"开启{self._label}失败: {message}")

    def start(self) -> dict[str, Any]:
        with self._condition:
            if self._enabled:
                return self.status()
            if self._starting:
                raise BackendError(f"{self._label}正在启动")
            self._starting = True
            self._error = None

        pipeline = None
        try:
            self._load_dependencies()
            rs = self._rs
            pipeline = rs.pipeline()
            stream_config = rs.config()
            serial_number = str(self._config.get("serial_number", "")).strip()
            if serial_number:
                stream_config.enable_device(serial_number)
            stream_config.enable_stream(
                rs.stream.color,
                int(self._config["rgb_width"]),
                int(self._config["rgb_height"]),
                rs.format.bgr8,
                int(self._config["capture_fps"]),
            )
            stream_config.enable_stream(
                rs.stream.depth,
                int(self._config["depth_width"]),
                int(self._config["depth_height"]),
                rs.format.z16,
                int(self._config["capture_fps"]),
            )
            profile = pipeline.start(stream_config)
            device = profile.get_device()
            depth_scale_mm = float(device.first_depth_sensor().get_depth_scale()) * 1000.0
            align = rs.align(rs.stream.color)
            camera_info = dict(self._camera_info)
            camera_info.update(
                model=device.get_info(rs.camera_info.name),
                serial_number=device.get_info(rs.camera_info.serial_number),
                firmware_version=device.get_info(rs.camera_info.firmware_version),
                rgb_profile={
                    "width": int(self._config["rgb_width"]),
                    "height": int(self._config["rgb_height"]),
                    "fps": int(self._config["capture_fps"]),
                    "format": "BGR8",
                },
                depth_profile={
                    "width": int(self._config["depth_width"]),
                    "height": int(self._config["depth_height"]),
                    "fps": int(self._config["capture_fps"]),
                    "format": "Z16",
                },
                output_size=[
                    int(self._config["rgb_width"]),
                    int(self._config["rgb_height"]),
                ],
                depth_scale_mm=depth_scale_mm,
            )

            with self._condition:
                self._pipeline = pipeline
                self._align = align
                self._camera_info = camera_info
                self._stop_event.clear()
                self._enabled = True
                self._starting = False
                self._sequence = 0
                self._raw_sequence = 0
                self._latest_rgb_bgr = None
                self._latest_depth_mm = None
                self._latest_rgb_jpeg = None
                self._fps_started_at = time.monotonic()
                self._fps_frames = 0
                self._preview_fps_started_at = time.monotonic()
                self._preview_fps_frames = 0
                self._capture_thread = threading.Thread(
                    target=self._capture_loop,
                    name=f"realsense-{self._camera_id}",
                    daemon=True,
                )
                self._preview_thread = threading.Thread(
                    target=self._preview_loop,
                    name=f"realsense-{self._camera_id}-preview",
                    daemon=True,
                )
                self._capture_thread.start()
                self._preview_thread.start()
                deadline = time.monotonic() + 6.0
                while self._enabled and self._sequence == 0 and time.monotonic() < deadline:
                    self._condition.wait(timeout=0.2)
                if self._sequence > 0:
                    return self.status()
                error = self._error or "等待首帧超时"
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
        raise BackendError(f"开启{self._label}失败: {error}")

    def _capture_loop(self) -> None:
        try:
            while not self._stop_event.is_set():
                frames = self._pipeline.wait_for_frames(1000)
                aligned_frames = self._align.process(frames)
                color_frame = aligned_frames.get_color_frame()
                depth_frame = aligned_frames.get_depth_frame()
                if not color_frame or not depth_frame:
                    continue
                rgb_bgr = self._np.asanyarray(color_frame.get_data()).copy()
                depth_raw = self._np.asanyarray(depth_frame.get_data())
                depth_mm = depth_raw.astype(self._np.float32) * float(
                    self._camera_info["depth_scale_mm"]
                )
                if depth_mm.shape != rgb_bgr.shape[:2]:
                    raise BackendError(
                        "RealSense 对齐后尺寸不一致: "
                        f"RGB={rgb_bgr.shape[1]}x{rgb_bgr.shape[0]}, "
                        f"Depth={depth_mm.shape[1]}x{depth_mm.shape[0]}"
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
                    if next_preview > now:
                        self._condition.wait(timeout=next_preview - now)
                        continue
                    rgb_bgr = self._latest_rgb_bgr
                    seen_raw_sequence = self._raw_sequence
                if rgb_bgr is None:
                    continue
                preview = self._cv2.resize(
                    rgb_bgr,
                    (
                        int(self._config["preview_width"]),
                        int(self._config["preview_height"]),
                    ),
                    interpolation=self._cv2.INTER_AREA,
                )
                ok, jpeg = self._cv2.imencode(
                    ".jpg",
                    preview,
                    [self._cv2.IMWRITE_JPEG_QUALITY, int(self._config["jpeg_quality"])],
                )
                if not ok:
                    raise BackendError("编码 RealSense RGB 预览帧失败")
                completed_at = time.monotonic()
                next_preview = max(next_preview + display_interval, completed_at)
                self._preview_fps_frames += 1
                elapsed = completed_at - self._preview_fps_started_at
                if elapsed >= 1.0:
                    self._actual_preview_fps = self._preview_fps_frames / elapsed
                    self._preview_fps_frames = 0
                    self._preview_fps_started_at = completed_at
                with self._condition:
                    self._latest_rgb_jpeg = jpeg.tobytes()
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
            capture_thread = self._capture_thread
            preview_thread = self._preview_thread
            self._stop_event.set()
            self._enabled = False
            self._condition.notify_all()
        if capture_thread is not None and capture_thread is not threading.current_thread():
            capture_thread.join(timeout=2.5)
        if preview_thread is not None and preview_thread is not threading.current_thread():
            preview_thread.join(timeout=2.5)
        with self._condition:
            if self._pipeline is not None:
                try:
                    self._pipeline.stop()
                except Exception:
                    pass
            self._pipeline = None
            self._align = None
            self._capture_thread = None
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
            return {
                "available": importlib.util.find_spec("pyrealsense2") is not None,
                "enabled": self._enabled,
                "starting": self._starting,
                "model": self._camera_info.get("model", "Intel RealSense D435i"),
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
            raise BackendError("未知 RealSense 摄像头流")
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
                raise BackendError(f"{self._label}未开启或尚未取得完整 RGB/深度帧")
            return CameraSnapshot(
                rgb_bgr=self._latest_rgb_bgr.copy(),
                depth_aligned_mm=self._latest_depth_mm.copy(),
                captured_at=self._latest_captured_at,
                color_timestamp_ms=self._latest_color_timestamp_ms,
                depth_timestamp_ms=self._latest_depth_timestamp_ms,
                sequence=self._sequence,
                camera_info=dict(self._camera_info),
            )

    def close(self) -> None:
        self.stop()
