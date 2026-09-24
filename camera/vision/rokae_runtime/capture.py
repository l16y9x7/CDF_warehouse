"""V4L2 / Orbbec SDK / Fake 取流 worker。"""

from __future__ import annotations

import logging
import threading
import time
from typing import Callable, Optional

import numpy as np

LOGGER = logging.getLogger(__name__)


class V4L2CaptureWorker:
    def __init__(
        self,
        *,
        camera_id: str,
        device: str,
        width: int = 640,
        height: int = 480,
        fps: int = 10,
        warmup_frames: int = 3,
        fourcc: str = "",
        device_resolver: Optional[Callable[[], Optional[str]]] = None,
    ) -> None:
        self.camera_id = camera_id
        self.device = device
        self.width = width
        self.height = height
        self.fps = max(int(fps), 1)
        self.warmup_frames = max(int(warmup_frames), 1)
        self.fourcc = fourcc
        self._device_resolver = device_resolver
        self.reconnect_delay_sec = 1.0
        self._cap = None
        self._thread: Optional[threading.Thread] = None
        self._stop = threading.Event()
        self._lock = threading.Lock()
        self._frame: Optional[np.ndarray] = None
        self._stamp = 0.0
        self._running = False

    def _open_capture(self) -> bool:
        import cv2

        cap = cv2.VideoCapture(self.device, cv2.CAP_V4L2)
        if not cap.isOpened():
            cap.release()
            return False
        self._cap = cap
        try:
            if self.fourcc:
                cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*self.fourcc))
            cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
            cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)
            cap.set(cv2.CAP_PROP_FPS, self.fps)
            cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        except Exception:
            self._close_capture()
            raise
        return True

    def _close_capture(self) -> None:
        cap, self._cap = self._cap, None
        if cap is not None:
            cap.release()
        with self._lock:
            self._frame = None
            self._stamp = 0.0

    def start(self) -> bool:
        if self._thread and self._thread.is_alive():
            return not self._stop.is_set()
        if not self.device and self._device_resolver is None:
            return False
        if self.device and not self._open_capture():
            LOGGER.warning("open failed camera=%s device=%s", self.camera_id, self.device)
            if self._device_resolver is None:
                return False
        self._stop.clear()
        self._thread = threading.Thread(
            target=self._loop, name=f"rokae-v4l2-{self.camera_id}", daemon=True
        )
        self._thread.start()
        self._running = True
        return True

    def _loop(self) -> None:
        interval = 1.0 / float(self.fps)
        failures = warmed = 0
        while not self._stop.is_set():
            if self._cap is None:
                if self._stop.wait(self.reconnect_delay_sec):
                    return
                try:
                    device = self._device_resolver() if self._device_resolver else self.device
                    if not device or self._stop.is_set():
                        continue
                    self.device = device
                    if not self._open_capture():
                        continue
                    LOGGER.info("V4L2 reopened camera=%s device=%s", self.camera_id, device)
                    failures = warmed = 0
                except Exception:
                    LOGGER.warning("V4L2 reopen failed camera=%s", self.camera_id, exc_info=True)
                    continue
            try:
                ok, frame = self._cap.read()
            except Exception:
                LOGGER.warning("V4L2 read failed camera=%s", self.camera_id, exc_info=True)
                ok, frame = False, None
            if self._stop.is_set():
                return
            if ok and frame is not None:
                failures = 0
                warmed += 1
                if warmed >= self.warmup_frames:
                    with self._lock:
                        self._frame = frame
                        self._stamp = time.time()
                self._stop.wait(interval)
            else:
                failures += 1
                if failures >= 3:
                    LOGGER.warning("V4L2 stream lost camera=%s; retrying original binding", self.camera_id)
                    try:
                        self._close_capture()
                    except Exception:
                        LOGGER.warning("V4L2 release failed camera=%s", self.camera_id, exc_info=True)
                else:
                    self._stop.wait(0.05)

    def get_frame(self, max_stale_sec: float = 2.0) -> Optional[np.ndarray]:
        with self._lock:
            if self._stop.is_set() or self._frame is None:
                return None
            if max_stale_sec > 0 and (time.time() - self._stamp) > max_stale_sec:
                return None
            return self._frame.copy()

    def age_sec(self) -> Optional[float]:
        with self._lock:
            if self._stamp <= 0:
                return None
            return time.time() - self._stamp

    def ready(self, max_stale_sec: float = 2.0) -> bool:
        return self.get_frame(max_stale_sec=max_stale_sec) is not None

    def stop(self) -> None:
        self._stop.set()
        if self._thread and self._thread.is_alive():
            self._thread.join(timeout=2.0)
        if self._thread and self._thread.is_alive():
            # Never release a native handle while its reader still owns it.
            raise RuntimeError(f"capture thread did not stop: {self.camera_id}")
        self._thread = None
        if self._cap is not None:
            try:
                self._cap.release()
            except Exception:
                pass
            self._cap = None
        self._running = False
        with self._lock:
            self._frame = None
            self._stamp = 0.0


class OrbbecSdkCaptureWorker:
    """通过可选 pyorbbecsdk 采集 RGB；设备选择必须无歧义。"""

    def __init__(
        self,
        *,
        camera_id: str,
        width: int = 640,
        height: int = 480,
        fps: int = 10,
        serial: str = "",
    ) -> None:
        self.camera_id = camera_id
        self.width = width
        self.height = height
        self.fps = max(int(fps), 1)
        self.serial = str(serial or "").strip()
        self._pipeline = None
        self._context = None
        self._needs_stop = False
        self._sdk = None
        self.reconnect_delay_sec = 2.0
        self.frame_timeout_sec = 5.0
        self._last_error = ""
        self._thread: Optional[threading.Thread] = None
        self._stop = threading.Event()
        self._lock = threading.Lock()
        self._frame: Optional[np.ndarray] = None
        self._stamp = 0.0
        self._running = False

    def start(self) -> bool:
        if self._thread and self._thread.is_alive():
            return not self._stop.is_set()
        if self._pipeline is not None:
            # A failed stop retains ownership; never create a second pipeline.
            return False
        try:
            import pyorbbecsdk as sdk
        except Exception as exc:
            LOGGER.warning("pyorbbecsdk missing camera=%s: %s", self.camera_id, exc)
            return False
        self._sdk = sdk
        self._stop.clear()
        self._set_error("CAPTURE_STARTING")
        self._thread = threading.Thread(
            target=self._loop, name=f"rokae-orbbec-{self.camera_id}", daemon=True
        )
        self._running = True
        self._thread.start()
        return True

    @property
    def last_error(self) -> str:
        with self._lock:
            return self._last_error

    def _set_error(self, error: str) -> None:
        with self._lock:
            self._last_error = error

    def _close_pipeline(self) -> None:
        with self._lock:
            self._frame = None
            self._stamp = 0.0
        if self._pipeline is not None and self._needs_stop:
            # Keep the handle if the native SDK cannot confirm it stopped.
            self._pipeline.stop()
        self._pipeline = None
        self._needs_stop = False
        self._context = None

    def _open_pipeline(self) -> bool:
        sdk = self._sdk
        try:
            ctx = sdk.Context()
            device_list = ctx.query_devices()
            candidates = []
            for index in range(int(device_list.get_count())):
                candidate = device_list.get_device_by_index(index)
                info = candidate.get_device_info()
                sn = str(info.get_serial_number() or "")
                name = str(info.get_name() or "").upper()
                if (self.serial and sn == self.serial) or (not self.serial and "GEMINI" in name):
                    candidates.append((candidate, sn))
            if len(candidates) != 1:
                self._set_error("DEVICE_NOT_FOUND" if not candidates else "DEVICE_AMBIGUOUS")
                LOGGER.warning("Orbbec selection ambiguous/missing camera=%s count=%d; configure serial",
                               self.camera_id, len(candidates))
                return False
            device, serial = candidates[0]
            pipeline = sdk.Pipeline(device)
            self._pipeline = pipeline
            self._context = ctx
            profiles = pipeline.get_stream_profile_list(sdk.OBSensorType.COLOR_SENSOR)
            supported = {sdk.OBFormat.MJPG, sdk.OBFormat.RGB, sdk.OBFormat.BGR,
                         sdk.OBFormat.YUYV, sdk.OBFormat.UYVY}
            items = []
            for index in range(profiles.get_count()):
                profile = profiles.get_stream_profile_by_index(index).as_video_stream_profile()
                if profile.get_format() in supported:
                    items.append(profile)
            if not items:
                self._set_error("COLOR_PROFILE_NOT_FOUND")
                LOGGER.warning("no Orbbec color profiles camera=%s", self.camera_id)
                return False

            def _score(p) -> tuple:
                fmt = p.get_format()
                prefer = 1 if fmt == sdk.OBFormat.MJPG else 0
                return (
                    int(p.get_width()) == self.width,
                    int(p.get_height()) == self.height,
                    int(p.get_fps()) == self.fps,
                    prefer,
                    -abs(int(p.get_fps()) - self.fps),
                    int(p.get_width()) * int(p.get_height()),
                )

            color_profile = max(items, key=_score)
            config = sdk.Config()
            config.enable_stream(color_profile)
            # Pin the resolved identity even when the first start fails.
            self.serial = serial
            if self._stop.is_set():
                return False
            self._needs_stop = True
            pipeline.start(config)
            self._set_error("WAITING_FOR_FRAME")
            LOGGER.info(
                "orbbec started camera=%s sn=%s %dx%d@%d fmt=%s",
                self.camera_id,
                serial,
                color_profile.get_width(),
                color_profile.get_height(),
                color_profile.get_fps(),
                color_profile.get_format(),
            )
            return True
        except Exception as exc:
            self._set_error("CAPTURE_START_FAILED")
            LOGGER.exception("orbbec start failed camera=%s: %s", self.camera_id, exc)
            return False

    def _color_to_bgr(self, frame) -> Optional[np.ndarray]:
        import cv2

        sdk = self._sdk
        if sdk is None or frame is None:
            return None
        width = int(frame.get_width())
        height = int(frame.get_height())
        data = np.asanyarray(frame.get_data())
        fmt = frame.get_format()
        if fmt == sdk.OBFormat.MJPG:
            return cv2.imdecode(data, cv2.IMREAD_COLOR)
        if fmt == sdk.OBFormat.RGB:
            return cv2.cvtColor(data.reshape((height, width, 3)), cv2.COLOR_RGB2BGR)
        if fmt == sdk.OBFormat.BGR:
            return data.reshape((height, width, 3)).copy()
        if fmt == sdk.OBFormat.YUYV:
            return cv2.cvtColor(data.reshape((height, width, 2)), cv2.COLOR_YUV2BGR_YUYV)
        if fmt == sdk.OBFormat.UYVY:
            return cv2.cvtColor(data.reshape((height, width, 2)), cv2.COLOR_YUV2BGR_UYVY)
        LOGGER.warning("unsupported orbbec format=%s camera=%s", fmt, self.camera_id)
        return None

    def _loop(self) -> None:
        interval = 1.0 / float(self.fps)
        last_valid = time.monotonic()
        try:
            while not self._stop.is_set():
                if self._pipeline is None:
                    if not self._open_pipeline():
                        self._close_pipeline()
                        if self._stop.wait(self.reconnect_delay_sec):
                            break
                        continue
                    last_valid = time.monotonic()
                if self._stop.is_set():
                    break
                bgr = None
                try:
                    frames = self._pipeline.wait_for_frames(100)
                    if frames is not None:
                        color = frames.get_color_frame()
                        if color is not None:
                            bgr = self._color_to_bgr(color)
                except Exception:
                    self._set_error("CAPTURE_READ_FAILED")
                if bgr is not None:
                    last_valid = time.monotonic()
                    with self._lock:
                        self._frame = bgr
                        self._stamp = time.time()
                        self._last_error = ""
                    self._stop.wait(interval)
                elif time.monotonic() - last_valid >= self.frame_timeout_sec:
                    self._set_error("FRAME_TIMEOUT")
                    LOGGER.warning("Orbbec no valid color frames camera=%s; reconnecting same serial", self.camera_id)
                    self._close_pipeline()
                    if self._stop.wait(self.reconnect_delay_sec):
                        break
                else:
                    self._stop.wait(0.01)
        except Exception:
            self._set_error("CAPTURE_STOP_FAILED")
            LOGGER.exception("Orbbec recovery failed camera=%s", self.camera_id)
        finally:
            try:
                self._close_pipeline()
            except Exception:
                self._set_error("CAPTURE_STOP_FAILED")
                LOGGER.exception("Orbbec cleanup failed camera=%s", self.camera_id)
            self._running = False

    def get_frame(self, max_stale_sec: float = 2.0) -> Optional[np.ndarray]:
        with self._lock:
            if self._stop.is_set() or self._frame is None:
                return None
            if max_stale_sec > 0 and (time.time() - self._stamp) > max_stale_sec:
                return None
            return self._frame.copy()

    def age_sec(self) -> Optional[float]:
        with self._lock:
            if self._stamp <= 0:
                return None
            return time.time() - self._stamp

    def ready(self, max_stale_sec: float = 2.0) -> bool:
        return self.get_frame(max_stale_sec=max_stale_sec) is not None

    def stop(self) -> None:
        self._stop.set()
        if self._thread and self._thread.is_alive():
            self._thread.join(timeout=2.0)
        if self._thread and self._thread.is_alive():
            raise RuntimeError(f"capture thread did not stop: {self.camera_id}")
        self._thread = None
        self._close_pipeline()
        self._sdk = None
        self._context = None
        self._running = False
        with self._lock:
            self._frame = None
            self._stamp = 0.0


class FakeCaptureWorker:
    """无硬件联调：生成纯色帧。"""

    def __init__(self, *, camera_id: str, width: int = 640, height: int = 480, color=(40, 160, 40)) -> None:
        self.camera_id = camera_id
        self.width = width
        self.height = height
        self.color = color
        self._stamp = time.time()
        self._running = False

    def start(self) -> bool:
        self._running = True
        self._stamp = time.time()
        return True

    def get_frame(self, max_stale_sec: float = 2.0) -> Optional[np.ndarray]:
        del max_stale_sec
        if not self._running:
            return None
        self._stamp = time.time()
        frame = np.zeros((self.height, self.width, 3), dtype=np.uint8)
        frame[:] = self.color
        return frame

    def age_sec(self) -> Optional[float]:
        return 0.0 if self._running else None

    def ready(self, max_stale_sec: float = 2.0) -> bool:
        del max_stale_sec
        return self._running

    def stop(self) -> None:
        self._running = False
