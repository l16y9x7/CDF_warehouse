"""Push HTTP MJPEG or direct ROS RGB through FFmpeg, without device ownership."""

from __future__ import annotations

from collections import deque
import logging
import os
import shutil
import subprocess
import threading
import time
from typing import Any, Callable, Deque, Dict, List, Mapping, Optional, Sequence

from vision.adapters.fake import map_camera_id
from vision.rtmp_target import (
    DEFAULT_ROBOT_RTMP_PATH_TEMPLATE,
    ROBOT_STREAM_SLOTS,
    build_rtmp_target,
    redact_url,
)

LOGGER = logging.getLogger(__name__)

PopenFactory = Callable[..., Any]
_STDERR_AUTH_MARKERS = (
    "auth failed",
    "unauthorized",
    "401",
    "server error",
)
_STDERR_HINT_MARKERS = _STDERR_AUTH_MARKERS + ("operation not permitted",)
_NVIDIA_GSTREAMER_PLUGINS = (
    "rawvideoparse",
    "queue",
    "videoconvert",
    "nvvidconv",
    "nvv4l2h264enc",
    "h264parse",
    "flvmux",
    "rtmpsink",
)


def pick_stderr_hint(lines: Sequence[str]) -> str:
    cleaned = [str(line or "").strip() for line in lines if str(line or "").strip()]
    if not cleaned:
        return ""
    for markers in (_STDERR_AUTH_MARKERS, _STDERR_HINT_MARKERS):
        for line in reversed(cleaned):
            lower = line.lower()
            if any(marker in lower for marker in markers):
                return line
    return cleaned[-1]


def resolve_ffmpeg_bin(configured: str = "") -> str:
    """配置优先；否则 IMAGEIO/PATH；再回退 imageio_ffmpeg 自带二进制。"""
    explicit = str(configured or "").strip()
    if explicit and explicit != "ffmpeg":
        return explicit
    env = str(os.environ.get("FFMPEG_BIN") or "").strip()
    if env:
        return env
    # Prefer conda env ffmpeg（imageio 自带二进制推 RTMP 会 segfault）
    conda_prefix = str(os.environ.get("CONDA_PREFIX") or "").strip()
    if conda_prefix:
        conda_ff = os.path.join(conda_prefix, "bin", "ffmpeg")
        if os.path.isfile(conda_ff) and os.access(conda_ff, os.X_OK):
            return conda_ff
    found = shutil.which("ffmpeg")
    if found:
        return found
    try:
        from imageio_ffmpeg import get_ffmpeg_exe

        return get_ffmpeg_exe()
    except Exception:
        return explicit or "ffmpeg"


def _resolve_executable(configured: str, default: str) -> str:
    explicit = str(configured or "").strip()
    if explicit:
        return explicit
    return shutil.which(default) or default


def nvidia_gstreamer_available(
    gst_launch_bin: str = "",
    gst_inspect_bin: str = "",
    *,
    run: Callable[..., Any] = subprocess.run,
) -> bool:
    """Return true only when the launcher and every required Jetson plugin work."""
    launch = _resolve_executable(gst_launch_bin, "gst-launch-1.0")
    inspect = _resolve_executable(gst_inspect_bin, "gst-inspect-1.0")
    for executable in (launch, inspect):
        if os.path.sep in executable and not os.access(executable, os.X_OK):
            return False
        if os.path.sep not in executable and shutil.which(executable) is None:
            return False
    try:
        for plugin in _NVIDIA_GSTREAMER_PLUGINS:
            completed = run(
                [inspect, plugin],
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
                timeout=3,
                check=False,
            )
            if completed.returncode != 0:
                return False
    except (OSError, subprocess.SubprocessError):
        return False
    return True


def resolve_encoder_backend(
    requested: str,
    *,
    source: str,
    gst_launch_bin: str = "",
    gst_inspect_bin: str = "",
) -> tuple[str, str]:
    choice = str(requested or "auto").strip().lower()
    if choice not in {"auto", "nvidia_gstreamer", "ffmpeg"}:
        raise ValueError("media.push.encoder_backend must be auto, nvidia_gstreamer, or ffmpeg")
    if choice == "ffmpeg":
        return "ffmpeg", "configured ffmpeg"
    if source != "ros":
        if choice == "nvidia_gstreamer":
            raise ValueError("nvidia_gstreamer currently requires media.push.source=ros")
        return "ffmpeg", "HTTP input uses ffmpeg"
    available = nvidia_gstreamer_available(gst_launch_bin, gst_inspect_bin)
    if available:
        return "nvidia_gstreamer", "Jetson GStreamer plugins available"
    if choice == "nvidia_gstreamer":
        raise ValueError("nvidia_gstreamer requested but required executables/plugins are unavailable")
    return "ffmpeg", "Jetson GStreamer unavailable; using ffmpeg"


class PushRuntime:
    def __init__(
        self,
        config: Mapping[str, Any],
        *,
        source_uri: Callable[[str], str],
        popen_factory: PopenFactory = subprocess.Popen,
    ) -> None:
        self._source_uri = source_uri
        self._lifecycle = threading.RLock()
        self._popen_factory = popen_factory
        push = _push_config(config)
        self.source = str(push.get("source", "http"))
        if self.source not in {"http", "ros"}:
            raise ValueError("media.push.source must be http or ros")
        self._ros_source = None
        if self.source == "ros":
            # SMT Tianji rule: platform video subscribes to Topics only — never USB.
            from vision.ownership import ensure_ros_media_subscriber_only
            ensure_ros_media_subscriber_only()
            from vision.ros_push import RosPushSource
            self._ros_source = RosPushSource([str(spec["camera_id"]) for spec in _stream_specs(push)
                                              if spec.get("enabled", True)])
        self.enabled = bool(push.get("enabled", True))
        self.autostart = bool(push.get("autostart", False))
        self.device_sn = str(push.get("device_sn") or "").strip()
        self.stream_server_url = str(
            push.get("stream_server_url") or "rtmp://robotsolution.cn:1935"
        ).strip()
        self.stream_key = str(push.get("stream_key") or "").strip()
        self.stream_path_template = str(
            push.get("stream_path_template") or DEFAULT_ROBOT_RTMP_PATH_TEMPLATE
        ).strip()
        self.ffmpeg_bin = resolve_ffmpeg_bin(str(push.get("ffmpeg_bin") or ""))
        self.gst_launch_bin = _resolve_executable(
            str(push.get("gst_launch_bin") or ""), "gst-launch-1.0"
        )
        self.encoder_backend_requested = str(push.get("encoder_backend") or "auto").strip().lower()
        self.encoder_backend, self.encoder_backend_reason = resolve_encoder_backend(
            self.encoder_backend_requested,
            source=self.source,
            gst_launch_bin=self.gst_launch_bin,
            gst_inspect_bin=str(push.get("gst_inspect_bin") or ""),
        )
        self._workers: Dict[str, _FfmpegWorker] = {}
        for spec in _stream_specs(push):
            worker = _FfmpegWorker(
                spec=spec,
                source_uri=source_uri,
                server_url=self.stream_server_url,
                stream_key=self.stream_key,
                path_template=self.stream_path_template,
                default_sn=self.device_sn,
                ffmpeg_bin=self.ffmpeg_bin,
                gst_launch_bin=self.gst_launch_bin,
                encoder_backend=self.encoder_backend,
                popen_factory=popen_factory,
                raw_source=self._ros_source,
            )
            self._workers[worker.camera_id] = worker

    def status(self, *, camera_id: str = "") -> Dict[str, Any]:
        wanted = map_camera_id(camera_id) if camera_id else ""
        workers = [
            worker
            for key, worker in self._workers.items()
            if not camera_id or (wanted is not None and key == wanted)
        ]
        return {
            "enabled": self.enabled,
            "source": self.source,
            "encoder_backend": self.encoder_backend,
            "encoder_backend_requested": self.encoder_backend_requested,
            "encoder_backend_reason": self.encoder_backend_reason,
            "server": self.stream_server_url,
            "device_sn": self.device_sn,
            "streams": [worker.snapshot() for worker in workers],
        }

    def start(self, *, camera_id: str = "", device_sn: str = "", stream_key: str = "") -> Dict[str, Any]:
        with self._lifecycle:
            return self._start_locked(camera_id=camera_id, device_sn=device_sn, stream_key=stream_key)

    def _start_locked(self, *, camera_id: str = "", device_sn: str = "", stream_key: str = "") -> Dict[str, Any]:
        if not self.enabled:
            return {"ok": False, "error_code": "MEDIA_PUSH_DISABLED", "message": "cloud push is disabled"}
        sn = str(device_sn or "").strip() or self.device_sn
        if sn:
            self.device_sn = sn
        workers = self._select(camera_id)
        if camera_id and not workers:
            return {
                "ok": False,
                "error_code": "CAMERA_NOT_FOUND",
                "message": f"unknown camera_id={camera_id}",
            }
        if any(not worker.enabled for worker in workers):
            return {"ok": False, "error_code": "MEDIA_STREAM_DISABLED",
                    "message": f"stream is disabled camera_id={camera_id}"}
        key = str(stream_key or "").strip()
        if key:
            self.stream_key = key
        if self._ros_source is not None:
            self._ros_source.start()
        for worker in workers:
            if key:
                worker.set_stream_key(key)
            worker.start(device_sn=sn)
        return {"ok": True, **self.status(camera_id=camera_id)}

    def stop(self, *, camera_id: str = "") -> Dict[str, Any]:
        with self._lifecycle:
            return self._stop_locked(camera_id=camera_id)

    def _stop_locked(self, *, camera_id: str = "") -> Dict[str, Any]:
        workers = self._select(camera_id)
        if camera_id and not workers:
            return {
                "ok": False,
                "error_code": "CAMERA_NOT_FOUND",
                "message": f"unknown camera_id={camera_id}",
            }
        for worker in workers:
            worker.stop()
        return {"ok": True, **self.status(camera_id=camera_id)}

    def stop_all(self) -> None:
        with self._lifecycle:
            self._stop_all_locked()

    def _stop_all_locked(self) -> None:
        for worker in self._workers.values():
            worker.stop()
        if self._ros_source is not None:
            self._ros_source.stop()

    def _select(self, camera_id: str) -> List["_FfmpegWorker"]:
        wanted = map_camera_id(camera_id) if camera_id else ""
        if camera_id:
            worker = self._workers.get(wanted)
            return [worker] if worker is not None else []
        return [worker for worker in self._workers.values() if worker.enabled]


class _FfmpegWorker:
    def __init__(
        self,
        *,
        spec: Mapping[str, Any],
        source_uri: Callable[[str], str],
        server_url: str,
        stream_key: str,
        path_template: str,
        default_sn: str,
        ffmpeg_bin: str,
        gst_launch_bin: str,
        encoder_backend: str,
        popen_factory: PopenFactory,
        raw_source=None,
    ) -> None:
        self.camera_id = str(spec.get("camera_id") or "").strip()
        self.enabled = bool(spec.get("enabled", True))
        self.stream_slot = int(spec.get("stream_slot") or ROBOT_STREAM_SLOTS.get(self.camera_id, 1))
        self.camera_index = int(spec.get("camera_index") or 99)
        self.video_channel_index = int(spec.get("video_channel_index") or 0)
        self.width = int(spec.get("width") or 640)
        self.height = int(spec.get("height") or 480)
        self.fps = int(spec.get("fps") or 15)
        self.bitrate = str(spec.get("bitrate") or "1200k").strip()
        self.preset = str(spec.get("preset") or "ultrafast").strip()
        self.restart_initial_sec = max(float(spec.get("restart_initial_sec") or 1.0), 0.1)
        self.restart_max_sec = max(float(spec.get("restart_max_sec") or 30.0), self.restart_initial_sec)
        self._source_uri = source_uri
        self._raw_source = raw_source
        self._feed_stop = threading.Event()
        self._feeder = None
        self._lifecycle = threading.RLock()
        if min(self.width, self.height, self.fps) <= 0:
            raise ValueError("media dimensions and fps must be positive")
        self._server_url = server_url
        self._stream_key = stream_key
        self._path_template = path_template
        self._default_sn = default_sn
        self._ffmpeg_bin = ffmpeg_bin
        self._gst_launch_bin = gst_launch_bin
        self.encoder_backend = encoder_backend
        self._popen_factory = popen_factory
        self._lock = threading.Lock()
        self._process: Optional[Any] = None
        self._watchdog: Optional[threading.Thread] = None
        self._stop = threading.Event()
        self._desired = False
        self._device_sn = default_sn
        self._reason = "managed stream not started"
        self._restart_attempt = 0
        self._spawned_at = 0.0
        self._target_url = ""
        self._spawning = False
        self._stderr_tail: Deque[str] = deque(maxlen=24)
        self._input_failure = None
        self._frames_sent = 0
        self._last_progress_at = 0.0

    def set_stream_key(self, stream_key: str) -> None:
        self._stream_key = str(stream_key or "").strip()

    def snapshot(self) -> Dict[str, Any]:
        with self._lock:
            process = self._process
            running = process is not None and process.poll() is None
            reason = self._reason
            if not running and process is not None:
                code = process.poll()
                reason = (self._input_failure[1] if self._input_failure is not None
                          and self._input_failure[0] is process else
                          self._failure_reason(f"encoder exited: code={code}")) or reason
            progress_age = (
                max(0.0, time.monotonic() - self._last_progress_at)
                if self._last_progress_at else None
            )
            return {
                "camera_id": self.camera_id,
                "source": "ros" if self._raw_source is not None else "http",
                "source_ready": self._raw_source.latest(self.camera_id) is not None if self._raw_source is not None else None,
                "enabled": self.enabled,
                "stream_slot": self.stream_slot,
                "online": bool(running),
                "desired": self._desired,
                "encoder_backend": self.encoder_backend,
                "frames_sent": self._frames_sent,
                "progress_age_sec": round(progress_age, 3) if progress_age is not None else None,
                "target": redact_url(self._target_url),
                "reason": "" if running else reason,
            }

    def start(self, *, device_sn: str = "") -> None:
        with self._lifecycle:
            self._start_locked(device_sn=device_sn)

    def _start_locked(self, *, device_sn: str = "") -> None:
        if self._stop.is_set() and self._watchdog is not None and self._watchdog.is_alive():
            raise RuntimeError("previous media watchdog is stopping")
        sn = str(device_sn or "").strip() or self._default_sn
        with self._lock:
            self._device_sn = sn or self._device_sn
            self._desired = True
            self._stop.clear()
        self._spawn()
        self._ensure_watchdog()

    def stop(self) -> None:
        with self._lifecycle:
            with self._lock:
                self._desired = False
                self._stop.set()
            self._terminate("managed stream stopped")
        watchdog = self._watchdog
        if watchdog is not None and watchdog is not threading.current_thread():
            watchdog.join(timeout=1.0)
        if watchdog is None or not watchdog.is_alive():
            self._watchdog = None

    def _ensure_watchdog(self) -> None:
        if self._watchdog is not None and self._watchdog.is_alive():
            return
        self._watchdog = threading.Thread(
            target=self._watch,
            name=f"media-push-{self.camera_id}",
            daemon=True,
        )
        self._watchdog.start()

    def _watch(self) -> None:
        while not self._stop.wait(0.5):
            with self._lock:
                desired = self._desired
                process = self._process
                spawned_at = self._spawned_at
            if not desired:
                return
            if process is not None and process.poll() is None:
                if spawned_at and time.monotonic() - spawned_at >= 15.0:
                    with self._lock:
                        self._restart_attempt = 0
                        self._reason = ""
                continue
            if process is not None:
                code = process.poll()
                time.sleep(0.2)
                with self._lifecycle:
                    if self._process is not process:
                        continue
                    if self._input_failure is not None and self._input_failure[0] is process:
                        self._mark(self._input_failure[1])
                    else:
                        self._mark(self._failure_reason(f"encoder exited: code={code}"))
                    self._terminate(self._reason)
            # Missing camera input is not an RTMP failure. Poll without spawning
            # the encoder or growing the network-failure backoff while it is absent.
            if self._reason in {"camera source uri unavailable", "ROS color source stale or unavailable"}:
                if self._stop.wait(self.restart_initial_sec):
                    return
                self._spawn()
                continue
            delay = min(
                self.restart_initial_sec * (2 ** min(self._restart_attempt, 8)),
                self.restart_max_sec,
            )
            reason = (self._reason or "").lower()
            if "401" in reason or "auth failed" in reason or "unauthorized" in reason:
                delay = max(delay, self.restart_max_sec)
            self._restart_attempt += 1
            LOGGER.warning(
                "media push retry: camera=%s attempt=%s delay=%.1fs reason=%s",
                self.camera_id,
                self._restart_attempt,
                delay,
                self._reason,
            )
            if self._stop.wait(delay):
                return
            self._spawn()

    def _spawn(self) -> None:
        with self._lifecycle:
            if self._stop.is_set() or not self._desired:
                return
            self._spawn_locked()

    def _spawn_locked(self) -> None:
        with self._lock:
            if self._spawning:
                return
            if self._process is not None and self._process.poll() is None:
                return
            self._spawning = True
        try:
            if self._process is not None or self._feeder is not None:
                self._terminate(self._reason)
            self._spawn_unlocked()
        finally:
            with self._lock:
                self._spawning = False

    def _spawn_unlocked(self) -> None:
        raw_shape = None
        if self._raw_source is not None:
            latest = self._raw_source.latest(self.camera_id)
            if latest is None:
                self._mark("ROS color source stale or unavailable")
                return
            raw_frame = latest[0]
            raw_shape = raw_frame.shape
            raw_pixel_format = getattr(raw_frame, "pixel_format", "bgr24")
            source = "pipe:0"
        else:
            raw_pixel_format = "bgr24"
            source = str(self._source_uri(self.camera_id) or "").strip()
        if not source or source.startswith("fake://"):
            self._mark("camera source uri unavailable")
            return
        target = resolve_push_target(
            self._server_url,
            self._stream_key,
            device_sn=self._device_sn,
            stream_slot=self.stream_slot,
            camera_index=self.camera_index,
            video_channel_index=self.video_channel_index,
            camera_id=self.camera_id,
            stream_path_template=self._path_template,
        )
        if not target:
            self._mark("rtmp target is empty")
            return
        if self.encoder_backend == "nvidia_gstreamer":
            if raw_shape is None:
                self._mark("nvidia_gstreamer requires raw ROS input")
                return
            command = _gstreamer_command(
                gst_launch_bin=self._gst_launch_bin,
                target=target,
                width=self.width,
                height=self.height,
                fps=self.fps,
                bitrate=self.bitrate,
                raw_shape=raw_shape,
                raw_pixel_format=raw_pixel_format,
            )
        else:
            command = _ffmpeg_command(
                ffmpeg_bin=self._ffmpeg_bin,
                source=source,
                target=target,
                width=self.width,
                height=self.height,
                fps=self.fps,
                bitrate=self.bitrate,
                preset=self.preset,
                raw_shape=raw_shape,
                raw_pixel_format=raw_pixel_format,
            )
        try:
            process = self._popen_factory(
                command,
                stdin=subprocess.PIPE if raw_shape is not None else subprocess.DEVNULL,
                stdout=subprocess.DEVNULL,
                stderr=subprocess.PIPE,
                start_new_session=True,
            )
        except Exception as exc:
            self._mark(f"encoder start failed: {type(exc).__name__}")
            LOGGER.warning(
                "media push start failed: camera=%s error=%s",
                self.camera_id,
                type(exc).__name__,
            )
            return
        with self._lock:
            self._process = process
            self._target_url = target
            self._spawned_at = time.monotonic()
            self._stderr_tail.clear()
            self._input_failure = None
        if raw_shape is not None:
            self._feed_stop = threading.Event()
            self._feeder = threading.Thread(
                target=self._feed,
                args=(process, raw_shape, raw_pixel_format, self._feed_stop),
                                            name=f"media-raw-{self.camera_id}", daemon=True)
            self._feeder.start()
        threading.Thread(
            target=self._drain_stderr,
            args=(process,),
            name=f"media-push-{self.camera_id}-stderr",
            daemon=True,
        ).start()
        LOGGER.info(
            "media push started: camera=%s source=%s target=%s",
            self.camera_id,
            source,
            redact_url(target),
        )

    def _feed(self, process, shape, pixel_format, stop):
        from vision.ros_push import feed_frames
        try:
            feed_frames(
                process,
                self._raw_source,
                self.camera_id,
                shape,
                self.fps,
                stop,
                on_frame=self._record_frame_sent,
                pixel_format=pixel_format,
            )
        except Exception as exc:
            if not stop.is_set():
                reason = str(exc) if isinstance(exc, (TimeoutError, ValueError)) else "rawvideo input failed"
                with self._lock:
                    if self._process is process:
                        self._input_failure = (process, reason)
                        self._reason = reason
                # This feeder owns only its encoder; never terminate a replacement.
                try:
                    process.kill()
                except ProcessLookupError:
                    pass

    def _record_frame_sent(self) -> None:
        with self._lock:
            self._frames_sent += 1
            self._last_progress_at = time.monotonic()

    def _drain_stderr(self, process: Any) -> None:
        stderr = getattr(process, "stderr", None)
        if stderr is None:
            return
        try:
            for raw in stderr:
                if isinstance(raw, bytes):
                    line = raw.decode("utf-8", errors="replace").strip()
                else:
                    line = str(raw or "").strip()
                if line:
                    with self._lock:
                        if self._process is process:
                            self._stderr_tail.append(redact_url(line)[:300])
        except Exception:
            return
        finally:
            stderr.close()

    def _failure_reason(self, reason: str) -> str:
        hint = pick_stderr_hint(self._stderr_tail)
        if not hint:
            return reason
        return f"{reason}; stderr={hint}"

    def _terminate(self, reason: str) -> None:
        with self._lifecycle:
            self._terminate_locked(reason)

    def _terminate_locked(self, reason: str) -> None:
        self._feed_stop.set()
        if self._feeder is not None:
            self._feeder.join(timeout=1.0)
            if self._feeder.is_alive():
                if self._process is not None and self._process.poll() is None:
                    self._process.kill()
                self._feeder.join(timeout=0.5)
                if self._feeder.is_alive():
                    raise RuntimeError("rawvideo feeder did not stop")
            self._feeder = None
        with self._lock:
            process = self._process
            self._process = None
            if reason:
                self._reason = reason
        if process is None:
            return
        try:
            if process.poll() is None:
                process.terminate()
                try:
                    process.wait(timeout=3.0)
                except subprocess.TimeoutExpired:
                    process.kill()
                    process.wait(timeout=1.0)
        except Exception as exc:
            LOGGER.warning(
                "media push cleanup failed: camera=%s error=%s",
                self.camera_id,
                type(exc).__name__,
            )
        finally:
            if self._raw_source is not None and getattr(process, "stdin", None) is not None:
                process.stdin.close()

    def _mark(self, reason: str) -> None:
        with self._lock:
            self._reason = reason


def _input_demuxer(source: str) -> str:
    """Owner/天机 HTTP 流是 multipart MJPEG，需 mpjpeg；裸文件/管道仍用 mjpeg。"""
    lower = str(source or "").strip().lower()
    if lower.startswith("http://") or lower.startswith("https://"):
        return "mpjpeg"
    return "mjpeg"


def resolve_push_target(
    stream_server_url: str,
    stream_key: str = "",
    *,
    device_sn: str,
    stream_slot: int,
    camera_index: int = 99,
    video_channel_index: int = 0,
    camera_id: str = "",
    stream_path_template: str = DEFAULT_ROBOT_RTMP_PATH_TEMPLATE,
) -> str:
    """云端 RTMP，或本地联调 `file:///path/dir` → `/path/dir/{camera_id}.flv`。"""
    import os
    from urllib.parse import urlparse, unquote

    raw = str(stream_server_url or "").strip()
    parsed = urlparse(raw)
    if parsed.scheme == "file":
        base = unquote(parsed.path or "")
        if not base:
            return ""
        os.makedirs(base, exist_ok=True)
        name = str(camera_id or f"slot{stream_slot}" or "stream").strip() or "stream"
        return os.path.join(base, f"{name}.flv")
    return build_rtmp_target(
        stream_server_url,
        stream_key,
        device_sn=device_sn,
        stream_slot=stream_slot,
        camera_index=camera_index,
        video_channel_index=video_channel_index,
        camera_id=camera_id,
        stream_path_template=stream_path_template,
    )


def _ffmpeg_command(
    *,
    ffmpeg_bin: str,
    source: str,
    target: str,
    width: int,
    height: int,
    fps: int,
    bitrate: str,
    preset: str,
    raw_shape=None,
    raw_pixel_format: str = "bgr24",
) -> List[str]:
    bufsize = _double_bitrate(bitrate)
    video_filter = (
        f"scale={width}:{height}:force_original_aspect_ratio=decrease,"
        f"pad={width}:{height}:(ow-iw)/2:(oh-ih)/2"
    )
    http_source = str(source or "").lower().startswith(("http://", "https://"))
    command = [
        ffmpeg_bin,
        "-hide_banner",
        "-nostats",
        "-loglevel",
        "warning",
    ]
    if http_source:
        command.extend(
            [
                "-reconnect",
                "1",
                "-reconnect_streamed",
                "1",
                "-reconnect_delay_max",
                "2",
            ]
        )
    if raw_shape is not None:
        if raw_pixel_format not in {"rgb24", "bgr24"}:
            raise ValueError("raw pixel format must be rgb24 or bgr24")
        command.extend(["-pixel_format", raw_pixel_format, "-video_size",
                        f"{raw_shape[1]}x{raw_shape[0]}", "-framerate", str(fps)])
    command.extend(
        [
            "-fflags",
            "+genpts",
            "-f",
            "rawvideo" if raw_shape is not None else _input_demuxer(source),
            "-i",
            source,
            "-an",
            "-vf",
            video_filter,
            "-r",
            str(max(fps, 1)),
            "-c:v",
            "libx264",
            "-preset",
            preset or "ultrafast",
            "-tune",
            "zerolatency",
            "-profile:v",
            "baseline",
            "-b:v",
            bitrate,
            "-maxrate",
            bitrate,
            "-bufsize",
            bufsize,
            "-g",
            str(max(fps, 1)),
            "-keyint_min",
            str(max(fps, 1)),
            "-sc_threshold",
            "0",
            "-bf",
            "0",
            "-pix_fmt",
            "yuv420p",
            "-f",
            "flv",
            "-flvflags",
            "no_duration_filesize",
            target,
        ]
    )
    if not str(target).lower().startswith("rtmp"):
        # 本地 file 目标尽早落盘，便于冒烟验收
        for i, tok in enumerate(command):
            if tok == "-f" and i + 1 < len(command) and command[i + 1] == "flv":
                command.insert(i, "1")
                command.insert(i, "-flush_packets")
                break
    return command


def _gstreamer_command(
    *,
    gst_launch_bin: str,
    target: str,
    width: int,
    height: int,
    fps: int,
    bitrate: str,
    raw_shape,
    raw_pixel_format: str = "bgr24",
) -> List[str]:
    """Build a bounded RGB/BGR stdin -> Jetson NVENC -> FLV command."""
    if raw_shape is None or len(raw_shape) != 3 or raw_shape[2] != 3:
        raise ValueError("nvidia_gstreamer requires HxWx3 RGB/BGR input")
    if raw_pixel_format not in {"rgb24", "bgr24"}:
        raise ValueError("raw pixel format must be rgb24 or bgr24")
    rate = _bitrate_bps(bitrate)
    sink = (
        ["rtmpsink", f"location={target}", "sync=false", "async=false"]
        if str(target).lower().startswith("rtmp")
        else ["filesink", f"location={target}", "sync=false"]
    )
    return [
        gst_launch_bin,
        "-q",
        "fdsrc", "fd=0", "do-timestamp=true",
        "!", "rawvideoparse", f"format={raw_pixel_format[:-2]}", f"width={raw_shape[1]}",
        f"height={raw_shape[0]}", f"framerate={max(fps, 1)}/1",
        "!", "queue", "max-size-buffers=1", "max-size-bytes=0",
        "max-size-time=0", "leaky=downstream",
        "!", "videoconvert",
        "!", "video/x-raw,format=BGRx",
        "!", "nvvidconv",
        "!", f"video/x-raw(memory:NVMM),format=NV12,width={width},height={height},framerate={max(fps, 1)}/1",
        "!", "nvv4l2h264enc", f"bitrate={rate}", "control-rate=1",
        f"iframeinterval={max(fps, 1)}", f"idrinterval={max(fps, 1)}",
        "insert-sps-pps=true", "num-B-Frames=0", "profile=0", "maxperf-enable=true",
        "!", "h264parse", "config-interval=1",
        "!", "flvmux", "streamable=true",
        "!", *sink,
    ]

def _push_config(config: Mapping[str, Any]) -> Mapping[str, Any]:
    media = config.get("media") or {}
    if not isinstance(media, Mapping):
        return {}
    push = media.get("push") or {}
    return push if isinstance(push, Mapping) else {}


def _stream_specs(push: Mapping[str, Any]) -> Sequence[Mapping[str, Any]]:
    raw = push.get("streams")
    if isinstance(raw, list) and raw:
        return [item for item in raw if isinstance(item, Mapping) and item.get("camera_id")]
    return [
        {"camera_id": "head", "stream_slot": 1, "enabled": True},
        {"camera_id": "hand_left", "stream_slot": 2, "enabled": True},
        {"camera_id": "hand_right", "stream_slot": 3, "enabled": True},
    ]


def _double_bitrate(value: str) -> str:
    normalized = str(value or "").strip().lower()
    if normalized.endswith("k"):
        try:
            return f"{float(normalized[:-1]) * 2:g}k"
        except ValueError:
            return "2400k"
    try:
        return str(int(normalized) * 2)
    except ValueError:
        return "2400k"


def _bitrate_bps(value: str) -> int:
    normalized = str(value or "").strip().lower()
    multiplier = 1
    if normalized.endswith("k"):
        normalized, multiplier = normalized[:-1], 1000
    elif normalized.endswith("m"):
        normalized, multiplier = normalized[:-1], 1000 * 1000
    try:
        result = int(float(normalized) * multiplier)
    except ValueError as exc:
        raise ValueError(f"invalid bitrate: {value}") from exc
    if result <= 0:
        raise ValueError("bitrate must be positive")
    return result
