"""Camera HTTP gateway helpers that do not depend on ROS."""

from __future__ import annotations

import io
import json
import threading
import time
import zipfile
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Optional
from urllib.parse import parse_qs, urlsplit

import cv2
import numpy as np
import yaml


STREAM_TYPES = ("color", "depth")
HEAD_RESOLUTIONS = (720, 1080)


def _positive_number(value: Any, field: str) -> float:
    try:
        parsed = float(value)
    except (TypeError, ValueError) as exc:
        raise ValueError(f"{field} must be a positive number") from exc
    if not np.isfinite(parsed) or parsed <= 0:
        raise ValueError(f"{field} must be a positive number")
    return parsed


def _positive_int(value: Any, field: str) -> int:
    parsed = _positive_number(value, field)
    integer = int(parsed)
    if float(integer) != parsed:
        raise ValueError(f"{field} must be a positive integer")
    return integer


def _parse_video_profile(value: Any, field: str) -> tuple[int, int, int]:
    if not isinstance(value, str):
        raise ValueError(f"{field} must be WIDTHxHEIGHTxFPS")
    parts = value.lower().replace(" ", "").split("x")
    if len(parts) != 3:
        raise ValueError(f"{field} must be WIDTHxHEIGHTxFPS")
    width = _positive_int(parts[0], f"{field}.width")
    height = _positive_int(parts[1], f"{field}.height")
    fps = _positive_int(parts[2], f"{field}.fps")
    return width, height, fps


@dataclass(frozen=True)
class CameraStreamConfig:
    camera_id: str
    stream_type: str
    topic: str

    @property
    def key(self) -> str:
        return f"{self.camera_id}/{self.stream_type}"


@dataclass(frozen=True)
class HeadResolutionProfile:
    resolution: int
    color_profile: str
    depth_profile: Optional[str]
    enable_depth: bool
    align_depth: bool
    settle_frames: int

    @property
    def color_size(self) -> tuple[int, int]:
        width, height, _ = _parse_video_profile(
            self.color_profile,
            f"head_resolution_switch.profiles.{self.resolution}.color_profile",
        )
        return width, height

    @property
    def color_fps(self) -> int:
        _, _, fps = _parse_video_profile(
            self.color_profile,
            f"head_resolution_switch.profiles.{self.resolution}.color_profile",
        )
        return fps

    @property
    def depth_size(self) -> Optional[tuple[int, int]]:
        if self.depth_profile is None:
            return None
        width, height, _ = _parse_video_profile(
            self.depth_profile,
            f"head_resolution_switch.profiles.{self.resolution}.depth_profile",
        )
        return width, height


@dataclass(frozen=True)
class HeadResolutionSwitchConfig:
    enabled: bool
    camera_id: str
    node_name: str
    parameter_timeout_sec: float
    readiness_timeout_sec: float
    startup_timeout_sec: float
    profiles: dict[int, HeadResolutionProfile]
    serial_no: str
    process_runtime_dir: str
    process_profile_configs: dict[int, str]
    process_stop_timeout_sec: float
    process_startup_probe_sec: float

    def profile(self, resolution: int) -> HeadResolutionProfile:
        try:
            return self.profiles[int(resolution)]
        except (KeyError, TypeError, ValueError) as exc:
            raise ValueError("resolution must be 720 or 1080") from exc


@dataclass(frozen=True)
class CameraGatewayConfig:
    stale_after_sec: float
    jpeg_quality: int
    max_depth_mm: int
    stream_fps: float
    streams: tuple[CameraStreamConfig, ...]
    aliases: dict[str, str]
    head_resolution_switch: Optional[HeadResolutionSwitchConfig]

    def resolve_camera_id(self, camera_id: str) -> Optional[str]:
        camera_id = camera_id.strip()
        if not camera_id:
            return None
        if any(stream.camera_id == camera_id for stream in self.streams):
            return camera_id
        return self.aliases.get(camera_id) or self.aliases.get(camera_id.lower())

    def find_stream(
        self, camera_id: str, stream_type: str
    ) -> Optional[CameraStreamConfig]:
        resolved = self.resolve_camera_id(camera_id)
        if resolved is None or stream_type not in STREAM_TYPES:
            return None
        for stream in self.streams:
            if stream.camera_id == resolved and stream.stream_type == stream_type:
                return stream
        return None

    def camera_ids(self) -> list[str]:
        ordered: list[str] = []
        for stream in self.streams:
            if stream.camera_id not in ordered:
                ordered.append(stream.camera_id)
        return ordered


def load_camera_gateway_config(path: str | Path) -> CameraGatewayConfig:
    with open(path, "r", encoding="utf-8") as handle:
        raw = yaml.safe_load(handle) or {}

    cameras = raw.get("cameras") or {}
    if not isinstance(cameras, dict) or not cameras:
        raise ValueError("camera_streams.yaml must define a non-empty cameras map")

    streams: list[CameraStreamConfig] = []
    for camera_id, topics in cameras.items():
        if not isinstance(camera_id, str) or not camera_id.strip():
            raise ValueError("camera id must be a non-empty string")
        if not isinstance(topics, dict):
            raise ValueError(f"camera {camera_id} topics must be a mapping")
        for stream_type in STREAM_TYPES:
            topic = topics.get(stream_type)
            if topic is None:
                continue
            if not isinstance(topic, str) or not topic.strip():
                raise ValueError(
                    f"camera {camera_id}/{stream_type} topic must be a non-empty string"
                )
            streams.append(
                CameraStreamConfig(
                    camera_id=camera_id.strip(),
                    stream_type=stream_type,
                    topic=topic.strip(),
                )
            )

    if not streams:
        raise ValueError("camera_streams.yaml produced no streams")

    aliases_raw = raw.get("aliases") or {}
    if not isinstance(aliases_raw, dict):
        raise ValueError("aliases must be a mapping")
    aliases = {
        str(key).strip(): str(value).strip()
        for key, value in aliases_raw.items()
        if str(key).strip() and str(value).strip()
    }

    switch_config = _load_head_resolution_switch(
        raw,
        config_dir=Path(path).resolve().parent,
    )

    return CameraGatewayConfig(
        stale_after_sec=float(raw.get("stale_after_sec", 2.0)),
        jpeg_quality=int(raw.get("jpeg_quality", 80)),
        max_depth_mm=int(raw.get("max_depth_mm", 4000)),
        stream_fps=float(raw.get("stream_fps", 15)),
        streams=tuple(streams),
        aliases=aliases,
        head_resolution_switch=switch_config,
    )


def _load_head_resolution_switch(
    raw: dict[str, Any],
    *,
    config_dir: Path,
) -> Optional[HeadResolutionSwitchConfig]:
    switch_raw = raw.get("head_resolution_switch")
    if switch_raw is None:
        return None
    if not isinstance(switch_raw, dict):
        raise ValueError("head_resolution_switch must be a mapping")
    enabled = switch_raw.get("enabled", True)
    if not isinstance(enabled, bool):
        raise ValueError("head_resolution_switch.enabled must be a boolean")
    if not enabled:
        return HeadResolutionSwitchConfig(
            enabled=False,
            camera_id="head",
            node_name="/camera/head",
            parameter_timeout_sec=10.0,
            readiness_timeout_sec=10.0,
            startup_timeout_sec=60.0,
            profiles={},
            serial_no="",
            process_runtime_dir="",
            process_profile_configs={},
            process_stop_timeout_sec=8.0,
            process_startup_probe_sec=1.0,
        )

    camera_id = str(switch_raw.get("camera_id", "head")).strip()
    node_name = str(switch_raw.get("node_name", "/camera/head")).strip()
    if not camera_id:
        raise ValueError("head_resolution_switch.camera_id is required")
    if not node_name.startswith("/"):
        raise ValueError("head_resolution_switch.node_name must be absolute")

    profiles_raw = switch_raw.get("profiles")
    if not isinstance(profiles_raw, dict):
        raise ValueError("head_resolution_switch.profiles must be a mapping")
    profiles: dict[int, HeadResolutionProfile] = {}
    for resolution in HEAD_RESOLUTIONS:
        profile_raw = profiles_raw.get(resolution)
        if profile_raw is None:
            profile_raw = profiles_raw.get(str(resolution))
        if not isinstance(profile_raw, dict):
            raise ValueError(
                f"head_resolution_switch.profiles.{resolution} is required"
            )
        color_profile = str(profile_raw.get("color_profile") or "").strip()
        color_width, color_height, _ = _parse_video_profile(
            color_profile,
            f"head_resolution_switch.profiles.{resolution}.color_profile",
        )
        if color_height != resolution:
            raise ValueError(
                f"profile {resolution} color height must equal {resolution}"
            )
        enable_depth = profile_raw.get("enable_depth", resolution == 720)
        align_depth = profile_raw.get("align_depth", resolution == 720)
        if not isinstance(enable_depth, bool) or not isinstance(align_depth, bool):
            raise ValueError(
                f"profile {resolution} depth flags must be booleans"
            )
        if align_depth and not enable_depth:
            raise ValueError(
                f"profile {resolution} cannot align depth while depth is disabled"
            )
        depth_value = profile_raw.get("depth_profile")
        depth_profile = (
            None if depth_value is None else str(depth_value).strip() or None
        )
        if enable_depth:
            if depth_profile is None:
                raise ValueError(
                    f"profile {resolution} requires depth_profile"
                )
            depth_width, depth_height, _ = _parse_video_profile(
                depth_profile,
                f"head_resolution_switch.profiles.{resolution}.depth_profile",
            )
            if align_depth and (depth_width, depth_height) != (
                color_width,
                color_height,
            ):
                raise ValueError(
                    f"profile {resolution} aligned depth must match color size"
                )
        elif depth_profile is not None:
            _parse_video_profile(
                depth_profile,
                f"head_resolution_switch.profiles.{resolution}.depth_profile",
            )
        profiles[resolution] = HeadResolutionProfile(
            resolution=resolution,
            color_profile=color_profile,
            depth_profile=depth_profile,
            enable_depth=enable_depth,
            align_depth=align_depth,
            settle_frames=_positive_int(
                profile_raw.get("settle_frames", 15 if resolution == 1080 else 5),
                f"head_resolution_switch.profiles.{resolution}.settle_frames",
            ),
        )

    if profiles[1080].enable_depth or profiles[1080].align_depth:
        raise ValueError("1080 profile must be RGB-only")
    if not profiles[720].enable_depth or not profiles[720].align_depth:
        raise ValueError("720 profile must restore aligned RGB-D")

    serial_no = str(switch_raw.get("serial_no") or "").strip().lstrip("_")
    if not serial_no:
        raise ValueError("head_resolution_switch.serial_no is required")
    runtime_value = str(
        switch_raw.get("process_runtime_dir")
        or "/tmp/retail_nav_bridge/head_camera"
    ).strip()
    if not runtime_value:
        raise ValueError(
            "head_resolution_switch.process_runtime_dir is required"
        )
    runtime_dir = str(Path(runtime_value).expanduser().resolve())
    process_profiles_raw = switch_raw.get("process_profile_configs")
    if not isinstance(process_profiles_raw, dict):
        raise ValueError(
            "head_resolution_switch.process_profile_configs must be a mapping"
        )
    process_profile_configs: dict[int, str] = {}
    for resolution in HEAD_RESOLUTIONS:
        value = process_profiles_raw.get(resolution)
        if value is None:
            value = process_profiles_raw.get(str(resolution))
        if not isinstance(value, str) or not value.strip():
            raise ValueError(
                "head_resolution_switch.process_profile_configs."
                f"{resolution} is required"
            )
        path = Path(value.strip()).expanduser()
        if not path.is_absolute():
            path = config_dir / path
        path = path.resolve()
        if not path.is_file():
            raise ValueError(
                f"head {resolution} process profile does not exist: {path}"
            )
        process_profile_configs[resolution] = str(path)

    return HeadResolutionSwitchConfig(
        enabled=True,
        camera_id=camera_id,
        node_name=node_name,
        parameter_timeout_sec=_positive_number(
            switch_raw.get("parameter_timeout_sec", 10.0),
            "head_resolution_switch.parameter_timeout_sec",
        ),
        readiness_timeout_sec=_positive_number(
            switch_raw.get("readiness_timeout_sec", 12.0),
            "head_resolution_switch.readiness_timeout_sec",
        ),
        startup_timeout_sec=_positive_number(
            switch_raw.get("startup_timeout_sec", 60.0),
            "head_resolution_switch.startup_timeout_sec",
        ),
        profiles=profiles,
        serial_no=serial_no,
        process_runtime_dir=runtime_dir,
        process_profile_configs=process_profile_configs,
        process_stop_timeout_sec=_positive_number(
            switch_raw.get("process_stop_timeout_sec", 8.0),
            "head_resolution_switch.process_stop_timeout_sec",
        ),
        process_startup_probe_sec=_positive_number(
            switch_raw.get("process_startup_probe_sec", 1.0),
            "head_resolution_switch.process_startup_probe_sec",
        ),
    )


@dataclass
class FrameBufferEntry:
    jpeg: Optional[bytes]
    received_at: float
    width: int
    height: int
    encoding: str
    raw: Optional[bytes] = None
    step: Optional[int] = None
    is_bigendian: int = 0
    stamp_sec: int = 0
    stamp_nanosec: int = 0
    frame_id: str = ""
    generation: int = 0

    @property
    def stamp(self) -> tuple[int, int]:
        return self.stamp_sec, self.stamp_nanosec


@dataclass(frozen=True)
class RgbdPair:
    camera_id: str
    color: FrameBufferEntry
    depth: FrameBufferEntry

    @property
    def generation(self) -> int:
        return self.color.generation


class LatestFrameStore:
    """Thread-safe latest-JPEG store for subscribed camera streams."""

    def __init__(self) -> None:
        self._lock = threading.Lock()
        self._frames: dict[str, FrameBufferEntry] = {}

    def update(
        self,
        key: str,
        jpeg: bytes,
        *,
        width: int,
        height: int,
        encoding: str,
        raw: Optional[bytes] = None,
        step: Optional[int] = None,
        is_bigendian: int = 0,
        stamp_sec: int = 0,
        stamp_nanosec: int = 0,
        frame_id: str = "",
        generation: int = 0,
        received_at: Optional[float] = None,
    ) -> FrameBufferEntry:
        entry = FrameBufferEntry(
            jpeg=jpeg,
            received_at=time.monotonic() if received_at is None else received_at,
            width=width,
            height=height,
            encoding=encoding,
            raw=raw,
            step=step,
            is_bigendian=is_bigendian,
            stamp_sec=stamp_sec,
            stamp_nanosec=stamp_nanosec,
            frame_id=frame_id,
            generation=generation,
        )
        with self._lock:
            self._frames[key] = entry
        return entry

    def get(self, key: str) -> Optional[FrameBufferEntry]:
        with self._lock:
            return self._frames.get(key)

    def snapshot(self) -> dict[str, FrameBufferEntry]:
        with self._lock:
            return dict(self._frames)

    def clear_camera(self, camera_id: str) -> None:
        prefix = f"{camera_id}/"
        with self._lock:
            stale_keys = [key for key in self._frames if key.startswith(prefix)]
            for key in stale_keys:
                self._frames.pop(key, None)


class LatestRgbdPairStore:
    """按ROS时间戳配对并仅保留每台相机最新RGB-D。"""

    def __init__(self, queue_size: int = 4) -> None:
        self._lock = threading.Lock()
        self._queue_size = max(2, int(queue_size))
        self._buffers: dict[
            str, dict[str, dict[tuple[int, int], FrameBufferEntry]]
        ] = {}
        self._pairs: dict[str, RgbdPair] = {}

    def update(
        self,
        camera_id: str,
        stream_type: str,
        entry: FrameBufferEntry,
    ) -> Optional[RgbdPair]:
        if stream_type not in STREAM_TYPES:
            raise ValueError(f"invalid stream type: {stream_type}")
        stamp = entry.stamp
        with self._lock:
            streams = self._buffers.setdefault(
                camera_id,
                {"color": {}, "depth": {}},
            )
            streams[stream_type][stamp] = entry
            other_type = "depth" if stream_type == "color" else "color"
            other = streams[other_type].get(stamp)
            if other is not None and other.generation == entry.generation:
                color = entry if stream_type == "color" else other
                depth = entry if stream_type == "depth" else other
                pair = RgbdPair(camera_id, color, depth)
                self._pairs[camera_id] = pair
                streams["color"].pop(stamp, None)
                streams["depth"].pop(stamp, None)
            else:
                pair = None
            for buffer in streams.values():
                while len(buffer) > self._queue_size:
                    buffer.pop(next(iter(buffer)))
            return pair

    def get(self, camera_id: str) -> Optional[RgbdPair]:
        with self._lock:
            return self._pairs.get(camera_id)

    def clear_camera(self, camera_id: str) -> None:
        with self._lock:
            self._buffers.pop(camera_id, None)
            self._pairs.pop(camera_id, None)


def parse_head_resolution_payload(payload: Any) -> int:
    if not isinstance(payload, dict):
        raise ValueError("request body must be a JSON object")
    if set(payload) != {"resolution"}:
        raise ValueError("request must contain only resolution")
    value = payload["resolution"]
    if isinstance(value, bool):
        raise ValueError("resolution must be 720 or 1080")
    if isinstance(value, str):
        value = value.strip()
        if not value.isdigit():
            raise ValueError("resolution must be 720 or 1080")
        value = int(value)
    if not isinstance(value, int) or value not in HEAD_RESOLUTIONS:
        raise ValueError("resolution must be 720 or 1080")
    return value


def evaluate_camera_health(
    *,
    frame_ages_sec: dict[str, Optional[float]],
    stale_after_sec: float,
) -> str:
    """Aggregate stream freshness into STARTING / READY / ERROR."""
    if not frame_ages_sec:
        return "STARTING"

    ages = list(frame_ages_sec.values())
    if all(age is None for age in ages):
        return "STARTING"
    if any(age is not None and age <= stale_after_sec for age in ages):
        return "READY"
    return "ERROR"


def encode_sensor_image_to_jpeg(
    *,
    data: bytes,
    height: int,
    width: int,
    encoding: str,
    is_bigendian: int,
    step: int,
    jpeg_quality: int,
    max_depth_mm: int,
    stream_type: str,
) -> bytes:
    """Convert a sensor_msgs/Image payload into JPEG bytes."""
    if height <= 0 or width <= 0:
        raise ValueError("invalid image dimensions")

    encoding_l = encoding.lower()
    if stream_type == "depth" or encoding_l in ("16uc1", "mono16"):
        image = _decode_depth(
            data=data,
            height=height,
            width=width,
            is_bigendian=is_bigendian,
            step=step,
            max_depth_mm=max_depth_mm,
        )
    else:
        image = _decode_color(
            data=data,
            height=height,
            width=width,
            encoding=encoding_l,
            step=step,
        )

    ok, encoded = cv2.imencode(
        ".jpg",
        image,
        [int(cv2.IMWRITE_JPEG_QUALITY), int(jpeg_quality)],
    )
    if not ok:
        raise RuntimeError("cv2.imencode failed")
    return encoded.tobytes()


def _decode_color(
    *,
    data: bytes,
    height: int,
    width: int,
    encoding: str,
    step: int,
) -> np.ndarray:
    if encoding in ("rgb8", "bgr8"):
        channels = 3
    elif encoding in ("mono8", "8uc1"):
        channels = 1
    else:
        raise ValueError(f"unsupported color encoding: {encoding}")

    expected = step * height
    if len(data) < expected:
        raise ValueError("image data shorter than height*step")

    array = np.frombuffer(data, dtype=np.uint8, count=expected)
    if channels == 1:
        return array.reshape((height, step))[:, :width].copy()

    row = array.reshape((height, step))[:, : width * channels]
    image = row.reshape((height, width, channels))
    if encoding == "rgb8":
        return cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
    return image.copy()


def _decode_depth(
    *,
    data: bytes,
    height: int,
    width: int,
    is_bigendian: int,
    step: int,
    max_depth_mm: int,
) -> np.ndarray:
    depth = decode_depth_u16(
        data=data,
        height=height,
        width=width,
        is_bigendian=is_bigendian,
        step=step,
    )
    clipped = np.clip(depth, 0, max(1, max_depth_mm))
    normalized = (clipped.astype(np.float32) * (255.0 / max(1, max_depth_mm))).astype(
        np.uint8
    )
    return cv2.applyColorMap(normalized, cv2.COLORMAP_JET)


def decode_depth_u16(
    *,
    data: bytes,
    height: int,
    width: int,
    is_bigendian: int,
    step: int,
) -> np.ndarray:
    """Decode an aligned 16UC1 image into native-endian millimeter data."""
    dtype = np.dtype(">u2" if is_bigendian else "<u2")
    expected = step * height
    if len(data) < expected:
        raise ValueError("depth data shorter than height*step")

    if step < width * 2:
        raise ValueError("depth step smaller than width*2")
    flat = np.frombuffer(data, dtype=np.uint8, count=expected)
    row_bytes = np.ascontiguousarray(flat.reshape((height, step))[:, : width * 2])
    depth = row_bytes.view(dtype).reshape((height, width))
    return depth.astype(np.uint16, copy=True)


def build_rgbd_archive(
    pair: RgbdPair,
    *,
    jpeg_quality: int,
) -> bytes:
    """Build one request-time RGB JPEG + aligned uint16 depth package."""
    color = pair.color
    depth = pair.depth
    if color.raw is None or depth.raw is None:
        raise ValueError("RGB-D pair has no raw image payload")
    if color.stamp != depth.stamp:
        raise ValueError("RGB-D timestamps do not match")
    if (color.width, color.height) != (depth.width, depth.height):
        raise ValueError("aligned depth dimensions do not match RGB")
    if depth.encoding.lower() not in ("16uc1", "mono16"):
        raise ValueError(f"unsupported depth encoding: {depth.encoding}")

    rgb_jpeg = encode_sensor_image_to_jpeg(
        data=color.raw,
        height=color.height,
        width=color.width,
        encoding=color.encoding,
        is_bigendian=color.is_bigendian,
        step=int(color.step or color.width * 3),
        jpeg_quality=jpeg_quality,
        max_depth_mm=1,
        stream_type="color",
    )
    depth_mm = decode_depth_u16(
        data=depth.raw,
        height=depth.height,
        width=depth.width,
        is_bigendian=depth.is_bigendian,
        step=int(depth.step or depth.width * 2),
    )
    depth_buffer = io.BytesIO()
    np.save(depth_buffer, depth_mm, allow_pickle=False)
    metadata = {
        "schema_version": 1,
        "camera": pair.camera_id,
        "stamp_sec": color.stamp_sec,
        "stamp_nanosec": color.stamp_nanosec,
        "width": color.width,
        "height": color.height,
        "rgb": {
            "file": "rgb.jpg",
            "encoding": color.encoding,
            "frame_id": color.frame_id,
        },
        "depth": {
            "file": "depth_mm.npy",
            "encoding": depth.encoding,
            "dtype": "uint16",
            "unit": "millimeter",
            "aligned_to": "rgb",
            "frame_id": depth.frame_id,
        },
    }
    output = io.BytesIO()
    with zipfile.ZipFile(output, mode="w") as archive:
        archive.writestr(
            "rgb.jpg",
            rgb_jpeg,
            compress_type=zipfile.ZIP_STORED,
        )
        archive.writestr(
            "depth_mm.npy",
            depth_buffer.getvalue(),
            compress_type=zipfile.ZIP_DEFLATED,
            compresslevel=1,
        )
        archive.writestr(
            "meta.json",
            json.dumps(metadata, ensure_ascii=False, separators=(",", ":")),
            compress_type=zipfile.ZIP_DEFLATED,
            compresslevel=1,
        )
    return output.getvalue()


def parse_camera_query(path_with_query: str) -> dict[str, Any]:
    """Parse /camera/stream|snapshot query string."""
    parts = urlsplit(path_with_query)
    query = parse_qs(parts.query)
    camera = (query.get("camera") or [""])[0].strip()
    stream_type = (query.get("type") or ["color"])[0].strip().lower()
    response_format = (query.get("format") or [""])[0].strip().lower()
    return {
        "path": parts.path,
        "camera": camera,
        "type": stream_type,
        "format": response_format,
    }


def build_camera_list_payload(
    config: CameraGatewayConfig,
    frame_store: LatestFrameStore,
    *,
    now: Optional[float] = None,
) -> dict[str, Any]:
    now = time.monotonic() if now is None else now
    cameras = []
    for camera_id in config.camera_ids():
        streams_payload = []
        online = False
        for stream_type in STREAM_TYPES:
            stream = config.find_stream(camera_id, stream_type)
            if stream is None:
                continue
            entry = frame_store.get(stream.key)
            age = None if entry is None else max(0.0, now - entry.received_at)
            stream_online = entry is not None and age <= config.stale_after_sec
            online = online or stream_online
            item: dict[str, Any] = {
                "type": stream_type,
                "topic": stream.topic,
                "online": stream_online,
            }
            if entry is not None:
                item["width"] = entry.width
                item["height"] = entry.height
                item["encoding"] = entry.encoding
                item["age_sec"] = round(age or 0.0, 3)
            streams_payload.append(item)
        cameras.append(
            {
                "id": camera_id,
                "online": online,
                "streams": streams_payload,
            }
        )
    return {"cameras": cameras}
