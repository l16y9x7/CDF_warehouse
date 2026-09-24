from collections.abc import Iterable
from dataclasses import dataclass
from enum import Enum
from typing import Protocol

from agent.capabilities.common import HealthResult

Matrix3 = tuple[tuple[float, ...], ...]


class CameraStream(str, Enum):
    COLOR = "color"
    DEPTH = "depth"


class DepthFormat(str, Enum):
    RAW = "raw"
    PREVIEW = "preview"


CAMERA_IDS = frozenset({"head", "left_wrist", "right_wrist"})


@dataclass(frozen=True)
class CapturedFrame:
    path: str
    format: str
    width: int
    height: int
    aligned: bool | None = None


@dataclass(frozen=True)
class CaptureResult:
    capture_id: str
    camera: str
    same_shot: bool
    color: CapturedFrame | None
    depth: CapturedFrame | None
    color_intrinsics: Matrix3 | None = None
    depth_unit: str | None = None


class CameraCapability(Protocol):
    def health(self) -> HealthResult: ...

    def capture(
        self,
        camera: str,
        streams: Iterable[CameraStream] = (CameraStream.COLOR,),
        *,
        depth_format: DepthFormat = DepthFormat.RAW,
    ) -> CaptureResult: ...

    def read_frame_bytes(self, frame: CapturedFrame) -> bytes: ...
    def read_image_bytes(self, path: str) -> bytes: ...


def validate_capture_request(
    camera: str,
    streams: Iterable[CameraStream],
    depth_format: DepthFormat,
) -> tuple[CameraStream, ...]:
    if camera not in CAMERA_IDS:
        raise ValueError(f"unsupported camera id: {camera}")
    try:
        items = tuple(CameraStream(item) for item in streams)
    except (TypeError, ValueError) as exc:
        raise ValueError("streams must contain only color and/or depth") from exc
    if not items:
        raise ValueError("streams must not be empty")
    normalized = frozenset(items)
    if len(items) != len(normalized):
        raise ValueError("streams must not contain duplicates")
    if not isinstance(depth_format, DepthFormat):
        try:
            depth_format = DepthFormat(depth_format)
        except (TypeError, ValueError) as exc:
            raise ValueError("depth format must be raw or preview") from exc
    return tuple(stream for stream in CameraStream if stream in normalized)
