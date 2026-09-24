from collections.abc import Iterable
from typing import Any

from agent.contracts import ExecutionContext

from .contract import CameraCapability, CameraStream, CapturedFrame, CaptureResult, DepthFormat


def capture_with_event(
    context: ExecutionContext,
    capability: CameraCapability,
    camera: str,
    streams: Iterable[CameraStream] = (CameraStream.COLOR,),
    *,
    depth_format: DepthFormat = DepthFormat.RAW,
    skill: str | None = None,
) -> CaptureResult:
    """Capture frames and expose their metadata to the active execution."""
    result = capability.capture(camera, streams, depth_format=depth_format)
    event: dict[str, Any] = {
        "capture_id": result.capture_id,
        "camera": result.camera,
        "same_shot": result.same_shot,
        "color": _frame_payload(result.color),
        "depth": _frame_payload(result.depth),
        "depth_unit": result.depth_unit,
        "color_intrinsics": (
            [list(row) for row in result.color_intrinsics]
            if result.color_intrinsics is not None
            else None
        ),
    }
    if skill is not None:
        event["skill"] = skill
    context.emit("camera.captured", **event)
    return result


def _frame_payload(frame: CapturedFrame | None) -> dict[str, Any] | None:
    if frame is None:
        return None
    payload: dict[str, Any] = {
        "path": frame.path,
        "format": frame.format,
        "width": frame.width,
        "height": frame.height,
    }
    if frame.aligned is not None:
        payload["aligned"] = frame.aligned
    return payload
