from collections.abc import Iterable
from pathlib import Path
from typing import Any

from agent.capabilities.common import CapabilityError, HealthResult, HealthStatus
from agent.capabilities.http import HttpCapabilityClient

from .contract import (
    CameraStream,
    CapturedFrame,
    CaptureResult,
    DepthFormat,
    validate_capture_request,
)


class HttpCameraCapability:
    def __init__(self, client: HttpCapabilityClient):
        self.client = client

    def health(self) -> HealthResult:
        payload = self.client.get("/camera/health")
        if not isinstance(payload, dict) or payload.get("ok") is not True:
            raise CapabilityError(
                str(payload.get("error_code", "CAMERA_NOT_READY"))
                if isinstance(payload, dict)
                else "CAMERA_NOT_READY",
                str(payload.get("message", "camera is not ready"))
                if isinstance(payload, dict)
                else "camera health response must be an object",
            )
        return HealthResult(HealthStatus(str(payload.get("status", "READY"))))

    def capture(
        self,
        camera: str,
        streams: Iterable[CameraStream] = (CameraStream.COLOR,),
        *,
        depth_format: DepthFormat = DepthFormat.RAW,
    ) -> CaptureResult:
        requested = validate_capture_request(camera, streams, depth_format)
        effective_depth_format = (
            depth_format if requested == (CameraStream.DEPTH,) else DepthFormat.RAW
        )
        if camera == "head" and requested == (CameraStream.COLOR, CameraStream.DEPTH):
            payload = self.client.get("/camera/rgbd", params={"camera": camera})
            return _rgbd_result(payload, camera)
        params = {"camera": camera}
        if requested != (CameraStream.COLOR,):
            params["streams"] = ",".join(stream.value for stream in requested)
        if effective_depth_format is not DepthFormat.RAW:
            params["format"] = effective_depth_format.value

        payload = self.client.get("/camera/capture", params=params)
        if not isinstance(payload, dict):
            raise ValueError("camera capture response must be an object")
        if payload.get("ok") is not True:
            raise CapabilityError(
                str(payload.get("error_code", "CAPTURE_FAILED")),
                str(payload.get("message", "camera capture failed")),
            )
        return _capture_result(payload, camera, requested, effective_depth_format)

    def read_frame_bytes(self, frame: CapturedFrame) -> bytes:
        return self.read_image_bytes(frame.path)

    def read_image_bytes(self, path: str) -> bytes:
        return Path(path).read_bytes()


def _capture_result(
    payload: dict[str, Any],
    requested_camera: str,
    requested: tuple[CameraStream, ...],
    depth_format: DepthFormat,
) -> CaptureResult:
    if payload.get("camera") != requested_camera:
        raise ValueError("camera capture response camera does not match request")

    wants_color = CameraStream.COLOR in requested
    wants_depth = CameraStream.DEPTH in requested
    color = _frame(payload.get("color"), "color") if wants_color else None
    depth = _frame(payload.get("depth"), "depth") if wants_depth else None
    if (not wants_color and payload.get("color") is not None) or (
        not wants_depth and payload.get("depth") is not None
    ):
        raise ValueError("camera capture response contains an unrequested stream")
    if (wants_color and color is None) or (wants_depth and depth is None):
        raise ValueError("camera capture response is missing a requested stream")
    if color is not None and color.format != "jpeg":
        raise ValueError("camera color capture must use jpeg format")
    if depth is not None and depth.aligned is not True:
        raise CapabilityError("DEPTH_NOT_ALIGNED", "camera depth capture is not aligned")
    if depth is not None and depth.format != depth_format.value:
        raise ValueError("camera depth capture format does not match request")

    same_shot = payload.get("same_shot")
    if not isinstance(same_shot, bool):
        raise ValueError("camera capture response same_shot must be boolean")
    if wants_color and wants_depth and same_shot is not True:
        raise CapabilityError("CAPTURE_FAILED", "color and depth were not captured in the same shot")
    capture_id = payload.get("capture_id")
    if not isinstance(capture_id, str) or not capture_id:
        raise ValueError("camera capture response requires capture_id")

    expected_prefix = f"/shared/frames/{capture_id}/"
    for frame in (color, depth):
        if frame is not None and not frame.path.startswith(expected_prefix):
            raise ValueError("camera frame path does not belong to capture_id")
    return CaptureResult(capture_id, requested_camera, same_shot, color, depth)


def _rgbd_result(payload: Any, requested_camera: str) -> CaptureResult:
    if not isinstance(payload, dict):
        raise ValueError("camera rgbd response must be an object")
    if payload.get("ok") is not True:
        raise CapabilityError(
            str(payload.get("error_code", "CAPTURE_FAILED")),
            str(payload.get("message", "camera rgbd capture failed")),
        )
    if payload.get("camera") != requested_camera:
        raise ValueError("camera rgbd response camera does not match request")
    if payload.get("same_shot") is not True:
        raise CapabilityError("CAPTURE_FAILED", "RGB and depth are not from the same shot")
    if payload.get("t_unit") != "mm":
        raise ValueError("camera rgbd depth unit must be mm")

    capture_id = payload.get("capture_id")
    if not isinstance(capture_id, str) or not capture_id:
        raise ValueError("camera rgbd response requires capture_id")
    intrinsics = payload.get("color_intrinsics")
    if not isinstance(intrinsics, dict):
        raise ValueError("camera rgbd response requires color_intrinsics")
    width, height = intrinsics.get("width"), intrinsics.get("height")
    if not isinstance(width, int) or isinstance(width, bool) or width <= 0:
        raise ValueError("camera rgbd color_intrinsics.width is invalid")
    if not isinstance(height, int) or isinstance(height, bool) or height <= 0:
        raise ValueError("camera rgbd color_intrinsics.height is invalid")
    matrix = _matrix3(intrinsics.get("camera_matrix"))

    rgb, depth = payload.get("rgb"), payload.get("depth")
    if not isinstance(rgb, str) or not rgb:
        raise ValueError("camera rgbd response requires rgb path")
    if not isinstance(depth, str) or not depth:
        raise ValueError("camera rgbd response requires depth path")
    expected_prefix = f"/shared/frames/{capture_id}/"
    if not rgb.startswith(expected_prefix) or not depth.startswith(expected_prefix):
        raise ValueError("camera rgbd frame path does not belong to capture_id")
    return CaptureResult(
        capture_id,
        requested_camera,
        True,
        CapturedFrame(rgb, "jpeg", width, height),
        CapturedFrame(depth, "raw", width, height, aligned=True),
        matrix,
        "mm",
    )


def _matrix3(value: Any) -> tuple[tuple[float, ...], ...]:
    if not isinstance(value, list) or len(value) != 3:
        raise ValueError("camera_matrix must be a 3x3 numeric matrix")
    rows: list[tuple[float, ...]] = []
    for row in value:
        if not isinstance(row, list) or len(row) != 3:
            raise ValueError("camera_matrix must be a 3x3 numeric matrix")
        try:
            values = tuple(float(item) for item in row)
        except (TypeError, ValueError) as exc:
            raise ValueError("camera_matrix must be a 3x3 numeric matrix") from exc
        rows.append(values)
    if rows[0][0] <= 0.0 or rows[1][1] <= 0.0:
        raise ValueError("camera intrinsics focal lengths must be positive")
    return tuple(rows)


def _frame(value: Any, stream: str) -> CapturedFrame | None:
    if value is None:
        return None
    if not isinstance(value, dict):
        raise ValueError(f"camera {stream} frame must be an object")
    try:
        path = value["path"]
        frame_format = value["format"]
        width = value["width"]
        height = value["height"]
    except KeyError as exc:
        raise ValueError(f"camera {stream} frame is incomplete") from exc
    if not isinstance(path, str) or not path:
        raise ValueError(f"camera {stream} frame path is invalid")
    if not isinstance(frame_format, str) or not frame_format:
        raise ValueError(f"camera {stream} frame format is invalid")
    if not isinstance(width, int) or isinstance(width, bool) or width <= 0:
        raise ValueError(f"camera {stream} frame width is invalid")
    if not isinstance(height, int) or isinstance(height, bool) or height <= 0:
        raise ValueError(f"camera {stream} frame height is invalid")
    aligned = value.get("aligned")
    if aligned is not None and not isinstance(aligned, bool):
        raise ValueError(f"camera {stream} frame aligned must be boolean")
    return CapturedFrame(path, frame_format, width, height, aligned)
