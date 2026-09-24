from collections.abc import Iterable

from agent.capabilities.common import HealthResult, HealthStatus
from agent.capabilities.mocks.delay import pause_mock_processing

from .contract import (
    CameraStream,
    CapturedFrame,
    CaptureResult,
    DepthFormat,
    validate_capture_request,
)


class MockCameraCapability:
    def __init__(self) -> None:
        self.captures: list[str] = []
        self._frames = 0
        self.image_bytes: dict[str, bytes] = {}
        self.image_reads: list[str] = []

    def health(self) -> HealthResult:
        pause_mock_processing()
        return HealthResult(HealthStatus.READY)

    def capture(
        self,
        camera: str,
        streams: Iterable[CameraStream] = (CameraStream.COLOR,),
        *,
        depth_format: DepthFormat = DepthFormat.RAW,
    ) -> CaptureResult:
        pause_mock_processing()
        requested = validate_capture_request(camera, streams, depth_format)
        effective_depth_format = (
            depth_format if requested == (CameraStream.DEPTH,) else DepthFormat.RAW
        )
        self.captures.append(camera)
        self._frames += 1
        capture_id = f"capture-{self._frames}"
        base = f"/shared/frames/{capture_id}"
        width, height = (1280, 720) if camera == "head" else (640, 480)
        color = (
            CapturedFrame(f"{base}/rgb.jpg", "jpeg", width, height)
            if CameraStream.COLOR in requested
            else None
        )
        depth = (
            CapturedFrame(
                f"{base}/{'depth_mm.npy' if effective_depth_format is DepthFormat.RAW else 'depth.jpg'}",
                effective_depth_format.value,
                width,
                height,
                aligned=True,
            )
            if CameraStream.DEPTH in requested
            else None
        )
        intrinsics = None
        depth_unit = None
        if camera == "head" and color is not None and depth is not None:
            intrinsics = (
                (612.772339587192, 0.0, 641.544745101529),
                (0.0, 611.998287842651, 358.604256964916),
                (0.0, 0.0, 1.0),
            )
            depth_unit = "mm"
        return CaptureResult(
            capture_id,
            camera,
            len(requested) > 1,
            color,
            depth,
            intrinsics,
            depth_unit,
        )

    def read_frame_bytes(self, frame: CapturedFrame) -> bytes:
        if frame.format == "raw":
            import io

            import numpy as np

            output = io.BytesIO()
            np.save(output, np.ones((frame.height, frame.width), dtype=np.uint16), allow_pickle=False)
            return output.getvalue()
        return self.read_image_bytes(frame.path)

    def read_image_bytes(self, path: str) -> bytes:
        self.image_reads.append(path)
        return self.image_bytes.get(path, b"mock-jpeg")
