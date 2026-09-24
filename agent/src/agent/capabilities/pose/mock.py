from agent.capabilities.common import ActionResult, ActionStatus, HealthResult, HealthStatus
from agent.capabilities.mocks.delay import pause_mock_processing

from .contract import HEAD_CAMERA, CameraTransform

IDENTITY_T = (
    (-0.047653753369, -0.428149456867, 0.902450642625, 0.113767949307),
    (-0.998816369947, 0.011610119563, -0.04723414285, 0.018099569521),
    (0.009745712746, -0.903633359117, -0.428195952077, 1.45137337738),
    (0.0, 0.0, 0.0, 1.0),
)


class MockBodyPoseCapability:
    def __init__(self) -> None:
        self.calls: list[tuple[str, str | None]] = []
        self.transform_calls: list[str] = []
        self.history: list[str] = []

    def health(self) -> HealthResult:
        pause_mock_processing()
        return HealthResult(HealthStatus.READY)

    def prepare(
        self,
        pose_type: str,
        level: str | None = None,
        *,
        idempotency_key: str | None = None,
    ) -> ActionResult:
        pause_mock_processing()
        self.calls.append((pose_type, level))
        self.history.append(f"prepare:{pose_type}:{level or ''}")
        return ActionResult(ActionStatus.SUCCEEDED)

    def camera_transform(self, camera: str = HEAD_CAMERA) -> CameraTransform:
        pause_mock_processing()
        if camera != HEAD_CAMERA:
            raise ValueError("only head camera extrinsics are supported")
        self.transform_calls.append(camera)
        self.history.append(f"camera_transform:{camera}")
        return CameraTransform(
            IDENTITY_T,
            "m",
            "head",
            "head_camera_color_optical_frame",
            "chassis_link",
        )
