from typing import Any

from agent.capabilities.common import ActionResult, ActionStatus, HealthResult, HealthStatus
from agent.capabilities.http import HttpCapabilityClient

from .contract import HEAD_CAMERA, CameraTransform

# /pose/health may activate the right gripper; allow time for that hardware step.
HEALTH_TIMEOUT_S = 60.0
TRANSFORM_TIMEOUT_S = 15.0
# Synchronous motion; this is a client wait, not a robot stop timeout.
MOTION_TIMEOUT_S = 1800.0


class HttpBodyPoseCapability:
    def __init__(self, client: HttpCapabilityClient):
        self.client = client

    def health(self) -> HealthResult:
        """GET /pose/health. May activate the right gripper; not a read-only probe."""
        payload = self.client.get("/pose/health", timeout=HEALTH_TIMEOUT_S)
        return HealthResult(HealthStatus(payload["status"]))

    def prepare(
        self,
        pose_type: str,
        level: str | None = None,
        *,
        idempotency_key: str | None = None,
    ) -> ActionResult:
        request: dict[str, Any] = {"pose_type": pose_type}
        if level:
            request["level"] = level
        payload = self.client.post_action(
            "/pose/prepare",
            request,
            idempotency_key=idempotency_key,
            timeout=MOTION_TIMEOUT_S,
        )
        return ActionResult(ActionStatus(payload["status"]))

    def camera_transform(self, camera: str = HEAD_CAMERA) -> CameraTransform:
        if camera != HEAD_CAMERA:
            raise ValueError("only head camera extrinsics are supported")
        payload = self.client.get(
            "/pose/camera_transform",
            params={"camera": camera},
            timeout=TRANSFORM_TIMEOUT_S,
        )
        return CameraTransform.from_payload(payload, expected_camera=camera)
