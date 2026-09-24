from .contract import (
    HEAD_CAMERA,
    BodyPoseCapability,
    CameraTransform,
)
from .http_adapter import HttpBodyPoseCapability
from .mock import MockBodyPoseCapability

# Short aliases retained for existing skills.
PoseControl = BodyPoseCapability
MockPoseControl = MockBodyPoseCapability

__all__ = [
    "BodyPoseCapability",
    "CameraTransform",
    "HEAD_CAMERA",
    "HttpBodyPoseCapability",
    "MockBodyPoseCapability",
    "MockPoseControl",
    "PoseControl",
]
