"""Stable capability contracts, HTTP adapters, and deterministic mocks."""

from agent.capabilities.camera import CameraCapability
from agent.capabilities.estimation import EstimationCapability
from agent.capabilities.hand import HandCapability
from agent.capabilities.manipulation import ManipulationCapability
from agent.capabilities.navigation import NavigationCapability
from agent.capabilities.perception import PerceptionCapability
from agent.capabilities.pose import BodyPoseCapability
from agent.capabilities.vla import VlaCapability

__all__ = [
    "BodyPoseCapability",
    "CameraCapability",
    "EstimationCapability",
    "HandCapability",
    "ManipulationCapability",
    "NavigationCapability",
    "PerceptionCapability",
    "VlaCapability",
]
