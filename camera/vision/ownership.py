"""Tianji-like ownership: who may open cameras vs who only consumes Topics."""

from __future__ import annotations

import sys
from typing import Iterable

# Helios vision units mapped from kim/dog_device SMT camera_bringup / RosTopicVideoAdapter.
SOURCE_RUNTIME_UNITS = {
    "head": "vision-rokae-preview-head-rgbd.service",
    "hand_left": "vision-rokae-preview-left-color.service",
    "hand_right": "vision-rokae-preview-right-rgbd.service",
    "media": "vision-rokae-preview-media.service",
    "owner": "vision-rokae-preview-owner.service",
    "synced": "vision-rokae-preview-synced-rgbd.service",
}

# Modules that acquire USB / V4L2 / vendor SDK devices. Media with source=ros must
# never import these for frame acquisition (SMT: platform video never opens USB).
_DEVICE_ACQUISITION_MODULES = (
    "vision.rokae_runtime.capture",
    "vision.rokae_runtime.devices",
    "vision.rokae_runtime.owner",
    "pyorbbecsdk",
    "pyrealsense2",
)


def loaded_device_acquisition_modules(modules: Iterable[str] | None = None) -> tuple[str, ...]:
    names = sys.modules if modules is None else modules
    hit = []
    for name in _DEVICE_ACQUISITION_MODULES:
        if name in names:
            hit.append(name)
    return tuple(hit)


def ensure_ros_media_subscriber_only() -> None:
    """Fail closed if this process already loaded device-acquisition modules.

    Call from Media PushRuntime when media.push.source=ros. Matches SMT
    RosTopicVideoAdapter: own FFmpeg consumers, never camera devices.
    """
    loaded = loaded_device_acquisition_modules()
    if loaded:
        raise RuntimeError(
            "media.push.source=ros forbids device acquisition modules in the Media "
            f"process (Tianji camera_bringup ownership); loaded={list(loaded)}"
        )
