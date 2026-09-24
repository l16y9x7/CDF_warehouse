"""Adapter package."""

from navigation.tianji_runtime.adapters.base import (
    AdapterResult,
    HealthInfo,
    NavAdapter,
    NavGoal,
    Pose2D,
)
from navigation.tianji_runtime.adapters.mock import MockNavAdapter

__all__ = [
    "AdapterResult",
    "HealthInfo",
    "MockNavAdapter",
    "NavAdapter",
    "NavGoal",
    "Pose2D",
    "S2NavAdapter",
]


def __getattr__(name: str):
    # Lazy import: S2 needs ROS vendor msgs; keep mock/pytest importable without them.
    if name == "S2NavAdapter":
        from navigation.tianji_runtime.adapters.s2 import S2NavAdapter

        return S2NavAdapter
    raise AttributeError(f"module {__name__!r} has no attribute {name!r}")
