"""珞石厂家后端。ros / sros 懒加载，单测只 import mock。"""

from __future__ import annotations

from typing import Any, Dict

from navigation.rokae_runtime.base import (
    DEFAULT_STATIONS,
    LOCATION_STATE_RUNNING,
    RokaeVendor,
    load_stations,
    resolve_station_id,
)
from navigation.rokae_runtime.mock import MockRokaeBackend

__all__ = [
    "DEFAULT_STATIONS",
    "LOCATION_STATE_RUNNING",
    "MockRokaeBackend",
    "RokaeRosClient",
    "RokaeSrosClient",
    "RokaeVendor",
    "build_backend",
    "load_stations",
    "resolve_station_id",
]


def build_backend(config: Dict[str, Any]) -> RokaeVendor:
    # sros = SDK 直连底盘（含 sim）；ros = 车上 ROS action，中免这条链不用。
    rokae = config.get("rokae") or {}
    name = str(rokae.get("backend") or "ros").strip().lower()
    if name == "mock":
        return MockRokaeBackend(config)
    if name == "sros":
        from navigation.rokae_runtime.sros import RokaeSrosClient

        return RokaeSrosClient(config)
    if name == "ros":
        from navigation.rokae_runtime.ros import RokaeRosClient

        return RokaeRosClient(config)
    raise ValueError(f"unknown rokae backend: {name}")


def __getattr__(name: str):
    if name == "RokaeRosClient":
        from navigation.rokae_runtime.ros import RokaeRosClient

        return RokaeRosClient
    if name == "RokaeSrosClient":
        from navigation.rokae_runtime.sros import RokaeSrosClient

        return RokaeSrosClient
    raise AttributeError(f"module {__name__!r} has no attribute {name!r}")
