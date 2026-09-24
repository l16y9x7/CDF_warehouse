"""珞石厂家后端合同：ROS / sros_sdk_py / mock 都吐同一份 state dict。"""

from __future__ import annotations

from typing import Any, Dict, Optional, Protocol

LOCATION_STATE_RUNNING = 3  # MATRIX：已定位、可导航。不是 3 时厂家会报 321005。

DEFAULT_STATIONS = [
    {"id": "1", "name": "A", "aliases": ["A", "s1-A"], "x": 0.2887, "y": -0.8487, "yaw": 0.0},
    {"id": "2", "name": "B", "aliases": ["B", "s2-B"], "x": 0.5010, "y": -0.8487, "yaw": 0.0},
]


class RokaeVendor(Protocol):
    def latest_state(self) -> Optional[Dict[str, Any]]: ...

    def latest_battery(self) -> Optional[Dict[str, Any]]: ...

    def move_to_station(self, station_id: int, timeout_sec: float) -> Dict[str, Any]: ...

    def close(self) -> None: ...


def resolve_station_id(station_id: str, stations: list[Dict[str, Any]]) -> Optional[int]:
    # "A" / "1" / "s1-A" 都映射到 MATRIX 站号 1。
    raw = str(station_id or "").strip()
    if not raw:
        return None
    key = raw.upper()
    for row in stations:
        aliases = {str(row.get("id") or "").upper(), str(row.get("name") or "").upper()}
        aliases.discard("")
        for alias in row.get("aliases") or []:
            if str(alias).strip():
                aliases.add(str(alias).strip().upper())
        if key in aliases:
            return int(row["id"])
    return None


def load_stations(rokae: Dict[str, Any]) -> list[Dict[str, Any]]:
    stations = rokae.get("stations")
    if isinstance(stations, list) and stations:
        return list(stations)
    return list(DEFAULT_STATIONS)
