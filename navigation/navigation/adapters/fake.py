"""联调用 Fake Adapter，不碰硬件。"""

from __future__ import annotations

import threading
from typing import Any, Dict


class FakeNavigationAdapter:
    def __init__(self, *, map_id: str = "fake-map") -> None:
        self._lock = threading.Lock()
        self._ready = True
        self._map_id = map_id
        self._pose = {"x": 0.17, "y": 0.31, "yaw": 0.0}
        self._station_id = ""
        self._nav_state = "IDLE"
        self._stations = {
            "station.home": {"x": 0.0, "y": 0.0, "yaw": 0.0},
            "start": {"x": -0.33, "y": -1.0, "yaw": -3.1},
        }

    def set_ready(self, ready: bool) -> None:
        with self._lock:
            self._ready = bool(ready)

    def set_pose(self, x: float, y: float, yaw: float) -> None:
        with self._lock:
            self._pose = {"x": float(x), "y": float(y), "yaw": float(yaw)}

    def ready(self) -> bool:
        with self._lock:
            return self._ready

    def known_station(self, station_id: str) -> bool:
        with self._lock:
            return str(station_id) in self._stations

    def snapshot(self) -> Dict[str, Any]:
        with self._lock:
            out: Dict[str, Any] = {
                "ready": self._ready,
                "nav_state": self._nav_state,
                "station_id": self._station_id,
                "position": dict(self._pose),
                "map_id": self._map_id,
                "stations": [
                    {"station_id": sid, **coords}
                    for sid, coords in self._stations.items()
                ],
            }
            battery = getattr(self, "_battery", None)
            chassis = getattr(self, "_chassis_status", None)
            if isinstance(battery, dict) and battery:
                out["battery"] = dict(battery)
            if isinstance(chassis, dict) and chassis:
                out["chassis_status"] = dict(chassis)
            occ = self.occupancy(self._map_id)
            from navigation.occupancy import occupancy_revision

            revision = occupancy_revision(occ)
            if revision:
                out["occupancy_revision"] = revision
            return out

    def goto(
        self,
        *,
        station_id: str,
        idempotency_key: str,
        timeout_sec: float,
    ) -> Dict[str, Any]:
        del idempotency_key, timeout_sec
        with self._lock:
            if not self._ready:
                return {
                    "terminal_state": "REJECTED",
                    "error_code": "NAVIGATION_NOT_READY",
                    "message": "fake adapter not ready",
                }
            if station_id not in self._stations:
                return {
                    "terminal_state": "REJECTED",
                    "error_code": "NAVIGATION_UNKNOWN_STATION",
                    "message": f"unknown station_id={station_id}",
                }
            coords = self._stations[station_id]
            self._nav_state = "NAVIGATING"
            self._station_id = station_id
            self._pose = {
                "x": float(coords["x"]),
                "y": float(coords["y"]),
                "yaw": float(coords["yaw"]),
            }
            self._nav_state = "IDLE"
        return {
            "terminal_state": "SUCCEEDED",
            "evidence": {"arrived": True, "station_id": station_id},
        }

    def stop(self) -> Dict[str, Any]:
        with self._lock:
            self._nav_state = "IDLE"
        return {"accepted": True, "terminal_state": "CANCELLED"}

    def load_map(self, map_id: str) -> Dict[str, Any]:
        with self._lock:
            self._map_id = str(map_id)
        return {"accepted": True, "map_id": str(map_id)}

    def occupancy(self, map_id: str = "") -> Dict[str, Any]:
        del map_id
        return {
            "width": 2,
            "height": 2,
            "resolution": 0.05,
            "origin": {"x": 0.0, "y": 0.0},
            "data": [0, 100, -1, 0],
        }

    def apply_map(self, map_id: str, stations: list[Dict[str, Any]]) -> Dict[str, Any]:
        parsed: Dict[str, Dict[str, float]] = {}
        for row in stations:
            sid = str(row.get("station_id") or "").strip()
            if not sid:
                continue
            parsed[sid] = {
                "x": float(row.get("x") or 0.0),
                "y": float(row.get("y") or 0.0),
                "yaw": float(row.get("yaw") or 0.0),
            }
        if not parsed:
            return {"accepted": False, "error_code": "NAVIGATION_INVALID_REQUEST", "message": "stations is empty"}
        with self._lock:
            self._map_id = str(map_id)
            self._stations = parsed
        return {"accepted": True, "map_id": str(map_id)}
