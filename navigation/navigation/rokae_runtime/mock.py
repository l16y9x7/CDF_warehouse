"""单测用 mock 后端，不碰 ROS / SDK。"""

from __future__ import annotations

from typing import Any, Dict, Optional


class MockRokaeBackend:
    def __init__(self, _config: Any = None) -> None:
        self.state: Optional[Dict[str, Any]] = {
            "location_state": 3,
            "x": 0.2887,
            "y": -0.8487,
            "yaw": 0.0,
            "map_name": "AB_0619",
            "current_station_id": 1,
            "estop_active": False,
            "executing_movement_task": False,
            "linear_velocity_x": 0.0,
            "linear_velocity_y": 0.0,
            "angular_velocity": 0.0,
        }
        self.battery: Optional[Dict[str, Any]] = {
            "capacity_percent": 80.0,
            "temperature": 31.0,
            "voltage": 50.4,
            "charging": False,
            "cycle": 12,
        }
        self.server_ok = True
        self.move_ok = True
        self.rejected = False
        self.timed_out = False
        self.move_error = ""
        self.calls: list[int] = []

    def latest_state(self) -> Optional[Dict[str, Any]]:
        return dict(self.state) if self.state else None

    def latest_battery(self) -> Optional[Dict[str, Any]]:
        return dict(self.battery) if self.battery else None

    def move_to_station(self, station_id: int, timeout_sec: float) -> Dict[str, Any]:
        del timeout_sec
        self.calls.append(int(station_id))
        if not self.server_ok:
            return {"accepted": False, "unavailable": True, "message": "unavailable"}
        if self.rejected:
            return {"accepted": False, "message": "vendor rejected"}
        if self.timed_out:
            return {"accepted": True, "timed_out": True, "message": "timeout"}
        if self.move_ok:
            return {"accepted": True, "result": True, "error_code": ""}
        return {"accepted": True, "result": False, "error_code": self.move_error or "VENDOR"}

    def close(self) -> None:
        return None
