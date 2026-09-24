"""假 SrpClient：方法名和真 SDK 一样，不连 5001。"""

from __future__ import annotations

import threading
import time
from types import SimpleNamespace
from typing import Any, Dict, Optional

from navigation.rokae_runtime.base import DEFAULT_STATIONS, LOCATION_STATE_RUNNING


class SimSrpClient:
    def __init__(self, *, stations: Optional[list[Dict[str, Any]]] = None, travel_sec: float = 0.4) -> None:
        self.travel_sec = max(0.0, float(travel_sec))
        self._stations_mm = _stations_mm(stations or DEFAULT_STATIONS)
        x, y, yaw = self._stations_mm.get(1, (288.7, -848.7, 0.0))
        self._lock = threading.Lock()
        self._connected = False
        self._system = SimpleNamespace(
            sys_state=2,
            location_state=LOCATION_STATE_RUNNING,
            location_pose=SimpleNamespace(x=x, y=y, z=0, roll=0, pitch=0, yaw=yaw, confidence=100),
            emergency_state=1,
            emergency_source=0,
            run_state=1,
            operation_state=1,
            scheduling_mode=2,
            fleet_mode=1,
            location_type=1,
            load_state=1,
            fresh_state=2,
            control_mutex_lock_state=2,
            manual_button_state=1,
            oba_enable_state=1,
            speed_level=1,
            new_movement_task_state=1,
            current_pause_source=0,
            effective_speed_limit_source=0,
            last_error_code=0,
            manual_control_oba_state=False,
            faults=[],
            map_name="AB_0619",
            station_no=1,
            mc_state=SimpleNamespace(state=2, v_x=0, v_y=0, w=0, path_no=0),
            movement_state=SimpleNamespace(
                no=0, state="FINISHED", result="OK", failed_code="", finished=True, ok=True
            ),
        )
        self._hardware = SimpleNamespace(
            battery_percentage=80,
            battery_temperature=31,
            battery_voltage=50400,
            battery_current=0,
            battery_state=0,
            battery_use_cycles=12,
            cpu_usage=12,
            memory_usage=34,
            disk_usage=20,
            remain_disk_space=4096,
            cpu_temperature=41,
            box_temperature=36,
            wifi_strength=70,
            wifi_state=3,
            laser_state=2,
            power_state=1,
            brake_sw_state=1,
            hardware_state=2,
            hardware_error_code=0,
            ip_address="127.0.0.1",
            src_hardware_state=SimpleNamespace(
                m1_status_code=0,
                m2_status_code=0,
                m3_status_code=0,
                m4_status_code=0,
                src_state=0,
                src_state_error_reason=0,
                total_mileage=12,
                total_power_cycle=3,
                total_poweron_time=3600,
            ),
            devices=[
                SimpleNamespace(
                    id=121,
                    name="IMU",
                    state=1,
                    error_code=0,
                    model_no="CH040-SR",
                    version_no="1.0",
                    interface_name="CAN1",
                ),
                SimpleNamespace(
                    id=231,
                    name="SRC",
                    state=1,
                    error_code=0,
                    model_no="VC400-SR",
                    version_no="SRTOS",
                    interface_name="ETH1",
                ),
                SimpleNamespace(
                    id=626,
                    name="depth_camera",
                    state=1,
                    error_code=0,
                    model_no="D435",
                    version_no="1.0",
                    interface_name="USB1",
                ),
                SimpleNamespace(
                    id=711,
                    name="battery",
                    state=1,
                    error_code=0,
                    model_no="BAT-48V",
                    version_no="1.0",
                    interface_name="CAN2",
                ),
            ],
        )

    def connect(self, _config: Any = None) -> bool:
        self._connected = True
        return True

    def is_connected(self) -> bool:
        return self._connected

    def disconnect(self) -> None:
        self._connected = False

    def set_stations(self, stations: list[Dict[str, Any]]) -> None:
        with self._lock:
            self._stations_mm = _stations_mm(stations)

    def fetch_system_state(self) -> Any:
        with self._lock:
            return self._system

    def fetch_hardware_state(self) -> Any:
        return self._hardware

    def get_current_system_state(self) -> Any:
        with self._lock:
            return self._system

    def get_current_hardware_state(self) -> Any:
        return self._hardware

    def get_info(self) -> Any:
        return SimpleNamespace(
            serial_no="SIM",
            nickname="sros-sim",
            vehicle_type="Oasis-sim",
            vehicle_serial_no="SIM-000",
            hardware_version="SIM",
            sros_version_str="sim",
            src_version_str="sim",
            kernel_release="linux-sim",
        )

    def move_to_station(self, no: int, station_id: int) -> None:
        pose = self._stations_mm.get(int(station_id))
        if pose is None:
            raise RuntimeError(f"unknown station_id={station_id}")
        with self._lock:
            self._system.movement_state = SimpleNamespace(
                no=int(no),
                state="RUNNING",
                result="",
                failed_code="",
                finished=False,
                ok=False,
            )
        if self.travel_sec <= 0:
            self._arrive(no, station_id, pose)
            return
        threading.Thread(
            target=self._arrive_later,
            args=(no, station_id, pose),
            daemon=True,
        ).start()

    def switch_map(self, map_name: str, x_mm: int, y_mm: int, yaw_rad: float, absolute_location: bool = False) -> None:
        del absolute_location
        with self._lock:
            self._system.map_name = str(map_name)
            self._system.location_pose.x = int(x_mm)
            self._system.location_pose.y = int(y_mm)
            self._system.location_pose.yaw = int(round(float(yaw_rad) * 1000))

    def cancel_movement_task(self, _soft: bool = True) -> None:
        with self._lock:
            ms = self._system.movement_state
            self._system.movement_state = SimpleNamespace(
                no=getattr(ms, "no", 0),
                state="FINISHED",
                result="CANCELLED",
                failed_code="CANCELLED",
                finished=True,
                ok=False,
            )

    def _arrive_later(self, no: int, station_id: int, pose: tuple[float, float, float]) -> None:
        time.sleep(self.travel_sec)
        self._arrive(no, station_id, pose)

    def _arrive(self, no: int, station_id: int, pose: tuple[float, float, float]) -> None:
        x, y, yaw = pose
        with self._lock:
            self._system.station_no = int(station_id)
            self._system.location_pose = SimpleNamespace(x=x, y=y, yaw=yaw, confidence=100)
            self._system.movement_state = SimpleNamespace(
                no=int(no),
                state="FINISHED",
                result="OK",
                failed_code="",
                finished=True,
                ok=True,
            )


def _stations_mm(stations: list[Dict[str, Any]]) -> Dict[int, tuple[float, float, float]]:
    out: Dict[int, tuple[float, float, float]] = {}
    for row in stations:
        try:
            vendor_id = int(row.get("id") or 0)
        except (TypeError, ValueError):
            continue
        if vendor_id <= 0:
            continue
        out[vendor_id] = (
            float(row.get("x") or 0.0) * 1000.0,
            float(row.get("y") or 0.0) * 1000.0,
            float(row.get("yaw") or 0.0) * 1000.0,
        )
    return out
