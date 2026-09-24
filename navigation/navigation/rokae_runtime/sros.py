"""直连 sros_sdk_py，TCP 到 MATRIX 192.168.71.50:5001，不经过 ROS。

sros_sdk_py 是 pip/旁边仓的包，不在本仓库。sim:true 时换成 sros_sim，不连真车。
"""

from __future__ import annotations

import logging
import socket
import threading
import time
from typing import Any, Dict, Optional

from navigation.rokae_runtime.base import LOCATION_STATE_RUNNING

LOGGER = logging.getLogger(__name__)

_MM_TO_M = 0.001

# proto MovementTask.TaskState：MT_NA=0 空闲，MT_FINISHED=5 结束。
# 只有中间这些才算在跑；!= FINISHED 会把空闲 MT_NA 误报成 moving/RUNNING。
_MT_EXECUTING_CODES = frozenset({2, 3, 4, 6, 8})
_MT_EXECUTING_NAMES = frozenset(
    {
        "MT_WAIT_FOR_START",
        "WAIT_FOR_START",
        "MT_RUNNING",
        "RUNNING",
        "MT_PAUSED",
        "PAUSED",
        "MT_IN_CANCEL",
        "IN_CANCEL",
        "MT_WAIT_FOR_CHECKPOINT",
        "WAIT_FOR_CHECKPOINT",
    }
)


def movement_task_executing(movement: Any, extra: Any = ()) -> bool:
    if movement is None:
        return False
    if getattr(movement, "finished", False) is True:
        return False
    state = getattr(movement, "state", None)
    if state is None:
        return False
    if extra and state in extra:
        return True
    if isinstance(state, bool):
        return False
    if isinstance(state, int):
        return int(state) in _MT_EXECUTING_CODES
    name = str(getattr(state, "name", None) or state).rsplit(".", 1)[-1].upper()
    return name in _MT_EXECUTING_NAMES


def _install_ephemeral_connect(srp: Any) -> None:
    """Orin 上 SDK 默认 bind 8888 会 Errno 99，改绑系统分配端口。"""
    from sros_sdk_py.srp import SrpProtocol

    async def _connect(ip_addr: str, source_port: Any = None) -> bool:
        del source_port
        try:
            sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            sock.settimeout(1)
            sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            sock.bind(("0.0.0.0", 0))
            sock.connect((ip_addr, 5001))
            srp._transport, srp._protocol = await srp._loop.create_connection(
                lambda: SrpProtocol(srp), sock=sock
            )
            LOGGER.info("sros tcp ok: local=%s -> %s:5001", sock.getsockname(), ip_addr)
            return True
        except BaseException as exc:
            LOGGER.error("sros tcp connect failed: %s", exc)
            return False

    srp._connect = _connect


class RokaeSrosClient:
    def __init__(self, config: Dict[str, Any], *, client: Any = None) -> None:
        sros = (config.get("rokae") or {}).get("sros") or {}
        if not isinstance(sros, dict):
            sros = {}
        self.host = str(sros.get("host") or "192.168.71.50")
        self.username = str(sros.get("username") or "admin")
        self.password = str(sros.get("password") or "admin")
        self.poll_sec = float(sros.get("poll_sec", 0.1))
        self.freshness_sec = float(sros.get("freshness_sec", 2.0))
        self.sim = bool(sros.get("sim"))
        # 0 / 缺省：不绑死 8888。Orin 上 bind 8888 会 Errno 99。
        raw_port = sros.get("source_port", 0)
        try:
            parsed_port = int(raw_port)
        except (TypeError, ValueError):
            parsed_port = 0
        self.source_port = parsed_port if parsed_port > 0 else None
        self.travel_sec = float(sros.get("travel_sec", 0.4))
        rokae_cfg = config.get("rokae") if isinstance(config.get("rokae"), dict) else {}
        self._stations = rokae_cfg.get("stations") if isinstance(rokae_cfg, dict) else None
        self._lock = threading.Lock()
        self._state: Optional[Dict[str, Any]] = None
        self._battery: Optional[Dict[str, Any]] = None
        self._info: Optional[Dict[str, Any]] = None
        self._info_at = 0.0
        self._movement_no = 0
        self._started_at = time.monotonic()
        self._sample_count = 0
        self._error_count = 0
        self._last_sample_at: Optional[float] = None
        self._last_error_code: Any = None
        self._client = client
        self._ok = False
        self._mt_finished: Any = "FINISHED"
        self._mt_executing: frozenset[Any] = frozenset()
        self._task_ok: Any = "OK"
        self._loc_running = LOCATION_STATE_RUNNING
        self._estop_none: tuple[Any, ...] = (0, 1, "NONE", "NA", None)
        self._charging_state: Any = 1
        if client is None:
            self.start()
        else:
            self._ok = True

    def start(self) -> None:
        if self.sim:
            from navigation.rokae_runtime.sros_sim import SimSrpClient

            client = SimSrpClient(stations=self._stations, travel_sec=self.travel_sec)
            client.connect()
            self._client = client
            self._ok = True
            LOGGER.info("rokae sros sim started: no TCP, travel_sec=%s", self.travel_sec)
            return
        try:
            from sros_sdk_py.client import SrpClient
            from sros_sdk_py.client_types import SrpConnectionConfig
            from sros_sdk_py.main_pb2 import HardwareState, MovementTask, SystemState, TaskResult
        except Exception as exc:
            LOGGER.warning("rokae sros_sdk_py unavailable: %s", exc)
            return

        self._mt_finished = MovementTask.TaskState.MT_FINISHED
        self._mt_executing = frozenset(
            {
                MovementTask.TaskState.MT_WAIT_FOR_START,
                MovementTask.TaskState.MT_RUNNING,
                MovementTask.TaskState.MT_PAUSED,
                MovementTask.TaskState.MT_IN_CANCEL,
                MovementTask.TaskState.MT_WAIT_FOR_CHECKPOINT,
            }
        )
        self._task_ok = TaskResult.TASK_RESULT_OK
        self._loc_running = int(SystemState.LocationState.LOCATION_STATE_RUNNING)
        self._estop_none = (
            SystemState.EmergencyState.STATE_EMERGENCY_NA,
            SystemState.EmergencyState.STATE_EMERGENCY_NONE,
        )
        self._charging_state = HardwareState.BatteryState.BATTERY_CHARGING
        client = SrpClient()
        _install_ephemeral_connect(client._srp)
        connect_kwargs: Dict[str, Any] = {
            "ip": self.host,
            "username": self.username,
            "passwd": self.password,
        }
        try:
            config = SrpConnectionConfig(
                **connect_kwargs, source_port=self.source_port
            )
        except TypeError:
            config = SrpConnectionConfig(**connect_kwargs)
        if hasattr(config, "source_port"):
            try:
                config.source_port = self.source_port
            except Exception:
                pass
        LOGGER.info(
            "rokae sros connecting: host=%s source_port=%s",
            self.host,
            getattr(config, "source_port", self.source_port),
        )
        ok = client.connect(config)
        if not ok:
            LOGGER.warning("rokae sros connect failed: host=%s", self.host)
            return
        self._client = client
        self._ok = True
        LOGGER.info("rokae sros started: host=%s", self.host)

    def apply_stations(self, stations: list[Dict[str, Any]]) -> None:
        self._stations = stations
        setter = getattr(self._client, "set_stations", None)
        if callable(setter):
            setter(stations)

    def latest_state(self) -> Optional[Dict[str, Any]]:
        self._sync()
        with self._lock:
            if not self._state:
                return None
            snapshot = dict(self._state)
        snapshot["collector"] = self._collector_diag()
        snapshot["transport"] = self._transport_diag()
        return snapshot

    def latest_battery(self) -> Optional[Dict[str, Any]]:
        self._sync()
        with self._lock:
            return dict(self._battery) if self._battery else None

    def move_to_station(self, station_id: int, timeout_sec: float) -> Dict[str, Any]:
        # SDK 的 move_to_station 只负责下发；是否到站靠下面轮询任务状态。
        client = self._client
        if client is None or not self._ok:
            return {"accepted": False, "unavailable": True, "message": "sros_sdk_py not connected"}
        self._movement_no += 1
        movement_no = self._movement_no
        try:
            client.move_to_station(movement_no, int(station_id))
        except Exception as exc:
            code = getattr(exc, "result_code", "") or str(exc)
            LOGGER.error("sros move_to_station failed to start: station=%s error=%s", station_id, code)
            return {"accepted": False, "unavailable": True, "message": str(code)}
        deadline = time.monotonic() + max(0.1, float(timeout_sec))
        while time.monotonic() < deadline:
            done = self._poll_movement(movement_no)
            if done is not None:
                return done
            time.sleep(self.poll_sec)
        return {
            "accepted": True,
            "timed_out": True,
            "message": "move_to_station result timeout",
        }

    def stop_motion(self) -> Dict[str, Any]:
        client = self._client
        if client is None or not hasattr(client, "cancel_movement_task"):
            return {
                "accepted": False,
                "error_code": "NAVIGATION_UNAVAILABLE",
                "message": "sros cancel_movement_task unavailable",
            }
        try:
            client.cancel_movement_task(True)
        except Exception as exc:
            return {
                "accepted": False,
                "error_code": "NAVIGATION_VENDOR",
                "message": str(exc),
            }
        return {"accepted": True, "terminal_state": "CANCELLED"}

    def switch_map(self, map_name: str, x_m: float, y_m: float, yaw_rad: float) -> Dict[str, Any]:
        # 底盘 CMD_MAP_SWITCHING：取消定位、换图、再按位姿定位。x/y 协议单位是毫米，yaw 是弧度。
        # 车上的 SrpClient 没有 switch_map，切图在 _srp.switch_map。不要走 set_map_with_initial_pose，它会先把 yaw 收成整数。
        client = self._client
        if client is None or not self._ok:
            return {
                "accepted": False,
                "error_code": "NAVIGATION_UNAVAILABLE",
                "message": "sros switch_map unavailable",
            }
        x_mm = int(round(float(x_m) * 1000))
        y_mm = int(round(float(y_m) * 1000))
        yaw = float(yaw_rad)
        switch = getattr(client, "switch_map", None)
        if not callable(switch):
            switch = getattr(getattr(client, "_srp", None), "switch_map", None)
        if not callable(switch):
            return {
                "accepted": False,
                "error_code": "NAVIGATION_UNAVAILABLE",
                "message": "sros switch_map unavailable",
            }
        try:
            switch(str(map_name), x_mm, y_mm, yaw)
        except Exception as exc:
            LOGGER.error("sros switch_map failed: map=%s error=%s", map_name, exc)
            return {
                "accepted": False,
                "error_code": "NAVIGATION_VENDOR",
                "message": str(exc),
            }
        return {"accepted": True, "map_name": str(map_name)}

    def close(self) -> None:
        client = self._client
        disconnect = getattr(client, "disconnect", None) if client is not None else None
        if callable(disconnect):
            try:
                disconnect()
            except Exception:
                pass
        self._client = None
        self._ok = False

    def _sync(self) -> None:
        client = self._client
        if client is None:
            return
        try:
            fetch_sys = getattr(client, "fetch_system_state", None)
            fetch_hw = getattr(client, "fetch_hardware_state", None)
            if callable(fetch_sys):
                fetch_sys()
            if callable(fetch_hw):
                fetch_hw()
            sys_state = getattr(client, "get_current_system_state", lambda: None)()
            hw_state = getattr(client, "get_current_hardware_state", lambda: None)()
            self._ingest(sys_state, hw_state)
            self._maybe_refresh_info()
            if sys_state is None and hw_state is None:
                self._error_count += 1
                self._last_error_code = "empty_state"
                return
            self._sample_count += 1
            self._last_sample_at = time.monotonic()
            self._last_error_code = None
        except Exception as exc:
            self._error_count += 1
            self._last_error_code = str(getattr(exc, "result_code", "") or exc)
            LOGGER.warning("rokae sros poll failed: %s", self._last_error_code)

    def _collector_diag(self) -> Dict[str, Any]:
        now = time.monotonic()
        age_ms = None
        if self._last_sample_at is not None:
            age_ms = int(max(0.0, (now - self._last_sample_at) * 1000.0))
        fresh = age_ms is not None and (age_ms / 1000.0) <= max(0.1, self.freshness_sec)
        chassis: Dict[str, Any] = {
            "connected": bool(self._ok),
            "fresh": fresh,
            "sample_count": self._sample_count,
            "error_count": self._error_count,
            "freshness_sec": self.freshness_sec,
        }
        if age_ms is not None:
            chassis["last_sample_age_ms"] = age_ms
        if self._last_error_code:
            chassis["last_error_code"] = self._last_error_code
        return {
            "name": "rokae_composite_state",
            "schema_version": "1.0.0",
            "started": bool(self._ok or self._client is not None),
            "runtime_sec": round(max(0.0, now - self._started_at), 3),
            "poll_interval_sec": self.poll_sec,
            "chassis": chassis,
        }

    def _transport_diag(self) -> Dict[str, Any]:
        return {
            "protocol": "srp",
            "host": self.host,
            "port": 5001,
            "connected": bool(self._ok),
        }

    def _poll_movement(self, movement_no: int) -> Optional[Dict[str, Any]]:
        self._sync()
        client = self._client
        if client is None:
            return {"accepted": True, "result": False, "error_code": "NAVIGATION_VENDOR"}
        sys_state = getattr(client, "get_current_system_state", lambda: None)()
        movement = getattr(sys_state, "movement_state", None) if sys_state is not None else None
        if movement is None:
            return None
        if getattr(movement, "no", None) != movement_no:
            return None
        if getattr(movement, "finished", None) is True:
            return {"accepted": True, "result": bool(getattr(movement, "ok", True)), "error_code": ""}
        if getattr(movement, "state", None) != self._mt_finished:
            return None
        ok = getattr(movement, "result", None) == self._task_ok
        error_code = "" if ok else str(getattr(movement, "failed_code", "") or "NAVIGATION_VENDOR")
        return {"accepted": True, "result": bool(ok), "error_code": error_code}

    def _ingest(self, sys_state: Any, hw_state: Any) -> None:
        now_ms = int(time.time() * 1000)
        with self._lock:
            previous = dict(self._state) if self._state else {}
        snapshot = dict(previous)
        snapshot["measured_at_ms"] = now_ms
        if self._info:
            snapshot["info"] = dict(self._info)
        if sys_state is not None:
            snapshot.update(self._system_snapshot(sys_state))
        if hw_state is not None:
            hardware = self._hardware_snapshot(hw_state)
            snapshot["hardware"] = hardware
            src = hardware.get("src") if isinstance(hardware.get("src"), dict) else {}
            if src.get("total_mileage") is not None:
                snapshot["total_mileage"] = src["total_mileage"]
            with self._lock:
                self._battery = self._battery_snapshot(hw_state)
        with self._lock:
            if self._info:
                snapshot["info"] = dict(self._info)
            self._state = snapshot

    def _system_snapshot(self, sys_state: Any) -> Dict[str, Any]:
        pose = getattr(sys_state, "location_pose", None)
        emergency = getattr(sys_state, "emergency_state", None)
        mc = getattr(sys_state, "mc_state", None)
        movement = getattr(sys_state, "movement_state", None)
        snapshot: Dict[str, Any] = {
            "location_state": _int(getattr(sys_state, "location_state", -1), default=-1),
            "estop_active": bool(
                emergency is not None and emergency not in self._estop_none
            ),
            "executing_movement_task": movement_task_executing(
                movement, self._mt_executing
            ),
            "linear_velocity_x": _mm(getattr(mc, "v_x", 0) if mc is not None else 0),
            "linear_velocity_y": _mm(getattr(mc, "v_y", 0) if mc is not None else 0),
            "angular_velocity": _mm(getattr(mc, "w", 0) if mc is not None else 0),
            "map_name": str(getattr(sys_state, "map_name", "") or ""),
            "current_station_id": _int(getattr(sys_state, "station_no", 0)),
            "station_no": _int(getattr(sys_state, "station_no", 0)),
            "sys_state": getattr(sys_state, "sys_state", None),
            "run_state": getattr(sys_state, "run_state", None),
            "operation_state": getattr(sys_state, "operation_state", None),
            "scheduling_mode": getattr(sys_state, "scheduling_mode", None),
            "fleet_mode": getattr(sys_state, "fleet_mode", None),
            "location_type": getattr(sys_state, "location_type", None),
            "load_state": getattr(sys_state, "load_state", None),
            "fresh_state": getattr(sys_state, "fresh_state", None),
            "control_mutex_lock_state": getattr(sys_state, "control_mutex_lock_state", None),
            "manual_button_state": getattr(sys_state, "manual_button_state", None),
            "emergency_state": emergency,
            "emergency_source": getattr(sys_state, "emergency_source", None),
            "oba": getattr(sys_state, "oba_enable_state", None),
            "speed_level": getattr(sys_state, "speed_level", None),
            "new_movement_task_state": getattr(sys_state, "new_movement_task_state", None),
            "pause_source": getattr(sys_state, "current_pause_source", None),
            "speed_limit_source": getattr(sys_state, "effective_speed_limit_source", None),
            "last_error_code": getattr(sys_state, "last_error_code", None),
            "manual_control_oba_active": bool(
                getattr(sys_state, "manual_control_oba_state", False)
            ),
        }
        if mc is not None:
            snapshot["mc_state"] = getattr(mc, "state", None)
            snapshot["path_no"] = getattr(mc, "path_no", None)
        if pose is not None:
            snapshot["x"] = _mm(getattr(pose, "x", 0))
            snapshot["y"] = _mm(getattr(pose, "y", 0))
            snapshot["z"] = _mm(getattr(pose, "z", 0))
            snapshot["roll"] = _mm(getattr(pose, "roll", 0))
            snapshot["pitch"] = _mm(getattr(pose, "pitch", 0))
            snapshot["yaw"] = _mm(getattr(pose, "yaw", 0))
            snapshot["confidence"] = _int(getattr(pose, "confidence", 0))
            snapshot["location_confidence"] = snapshot["confidence"]
        faults = []
        for fault in getattr(sys_state, "faults", None) or []:
            faults.append(
                {
                    "code": _int(getattr(fault, "id", getattr(fault, "code", 0))),
                    "level": _int(getattr(fault, "level", 0)),
                    "start_time_s": _int(getattr(fault, "raise_timestamp", 0)),
                }
            )
        if faults:
            snapshot["faults"] = faults
        return snapshot

    def _hardware_snapshot(self, hw_state: Any) -> Dict[str, Any]:
        src = getattr(hw_state, "src_hardware_state", None)
        hardware: Dict[str, Any] = {
            "cpu_usage": getattr(hw_state, "cpu_usage", None),
            "memory_usage": getattr(hw_state, "memory_usage", None),
            "disk_usage": getattr(hw_state, "disk_usage", None),
            "remain_disk_space": getattr(hw_state, "remain_disk_space", None),
            "cpu_temperature": getattr(hw_state, "cpu_temperature", None),
            "box_temperature": getattr(hw_state, "box_temperature", None),
            "wifi_strength": getattr(hw_state, "wifi_strength", None),
            "wifi_state": getattr(hw_state, "wifi_state", None),
            "laser_state": getattr(hw_state, "laser_state", None),
            "power_state": getattr(hw_state, "power_state", None),
            "brake_sw_state": getattr(hw_state, "brake_sw_state", None),
            "hardware_state": getattr(hw_state, "hardware_state", None),
            "hardware_error_code": getattr(hw_state, "hardware_error_code", None),
            "ip_address": str(getattr(hw_state, "ip_address", "") or ""),
        }
        if src is not None:
            hardware["src"] = {
                "m1_status_code": getattr(src, "m1_status_code", None),
                "m2_status_code": getattr(src, "m2_status_code", None),
                "m3_status_code": getattr(src, "m3_status_code", None),
                "m4_status_code": getattr(src, "m4_status_code", None),
                "src_state": getattr(src, "src_state", None),
                "src_state_error_reason": getattr(src, "src_state_error_reason", None),
                "total_mileage": getattr(src, "total_mileage", None),
                "total_power_cycle": getattr(src, "total_power_cycle", None),
                "total_poweron_time": getattr(src, "total_poweron_time", None),
            }
        devices = []
        for device in getattr(hw_state, "devices", None) or []:
            devices.append(
                {
                    "id": _int(getattr(device, "id", 0)),
                    "name": str(getattr(device, "name", "") or ""),
                    "state": getattr(device, "state", None),
                    "error_code": _int(getattr(device, "error_code", 0)),
                    "model": str(getattr(device, "model_no", "") or ""),
                    "version": str(getattr(device, "version_no", "") or ""),
                    "interface": str(getattr(device, "interface_name", "") or ""),
                }
            )
        if devices:
            hardware["devices"] = devices
        return hardware

    def _battery_snapshot(self, hw_state: Any) -> Dict[str, Any]:
        voltage_mv = float(getattr(hw_state, "battery_voltage", 0) or 0)
        current_ma = getattr(hw_state, "battery_current", None)
        temperature = float(getattr(hw_state, "battery_temperature", 0) or 0)
        percentage = float(getattr(hw_state, "battery_percentage", 0) or 0)
        state_raw = getattr(hw_state, "battery_state", None)
        battery: Dict[str, Any] = {
            "capacity_percent": percentage,
            "temperature": temperature,
            "charging": state_raw == self._charging_state,
            "cycle": int(getattr(hw_state, "battery_use_cycles", 0) or 0),
            "source": "sros_amr",
            "soc_raw": percentage,
            "temperature_raw": temperature,
        }
        if voltage_mv > 0:
            battery["voltage"] = round(voltage_mv / 1000.0, 3)
            battery["pack_voltage_raw"] = voltage_mv
        if current_ma is not None:
            battery["current_raw"] = current_ma
        if state_raw is not None:
            battery["state_raw"] = _int(state_raw)
        return battery

    def _maybe_refresh_info(self) -> None:
        now = time.monotonic()
        if self._info is not None and now - self._info_at < 30.0:
            return
        client = self._client
        getter = getattr(client, "get_info", None)
        if not callable(getter):
            srp = getattr(client, "_srp", None)
            getter = getattr(srp, "get_info", None)
        if not callable(getter):
            return
        try:
            info = getter()
        except Exception:
            return
        if info is None:
            return
        payload = {
            "serial_no": str(getattr(info, "serial_no", "") or ""),
            "nickname": str(getattr(info, "nickname", "") or ""),
            "vehicle_type": str(getattr(info, "vehicle_type", "") or ""),
            "vehicle_serial_no": str(getattr(info, "vehicle_serial_no", "") or ""),
            "hardware_version": str(getattr(info, "hardware_version", "") or ""),
            "sros_version": str(
                getattr(info, "sros_version_str", "") or getattr(info, "sros_version", "") or ""
            ),
            "src_version": str(
                getattr(info, "src_version_str", "") or getattr(info, "src_version", "") or ""
            ),
            "kernel_release": str(getattr(info, "kernel_release", "") or ""),
        }
        self._info = {key: value for key, value in payload.items() if value}
        self._info_at = now
        with self._lock:
            if self._state is not None:
                self._state["info"] = dict(self._info)


def _mm(value: Any) -> float:
    try:
        return float(value) * _MM_TO_M
    except (TypeError, ValueError):
        return 0.0


def _int(value: Any, default: int = 0) -> int:
    try:
        return int(value)
    except (TypeError, ValueError):
        return default
