"""珞石 SROS → OSD 底盘指标。字段对齐《机器人上报指标》珞石表；无值整块省略。

collector / transport 只出底盘源。臂/腰仍要 xCore，nav 不填 left_arm/right_arm/body。
"""

from __future__ import annotations

import math
import time
from typing import Any, Dict, Mapping, Optional

SOURCE = "sros_amr"
LOCATION_STATE_RUNNING = 3
STOPPED_LINEAR_MPS = 0.01
STOPPED_ANGULAR_RADPS = 0.01

# SROS device.id：电机/IMU/SRC 来自部件映射总表；相机/电池用 Helios 现场号。
IMU_ID = 121
SRC_ID = 231
MOTOR_ID_MIN = 210
MOTOR_ID_MAX = 229
LIDAR_ID_MIN, LIDAR_ID_MAX = 100, 119
CAMERA_ID_MIN, CAMERA_ID_MAX = 626, 629
BATTERY_DEV_ID = 711
COLLECTOR_NAME = "rokae_composite_state"
COLLECTOR_SCHEMA = "1.0.0"

SYS_STATE = {
    0: "zero",
    1: "initialing",
    2: "idle",
    3: "error",
    4: "start_locating",
    5: "task_nav_initialing",
    6: "task_nav_finding_path",
    7: "task_nav_waiting_finish",
    8: "task_nav_waiting_finish_slow",
    9: "task_nav_refinding_path",
    10: "task_nav_paused",
    11: "task_nav_no_way",
    12: "task_newmap_drawing",
    13: "task_newmap_saving",
    14: "task_path_nav_initialing",
    15: "task_path_waiting_finish",
    16: "task_path_waiting_finish_slow",
    17: "task_path_paused",
    18: "task_nav_no_station",
    19: "task_manual_paused",
    20: "task_nav_path_error",
    21: "task_manual_path_error",
    22: "hardware_error",
    23: "task_path_waiting_checkpoint",
    24: "task_path_waiting_checkpoint_slow",
}
LOCATION_STATE = {
    0: "none",
    1: "none",
    2: "initialing",
    3: "running",
    4: "relocating",
    5: "error",
}
EMERGENCY_STATE = {0: "na", 1: "none", 2: "trigger", 3: "recoverable"}
RUN_STATE = {0: "na", 1: "idle", 2: "running", 3: "block_slowdown", 4: "block_stop"}
OPERATION_STATE = {0: "none", 1: "auto", 2: "manual"}
SCHEDULING_MODE = {0: "none", 1: "manual", 2: "automatic", 3: "maintenance"}
FLEET_MODE = {0: "none", 1: "offline", 2: "online"}
LOCATION_TYPE = {
    0: "none",
    1: "laser",
    2: "qr_code",
    3: "lmk",
    4: "action_odometry",
    5: "odometry",
    6: "disable_map",
}
LOAD_STATE = {0: "none", 1: "free", 2: "full"}
FRESH_STATE = {0: "na", 1: "no", 2: "yes"}
MUTEX_STATE = {0: "none", 1: "locked", 2: "unlocked"}
MANUAL_BUTTON = {0: "none", 1: "auto", 2: "manual"}
OBA_STATE = {0: "na", 1: "enabled", 2: "disabled"}
MOVEMENT_TASK_STATE = {0: "none", 1: "ready", 2: "useless"}
PAUSE_SOURCE = {
    0: "none",
    1: "user",
    2: "nav",
    3: "io",
    4: "fault",
    5: "action_obs",
    6: "work_area_obs",
    7: "up_camera_offset",
    8: "verify",
}
SPEED_LIMIT_SOURCE = {0: "none", 1: "area", 2: "nav", 3: "user", 4: "vfh"}
MC_STATE = {0: "zero", 1: "initialing", 2: "idle", 3: "path_running", 4: "velocity_running"}
POWER_STATE = {0: "na", 1: "normal", 2: "save_mode"}
WIFI_STATE = {0: "na", 2: "disconnected", 3: "connected", 4: "scanning"}
LASER_STATE = {0: "na", 1: "initing", 2: "ok", 3: "error"}
HARDWARE_STATE = {0: "zero", 1: "initialing", 2: "ok", 3: "error"}
BRAKE_SWITCH = {0: "na", 1: "off", 2: "on"}
DEVICE_STATE = {0: "none", 1: "ok", 64: "off", 128: "error", 129: "error_open_failed", 130: "error_timeout"}

SYS_ERROR = {3, 11, 20, 21, 22}
SYS_PAUSED = {10, 17, 19}
SYS_INITIALING = {1, 4, 5, 14}

OSD_CHASSIS_KEYS = (
    "battery",
    "chassis_status",
    "robot_mode",
    "odometry",
    "locomotion",
    "safety",
    "alarm_status",
    "devices_info",
    "motors",
    "mainboard",
    "imu_status",
    "imu",
    "stm32_status",
    "collector",
    "transport",
)


def osd_from_vendor(
    state: Dict[str, Any],
    battery: Dict[str, Any] | None,
) -> Dict[str, Any]:
    fragment: Dict[str, Any] = {}
    battery_osd = battery_osd_from_vendor(battery)
    if battery_osd:
        fragment["battery"] = battery_osd
    if not state:
        return fragment
    chassis = chassis_from_state(state)
    if chassis:
        fragment["chassis_status"] = chassis
    robot_mode = robot_mode_from_state(state)
    if robot_mode:
        fragment["robot_mode"] = robot_mode
    odometry = odometry_from_state(state)
    if odometry:
        fragment["odometry"] = odometry
    locomotion = locomotion_from_state(state)
    if locomotion:
        fragment["locomotion"] = locomotion
    safety = safety_from_state(state)
    if safety:
        fragment["safety"] = safety
    alarm = alarm_from_state(state)
    if alarm:
        fragment["alarm_status"] = alarm
    hardware = _hardware(state)
    devices_info = devices_info_from_hardware(hardware, measured_at_ms=_measured_at(state))
    if devices_info:
        fragment["devices_info"] = devices_info
    motors = motors_from_hardware(hardware, state)
    if motors:
        fragment["motors"] = motors
    mainboard = mainboard_from_state(state)
    if mainboard:
        fragment["mainboard"] = mainboard
    imu_status, imu = imu_from_state(state)
    if imu_status:
        fragment["imu_status"] = imu_status
    if imu:
        fragment["imu"] = imu
    stm32 = stm32_from_hardware(hardware, measured_at_ms=_measured_at(state))
    if stm32:
        fragment["stm32_status"] = stm32
    collector = collector_from_state(state)
    if collector:
        fragment["collector"] = collector
    transport = transport_from_state(state)
    if transport:
        fragment["transport"] = transport
    return fragment


def battery_osd_from_vendor(battery: Dict[str, Any] | None) -> Dict[str, Any]:
    if not isinstance(battery, Mapping) or not battery:
        return {}
    out: Dict[str, Any] = {}
    if "capacity_percent" in battery:
        out["capacity_percent"] = battery["capacity_percent"]
    if "voltage" in battery:
        out["voltage"] = battery["voltage"]
    if "temperature" in battery:
        out["temperature"] = battery["temperature"]
    if "charging" in battery:
        out["charging"] = bool(battery["charging"])
    cycle = battery.get("cycle")
    if isinstance(cycle, (int, float)) and not isinstance(cycle, bool) and int(cycle) > 0:
        out["cycle"] = int(cycle)
    if out:
        out["source"] = str(battery.get("source") or SOURCE)
    for key in ("soc_raw", "pack_voltage_raw", "current_raw", "temperature_raw", "state_raw"):
        if battery.get(key) is not None:
            out[key] = battery[key]
    return out


def chassis_from_state(state: Dict[str, Any]) -> Dict[str, Any]:
    if not state:
        return {}
    vx, vy, wz, stopped = _motion_values(state)
    sys_code = _code(state.get("sys_state"))
    loc_code = _code(state.get("location_state"))
    emergency_code = _code(state.get("emergency_state"))
    if state.get("estop_active") or emergency_code == 2:
        chassis_state = "emergency"
    elif sys_code in SYS_ERROR or loc_code == 5:
        chassis_state = "error"
    elif sys_code in SYS_PAUSED or _code(state.get("run_state")) == 4:
        chassis_state = "paused"
    elif sys_code in SYS_INITIALING or loc_code == 2:
        chassis_state = "initializing"
    elif not stopped or state.get("executing_movement_task"):
        chassis_state = "moving"
    else:
        chassis_state = "idle"
    motion = {
        "stopped": stopped,
        "linear_x_mps": round(vx, 4),
        "linear_y_mps": round(vy, 4),
        "linear_speed_mps": round(math.hypot(vx, vy), 4),
        "angular_radps": round(wz, 4),
    }
    return {"state": chassis_state, "motion": motion}


def robot_mode_from_state(state: Mapping[str, Any]) -> Dict[str, Any]:
    out: Dict[str, Any] = {"source": SOURCE}
    _put_enum(out, "sys_state", state.get("sys_state"), SYS_STATE, code_key="sys_state_code")
    _put_enum(out, "run_state", state.get("run_state"), RUN_STATE)
    _put_enum(out, "operation_state", state.get("operation_state"), OPERATION_STATE)
    _put_enum(out, "scheduling_mode", state.get("scheduling_mode"), SCHEDULING_MODE)
    _put_enum(out, "fleet_mode", state.get("fleet_mode"), FLEET_MODE)
    _put_enum(
        out,
        "location_state",
        state.get("location_state"),
        LOCATION_STATE,
        code_key="location_state_code",
    )
    _put_enum(out, "location_type", state.get("location_type"), LOCATION_TYPE)
    confidence = _code(state.get("location_confidence", state.get("confidence")))
    if confidence is not None:
        out["location_confidence"] = confidence
    map_name = str(state.get("map_name") or "").strip()
    if map_name:
        out["map_name"] = map_name
    out["map_loaded"] = bool(map_name) and map_name.upper() != "NO_MAP"
    station_no = _code(state.get("station_no", state.get("current_station_id")))
    if station_no is not None:
        out["station_no"] = station_no
    _put_enum(out, "load_state", state.get("load_state"), LOAD_STATE)
    _put_enum(out, "fresh_state", state.get("fresh_state"), FRESH_STATE)
    _put_enum(out, "control_mutex_lock_state", state.get("control_mutex_lock_state"), MUTEX_STATE)
    _put_enum(out, "manual_button_state", state.get("manual_button_state"), MANUAL_BUTTON)
    return out if len(out) > 1 else {}


def odometry_from_state(state: Mapping[str, Any]) -> Dict[str, Any]:
    pose = _pose(state)
    vx, vy, wz, _stopped = _motion_values(state)
    if not pose and vx == 0.0 and vy == 0.0 and wz == 0.0 and "linear_velocity_x" not in state:
        if state.get("total_mileage") is None:
            return {}
    out: Dict[str, Any] = {"source": SOURCE, "frame_id": "map"}
    loc = _code(state.get("location_state"))
    confidence = _code(state.get("location_confidence", state.get("confidence")))
    out["pose_confirmed"] = loc == LOCATION_STATE_RUNNING and (confidence is None or confidence > 0)
    if pose:
        out["position"] = {
            "x": pose["x"],
            "y": pose["y"],
            "z": pose.get("z", 0.0),
        }
        orientation = {
            "roll": pose.get("roll", 0.0),
            "pitch": pose.get("pitch", 0.0),
            "yaw": pose["yaw"],
        }
        out["orientation"] = orientation
    if confidence is not None:
        out["confidence"] = confidence
    if "linear_velocity_x" in state or "linear_velocity_y" in state or "angular_velocity" in state:
        out["velocity"] = {
            "linear_x_mps": round(vx, 4),
            "linear_y_mps": round(vy, 4),
            "angular_radps": round(wz, 4),
        }
    mileage = state.get("total_mileage")
    if mileage is None:
        mileage = (_hardware(state).get("src") or {}).get("total_mileage")
    if mileage is not None:
        out["total_mileage_raw"] = mileage
    return out


def locomotion_from_state(state: Mapping[str, Any]) -> Dict[str, Any]:
    if not state:
        return {}
    _vx, _vy, _wz, stopped = _motion_values(state)
    out: Dict[str, Any] = {"source": SOURCE, "stopped": stopped}
    speed_level = _code(state.get("speed_level"))
    if speed_level is not None:
        out["speed_level"] = speed_level
    _put_enum(out, "movement_task_state", state.get("new_movement_task_state"), MOVEMENT_TASK_STATE)
    _put_enum(out, "pause_source", state.get("pause_source"), PAUSE_SOURCE)
    _put_enum(out, "speed_limit_source", state.get("speed_limit_source"), SPEED_LIMIT_SOURCE)
    _put_enum(out, "mc_state", state.get("mc_state"), MC_STATE, code_key="mc_state_code")
    path_no = _code(state.get("path_no"))
    if path_no is not None:
        out["path_no"] = path_no
    return out


def safety_from_state(state: Mapping[str, Any]) -> Dict[str, Any]:
    if not state:
        return {}
    out: Dict[str, Any] = {"source": SOURCE, "certified": False}
    emergency_code = _code(state.get("emergency_state"))
    _put_enum(
        out,
        "emergency_state",
        state.get("emergency_state"),
        EMERGENCY_STATE,
        code_key="emergency_state_code",
    )
    out["emergency_stop"] = bool(state.get("estop_active") or emergency_code == 2)
    source = _emergency_source_name(state.get("emergency_source"))
    if source:
        out["emergency_source"] = source
    _put_enum(out, "obstacle_avoidance", state.get("oba"), OBA_STATE)
    last_error = _code(state.get("last_error_code"))
    if last_error:
        out["last_error_code"] = last_error
    if "manual_control_oba_active" in state:
        out["manual_control_oba_active"] = bool(state.get("manual_control_oba_active"))
    fault_codes = [
        int(item["code"])
        for item in (state.get("faults") or [])
        if isinstance(item, Mapping) and _code(item.get("code")) is not None
    ]
    if not fault_codes:
        raw = state.get("fault_codes")
        if isinstance(raw, list):
            fault_codes = [int(code) for code in raw if _code(code) is not None]
    if fault_codes:
        out["fault_codes"] = fault_codes
    out["fault_count"] = len(fault_codes)
    hardware = _hardware(state)
    _put_enum(out, "hardware_state", hardware.get("hardware_state"), HARDWARE_STATE)
    hw_error = _code(hardware.get("hardware_error_code"))
    if hw_error:
        out["hardware_error_code"] = hw_error
    _put_enum(out, "brake_switch", hardware.get("brake_sw_state", state.get("brake_sw_state")), BRAKE_SWITCH)
    return out


def alarm_from_state(state: Mapping[str, Any]) -> Dict[str, Any]:
    faults = state.get("faults") if isinstance(state.get("faults"), list) else []
    alarms = []
    for item in faults:
        if not isinstance(item, Mapping):
            continue
        code = _code(item.get("code", item.get("id")))
        if code is None:
            continue
        alarm: Dict[str, Any] = {"error_code": code}
        level = _code(item.get("level"))
        if level is not None:
            alarm["level"] = level
        start = item.get("start_time_s", item.get("raise_timestamp"))
        start_code = _code(start)
        if start_code:
            alarm["start_time_ms"] = int(start_code) * 1000
        alarms.append(alarm)
    if not alarms:
        return {}
    return {"measured_at_ms": _measured_at(state), "alarms": alarms}


def collector_from_state(state: Mapping[str, Any]) -> Dict[str, Any]:
    source = state.get("collector")
    if not isinstance(source, Mapping) or not source:
        return {}
    chassis = source.get("chassis") if isinstance(source.get("chassis"), Mapping) else {}
    if not chassis and source.get("connected") is None and source.get("sample_count") is None:
        return {}
    out: Dict[str, Any] = {
        "name": str(source.get("name") or COLLECTOR_NAME),
        "schema_version": str(source.get("schema_version") or COLLECTOR_SCHEMA),
        "started": bool(source.get("started", True)),
    }
    runtime = source.get("runtime_sec")
    if isinstance(runtime, (int, float)) and not isinstance(runtime, bool) and runtime >= 0:
        out["runtime_sec"] = float(runtime)
    poll = source.get("poll_interval_sec")
    if isinstance(poll, (int, float)) and not isinstance(poll, bool) and poll > 0:
        out["poll_interval_sec"] = float(poll)
    chassis_diag = _source_diag(chassis or source)
    sources: Dict[str, Any] = {}
    if chassis_diag:
        sources["chassis"] = chassis_diag
    out["source_count"] = 1
    out["fresh_source_count"] = 1 if chassis_diag.get("fresh") else 0
    if sources:
        out["sources"] = sources
    return out


def transport_from_state(state: Mapping[str, Any]) -> Dict[str, Any]:
    source = state.get("transport")
    if not isinstance(source, Mapping) or not source:
        return {}
    chassis = source.get("chassis") if isinstance(source.get("chassis"), Mapping) else source
    endpoint: Dict[str, Any] = {}
    protocol = str(chassis.get("protocol") or "").strip()
    if protocol:
        endpoint["protocol"] = protocol
    host = str(chassis.get("host") or "").strip()
    if host:
        endpoint["host"] = host
    port = _code(chassis.get("port"))
    if port is not None:
        endpoint["port"] = port
    if "connected" in chassis:
        endpoint["connected"] = bool(chassis.get("connected"))
    if not endpoint:
        return {}
    return {
        "endpoint_count": 1,
        "endpoints": {"chassis": endpoint},
    }


def _source_diag(source: Mapping[str, Any]) -> Dict[str, Any]:
    out: Dict[str, Any] = {}
    if "connected" in source:
        out["connected"] = bool(source.get("connected"))
    if "fresh" in source:
        out["fresh"] = bool(source.get("fresh"))
    for key in ("sample_count", "error_count"):
        value = _code(source.get(key))
        if value is not None:
            out[key] = value
    freshness = source.get("freshness_sec")
    if isinstance(freshness, (int, float)) and not isinstance(freshness, bool) and freshness > 0:
        out["freshness_sec"] = float(freshness)
    age = source.get("last_sample_age_ms")
    if isinstance(age, (int, float)) and not isinstance(age, bool) and age >= 0:
        out["last_sample_age_ms"] = int(age)
    error = source.get("last_error_code")
    if error not in (None, ""):
        out["last_error_code"] = error
    return out


def devices_info_from_hardware(
    hardware: Mapping[str, Any],
    *,
    measured_at_ms: int,
) -> Dict[str, Any]:
    devices = hardware.get("devices") if isinstance(hardware.get("devices"), list) else []
    grouped: Dict[str, Dict[str, Any]] = {}
    for device in devices:
        if not isinstance(device, Mapping):
            continue
        kind = _device_kind(device)
        if not kind or kind in grouped:
            continue
        entry = _device_model_version(device)
        if entry:
            grouped[kind] = entry
    if not grouped:
        return {}
    out: Dict[str, Any] = {"measured_at_ms": measured_at_ms}
    out.update(grouped)
    return out


def motors_from_hardware(hardware: Mapping[str, Any], state: Mapping[str, Any]) -> Dict[str, Any]:
    src = hardware.get("src") if isinstance(hardware.get("src"), Mapping) else {}
    devices = hardware.get("devices") if isinstance(hardware.get("devices"), list) else []
    out: Dict[str, Any] = {"source": SOURCE, "controller": "src"}
    status_codes: Dict[str, int] = {}
    for key in ("m1", "m2", "m3", "m4"):
        code = _code(src.get(f"{key}_status_code"))
        if code is not None:
            status_codes[key] = code
    if status_codes:
        out["status_codes"] = status_codes
    src_state = _code(src.get("src_state"))
    if src_state is not None:
        out["src_state"] = src_state
    src_err = _code(src.get("src_state_error_reason"))
    if src_err:
        out["src_state_error_reason"] = src_err
    wheel = 0
    steer = 0
    faulted = []
    src_device = None
    for device in devices:
        if not isinstance(device, Mapping):
            continue
        dev_id = _code(device.get("id")) or 0
        kind = _device_kind(device)
        if kind == "wheel_motor":
            wheel += 1
        elif kind == "steer_motor":
            steer += 1
        elif kind == "stm32":
            src_device = device
        state_name = _enum_name(device.get("state"), DEVICE_STATE)
        if kind in {"wheel_motor", "steer_motor"} and state_name and state_name != "ok":
            row: Dict[str, Any] = {"id": dev_id, "state": state_name}
            err = _code(device.get("error_code"))
            if err:
                row["error_code"] = err
            faulted.append(row)
    if wheel:
        out["wheel_motor_count"] = wheel
    if steer:
        out["steer_motor_count"] = steer
    if faulted:
        out["faulted"] = faulted
    if src_device:
        model = str(src_device.get("model") or "").strip()
        version = str(src_device.get("version") or "").strip()
        if model:
            out["src_model"] = model
        if version:
            out["src_version"] = version
        state_name = _enum_name(src_device.get("state"), DEVICE_STATE)
        if state_name:
            out["src_device_state"] = state_name
    mileage = src.get("total_mileage")
    if mileage is None:
        mileage = state.get("total_mileage")
    if mileage is not None:
        out["total_mileage_raw"] = mileage
    if len(out) <= 2:
        return {}
    return out


def mainboard_from_state(state: Mapping[str, Any]) -> Dict[str, Any]:
    info = state.get("info") if isinstance(state.get("info"), Mapping) else {}
    hardware = _hardware(state)
    if not info and not hardware:
        return {}
    out: Dict[str, Any] = {"source": SOURCE}
    for key in (
        "serial_no",
        "nickname",
        "vehicle_type",
        "vehicle_serial_no",
        "hardware_version",
        "kernel_release",
        "ip_address",
    ):
        text = str(info.get(key) or hardware.get(key) or "").strip()
        if text:
            out[key] = text
    sros_version = str(info.get("sros_version_str") or info.get("sros_version") or "").strip()
    if sros_version:
        out["sros_version"] = sros_version
    src_version = str(info.get("src_version_str") or info.get("src_version") or "").strip()
    if src_version:
        out["src_version"] = src_version
    for src_key, dst_key in (
        ("cpu_usage", "cpu_usage_percent"),
        ("memory_usage", "memory_usage_percent"),
        ("disk_usage", "disk_usage_percent"),
        ("remain_disk_space", "remain_disk_space_mb"),
        ("cpu_temperature", "cpu_temperature_c"),
        ("box_temperature", "box_temperature_c"),
        ("wifi_strength", "wifi_strength"),
        ("total_power_cycle", "total_power_cycle"),
        ("total_poweron_time", "total_poweron_time_raw"),
    ):
        value = hardware.get(src_key)
        if value is None and src_key in {"total_power_cycle", "total_poweron_time"}:
            value = (hardware.get("src") or {}).get(src_key)
        if isinstance(value, (int, float)) and not isinstance(value, bool):
            out[dst_key] = value
    _put_enum(out, "power_state", hardware.get("power_state"), POWER_STATE)
    _put_enum(out, "wifi_state", hardware.get("wifi_state"), WIFI_STATE)
    _put_enum(out, "laser_state", hardware.get("laser_state"), LASER_STATE)
    devices = []
    for device in hardware.get("devices") or []:
        if not isinstance(device, Mapping):
            continue
        row: Dict[str, Any] = {}
        dev_id = _code(device.get("id"))
        if dev_id is not None:
            row["id"] = dev_id
        name = str(device.get("name") or "").strip()
        if name:
            row["name"] = name
        state_name = _enum_name(device.get("state"), DEVICE_STATE)
        if state_name:
            row["state"] = state_name
        interface = str(device.get("interface") or "").strip()
        if interface:
            row["interface"] = interface
        if row:
            devices.append(row)
    if devices:
        out["devices"] = devices
    return out if len(out) > 1 else {}


def imu_from_state(state: Mapping[str, Any]) -> tuple[Dict[str, Any], Dict[str, Any]]:
    pose = _pose(state)
    imu_device = _device_by_id(_hardware(state), IMU_ID)
    if not pose and not imu_device:
        return {}, {}
    now = _measured_at(state)
    status: Dict[str, Any] = {"frame_id": "sros_imu", "measured_at_ms": now}
    if pose:
        status["euler_angles"] = {
            "roll": pose.get("roll", 0.0),
            "pitch": pose.get("pitch", 0.0),
            "yaw": pose["yaw"],
        }
    if imu_device:
        status["is_connected"] = _enum_name(imu_device.get("state"), DEVICE_STATE) == "ok"
        err = _code(imu_device.get("error_code"))
        if err is not None:
            status["error_code"] = err
    imu: Dict[str, Any] = {"source": SOURCE, "attitude_source": "location_pose"}
    if imu_device:
        model = str(imu_device.get("model") or "").strip()
        version = str(imu_device.get("version") or "").strip()
        if model:
            imu["device_model"] = model
        if version:
            imu["device_version"] = version
    return status, imu


def stm32_from_hardware(hardware: Mapping[str, Any], *, measured_at_ms: int) -> Dict[str, Any]:
    src_device = _device_by_id(hardware, SRC_ID)
    src = hardware.get("src") if isinstance(hardware.get("src"), Mapping) else {}
    if not src_device and not src:
        return {}
    out: Dict[str, Any] = {"frame_id": "sros_src", "measured_at_ms": measured_at_ms}
    if src_device:
        out["is_connected"] = _enum_name(src_device.get("state"), DEVICE_STATE) == "ok"
    errors = []
    src_err = _code(src.get("src_state_error_reason"))
    if src_err:
        errors.append(src_err)
    if src_device:
        dev_err = _code(src_device.get("error_code"))
        if dev_err:
            errors.append(dev_err)
    if errors:
        out["error_code"] = errors
    return out


def _motion_values(state: Mapping[str, Any]) -> tuple[float, float, float, bool]:
    vx = float(state.get("linear_velocity_x") or 0.0)
    vy = float(state.get("linear_velocity_y") or 0.0)
    wz = float(state.get("angular_velocity") or 0.0)
    stopped = abs(vx) <= STOPPED_LINEAR_MPS and abs(vy) <= STOPPED_LINEAR_MPS and abs(wz) <= STOPPED_ANGULAR_RADPS
    return vx, vy, wz, stopped


def _pose(state: Mapping[str, Any]) -> Dict[str, float]:
    if state.get("x") is None or state.get("y") is None:
        pose = state.get("location_pose")
        if isinstance(pose, Mapping) and pose.get("x") is not None and pose.get("y") is not None:
            return {
                "x": float(pose["x"]),
                "y": float(pose["y"]),
                "z": float(pose.get("z") or 0.0),
                "roll": float(pose.get("roll") or 0.0),
                "pitch": float(pose.get("pitch") or 0.0),
                "yaw": float(pose.get("yaw") or 0.0),
            }
        return {}
    return {
        "x": float(state["x"]),
        "y": float(state["y"]),
        "z": float(state.get("z") or 0.0),
        "roll": float(state.get("roll") or 0.0),
        "pitch": float(state.get("pitch") or 0.0),
        "yaw": float(state.get("yaw") or 0.0),
    }


def _hardware(state: Mapping[str, Any]) -> Dict[str, Any]:
    hardware = state.get("hardware")
    return dict(hardware) if isinstance(hardware, Mapping) else {}


def _measured_at(state: Mapping[str, Any]) -> int:
    value = _code(state.get("measured_at_ms"))
    if value:
        return int(value)
    return int(time.time() * 1000)


def _device_kind(device: Mapping[str, Any]) -> str:
    dev_id = _code(device.get("id")) or 0
    if LIDAR_ID_MIN <= dev_id <= LIDAR_ID_MAX:
        return "lidar"
    if dev_id == IMU_ID or 120 <= dev_id <= 129:
        return "imu"
    if CAMERA_ID_MIN <= dev_id <= CAMERA_ID_MAX:
        return "camera"
    if MOTOR_ID_MIN <= dev_id <= MOTOR_ID_MAX:
        return "steer_motor" if dev_id % 2 == 0 else "wheel_motor"
    if dev_id == SRC_ID:
        return "stm32"
    if dev_id == BATTERY_DEV_ID:
        return "battery"
    return ""


def _device_model_version(device: Mapping[str, Any]) -> Dict[str, str]:
    entry: Dict[str, str] = {}
    model = str(device.get("model") or "").strip()
    version = str(device.get("version") or "").strip()
    if model:
        entry["model"] = model
    if version:
        entry["version"] = version
    return entry


def _device_by_id(hardware: Mapping[str, Any], dev_id: int) -> Optional[Dict[str, Any]]:
    for device in hardware.get("devices") or []:
        if isinstance(device, Mapping) and _code(device.get("id")) == dev_id:
            return dict(device)
    return None


def _put_enum(
    out: Dict[str, Any],
    key: str,
    value: Any,
    mapping: Mapping[int, str],
    *,
    code_key: str = "",
) -> None:
    name = _enum_name(value, mapping)
    if name:
        out[key] = name
    code = _code(value)
    if code_key and code is not None:
        out[code_key] = code


def _enum_name(value: Any, mapping: Mapping[int, str]) -> str:
    if value is None or isinstance(value, bool):
        return ""
    if isinstance(value, str):
        text = value.strip()
        if not text:
            return ""
        lower = text.lower()
        for prefix in (
            "sys_state_",
            "location_state_",
            "state_emergency_",
            "emergency_src_",
            "run_",
            "operation_",
            "mode_",
            "fleet_mode_",
            "location_type_",
            "load_",
            "fresh_",
            "oba_",
            "mc_",
            "power_",
            "wifi_",
            "laser_",
            "h_state_",
            "brake_sw_",
            "device_",
            "manualbutton_",
        ):
            if lower.startswith(prefix):
                lower = lower[len(prefix) :]
                break
        return lower
    code = _code(value)
    if code is None:
        return ""
    return mapping.get(code, f"unknown_{code}")


def _code(value: Any) -> Optional[int]:
    if value is None or isinstance(value, bool):
        return None
    if isinstance(value, int):
        return value
    if isinstance(value, float) and value.is_integer():
        return int(value)
    if isinstance(value, str):
        text = value.strip()
        if text.isdigit() or (text.startswith("-") and text[1:].isdigit()):
            return int(text)
        return None
    try:
        return int(value)
    except (TypeError, ValueError):
        return None


def _emergency_source_name(value: Any) -> str:
    if value is None:
        return ""
    if isinstance(value, str):
        return _enum_name(value, {})
    code = _code(value)
    if code is None:
        return ""
    if code == 0:
        return "none"
    return f"unknown_{code}"
