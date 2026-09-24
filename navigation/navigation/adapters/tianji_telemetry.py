"""天机 GetState → OSD battery / chassis，算法对齐 dog_device-SMT。"""

from __future__ import annotations

import logging
import math
import time
from typing import Any, Mapping, Optional

LOGGER = logging.getLogger(__name__)
_WARN_INTERVAL_SEC = 30.0
_last_warn_at: dict[str, float] = {}


def _warn_throttled(key: str, message: str, *args: object) -> None:
    now = time.monotonic()
    previous = _last_warn_at.get(key, 0.0)
    if now - previous < _WARN_INTERVAL_SEC:
        return
    _last_warn_at[key] = now
    LOGGER.warning(message, *args)


MIN_CAPACITY_MAX_TEMPERATURE = "min_capacity_max_temperature"
DEFAULT_STOPPED_LINEAR_THRESHOLD_MPS = 0.01
DEFAULT_STOPPED_ANGULAR_THRESHOLD_RADPS = 0.01
DEFAULT_BATTERY_CYCLE = 31


def parse_get_state_content(envelope: Any) -> dict[str, Any]:
    """校验厂家信封，只返回本次 Content。失败返回空。"""

    if not isinstance(envelope, Mapping):
        if envelope is not None:
            _warn_throttled(
                "envelope-type",
                "GetState envelope is not an object: type=%s",
                type(envelope).__name__,
            )
        return {}
    error_code = envelope.get("ErrorCode")
    if (
        not isinstance(error_code, int)
        or isinstance(error_code, bool)
        or error_code != 0
    ):
        _warn_throttled(
            "error-code",
            "GetState ErrorCode rejected: ErrorCode=%r",
            error_code,
        )
        return {}
    content = envelope.get("Content")
    if not isinstance(content, Mapping):
        _warn_throttled(
            "content-type",
            "GetState Content is not an object: type=%s",
            type(content).__name__,
        )
        return {}
    return dict(content)


def aggregate_battery_channels(
    value: Any,
    *,
    strategy: str = MIN_CAPACITY_MAX_TEMPERATURE,
    cycle_default: Optional[int] = DEFAULT_BATTERY_CYCLE,
) -> dict[str, Any]:
    """电量取小、温度取大；cycle 为 SMT 产品约定值，不是实测。"""

    if str(strategy or "").strip() != MIN_CAPACITY_MAX_TEMPERATURE:
        raise ValueError(
            "battery_aggregation must be " + MIN_CAPACITY_MAX_TEMPERATURE
        )
    if not isinstance(value, list):
        if value is not None:
            _warn_throttled(
                "battery-type",
                "batteryStateList is not a list: type=%s",
                type(value).__name__,
            )
        return {}

    capacities: list[float] = []
    temperatures: list[float] = []
    voltages: list[float] = []
    valid_channel_count = 0
    charging_false_count = 0
    any_charging = False
    for channel in value:
        if not _valid_battery_channel(channel):
            continue
        valid_channel_count += 1
        capacity = _finite_float(channel.get("energy"))
        if capacity is not None and 0.0 <= capacity <= 100.0:
            capacities.append(capacity)
        temperature = _finite_float(channel.get("temperature"))
        if temperature is not None:
            temperatures.append(temperature)
        voltage = _finite_float(channel.get("voltage"))
        if voltage is not None and voltage >= 0.0:
            voltages.append(voltage)
        charging = channel.get("isCharging")
        if charging is True:
            any_charging = True
        elif charging is False:
            charging_false_count += 1

    battery: dict[str, Any] = {}
    if capacities:
        battery["capacity_percent"] = round(min(capacities), 1)
    if temperatures:
        battery["temperature"] = round(max(temperatures), 1)
    if voltages:
        battery["voltage"] = round(min(voltages), 3)
    if any_charging:
        battery["charging"] = True
    elif valid_channel_count and charging_false_count == valid_channel_count:
        battery["charging"] = False
    if battery and cycle_default is not None:
        battery["cycle"] = int(cycle_default)
    if value and not battery:
        _warn_throttled(
            "battery-empty",
            "batteryStateList has %s rows but no valid OSD battery fields",
            len(value),
        )
    elif battery:
        LOGGER.debug(
            "OSD battery: capacity=%s temp=%s charging=%s channels=%s",
            battery.get("capacity_percent"),
            battery.get("temperature"),
            battery.get("charging"),
            valid_channel_count,
        )
    return battery


def normalize_chassis_status(
    value: Any,
    *,
    stopped_linear_threshold_mps: float = DEFAULT_STOPPED_LINEAR_THRESHOLD_MPS,
    stopped_angular_threshold_radps: float = DEFAULT_STOPPED_ANGULAR_THRESHOLD_RADPS,
) -> dict[str, Any]:
    """从同一份 GetState Content 解析底盘；不用位姿差分估速度。"""

    if not isinstance(value, Mapping):
        return {}
    linear_threshold = _nonnegative_finite(
        stopped_linear_threshold_mps, "stopped_linear_threshold_mps"
    )
    angular_threshold = _nonnegative_finite(
        stopped_angular_threshold_radps, "stopped_angular_threshold_radps"
    )
    chassis: dict[str, Any] = {}

    linear = value.get("linearSpeedVec3")
    angular = value.get("angularSpeedVec3")
    linear = linear if isinstance(linear, Mapping) else {}
    angular = angular if isinstance(angular, Mapping) else {}
    motion: dict[str, Any] = {}
    measured: dict[str, float] = {}
    for source, key, target in (
        (linear, "x", "linear_x_mps"),
        (linear, "y", "linear_y_mps"),
        (angular, "z", "angular_radps"),
    ):
        normalized = _finite_float(source.get(key))
        if normalized is not None:
            measured[target] = normalized
            motion[target] = round(normalized, 4)
    if "linear_x_mps" in measured and "linear_y_mps" in measured:
        linear_speed = math.hypot(
            measured["linear_x_mps"], measured["linear_y_mps"]
        )
        if math.isfinite(linear_speed):
            motion["linear_speed_mps"] = round(linear_speed, 4)
    stopped: bool | None = None
    if len(measured) == 3:
        stopped = bool(
            abs(measured["linear_x_mps"]) <= linear_threshold
            and abs(measured["linear_y_mps"]) <= linear_threshold
            and abs(measured["angular_radps"]) <= angular_threshold
        )
        motion["stopped"] = stopped
    if motion:
        chassis["motion"] = motion

    if value.get("isScram") is True:
        chassis["state"] = "emergency"
    elif value.get("isNormal") is False:
        chassis["state"] = "fault"
    elif _numeric_code_is(value.get("connectState"), 0):
        chassis["state"] = "offline"
    elif stopped is False:
        chassis["state"] = "moving"
    elif (
        stopped is True
        and value.get("isNormal") is True
        and _numeric_code_is(value.get("connectState"), 1)
    ):
        chassis["state"] = "idle"
    return chassis


def osd_chassis_status(chassis: Mapping[str, Any]) -> dict[str, Any]:
    """SMT 对外 OSD：只保留 state + motion，缺 xy 时 linear_speed_mps 为 null。"""

    if not isinstance(chassis, Mapping) or not chassis:
        return {}
    result: dict[str, Any] = {}
    state = chassis.get("state")
    if isinstance(state, str) and state.strip():
        result["state"] = state.strip()
    motion_src = chassis.get("motion")
    motion: dict[str, Any] = {}
    if isinstance(motion_src, Mapping):
        for key in ("stopped", "linear_x_mps", "linear_y_mps", "angular_radps"):
            if key in motion_src:
                motion[key] = motion_src[key]
        speed = motion_src.get("linear_speed_mps")
        if (
            isinstance(speed, (int, float))
            and not isinstance(speed, bool)
            and math.isfinite(float(speed))
            and float(speed) >= 0.0
        ):
            motion["linear_speed_mps"] = float(speed)
        else:
            motion["linear_speed_mps"] = None
    elif result:
        motion["linear_speed_mps"] = None
    if motion:
        result["motion"] = motion
    return result


def osd_from_get_state(
    content: Any,
    *,
    cycle_default: Optional[int] = DEFAULT_BATTERY_CYCLE,
) -> dict[str, Any]:
    fragment: dict[str, Any] = {}
    battery = aggregate_battery_channels(
        content.get("batteryStateList") if isinstance(content, Mapping) else None,
        cycle_default=cycle_default,
    )
    if battery:
        fragment["battery"] = battery
    chassis = osd_chassis_status(normalize_chassis_status(content))
    if chassis:
        fragment["chassis_status"] = chassis
    if isinstance(content, Mapping) and not fragment:
        _warn_throttled(
            "osd-empty",
            "GetState Content parsed but OSD battery/chassis are empty",
        )
    return fragment


def _valid_battery_channel(value: Any) -> bool:
    if not isinstance(value, Mapping):
        return False
    error_code = value.get("errorCode")
    return bool(
        value.get("isConnected") is True
        and value.get("isInPlace") is True
        and isinstance(error_code, (int, float))
        and not isinstance(error_code, bool)
        and math.isfinite(float(error_code))
        and float(error_code) == 0.0
    )


def _finite_float(value: Any) -> float | None:
    if value is None or isinstance(value, bool):
        return None
    try:
        normalized = float(value)
    except (TypeError, ValueError, OverflowError):
        return None
    return normalized if math.isfinite(normalized) else None


def _nonnegative_finite(value: Any, name: str) -> float:
    normalized = _finite_float(value)
    if normalized is None or normalized < 0.0:
        raise ValueError(f"{name} must be a finite non-negative number")
    return normalized


def _numeric_code_is(value: Any, expected: int) -> bool:
    return bool(
        isinstance(value, (int, float))
        and not isinstance(value, bool)
        and math.isfinite(float(value))
        and float(value) == float(expected)
    )
