"""Linux 主机温度与进程运行时长，对齐 dog_device-SMT HostStateReader。"""

from __future__ import annotations

import glob
import math
import os
import time
from typing import Any, Callable, Dict, Optional


class HostStateReader:
    """读取本机 thermal zone 最高温度和进程运行秒数。"""

    def __init__(
        self,
        *,
        thermal_root: str = "/sys/class/thermal",
        monotonic: Callable[[], float] = time.monotonic,
        start_time: Optional[float] = None,
    ) -> None:
        self.thermal_root = thermal_root
        self._monotonic = monotonic
        configured_start = _as_finite_float(start_time)
        self._started_at = (
            configured_start
            if configured_start is not None
            else self._read_monotonic()
        )

    def get_status(self) -> Dict[str, Any]:
        status: Dict[str, Any] = {"runtime_sec": self._runtime_sec()}
        temperature = self._read_temperature()
        if temperature is not None:
            status["temperature_c"] = temperature
        return status

    def _runtime_sec(self) -> int:
        current = self._read_monotonic()
        if current is None or self._started_at is None:
            return 0
        elapsed = current - self._started_at
        if not math.isfinite(elapsed):
            return 0
        return max(int(elapsed), 0)

    def _read_monotonic(self) -> Optional[float]:
        try:
            return _as_finite_float(self._monotonic())
        except Exception:
            return None

    def _read_temperature(self) -> Optional[float]:
        try:
            pattern = os.path.join(self.thermal_root, "thermal_zone*", "temp")
            paths = glob.glob(pattern)
        except (OSError, TypeError, ValueError):
            return None
        values = []
        for path in paths:
            value = _read_float(path)
            if value is not None:
                values.append(_normalize_temperature(value))
        if not values:
            return None
        return round(max(values), 1)


def _read_float(path: str) -> Optional[float]:
    try:
        with open(path, "r", encoding="utf-8") as stream:
            raw = stream.read().strip()
    except (OSError, UnicodeError, TypeError):
        return None
    return _as_finite_float(raw)


def _as_finite_float(value: Any) -> Optional[float]:
    if value is None or value == "":
        return None
    try:
        normalized = float(value)
    except (TypeError, ValueError, OverflowError):
        return None
    return normalized if math.isfinite(normalized) else None


def _normalize_temperature(value: float) -> float:
    return value / 1000.0 if abs(value) > 200.0 else value
