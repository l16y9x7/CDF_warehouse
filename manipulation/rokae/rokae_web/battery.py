"""Read-only battery telemetry cache. No ROS imports or controller commands."""
from __future__ import annotations

import math
import threading
import time


class BatteryTelemetry:
    def __init__(self, stale_seconds=15.0, clock=time.monotonic):
        self.stale_seconds = float(stale_seconds)
        if not math.isfinite(self.stale_seconds) or self.stale_seconds <= 0:
            raise ValueError("battery stale timeout must be positive and finite")
        self._clock = clock
        self._lock = threading.Lock()
        self._received_at = None
        self._data = None
        self._error = "等待电池数据"

    def unavailable(self, error):
        with self._lock:
            self._data = None
            self._received_at = None
            self._error = str(error)

    def update(self, message):
        # sr_amr_interfaces/BatteryState.remaining_percentage is already 0–100.
        level = getattr(message, "remaining_percentage", None)
        state = getattr(message, "state", None)
        if type(level) is not int or not 0 <= level <= 100:
            self.unavailable("电池电量数据无效")
            return
        if state == 0:  # STATE_NA: do not turn an absent battery into 0%.
            self.unavailable("电池状态不可用")
            return
        voltage = getattr(message, "voltage", None)
        voltage_v = voltage / 1000 if type(voltage) is int and voltage > 0 else None
        with self._lock:
            self._data = {"percentage": level, "charging": True if state == 1 else False if state == 3 else None,
                          "voltage_v": voltage_v}
            self._received_at = self._clock()
            self._error = ""

    def snapshot(self):
        with self._lock:
            age = None if self._received_at is None else max(0.0, self._clock() - self._received_at)
            stale = age is not None and age >= self.stale_seconds
            available = self._data is not None and not stale
            return {"available": available, "percentage": self._data["percentage"] if available else None,
                    "charging": self._data["charging"] if available else None,
                    "voltage_v": self._data["voltage_v"] if available else None,
                    "stale": stale, "age_seconds": age, "stale_after_seconds": self.stale_seconds,
                    "source": "ros2", "error": "电池数据已过期" if stale else self._error}
