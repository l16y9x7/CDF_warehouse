"""Periodic, read-only snapshots sharing the motion backend's SDK objects."""
from __future__ import annotations

import copy
from datetime import datetime, timezone
import math
import threading
import time


MODULES = {"left_arm": 7, "right_arm": 7, "trunk": 4}


class TelemetryBusy(RuntimeError):
    """Motion owns the SDK lock; keep the previous sample until it expires."""


def module_data(module, joints, pose, state):
    joints, pose = list(joints), list(pose)
    if (len(joints) != MODULES[module] or len(pose) != 6
            or not all(math.isfinite(float(v)) for v in joints + pose)):
        raise ValueError(f"{module} 返回关节数或位姿无效")
    if not isinstance(state, str) or not state or state == "unknown":
        raise ValueError(f"{module} 运行状态不可用")
    return {
        "joint_positions_deg": joints,
        "end_pose": {"position_mm": pose[:3], "rpy_deg": pose[3:],
                     "frame": f"{module}_controller_base", "coordinate_type": "flangeInBase"},
        "state": state,
    }


class UpperBodyTelemetry:
    def __init__(self, backend, config, *, hardware=False, audit=None):
        self.backend, self.hardware, self.audit = backend, hardware, audit
        self.interval = float(config.get("poll_interval_seconds", 1.0))
        self.stale_after = float(config.get("stale_after_seconds", 3.0))
        if (not math.isfinite(self.interval) or not .1 <= self.interval <= 60
                or not math.isfinite(self.stale_after) or self.stale_after < 2 * self.interval):
            raise ValueError("遥测采样周期须为 0.1–60 秒，过期时间须至少为采样周期的两倍")
        self._lock, self._stop = threading.Lock(), threading.Event()
        self._thread = None
        self._records = {m: {"sequence": 0, "error": "尚未采样"} for m in MODULES}

    def start(self):
        with self._lock:
            if self._thread is not None:
                return
            self._thread = threading.Thread(target=self._run, name="upper-body-telemetry", daemon=True)
            self._thread.start()

    def close(self):
        self._stop.set()
        if self._thread is not None:
            self._thread.join(timeout=2.0)
        # SDK lifetime belongs to ControlService, never to a telemetry reader.

    def poll_once(self):
        for module in MODULES:
            if self._stop.is_set():
                return
            started = time.monotonic()
            try:
                data = self.backend.read_telemetry_module(module)
                # Validate the backend result before putting it into the cache.
                module_data(module, data["joint_positions_deg"],
                            data["end_pose"]["position_mm"] + data["end_pose"]["rpy_deg"], data["state"])
            except TelemetryBusy:
                continue
            except Exception as exc:
                with self._lock:
                    previous = self._records[module]
                    old_error = previous.get("error")
                    previous["error"] = str(exc)
                if self.audit and old_error != str(exc):
                    self.audit("telemetry_error", module=module, error=str(exc))
                continue
            now = time.monotonic()
            with self._lock:
                sequence = self._records[module]["sequence"] + 1
                self._records[module] = {"sequence": sequence, "error": None, "data": copy.deepcopy(data),
                    "sampled_at": datetime.now(timezone.utc).isoformat(), "monotonic": started,
                    "read_duration_ms": round((now - started) * 1000, 3)}

    def _run(self):
        while not self._stop.is_set():
            started = time.monotonic()
            self.poll_once()
            self._stop.wait(max(.01, self.interval - (time.monotonic() - started)))

    def snapshot(self, module=None):
        if module is not None and module not in MODULES:
            raise ValueError("未知上身模块")
        with self._lock:
            records = copy.deepcopy(self._records)
        now = time.monotonic()
        result = {}
        for name in ([module] if module else MODULES):
            record = records[name]
            age = now - record["monotonic"] if "monotonic" in record else None
            fresh = age is not None and age <= self.stale_after and record.get("error") is None
            values = record.get("data") if fresh else {"joint_positions_deg": None, "end_pose": None, "state": None}
            result[name] = {**values, "valid": fresh, "stale": not fresh,
                "sampled_at": record.get("sampled_at"), "age_ms": round(age * 1000, 3) if age is not None else None,
                "sequence": record["sequence"], "read_duration_ms": record.get("read_duration_ms"),
                "error": record.get("error") or (None if fresh else "采样已过期")}
        return {"schema_version": 1, "mode": "hardware" if self.hardware else "mock",
                "poll_interval_seconds": self.interval, "stale_after_seconds": self.stale_after,
                "all_valid": all(item["valid"] for item in result.values()), "modules": result}
