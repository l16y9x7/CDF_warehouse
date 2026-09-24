"""独立 trajectory Topic 周期上报真实位姿；无效值本周期不发送。"""

from __future__ import annotations

import logging
import math
import threading
import time
from typing import Any, Dict, Mapping, Optional

from gateway.mqtt.osd import current_pose

LOGGER = logging.getLogger(__name__)


class TrajectoryReporter:
    def __init__(
        self,
        *,
        config: Mapping[str, Any],
        mqtt_client: Any,
        collector,
    ) -> None:
        self.mqtt_client = mqtt_client
        self.collector = collector
        trajectory_cfg = config.get("trajectory") or {}
        device_cfg = config.get("device") or {}
        self.enabled = bool(trajectory_cfg.get("enabled", True))
        self.report_interval = max(
            float(trajectory_cfg.get("report_interval_sec") or 1.0), 0.1
        )
        self.map_frame = str(trajectory_cfg.get("map_frame") or "map")
        self.robot_frame = str(trajectory_cfg.get("robot_frame") or "base_link")
        self.device_sn = str(
            getattr(mqtt_client, "device_sn", "") or device_cfg.get("sn") or ""
        )
        self.total_reported = 0
        self._stop_event = threading.Event()
        self._thread: Optional[threading.Thread] = None
        self._last_mqtt_warning_time = 0.0
        self._last_runtime_log_time = 0.0

    def start(self) -> None:
        if not self.enabled:
            LOGGER.info("trajectory reporter disabled")
            return
        topic = str(getattr(self.mqtt_client, "topic_trajectory", "") or "")
        if not topic:
            LOGGER.warning("trajectory topic is not configured; reporter disabled")
            return
        if self._thread is not None and self._thread.is_alive():
            return
        self._stop_event.clear()
        self._thread = threading.Thread(
            target=self._report_loop, name="trajectory-reporter", daemon=True
        )
        self._thread.start()
        LOGGER.info(
            "trajectory reporter started: interval=%ss topic=%s",
            self.report_interval,
            topic,
        )

    def stop(self) -> None:
        self._stop_event.set()
        if self._thread is not None:
            self._thread.join(timeout=2.0)
        self._thread = None
        LOGGER.info("trajectory reporter stopped")

    def report_now(self) -> bool:
        if not self.enabled:
            return False
        if not bool(getattr(self.mqtt_client, "is_connected", False)):
            now = time.time()
            if now - self._last_mqtt_warning_time > 30.0:
                LOGGER.warning("MQTT not connected; skip trajectory publish")
                self._last_mqtt_warning_time = now
            return False
        pose = current_pose(
            self.collector.snapshot(),
            map_sync=getattr(self.collector, "map_sync", None),
        )
        if pose is None:
            return False
        qx, qy, qz, qw = _quaternion_from_yaw(pose["yaw"])
        payload: Dict[str, Any] = {
            "timestamp": int(time.time() * 1000),
            "device_sn": self.device_sn,
            "frame_id": self.map_frame,
            "child_frame_id": self.robot_frame,
            "translation": {"x": pose["x"], "y": pose["y"], "z": 0.0},
            "rotation": {"x": qx, "y": qy, "z": qz, "w": qw},
        }
        if pose.get("map_id"):
            payload["map_id"] = pose["map_id"]
        try:
            success = bool(self.mqtt_client.publish_trajectory(payload))
        except Exception as exc:
            LOGGER.warning("trajectory publish failed: %s", exc, exc_info=True)
            return False
        if success:
            self.total_reported += 1
            now = time.time()
            if now - self._last_runtime_log_time >= 30.0:
                LOGGER.debug(
                    "trajectory publishing: count=%s x=%.3f y=%.3f yaw=%.3f map=%s",
                    self.total_reported,
                    pose["x"],
                    pose["y"],
                    pose["yaw"],
                    pose.get("map_id") or "",
                )
                self._last_runtime_log_time = now
        return success

    def _report_loop(self) -> None:
        while not self._stop_event.is_set():
            try:
                self.report_now()
            except Exception as exc:
                LOGGER.warning("trajectory report failed: %s", exc, exc_info=True)
            self._stop_event.wait(self.report_interval)


def _quaternion_from_yaw(yaw: float) -> tuple[float, float, float, float]:
    half = float(yaw) * 0.5
    return (0.0, 0.0, math.sin(half), math.cos(half))
