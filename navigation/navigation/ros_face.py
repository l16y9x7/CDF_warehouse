"""可选 ROS 脸：订阅天机位姿/状态，写入 Adapter。HTTP 与 ROS 共用同一份 Service。"""

from __future__ import annotations

import logging
import threading
from typing import Any, Optional

LOGGER = logging.getLogger(__name__)


class RosStatusBridge:
    def __init__(self, adapter: Any, config: dict) -> None:
        self.adapter = adapter
        ros_cfg = config.get("ros") or {}
        self.enabled = bool(ros_cfg.get("enabled", False))
        self.pose_topic = str(ros_cfg.get("pose_topic") or "/retail_nav/v1/pose")
        self.status_topic = str(ros_cfg.get("status_topic") or "/retail_nav/v1/status")
        self.event_topic = str(ros_cfg.get("event_topic") or "/retail_nav/v1/event")
        self._node = None
        self._thread: Optional[threading.Thread] = None
        self._ok = False

    def start(self) -> None:
        if not self.enabled:
            LOGGER.info("navigation ROS face disabled")
            return
        try:
            import rclpy
            from rclpy.node import Node
            from retail_nav_msgs.msg import NavEvent, NavPose, NavStatus
        except Exception as exc:
            LOGGER.warning("navigation ROS face unavailable: %s", exc)
            return

        if not rclpy.ok():
            rclpy.init()
        node = Node("capability_navigation")

        def on_pose(msg: Any) -> None:
            if hasattr(self.adapter, "note_pose"):
                self.adapter.note_pose(float(msg.x), float(msg.y), float(msg.yaw))

        def on_status(msg: Any) -> None:
            if hasattr(self.adapter, "note_status"):
                self.adapter.note_status(
                    nav_state=str(msg.nav_state or ""),
                    station_id=str(msg.active_station_id or ""),
                    error_msg=str(msg.error_msg or ""),
                )

        def on_event(msg: Any) -> None:
            if str(msg.type or "").upper() == "ARRIVED" and hasattr(self.adapter, "note_status"):
                self.adapter.note_status(nav_state="ARRIVED", station_id=str(msg.station_id or ""))

        node.create_subscription(NavPose, self.pose_topic, on_pose, 10)
        node.create_subscription(NavStatus, self.status_topic, on_status, 10)
        node.create_subscription(NavEvent, self.event_topic, on_event, 10)
        self._node = node
        self._ok = True
        self._thread = threading.Thread(target=rclpy.spin, args=(node,), name="nav-ros", daemon=True)
        self._thread.start()
        LOGGER.info(
            "navigation ROS face started: pose=%s status=%s",
            self.pose_topic,
            self.status_topic,
        )

    def stop(self) -> None:
        if self._node is None:
            return
        try:
            self._node.destroy_node()
        except Exception:
            pass
        self._node = None
        LOGGER.info("navigation ROS face stopped")
