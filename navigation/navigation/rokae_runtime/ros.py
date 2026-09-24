"""车上后端：sr_amr_control 的 ROS 客户端。"""

from __future__ import annotations

import logging
import math
import threading
import time
from typing import Any, Dict, Optional

LOGGER = logging.getLogger(__name__)


def _yaw_from_quat(orientation: Any) -> float:
    x = float(getattr(orientation, "x", 0.0) or 0.0)
    y = float(getattr(orientation, "y", 0.0) or 0.0)
    z = float(getattr(orientation, "z", 0.0) or 0.0)
    w = float(getattr(orientation, "w", 1.0) or 1.0)
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def _wait_future(future: Any, timeout_sec: float) -> Any:
    deadline = time.monotonic() + max(0.1, float(timeout_sec))
    while not future.done():
        if time.monotonic() > deadline:
            return None
        time.sleep(0.05)
    try:
        return future.result()
    except Exception as exc:  # noqa: BLE001
        LOGGER.warning("rokae ROS future failed: %s", exc)
        return None


class RokaeRosClient:
    def __init__(self, config: Dict[str, Any]) -> None:
        rokae = config.get("rokae") or {}
        self.action_name = str(
            rokae.get("move_to_station_action") or "/sr_amr_control/move_to_station"
        )
        self.state_topic = str(
            rokae.get("system_state_topic") or "/sr_amr_control/system_state"
        )
        self.battery_topic = str(
            rokae.get("battery_topic") or "/sr_amr_control/battery_state"
        )
        self.server_wait_sec = float(rokae.get("server_wait_sec", 3.0))
        self._lock = threading.Lock()
        self._state: Optional[Dict[str, Any]] = None
        self._battery: Optional[Dict[str, Any]] = None
        self._node = None
        self._move_client = None
        self._ok = False
        self.start()

    def start(self) -> None:
        try:
            import rclpy
            from rclpy.action import ActionClient
            from rclpy.callback_groups import ReentrantCallbackGroup
            from rclpy.executors import MultiThreadedExecutor
            from rclpy.node import Node
            from sr_amr_interfaces.action import MoveToStation
            from sr_amr_interfaces.msg import BatteryState, SystemState
        except Exception as exc:
            LOGGER.warning("rokae ROS unavailable: %s", exc)
            return

        if not rclpy.ok():
            rclpy.init()
        node = Node("capability_navigation_rokae")
        callback_group = ReentrantCallbackGroup()
        self._move_client = ActionClient(
            node,
            MoveToStation,
            self.action_name,
            callback_group=callback_group,
        )
        node.create_subscription(
            SystemState,
            self.state_topic,
            self._on_system_state,
            10,
            callback_group=callback_group,
        )
        node.create_subscription(
            BatteryState,
            self.battery_topic,
            self._on_battery,
            10,
            callback_group=callback_group,
        )
        executor = MultiThreadedExecutor(num_threads=2)
        executor.add_node(node)
        self._node = node
        self._executor = executor
        self._MoveToStation = MoveToStation
        self._ok = True
        threading.Thread(target=executor.spin, name="nav-rokae-ros", daemon=True).start()
        LOGGER.info(
            "rokae ROS started: action=%s state=%s battery=%s",
            self.action_name,
            self.state_topic,
            self.battery_topic,
        )

    def _on_system_state(self, msg: Any) -> None:
        pose = getattr(msg, "current_pose", None)
        position = getattr(getattr(pose, "pose", None), "position", None)
        orientation = getattr(getattr(pose, "pose", None), "orientation", None)
        snapshot: Dict[str, Any] = {
            "location_state": int(msg.location_state),
            "estop_active": bool(msg.estop_active),
            "executing_movement_task": bool(msg.executing_movement_task),
            "linear_velocity_x": float(msg.linear_velocity_x),
            "linear_velocity_y": float(msg.linear_velocity_y),
            "angular_velocity": float(msg.angular_velocity),
            "map_name": str(msg.current_map_name or ""),
            "current_station_id": int(msg.current_station_id or 0),
        }
        if position is not None:
            snapshot["x"] = float(position.x)
            snapshot["y"] = float(position.y)
            snapshot["yaw"] = _yaw_from_quat(orientation) if orientation is not None else 0.0
        with self._lock:
            self._state = snapshot

    def _on_battery(self, msg: Any) -> None:
        charging_const = getattr(type(msg), "STATE_CHARING", 1)
        voltage_mv = float(msg.voltage)
        battery: Dict[str, Any] = {
            "capacity_percent": float(msg.remaining_percentage),
            "temperature": float(msg.temperature),
            "charging": int(msg.state) == int(charging_const),
            "cycle": int(msg.use_cycles),
        }
        if voltage_mv > 0:
            battery["voltage"] = round(voltage_mv / 1000.0, 3)
        with self._lock:
            self._battery = battery

    def latest_state(self) -> Optional[Dict[str, Any]]:
        with self._lock:
            return dict(self._state) if self._state else None

    def latest_battery(self) -> Optional[Dict[str, Any]]:
        with self._lock:
            return dict(self._battery) if self._battery else None

    def move_to_station(self, station_id: int, timeout_sec: float) -> Dict[str, Any]:
        client = self._move_client
        move_cls = getattr(self, "_MoveToStation", None)
        if client is None or move_cls is None:
            return {"accepted": False, "unavailable": True, "message": "rokae ROS not started"}
        wait_sec = min(self.server_wait_sec, max(0.1, float(timeout_sec)))
        if not client.wait_for_server(timeout_sec=wait_sec):
            return {
                "accepted": False,
                "unavailable": True,
                "message": f"{self.action_name} unavailable",
            }
        goal = move_cls.Goal()
        goal.station_id = int(station_id)
        handle = _wait_future(client.send_goal_async(goal), 3.0)
        if handle is None or not handle.accepted:
            return {"accepted": False, "message": f"vendor rejected station_id={station_id}"}
        wrapped = _wait_future(handle.get_result_async(), max(0.1, float(timeout_sec)))
        if wrapped is None:
            return {
                "accepted": True,
                "timed_out": True,
                "message": "move_to_station result timeout",
            }
        vendor_result = getattr(wrapped, "result", None)
        if vendor_result is None:
            return {
                "accepted": True,
                "result": False,
                "error_code": "NAVIGATION_VENDOR",
                "message": "move_to_station empty result",
            }
        return {
            "accepted": True,
            "result": bool(vendor_result.result),
            "error_code": str(vendor_result.error_code or ""),
        }

    def close(self) -> None:
        executor = getattr(self, "_executor", None)
        if executor is not None:
            try:
                executor.shutdown()
            except Exception:
                pass
        if self._node is not None:
            try:
                self._node.destroy_node()
            except Exception:
                pass
        self._node = None
        self._move_client = None
        self._ok = False
