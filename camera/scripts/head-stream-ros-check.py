"""Bounded ROS freshness probe for every enabled ROKAE RGB-D camera."""

from __future__ import annotations

import argparse
import json
import time
from pathlib import Path


ROS_IDS = {
    "head": "head",
    "hand_left": "left_wrist",
    "hand_right": "right_wrist",
}


def enabled_ros_ids(config):
    cameras = ((config.get("rokae") or {}).get("owner") or {}).get("cameras") or {}
    return [
        ROS_IDS[camera_id]
        for camera_id in ROS_IDS
        if camera_id in cameras and (cameras.get(camera_id) or {}).get("enabled", True) is not False
    ]


def camera_requirements(config):
    health = config.get("health") or {}
    overrides = health.get("ros_requirements") or {}
    requirements = []
    for internal_id, ros_id in ROS_IDS.items():
        cameras = ((config.get("rokae") or {}).get("owner") or {}).get("cameras") or {}
        if internal_id not in cameras or (cameras.get(internal_id) or {}).get("enabled", True) is False:
            continue
        override = overrides.get(internal_id) or overrides.get(ros_id) or {}
        camera_cfg = cameras.get(internal_id) or {}
        require_depth_value = override.get(
            "require_depth",
            camera_cfg.get("require_depth", camera_cfg.get("enable_depth", True)),
        )
        require_synced_value = override.get(
            "require_synced", camera_cfg.get("require_synced", require_depth_value)
        )
        if not isinstance(require_depth_value, bool) or not isinstance(require_synced_value, bool):
            raise ValueError(f"{internal_id}: depth requirements must be booleans")
        require_depth = require_depth_value
        require_synced = require_synced_value
        if require_depth and camera_cfg.get("enable_depth", True) is False:
            raise ValueError(f"{internal_id}: required depth is disabled")
        if require_synced and not require_depth:
            raise ValueError(f"{internal_id}: synchronized RGB-D requires depth")
        depth_topic = str(
            override.get("depth_topic")
            or f"/camera/{ros_id}/aligned_depth_to_color/image_raw"
        )
        requirements.append({
            "internal_id": internal_id,
            "ros_id": ros_id,
            "depth_topic": depth_topic,
            "require_depth": require_depth,
            "require_synced": require_synced,
            "repair_unit": str(override.get("repair_unit") or f"{internal_id}-source"),
        })
    return requirements


def probe(config, *, timeout_sec=8.0):
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import CameraInfo, Image

    requirements = camera_requirements(config)
    if not requirements:
        return {"ok": False, "raw_ok": False, "synced_ok": False,
                "cameras": {}, "error": "NO_ENABLED_CAMERA"}

    rclpy.init(args=None)
    node = Node("vision_rgbd_health_probe")
    received = {item["ros_id"]: set() for item in requirements}
    subscriptions = []

    def mark(camera_id, key):
        return lambda _message: received[camera_id].add(key)

    try:
        required_by_camera = {}
        for item in requirements:
            camera_id = item["ros_id"]
            topics = [(Image, f"/camera/{camera_id}/color/image_raw", "raw_color",
                       qos_profile_sensor_data)]
            required = {"raw_color"}
            if item["require_depth"]:
                topics.append((Image, item["depth_topic"], "raw_depth", qos_profile_sensor_data))
                required.add("raw_depth")
            if item["require_synced"]:
                topics.extend([
                    # BEST_EFFORT subscribers are compatible with both the
                    # field BEST_EFFORT forwarder and RELIABLE publishers.
                    # An integer depth creates a RELIABLE subscription and
                    # silently misses the former with an incompatible-QoS
                    # warning.
                    (Image, f"/camera/{camera_id}/synced/color/image_raw",
                     "synced_color", qos_profile_sensor_data),
                    (Image, f"/camera/{camera_id}/synced/depth/image_raw",
                     "synced_depth", qos_profile_sensor_data),
                    (CameraInfo,
                     f"/camera/{camera_id}/synced/color/camera_info",
                     "synced_info", qos_profile_sensor_data),
                ])
                required.update({"synced_color", "synced_depth", "synced_info"})
            required_by_camera[camera_id] = required
            for message_type, topic, key, qos in topics:
                subscriptions.append(node.create_subscription(
                    message_type, topic, mark(camera_id, key), qos,
                ))
        deadline = time.monotonic() + timeout_sec
        while time.monotonic() < deadline and any(
            not required_by_camera[camera_id].issubset(values)
            for camera_id, values in received.items()
        ):
            rclpy.spin_once(node, timeout_sec=0.2)
    finally:
        node.destroy_node()
        rclpy.shutdown()

    cameras = {}
    by_ros_id = {item["ros_id"]: item for item in requirements}
    for camera_id, values in received.items():
        item = by_ros_id[camera_id]
        raw_required = {"raw_color"}
        if item["require_depth"]:
            raw_required.add("raw_depth")
        raw_ok = raw_required.issubset(values)
        synced_ok = (
            not item["require_synced"]
            or {"synced_color", "synced_depth", "synced_info"}.issubset(values)
        )
        cameras[camera_id] = {
            "raw_ok": raw_ok,
            "synced_ok": synced_ok,
            "synced_required": item["require_synced"],
            "depth_required": item["require_depth"],
            "repair_unit": item["repair_unit"],
            "depth_topic": item["depth_topic"],
            "received": sorted(values),
        }
    raw_ok = all(row["raw_ok"] for row in cameras.values())
    synced_ok = all(row["synced_ok"] for row in cameras.values())
    return {"ok": raw_ok and synced_ok, "raw_ok": raw_ok,
            "synced_ok": synced_ok, "cameras": cameras}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", required=True)
    parser.add_argument("--timeout", type=float, default=8.0)
    args = parser.parse_args()
    config = json.loads(Path(args.config).read_text())
    result = probe(config, timeout_sec=max(0.1, args.timeout))
    print(json.dumps(result, ensure_ascii=False))
    return 0 if result["ok"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
