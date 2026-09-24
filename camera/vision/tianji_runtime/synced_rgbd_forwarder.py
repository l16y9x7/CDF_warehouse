"""Time-sync RGB + aligned depth, attach color camera_info, then republish."""

from __future__ import annotations

import os
from pathlib import Path
from typing import Any

import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from message_filters import ApproximateTimeSynchronizer, Subscriber
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CameraInfo, Image


def _default_config_path() -> str:
    try:
        share = get_package_share_directory("retail_nav_bridge")
        return os.path.join(share, "config", "synced_rgbd.yaml")
    except Exception:
        return str(
            Path(__file__).resolve().parents[1] / "config" / "synced_rgbd.yaml"
        )


def load_synced_config(path: str) -> dict[str, Any]:
    with open(path, "r", encoding="utf-8") as stream:
        raw = yaml.safe_load(stream) or {}
    if not isinstance(raw, dict):
        raise ValueError("synced_rgbd config must be a map")
    cameras = raw.get("cameras") or {}
    if not isinstance(cameras, dict) or not cameras:
        raise ValueError("synced_rgbd config requires a non-empty cameras map")
    enabled = []
    for camera_id, cfg in cameras.items():
        if not isinstance(cfg, dict):
            raise ValueError(f"camera {camera_id} config must be a map")
        if cfg.get("enabled", False) is True:
            enabled.append(str(camera_id))
    if not enabled:
        raise ValueError("no camera enabled in synced_rgbd config")
    slop = float(raw.get("slop", 0.1))
    queue_size = int(raw.get("queue_size", 30))
    if slop <= 0.0:
        raise ValueError("slop must be positive")
    if queue_size <= 0:
        raise ValueError("queue_size must be positive")
    reliability = str(raw.get("qos_reliability", "reliable")).strip().lower()
    if reliability not in ("best_effort", "reliable"):
        raise ValueError("qos_reliability must be best_effort or reliable")
    return {
        "cameras": enabled,
        "slop": slop,
        "queue_size": queue_size,
        "qos_reliability": reliability,
    }


class SyncedRgbdForwarder(Node):
    def __init__(self) -> None:
        super().__init__("retail_synced_rgbd_forwarder")
        self.declare_parameter("synced_config_file", _default_config_path())
        config_file = str(self.get_parameter("synced_config_file").value)
        self._config = load_synced_config(config_file)

        reliability = (
            ReliabilityPolicy.BEST_EFFORT
            if self._config["qos_reliability"] == "best_effort"
            else ReliabilityPolicy.RELIABLE
        )
        qos = QoSProfile(
            reliability=reliability,
            history=HistoryPolicy.KEEP_LAST,
            depth=self._config["queue_size"],
        )

        self._syncers = []
        self._subs = []
        self._info_cache: dict[str, CameraInfo] = {}
        self._pubs: dict[str, tuple] = {}

        for camera_id in self._config["cameras"]:
            color_topic = f"/camera/{camera_id}/color/image_raw"
            depth_topic = f"/camera/{camera_id}/aligned_depth_to_color/image_raw"
            info_topic = f"/camera/{camera_id}/color/camera_info"
            out_color = f"/camera/{camera_id}/synced/color/image_raw"
            out_depth = f"/camera/{camera_id}/synced/depth/image_raw"
            out_info = f"/camera/{camera_id}/synced/color/camera_info"

            # 2-way sync (color+depth). camera_info is cached separately so a
            # stamp skew on info does not block RGB-D pairs.
            color_sub = Subscriber(self, Image, color_topic, qos_profile=qos)
            depth_sub = Subscriber(self, Image, depth_topic, qos_profile=qos)
            self._subs.extend([color_sub, depth_sub])
            self.create_subscription(
                CameraInfo,
                info_topic,
                self._make_info_callback(camera_id),
                qos,
            )

            color_pub = self.create_publisher(Image, out_color, qos)
            depth_pub = self.create_publisher(Image, out_depth, qos)
            info_pub = self.create_publisher(CameraInfo, out_info, qos)
            self._pubs[camera_id] = (color_pub, depth_pub, info_pub)

            syncer = ApproximateTimeSynchronizer(
                [color_sub, depth_sub],
                queue_size=self._config["queue_size"],
                slop=self._config["slop"],
            )
            syncer.registerCallback(self._make_callback(camera_id))
            self._syncers.append(syncer)

            self.get_logger().info(
                f"syncing {camera_id}: {color_topic} + {depth_topic} "
                f"(info cache {info_topic}) -> {out_color}, {out_depth}, {out_info}"
            )

        self.get_logger().info(
            f"synced RGB-D forwarder ready cameras={self._config['cameras']} "
            f"slop={self._config['slop']} qos={self._config['qos_reliability']}"
        )

    def _make_info_callback(self, camera_id: str):
        def _on_info(msg: CameraInfo) -> None:
            self._info_cache[camera_id] = msg

        return _on_info

    def _make_callback(self, camera_id: str):
        def _callback(color: Image, depth: Image) -> None:
            info = self._info_cache.get(camera_id)
            if info is None:
                return

            color_pub, depth_pub, info_pub = self._pubs[camera_id]
            stamp = color.header.stamp

            out_color = Image()
            out_color.header = color.header
            out_color.height = color.height
            out_color.width = color.width
            out_color.encoding = color.encoding
            out_color.is_bigendian = color.is_bigendian
            out_color.step = color.step
            out_color.data = color.data

            out_depth = Image()
            out_depth.header = depth.header
            out_depth.header.stamp = stamp
            out_depth.height = depth.height
            out_depth.width = depth.width
            out_depth.encoding = depth.encoding
            out_depth.is_bigendian = depth.is_bigendian
            out_depth.step = depth.step
            out_depth.data = depth.data

            out_info = CameraInfo()
            out_info.header = info.header
            out_info.header.stamp = stamp
            out_info.header.frame_id = color.header.frame_id or info.header.frame_id
            out_info.height = info.height
            out_info.width = info.width
            out_info.distortion_model = info.distortion_model
            out_info.d = list(info.d)
            out_info.k = list(info.k)
            out_info.r = list(info.r)
            out_info.p = list(info.p)
            out_info.binning_x = info.binning_x
            out_info.binning_y = info.binning_y
            out_info.roi = info.roi

            color_pub.publish(out_color)
            depth_pub.publish(out_depth)
            info_pub.publish(out_info)

        return _callback


def main(args=None) -> None:
    rclpy.init(args=args)
    node = SyncedRgbdForwarder()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
