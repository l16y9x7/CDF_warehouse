"""Publish one configured RealSense wrist color stream directly to ROS 2."""

from __future__ import annotations

import argparse
import json
import logging
import signal
import threading
from pathlib import Path

from vision.rokae_runtime.driver_supervisor import (
    validate_realsense_color_profile,
    validate_unique_realsense_owner,
)
from vision.rokae_runtime.ros_contract import ros_camera_id


LOGGER = logging.getLogger(__name__)
WRIST_ROLES = ("hand_left", "hand_right")


def load_source(config_path: str | Path, camera: str):
    """Load and validate a color-only RealSense wrist source."""
    if camera not in WRIST_ROLES:
        raise ValueError(
            f"RealSense color source requires a wrist role: {camera}"
        )
    data = json.loads(Path(config_path).resolve().read_text())
    cameras = (
        ((data.get("rokae") or {}).get("owner") or {}).get("cameras") or {}
    )
    cfg = cameras.get(camera, {})
    if not isinstance(cfg, dict):
        raise ValueError(f"Camera configuration must be an object: {camera}")
    if cfg.get("enabled") is not True:
        raise ValueError(f"Cannot launch a disabled camera: {camera}")
    flags = {
        name: cfg.get(name, default)
        for name, default in (
            ("enable_depth", False),
            ("require_depth", False),
            ("require_synced", False),
        )
    }
    if not all(isinstance(value, bool) for value in flags.values()):
        raise ValueError("Depth and synchronization flags must be booleans")
    if any(flags.values()):
        raise ValueError(
            f"Direct RealSense color source forbids depth/synced: {camera}"
        )
    validate_unique_realsense_owner(data, camera)
    return data, cfg


def publish(
    config_path: str | Path, camera: str, *, rs_module=None, ros=None
) -> None:
    """Open one device/profile and publish RGB8 frames until stopped."""
    _data, cfg = load_source(config_path, camera)
    if rs_module is None:
        try:
            import pyrealsense2 as rs_module
        except ImportError as exc:
            raise RuntimeError(
                "pyrealsense2 is missing from the selected Python runtime; "
                "install it inside Conda nav (user-site packages are disabled)"
            ) from exc
    width, height, fps = validate_realsense_color_profile(cfg, rs_module)
    serial = str((cfg.get("match") or {}).get("value", "")).strip()

    if ros is None:
        import rclpy
        from rclpy.node import Node
        from rclpy.qos import qos_profile_sensor_data
        from rclpy.signals import SignalHandlerOptions
        from sensor_msgs.msg import Image

        ros = {
            "rclpy": rclpy,
            "Node": Node,
            "Image": Image,
            "qos": qos_profile_sensor_data,
            "signal_options": SignalHandlerOptions.NO,
        }

    rclpy = ros["rclpy"]
    stopping = threading.Event()
    pipeline = rs_module.pipeline()
    pipeline_config = rs_module.config()
    pipeline_config.enable_device(serial)
    pipeline_config.enable_stream(
        rs_module.stream.color,
        width,
        height,
        rs_module.format.rgb8,
        fps,
    )
    ros_id = ros_camera_id(camera)
    topic = f"/camera/{ros_id}/color/image_raw"
    frame_id = f"{ros_id}_color_optical_frame"
    node = None
    started = False
    ros_initialized = False

    def request_stop(*_args):
        stopping.set()

    old_handlers = {}
    for signum in (signal.SIGINT, signal.SIGTERM):
        try:
            old_handlers[signum] = signal.signal(signum, request_stop)
        except ValueError:
            # Unit tests may invoke the publisher outside the main thread.
            pass

    try:
        rclpy.init(signal_handler_options=ros["signal_options"])
        ros_initialized = True
        node = ros["Node"](f"vision_{ros_id}_color_source")
        publisher = node.create_publisher(ros["Image"], topic, ros["qos"])
        pipeline.start(pipeline_config)
        started = True
        LOGGER.info(
            "RealSense color source started camera=%s serial=%s "
            "profile=%dx%d@%d topic=%s",
            camera, serial, width, height, fps, topic,
        )
        while not stopping.is_set() and rclpy.ok():
            try:
                frames = pipeline.wait_for_frames(1000)
            except RuntimeError as exc:
                if stopping.is_set():
                    break
                raise RuntimeError(
                    f"RealSense color frame wait failed: {exc}"
                ) from exc
            color = frames.get_color_frame() if frames is not None else None
            if color is None:
                continue
            if (
                int(color.get_width()) != width
                or int(color.get_height()) != height
            ):
                raise RuntimeError(
                    f"RealSense negotiated unexpected color frame "
                    f"{color.get_width()}x{color.get_height()}; "
                    f"expected {width}x{height}"
                )
            payload = bytes(color.get_data())
            expected = width * height * 3
            if len(payload) != expected:
                raise RuntimeError(
                    f"RealSense RGB8 payload has {len(payload)} bytes; "
                    f"expected {expected}"
                )
            message = ros["Image"]()
            message.header.stamp = node.get_clock().now().to_msg()
            message.header.frame_id = frame_id
            message.height = height
            message.width = width
            message.encoding = "rgb8"
            message.is_bigendian = 0
            message.step = width * 3
            message.data = payload
            publisher.publish(message)
    finally:
        stopping.set()
        if started:
            try:
                pipeline.stop()
            except Exception:
                LOGGER.exception(
                    "RealSense pipeline stop failed camera=%s", camera
                )
        if node is not None:
            try:
                node.destroy_node()
            except Exception:
                LOGGER.exception("ROS node cleanup failed camera=%s", camera)
        if ros_initialized:
            try:
                rclpy.shutdown()
            except Exception:
                LOGGER.exception("ROS shutdown failed camera=%s", camera)
        for signum, handler in old_handlers.items():
            signal.signal(signum, handler)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", required=True)
    parser.add_argument("--camera", choices=WRIST_ROLES, required=True)
    args = parser.parse_args()
    logging.basicConfig(level=logging.INFO)
    publish(args.config, args.camera)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
