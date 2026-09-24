"""Exit an unhealthy native RGB-D launch so systemd restarts its entire group."""
from __future__ import annotations

import argparse
import fcntl
import json
import logging
import os
import signal
import subprocess
import threading
import time
from pathlib import Path

from vision.rokae_runtime.ros_contract import ros_camera_id

LOGGER = logging.getLogger(__name__)

# USBDEVFS_RESET = _IO('U', 20)
_USBDEVFS_RESET = (ord("U") << 8) | 20
_ORBBEC_VID_PID = ("2bc5", "0804")


def validate_realsense_color_profile(camera_cfg, rs_module=None):
    """Fail before ROS launch when the selected device cannot provide the profile."""
    if str(camera_cfg.get("backend", "realsense")).strip().lower() != "realsense":
        raise ValueError("RealSense source requires backend=realsense")
    match = camera_cfg.get("match") or {}
    if match.get("type") != "realsense_serial":
        raise ValueError("RealSense source requires match.type=realsense_serial")
    serial = str(match.get("value", "")).strip()
    width = int(camera_cfg.get("width", 640))
    height = int(camera_cfg.get("height", 480))
    fps = int(camera_cfg.get("fps", camera_cfg.get("ros_fps", 15)))
    if not serial or min(width, height, fps) <= 0:
        raise ValueError("RealSense serial and positive color profile are required")
    if rs_module is None:
        import pyrealsense2 as rs_module
    selected = None
    for device in rs_module.context().query_devices():
        if device.get_info(rs_module.camera_info.serial_number) == serial:
            selected = device
            break
    if selected is None:
        raise RuntimeError(f"RealSense serial not found: {serial}")
    available = set()
    for sensor in selected.query_sensors():
        for profile in sensor.get_stream_profiles():
            try:
                video = profile.as_video_stream_profile()
                if (
                    profile.stream_type() == rs_module.stream.color
                    and profile.format() == rs_module.format.rgb8
                ):
                    available.add((video.width(), video.height(), profile.fps()))
            except (AttributeError, RuntimeError):
                continue
    requested = (width, height, fps)
    if requested not in available:
        rendered = ", ".join(f"{w}x{h}@{rate}" for w, h, rate in sorted(available)) or "none"
        raise RuntimeError(
            f"Unsupported RealSense color profile {width}x{height}@{fps} for {serial}; "
            f"available={rendered}"
        )
    return requested


def validate_unique_realsense_owner(data, camera):
    """Reject two enabled ROS source units that claim the same RealSense serial."""
    cameras = (((data.get("rokae") or {}).get("owner") or {}).get("cameras") or {})
    selected = cameras.get(camera) or {}
    selected_match = selected.get("match") or {}
    if selected_match.get("type") != "realsense_serial":
        return
    serial = str(selected_match.get("value", "")).strip()
    if not serial:
        return
    conflicts = []
    for role, cfg in cameras.items():
        if (
            role == camera
            or not isinstance(cfg, dict)
            or cfg.get("enabled", True) is False
        ):
            continue
        match = cfg.get("match") or {}
        backend = str(cfg.get("backend", "realsense")).strip().lower()
        if (
            backend == "realsense"
            and match.get("type") == "realsense_serial"
            and str(match.get("value", "")).strip() == serial
        ):
            conflicts.append(str(role))
    if conflicts:
        raise ValueError(
            f"RealSense serial {serial} is assigned to multiple enabled roles: "
            f"{camera}, {', '.join(sorted(conflicts))}"
        )


class RgbdLiveness:
    def __init__(self, started: float, startup_timeout=60.0, stale_timeout=15.0):
        self.started, self.startup_timeout, self.stale_timeout = started, startup_timeout, stale_timeout
        self.last = {}
        self.was_ready = False

    def record(self, stream: str, now: float):
        if stream not in {"color", "depth", "info"}:
            raise ValueError("unknown RGB-D stream")
        self.last[stream] = now
        # Color alone is enough to mark ready: Media only needs RGB. Depth/info
        # hiccups must not block readiness (Tianji: platform push ≠ RGB-D health).
        color = self.last.get("color")
        if color is not None and now - color < self.stale_timeout:
            self.was_ready = True

    def failure(self, now: float):
        if not self.was_ready:
            return "RGB-D startup timeout" if now - self.started > self.startup_timeout else ""
        # Steady-state: only color loss takes down the launch. Depth/info stalls
        # must not SIGINT Orbbec into USB Access-denied restart loops.
        color = self.last.get("color")
        if color is None or now - color > self.stale_timeout:
            return "RGB-D stream stalled"
        return ""


def stop_child(child):
    try:
        os.killpg(child.pid, signal.SIGINT)
    except ProcessLookupError:
        pass
    try:
        child.wait(timeout=10)
    except subprocess.TimeoutExpired:
        try:
            os.killpg(child.pid, signal.SIGKILL)
        except ProcessLookupError:
            pass
        child.wait(timeout=3)


def reset_orbbec_usb(vid_pid=_ORBBEC_VID_PID):
    """Release a stuck Orbbec handle so systemd can reopen after supervisor exit."""
    vid, pid = vid_pid
    sys_root = Path("/sys/bus/usb/devices")
    if not sys_root.is_dir():
        return
    for device in sys_root.iterdir():
        try:
            if (device / "idVendor").read_text().strip() != vid:
                continue
            if (device / "idProduct").read_text().strip() != pid:
                continue
            bus = (device / "busnum").read_text().strip().zfill(3)
            dev = (device / "devnum").read_text().strip().zfill(3)
        except OSError:
            continue
        path = f"/dev/bus/usb/{bus}/{dev}"
        try:
            fd = os.open(path, os.O_RDWR)
        except OSError as exc:
            LOGGER.warning("orbbec usb open failed path=%s err=%s", path, exc)
            continue
        try:
            fcntl.ioctl(fd, _USBDEVFS_RESET, 0)
            LOGGER.info("orbbec usb reset path=%s", path)
        except OSError as exc:
            LOGGER.warning("orbbec usb reset failed path=%s err=%s", path, exc)
        finally:
            os.close(fd)
        return


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", required=True)
    parser.add_argument("--camera", choices=["head", "hand_left", "hand_right"], required=True)
    args = parser.parse_args()
    config = Path(args.config).resolve()
    data = json.loads(config.read_text())
    if data["rokae"]["owner"]["cameras"].get(args.camera, {}).get("enabled") is not True:
        raise ValueError("Cannot launch a disabled camera")
    camera_cfg = data["rokae"]["owner"]["cameras"][args.camera]
    if args.camera != "head":
        validate_unique_realsense_owner(data, args.camera)
        validate_realsense_color_profile(camera_cfg)
    logging.basicConfig(level=logging.INFO)
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from rclpy.signals import SignalHandlerOptions
    from sensor_msgs.msg import Image, CameraInfo

    stopping = threading.Event()
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = Node(f"vision_{args.camera}_rgbd_supervisor")
    child = None
    signal.signal(signal.SIGINT, lambda *_: stopping.set())
    signal.signal(signal.SIGTERM, lambda *_: stopping.set())
    live = RgbdLiveness(time.monotonic(), stale_timeout=45.0)
    reset_usb = args.camera == "head"

    def receive(stream, msg):
        # Liveness tracks delivery, not device-clock freshness. Orbbec on USB2
        # can keep publishing while stamp skew exceeds Media's encode gate; killing
        # the launch for skew alone causes the head crash loop seen on ROBOT_001.
        if msg.width <= 0 or msg.height <= 0:
            return
        if stream == "info":
            if msg.k[0] <= 0 or msg.k[4] <= 0:
                return
        elif not msg.data:
            return
        live.record(stream, time.monotonic())

    try:
        for stream, suffix, kind in [
            ("color", "color/image_raw", Image),
            ("depth", "depth/image_rect_raw", Image),
            ("depth", "aligned_depth_to_color/image_raw", Image),
            ("info", "color/camera_info", CameraInfo),
        ]:
            node.create_subscription(kind, f"/camera/{ros_camera_id(args.camera)}/{suffix}",
                                     lambda msg, stream=stream: receive(stream, msg), qos_profile_sensor_data)
        launch_file = Path(__file__).resolve().parents[2] / "config/ros/rgbd.launch.py"
        child = subprocess.Popen(["ros2", "launch", str(launch_file),
                                  "vision_config:=" + str(config), "camera:=" + args.camera],
                                 start_new_session=True)
        while not stopping.is_set():
            rclpy.spin_once(node, timeout_sec=0.25)
            if child.poll() is not None:
                raise RuntimeError(f"Native ROS launch exited: {child.returncode}")
            error = live.failure(time.monotonic())
            if error:
                raise RuntimeError(error)
    finally:
        if child is not None:
            stop_child(child)
        if reset_usb:
            # Give the kernel a moment after SIGINT before bus reset.
            time.sleep(1.0)
            reset_orbbec_usb()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
