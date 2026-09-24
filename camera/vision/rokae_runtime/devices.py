"""V4L2 设备发现：entity name / USB 端口路径（禁止依赖漂移的 /dev/videoN）。"""

from __future__ import annotations

import glob
import logging
import os
from pathlib import Path
from typing import List, Optional

LOGGER = logging.getLogger(__name__)


def _fourcc_name(cap) -> str:
    import cv2

    fourcc = int(cap.get(cv2.CAP_PROP_FOURCC))
    return "".join(chr((fourcc >> (8 * i)) & 0xFF) for i in range(4))


def _probe_color_node(node: str, width: int, height: int, fourcc: str = "") -> bool:
    import cv2

    cap = cv2.VideoCapture(node, cv2.CAP_V4L2)
    if not cap.isOpened():
        cap.release()
        return False
    try:
        if fourcc:
            cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*fourcc))
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
        if _fourcc_name(cap) not in {"YUYV", "MJPG", "UYVY"}:
            return False
        for _ in range(5):
            ok, frame = cap.read()
            if ok and frame is not None:
                return True
        return False
    except Exception:
        LOGGER.warning("probe failed node=%s", node, exc_info=True)
        return False
    finally:
        cap.release()


def find_by_entity(entity: str, width: int = 640, height: int = 480, fourcc: str = "", exclude: Optional[set[str]] = None) -> Optional[str]:
    """按 sysfs video4linux name 匹配 Orbbec 等设备的彩色节点。"""
    entity = str(entity or "").strip()
    if not entity:
        return None
    candidates: List[str] = []
    for vid in sorted(
        glob.glob("/sys/class/video4linux/video*"),
        key=lambda p: int(p.rsplit("video", 1)[-1]),
    ):
        try:
            name = Path(vid, "name").read_text(encoding="utf-8").strip()
        except OSError:
            continue
        if entity.lower() not in name.lower():
            continue
        node = "/dev/" + os.path.basename(vid)
        if os.path.exists(node):
            candidates.append(node)

    # Keep the verified formats; format names alone do not prove image quality.
    color_fourccs = {"YUYV", "MJPG", "UYVY"}
    import cv2

    for node in candidates:
        if os.path.realpath(node) in (exclude or set()):
            continue
        cap = cv2.VideoCapture(node, cv2.CAP_V4L2)
        if not cap.isOpened():
            cap.release()
            continue
        try:
            if fourcc:
                cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*fourcc))
            cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
            cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
            cc = _fourcc_name(cap)
            if cc not in color_fourccs:
                continue
            ok = False
            for _ in range(5):
                r, frame = cap.read()
                if r and frame is not None:
                    ok = True
                    break
        except Exception:
            LOGGER.warning("probe failed node=%s", node, exc_info=True)
            continue
        finally:
            cap.release()
        if not ok:
            continue
        if cc in color_fourccs:
            LOGGER.info("entity %r -> %s (fourcc=%s)", entity, node, cc)
            return node
        LOGGER.info("entity %r skip %s (fourcc=%s not color)", entity, node, cc)
    return None


def find_by_usb_path(usb_path: str, width: int = 640, height: int = 480, fourcc: str = "", exclude: Optional[set[str]] = None) -> Optional[str]:
    """按 USB 物理端口路径匹配（D435i serial 常不在 sysfs）。"""
    usb_path = str(usb_path or "").strip().strip("/")
    if not usb_path:
        return None
    matches: List[str] = []
    for vid in glob.glob("/sys/class/video4linux/video*"):
        device_link = Path(vid) / "device"
        try:
            link = str(device_link.resolve())
        except OSError:
            continue
        if f"/{usb_path}/" not in link and f"/{usb_path}:" not in link:
            continue
        node = "/dev/" + os.path.basename(vid)
        if os.path.exists(node):
            matches.append(node)
    matches = sorted(matches, key=lambda n: int(n.replace("/dev/video", "") or 0))
    for node in matches:
        if os.path.realpath(node) in (exclude or set()):
            continue
        if _probe_color_node(node, width, height, **({"fourcc": fourcc} if fourcc else {})):
            LOGGER.info("usb_path %r -> %s", usb_path, node)
            return node
    return None


def find_by_usb_serial(serial: str, width: int = 640, height: int = 480,
                       fourcc: str = "", exclude: Optional[set[str]] = None) -> Optional[str]:
    """Match the camera's USB serial, never a hub serial or a drifting node number."""
    serial = str(serial or "").strip()
    if not serial:
        return None
    candidates = []
    physical_devices = set()
    for vid in glob.glob("/sys/class/video4linux/video*"):
        try:
            target = (Path(vid) / "device").resolve()
            for parent in (target, *target.parents):
                if not (parent / "idVendor").is_file():
                    continue
                # Stop at the nearest USB device even if it has no serial.
                if (parent / "serial").read_text(encoding="utf-8").strip() == serial:
                    physical_devices.add(str(parent))
                    candidates.append("/dev/" + os.path.basename(vid))
                break
        except OSError:
            continue
    if len(physical_devices) != 1:
        return None
    for node in sorted(candidates, key=lambda n: int(n.rsplit("video", 1)[-1])):
        if os.path.exists(node) and os.path.realpath(node) not in (exclude or set()):
            if _probe_color_node(node, width, height, fourcc=fourcc):
                LOGGER.info("usb_serial %r -> %s", serial, node)
                return node
    return None


def find_by_realsense_serial(serial: str, width: int = 640, height: int = 480,
                             fourcc: str = "", exclude: Optional[set[str]] = None) -> Optional[str]:
    """Resolve SDK serials absent from USB descriptors, then capture using V4L2."""
    serial = str(serial or "").strip()
    if not serial:
        return None
    try:
        import pyrealsense2 as rs

        context = rs.context()
        matches = [device for device in context.query_devices()
                   if device.get_info(rs.camera_info.serial_number) == serial]
        if len(matches) != 1:
            return None
        color_sensors = [sensor for sensor in matches[0].query_sensors()
                         if any(profile.stream_type() == rs.stream.color
                                for profile in sensor.get_stream_profiles())]
        if len(color_sensors) != 1:
            return None
        port = color_sensors[0].get_info(rs.camera_info.physical_port)
        if not port or not Path(port).is_absolute():
            return None
        target = Path(port).resolve(strict=True)
        for parent in (target, *target.parents):
            vendor = parent / "idVendor"
            if not vendor.is_file():
                continue
            # Only the nearest physical USB device is eligible, never a hub.
            if vendor.read_text().strip().lower() != "8086":
                return None
            device_class = parent / "bDeviceClass"
            if device_class.is_file() and device_class.read_text().strip() == "09":
                return None
            # D435i infrared nodes can also accept YUYV. Probe only the SDK's
            # RGB sensor node, never the first YUYV node of the whole device.
            if not target.name.startswith("video") or not target.name[5:].isdigit():
                return None
            node = "/dev/" + target.name
            if not os.path.exists(node) or os.path.realpath(node) in (exclude or set()):
                return None
            if _probe_color_node(node, width, height, fourcc=fourcc):
                return node
            return None
    except Exception:
        # SDK initialization and disconnect errors are retried by the V4L2 worker.
        LOGGER.debug("RealSense serial discovery unavailable serial=%s", serial, exc_info=True)
    return None


def resolve_device(match: dict, width: int = 640, height: int = 480, fourcc: str = "", exclude: Optional[set[str]] = None) -> Optional[str]:
    """根据 match 配置解析设备节点。"""
    if not isinstance(match, dict):
        return None
    kind = str(match.get("type") or "").strip().lower()
    value = str(match.get("value") or "").strip()
    if kind == "usb_serial":
        # An old explicit node must never bypass a serial identity constraint.
        return find_by_usb_serial(value, width, height, fourcc, exclude)
    if kind == "realsense_serial":
        return find_by_realsense_serial(value, width, height, fourcc, exclude)
    explicit = str(match.get("device") or "").strip()
    if explicit and os.path.exists(explicit) and os.path.realpath(explicit) not in (exclude or set()):
        return explicit
    if kind == "entity":
        return find_by_entity(value, width=width, height=height, fourcc=fourcc, exclude=exclude)
    if kind == "usb_path":
        return find_by_usb_path(value, width=width, height=height, fourcc=fourcc, exclude=exclude)
    if kind == "device" and value and os.path.exists(value) and os.path.realpath(value) not in (exclude or set()):
        return value
    return None
