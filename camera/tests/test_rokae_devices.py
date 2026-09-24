"""devices.py 解析单测：mock sysfs，不碰真机。"""

from __future__ import annotations

import os
import tempfile
import types
import unittest
from pathlib import Path
from unittest import mock

from vision.rokae_runtime import devices


class TestRokaeDevices(unittest.TestCase):
    def test_realsense_sdk_serial_without_usb_serial(self):
        with tempfile.TemporaryDirectory() as tmp:
            usb = Path(tmp) / "1-4.2.4.4"
            node = usb / "interface" / "video4linux" / "video10"
            node.mkdir(parents=True)
            (usb / "idVendor").write_text("8086")
            camera = mock.Mock()
            camera.get_info.side_effect = lambda key: {"serial": "actual-right", "port": str(node)}[key]
            sdk = types.SimpleNamespace(context=mock.Mock(), camera_info=types.SimpleNamespace(
                serial_number="serial", physical_port="port"), stream=types.SimpleNamespace(color="color"))
            color_sensor = mock.Mock()
            color_sensor.get_info.side_effect = camera.get_info.side_effect
            color_profile = mock.Mock()
            color_profile.stream_type.return_value = "color"
            color_sensor.get_stream_profiles.return_value = [color_profile]
            infrared_sensor = mock.Mock()
            infrared_profile = mock.Mock()
            infrared_profile.stream_type.return_value = "infrared"
            infrared_sensor.get_stream_profiles.return_value = [infrared_profile]
            camera.query_sensors.return_value = [infrared_sensor, color_sensor]
            sdk.context.return_value.query_devices.return_value = [camera]
            with mock.patch.dict("sys.modules", {"pyrealsense2": sdk}), mock.patch.object(
                devices, "_probe_color_node", return_value=True
            ) as probe, mock.patch.object(devices.os.path, "exists", return_value=True):
                match = {"type": "realsense_serial", "value": "actual-right", "device": "/dev/video0"}
                self.assertEqual(devices.resolve_device(match, exclude={"/dev/video6"}), "/dev/video10")
                probe.assert_called_once_with("/dev/video10", 640, 480, fourcc="")
                probe.reset_mock()
                self.assertIsNone(devices.resolve_device(match, exclude={"/dev/video10"}))
                probe.assert_not_called()
                camera.query_sensors.return_value = [infrared_sensor]
                self.assertIsNone(devices.resolve_device(match))
                camera.query_sensors.return_value = [color_sensor, color_sensor]
                self.assertIsNone(devices.resolve_device(match))
                camera.query_sensors.return_value = [infrared_sensor, color_sensor]
                probe.reset_mock()
                sdk.context.return_value.query_devices.return_value = []
                self.assertIsNone(devices.resolve_device(match))
                sdk.context.return_value.query_devices.return_value = [camera, camera]
                self.assertIsNone(devices.resolve_device(match))
                sdk.context.return_value.query_devices.side_effect = RuntimeError("USB disconnect")
                self.assertIsNone(devices.resolve_device(match))
                probe.assert_not_called()

    def test_realsense_rejects_invalid_port_wrong_serial_and_hub(self):
        with tempfile.TemporaryDirectory() as tmp:
            usb = Path(tmp) / "usb-camera"
            usb.mkdir()
            (usb / "idVendor").write_text("8086")
            camera = mock.Mock()
            values = {"serial": "actual", "port": str(usb)}
            camera.get_info.side_effect = lambda key: values[key]
            sdk = types.SimpleNamespace(context=mock.Mock(), camera_info=types.SimpleNamespace(
                serial_number="serial", physical_port="port"), stream=types.SimpleNamespace(color="color"))
            color_sensor = mock.Mock()
            color_sensor.get_info.side_effect = camera.get_info.side_effect
            color_profile = mock.Mock()
            color_profile.stream_type.return_value = "color"
            color_sensor.get_stream_profiles.return_value = [color_profile]
            infrared_sensor = mock.Mock()
            infrared_profile = mock.Mock()
            infrared_profile.stream_type.return_value = "infrared"
            infrared_sensor.get_stream_profiles.return_value = [infrared_profile]
            camera.query_sensors.return_value = [infrared_sensor, color_sensor]
            sdk.context.return_value.query_devices.return_value = [camera]
            with mock.patch.dict("sys.modules", {"pyrealsense2": sdk}), mock.patch.object(
                devices, "_probe_color_node"
            ) as probe:
                self.assertIsNone(devices.find_by_realsense_serial("old-serial"))
                for port in ["", "relative/video0", str(usb / "missing")]:
                    values["port"] = port
                    self.assertIsNone(devices.find_by_realsense_serial("actual"))
                values["port"] = str(usb)
                (usb / "idVendor").write_text("0bda")
                self.assertIsNone(devices.find_by_realsense_serial("actual"))
                (usb / "idVendor").write_text("8086")
                (usb / "bDeviceClass").write_text("09")
                self.assertIsNone(devices.find_by_realsense_serial("actual"))
                probe.assert_not_called()
            with mock.patch.dict("sys.modules", {"pyrealsense2": None}):
                self.assertIsNone(devices.find_by_realsense_serial("actual"))

    def test_serial_pins_physical_camera_and_probes_color_only(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            entries = []
            for number, serial in [(0, "wrong-camera"), (14, "right-camera"), (18, "right-camera")]:
                usb = root / serial
                usb.mkdir(exist_ok=True)
                (usb / "idVendor").write_text("8086")
                (usb / "serial").write_text(serial)
                node = root / f"video{number}"
                node.mkdir()
                (node / "device").symlink_to(usb / "interface")
                entries.append(str(node))
            with mock.patch.object(devices.glob, "glob", return_value=entries), mock.patch.object(
                devices.os.path, "exists", return_value=True
            ), mock.patch.object(devices, "_probe_color_node", side_effect=lambda node, *a, **kw: node == "/dev/video18") as probe:
                got = devices.resolve_device({"type": "usb_serial", "value": "right-camera", "device": "/dev/video0"})
                self.assertEqual(got, "/dev/video18")
                self.assertEqual([c.args[0] for c in probe.call_args_list], ["/dev/video14", "/dev/video18"])
                probe.reset_mock()
                self.assertIsNone(devices.resolve_device({"type": "usb_serial", "value": "absent", "device": "/dev/video0"}))
                probe.assert_not_called()

    def test_serial_does_not_match_hub_or_ambiguous_devices(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            (root / "idVendor").write_text("hub")
            (root / "serial").write_text("hub-serial")
            entries = []
            for number in [0, 2]:
                usb = root / f"usb{number}"
                usb.mkdir()
                (usb / "idVendor").write_text("8086")
                (usb / "serial").write_text("duplicate")
                node = root / f"video{number}"
                node.mkdir()
                (node / "device").symlink_to(usb / "interface")
                entries.append(str(node))
            with mock.patch.object(devices.glob, "glob", return_value=entries), mock.patch.object(devices, "_probe_color_node") as probe:
                self.assertIsNone(devices.find_by_usb_serial("duplicate"))
                self.assertIsNone(devices.find_by_usb_serial("hub-serial"))
                (root / "usb0" / "serial").unlink()
                self.assertIsNone(devices.find_by_usb_serial("hub-serial"))
                probe.assert_not_called()

    def test_resolve_explicit_device(self) -> None:
        with tempfile.NamedTemporaryFile(prefix="video", delete=False) as fh:
            path = fh.name
        try:
            got = devices.resolve_device({"type": "entity", "value": "x", "device": path})
            self.assertEqual(got, path)
        finally:
            os.unlink(path)

    def test_find_by_usb_path_prefers_probed_node(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            v0 = root / "video0"
            v2 = root / "video2"
            v0.mkdir()
            v2.mkdir()
            (v0 / "device").symlink_to(root / "sys" / "1-4.2.1" / "video4linux" / "video0")
            (v2 / "device").symlink_to(root / "sys" / "1-4.2.1" / "video4linux" / "video2")
            (root / "sys" / "1-4.2.1" / "video4linux" / "video0").mkdir(parents=True)
            (root / "sys" / "1-4.2.1" / "video4linux" / "video2").mkdir(parents=True)

            def fake_glob(pattern):
                del pattern
                return [str(v0), str(v2)]

            def fake_exists(path):
                return path in {"/dev/video0", "/dev/video2"}

            def fake_probe(node, width, height):
                del width, height
                return node == "/dev/video2"

            with mock.patch.object(devices.glob, "glob", side_effect=fake_glob), mock.patch.object(
                devices.os.path, "exists", side_effect=fake_exists
            ), mock.patch.object(devices, "_probe_color_node", side_effect=fake_probe):
                got = devices.find_by_usb_path("1-4.2.1", width=640, height=480)
            self.assertEqual(got, "/dev/video2")

    def test_find_by_entity_prefers_yuyv(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            v8 = root / "video8"
            v12 = root / "video12"
            v8.mkdir()
            v12.mkdir()
            (v8 / "name").write_text("Orbbec Gemini 335L", encoding="utf-8")
            (v12 / "name").write_text("Orbbec Gemini 335L", encoding="utf-8")

            class FakeCap:
                def __init__(self, node):
                    self.node = node
                    self._opened = True

                def isOpened(self):
                    return True

                def set(self, *_a, **_k):
                    return True

                def get(self, *_a, **_k):
                    # YUYV fourcc packed little-endian int for video12; GREY for video8
                    if self.node.endswith("12"):
                        return int.from_bytes(b"YUYV", "little")
                    return int.from_bytes(b"GREY", "little")

                def read(self):
                    import numpy as np

                    return True, np.zeros((48, 64, 3), dtype=np.uint8)

                def release(self):
                    return None

            def fake_glob(pattern):
                del pattern
                return [str(v8), str(v12)]

            def fake_exists(path):
                return path in {"/dev/video8", "/dev/video12"}

            def fake_capture(node, *_a, **_k):
                return FakeCap(node)

            import cv2

            with mock.patch.object(devices.glob, "glob", side_effect=fake_glob), mock.patch.object(
                devices.os.path, "exists", side_effect=fake_exists
            ), mock.patch.object(cv2, "VideoCapture", side_effect=fake_capture):
                got = devices.find_by_entity("Orbbec Gemini 335L", width=64, height=48)
            self.assertEqual(got, "/dev/video12")

    def test_resolve_unknown_kind(self) -> None:
        self.assertIsNone(devices.resolve_device({"type": "serial", "value": "x"}))


if __name__ == "__main__":
    unittest.main()
