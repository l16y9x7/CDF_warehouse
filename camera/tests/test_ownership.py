"""Tianji-like Media ownership: ros source must not load device modules."""

from __future__ import annotations

import json
import sys
import unittest
from pathlib import Path
from unittest.mock import Mock, patch

class OwnershipTests(unittest.TestCase):
    def test_rokae_application_services_use_the_nav_environment(self):
        service_dir = Path(__file__).resolve().parents[1] / "config" / "ros"
        names = (
            "vision-rokae-preview-owner.service",
            "vision-rokae-preview-media.service",
            "vision-rokae-preview-synced-rgbd.service",
            "vision-rokae-preview-health.service",
        )
        for name in names:
            with self.subTest(service=name):
                text = (service_dir / name).read_text()
                self.assertIn("Environment=PYTHONNOUSERSITE=1", text)
                self.assertIn("/home/admin/miniconda3/envs/nav/bin/python", text)
                self.assertNotIn("/home/admin/miniconda3/envs/smt", text)
                self.assertNotIn("/usr/bin/python3 -m vision", text)
                self.assertNotIn(" uv ", text)

    def test_vendor_rgbd_supervisors_keep_ros_humble_python_abi(self):
        service_dir = Path(__file__).resolve().parents[1] / "config" / "ros"
        for name in (
            "vision-rokae-preview-head-rgbd.service",
            "vision-rokae-preview-right-rgbd.service",
        ):
            with self.subTest(service=name):
                text = (service_dir / name).read_text()
                self.assertIn("Environment=PYTHONNOUSERSITE=1", text)
                self.assertIn("/usr/bin/python3 -m vision.rokae_runtime.driver_supervisor", text)
                self.assertNotIn("envs/smt", text)
                self.assertNotIn(" uv ", text)

        left = (service_dir / "vision-rokae-preview-left-color.service").read_text()
        self.assertIn("Environment=PYTHONNOUSERSITE=1", left)
        self.assertIn(
            "/home/admin/miniconda3/envs/nav/bin/python -m "
            "vision.rokae_runtime.realsense_color_publisher",
            left,
        )
        self.assertNotIn("realsense2_camera", left)
        self.assertNotIn("envs/smt", left)
        self.assertNotIn(" uv ", left)

    def test_application_services_do_not_start_disabled_camera_drivers(self):
        service_dir = Path(__file__).resolve().parents[1] / "config" / "ros"
        for name in (
            "vision-rokae-preview-owner.service",
            "vision-rokae-preview-synced-rgbd.service",
        ):
            with self.subTest(service=name):
                text = (service_dir / name).read_text()
                wants = next(
                    line for line in text.splitlines() if line.startswith("Wants=")
                )
                self.assertNotIn("head-rgbd.service", wants)
                self.assertNotIn("left-color.service", wants)
                self.assertNotIn("right-rgbd.service", wants)

    def test_ros_media_rejects_loaded_capture_module(self):
        from vision.ownership import ensure_ros_media_subscriber_only

        sys.modules["vision.rokae_runtime.capture"] = Mock()
        try:
            with self.assertRaises(RuntimeError) as ctx:
                ensure_ros_media_subscriber_only()
            self.assertIn("vision.rokae_runtime.capture", str(ctx.exception))
        finally:
            sys.modules.pop("vision.rokae_runtime.capture", None)

    def test_push_runtime_ros_calls_subscriber_guard(self):
        from vision.push import PushRuntime

        cfg = {
            "media": {
                "push": {
                    "source": "ros",
                    "device_sn": "TEST",
                    "streams": [
                        {
                            "camera_id": "head",
                            "enabled": True,
                            "stream_slot": 1,
                            "width": 640,
                            "height": 480,
                            "fps": 8,
                        }
                    ],
                }
            }
        }
        with patch("vision.ownership.ensure_ros_media_subscriber_only") as guard, \
             patch("vision.ros_push.RosPushSource") as source_cls:
            source_cls.return_value = Mock()
            PushRuntime(cfg, source_uri=Mock(side_effect=AssertionError))
            guard.assert_called_once()

    def test_source_runtime_units_map_smt_roles(self):
        from vision.ownership import SOURCE_RUNTIME_UNITS

        self.assertEqual(
            SOURCE_RUNTIME_UNITS["head"],
            "vision-rokae-preview-head-rgbd.service",
        )
        self.assertEqual(
            SOURCE_RUNTIME_UNITS["media"],
            "vision-rokae-preview-media.service",
        )
        self.assertEqual(
            SOURCE_RUNTIME_UNITS["hand_left"],
            "vision-rokae-preview-left-color.service",
        )

    def test_left_color_fragment_explicitly_disables_right_wrist(self):
        path = Path(__file__).resolve().parents[1] / "config/ros/left-color.example.json"
        config = json.loads(path.read_text())
        cameras = config["rokae"]["owner"]["cameras"]
        self.assertIs(cameras["hand_left"]["enabled"], True)
        self.assertIs(cameras["hand_right"]["enabled"], False)
        streams = {
            row["camera_id"]: row
            for row in config["media"]["push"]["streams"]
        }
        self.assertIs(streams["hand_left"]["enabled"], True)
        self.assertIs(streams["hand_right"]["enabled"], False)


if __name__ == "__main__":
    unittest.main()
