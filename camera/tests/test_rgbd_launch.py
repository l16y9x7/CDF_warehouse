"""Verify driver profile arguments without importing ROS or opening cameras."""
import importlib.util
import json
from pathlib import Path
import tempfile
import types
import unittest
from unittest.mock import patch


class Action:
    def __init__(self, *args, **kwargs):
        self.args = args
        self.kwargs = kwargs


class LaunchConfiguration:
    def __init__(self, name):
        self.name = name

    def perform(self, context):
        return context[self.name]


def load_launch():
    exports = {
        "ament_index_python": {},
        "ament_index_python.packages": {
            "get_package_share_directory": lambda name: "/mock/share/" + name,
        },
        "launch": {"LaunchDescription": Action},
        "launch.actions": {
            "DeclareLaunchArgument": Action,
            "IncludeLaunchDescription": Action,
            "OpaqueFunction": Action,
        },
        "launch.launch_description_sources": {"PythonLaunchDescriptionSource": Action},
        "launch.substitutions": {"LaunchConfiguration": LaunchConfiguration},
        "launch_ros": {},
        "launch_ros.actions": {"Node": Action},
    }
    modules = {}
    for name, attributes in exports.items():
        module = types.ModuleType(name)
        module.__dict__.update(attributes)
        modules[name] = module
    source = Path(__file__).resolve().parents[1] / "config/ros/rgbd.launch.py"
    spec = importlib.util.spec_from_file_location("rgbd_launch_under_test", source)
    module = importlib.util.module_from_spec(spec)
    with patch.dict("sys.modules", modules):
        spec.loader.exec_module(module)
    return module


class CameraProfileTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.launch = load_launch()

    def test_synced_template_enables_all_three_camera_roles(self):
        import yaml
        template = Path(__file__).resolve().parents[1] / "config/ros/synced_rgbd.yaml"
        cameras = yaml.safe_load(template.read_text())["cameras"]
        self.assertEqual(set(cameras), {"head", "left_wrist", "right_wrist"})
        for camera in cameras.values():
            self.assertIs(camera["enabled"], True)

    def build(self, camera="head", **overrides):
        omit_ros_fps = overrides.pop("_omit_ros_fps", False)
        cfg = {
            "enabled": True,
            "match": {
                "type": "orbbec" if camera == "head" else "realsense_serial",
                "value": "test-serial",
            },
            "width": 640, "height": 480, "ros_fps": 15,
        }
        cfg.update(overrides)
        if omit_ros_fps:
            cfg.pop("ros_fps", None)
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "vision.json"
            path.write_text(json.dumps({"rokae": {"owner": {"cameras": {camera: cfg}}}}))
            return self.launch.build({"vision_config": str(path), "camera": camera})[0]

    def test_head_mixed_profile_depth_without_hw_d2c(self):
        node = self.build(width=1280, height=720, depth_width=640, depth_height=480)
        params = node.kwargs["parameters"][0]
        self.assertEqual((params["color_width"], params["color_height"], params["color_fps"]),
                         (1280, 720, 15))
        self.assertEqual((params["depth_width"], params["depth_height"], params["depth_fps"]),
                         (640, 480, 15))
        self.assertTrue(params["enable_depth"])
        self.assertFalse(params["depth_registration"])
        self.assertEqual(params["align_mode"], "HW")
        self.assertFalse(node.kwargs.get("respawn", False))
        self.assertFalse(params["enable_frame_sync"])
        self.assertEqual(params["color_qos"], "default")
        self.assertIn(("depth/image_raw", "depth/image_rect_raw"),
                      node.kwargs["remappings"])

    def test_legacy_head_depth_follows_color_and_ros_fps(self):
        params = self.build(width=1280, height=800, ros_fps=5).kwargs["parameters"][0]
        self.assertEqual((params["depth_width"], params["depth_height"], params["depth_fps"]),
                         (1280, 800, 5))
        self.assertEqual(params["color_fps"], 5)

    def test_right_profile_unchanged(self):
        arguments = dict(self.build(camera="hand_right").kwargs["launch_arguments"])
        self.assertEqual(arguments["rgb_camera.color_profile"], "640x480x15")
        self.assertEqual(arguments["depth_module.depth_profile"], "640x480x15")
        self.assertEqual(arguments["serial_no"], "_test-serial")
        self.assertEqual(arguments["align_depth.enable"], "true")
        self.assertEqual(arguments["initial_reset"], "false")

    def test_right_independent_depth_profile(self):
        arguments = dict(self.build(camera="hand_right", width=1280, height=720,
                                    depth_width=640, depth_height=480).kwargs["launch_arguments"])
        self.assertEqual(arguments["rgb_camera.color_profile"], "1280x720x15")
        self.assertEqual(arguments["depth_module.depth_profile"], "640x480x15")

    def test_left_color_only_profile_uses_independent_role_and_disables_depth(self):
        arguments = dict(self.build(
            camera="hand_left", width=1280, height=720, fps=30,
            depth_width=0, depth_height=0,
            enable_depth=False, require_synced=False, _omit_ros_fps=True,
        ).kwargs["launch_arguments"])
        self.assertEqual(arguments["camera_name"], "left_wrist")
        self.assertEqual(arguments["serial_no"], "_test-serial")
        self.assertEqual(arguments["rgb_camera.color_profile"], "1280x720x30")
        self.assertEqual(arguments["enable_depth"], "false")
        self.assertEqual(arguments["depth_module.depth_profile"], "1280x720x30")
        self.assertEqual(arguments["align_depth.enable"], "false")
        self.assertEqual(arguments["enable_sync"], "false")

    def test_color_only_profile_rejects_synced_requirement(self):
        with self.assertRaisesRegex(ValueError, "requires depth"):
            self.build(
                camera="hand_left", enable_depth=False, require_synced=True,
            )

    def test_profile_rejects_required_but_disabled_depth(self):
        with self.assertRaisesRegex(ValueError, "Required depth must be enabled"):
            self.build(
                camera="hand_left", enable_depth=False,
                require_depth=True, require_synced=False,
            )

    def test_profile_flags_must_be_json_booleans(self):
        with self.assertRaisesRegex(ValueError, "flags must be booleans"):
            self.build(camera="hand_left", enable_depth="false")

    def test_canonical_fps_takes_precedence_over_legacy_ros_fps(self):
        arguments = dict(self.build(
            camera="hand_left", fps=15, ros_fps=30,
            enable_depth=False, require_depth=False, require_synced=False,
        ).kwargs["launch_arguments"])
        self.assertEqual(arguments["rgb_camera.color_profile"], "640x480x15")

    def test_invalid_depth_dimensions_rejected_before_driver_creation(self):
        for camera in ("head", "hand_right"):
            for field in ("depth_width", "depth_height"):
                for value in (0, -1):
                    with self.subTest(camera=camera, field=field, value=value):
                        with self.assertRaisesRegex(ValueError, "Invalid RGB-D profile"):
                            self.build(camera=camera, **{field: value})


if __name__ == "__main__":
    unittest.main()
