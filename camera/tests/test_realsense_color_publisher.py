import json
from pathlib import Path
from types import SimpleNamespace
import tempfile
import unittest
from unittest.mock import Mock, patch

from vision.rokae_runtime.realsense_color_publisher import load_source, publish


class Image:
    def __init__(self):
        self.header = SimpleNamespace(stamp=None, frame_id="")
        self.height = self.width = self.step = 0
        self.encoding = ""
        self.is_bigendian = 0
        self.data = b""


def write_config(directory, camera="hand_left", **overrides):
    cfg = {
        "enabled": True,
        "backend": "realsense",
        "match": {"type": "realsense_serial", "value": "serial-1"},
        "width": 2,
        "height": 1,
        "fps": 15,
        "enable_depth": False,
        "require_depth": False,
        "require_synced": False,
    }
    cfg.update(overrides)
    path = Path(directory) / "vision.json"
    path.write_text(
        json.dumps({"rokae": {"owner": {"cameras": {camera: cfg}}}})
    )
    return path


def realsense_fixture(
    *, profile=(2, 1, 15), payload=b"\x01\x02\x03\x04\x05\x06"
):
    width, height, fps = profile
    video = SimpleNamespace(width=lambda: width, height=lambda: height)
    stream_profile = SimpleNamespace(
        as_video_stream_profile=lambda: video,
        stream_type=lambda: "color",
        format=lambda: "rgb8",
        fps=lambda: fps,
    )
    device = SimpleNamespace(
        get_info=lambda _key: "serial-1",
        query_sensors=lambda: [
            SimpleNamespace(get_stream_profiles=lambda: [stream_profile])
        ],
    )
    color = SimpleNamespace(
        get_width=lambda: width,
        get_height=lambda: height,
        get_data=lambda: payload,
    )
    pipeline = Mock()
    pipeline.wait_for_frames.return_value = SimpleNamespace(
        get_color_frame=lambda: color
    )
    pipeline_config = Mock()
    rs = SimpleNamespace(
        context=lambda: SimpleNamespace(query_devices=lambda: [device]),
        camera_info=SimpleNamespace(serial_number="serial"),
        stream=SimpleNamespace(color="color"),
        format=SimpleNamespace(rgb8="rgb8"),
        pipeline=Mock(return_value=pipeline),
        config=Mock(return_value=pipeline_config),
    )
    return rs, pipeline, pipeline_config


def ros_fixture():
    published = []
    publisher = SimpleNamespace(publish=published.append)
    node = Mock()
    node.create_publisher.return_value = publisher
    node.get_clock.return_value.now.return_value.to_msg.return_value = "stamp"
    states = iter((True, False))
    rclpy = SimpleNamespace(
        init=Mock(),
        ok=lambda: next(states, False),
        shutdown=Mock(),
    )
    qos = object()
    ros = {
        "rclpy": rclpy,
        "Node": Mock(return_value=node),
        "Image": Image,
        "qos": qos,
        "signal_options": object(),
    }
    return ros, node, qos, published


class TestRealSenseColorPublisher(unittest.TestCase):
    def test_missing_runtime_dependency_has_actionable_error(self):
        with tempfile.TemporaryDirectory() as directory:
            path = write_config(directory)
            with patch.dict("sys.modules", {"pyrealsense2": None}):
                with self.assertRaisesRegex(
                    RuntimeError, "install.*inside Conda nav"
                ):
                    publish(path, "hand_left", ros={})

    def test_load_rejects_non_wrist_disabled_depth_and_duplicate_serial(self):
        with tempfile.TemporaryDirectory() as directory:
            path = write_config(directory)
            with self.assertRaisesRegex(ValueError, "wrist role"):
                load_source(path, "head")
            path = write_config(directory, enabled=False)
            with self.assertRaisesRegex(ValueError, "disabled"):
                load_source(path, "hand_left")
            path.write_text(json.dumps({
                "rokae": {"owner": {"cameras": {"hand_left": []}}}
            }))
            with self.assertRaisesRegex(ValueError, "must be an object"):
                load_source(path, "hand_left")
            for field in ("enable_depth", "require_depth", "require_synced"):
                path = write_config(directory, **{field: True})
                with self.subTest(field=field), self.assertRaisesRegex(
                    ValueError, "forbids"
                ):
                    load_source(path, "hand_left")
            path = write_config(directory, enable_depth="false")
            with self.assertRaisesRegex(ValueError, "must be booleans"):
                load_source(path, "hand_left")

            data = json.loads(write_config(directory).read_text())
            data["rokae"]["owner"]["cameras"]["hand_right"] = {
                **data["rokae"]["owner"]["cameras"]["hand_left"],
                "enabled": True,
            }
            path.write_text(json.dumps(data))
            with self.assertRaisesRegex(ValueError, "multiple enabled roles"):
                load_source(path, "hand_left")

    def test_left_publishes_exact_rgb8_sensor_qos_and_frame_id(self):
        with tempfile.TemporaryDirectory() as directory:
            path = write_config(directory)
            rs, pipeline, pipeline_config = realsense_fixture()
            ros, node, qos, published = ros_fixture()
            publish(path, "hand_left", rs_module=rs, ros=ros)

        pipeline_config.enable_device.assert_called_once_with("serial-1")
        pipeline_config.enable_stream.assert_called_once_with(
            "color", 2, 1, "rgb8", 15
        )
        pipeline.start.assert_called_once_with(pipeline_config)
        pipeline.stop.assert_called_once_with()
        node.create_publisher.assert_called_once_with(
            Image, "/camera/left_wrist/color/image_raw", qos,
        )
        self.assertEqual(len(published), 1)
        message = published[0]
        self.assertEqual(message.header.stamp, "stamp")
        self.assertEqual(
            message.header.frame_id, "left_wrist_color_optical_frame"
        )
        self.assertEqual(
            (message.width, message.height, message.step), (2, 1, 6)
        )
        self.assertEqual(
            (message.encoding, message.data),
            ("rgb8", b"\x01\x02\x03\x04\x05\x06"),
        )
        node.destroy_node.assert_called_once_with()
        ros["rclpy"].shutdown.assert_called_once_with()

    def test_right_uses_right_role_topic_and_frame(self):
        with tempfile.TemporaryDirectory() as directory:
            path = write_config(directory, camera="hand_right")
            rs, pipeline, _pipeline_config = realsense_fixture()
            ros, node, _qos, published = ros_fixture()
            publish(path, "hand_right", rs_module=rs, ros=ros)

        self.assertEqual(
            node.create_publisher.call_args.args[1],
            "/camera/right_wrist/color/image_raw",
        )
        self.assertEqual(
            published[0].header.frame_id,
            "right_wrist_color_optical_frame",
        )
        pipeline.stop.assert_called_once_with()

    def test_unsupported_profile_fails_before_pipeline_is_created(self):
        with tempfile.TemporaryDirectory() as directory:
            path = write_config(directory, width=1280, height=720)
            rs, _pipeline, _pipeline_config = realsense_fixture()
            ros, _node, _qos, _published = ros_fixture()
            with self.assertRaisesRegex(
                RuntimeError, "Unsupported.*1280x720@15"
            ):
                publish(path, "hand_left", rs_module=rs, ros=ros)
        rs.pipeline.assert_not_called()
        ros["rclpy"].init.assert_not_called()

    def test_start_failure_cleans_ros_without_stopping_pipeline(self):
        with tempfile.TemporaryDirectory() as directory:
            path = write_config(directory)
            rs, pipeline, _pipeline_config = realsense_fixture()
            pipeline.start.side_effect = RuntimeError("busy")
            ros, node, _qos, _published = ros_fixture()
            with self.assertRaisesRegex(RuntimeError, "busy"):
                publish(path, "hand_left", rs_module=rs, ros=ros)
        pipeline.stop.assert_not_called()
        node.destroy_node.assert_called_once_with()
        ros["rclpy"].shutdown.assert_called_once_with()

    def test_bad_payload_stops_pipeline_and_cleans_up(self):
        with tempfile.TemporaryDirectory() as directory:
            path = write_config(directory)
            rs, pipeline, _pipeline_config = realsense_fixture(payload=b"bad")
            ros, node, _qos, _published = ros_fixture()
            with self.assertRaisesRegex(RuntimeError, "payload"):
                publish(path, "hand_left", rs_module=rs, ros=ros)
        pipeline.stop.assert_called_once_with()
        node.destroy_node.assert_called_once_with()
        ros["rclpy"].shutdown.assert_called_once_with()


if __name__ == "__main__":
    unittest.main()
