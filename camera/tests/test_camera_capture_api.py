"""Unit tests for GET /camera/capture helpers and Owner capture."""

from __future__ import annotations

import os
import tempfile
import unittest
from unittest.mock import patch

import numpy as np

from vision.rokae_runtime.capture_api import (
    parse_capture_query,
    parse_format,
    parse_streams,
    resolve_camera,
    write_capture_dir,
)
from vision.rokae_runtime.owner import RokaeCameraOwner
from vision.rokae_runtime.ros_bridge import RosCameraOwner, RosImageWorker, image_to_depth_mm


class TestCaptureParse(unittest.TestCase):
    def test_resolve_camera_aliases(self):
        self.assertEqual(resolve_camera("head"), ("head", "head"))
        self.assertEqual(resolve_camera("left_wrist"), ("left_wrist", "hand_left"))
        self.assertEqual(resolve_camera("hand_wrist"), ("hand_wrist", "hand_right"))
        self.assertEqual(resolve_camera("hand_right"), ("hand_wrist", "hand_right"))
        self.assertEqual(resolve_camera("right_wrist"), ("hand_wrist", "hand_right"))
        self.assertEqual(resolve_camera("right"), ("hand_wrist", "hand_right"))
        self.assertIsNone(resolve_camera("unknown"))

    def test_streams_default_and_invalid(self):
        self.assertEqual(parse_streams(None), ({"color"}, None))
        self.assertEqual(parse_streams("color,depth"), ({"color", "depth"}, None))
        self.assertEqual(parse_streams("depth,color"), ({"color", "depth"}, None))
        self.assertEqual(parse_streams(""), (None, "INVALID_STREAMS"))
        self.assertEqual(parse_streams("rgb"), (None, "INVALID_STREAMS"))

    def test_format_optional_for_depth(self):
        self.assertEqual(parse_format(None, need_depth=True), ("raw", None))
        self.assertEqual(parse_format("preview", need_depth=True), ("preview", None))
        self.assertEqual(parse_format("png", need_depth=True), (None, "INVALID_FORMAT"))
        self.assertEqual(parse_format("raw", need_depth=False), (None, None))

    def test_parse_query_success_and_errors(self):
        ok, err = parse_capture_query({"camera": "head"})
        self.assertIsNone(err)
        self.assertEqual(ok["streams"], {"color"})
        self.assertEqual(ok["contract"], "head")
        wrist, err = parse_capture_query({"camera": "right_wrist"})
        self.assertIsNone(err)
        self.assertEqual(wrist["contract"], "hand_wrist")
        self.assertEqual(wrist["internal"], "hand_right")
        _, err = parse_capture_query({"camera": "nope"})
        self.assertEqual(err["error_code"], "CAMERA_NOT_FOUND")
        _, err = parse_capture_query({"camera": "head", "streams": ""})
        self.assertEqual(err["error_code"], "INVALID_STREAMS")


class TestCaptureWrite(unittest.TestCase):
    def test_write_color_and_depth_atomic(self):
        # Minimal 1x1 JPEG (SOF0 width=1 height=1)
        jpeg = bytes.fromhex(
            "ffd8ffe000104a46494600010100000100010000"
            "ffc0000b080001000101011100"
            "ffd9"
        )
        depth = np.arange(24, dtype=np.uint16).reshape(4, 6)
        with tempfile.TemporaryDirectory() as tmp:
            written = write_capture_dir(
                capture_id="capture-ut1",
                color_jpeg=jpeg,
                depth_mm=depth,
                depth_format="raw",
                root=tmp,
            )
            self.assertTrue(os.path.isfile(written["color"]["path"]))
            self.assertEqual((written["color"]["width"], written["color"]["height"]), (1, 1))
            self.assertTrue(os.path.isfile(written["depth"]["path"]))
            loaded = np.load(written["depth"]["path"])
            np.testing.assert_array_equal(loaded, depth)


class TestOwnerCapture(unittest.TestCase):
    def test_color_only_capture(self):
        owner = RokaeCameraOwner({"rokae": {"owner": {"cameras": {}}}})
        jpeg = bytes.fromhex(
            "ffd8ffe000104a46494600010100000100010000"
            "ffc0000b080008000801011100"
            "ffd9"
        )
        with patch.object(owner, "get_jpeg", return_value=jpeg), \
             tempfile.TemporaryDirectory() as tmp, \
             patch.dict(os.environ, {"VISION_CAPTURE_ROOT": tmp}):
            result = owner.capture(contract="head", internal="head", streams={"color"}, format=None)
            self.assertTrue(result["ok"], result)
            self.assertTrue(os.path.isfile(result["color"]["path"]))
            self.assertEqual(result["color"]["height"], 8)
            self.assertIsNone(result["depth"])
            self.assertTrue(result["same_shot"])

    def test_depth_not_ready_on_direct_owner(self):
        owner = RokaeCameraOwner({"rokae": {"owner": {"cameras": {}}}})
        result = owner.capture(contract="head", internal="head", streams={"depth"}, format="raw")
        self.assertFalse(result["ok"])
        self.assertEqual(result["error_code"], "CAMERA_NOT_READY")

    def test_ros_owner_depth_prefers_aligned(self):
        owner = RosCameraOwner({"rokae": {"owner": {"cameras": {}}}})
        aligned = RosImageWorker(decode=image_to_depth_mm)
        native = RosImageWorker(decode=image_to_depth_mm)
        aligned._frame = np.ones((2, 2), dtype=np.uint16) * 100
        aligned._received = __import__("time").monotonic()
        aligned._source_stamp = __import__("time").time()
        native._frame = np.ones((2, 2), dtype=np.uint16) * 200
        native._received = aligned._received
        native._source_stamp = aligned._source_stamp
        owner._depth_workers["head"] = {"aligned": aligned, "native": native, "stale": 2.0}
        frame, is_aligned = owner.get_depth_mm("head")
        np.testing.assert_array_equal(frame, aligned._frame)
        self.assertTrue(is_aligned)
        self.assertIsNone(owner.get_depth_mm("left_wrist"))

    def test_capture_rejects_unaligned_depth(self):
        owner = RokaeCameraOwner({"rokae": {"owner": {"cameras": {}}}})
        depth = np.ones((2, 2), dtype=np.uint16)
        with patch.object(owner, "get_depth_mm", return_value=(depth, False)):
            result = owner.capture(
                contract="head", internal="head", streams={"depth"}, format="raw",
            )
        self.assertFalse(result["ok"])
        self.assertEqual(result["error_code"], "DEPTH_NOT_ALIGNED")


class TestDepthDecode(unittest.TestCase):
    def test_image_to_depth_mm(self):
        from types import SimpleNamespace
        import time

        data = np.array([[1, 2], [3, 4]], dtype=np.uint16).tobytes()
        msg = SimpleNamespace(
            encoding="16UC1", width=2, height=2, step=4, data=data,
            header=SimpleNamespace(stamp=SimpleNamespace(sec=int(time.time()), nanosec=0)),
        )
        out = image_to_depth_mm(msg)
        self.assertEqual(out.tolist(), [[1, 2], [3, 4]])


if __name__ == "__main__":
    unittest.main()
