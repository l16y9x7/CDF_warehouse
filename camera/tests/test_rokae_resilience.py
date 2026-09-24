"""ROKAE readiness, ownership and SDK fault regression tests (no hardware)."""
import json
import threading
import time
import unittest
from types import SimpleNamespace
from unittest.mock import Mock, patch
from urllib.request import urlopen

import numpy as np

from vision.adapters.rokae import RokaeCameraAdapter
from vision.rokae_runtime.capture import OrbbecSdkCaptureWorker, V4L2CaptureWorker
from vision.rokae_runtime.devices import find_by_usb_path
from vision.rokae_runtime.http_app import serve_owner
from vision.rokae_runtime.owner import CAMERA_IDS, RokaeCameraOwner


def configuration(**overrides):
    cameras = {cid: {"backend": "fake", "width": 32, "height": 24} for cid in CAMERA_IDS}
    cameras.update(overrides)
    return {"rokae": {"owner": {"host": "127.0.0.1", "port": 0, "cameras": cameras}}}


class TestRokaeReadiness(unittest.TestCase):
    def test_enabled_and_health_do_not_fabricate_frame(self):
        rows = [{"id": "head", "enabled": True},
                {"id": "hand_left", "enabled": True, "ready": False, "online": True},
                {"id": "hand_right", "online": True, "fresh": False}]
        with patch("vision.adapters.rokae._http_json", return_value=(200, {"cameras": rows, "status": "READY"})):
            adapter = RokaeCameraAdapter({})
            self.assertFalse(adapter.listing()["ok"])
            for cid in CAMERA_IDS:
                self.assertEqual(adapter.frame(cid)["error_code"], "CAMERA_NOT_READY")

    def test_three_http_roles_failure_and_recovery(self):
        owner = RokaeCameraOwner(configuration())
        owner.start()
        server = serve_owner(owner)
        thread = threading.Thread(target=server.serve_forever, daemon=True)
        thread.start()
        base = f"http://127.0.0.1:{server.server_port}"
        adapter = RokaeCameraAdapter({"rokae": {"base_url": base}})
        try:
            images = []
            for cid in CAMERA_IDS:
                with urlopen(adapter.frame(cid)["uri"], timeout=2) as response:
                    images.append(response.read())
            self.assertEqual(len(set(images)), 3)
            self.assertTrue(owner.health()["all_ready"])
            owner._workers["hand_left"]["worker"].stop()
            self.assertEqual(adapter.frame("hand_left")["error_code"], "CAMERA_NOT_READY")
            self.assertTrue(adapter.frame("head")["ok"])
            self.assertTrue(adapter.frame("hand_right")["ok"])
            self.assertFalse(owner.health()["all_ready"])
            owner._workers["hand_left"]["worker"].start()
            self.assertTrue(adapter.frame("hand_left")["ok"])
        finally:
            server.shutdown()
            server.server_close()
            owner.stop()
            thread.join(2)

    def test_duplicate_device_and_repeated_start(self):
        cfg = configuration(head={"backend": "v4l2"}, hand_left={"backend": "v4l2"})
        worker = Mock()
        worker.start.return_value = True
        worker.ready.return_value = True
        worker.peek.return_value = {
            "online": True, "width": 32, "height": 24,
            "age_sec": 0.0, "stamp": 1.0,
        }
        with patch("vision.rokae_runtime.owner.resolve_device", return_value="/dev/video42"), patch(
            "vision.rokae_runtime.owner.V4L2CaptureWorker", return_value=worker
        ):
            owner = RokaeCameraOwner(cfg)
            try:
                owner.start()
                owner.start()
                worker.start.assert_called_once()
                rows = {r["id"]: r for r in owner.listing()["cameras"]}
                self.assertEqual(rows["left_wrist"]["error"], "DEVICE_ALREADY_ASSIGNED")
                self.assertTrue(rows["hand_wrist"]["ready"])
            finally:
                owner.stop()

    def test_invalid_camera_does_not_prevent_other_roles(self):
        for bad in ({"match": "bad"}, {"backend": "typo"}, {"width": "bad"}, {"frame_stale_sec": -1}):
            with self.subTest(bad=bad):
                owner = RokaeCameraOwner(configuration(head=bad))
                try:
                    owner.start()
                    self.assertFalse(owner.camera_ready("head"))
                    self.assertTrue(owner.camera_ready("hand_left"))
                    self.assertTrue(owner.camera_ready("hand_right"))
                finally:
                    owner.stop()

    def test_usb_no_unprobed_fallback(self):
        with patch("vision.rokae_runtime.devices.glob.glob", return_value=["/sys/class/video4linux/video0"]), patch(
            "vision.rokae_runtime.devices.Path.resolve", return_value="/sys/usb/1-2/video4linux/video0"
        ), patch("vision.rokae_runtime.devices.os.path.exists", return_value=True), patch(
            "vision.rokae_runtime.devices._probe_color_node", return_value=False
        ):
            self.assertIsNone(find_by_usb_path("1-2"))

    def test_stopped_or_stale_physical_worker_has_no_frame(self):
        for worker in (V4L2CaptureWorker(camera_id="head", device="unused"), OrbbecSdkCaptureWorker(camera_id="head")):
            worker._frame = np.zeros((2, 2, 3), dtype=np.uint8)
            worker._stamp = time.time() - 10
            self.assertFalse(worker.ready())
            worker._stamp = time.time()
            self.assertTrue(worker.ready())
            worker.stop()
            self.assertFalse(worker.ready())


def sdk_fixture(serials=("A",), formats=("RGB",)):
    profiles = []
    for fmt in formats:
        profile = Mock()
        profile.get_format.return_value = fmt
        profile.get_width.return_value = 2
        profile.get_height.return_value = 2
        profile.get_fps.return_value = 10
        profiles.append(profile)
    devices = []
    for serial in serials:
        device = Mock()
        device.get_device_info().get_serial_number.return_value = serial
        device.get_device_info().get_name.return_value = "Gemini 335L"
        devices.append(device)
    sdk = SimpleNamespace(Context=Mock(), Pipeline=Mock(), Config=Mock(),
                          OBSensorType=SimpleNamespace(COLOR_SENSOR="color"),
                          OBFormat=SimpleNamespace(**{x: x for x in ("RGB", "BGR", "MJPG", "YUYV", "UYVY")}))
    listing = sdk.Context().query_devices()
    listing.get_count.return_value = len(devices)
    listing.get_device_by_index.side_effect = devices.__getitem__
    pipeline = sdk.Pipeline.return_value
    # Match the native SDK interface: no __getitem__ and base profiles need casting.
    entries = [SimpleNamespace(as_video_stream_profile=lambda p=p: p) for p in profiles]
    pipeline.get_stream_profile_list.return_value = SimpleNamespace(
        get_count=lambda: len(entries), get_stream_profile_by_index=entries.__getitem__)

    pipeline.wait_for_frames.return_value = None
    return sdk, pipeline


def eventually(predicate, timeout=2):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if predicate():
            return True
        time.sleep(.01)
    return False


class TestOrbbecFaults(unittest.TestCase):
    def test_ambiguous_or_missing_serial_does_not_open_pipeline(self):
        for serial in ("", "missing"):
            sdk, _ = sdk_fixture(("A", "B"))
            with patch.dict("sys.modules", pyorbbecsdk=sdk):
                worker = OrbbecSdkCaptureWorker(camera_id="head", serial=serial)
                try:
                    self.assertTrue(worker.start())
                    self.assertTrue(eventually(lambda: worker.last_error in {"DEVICE_NOT_FOUND", "DEVICE_AMBIGUOUS"}))
                    sdk.Pipeline.assert_not_called()
                    self.assertFalse(worker.ready())
                finally:
                    worker.stop()

    def test_exact_serial_and_duplicate_start(self):
        sdk, pipeline = sdk_fixture(("A", "B"))
        worker = OrbbecSdkCaptureWorker(camera_id="head", serial="B")
        with patch.dict("sys.modules", pyorbbecsdk=sdk):
            try:
                self.assertTrue(worker.start())
                self.assertTrue(worker.start())
                self.assertTrue(eventually(lambda: pipeline.start.called))
                pipeline.start.assert_called_once()
                self.assertEqual(worker.serial, "B")
            finally:
                worker.stop()
            pipeline.stop.assert_called_once()

    def test_start_failure_releases_pipeline(self):
        sdk, pipeline = sdk_fixture()
        pipeline.start.side_effect = RuntimeError("start failure")
        with patch.dict("sys.modules", pyorbbecsdk=sdk):
            worker = OrbbecSdkCaptureWorker(camera_id="head")
            try:
                self.assertTrue(worker.start())
                self.assertTrue(eventually(lambda: pipeline.stop.called))
                pipeline.stop.assert_called_once()
                self.assertFalse(worker.ready())
            finally:
                worker.stop()

    def test_unsupported_profile_is_not_started(self):
        sdk, pipeline = sdk_fixture(formats=("Y16",))
        with patch.dict("sys.modules", pyorbbecsdk=sdk):
            worker = OrbbecSdkCaptureWorker(camera_id="head")
            try:
                self.assertTrue(worker.start())
                self.assertTrue(eventually(lambda: worker.last_error == "COLOR_PROFILE_NOT_FOUND"))
                pipeline.start.assert_not_called()
            finally:
                worker.stop()

    def test_bad_color_frame_does_not_kill_capture(self):
        sdk, pipeline = sdk_fixture()
        worker = OrbbecSdkCaptureWorker(camera_id="head")
        bad = Mock()
        bad.get_width.return_value = 2
        bad.get_height.return_value = 2
        bad.get_format.return_value = "RGB"
        bad.get_data.return_value = np.zeros(1, dtype=np.uint8)
        good = Mock()
        good.get_width.return_value = 2
        good.get_height.return_value = 2
        good.get_format.return_value = "RGB"
        good.get_data.return_value = np.tile([255, 0, 0], 4).astype(np.uint8)
        frames = [SimpleNamespace(get_color_frame=lambda: bad), SimpleNamespace(get_color_frame=lambda: good)]
        def next_frames(_timeout):
            if frames:
                return frames.pop(0)
            worker._stop.wait(0.01)
            return None
        pipeline.wait_for_frames.side_effect = next_frames
        with patch.dict("sys.modules", pyorbbecsdk=sdk):
            try:
                self.assertTrue(worker.start())
                deadline = time.monotonic() + 2
                while not worker.ready() and time.monotonic() < deadline:
                    time.sleep(0.01)
                self.assertTrue(worker.ready())
                np.testing.assert_array_equal(worker.get_frame()[0, 0], [0, 0, 255])
                self.assertTrue(worker._thread.is_alive())
            finally:
                worker.stop()


class TestOwnerProcess(unittest.TestCase):
    def test_sigterm_exits_without_deadlock(self):
        import os
        import socket
        import subprocess
        import sys
        import tempfile
        from pathlib import Path

        with socket.socket() as sock:
            sock.bind(("127.0.0.1", 0))
            port = sock.getsockname()[1]
        cfg = configuration()
        cfg["rokae"]["owner"]["port"] = port
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp) / "config.json"
            path.write_text(json.dumps(cfg))
            process = subprocess.Popen([sys.executable, "-m", "vision.rokae_runtime", "--config", str(path)],
                                       stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
            try:
                deadline = time.monotonic() + 5
                while time.monotonic() < deadline:
                    try:
                        with urlopen(f"http://127.0.0.1:{port}/camera/health", timeout=0.2) as response:
                            self.assertTrue(json.load(response)["ok"])
                        break
                    except OSError:
                        time.sleep(0.05)
                else:
                    self.fail("owner did not start")
                process.terminate()
                self.assertEqual(process.wait(timeout=5), 0)
            finally:
                if process.poll() is None:
                    process.kill()
                process.wait()

    def test_three_streams_encode_and_decode(self):
        import subprocess
        import tempfile
        from pathlib import Path
        from vision.push import resolve_ffmpeg_bin
        from vision.service import CameraService, MediaService

        ffmpeg = resolve_ffmpeg_bin("")
        cfg = configuration()
        owner = RokaeCameraOwner(cfg)
        owner.start()
        server = serve_owner(owner)
        thread = threading.Thread(target=server.serve_forever, daemon=True)
        thread.start()
        media = None
        try:
            with tempfile.TemporaryDirectory() as tmp:
                cfg["adapter"] = "rokae"
                cfg["rokae"]["base_url"] = f"http://127.0.0.1:{server.server_port}"
                cfg["media"] = {"push": {"enabled": True, "device_sn": "LOCAL_TEST",
                    "stream_server_url": f"file://{tmp}", "ffmpeg_bin": ffmpeg,
                    "streams": [{"camera_id": cid, "stream_slot": index + 1,
                                 "enabled": True, "width": 32, "height": 24, "fps": 5}
                                for index, cid in enumerate(CAMERA_IDS)]}}
                media = MediaService(CameraService(cfg), cfg)
                self.assertTrue(media.start_push()["ok"])
                files = [Path(tmp) / f"{cid}.flv" for cid in CAMERA_IDS]
                deadline = time.monotonic() + 12
                while time.monotonic() < deadline:
                    if all(f.exists() and f.stat().st_size > 512 for f in files):
                        break
                    time.sleep(0.1)
                media.stop_push()
                for path in files:
                    self.assertGreater(path.stat().st_size, 512)
                    decoded = subprocess.run([ffmpeg, "-v", "error", "-i", str(path),
                        "-frames:v", "1", "-f", "null", "-"], capture_output=True, timeout=10)
                    self.assertEqual(decoded.returncode, 0, decoded.stderr.decode())
                self.assertTrue(owner.health()["all_ready"])
        finally:
            if media is not None:
                media.stop_push()
            server.shutdown()
            server.server_close()
            owner.stop()
            thread.join(2)
