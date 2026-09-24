import os
import sys
import unittest

PROJECT_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
if PROJECT_ROOT not in sys.path:
    sys.path.insert(0, PROJECT_ROOT)

from vision.adapters.fake import FakeCameraAdapter
from vision.adapters.rokae import RokaeCameraAdapter
from vision.adapters.tianji import TianjiCameraAdapter
from vision.service import CameraService, MediaService, build_adapter
import json


class TestCameraService(unittest.TestCase):
    def setUp(self) -> None:
        self.service = CameraService({"adapter": "fake"}, adapter=FakeCameraAdapter())

    def test_health_list_and_state(self) -> None:
        self.assertEqual(self.service.health(), {"ok": True})
        listing = self.service.listing()
        self.assertEqual(listing["cameras"][0]["camera_id"], "head")
        self.assertTrue(listing["cameras"][0]["ready"])
        self.assertFalse(listing["cameras"][2]["ready"])
        self.assertEqual(self.service.state()["self_check"]["status"], 0)

    def test_frame_aliases(self) -> None:
        head = self.service.frame("head")
        self.assertEqual(head["uri"], "fake://camera/head/color")
        left = self.service.frame("left_wrist")
        self.assertEqual(left["camera_id"], "hand_left")
        missing = self.service.frame("nope")
        self.assertEqual(missing["error_code"], "CAMERA_NOT_FOUND")

    def test_media_stream_uses_same_camera_ids(self) -> None:
        media = MediaService(self.service)
        body = media.stream("hand_right")
        self.assertEqual(body["camera_id"], "hand_right")
        self.assertTrue(body["uri"].endswith("/stream"))


class TestTianjiCameraAdapter(unittest.TestCase):
    def test_listing_maps_vendor_ids(self) -> None:
        def fake_http(url, timeout_sec):
            del timeout_sec
            if url.endswith("/camera/list"):
                return 200, {
                    "cameras": [
                        {"id": "head", "online": True},
                        {"id": "left_wrist", "online": False},
                        {"id": "right_wrist", "online": True},
                    ]
                }
            return 200, {"status": "READY"}

        import vision.adapters.tianji as tianji

        original = tianji._http_json
        tianji._http_json = fake_http
        try:
            adapter = TianjiCameraAdapter({"tianji": {"base_url": "http://127.0.0.1:8085"}})
            listing = adapter.listing()
            by_id = {row["camera_id"]: row for row in listing["cameras"]}
            self.assertTrue(by_id["head"]["ready"])
            self.assertFalse(by_id["hand_left"]["ready"])
            self.assertTrue(by_id["hand_right"]["ready"])
            frame = adapter.frame("hand_left")
            self.assertIn("camera=left_wrist", frame["uri"])
            self.assertEqual(adapter.stream_uri("head"), "http://127.0.0.1:8085/camera/stream?camera=head&type=color")
        finally:
            tianji._http_json = original


class TestRokaeCameraAdapter(unittest.TestCase):
    def test_listing_and_aliases(self) -> None:
        def fake_http(url, timeout_sec):
            del timeout_sec
            if url.endswith("/camera/list"):
                return 200, {
                    "cameras": [
                        {"id": "head", "online": True},
                        {"name": "left_wrist", "ready": True},
                        {"id": "hand_wrist", "enabled": False, "online": True},
                    ]
                }
            return 200, {"status": "READY"}

        import vision.adapters.rokae as rokae

        original = rokae._http_json
        rokae._http_json = fake_http
        try:
            adapter = RokaeCameraAdapter({"rokae": {"base_url": "http://127.0.0.1:8086"}})
            listing = adapter.listing()
            by_id = {row["camera_id"]: row for row in listing["cameras"]}
            self.assertTrue(by_id["head"]["ready"])
            self.assertTrue(by_id["hand_left"]["ready"])
            self.assertFalse(by_id["hand_right"]["ready"])
            frame = adapter.frame("left_wrist")
            self.assertEqual(frame["camera_id"], "hand_left")
            self.assertIn("camera=left_wrist", frame["uri"])
            not_ready = adapter.frame("hand_right")
            self.assertEqual(not_ready["error_code"], "CAMERA_NOT_READY")
            self.assertIn("camera=hand_wrist", adapter.stream_uri("hand_right"))
            self.assertEqual(
                adapter.stream_uri("head"),
                "http://127.0.0.1:8086/camera/stream?camera=head&type=color",
            )
            built = build_adapter({"adapter": "rokae", "rokae": {"base_url": "http://127.0.0.1:8086"}})
            self.assertIsInstance(built, RokaeCameraAdapter)
            with self.assertRaises(ValueError):
                build_adapter({"adapter": "luoshi"})
            with self.assertRaises(ValueError):
                build_adapter({"adapter": "helios"})
        finally:
            rokae._http_json = original


class TestVisionHttpRoutes(unittest.TestCase):
    def test_camera_and_media_routes(self) -> None:
        from fastapi.testclient import TestClient

        from vision.http_app import create_camera_app, create_media_app

        camera = TestClient(create_camera_app(CameraService({"adapter": "fake"})))
        head = camera.get("/frame/head")
        self.assertEqual(head.status_code, 200)
        self.assertEqual(head.json()["camera_id"], "head")
        missing = camera.get("/frame/nope")
        self.assertEqual(missing.status_code, 404)
        self.assertEqual(missing.json()["error_code"], "CAMERA_NOT_FOUND")
        unknown = camera.get("/nope")
        self.assertEqual(unknown.status_code, 404)
        self.assertEqual(unknown.json()["error_code"], "NOT_FOUND")
        media = TestClient(create_media_app(MediaService(CameraService({"adapter": "fake"}))))
        stream = media.get("/stream/head")
        self.assertEqual(stream.status_code, 200)
        self.assertIn("uri", stream.json())
        status = media.get("/push")
        self.assertEqual(status.status_code, 200)
        self.assertTrue(status.json()["ok"])
        started = media.post("/push/start", json={"device_sn": "ROBOT_100", "camera_id": "head"})
        self.assertEqual(started.status_code, 200)
        self.assertEqual(started.json()["streams"][0]["camera_id"], "head")
        self.assertFalse(started.json()["streams"][0]["online"])


class FakeProcess:
    def __init__(self) -> None:
        self.pid = 4242
        self._code = None
        self.stdin = None
        self.stdout = None
        self.stderr = None

    def poll(self):
        return self._code

    def terminate(self) -> None:
        self._code = 0

    def kill(self) -> None:
        self._code = 9

    def wait(self, timeout=None):
        del timeout
        if self._code is None:
            self._code = 0
        return self._code


class TestRtmpPush(unittest.TestCase):
    def test_build_rtmp_target_matches_smt_robot_path(self) -> None:
        from vision.rtmp_target import build_rtmp_target

        url = build_rtmp_target(
            "rtmp://robotsolution.cn:1935",
            "",
            device_sn="ROBOT_100",
            stream_slot=1,
            camera_index=99,
            video_channel_index=0,
        )
        self.assertEqual(
            url,
            "rtmp://robotsolution.cn:1935/dksl/stream/ROBOT_100_1_99-0-0_normal-0",
        )
        signed = build_rtmp_target(
            "rtmp://robotsolution.cn:1935",
            "secret-sign",
            device_sn="ROBOT_100",
            stream_slot=1,
            camera_index=99,
            video_channel_index=0,
        )
        self.assertEqual(
            signed,
            "rtmp://robotsolution.cn:1935/dksl/stream/ROBOT_100_1_99-0-0_normal-0?sign=secret-sign",
        )

    def test_pick_stderr_prefers_auth_line(self) -> None:
        from vision.push import pick_stderr_hint

        hint = pick_stderr_hint(
            [
                "[rtmp @ 0x1] Server error: [auth failed]: code:401 msg:\"Unauthorized\"",
                "rtmp://robotsolution.cn:1935/dksl/stream/ROBOT_100_1_99-0-0_normal-0: Operation not permitted",
            ]
        )
        self.assertIn("401", hint)
        self.assertIn("auth failed", hint)

    def test_push_runtime_spawns_ffmpeg_and_stops(self) -> None:
        from vision.push import PushRuntime

        spawned = []

        def fake_popen(command, **kwargs):
            del kwargs
            spawned.append(command)
            return FakeProcess()

        runtime = PushRuntime(
            {
                "media": {
                    "push": {
                        "enabled": True,
                        "device_sn": "ROBOT_100",
                        "stream_server_url": "rtmp://robotsolution.cn:1935",
                        "streams": [
                            {"camera_id": "head", "stream_slot": 1, "enabled": True}
                        ],
                    }
                }
            },
            source_uri=lambda camera_id: f"http://127.0.0.1:8085/camera/stream?camera={camera_id}",
            popen_factory=fake_popen,
        )
        result = runtime.start(device_sn="ROBOT_100")
        self.assertTrue(result["ok"])
        self.assertTrue(result["streams"][0]["online"])
        self.assertIn("libx264", spawned[0])
        # HTTP Owner 流是 multipart → mpjpeg demuxer
        idx = spawned[0].index("-f")
        self.assertEqual(spawned[0][idx + 1], "mpjpeg")
        self.assertEqual(
            spawned[0][spawned[0].index("-i") + 1],
            "http://127.0.0.1:8085/camera/stream?camera=head",
        )
        self.assertEqual(spawned[0][-1], "rtmp://robotsolution.cn:1935/dksl/stream/ROBOT_100_1_99-0-0_normal-0")
        stopped = runtime.stop()
        self.assertFalse(stopped["streams"][0]["online"])

    def test_http_source_uses_mpjpeg_demuxer(self) -> None:
        from vision.push import _ffmpeg_command, _input_demuxer, resolve_ffmpeg_bin, resolve_push_target

        self.assertEqual(_input_demuxer("http://127.0.0.1:8086/camera/stream?camera=head"), "mpjpeg")
        self.assertEqual(_input_demuxer("https://example/stream"), "mpjpeg")
        self.assertEqual(_input_demuxer("/tmp/frames.mjpg"), "mjpeg")
        self.assertTrue(resolve_ffmpeg_bin(""))
        cmd = _ffmpeg_command(
            ffmpeg_bin="ffmpeg",
            source="http://127.0.0.1:8086/camera/stream?camera=head",
            target="rtmp://127.0.0.1/live/x",
            width=640,
            height=480,
            fps=10,
            bitrate="1200k",
            preset="ultrafast",
        )
        self.assertEqual(cmd[cmd.index("-f") + 1], "mpjpeg")
        file_target = resolve_push_target(
            "file:///tmp/vision-media-unit",
            "",
            device_sn="ROBOT_100",
            stream_slot=1,
            camera_id="head",
        )
        self.assertTrue(file_target.endswith("/head.flv"))
        self.assertTrue(file_target.startswith("/tmp/vision-media-unit"))


class TestRokaeMediaPipeline(unittest.TestCase):
    def test_media_consumes_owner_uri_not_device(self) -> None:
        import threading
        from urllib.request import urlopen

        from vision.rokae_runtime.http_app import serve_owner
        from vision.rokae_runtime.owner import RokaeCameraOwner

        config = {
            "adapter": "rokae",
            "rokae": {
                "base_url": "http://127.0.0.1:18088",
                "owner": {
                    "host": "127.0.0.1",
                    "port": 18088,
                    "cameras": {
                        "head": {
                            "enabled": True,
                            "backend": "fake",
                            "match": {"type": "fake"},
                            "width": 64,
                            "height": 48,
                        },
                        "hand_left": {"enabled": False, "backend": "fake", "match": {"type": "fake"}},
                        "hand_right": {"enabled": False, "backend": "fake", "match": {"type": "fake"}},
                    },
                },
            },
            "media": {
                "push": {
                    "enabled": True,
                    "device_sn": "ROBOT_MEDIA",
                    "stream_server_url": "rtmp://127.0.0.1:1935",
                    "streams": [{"camera_id": "head", "stream_slot": 1, "enabled": True}],
                }
            },
        }
        owner = RokaeCameraOwner(config)
        owner.start()
        server = serve_owner(owner)
        thread = threading.Thread(target=server.serve_forever, daemon=True)
        thread.start()
        spawned = []

        def fake_popen(command, **kwargs):
            del kwargs
            spawned.append(list(command))
            return FakeProcess()

        try:
            camera = CameraService(config)
            media = MediaService(camera, config)
            # monkey-patch popen on existing workers
            media.push._popen_factory = fake_popen
            for worker in media.push._workers.values():
                worker._popen_factory = fake_popen

            stream = media.stream("head")
            self.assertIn("18088", stream["uri"])
            self.assertIn("/camera/stream", stream["uri"])
            self.assertNotIn("/dev/video", stream["uri"])

            started = media.start_push(camera_id="head", device_sn="ROBOT_MEDIA")
            self.assertTrue(started["ok"])
            self.assertTrue(spawned)
            cmd = spawned[0]
            joined = " ".join(cmd)
            self.assertIn("http://127.0.0.1:18088/camera/stream", joined)
            self.assertNotIn("/dev/video", joined)
            self.assertEqual(cmd[cmd.index("-f") + 1], "mpjpeg")
            # Owner 至少能出 JPEG（Media 不读设备）
            jpeg = urlopen("http://127.0.0.1:18088/camera/snapshot?camera=head", timeout=2).read()
            self.assertGreater(len(jpeg), 100)
            self.assertEqual(jpeg[:2], b"\xff\xd8")
        finally:
            media.stop_push()
            server.shutdown()
            server.server_close()
            owner.stop()


class TestRokaeRuntimeOwner(unittest.TestCase):
    def test_fake_owner_http_contract(self) -> None:
        import threading
        from urllib.request import urlopen

        from vision.rokae_runtime.http_app import serve_owner
        from vision.rokae_runtime.owner import RokaeCameraOwner

        config = {
            "rokae": {
                "owner": {
                    "host": "127.0.0.1",
                    "port": 18087,
                    "cameras": {
                        "head": {"enabled": True, "backend": "fake", "match": {"type": "fake"}, "width": 64, "height": 48},
                        "hand_left": {"enabled": True, "backend": "fake", "match": {"type": "fake"}, "width": 64, "height": 48},
                        "hand_right": {"enabled": False, "backend": "fake", "match": {"type": "fake"}},
                    },
                }
            }
        }
        owner = RokaeCameraOwner(config)
        owner.start()
        server = serve_owner(owner)
        thread = threading.Thread(target=server.serve_forever, daemon=True)
        thread.start()
        try:
            health = json.loads(urlopen("http://127.0.0.1:18087/camera/health", timeout=2).read().decode())
            self.assertEqual(health["status"], "READY")
            listing = json.loads(urlopen("http://127.0.0.1:18087/camera/list", timeout=2).read().decode())
            by_id = {row["id"]: row for row in listing["cameras"]}
            self.assertTrue(by_id["head"]["ready"])
            self.assertTrue(by_id["left_wrist"]["ready"])
            self.assertFalse(by_id["hand_wrist"]["ready"])
            jpeg = urlopen("http://127.0.0.1:18087/camera/snapshot?camera=head", timeout=2).read()
            self.assertGreater(len(jpeg), 100)
            self.assertEqual(jpeg[:2], b"\xff\xd8")
            adapter = RokaeCameraAdapter({"rokae": {"base_url": "http://127.0.0.1:18087"}})
            frame = adapter.frame("head")
            self.assertIn("camera=head", frame["uri"])
            self.assertIn("18087", frame["uri"])
            self.assertTrue(adapter.ready())
        finally:
            server.shutdown()
            server.server_close()
            owner.stop()


if __name__ == "__main__":
    unittest.main()
