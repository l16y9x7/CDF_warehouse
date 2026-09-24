import json
import os
import sys
import tempfile
import unittest
from pathlib import Path
from typing import Any, Dict
from urllib.error import URLError


PROJECT_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
if PROJECT_ROOT not in sys.path:
    sys.path.insert(0, PROJECT_ROOT)

from gateway.debug_http import create_app
from gateway.map_sync import MapSyncError, MapSyncService, occupancy_from_nav_json
from gateway.mqtt.osd import OsdReporter
from gateway.records import TaskStateStore


LOAD_MAP_URL = "http://127.0.0.1:8001/load_map"
TINY_OCCUPANCY = {
    "accepted": True,
    "map_id": "42",
    "occupancy": {
        "width": 2,
        "height": 1,
        "resolution": 0.1,
        "origin": {"x": 0.0, "y": 0.0},
        "data": [100, 100],
    },
}


class FakeResponse:
    def __init__(self, payload, status: int = 200) -> None:
        self.status = status
        if isinstance(payload, (bytes, bytearray)):
            self._raw = bytes(payload)
        else:
            self._raw = json.dumps(payload).encode("utf-8")

    def getcode(self) -> int:
        return self.status

    def read(self, size: int = -1) -> bytes:
        if size is None or size < 0:
            return self._raw
        return self._raw[:size]

    def __enter__(self) -> "FakeResponse":
        return self

    def __exit__(self, exc_type, exc, tb) -> bool:
        del exc_type, exc, tb
        return False


class FakeCollector:
    def __init__(self, snapshot: Dict[str, Any]) -> None:
        self._snapshot = snapshot
        self.map_sync = None

    def snapshot(self) -> Dict[str, Any]:
        return dict(self._snapshot)


def _opener(uploaded, *, occupancy=None, fail_upload: bool = False):
    grid = TINY_OCCUPANCY if occupancy is None else occupancy

    def opener(request, timeout=0):
        del timeout
        if "/load_map" in request.get_full_url():
            return FakeResponse(grid)
        if fail_upload:
            raise URLError("offline")
        body = json.loads(request.data.decode("utf-8"))
        uploaded.append(json.loads(body["data"]))
        return FakeResponse({"code": 0})

    return opener


class TestOccupancy(unittest.TestCase):
    def test_occupancy_from_nav_json(self) -> None:
        grid = occupancy_from_nav_json(
            {
                "accepted": True,
                "map_id": "SMT_test",
                "occupancy": {
                    "width": 2,
                    "height": 1,
                    "resolution": 0.05,
                    "origin": {"x": -1.0, "y": -2.0},
                    "data": [0, 100],
                },
            }
        )
        self.assertIsNotNone(grid)
        assert grid is not None
        self.assertEqual((grid.width, grid.height), (2, 1))
        self.assertEqual(grid.origin_x, -1.0)
        self.assertEqual(grid.origin_y, -2.0)
        self.assertEqual(grid.data, (0, 100))
        self.assertIsNone(occupancy_from_nav_json({"accepted": True, "map_id": "SMT_test"}))

    def test_nav_occupancy_keeps_resolution_origin(self) -> None:
        grid = occupancy_from_nav_json(
            {
                "occupancy": {
                    "width": 3,
                    "height": 2,
                    "resolution": 0.002,
                    "origin": {"x": 1.25, "y": -3.5},
                    "data": [0, 100, -1, 0, 100, 0],
                }
            }
        )
        self.assertIsNotNone(grid)
        assert grid is not None
        self.assertEqual((grid.width, grid.height), (3, 2))
        self.assertEqual(grid.resolution, 0.002)
        self.assertEqual(grid.origin_x, 1.25)
        self.assertEqual(grid.origin_y, -3.5)
        self.assertEqual(grid.data, (0, 100, -1, 0, 100, 0))


class TestMapSyncService(unittest.TestCase):
    def setUp(self) -> None:
        self._temp = tempfile.TemporaryDirectory()
        self.root = Path(self._temp.name)
        self.uploaded = []
        self.service = MapSyncService(
            {
                "device": {"sn": "ROBOT_100"},
                "map_sync": {
                    "enabled": True,
                    "report_url": "https://example.test/robotDog/map",
                    "identity_file": str(self.root / "map_index.json"),
                    "default_source_map_id": "42",
                    "load_map_url": LOAD_MAP_URL,
                },
            },
            opener=_opener(self.uploaded),
            clock=lambda: 1.0,
        )

    def tearDown(self) -> None:
        self._temp.cleanup()

    def test_sync_uploads_uuid_and_rewrites_osd(self) -> None:
        result = self.service.sync()
        self.assertEqual(result["source_map_id"], "42")
        self.assertTrue(result["map_id"])
        self.assertTrue(result["uploaded"])
        self.assertEqual(result["source_path"], LOAD_MAP_URL)
        self.assertEqual(self.uploaded[0]["sn"], "ROBOT_100")
        self.assertEqual(self.uploaded[0]["map_id"], result["map_id"])
        self.assertEqual(self.uploaded[0]["info"]["width"], 2)

        again = self.service.sync(source_map_id="42")
        self.assertEqual(again["map_id"], result["map_id"])
        self.assertEqual(len(self.uploaded), 2)

        collector = FakeCollector(
            {"map": {"map_id": "42", "stations": [{"station_id": "LM1"}]}}
        )
        collector.map_sync = self.service
        payload = OsdReporter(
            config={"device": {"sn": "ROBOT_100", "deviceType": "term"}},
            mqtt_client=object(),
            collector=collector,
        ).collect_osd_data()
        self.assertEqual(payload["data"]["map"]["map_id"], result["map_id"])
        self.assertEqual(payload["data"]["map"]["stations"][0]["station_id"], "LM1")

    def test_request_auth_headers_override_config(self) -> None:
        seen = []

        def opener(request, timeout=0):
            del timeout
            if "/load_map" in request.get_full_url():
                return FakeResponse(TINY_OCCUPANCY)
            seen.append(dict(request.header_items()))
            return FakeResponse({"code": 0})

        service = MapSyncService(
            {
                "device": {"sn": "ROBOT_100"},
                "map_sync": {
                    "report_url": "https://example.test/robotDog/map",
                    "identity_file": str(self.root / "auth.json"),
                    "default_source_map_id": "42",
                    "load_map_url": LOAD_MAP_URL,
                    "platform_api": {
                        "resource": "from-config",
                        "resource_token": "config-token",
                    },
                },
            },
            opener=opener,
        )
        service.sync(resource="robot_dog_service", resource_token="once-token")
        headers = {key.lower(): value for key, value in seen[0].items()}
        self.assertEqual(headers.get("resource"), "robot_dog_service")
        self.assertEqual(headers.get("resource-token"), "once-token")

    def test_partial_request_auth_is_rejected(self) -> None:
        service = MapSyncService(
            {
                "device": {"sn": "ROBOT_100"},
                "map_sync": {
                    "report_url": "https://example.test/robotDog/map",
                    "identity_file": str(self.root / "partial.json"),
                    "default_source_map_id": "42",
                    "load_map_url": LOAD_MAP_URL,
                },
            },
            opener=_opener([]),
        )
        with self.assertRaises(MapSyncError) as ctx:
            service.sync(resource="robot_dog_service")
        self.assertEqual(ctx.exception.code, "MAP_SYNC_AUTH_INVALID")

    def test_upload_failure_does_not_rewrite_osd(self) -> None:
        service = MapSyncService(
            {
                "device": {"sn": "ROBOT_100"},
                "map_sync": {
                    "report_url": "https://example.test/robotDog/map",
                    "identity_file": str(self.root / "fail.json"),
                    "default_source_map_id": "42",
                    "load_map_url": LOAD_MAP_URL,
                },
            },
            opener=_opener([], fail_upload=True),
        )
        with self.assertRaises(MapSyncError) as ctx:
            service.sync()
        self.assertEqual(ctx.exception.code, "MAP_SYNC_UPLOAD_FAILED")
        collector = FakeCollector({"map": {"map_id": "42"}})
        collector.map_sync = service
        payload = OsdReporter(
            config={"device": {"sn": "ROBOT_100"}},
            mqtt_client=object(),
            collector=collector,
        ).collect_osd_data()
        self.assertEqual(payload["data"]["map"]["map_id"], "42")

    def test_ready_nav_uses_new_map_uuid(self) -> None:
        collector = FakeCollector(
            {
                "self_check": {"status": 0, "message": "就绪"},
                "map": {"map_id": "floor_b"},
            }
        )
        service = MapSyncService(
            {
                "device": {"sn": "ROBOT_001"},
                "map_sync": {
                    "enabled": True,
                    "report_url": "https://example.test/robotDog/map",
                    "identity_file": str(self.root / "ready_nav.json"),
                    "default_source_map_id": "test_14",
                    "load_map_url": LOAD_MAP_URL,
                },
            },
            collector=collector,
            opener=_opener(self.uploaded),
            clock=lambda: 1.0,
        )
        default = service.sync(source_map_id="test_14")
        current = service.sync()
        self.assertEqual(current["source_map_id"], "floor_b")
        self.assertNotEqual(current["map_id"], default["map_id"])
        collector.map_sync = service
        payload = OsdReporter(
            config={"device": {"sn": "ROBOT_001"}},
            mqtt_client=object(),
            collector=collector,
        ).collect_osd_data()
        self.assertEqual(payload["data"]["map"]["map_id"], current["map_id"])

    def test_sync_fetches_nav_load_map(self) -> None:
        uploaded = []

        def opener(request, timeout=0):
            del timeout
            url = request.get_full_url()
            if "/load_map" in url:
                body = json.loads(request.data.decode("utf-8"))
                self.assertEqual(body.get("map_id"), "SMT_test")
                return FakeResponse(
                    {
                        "accepted": True,
                        "map_id": "SMT_test",
                        "occupancy": {
                            "width": 2,
                            "height": 2,
                            "resolution": 0.05,
                            "origin": [0.0, 0.0],
                            "data": [0, 100, 100, 0],
                        },
                    }
                )
            payload = json.loads(request.data.decode("utf-8"))
            uploaded.append(json.loads(payload["data"]))
            return FakeResponse({"code": 0})

        service = MapSyncService(
            {
                "device": {"sn": "ROBOT_001"},
                "state": {"navigation": {"url": "http://127.0.0.1:8001"}},
                "map_sync": {
                    "enabled": True,
                    "report_url": "https://example.test/robotDog/map",
                    "identity_file": str(self.root / "nav_index.json"),
                    "default_source_map_id": "test_zhongmian",
                    "load_map_url": "auto",
                },
            },
            opener=opener,
            clock=lambda: 1.0,
        )
        result = service.sync(source_map_id="SMT_test")
        self.assertTrue(result["uploaded"])
        self.assertEqual(result["width"], 2)
        self.assertEqual(result["height"], 2)
        self.assertEqual(result["origin"], {"x": 0.0, "y": 0.0})
        self.assertEqual(uploaded[0]["sn"], "ROBOT_001")
        self.assertEqual(service.load_map_url, LOAD_MAP_URL)

    def test_nav_occupancy_uploaded_as_is(self) -> None:
        uploaded = []

        def opener(request, timeout=0):
            del timeout
            url = request.get_full_url()
            if "/load_map" in url:
                return FakeResponse(
                    {
                        "accepted": True,
                        "map_id": "SMT_test",
                        "occupancy": {
                            "width": 3,
                            "height": 2,
                            "resolution": 0.002,
                            "origin": {"x": -0.53, "y": -0.33},
                            "data": [0, 100, -1, 0, 100, 0],
                        },
                    }
                )
            payload = json.loads(request.data.decode("utf-8"))
            uploaded.append(json.loads(payload["data"]))
            return FakeResponse({"code": 0})

        service = MapSyncService(
            {
                "device": {"sn": "ROBOT_001"},
                "state": {"navigation": "http://127.0.0.1:8001"},
                "map_sync": {
                    "enabled": True,
                    "report_url": "https://example.test/robotDog/map",
                    "identity_file": str(self.root / "as_is.json"),
                    "default_source_map_id": "test_zhongmian",
                },
            },
            opener=opener,
            clock=lambda: 1.0,
        )
        result = service.sync(source_map_id="SMT_test")
        self.assertEqual(result["resolution"], 0.002)
        self.assertEqual(result["origin"], {"x": -0.53, "y": -0.33})
        self.assertEqual(result["width"], 3)
        self.assertEqual(result["height"], 2)
        info = uploaded[0]["info"]
        self.assertEqual(info["resolution"], 0.002)
        self.assertEqual(info["origin"]["position"]["x"], -0.53)
        self.assertEqual(info["origin"]["position"]["y"], -0.33)
        self.assertEqual(uploaded[0]["data"], [0, 100, -1, 0, 100, 0])

    def test_state_map_id_change_posts_load_map(self) -> None:
        load_calls: list[str] = []

        def opener(request, timeout=0):
            del timeout
            url = request.get_full_url()
            if "/load_map" in url:
                body = json.loads(request.data.decode("utf-8"))
                map_id = str(body.get("map_id") or "")
                load_calls.append(map_id)
                return FakeResponse(
                    {
                        "accepted": True,
                        "map_id": map_id,
                        "occupancy": {
                            "width": 1,
                            "height": 1,
                            "resolution": 0.05,
                            "origin": [0.0, 0.0],
                            "data": [0],
                        },
                    }
                )
            return FakeResponse({"code": 0})

        collector = FakeCollector(
            {
                "self_check": {"status": 0, "message": "就绪"},
                "map": {"map_id": "map_a"},
            }
        )
        service = MapSyncService(
            {
                "device": {"sn": "ROBOT_001"},
                "state": {"navigation": {"url": "http://127.0.0.1:8001"}},
                "map_sync": {
                    "enabled": True,
                    "report_url": "https://example.test/robotDog/map",
                    "identity_file": str(self.root / "change_index.json"),
                    "default_source_map_id": "test_zhongmian",
                    "load_map_url": "auto",
                },
            },
            collector=collector,
            opener=opener,
            clock=lambda: 1.0,
        )
        service.ensure_active_map()
        first = service._ensure_thread
        self.assertIsNotNone(first)
        first.join(timeout=2.0)
        self.assertEqual(load_calls, ["map_a"])
        service.ensure_active_map()
        self.assertEqual(load_calls, ["map_a"])
        collector._snapshot = {
            "self_check": {"status": 0, "message": "就绪"},
            "map": {"map_id": "map_b"},
        }
        service.ensure_active_map()
        second = service._ensure_thread
        self.assertIsNotNone(second)
        second.join(timeout=2.0)
        self.assertEqual(load_calls, ["map_a", "map_b"])
        collector._snapshot = {
            "self_check": {"status": 0, "message": "就绪"},
            "map": {"map_id": "NO_MAP"},
        }
        service.ensure_active_map()
        self.assertEqual(load_calls, ["map_a", "map_b"])

    def test_ensure_active_map_uploads_when_nav_becomes_ready(self) -> None:
        collector = FakeCollector({})
        service = MapSyncService(
            {
                "device": {"sn": "ROBOT_001"},
                "map_sync": {
                    "enabled": True,
                    "report_url": "https://example.test/robotDog/map",
                    "identity_file": str(self.root / "ensure.json"),
                    "default_source_map_id": "test_14",
                    "load_map_url": LOAD_MAP_URL,
                },
            },
            collector=collector,
            opener=_opener(self.uploaded),
            clock=lambda: 1.0,
        )
        default = service.sync()
        collector._snapshot = {
            "self_check": {"status": 0, "message": "就绪"},
            "map": {"map_id": "floor_b"},
        }
        service.ensure_active_map()
        thread = service._ensure_thread
        self.assertIsNotNone(thread)
        thread.join(timeout=2.0)
        current = service.identities.peek("floor_b")
        self.assertIsNotNone(current)
        self.assertTrue(current.last_upload_ok)
        self.assertNotEqual(current.map_id, default["map_id"])

    def test_on_platform_ready_retries_failed_upload(self) -> None:
        attempts = {"n": 0}

        def opener(request, timeout=0):
            del timeout
            if "/load_map" in request.get_full_url():
                return FakeResponse(TINY_OCCUPANCY)
            attempts["n"] += 1
            if attempts["n"] == 1:
                raise URLError("offline")
            body = json.loads(request.data.decode("utf-8"))
            self.uploaded.append(json.loads(body["data"]))
            return FakeResponse({"code": 0})

        service = MapSyncService(
            {
                "device": {"sn": "ROBOT_100"},
                "map_sync": {
                    "enabled": True,
                    "sync_on_start": True,
                    "report_url": "https://example.test/robotDog/map",
                    "identity_file": str(self.root / "retry.json"),
                    "default_source_map_id": "42",
                    "load_map_url": LOAD_MAP_URL,
                },
            },
            opener=opener,
            clock=lambda: 1.0,
        )
        with self.assertRaises(MapSyncError) as ctx:
            service.sync()
        self.assertEqual(ctx.exception.code, "MAP_SYNC_UPLOAD_FAILED")
        identity = service.identities.peek("42")
        self.assertIsNotNone(identity)
        self.assertFalse(identity.last_upload_ok)
        service.on_platform_ready()
        identity = service.identities.peek("42")
        self.assertTrue(identity.last_upload_ok)
        self.assertEqual(attempts["n"], 2)

    def test_on_platform_ready_skips_when_already_uploaded(self) -> None:
        service = MapSyncService(
            {
                "device": {"sn": "ROBOT_100"},
                "map_sync": {
                    "enabled": True,
                    "sync_on_start": True,
                    "report_url": "https://example.test/robotDog/map",
                    "identity_file": str(self.root / "skip.json"),
                    "default_source_map_id": "42",
                    "load_map_url": LOAD_MAP_URL,
                },
            },
            opener=_opener(self.uploaded),
            clock=lambda: 1.0,
        )
        service.sync()
        uploaded = len(self.uploaded)
        service.on_platform_ready()
        self.assertEqual(len(self.uploaded), uploaded)

    def test_on_gateway_start_uploads_even_if_already_ok(self) -> None:
        load_calls: list[str] = []
        uploaded: list[Any] = []

        def opener(request, timeout=0):
            del timeout
            url = request.get_full_url()
            if "/load_map" in url:
                body = json.loads(request.data.decode("utf-8"))
                map_id = str(body.get("map_id") or "")
                load_calls.append(map_id)
                return FakeResponse(
                    {
                        "accepted": True,
                        "map_id": map_id,
                        "occupancy": {
                            "width": 1,
                            "height": 1,
                            "resolution": 0.05,
                            "origin": {"x": -0.53, "y": -0.33},
                            "data": [0],
                        },
                    }
                )
            uploaded.append(json.loads(request.data.decode("utf-8")))
            return FakeResponse({"code": 0})

        collector = FakeCollector(
            {
                "self_check": {"status": 0, "message": "就绪"},
                "map": {"map_id": "test_zhongmian"},
            }
        )
        service = MapSyncService(
            {
                "device": {"sn": "ROBOT_001"},
                "state": {"navigation": {"url": "http://127.0.0.1:8001"}},
                "map_sync": {
                    "enabled": True,
                    "sync_on_start": True,
                    "report_url": "https://example.test/robotDog/map",
                    "identity_file": str(self.root / "start_refresh.json"),
                    "default_source_map_id": "test_zhongmian",
                    "load_map_url": "auto",
                },
            },
            collector=collector,
            opener=opener,
            clock=lambda: 1.0,
        )
        first = service.sync()
        self.assertTrue(first["uploaded"])
        self.assertEqual(load_calls, ["test_zhongmian"])
        self.assertEqual(len(uploaded), 1)
        service.on_gateway_start()
        self.assertEqual(load_calls, ["test_zhongmian", "test_zhongmian"])
        self.assertEqual(len(uploaded), 2)
        self.assertEqual(first["map_id"], service.identities.peek("test_zhongmian").map_id)

    def test_load_map_missing_occupancy_does_not_upload(self) -> None:
        def opener(request, timeout=0):
            del timeout
            if "/load_map" in request.get_full_url():
                return FakeResponse({"accepted": True, "map_id": "test_zhongmian"})
            raise AssertionError("missing occupancy must not upload")

        service = MapSyncService(
            {
                "device": {"sn": "ROBOT_001"},
                "state": {"navigation": {"url": "http://127.0.0.1:8001"}},
                "map_sync": {
                    "enabled": True,
                    "report_url": "https://example.test/robotDog/map",
                    "identity_file": str(self.root / "no_occ.json"),
                    "default_source_map_id": "test_zhongmian",
                    "load_map_url": "auto",
                },
            },
            opener=opener,
            clock=lambda: 1.0,
        )
        with self.assertRaises(MapSyncError) as ctx:
            service.sync(source_map_id="test_zhongmian")
        self.assertEqual(ctx.exception.code, "MAP_SYNC_BMAP_NOT_FOUND")


class TestMapSyncHttp(unittest.TestCase):
    def test_fastapi_map_sync_routes(self) -> None:
        from fastapi.testclient import TestClient

        temp = tempfile.TemporaryDirectory()
        root = Path(temp.name)

        class _Gateway:
            def __init__(self) -> None:
                self.task_state = TaskStateStore()
                self.map_sync = MapSyncService(
                    {
                        "device": {"sn": "ROBOT_100"},
                        "map_sync": {
                            "report_url": "https://example.test/robotDog/map",
                            "identity_file": str(root / "map_index.json"),
                            "default_source_map_id": "42",
                            "load_map_url": LOAD_MAP_URL,
                        },
                    },
                    opener=_opener([]),
                )

        try:
            client = TestClient(create_app(_Gateway()))
            missing = client.get("/map/sync")
            self.assertEqual(missing.status_code, 200)
            self.assertEqual(missing.json()["current_source_map_id"], "42")
            self.assertEqual(missing.json()["current"], {})
            synced = client.post("/map/sync", json={})
            self.assertEqual(synced.status_code, 200)
            self.assertTrue(synced.json()["uploaded"])
            self.assertTrue(synced.json()["map_id"])
            after = client.get("/map/sync")
            self.assertTrue(after.json()["current"]["last_upload_ok"])
        finally:
            temp.cleanup()


if __name__ == "__main__":
    unittest.main()
