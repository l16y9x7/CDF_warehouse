import json
import os
import sys
import tempfile
import threading
import time
import unittest

PROJECT_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
if PROJECT_ROOT not in sys.path:
    sys.path.insert(0, PROJECT_ROOT)

from navigation.adapters.fake import FakeNavigationAdapter
from navigation.adapters.tianji import TianjiNavigationAdapter
from navigation.service import NavigationService


def _ctx(**overrides):
    body = {
        "task_id": "TASK-1",
        "request_id": "REQ-1",
        "timeout_sec": 8,
        "idempotency_key": "KEY-1",
        "station_id": "start",
    }
    body.update(overrides)
    return body


class TestNavigationService(unittest.TestCase):
    def setUp(self) -> None:
        self.adapter = FakeNavigationAdapter()
        self.service = NavigationService({"adapter": "fake"}, adapter=self.adapter)

    def test_health_and_state_include_pose_and_map(self) -> None:
        self.assertEqual(self.service.health(), {"ok": True})
        state = self.service.state()
        self.assertEqual(state["self_check"]["status"], 0)
        self.assertEqual(state["position"]["x"], 0.17)
        self.assertEqual(state["map"]["map_id"], "fake-map")
        self.assertTrue(any(row["station_id"] == "start" for row in state["map"]["stations"]))

    def test_missing_context_is_rejected(self) -> None:
        result = self.service.goto({"station_id": "start"})
        self.assertFalse(result["accepted"])
        self.assertEqual(result["terminal_state"], "REJECTED")
        self.assertEqual(result["error_code"], "NAVIGATION_INVALID_REQUEST")

    def test_unknown_station_is_rejected_by_adapter(self) -> None:
        result = self.service.goto(_ctx(station_id="no-such"))
        self.assertFalse(result["accepted"])
        self.assertEqual(result["terminal_state"], "REJECTED")
        self.assertEqual(result["error_code"], "NAVIGATION_UNKNOWN_STATION")

    def test_goto_returns_handle_then_succeeds(self) -> None:
        result = self.service.goto(_ctx())
        self.assertTrue(result["accepted"])
        self.assertEqual(result["request_id"], "REQ-1")
        self.assertIn(result["state"], {"ACCEPTED", "RUNNING", "SUCCEEDED"})
        deadline = time.monotonic() + 1.0
        status = {}
        while time.monotonic() < deadline:
            status = self.service.status_of("REQ-1")
            if status.get("terminal_state") == "SUCCEEDED":
                break
            time.sleep(0.01)
        self.assertEqual(status["terminal_state"], "SUCCEEDED")
        self.assertTrue(status["evidence"]["arrived"])

    def test_idempotency_returns_same_action(self) -> None:
        first = self.service.goto(_ctx())
        second = self.service.goto(_ctx(request_id="REQ-2"))
        self.assertEqual(first["request_id"], second["request_id"])
        conflict = self.service.goto(_ctx(request_id="REQ-3", station_id="station.home"))
        self.assertEqual(conflict["error_code"], "NAVIGATION_IDEMPOTENCY_CONFLICT")

    def test_busy_rejects_second_goto(self) -> None:
        gate = threading.Event()

        class SlowAdapter(FakeNavigationAdapter):
            def goto(self, **kwargs):
                gate.wait(timeout=2.0)
                return super().goto(**kwargs)

        service = NavigationService({"adapter": "fake"}, adapter=SlowAdapter())
        first = service.goto(_ctx())
        self.assertTrue(first["accepted"])
        second = service.goto(_ctx(request_id="REQ-2", idempotency_key="KEY-2", station_id="station.home"))
        self.assertFalse(second["accepted"])
        self.assertEqual(second["error_code"], "RESOURCE_BUSY")
        gate.set()

    def test_not_ready_rejects(self) -> None:
        self.adapter.set_ready(False)
        result = self.service.goto(_ctx())
        self.assertEqual(result["error_code"], "NAVIGATION_NOT_READY")

    def test_fake_stop_and_load_map(self) -> None:
        stop = self.service.stop({})
        self.assertTrue(stop["accepted"])
        loaded = self.service.load_map({"map_id": "m2"})
        self.assertTrue(loaded["accepted"])
        self.assertEqual(self.service.map_info()["map_id"], "m2")

    def test_load_map_saves_and_applies_unified_stations(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            service = NavigationService(
                {"adapter": "fake", "maps_dir": tmp},
                adapter=FakeNavigationAdapter(),
            )
            loaded = service.load_map(
                {
                    "map_id": "duty-ab",
                    "stations": [
                        {"station_id": "AGV_L", "vendor_id": "1", "x": 1.0, "y": 2.0, "yaw": 0.5}
                    ],
                }
            )
            self.assertTrue(loaded["accepted"])
            self.assertEqual(loaded["map_id"], "duty-ab")
            info = service.map_info()
            self.assertEqual(info["map_id"], "duty-ab")
            self.assertEqual(info["stations"][0]["station_id"], "AGV_L")
            with open(os.path.join(tmp, "duty-ab.json"), encoding="utf-8") as handle:
                saved = json.load(handle)
            self.assertEqual(saved["stations"][0]["vendor_id"], "1")
            again = NavigationService(
                {"adapter": "fake", "maps_dir": tmp},
                adapter=FakeNavigationAdapter(),
            ).load_map({"map_id": "duty-ab"})
            self.assertTrue(again["accepted"])
            self.assertEqual(again["stations"][0]["station_id"], "AGV_L")
            occ = loaded["occupancy"]
            self.assertEqual(len(occ["data"]), occ["width"] * occ["height"])
            self.assertEqual(occ["data"][1], 100)

    def test_load_map_current_is_idempotent_and_returns_occupancy(self) -> None:
        adapter = FakeNavigationAdapter()
        calls = {"apply": 0}
        original = adapter.apply_map

        def counted(map_id: str, stations: list) -> dict:
            calls["apply"] += 1
            return original(map_id, stations)

        adapter.apply_map = counted  # type: ignore[method-assign]
        service = NavigationService({"adapter": "fake"}, adapter=adapter)
        current = service.map_info()["map_id"]
        loaded = service.load_map({"map_id": current})
        again = service.load_map({"map_id": current})
        self.assertTrue(loaded["accepted"])
        self.assertEqual(calls["apply"], 0)
        self.assertEqual(loaded["map_id"], current)
        occ = loaded["occupancy"]
        self.assertEqual(occ["width"] * occ["height"], len(occ["data"]))
        self.assertEqual(occ["origin"]["x"], 0.0)
        self.assertEqual(again["occupancy"]["data"], occ["data"])
        self.assertTrue(service.state()["map"]["occupancy_revision"])

    def test_load_map_save_current(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            service = NavigationService(
                {"adapter": "fake", "maps_dir": tmp},
                adapter=self.adapter,
            )
            loaded = service.load_map({"map_id": "snap-1", "save_current": True})
            self.assertTrue(loaded["accepted"])
            self.assertTrue(os.path.isfile(os.path.join(tmp, "snap-1.json")))
            self.assertEqual(loaded["stations"][0]["station_id"], "station.home")

    def test_state_includes_battery_and_chassis(self) -> None:
        self.adapter._battery = {
            "capacity_percent": 66.0,
            "temperature": 33.0,
            "voltage": 49.7,
            "charging": False,
            "cycle": 31,
        }
        self.adapter._chassis_status = {
            "state": "idle",
            "motion": {
                "stopped": True,
                "linear_x_mps": 0.0,
                "linear_y_mps": 0.0,
                "linear_speed_mps": 0.0,
                "angular_radps": 0.0,
            },
        }
        state = self.service.state()
        self.assertEqual(state["battery"]["capacity_percent"], 66.0)
        self.assertEqual(state["chassis_status"]["state"], "idle")


class TestTianjiAdapter(unittest.TestCase):
    def test_ready_reads_vendor_health(self) -> None:
        calls = []

        def fake_http(url, **kwargs):
            calls.append(url)
            return 200, {"status": "READY"}

        import navigation.adapters.tianji as tianji

        original = tianji._http_json
        tianji._http_json = fake_http
        try:
            adapter = TianjiNavigationAdapter(
                {"tianji": {"base_url": "http://127.0.0.1:8081", "stations_yaml": ""}}
            )
            self.assertTrue(adapter.ready())
            self.assertIn("/navigation/health", calls[0])
        finally:
            tianji._http_json = original

    def test_goto_maps_succeeded(self) -> None:
        def fake_http(url, **kwargs):
            if url.endswith("/navigate"):
                return 200, {"status": "SUCCEEDED"}
            return 200, {"status": "READY"}

        import navigation.adapters.tianji as tianji

        original = tianji._http_json
        tianji._http_json = fake_http
        try:
            adapter = TianjiNavigationAdapter({"tianji": {"stations_yaml": ""}})
            result = adapter.goto(station_id="start", idempotency_key="k", timeout_sec=5)
            self.assertEqual(result["terminal_state"], "SUCCEEDED")
            self.assertTrue(result["evidence"]["arrived"])
        finally:
            tianji._http_json = original


class TestNavigationHttpRoutes(unittest.TestCase):
    def test_unknown_path_is_404(self) -> None:
        from fastapi.testclient import TestClient

        from navigation.http_app import create_app

        client = TestClient(create_app(NavigationService({"adapter": "fake"})))
        missing = client.get("/nope")
        self.assertEqual(missing.status_code, 404)
        self.assertEqual(missing.json()["error_code"], "NOT_FOUND")
        health = client.get("/health")
        self.assertEqual(health.status_code, 200)
        self.assertTrue(health.json()["ok"])

    def test_goto_missing_fields_is_200_rejected(self) -> None:
        from fastapi.testclient import TestClient

        from navigation.http_app import create_app

        client = TestClient(create_app(NavigationService({"adapter": "fake"})))
        result = client.post("/goto", json={"station_id": "start"})
        self.assertEqual(result.status_code, 200)
        self.assertEqual(result.json()["error_code"], "NAVIGATION_INVALID_REQUEST")

    def test_import_on_fake_adapter_is_rejected(self) -> None:
        from fastapi.testclient import TestClient

        from navigation.http_app import create_app

        client = TestClient(create_app(NavigationService({"adapter": "fake"})))
        result = client.post("/maps/import", json={"map_name": "nav_try_01", "topology": {}})
        self.assertEqual(result.status_code, 200)
        self.assertFalse(result.json()["accepted"])
        self.assertEqual(result.json()["error_code"], "NAVIGATION_UNAVAILABLE")


class _ImportAdapter:
    def __init__(self) -> None:
        self.switched = None
        self.refreshed = False

    def snapshot(self):
        return {
            "map_id": "test_zhongmian",
            "position": {"x": -0.231, "y": 0.228, "yaw": -0.007},
        }

    def switch_map(self, map_name, x_m, y_m, yaw):
        self.switched = (map_name, x_m, y_m, yaw)
        return {"accepted": True, "map_name": map_name}

    def refresh_stations(self, pull_occupancy=True):
        self.refreshed = pull_occupancy
        return {"accepted": True}


class TestMapImportSwitch(unittest.TestCase):
    def _service(self, adapter):
        return NavigationService(
            {"adapter": "rokae", "rokae": {"matrix": {"base_url": "http://chassis"}}},
            adapter=adapter,
        )

    def test_switch_defaults_false(self) -> None:
        from unittest.mock import patch

        adapter = _ImportAdapter()
        service = self._service(adapter)
        with patch("navigation.service.import_fms", return_value=(200, {})):
            result = service.import_map(
                {"map_name": "nav_upload", "topology": {"meta": {"size.x": 2, "size.y": 2}}}
            )
        self.assertTrue(result["accepted"])
        self.assertFalse(result["switched"])
        self.assertIsNone(adapter.switched)

    def test_switch_true_uses_current_pose(self) -> None:
        from unittest.mock import patch

        adapter = _ImportAdapter()
        service = self._service(adapter)
        with patch("navigation.service.import_fms", return_value=(200, {})):
            result = service.import_map(
                {
                    "map_name": "nav_upload",
                    "topology": {"meta": {"size.x": 2, "size.y": 2}},
                    "switch": True,
                }
            )
        self.assertTrue(result["accepted"])
        self.assertTrue(result["switched"])
        self.assertEqual(adapter.switched, ("nav_upload", -0.231, 0.228, -0.007))
        self.assertTrue(adapter.refreshed)

    def test_sros_switch_uses_inner_srp_when_client_has_no_method(self) -> None:
        from navigation.rokae_runtime.sros import RokaeSrosClient

        class Inner:
            def __init__(self) -> None:
                self.called = None

            def switch_map(self, name, x, y, yaw, absolute_location=False, timeout=60):
                self.called = (name, x, y, yaw, absolute_location)

        class Client:
            def __init__(self) -> None:
                self._srp = Inner()

        client = Client()
        sros = RokaeSrosClient({"rokae": {"sros": {"host": "127.0.0.1"}}}, client=client)
        result = sros.switch_map("nav_switch", -0.231, 0.228, -0.007)
        self.assertTrue(result["accepted"])
        self.assertEqual(client._srp.called[0], "nav_switch")
        self.assertEqual(client._srp.called[1:3], (-231, 228))
        self.assertAlmostEqual(client._srp.called[3], -0.007)

    def test_switch_must_be_boolean(self) -> None:
        adapter = _ImportAdapter()
        result = self._service(adapter).import_map(
            {"map_name": "nav_upload", "topology": {"meta": {"size.x": 2, "size.y": 2}}, "switch": "maybe"}
        )
        self.assertFalse(result["accepted"])
        self.assertEqual(result["error_code"], "NAVIGATION_INVALID_REQUEST")
        self.assertIsNone(adapter.switched)


class TestMatrixImportPack(unittest.TestCase):
    def test_gzip_contains_json_and_commented_pgm(self) -> None:
        import gzip
        import io
        import json
        import tarfile

        from navigation.rokae_runtime.matrix_import import fms_gzip

        doc = {
            "meta": {
                "size.x": 2,
                "size.y": 2,
                "zero_offset.x": 1,
                "zero_offset.y": 1,
                "resolution": 2,
            },
            "data": {"station": [{"id": 1, "name": "NAV_TRY"}]},
        }
        packed = fms_gzip("nav_try_01", doc)
        tar = tarfile.open(fileobj=io.BytesIO(gzip.decompress(packed)), mode="r:")
        names = set(tar.getnames())
        self.assertEqual(names, {"sros/map/nav_try_01.json", "sros/map/nav_try_010.pgm"})
        pgm = tar.extractfile("sros/map/nav_try_010.pgm").read()
        self.assertTrue(pgm.startswith(b"P5\n#1\n#1\n#2\n2 2\n255\n"))
        self.assertEqual(len(pgm), len(b"P5\n#1\n#1\n#2\n2 2\n255\n") + 4)
        stored = json.loads(tar.extractfile("sros/map/nav_try_01.json").read())
        self.assertEqual(stored["data"]["station"][0]["name"], "NAV_TRY")


if __name__ == "__main__":
    unittest.main()
