import json
import os
import sys
import tempfile
import unittest
from types import SimpleNamespace
from typing import Any, Optional
from unittest import mock

PROJECT_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
if PROJECT_ROOT not in sys.path:
    sys.path.insert(0, PROJECT_ROOT)

from navigation.adapters.rokae import RokaeNavigationAdapter, resolve_station_id
from navigation.rokae_runtime import build_backend
from navigation.rokae_runtime.mock import MockRokaeBackend
from navigation.rokae_runtime.sros import RokaeSrosClient, movement_task_executing
from navigation.service import build_adapter


def _adapter(backend: Optional[MockRokaeBackend] = None) -> tuple[RokaeNavigationAdapter, MockRokaeBackend]:
    vendor = backend or MockRokaeBackend()
    return (
        RokaeNavigationAdapter(
            {"adapter": "rokae", "rokae": {"backend": "mock", "localized_wait_sec": 0}},
            backend=vendor,
        ),
        vendor,
    )


class TestResolveStation(unittest.TestCase):
    def setUp(self) -> None:
        self.stations = [
            {"id": "1", "name": "A", "aliases": ["A", "s1-A"]},
            {"id": "2", "name": "B", "aliases": ["B", "s2-B"]},
        ]

    def test_aliases(self) -> None:
        self.assertEqual(resolve_station_id("A", self.stations), 1)
        self.assertEqual(resolve_station_id("b", self.stations), 2)
        self.assertEqual(resolve_station_id("1", self.stations), 1)
        self.assertEqual(resolve_station_id("2", self.stations), 2)
        self.assertEqual(resolve_station_id("s1-A", self.stations), 1)
        self.assertEqual(resolve_station_id("s2-B", self.stations), 2)

    def test_unknown(self) -> None:
        self.assertIsNone(resolve_station_id("start", self.stations))
        self.assertIsNone(resolve_station_id("3", self.stations))
        self.assertIsNone(resolve_station_id("", self.stations))


class TestRokaeAdapter(unittest.TestCase):
    def test_ready_requires_location_state_running(self) -> None:
        adapter, vendor = _adapter()
        self.assertTrue(adapter.ready())
        vendor.state["location_state"] = 1
        self.assertFalse(adapter.ready())
        vendor.state = None
        self.assertFalse(adapter.ready())

    def test_snapshot_includes_map_battery_chassis(self) -> None:
        adapter, _vendor = _adapter()
        snap = adapter.snapshot()
        self.assertTrue(snap["ready"])
        self.assertEqual(snap["map_id"], "AB_0619")
        self.assertEqual(snap["position"]["x"], 0.2887)
        self.assertEqual(snap["battery"]["capacity_percent"], 80.0)
        self.assertEqual(snap["chassis_status"]["state"], "idle")
        names = {row["station_id"] for row in snap["stations"]}
        self.assertEqual(names, {"A", "B"})

    def test_sros_sim_state_includes_chassis_osd_blocks(self) -> None:
        from navigation.service import NavigationService

        adapter = RokaeNavigationAdapter(
            {
                "rokae": {
                    "backend": "sros",
                    "localized_wait_sec": 0,
                    "sros": {"sim": True, "travel_sec": 0, "poll_sec": 0},
                }
            }
        )
        service = NavigationService({"adapter": "rokae"}, adapter=adapter)
        state = service.state()
        self.assertEqual(state["battery"]["source"], "sros_amr")
        self.assertEqual(state["battery"]["cycle"], 12)
        self.assertEqual(state["robot_mode"]["location_state"], "running")
        self.assertEqual(state["robot_mode"]["sys_state"], "idle")
        self.assertTrue(state["odometry"]["pose_confirmed"])
        self.assertFalse(state["safety"]["emergency_stop"])
        self.assertEqual(state["devices_info"]["imu"]["model"], "CH040-SR")
        self.assertEqual(state["devices_info"]["camera"]["model"], "D435")
        self.assertEqual(state["devices_info"]["battery"]["model"], "BAT-48V")
        self.assertEqual(state["motors"]["controller"], "src")
        self.assertEqual(state["mainboard"]["vehicle_type"], "Oasis-sim")
        self.assertTrue(state["stm32_status"]["is_connected"])
        self.assertEqual(state["collector"]["sources"]["chassis"]["connected"], True)
        self.assertNotIn("left_arm", state["collector"]["sources"])
        self.assertEqual(state["transport"]["endpoints"]["chassis"]["protocol"], "srp")
        adapter.close()

    def test_service_goto_b_reaches_vendor(self) -> None:
        import time

        from navigation.service import NavigationService

        adapter, vendor = _adapter()
        service = NavigationService({"adapter": "rokae"}, adapter=adapter)
        result = service.goto(
            {
                "task_id": "TASK-1",
                "request_id": "REQ-B",
                "timeout_sec": 8,
                "idempotency_key": "KEY-B",
                "station_id": "B",
            }
        )
        self.assertTrue(result["accepted"])
        deadline = time.monotonic() + 1.0
        status: dict = {}
        while time.monotonic() < deadline:
            status = service.status_of("REQ-B")
            if status.get("terminal_state") == "SUCCEEDED":
                break
            time.sleep(0.01)
        self.assertEqual(status["terminal_state"], "SUCCEEDED")
        self.assertEqual(vendor.calls, [2])

    def test_goto_sends_vendor_station_id(self) -> None:
        adapter, vendor = _adapter()
        result = adapter.goto(station_id="B", idempotency_key="k", timeout_sec=5)
        self.assertEqual(result["terminal_state"], "SUCCEEDED")
        self.assertTrue(result["evidence"]["arrived"])
        self.assertEqual(vendor.calls, [2])

        result_a = adapter.goto(station_id="1", idempotency_key="k2", timeout_sec=5)
        self.assertEqual(result_a["terminal_state"], "SUCCEEDED")
        self.assertEqual(vendor.calls[-1], 1)

    def test_unknown_station_rejected(self) -> None:
        adapter, vendor = _adapter()
        result = adapter.goto(station_id="start", idempotency_key="k", timeout_sec=5)
        self.assertEqual(result["terminal_state"], "REJECTED")
        self.assertEqual(result["error_code"], "NAVIGATION_UNKNOWN_STATION")
        self.assertEqual(vendor.calls, [])

    def test_service_rejects_unknown_station_before_vendor(self) -> None:
        from navigation.service import NavigationService

        adapter, vendor = _adapter()
        service = NavigationService({"adapter": "rokae"}, adapter=adapter)
        result = service.goto(
            {
                "task_id": "TASK-1",
                "request_id": "REQ-NOPE",
                "timeout_sec": 8,
                "idempotency_key": "KEY-NOPE",
                "station_id": "NOPE",
            }
        )
        self.assertFalse(result["accepted"])
        self.assertEqual(result["error_code"], "NAVIGATION_UNKNOWN_STATION")
        self.assertEqual(vendor.calls, [])

    def test_not_localized_rejected(self) -> None:
        adapter, vendor = _adapter()
        vendor.state["location_state"] = 1
        result = adapter.goto(station_id="A", idempotency_key="k", timeout_sec=5)
        self.assertEqual(result["terminal_state"], "REJECTED")
        self.assertEqual(result["error_code"], "NAVIGATION_NOT_READY")
        self.assertEqual(vendor.calls, [])

    def test_vendor_unavailable(self) -> None:
        adapter, vendor = _adapter()
        vendor.server_ok = False
        result = adapter.goto(station_id="A", idempotency_key="k", timeout_sec=5)
        self.assertEqual(result["terminal_state"], "FAILED")
        self.assertEqual(result["error_code"], "NAVIGATION_VENDOR_UNREACHABLE")


    def test_stop_unavailable_unified_map_applies(self) -> None:
        adapter, _vendor = _adapter()
        stop = adapter.stop()
        self.assertFalse(stop["accepted"])
        self.assertEqual(stop["error_code"], "NAVIGATION_UNAVAILABLE")
        loaded = adapter.apply_map(
            "AB_0619",
            [{"station_id": "A", "vendor_id": "1", "x": 0.1, "y": 0.2, "yaw": 0.0}],
        )
        self.assertTrue(loaded["accepted"])
        self.assertEqual(adapter.snapshot()["map_id"], "AB_0619")
        self.assertEqual(adapter.snapshot()["stations"][0]["station_id"], "A")

    def test_build_adapter_mock_and_keep_tianji(self) -> None:
        from navigation.adapters.fake import FakeNavigationAdapter
        from navigation.adapters.tianji import TianjiNavigationAdapter

        adapter = build_adapter({"adapter": "rokae", "rokae": {"backend": "mock"}})
        self.assertIsInstance(adapter, RokaeNavigationAdapter)
        self.assertTrue(adapter.ready())
        adapter.close()
        self.assertIsInstance(
            build_adapter({"adapter": "tianji", "tianji": {"stations_yaml": ""}}),
            TianjiNavigationAdapter,
        )
        self.assertIsInstance(build_adapter({"adapter": "fake"}), FakeNavigationAdapter)
        self.assertIsInstance(build_backend({"rokae": {"backend": "mock"}}), MockRokaeBackend)


class TestRokaeSrosBackend(unittest.TestCase):
    def test_goto_calls_sdk_move_to_station(self) -> None:
        stub = _FakeSrp()
        vendor = RokaeSrosClient({"rokae": {"sros": {"poll_sec": 0}}}, client=stub)
        adapter = RokaeNavigationAdapter(
            {"rokae": {"backend": "sros", "localized_wait_sec": 0}},
            backend=vendor,
        )
        result = adapter.goto(station_id="B", idempotency_key="k", timeout_sec=2)
        self.assertEqual(result["terminal_state"], "SUCCEEDED")
        self.assertEqual(stub.calls, [(1, 2)])
        self.assertAlmostEqual(adapter.snapshot()["position"]["x"], 0.2887, places=4)

    def test_sros_sim_goto_without_tcp(self) -> None:
        adapter = RokaeNavigationAdapter(
            {
                "rokae": {
                    "backend": "sros",
                    "localized_wait_sec": 0,
                    "sros": {"sim": True, "travel_sec": 0, "poll_sec": 0},
                }
            }
        )
        self.assertTrue(adapter.ready())
        result = adapter.goto(station_id="B", idempotency_key="k", timeout_sec=2)
        self.assertEqual(result["terminal_state"], "SUCCEEDED")
        self.assertAlmostEqual(adapter.snapshot()["position"]["x"], 0.5010, places=3)
        adapter.close()

    def test_stop_uses_sdk_cancel(self) -> None:
        stub = _FakeSrp()
        vendor = RokaeSrosClient({"rokae": {}}, client=stub)
        adapter = RokaeNavigationAdapter({"rokae": {"backend": "sros"}}, backend=vendor)
        stop = adapter.stop()
        self.assertTrue(stop["accepted"])
        self.assertTrue(stub.cancelled)

    def test_idle_mt_na_is_not_executing(self) -> None:
        from navigation.service import NavigationService

        stub = _FakeSrp()
        stub._system.movement_state = SimpleNamespace(
            no=0, state=0, result="", failed_code="", finished=False
        )
        vendor = RokaeSrosClient({"rokae": {"sros": {"poll_sec": 0}}}, client=stub)
        adapter = RokaeNavigationAdapter(
            {"rokae": {"backend": "sros", "localized_wait_sec": 0}},
            backend=vendor,
        )
        service = NavigationService({"adapter": "rokae"}, adapter=adapter)
        state = vendor.latest_state() or {}
        body = service.state()
        self.assertFalse(state["executing_movement_task"])
        self.assertEqual(body["chassis_status"]["state"], "idle")
        self.assertTrue(body["chassis_status"]["motion"]["stopped"])
        self.assertEqual(body["navigation_status"]["state"], "IDLE")

    def test_running_movement_is_executing(self) -> None:
        stub = _FakeSrp()
        stub._system.movement_state = SimpleNamespace(
            no=3, state=3, result="", failed_code="", finished=False
        )
        vendor = RokaeSrosClient({"rokae": {"sros": {"poll_sec": 0}}}, client=stub)
        state = vendor.latest_state() or {}
        self.assertTrue(state["executing_movement_task"])
        snap = RokaeNavigationAdapter(
            {"rokae": {"backend": "sros", "localized_wait_sec": 0}},
            backend=vendor,
        ).snapshot()
        self.assertEqual(snap["nav_state"], "NAVIGATING")
        self.assertEqual(snap["chassis_status"]["state"], "moving")


class TestMatrixStations(unittest.TestCase):
    def test_parse_topology_and_keep_ab_aliases(self) -> None:
        from navigation.rokae_runtime.matrix_stations import stations_from_topology

        fixture = os.path.join(PROJECT_ROOT, "tests", "fixtures", "matrix_hjl.json")
        with open(fixture, encoding="utf-8") as handle:
            doc = json.load(handle)
        rows = stations_from_topology(doc)
        self.assertEqual([row["station_id"] for row in rows], ["站点1", "站点2", "站点3"])
        adapter, vendor = _adapter()
        with tempfile.TemporaryDirectory() as tmp:
            adapter._config["maps_dir"] = tmp
            refreshed = adapter.refresh_stations(topology=doc)
            self.assertTrue(refreshed["accepted"])
            self.assertEqual(refreshed["map_id"], "AB_0619")
            self.assertTrue(os.path.isfile(os.path.join(tmp, "AB_0619.json")))
        ok = adapter.goto(station_id="A", idempotency_key="k-a", timeout_sec=5)
        self.assertEqual(ok["terminal_state"], "SUCCEEDED")
        self.assertEqual(vendor.calls[-1], 1)
        ok_cn = adapter.goto(station_id="站点2", idempotency_key="k-cn", timeout_sec=5)
        self.assertEqual(ok_cn["terminal_state"], "SUCCEEDED")
        self.assertEqual(vendor.calls[-1], 2)

    def test_edges_follow_stations_in_meters(self) -> None:
        from navigation.rokae_runtime.matrix_stations import edges_from_topology, nodes_from_topology
        from navigation.service import NavigationService

        doc = {
            "meta": {"length_unit": "mm"},
            "data": {
                "node": [{"id": 1, "x": -235, "y": 239, "yaw": 0, "desc": ""}],
                "station": [
                    {"id": 1, "name": "s1-A", "pos.x": 0, "pos.y": 0, "pos.yaw": 0}
                ],
                "edge": [
                    {
                        "id": 1,
                        "type": 1,
                        "s_node": 1,
                        "e_node": 2,
                        "sx": -235,
                        "sy": 239,
                        "ex": -236,
                        "ey": 655,
                        "cost": 416,
                        "s_facing": 2.404,
                        "e_facing": 3177.5,
                        "robot_direction": 4,
                        "rotate_direction": 1,
                    }
                ],
            },
        }
        nodes = nodes_from_topology(doc)
        edges = edges_from_topology(doc)
        self.assertAlmostEqual(nodes[0]["x"], -0.235)
        self.assertAlmostEqual(edges[0]["sx"], -0.235)
        self.assertAlmostEqual(edges[0]["cost"], 0.416)
        self.assertAlmostEqual(edges[0]["s_facing"], 2.404)
        self.assertAlmostEqual(edges[0]["e_facing"], 3.1775)
        self.assertEqual(edges[0]["robot_direction"], 4)
        adapter, _vendor = _adapter()
        with tempfile.TemporaryDirectory() as tmp:
            adapter._config["maps_dir"] = tmp
            self.assertTrue(adapter.refresh_stations(topology=doc, pull_occupancy=False)["accepted"])
        snap = adapter.snapshot()
        self.assertEqual(snap["edges"][0]["id"], 1)
        service = NavigationService({"adapter": "rokae"}, adapter=adapter)
        state = service.state()
        self.assertEqual(state["map"]["edges"][0]["e_node"], 2)
        self.assertEqual(service.map_info()["nodes"][0]["id"], 1)

    def test_goto_validates_without_refetch(self) -> None:
        adapter, vendor = _adapter()
        fixture = os.path.join(PROJECT_ROOT, "tests", "fixtures", "matrix_topology.json")
        with open(fixture, encoding="utf-8") as handle:
            doc = json.load(handle)
        with tempfile.TemporaryDirectory() as tmp:
            adapter._config["maps_dir"] = tmp
            self.assertTrue(adapter.refresh_stations(topology=doc)["accepted"])
        with mock.patch("navigation.rokae_runtime.matrix_stations.fetch_topology") as fetch:
            ok = adapter.goto(station_id="C", idempotency_key="k-c", timeout_sec=5)
            bad = adapter.goto(station_id="NOPE", idempotency_key="k-nope", timeout_sec=5)
            fetch.assert_not_called()
        self.assertEqual(ok["terminal_state"], "SUCCEEDED")
        self.assertEqual(vendor.calls[-1], 3)
        self.assertEqual(bad["terminal_state"], "REJECTED")
        self.assertEqual(bad["error_code"], "NAVIGATION_UNKNOWN_STATION")

    def test_startup_pulls_once_from_topology_file(self) -> None:
        fixture = os.path.join("tests", "fixtures", "matrix_topology.json")
        stub = _FakeSrp()
        vendor = RokaeSrosClient({"rokae": {"sros": {"poll_sec": 0}}}, client=stub)
        with tempfile.TemporaryDirectory() as tmp:
            adapter = RokaeNavigationAdapter(
                {
                    "maps_dir": tmp,
                    "rokae": {
                        "backend": "sros",
                        "localized_wait_sec": 0,
                        "matrix": {"topology_path": fixture},
                    },
                },
                backend=vendor,
            )
            names = {row["station_id"] for row in adapter.snapshot()["stations"]}
            self.assertEqual(names, {"A", "B", "C"})
            self.assertTrue(os.path.isfile(os.path.join(tmp, "AB_0619.json")))
        adapter.close()

    def test_map_catalog_revision_changes_with_md5(self) -> None:
        from navigation.rokae_runtime.matrix_stations import (
            catalog_revision,
            map_file_revision,
            maps_from_catalog,
        )

        doc = {
            "maps": [
                {
                    "name": "test_zhongmian",
                    "md5": "8ded2fbd6144513da6a553487cf558cd",
                    "modify_time": "2026-09-17 18:38:37",
                    "map_version": "1.13.0",
                }
            ]
        }
        maps = maps_from_catalog(doc)
        first = catalog_revision(maps, map_name="test_zhongmian")
        self.assertTrue(first)
        self.assertEqual(first, map_file_revision(maps[0], map_name="test_zhongmian"))
        maps[0]["md5"] = "ffffffffffffffffffffffffffffffff"
        self.assertNotEqual(first, catalog_revision(maps, map_name="test_zhongmian"))

    def test_catalog_unchanged_skips_station_export(self) -> None:
        fixture = os.path.join(PROJECT_ROOT, "tests", "fixtures", "matrix_topology.json")
        stub = _FakeSrp()
        vendor = RokaeSrosClient({"rokae": {"sros": {"poll_sec": 0}}}, client=stub)
        with tempfile.TemporaryDirectory() as tmp:
            adapter = RokaeNavigationAdapter(
                {
                    "maps_dir": tmp,
                    "rokae": {
                        "backend": "sros",
                        "localized_wait_sec": 0,
                        "matrix": {"topology_path": fixture, "poll_sec": 10},
                    },
                },
                backend=vendor,
            )
            adapter._map_file_revision = "keep"
            with mock.patch.object(adapter, "_peek_map_revision", return_value="keep"):
                with mock.patch.object(adapter, "refresh_stations") as refresh:
                    adapter._sync_matrix_worker("AB_0619", False)
                    refresh.assert_not_called()
        adapter.close()

    def test_catalog_md5_change_refreshes_stations_without_occupancy(self) -> None:
        fixture = os.path.join(PROJECT_ROOT, "tests", "fixtures", "matrix_topology.json")
        stub = _FakeSrp()
        vendor = RokaeSrosClient({"rokae": {"sros": {"poll_sec": 0}}}, client=stub)
        with tempfile.TemporaryDirectory() as tmp:
            adapter = RokaeNavigationAdapter(
                {
                    "maps_dir": tmp,
                    "rokae": {
                        "backend": "sros",
                        "localized_wait_sec": 0,
                        "matrix": {"topology_path": fixture, "poll_sec": 10},
                    },
                },
                backend=vendor,
            )
            adapter._map_file_revision = "old"
            with mock.patch.object(adapter, "_peek_map_revision", return_value="new-md5"):
                with mock.patch(
                    "navigation.rokae_runtime.matrix_occupancy.occupancy_from_rokae_config"
                ) as occupancy:
                    adapter._sync_matrix_worker("AB_0619", False)
                    occupancy.assert_not_called()
            self.assertEqual(adapter._map_file_revision, "new-md5")
        adapter.close()

    def test_topology_fingerprint_changes_when_station_moves(self) -> None:
        from navigation.rokae_runtime.matrix_stations import topology_fingerprint

        fixture = os.path.join(PROJECT_ROOT, "tests", "fixtures", "matrix_topology.json")
        with open(fixture, encoding="utf-8") as handle:
            doc = json.load(handle)
        first = topology_fingerprint(doc, map_name="AB_0619")
        self.assertEqual(first, topology_fingerprint(doc, map_name="AB_0619"))
        self.assertNotEqual(first, topology_fingerprint(doc, map_name="other"))
        doc["data"]["station"][0]["pos.x"] = 999
        self.assertNotEqual(first, topology_fingerprint(doc, map_name="AB_0619"))
        doc["meta"]["modified_timestamp"] = "2026-09-18T00:00:00"
        self.assertNotEqual(
            topology_fingerprint(doc, map_name="AB_0619"),
            first,
        )

    def test_matrix_poll_refreshes_moved_station(self) -> None:
        fixture = os.path.join(PROJECT_ROOT, "tests", "fixtures", "matrix_topology.json")
        stub = _FakeSrp()
        vendor = RokaeSrosClient({"rokae": {"sros": {"poll_sec": 0}}}, client=stub)
        with tempfile.TemporaryDirectory() as tmp:
            adapter = RokaeNavigationAdapter(
                {
                    "maps_dir": tmp,
                    "rokae": {
                        "backend": "sros",
                        "localized_wait_sec": 0,
                        "matrix": {"topology_path": fixture, "poll_sec": 10},
                    },
                },
                backend=vendor,
            )
            before = {row["station_id"]: row["x"] for row in adapter.snapshot()["stations"]}
            self.assertAlmostEqual(before["A"], 0.2887, places=3)
            with mock.patch.object(adapter, "refresh_stations") as refresh:
                adapter._sync_matrix_worker("AB_0619", False)
                refresh.assert_not_called()
            with open(fixture, encoding="utf-8") as handle:
                doc = json.load(handle)
            doc["data"]["station"][0]["pos.x"] = 999
            with mock.patch(
                "navigation.rokae_runtime.matrix_stations.topology_from_rokae_config",
                return_value=doc,
            ):
                adapter._sync_matrix_worker("AB_0619", False)
            after = {row["station_id"]: row["x"] for row in adapter.snapshot()["stations"]}
            self.assertAlmostEqual(after["A"], 0.999, places=3)
        adapter.close()

    def test_mock_startup_does_not_hit_matrix(self) -> None:
        with mock.patch("navigation.rokae_runtime.matrix_stations.fetch_topology") as fetch:
            adapter, _vendor = _adapter()
            fetch.assert_not_called()
        names = {row["station_id"] for row in adapter.snapshot()["stations"]}
        self.assertEqual(names, {"A", "B"})

    def test_load_map_current_returns_local_occupancy(self) -> None:
        from navigation.occupancy import encode_pgm_gray
        from navigation.service import NavigationService

        adapter, _vendor = _adapter()
        with tempfile.TemporaryDirectory() as tmp:
            pgm = os.path.join(tmp, "map.pgm")
            meta = os.path.join(tmp, "map.json")
            with open(meta, "w", encoding="utf-8") as handle:
                json.dump(
                    {
                        "meta": {
                            "length_unit": "m",
                            "resolution": 0.05,
                            "size.x": 2,
                            "size.y": 1,
                            "zero_offset.x": 0,
                            "zero_offset.y": 0,
                        }
                    },
                    handle,
                )
            with open(pgm, "wb") as handle:
                handle.write(encode_pgm_gray(2, 1, [255, 0]))
            adapter._config.setdefault("rokae", {})["matrix"] = {"occupancy_path": pgm}
            service = NavigationService({"adapter": "rokae", "maps_dir": tmp}, adapter=adapter)
            loaded = service.load_map({"map_id": adapter.snapshot()["map_id"]})
        self.assertTrue(loaded["accepted"])
        self.assertEqual(loaded["occupancy"]["data"], [0, 100])

    def test_refresh_stations_saves_occupancy_and_revision(self) -> None:
        from navigation.occupancy import encode_pgm_gray
        from navigation.service import NavigationService

        adapter, _vendor = _adapter()
        fixture = os.path.join(PROJECT_ROOT, "tests", "fixtures", "matrix_topology.json")
        with open(fixture, encoding="utf-8") as handle:
            doc = json.load(handle)
        with tempfile.TemporaryDirectory() as tmp:
            pgm = os.path.join(tmp, "map.pgm")
            meta = os.path.join(tmp, "map.json")
            with open(meta, "w", encoding="utf-8") as handle:
                json.dump(
                    {
                        "meta": {
                            "length_unit": "m",
                            "resolution": 0.05,
                            "size.x": 2,
                            "size.y": 1,
                            "zero_offset.x": 0,
                            "zero_offset.y": 0,
                        }
                    },
                    handle,
                )
            with open(pgm, "wb") as handle:
                handle.write(encode_pgm_gray(2, 1, [255, 0]))
            adapter._config["maps_dir"] = tmp
            adapter._config.setdefault("rokae", {})["matrix"] = {"occupancy_path": pgm}
            refreshed = adapter.refresh_stations(topology=doc)
            self.assertTrue(refreshed["accepted"])
            self.assertTrue(os.path.isfile(os.path.join(tmp, f"{refreshed['map_id']}.occupancy.json")))
            service = NavigationService({"adapter": "rokae", "maps_dir": tmp}, adapter=adapter)
            state = service.state()
            self.assertEqual(state["map"]["map_id"], refreshed["map_id"])
            self.assertTrue(state["map"]["occupancy_revision"])
            loaded = service.load_map({"map_id": refreshed["map_id"]})
            self.assertEqual(loaded["occupancy"]["data"], [0, 100])

    def test_service_refresh_stations(self) -> None:
        from navigation.service import NavigationService

        adapter, _vendor = _adapter()
        service = NavigationService({"adapter": "rokae"}, adapter=adapter)
        fixture = os.path.join(PROJECT_ROOT, "tests", "fixtures", "matrix_topology.json")
        with open(fixture, encoding="utf-8") as handle:
            doc = json.load(handle)
        with tempfile.TemporaryDirectory() as tmp:
            adapter._config["maps_dir"] = tmp
            adapter.refresh_stations(topology=doc)
        info = service.map_info()
        self.assertIn("C", {row["station_id"] for row in info["stations"]})


class _FakeSrp:
    def __init__(self) -> None:
        self.calls: list[tuple[int, int]] = []
        self.cancelled = False
        self._system = SimpleNamespace(
            location_state=3,
            location_pose=SimpleNamespace(x=288.7, y=-848.7, yaw=0),
            emergency_state="NONE",
            map_name="AB_0619",
            station_no=1,
            mc_state=SimpleNamespace(v_x=0, v_y=0, w=0),
            movement_state=SimpleNamespace(no=0, state="RUNNING", result="", failed_code=""),
        )

    def fetch_system_state(self) -> None:
        return None

    def fetch_hardware_state(self) -> None:
        return None

    def get_current_system_state(self) -> Any:
        return self._system

    def get_current_hardware_state(self) -> Any:
        return SimpleNamespace(
            battery_percentage=80,
            battery_temperature=31,
            battery_voltage=50400,
            battery_state=0,
            battery_use_cycles=12,
        )

    def move_to_station(self, no: int, station_id: int) -> None:
        self.calls.append((int(no), int(station_id)))
        self._system.movement_state = SimpleNamespace(
            no=int(no),
            state="FINISHED",
            result="OK",
            failed_code="",
            finished=True,
            ok=True,
        )
        self._system.station_no = int(station_id)

    def cancel_movement_task(self, _soft: bool = True) -> None:
        self.cancelled = True

    def disconnect(self) -> None:
        return None


class TestMovementTaskExecuting(unittest.TestCase):
    def test_mt_na_and_finished_are_idle(self) -> None:
        self.assertFalse(movement_task_executing(None))
        self.assertFalse(
            movement_task_executing(SimpleNamespace(state=0, finished=False))
        )
        self.assertFalse(
            movement_task_executing(SimpleNamespace(state="MT_NA", finished=False))
        )
        self.assertFalse(
            movement_task_executing(SimpleNamespace(state="FINISHED", finished=False))
        )
        self.assertFalse(
            movement_task_executing(SimpleNamespace(state=3, finished=True))
        )

    def test_running_family_is_executing(self) -> None:
        self.assertTrue(
            movement_task_executing(SimpleNamespace(state=3, finished=False))
        )
        self.assertTrue(
            movement_task_executing(SimpleNamespace(state="MT_RUNNING", finished=False))
        )
        self.assertTrue(
            movement_task_executing(SimpleNamespace(state="PAUSED", finished=False))
        )
        self.assertTrue(
            movement_task_executing(SimpleNamespace(state=6, finished=False))
        )


if __name__ == "__main__":
    unittest.main()
