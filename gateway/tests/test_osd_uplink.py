import os
import sys
import unittest
from typing import Any, Dict, Optional


PROJECT_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
if PROJECT_ROOT not in sys.path:
    sys.path.insert(0, PROJECT_ROOT)

from gateway.app import GatewayApp
from gateway.mqtt.osd import OsdReporter, StateCollector
from gateway.mqtt.trajectory import TrajectoryReporter
from gateway.mqtt.uplink import UplinkPublisher
from gateway.records import TaskStateStore
from gateway.state_sources import adapt_state_fragment, iter_state_sources, StateSource


class _EmptyHost:
    def get_status(self):
        return {}


class FakeCollector:
    def __init__(self, snapshot: Dict[str, Any]):
        self._snapshot = snapshot

    def snapshot(self) -> Dict[str, Any]:
        return dict(self._snapshot)


class FakeMqtt:
    def __init__(self):
        self.device_sn = "ROBOT_006"
        self.topic_osd = "thing/product/robot/ROBOT_006/osd"
        self.topic_trajectory = "thing/product/robot/ROBOT_006/trajectory"
        self.is_connected = True
        self.osd = []
        self.trajectory = []
        self.events = []

    def publish_osd(self, payload):
        self.osd.append(payload)
        return True

    def publish_trajectory(self, payload):
        self.trajectory.append(payload)
        return True

    def publish_event(self, event, data):
        self.events.append((event, data))
        return True


CONFIG = {
    "device": {"sn": "ROBOT_006", "deviceType": "term", "alias": "gateway"},
    "mqtt": {"osd_report_interval": 1},
    "trajectory": {
        "enabled": True,
        "map_frame": "map",
        "robot_frame": "base_link",
    },
}


class TestOsdAndTrajectory(unittest.TestCase):
    def test_osd_envelope_uses_capability_fragments_only(self):
        collector = FakeCollector(
            {
                "position": {"x": 0.17, "y": 0.31, "yaw": 0.63},
                "map": {
                    "map_id": "3b241101-e2bb-4f3a-b20a-eaf62a0ee8d5",
                    "stations": [{"station_id": "LM1", "x": 0.0, "y": 0.0, "yaw": 0.0}],
                    "nodes": [{"id": 1, "x": 0.0, "y": 0.0}],
                    "edges": [{"id": 1, "s_node": 1, "e_node": 2}],
                },
                "task": {
                    "task_id": "TASK-1",
                    "status": 1,
                },
                "self_check": {"status": 0, "message": "就绪"},
            }
        )
        reporter = OsdReporter(
            config=CONFIG,
            mqtt_client=FakeMqtt(),
            collector=collector,
            host_reader=_EmptyHost(),
        )
        payload = reporter.collect_osd_data()
        self.assertEqual(
            set(payload), {"tid", "timestamp", "deviceType", "data"}
        )
        self.assertEqual(payload["deviceType"], "term")
        self.assertEqual(payload["data"]["device_sn"], "ROBOT_006")
        self.assertEqual(payload["data"]["position"]["x"], 0.17)
        self.assertEqual(
            payload["data"]["map"]["map_id"],
            "3b241101-e2bb-4f3a-b20a-eaf62a0ee8d5",
        )
        self.assertEqual(payload["data"]["task"]["status"], 1)
        self.assertEqual(payload["data"]["task"]["current_tray_index"], 0)
        self.assertEqual(payload["data"]["task"]["has_tray_in_hand"], 0)
        self.assertEqual(payload["data"]["map"]["stations"][0]["name"], "LM1")
        self.assertEqual(payload["data"]["map"]["stations"][0]["x"], 0.0)
        self.assertEqual(payload["data"]["map"]["stations"][0]["y"], 0.0)
        self.assertEqual(payload["data"]["map"]["nodes"][0]["id"], 1)
        self.assertEqual(payload["data"]["map"]["edges"][0]["e_node"], 2)
        self.assertEqual(payload["data"]["self_check"]["status"], 0)

    def test_osd_omits_invalid_position_and_does_not_invent_map(self):
        reporter = OsdReporter(
            config=CONFIG,
            mqtt_client=FakeMqtt(),
            collector=FakeCollector({"position": {"x": "bad"}}),
            host_reader=_EmptyHost(),
        )
        data = reporter.collect_osd_data()["data"]
        self.assertNotIn("position", data)
        self.assertNotIn("map", data)
        self.assertEqual(data["task"]["status"], 0)
        self.assertEqual(data["task"]["current_tray_index"], 0)
        self.assertEqual(data["task"]["has_tray_in_hand"], 0)
        self.assertNotIn("host_status", data)
        self.assertNotIn("robot_mode", data)
        self.assertNotIn("arm_action", data)
        self.assertNotIn("collector", data)
        self.assertEqual(data["transport"]["mqtt_connected"], True)

    def test_osd_includes_smt_host_battery_chassis(self):
        class _Host:
            def get_status(self):
                return {"temperature_c": 47.5, "runtime_sec": 12}

        reporter = OsdReporter(
            config={
                **CONFIG,
                "device": {
                    **CONFIG["device"],
                    "robotType": "wheel_arm",
                },
            },
            mqtt_client=FakeMqtt(),
            collector=FakeCollector(
                {
                    "battery": {
                        "capacity_percent": 66.0,
                        "voltage": 49.7,
                        "temperature": 33.0,
                        "charging": False,
                        "cycle": 31,
                    },
                    "chassis_status": {
                        "state": "idle",
                        "drive_mode": {"has_task": False},
                        "motion": {
                            "stopped": True,
                            "linear_x_mps": 0.0,
                            "linear_y_mps": 0.0,
                            "linear_speed_mps": 0.0,
                            "angular_radps": 0.0,
                        },
                    },
                    "self_check": {"status": 0, "message": "就绪"},
                }
            ),
            host_reader=_Host(),
        )
        payload = reporter.collect_osd_data()
        data = payload["data"]
        self.assertEqual(payload["robotType"], "wheel_arm")
        self.assertEqual(data["host_status"]["temperature_c"], 47.5)
        self.assertEqual(data["host_status"]["runtime_sec"], 12)
        self.assertEqual(data["battery"]["capacity_percent"], 66.0)
        self.assertFalse(data["battery"]["charging"])
        self.assertEqual(data["chassis_status"]["state"], "idle")
        self.assertEqual(data["chassis_status"]["motion"]["linear_speed_mps"], 0.0)
        self.assertNotIn("drive_mode", data["chassis_status"])

    def test_osd_passes_through_manipulator_status(self):
        reporter = OsdReporter(
            config=CONFIG,
            mqtt_client=FakeMqtt(),
            collector=FakeCollector(
                {
                    "manipulator_status": {
                        "left_arm": {
                            "state": "idle",
                            "joint_positions_deg": [0.0, -10.0, 20.0],
                        }
                    }
                }
            ),
            host_reader=_EmptyHost(),
        )
        arms = reporter.collect_osd_data()["data"]["manipulator_status"]
        self.assertEqual(arms["left_arm"]["joint_positions_deg"][1], -10.0)
        self.assertNotIn("groups", arms)

    def test_osd_passes_through_optional_state_blocks(self):
        reporter = OsdReporter(
            config=CONFIG,
            mqtt_client=FakeMqtt(),
            collector=FakeCollector(
                {
                    "battery": {
                        "capacity_percent": 80.0,
                        "voltage": 54.2,
                        "charging": False,
                        "source": "sros_amr",
                        "pack_voltage_raw": 54200,
                    },
                    "chassis_status": {
                        "state": "PAUSED",
                        "motion": {
                            "stopped": True,
                            "linear_x_mps": 0.3,
                            "linear_y_mps": 0.4,
                            "angular_radps": 0.0,
                        },
                    },
                    "map": {"map_id": "NO_MAP"},
                    "robot_mode": {
                        "source": "sros_amr",
                        "location_state": "running",
                        "map_name": "NO_MAP",
                    },
                    "arm_action": {
                        "source": "rokae_xcore",
                        "endpoints": {
                            "body": {
                                "connected": True,
                                "joint_positions_deg": [0.0, 1.0, 2.0, 3.0, 4.0, 5.0],
                            }
                        },
                    },
                    "collector": {"name": "rokae_composite_state", "schema_version": "1.0.0"},
                    "alarm_status": {"alarms": []},
                    "self_check": {"status": 0, "message": "就绪"},
                }
            ),
            host_reader=_EmptyHost(),
        )
        data = reporter.collect_osd_data()["data"]
        self.assertEqual(data["battery"]["capacity_percent"], 80.0)
        self.assertEqual(data["battery"]["source"], "sros_amr")
        self.assertEqual(data["battery"]["pack_voltage_raw"], 54200)
        self.assertEqual(data["chassis_status"]["state"], "paused")
        self.assertAlmostEqual(data["chassis_status"]["motion"]["linear_speed_mps"], 0.5)
        self.assertNotIn("map", data)
        self.assertEqual(data["robot_mode"]["map_name"], "NO_MAP")
        self.assertEqual(
            data["arm_action"]["endpoints"]["body"]["joint_positions_deg"][-1], 5.0
        )
        self.assertEqual(data["collector"]["name"], "rokae_composite_state")
        self.assertNotIn("alarm_status", data)
        self.assertNotIn("motor_status", data)

    def test_trajectory_skips_invalid_pose(self):
        mqtt = FakeMqtt()
        reporter = TrajectoryReporter(
            config=CONFIG,
            mqtt_client=mqtt,
            collector=FakeCollector({"position": {"x": 1.0}}),
        )
        self.assertFalse(reporter.report_now())
        self.assertEqual(mqtt.trajectory, [])

    def test_trajectory_publishes_transform_envelope(self):
        mqtt = FakeMqtt()
        reporter = TrajectoryReporter(
            config=CONFIG,
            mqtt_client=mqtt,
            collector=FakeCollector(
                {
                    "position": {"x": 0.17, "y": 0.31, "yaw": 0.0},
                    "map": {"map_id": "3b241101-e2bb-4f3a-b20a-eaf62a0ee8d5"},
                }
            ),
        )
        self.assertTrue(reporter.report_now())
        frame = mqtt.trajectory[0]
        self.assertEqual(frame["device_sn"], "ROBOT_006")
        self.assertEqual(frame["frame_id"], "map")
        self.assertEqual(frame["child_frame_id"], "base_link")
        self.assertEqual(frame["translation"]["z"], 0.0)
        self.assertEqual(frame["rotation"]["w"], 1.0)
        self.assertEqual(
            frame["map_id"], "3b241101-e2bb-4f3a-b20a-eaf62a0ee8d5"
        )

    def test_state_collector_falls_back_to_task_store(self):
        store = TaskStateStore()
        store.on_accepted("TASK-9")

        class _Client:
            def get_json(self, url: str) -> Optional[Dict[str, Any]]:
                return None

        collector = StateCollector(
            {"state": {}, "scenarios": {}},
            scenario_client=_Client(),
            task_state=store,
        )
        snapshot = collector.snapshot()
        self.assertEqual(snapshot["task"]["task_id"], "TASK-9")
        self.assertEqual(snapshot["task"]["status"], 1)
        self.assertEqual(snapshot["self_check"]["status"], 0)

    def test_state_collector_merges_optional_osd_blocks(self):
        store = TaskStateStore()

        class _Client:
            def get_json(self, url: str) -> Optional[Dict[str, Any]]:
                if url.endswith("/state"):
                    return {
                        "robot_mode": {"source": "sros_amr", "location_state": "running"},
                        "self_check": {"status": 0, "message": "就绪"},
                    }
                return {"ok": True}

        collector = StateCollector(
            {"state": {"control": "http://127.0.0.1:8002"}, "scenarios": {}},
            scenario_client=_Client(),
            task_state=store,
        )
        snapshot = collector.snapshot()
        self.assertEqual(snapshot["robot_mode"]["source"], "sros_amr")

    def test_optional_sources_do_not_fail_self_check(self):
        store = TaskStateStore()

        class _Client:
            def get_json(self, url: str) -> Optional[Dict[str, Any]]:
                if "8001" in url and url.endswith("/state"):
                    return {
                        "self_check": {"status": 0, "message": "就绪"},
                        "position": {"x": 1.0, "y": 2.0, "yaw": 0.1},
                    }
                if url.endswith("/health"):
                    return {"ok": True}
                return None

        collector = StateCollector(
            {
                "state": {
                    "navigation": "http://127.0.0.1:8001",
                    "camera": "http://127.0.0.1:8003",
                },
                "scenarios": {
                    "rokae": {
                        "url": "http://127.0.0.1:8094",
                        "enabled": True,
                        "required": False,
                    }
                },
            },
            scenario_client=_Client(),
            task_state=store,
        )
        snapshot = collector.snapshot()
        self.assertEqual(snapshot["self_check"]["status"], 0)
        self.assertEqual(snapshot["position"]["x"], 1.0)

    def test_optional_self_check_does_not_overwrite_nav(self):
        store = TaskStateStore()

        class _Client:
            def get_json(self, url: str) -> Optional[Dict[str, Any]]:
                if "8001" in url and url.endswith("/state"):
                    return {
                        "self_check": {"status": 0, "message": "就绪"},
                        "position": {"x": 1.0, "y": 2.0, "yaw": 0.1},
                    }
                if "8003" in url and url.endswith("/state"):
                    return {"self_check": {"status": 1, "message": "相机离线"}}
                return {"ok": True}

        collector = StateCollector(
            {
                "state": {
                    "cache_ttl_sec": 0,
                    "navigation": "http://127.0.0.1:8001",
                    "camera": "http://127.0.0.1:8003",
                },
                "scenarios": {},
            },
            scenario_client=_Client(),
            task_state=store,
        )
        snapshot = collector.snapshot()
        self.assertEqual(snapshot["self_check"]["status"], 0)
        self.assertEqual(snapshot["self_check"]["message"], "就绪")

    def test_state_collector_reuses_snapshot_cache(self):
        store = TaskStateStore()
        calls: list[str] = []

        class _Client:
            def get_json(self, url: str) -> Optional[Dict[str, Any]]:
                calls.append(url)
                if url.endswith("/state"):
                    return {
                        "self_check": {"status": 0, "message": "就绪"},
                        "position": {"x": 1.0, "y": 2.0, "yaw": 0.1},
                    }
                return {"ok": True}

        collector = StateCollector(
            {
                "state": {
                    "cache_ttl_sec": 5,
                    "navigation": "http://127.0.0.1:8001",
                },
                "scenarios": {},
            },
            scenario_client=_Client(),
            task_state=store,
        )
        first = collector.snapshot()
        second = collector.snapshot()
        self.assertEqual(first["position"]["x"], second["position"]["x"])
        self.assertEqual(len(calls), 1)

    def test_forced_task_overrides_cached_snapshot(self):
        store = TaskStateStore()
        calls: list[str] = []

        class _Client:
            def get_json(self, url: str) -> Optional[Dict[str, Any]]:
                calls.append(url)
                if url.endswith("/state"):
                    return {
                        "self_check": {"status": 0, "message": "就绪"},
                        "task": {"task_id": "FROM-NAV", "status": 2},
                    }
                return {"ok": True}

        collector = StateCollector(
            {
                "state": {
                    "cache_ttl_sec": 5,
                    "navigation": "http://127.0.0.1:8001",
                },
                "scenarios": {},
            },
            scenario_client=_Client(),
            task_state=store,
        )
        first = collector.snapshot()
        self.assertEqual(first["task"]["task_id"], "FROM-NAV")
        call_count = len(calls)
        store.set(task_id="TASK-HTTP", status=1, forced=True)
        second = collector.snapshot()
        self.assertEqual(len(calls), call_count)
        self.assertEqual(second["task"]["task_id"], "TASK-HTTP")
        self.assertEqual(second["task"]["status"], 1)

    def test_optional_down_source_is_backed_off(self):
        store = TaskStateStore()
        urls: list[str] = []

        class _Client:
            def get_json(self, url: str) -> Optional[Dict[str, Any]]:
                urls.append(url)
                if "8001" in url and url.endswith("/state"):
                    return {
                        "self_check": {"status": 0, "message": "就绪"},
                        "position": {"x": 1.0, "y": 2.0, "yaw": 0.1},
                    }
                return None

        collector = StateCollector(
            {
                "state": {
                    "cache_ttl_sec": 0,
                    "skip_backoff_sec": 30,
                    "navigation": "http://127.0.0.1:8001",
                    "camera": "http://127.0.0.1:8003",
                },
                "scenarios": {},
            },
            scenario_client=_Client(),
            task_state=store,
        )
        collector.snapshot()
        first = list(urls)
        urls.clear()
        collector.snapshot()
        self.assertTrue(any("8003" in item for item in first))
        self.assertTrue(any("8001" in item for item in urls))
        self.assertFalse(any("8003" in item for item in urls))
        self.assertFalse(any(item.endswith("/health") for item in first + urls))

    def test_string_state_urls_match_work_gateway_shape(self):
        sources = {
            item.name: item
            for item in iter_state_sources(
                {
                    "state": {
                        "navigation": "http://127.0.0.1:8001",
                        "control": "http://127.0.0.1:8002",
                        "camera": "http://127.0.0.1:8003",
                    },
                    "scenarios": {},
                }
            )
        }
        self.assertTrue(sources["navigation"].required)
        self.assertFalse(sources["control"].required)
        self.assertFalse(sources["camera"].required)
        self.assertEqual(sources["navigation"].state_path, "/state")
        self.assertNotIn("manipulator", sources)

    def test_adapt_state_fragment_is_passthrough(self):
        source = StateSource(name="navigation", url="http://127.0.0.1:8001")
        body = {"position": {"x": 1.0, "y": 2.0}}
        self.assertEqual(adapt_state_fragment(source, body), body)

    def test_osd_fills_smt_contract_defaults(self):
        reporter = OsdReporter(
            config=CONFIG,
            mqtt_client=FakeMqtt(),
            collector=FakeCollector(
                {
                    "battery": {"capacity_percent": 40.0, "charging": False},
                    "chassis_status": {"state": "idle"},
                    "navigation_status": {"state": "IDLE"},
                    "map": {
                        "map_id": "3b241101-e2bb-4f3a-b20a-eaf62a0ee8d5",
                        "stations": [{"station_id": "A", "x": 1.0, "y": 2.0, "yaw": 0.0}],
                    },
                    "manipulator_status": {
                        "left_arm": {
                            "state": "idle",
                            "joint_positions_deg": [1, 2, 3, 4, 5, 6, 7],
                        }
                    },
                    "self_check": {"status": 0, "message": "就绪"},
                }
            ),
            host_reader=_EmptyHost(),
        )
        data = reporter.collect_osd_data()["data"]
        self.assertEqual(data["task"]["current_tray_index"], 0)
        self.assertEqual(data["task"]["has_tray_in_hand"], 0)
        self.assertEqual(data["battery"]["cycle"], 0)
        self.assertEqual(data["map"]["stations"][0]["name"], "A")
        self.assertEqual(data["map"]["stations"][0]["x"], 1.0)
        self.assertEqual(data["map"]["stations"][0]["y"], 2.0)
        self.assertEqual(data["navigation_status"]["target_station_id"], "")
        self.assertNotIn("alarm_status", data)
        self.assertNotIn("collector", data)
        self.assertTrue(data["transport"]["mqtt_connected"])

    def test_osd_fills_xcore_transport_endpoints(self):
        reporter = OsdReporter(
            config={
                **CONFIG,
                "site": {
                    "sros": {"host": "192.168.71.50", "port": 5001, "protocol": "srp"},
                    "xcore": {
                        "sdk_version": "0.7.1",
                        "left_arm_ip": "192.168.71.161",
                        "right_arm_ip": "192.168.71.160",
                        "trunk_ip": "192.168.71.162",
                    },
                },
            },
            mqtt_client=FakeMqtt(),
            collector=FakeCollector(
                {
                    "battery": {"capacity_percent": 40.0, "charging": False},
                    "chassis_status": {"state": "idle"},
                    "manipulator_status": {
                        "left_arm": {
                            "state": "idle",
                            "joint_positions_deg": [1, 2, 3, 4, 5, 6, 7],
                        }
                    },
                    "self_check": {"status": 0, "message": "就绪"},
                }
            ),
            host_reader=_EmptyHost(),
        )
        data = reporter.collect_osd_data()["data"]
        self.assertEqual(data["transport"]["endpoints"]["chassis"]["host"], "192.168.71.50")
        self.assertTrue(data["transport"]["endpoints"]["chassis"]["connected"])
        self.assertEqual(data["transport"]["endpoints"]["left_arm"]["protocol"], "xcore")
        self.assertEqual(data["transport"]["endpoints"]["left_arm"]["host"], "192.168.71.161")
        self.assertTrue(data["transport"]["endpoints"]["left_arm"]["connected"])
        self.assertEqual(data["arm_action"]["sdk_version"], "0.7.1")
        self.assertEqual(data["arm_action"]["endpoints"]["body"]["host"], "192.168.71.162")

    def test_uplink_queues_offline_events_and_flushes(self):
        mqtt = FakeMqtt()
        mqtt.is_connected = False
        store = TaskStateStore()
        uplink = UplinkPublisher(mqtt, task_state=store)
        result = uplink.publish_task_event(
            {
                "task_id": "TASK-Q",
                "step_index": 1,
                "stage": "pick_pem",
                "title": "grasp pose",
            }
        )
        self.assertTrue(result["accepted"])
        self.assertFalse(result["published"])
        self.assertEqual(mqtt.events, [])
        mqtt.is_connected = True
        self.assertEqual(uplink.flush_pending(), 1)
        self.assertEqual(mqtt.events[0][1]["task_id"], "TASK-Q")

    def test_uplink_result_updates_task_state_and_maps_stage(self):
        mqtt = FakeMqtt()
        store = TaskStateStore()
        uplink = UplinkPublisher(mqtt, task_state=store)
        result = uplink.publish_task_result(
            {
                "task_id": "TASK-1",
                "terminal_state": "SUCCEEDED",
                "title": "任务完成",
                "step_index": 20,
            }
        )
        self.assertTrue(result["accepted"])
        self.assertEqual(result["data"]["stage"], "task_completed")
        self.assertEqual(store.snapshot()["status"], 2)
        self.assertEqual(mqtt.events[0][0], "task_event")

    def test_uplink_event_marks_running_unless_forced(self):
        mqtt = FakeMqtt()
        store = TaskStateStore()
        uplink = UplinkPublisher(mqtt, task_state=store)
        uplink.publish_task_event(
            {
                "task_id": "TASK-1",
                "step_index": 1,
                "stage": "pick_pem",
                "title": "grasp pose",
            }
        )
        self.assertEqual(store.snapshot()["status"], 1)
        self.assertEqual(store.snapshot()["task_id"], "TASK-1")
        store.set(task_id="FORCED", status=2, forced=True)
        uplink.publish_task_event(
            {
                "task_id": "TASK-2",
                "step_index": 2,
                "stage": "place",
                "title": "place",
            }
        )
        self.assertEqual(store.snapshot()["task_id"], "FORCED")
        self.assertEqual(store.snapshot()["status"], 2)

    def test_forced_task_store_overrides_osd_task(self):
        store = TaskStateStore()
        store.set(task_id="TASK-HTTP", status=1)

        class _Client:
            def get_json(self, url: str) -> Optional[Dict[str, Any]]:
                if url.endswith("/state"):
                    return {"task": {"task_id": "FROM-SCENARIO", "status": 2}}
                return {"ok": True}

        collector = StateCollector(
            {"state": {"navigation": "http://127.0.0.1:8001"}, "scenarios": {}},
            scenario_client=_Client(),
            task_state=store,
        )
        snapshot = collector.snapshot()
        self.assertEqual(snapshot["task"]["task_id"], "TASK-HTTP")
        self.assertEqual(snapshot["task"]["status"], 1)


class TestOsdTaskHttp(unittest.TestCase):
    def test_apply_osd_task_update_and_clear(self):
        from gateway.debug_http import apply_osd_task_update

        store = TaskStateStore()
        snapshot = apply_osd_task_update(
            store,
            {
                "task_id": "TASK-1",
                "status": "running",
            },
        )
        self.assertTrue(store.forced)
        self.assertEqual(snapshot["status"], 1)
        self.assertEqual(snapshot["task_id"], "TASK-1")
        self.assertNotIn("current_tray_index", snapshot)
        self.assertNotIn("has_tray_in_hand", snapshot)
        cleared = store.clear()
        self.assertFalse(store.forced)
        self.assertEqual(cleared["status"], 0)
        self.assertEqual(cleared["task_id"], "")

    def test_fastapi_osd_routes(self):
        from fastapi.testclient import TestClient

        from gateway.debug_http import create_app

        class _Gateway:
            def __init__(self):
                self.task_state = TaskStateStore()

        client = TestClient(create_app(_Gateway()))
        empty = client.get("/osd/task")
        self.assertEqual(empty.status_code, 200)
        self.assertEqual(empty.json()["task"]["status"], 0)
        updated = client.post(
            "/osd/task",
            json={"task_id": "TASK-001", "status": 1},
        )
        self.assertEqual(updated.status_code, 200)
        self.assertEqual(updated.json()["task"]["task_id"], "TASK-001")
        missing = client.post("/osd/task", json={"task_id": "TASK-001"})
        self.assertEqual(missing.status_code, 400)
        self.assertEqual(missing.json()["error"], "INVALID_REQUEST")
        unknown = client.get("/nope")
        self.assertEqual(unknown.status_code, 404)
        self.assertEqual(unknown.json()["error"], "NOT_FOUND")
        listed = client.get("/")
        self.assertEqual(listed.status_code, 200)
        self.assertIn("POST /events", listed.json()["endpoints"])

    def test_fastapi_event_and_result_routes(self):
        from fastapi.testclient import TestClient

        from gateway.debug_http import create_app
        from gateway.mqtt.uplink import UplinkPublisher

        mqtt = FakeMqtt()

        class _Gateway:
            def __init__(self):
                self.task_state = TaskStateStore()
                self.uplink = UplinkPublisher(mqtt, task_state=self.task_state)

        gateway = _Gateway()
        client = TestClient(create_app(gateway))
        missing = client.post("/events", json={"stage": "pick_pem"})
        self.assertEqual(missing.status_code, 400)
        self.assertEqual(missing.json()["error"], "INVALID_REQUEST")
        sent = client.post(
            "/events",
            json={
                "data": {
                    "task_id": "TASK-001",
                    "step_index": 1,
                    "stage": "pick_pem",
                    "title": "grasp pose",
                    "thingking": "secret",
                }
            },
        )
        self.assertEqual(sent.status_code, 200)
        self.assertTrue(sent.json()["published"])
        self.assertEqual(sent.json()["data"]["thingking"], "")
        self.assertEqual(gateway.task_state.snapshot()["status"], 1)
        done = client.post(
            "/results",
            json={
                "task_id": "TASK-001",
                "terminal_state": "SUCCEEDED",
                "title": "任务完成",
                "step_index": 2,
            },
        )
        self.assertEqual(done.status_code, 200)
        self.assertEqual(done.json()["data"]["stage"], "task_completed")
        self.assertEqual(gateway.task_state.snapshot()["status"], 2)
        self.assertEqual(mqtt.events[0][0], "task_event")
        self.assertEqual(mqtt.events[1][1]["stage"], "task_completed")

    def test_unwrap_uplink_body_accepts_nested_data(self):
        from gateway.debug_http import unwrap_uplink_body

        nested = unwrap_uplink_body(
            {"tid": "", "data": {"task_id": "T1", "stage": "pick_pem"}}
        )
        self.assertEqual(nested["task_id"], "T1")
        flat = unwrap_uplink_body({"task_id": "T1", "stage": "pick_pem"})
        self.assertEqual(flat["stage"], "pick_pem")


class TestGatewayAppMqtt(unittest.TestCase):
    def test_app_registers_mqtt_command_handlers(self):
        app = GatewayApp(
            {
                "device": {"sn": "ROBOT_TEST"},
                "mqtt": {
                    "broker_host": "mqtt.example.test",
                    "topics": {
                        "services": "thing/product/robot/{sn}/services",
                        "services_reply": "thing/product/robot/{sn}/services_reply",
                        "events": "thing/product/robot/{sn}/event",
                        "osd": "thing/product/robot/{sn}/osd",
                        "trajectory": "thing/product/robot/{sn}/trajectory",
                    },
                },
                "scenarios": {
                    "smt": {"url": "http://127.0.0.1:8090", "enabled": True}
                },
            }
        )
        try:
            self.assertIn("task_start", app.mqtt_client.handlers)
            self.assertIn("task_stop", app.mqtt_client.handlers)
            self.assertIn("task_recovery", app.mqtt_client.handlers)
            self.assertIn("rokae_start", app.mqtt_client.handlers)
            self.assertNotIn("robot_move", app.mqtt_client.handlers)
            self.assertFalse(app.mqtt_connected)
            self.assertIn(
                app._retry_map_sync_on_mqtt,
                app.mqtt_client._connect_callbacks,
            )
        finally:
            app.mqtt_client.executor.shutdown(wait=False)


class TestDebugHttpPorts(unittest.TestCase):
    def test_listen_ports_default_fallback(self):
        from gateway.debug_http import listen_ports_from_config

        self.assertEqual(listen_ports_from_config({}), [8088, 8089])
        self.assertEqual(
            listen_ports_from_config({"port": 8088, "fallback_ports": [8089]}),
            [8088, 8089],
        )
        self.assertEqual(listen_ports_from_config({"port": 8099}), [8099])

    def test_first_free_port_skips_busy(self):
        import socket

        from gateway.debug_http import first_free_port

        busy = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        busy.bind(("127.0.0.1", 0))
        busy.listen(1)
        occupied = busy.getsockname()[1]
        probe = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        probe.bind(("127.0.0.1", 0))
        free = probe.getsockname()[1]
        probe.close()
        try:
            self.assertEqual(first_free_port("127.0.0.1", [occupied, free]), free)
        finally:
            busy.close()


if __name__ == "__main__":
    unittest.main()
