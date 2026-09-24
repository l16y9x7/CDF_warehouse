import json
import os
import sys
import unittest
from typing import Any, Dict, List, Tuple
from urllib.error import HTTPError, URLError


PROJECT_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
if PROJECT_ROOT not in sys.path:
    sys.path.insert(0, PROJECT_ROOT)

from gateway.dispatcher import CommandDispatcher
from gateway.mqtt.command_adapter import MqttCommandAdapter
from gateway.mqtt.envelopes import normalize_service_result, normalize_task_event
from gateway.scenario_client import ScenarioClient, ScenarioResponse


def _config(**scenario_overrides):
    scenarios = {
        "smt": {"url": "http://127.0.0.1:8090", "enabled": True},
        "wrc": {"url": "http://127.0.0.1:8091", "enabled": True},
        "piano": {"url": "http://127.0.0.1:8092", "enabled": True},
        "retail": {"url": "http://127.0.0.1:8093", "enabled": True},
        "rokae": {"url": "http://127.0.0.1:8094", "enabled": True},
    }
    scenarios.update(scenario_overrides)
    return {
        "default_scenario": "smt",
        "scenarios": scenarios,
        "mqtt": {"command_dedupe_ttl_sec": 300, "command_dedupe_max_entries": 64},
    }


class FakeScenarioClient:
    def __init__(self) -> None:
        self.calls: List[Tuple[str, str, Dict[str, Any]]] = []
        self.response = ScenarioResponse(
            reached=True,
            accepted=True,
            status_code=200,
            body={"accepted": True, "status": "running"},
        )
        self.by_url: Dict[str, ScenarioResponse] = {}

    def post(self, base_url: str, path: str, body: Dict[str, Any]) -> ScenarioResponse:
        self.calls.append((base_url, path, body))
        return self.by_url.get(base_url, self.response)

    def get_json(self, url: str):
        return None


class TestCommandDispatcher(unittest.TestCase):
    def setUp(self):
        self.client = FakeScenarioClient()
        self.dispatcher = CommandDispatcher(_config(), scenario_client=self.client)

    def test_task_start_defaults_to_smt_and_forwards_payload(self):
        payload = {
            "task_id": "TASK-1",
            "map_id": "3b241101-e2bb-4f3a-b20a-eaf62a0ee8d5",
            "total_trays": 1,
            "pickup": "LM2",
        }
        reply = self.dispatcher.dispatch("task_start", payload, tid="TID-1")

        self.assertEqual(reply["code"], 0)
        self.assertEqual(reply["data"]["result"], 0)
        self.assertEqual(reply["data"]["status"], "running")
        self.assertEqual(reply["data"]["task_id"], "TASK-1")
        self.assertEqual(len(self.client.calls), 1)
        url, path, body = self.client.calls[0]
        self.assertEqual(url, "http://127.0.0.1:8090")
        self.assertEqual(path, "/tasks")
        self.assertEqual(body["scenario"], "smt")
        self.assertEqual(body["task_id"], "TASK-1")
        self.assertEqual(body["idempotency_key"], "TID-1")
        self.assertEqual(body["payload"], payload)

    def test_task_start_uses_data_scenario(self):
        self.dispatcher.dispatch(
            "task_start",
            {"task_id": "TASK-2", "scenario": "retail"},
            tid="TID-2",
        )
        self.assertEqual(self.client.calls[0][0], "http://127.0.0.1:8093")
        self.assertEqual(self.client.calls[0][2]["scenario"], "retail")

    def test_task_start_uses_config_default_scenario(self):
        dispatcher = CommandDispatcher(
            {**_config(), "default_scenario": "rokae"},
            scenario_client=self.client,
        )
        dispatcher.dispatch("task_start", {"task_id": "TASK-R"}, tid="TID-R")
        self.assertEqual(self.client.calls[0][0], "http://127.0.0.1:8094")
        self.assertEqual(self.client.calls[0][2]["scenario"], "rokae")

    def test_forced_scenario_methods(self):
        self.dispatcher.dispatch(
            "start_material_task", {"task_id": "T", "material_code": "M1"}
        )
        self.dispatcher.dispatch("play", {"task_id": "T", "piece_id": "p1"})
        self.dispatcher.dispatch("retail_start", {"task_id": "T", "sku": "s1"})
        self.dispatcher.dispatch("rokae_start", {"task_id": "T", "program": "pick"})
        self.assertEqual(
            [call[0] for call in self.client.calls],
            [
                "http://127.0.0.1:8091",
                "http://127.0.0.1:8092",
                "http://127.0.0.1:8093",
                "http://127.0.0.1:8094",
            ],
        )
        self.assertEqual(self.client.calls[0][2]["payload"]["material_code"], "M1")
        self.assertEqual(self.client.calls[1][2]["payload"]["piece_id"], "p1")
        self.assertEqual(self.client.calls[3][2]["payload"]["program"], "pick")
        self.assertEqual(self.client.calls[3][2]["scenario"], "rokae")

    def test_task_stop_fans_out_to_all_enabled_scenarios(self):
        reply = self.dispatcher.dispatch(
            "task_stop", {"task_id": "TASK-1", "reason": "operator_stop"}
        )
        self.assertEqual(reply["code"], 0)
        paths = {(url, path) for url, path, _body in self.client.calls}
        self.assertEqual(
            paths,
            {
                ("http://127.0.0.1:8090", "/tasks/stop"),
                ("http://127.0.0.1:8091", "/tasks/stop"),
                ("http://127.0.0.1:8092", "/tasks/stop"),
                ("http://127.0.0.1:8093", "/tasks/stop"),
                ("http://127.0.0.1:8094", "/tasks/stop"),
            },
        )
        self.assertEqual(
            self.client.calls[0][2]["payload"]["reason"], "operator_stop"
        )

    def test_task_recovery_goes_to_smt(self):
        self.dispatcher.dispatch(
            "task_recovery",
            {"task_id": "TASK-1", "action": "return_home", "tray_removed": 1},
        )
        url, path, body = self.client.calls[0]
        self.assertEqual(url, "http://127.0.0.1:8090")
        self.assertEqual(path, "/tasks/recover")
        self.assertEqual(body["payload"]["tray_removed"], 1)

    def test_unknown_method_and_missing_task_id_are_rejected(self):
        unknown = self.dispatcher.dispatch("robot_move", {"command": "forward"})
        self.assertEqual(unknown["code"], -1)
        self.assertEqual(unknown["data"]["error"], "GATEWAY_UNKNOWN_METHOD")
        missing = self.dispatcher.dispatch("task_start", {"map_id": "x"})
        self.assertEqual(missing["data"]["error"], "GATEWAY_REQUEST_INVALID")
        self.assertEqual(self.client.calls, [])

    def test_unconfigured_scenario_is_rejected(self):
        dispatcher = CommandDispatcher(
            _config(wrc={"url": "http://127.0.0.1:8091", "enabled": False}),
            scenario_client=self.client,
        )
        reply = dispatcher.dispatch(
            "start_material_task", {"task_id": "TASK-1", "material_code": "M"}
        )
        self.assertEqual(reply["data"]["error"], "GATEWAY_SCENARIO_NOT_CONFIGURED")
        self.assertEqual(self.client.calls, [])

    def test_http_tid_dedupe_returns_cached_result(self):
        first = self.dispatcher.dispatch(
            "task_start", {"task_id": "TASK-1"}, tid="TID-DUP"
        )
        second = self.dispatcher.dispatch(
            "task_start", {"task_id": "TASK-1"}, tid="TID-DUP"
        )
        self.assertEqual(first, second)
        self.assertEqual(len(self.client.calls), 1)

    def test_records_capture_acceptance(self):
        self.dispatcher.dispatch("task_start", {"task_id": "TASK-9"}, tid="TID-9")
        records = self.dispatcher.command_log.list(task_id="TASK-9")
        self.assertEqual(len(records), 1)
        self.assertTrue(records[0]["accepted"])
        self.assertEqual(records[0]["method"], "task_start")

    def test_scenario_rejection_is_not_treated_as_accepted(self):
        self.client.response = ScenarioResponse(
            reached=True,
            accepted=False,
            status_code=200,
            body={"accepted": False, "error_code": "RESOURCE_BUSY"},
            error="busy",
            error_code="RESOURCE_BUSY",
        )
        reply = self.dispatcher.dispatch("task_start", {"task_id": "TASK-1"})
        self.assertEqual(reply["code"], -1)
        self.assertEqual(reply["data"]["error"], "RESOURCE_BUSY")
        self.assertEqual(self.dispatcher.task_state.snapshot()["status"], 0)


class TestMqttCommandAdapter(unittest.TestCase):
    def test_registers_architecture_methods(self):
        dispatcher = CommandDispatcher(_config(), scenario_client=FakeScenarioClient())
        methods = set(MqttCommandAdapter(dispatcher).handlers())
        self.assertTrue(
            {
                "task_start",
                "StartTask",
                "start_material_task",
                "play",
                "PlayPiece",
                "retail_start",
                "rokae_start",
                "task_stop",
                "task_recovery",
            }.issubset(methods)
        )
        self.assertNotIn("robot_move", methods)

    def test_handler_forwards_data_without_interpreting_fields(self):
        client = FakeScenarioClient()
        dispatcher = CommandDispatcher(_config(), scenario_client=client)
        reply = MqttCommandAdapter(dispatcher).handlers()["task_start"](
            {"task_id": "TASK-1", "total_trays": 99, "pickup": "LM2"}
        )
        self.assertEqual(reply["code"], 0)
        self.assertEqual(client.calls[0][2]["payload"]["total_trays"], 99)

    def test_handler_forwards_mqtt_tid_as_idempotency_key(self):
        client = FakeScenarioClient()
        dispatcher = CommandDispatcher(_config(), scenario_client=client)
        reply = MqttCommandAdapter(dispatcher).handlers()["task_start"](
            {"task_id": "TASK-1"},
            bid="B1",
            tid="TID-MQTT",
        )
        self.assertEqual(reply["code"], 0)
        self.assertEqual(client.calls[0][2]["idempotency_key"], "TID-MQTT")
        records = dispatcher.command_log.list(task_id="TASK-1")
        self.assertEqual(records[0]["tid"], "TID-MQTT")
        self.assertEqual(records[0]["bid"], "B1")


class TestScenarioClient(unittest.TestCase):
    def test_unreachable_maps_to_gateway_error(self):
        def opener(request, timeout):
            raise URLError("down")

        client = ScenarioClient(opener=opener)
        response = client.post("http://127.0.0.1:8090", "/tasks", {"task_id": "T"})
        self.assertFalse(response.reached)
        self.assertEqual(response.error_code, "GATEWAY_SCENARIO_UNREACHABLE")

    def test_http_200_accepted_false(self):
        class _Resp:
            status = 200

            def read(self):
                return json.dumps({"accepted": False, "error_code": "X"}).encode()

            def __enter__(self):
                return self

            def __exit__(self, *args):
                return False

        client = ScenarioClient(opener=lambda request, timeout: _Resp())
        response = client.post("http://127.0.0.1:8090", "/tasks", {})
        self.assertTrue(response.reached)
        self.assertFalse(response.accepted)
        self.assertEqual(response.error_code, "X")

    def test_get_json_uses_poll_timeout(self):
        seen = {}

        class _Resp:
            status = 200

            def read(self):
                return b'{"ok": true}'

            def __enter__(self):
                return self

            def __exit__(self, *args):
                return False

        def opener(request, timeout):
            seen["timeout"] = timeout
            return _Resp()

        client = ScenarioClient(
            timeout_sec=8.0, poll_timeout_sec=0.5, opener=opener
        )
        self.assertEqual(client.get_json("http://127.0.0.1/state"), {"ok": True})
        self.assertEqual(seen["timeout"], 0.5)

    def test_get_json_keeps_503_json_body(self):
        from io import BytesIO

        payload = {
            "ok": False,
            "data": {
                "modules": {
                    "left_arm": {
                        "valid": True,
                        "joint_positions_deg": [1, 2, 3, 4, 5, 6, 7],
                        "state": "idle",
                    }
                }
            },
        }

        def opener(request, timeout):
            raise HTTPError(
                "http://127.0.0.1/api/telemetry/upper-body",
                503,
                "Service Unavailable",
                hdrs=None,
                fp=BytesIO(json.dumps(payload).encode("utf-8")),
            )

        client = ScenarioClient(poll_timeout_sec=0.5, opener=opener)
        body = client.get_json("http://127.0.0.1/api/telemetry/upper-body")
        self.assertIsNotNone(body)
        self.assertFalse(body["ok"])
        self.assertTrue(body["data"]["modules"]["left_arm"]["valid"])


class TestEnvelopes(unittest.TestCase):
    def test_command_reply_normalization(self):
        normalized = normalize_service_result(
            {"result": -1, "error": "bad request"}
        )
        self.assertEqual(normalized["code"], -1)
        self.assertEqual(normalized["data"]["error"], "bad request")

    def test_task_event_forces_empty_thingking(self):
        event = normalize_task_event(
            {
                "task_id": "TASK-001",
                "step_index": 4,
                "stage": "pick_pem",
                "title": "grasp pose",
                "thingking": "secret reasoning",
                "description": "ok",
            }
        )
        self.assertEqual(event["thingking"], "")
        self.assertEqual(event["stage"], "pick_pem")
        self.assertNotIn("type", event)


if __name__ == "__main__":
    unittest.main()
