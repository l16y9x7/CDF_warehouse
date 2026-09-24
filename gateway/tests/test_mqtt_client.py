import json
import os
import sys
import unittest


PROJECT_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
if PROJECT_ROOT not in sys.path:
    sys.path.insert(0, PROJECT_ROOT)

from gateway.mqtt.client import MQTTClient
from gateway.mqtt.envelopes import normalize_service_result, result_succeeded


def _mqtt_config(**mqtt_overrides):
    mqtt_config = {
        "broker_host": "mqtt.example.test",
        "broker_port": 1883,
        "topics": {
            "services": "thing/product/robot/{sn}/services",
            "services_reply": "thing/product/robot/{sn}/services_reply",
            "events": "thing/product/robot/{sn}/event",
            "osd": "thing/product/robot/{sn}/osd",
            "trajectory": "thing/product/robot/{sn}/trajectory",
        },
    }
    mqtt_config.update(mqtt_overrides)
    return {"device": {"sn": "ROBOT_TEST"}, "mqtt": mqtt_config}


class _FakePublishResult:
    def __init__(self, *, rc=0, published=True, wait_error=None):
        self.rc = rc
        self.published = published
        self.wait_error = wait_error
        self.wait_timeouts = []

    def wait_for_publish(self, timeout):
        self.wait_timeouts.append(timeout)
        if self.wait_error is not None:
            raise self.wait_error

    def is_published(self):
        return self.published


class _FakeMQTTTransport:
    def __init__(self):
        self.reconnect_delays = []
        self.connect_async_calls = []
        self.loop_start_calls = 0
        self.loop_stop_calls = 0
        self.disconnect_calls = 0
        self.subscribe_calls = []
        self.publish_calls = []
        self.loop_start_rc = 0
        self.publish_result = _FakePublishResult()

    def reconnect_delay_set(self, *, min_delay, max_delay):
        self.reconnect_delays.append((min_delay, max_delay))

    def connect_async(self, host, port, keepalive):
        self.connect_async_calls.append((host, port, keepalive))

    def loop_start(self):
        self.loop_start_calls += 1
        return self.loop_start_rc

    def loop_stop(self):
        self.loop_stop_calls += 1

    def disconnect(self):
        self.disconnect_calls += 1

    def subscribe(self, topic, qos):
        self.subscribe_calls.append((topic, qos))
        return (0, 1)

    def publish(self, topic, payload, qos):
        self.publish_calls.append((topic, payload, qos))
        return self.publish_result


class TestMQTTClient(unittest.TestCase):
    def test_outbound_only_client_never_subscribes_or_accepts_handlers(self):
        client = MQTTClient(_mqtt_config(), subscribe_services=False)
        self.assertIsNone(client.client.on_message)
        transport = _FakeMQTTTransport()
        client.client = transport
        try:
            client._on_connect(transport, None, None, 0)
            self.assertTrue(client.is_connected)
            self.assertEqual(transport.subscribe_calls, [])
            with self.assertRaisesRegex(RuntimeError, "inbound MQTT"):
                client.register_handler("task_start", lambda payload: payload)
        finally:
            client.stop()

    def test_start_uses_async_connection_and_configured_backoff(self):
        client = MQTTClient(
            _mqtt_config(reconnect_initial_sec=2, reconnect_max_sec=32)
        )
        transport = _FakeMQTTTransport()
        client.client = transport
        try:
            client.start()
            client.start()
            client._on_connect_fail(transport, None)
            self.assertTrue(client.running)
            self.assertEqual(transport.reconnect_delays, [(2, 32)])
            self.assertEqual(
                transport.connect_async_calls,
                [("mqtt.example.test", 1883, 60)],
            )
            self.assertEqual(transport.loop_start_calls, 1)
            self.assertEqual(client._connect_failure_count, 1)
        finally:
            client.stop()
        self.assertEqual(transport.disconnect_calls, 1)
        self.assertEqual(transport.loop_stop_calls, 1)

    def test_reconnect_restores_subscription(self):
        client = MQTTClient(_mqtt_config())
        transport = _FakeMQTTTransport()
        client.client = transport
        try:
            client._on_connect(transport, None, None, 0)
            self.assertEqual(
                transport.subscribe_calls,
                [("thing/product/robot/ROBOT_TEST/services", 1)],
            )
        finally:
            client.stop()

    def test_publish_uses_compact_strict_json(self):
        client = MQTTClient(_mqtt_config())
        transport = _FakeMQTTTransport()
        client.client = transport
        client.is_connected = True
        payload = {"label": "华数", "values": [1, 2]}
        try:
            self.assertTrue(client.publish("topic/a", payload, qos=0))
            self.assertEqual(
                transport.publish_calls,
                [
                    (
                        "topic/a",
                        json.dumps(
                            payload,
                            ensure_ascii=False,
                            allow_nan=False,
                            separators=(",", ":"),
                        ),
                        0,
                    )
                ],
            )
        finally:
            client.stop()

    def test_publish_rejects_non_finite_json(self):
        client = MQTTClient(_mqtt_config())
        transport = _FakeMQTTTransport()
        client.client = transport
        client.is_connected = True
        try:
            with self.assertLogs(client.logger, level="WARNING"):
                self.assertFalse(
                    client.publish("topic/a", {"value": float("nan")}, qos=0)
                )
            self.assertEqual(transport.publish_calls, [])
        finally:
            client.stop()

    def test_task_events_use_smt_stage_envelope(self):
        client = MQTTClient(_mqtt_config())
        published = []
        client.publish = lambda topic, payload, qos=1: published.append(
            (topic, payload, qos)
        ) or True
        try:
            self.assertTrue(
                client.publish_event(
                    "task_event",
                    {
                        "task_id": "TASK-001",
                        "step_index": 4,
                        "stage": "pick_pem",
                        "title": "grasp pose",
                        "description": "pose estimate completed",
                        "thingking": "hidden",
                        "image_url": "https://example.test/pick_pem.png",
                    },
                )
            )
            topic, payload, qos = published[0]
            self.assertEqual(topic, "thing/product/robot/ROBOT_TEST/event")
            self.assertEqual(qos, 1)
            self.assertEqual(set(payload), {"tid", "bid", "timestamp", "sn", "data"})
            self.assertEqual(payload["tid"], "")
            self.assertEqual(payload["bid"], "")
            self.assertEqual(payload["sn"], "ROBOT_TEST")
            self.assertEqual(payload["data"]["stage"], "pick_pem")
            self.assertEqual(payload["data"]["thingking"], "")
            self.assertNotIn("type", payload["data"])
        finally:
            client.executor.shutdown(wait=False)

    def test_command_dedupe_returns_cached_reply(self):
        client = MQTTClient(_mqtt_config())
        replies = []
        client.send_service_reply = lambda bid, tid, result: replies.append(
            (bid, tid, result)
        )
        client.register_handler("task_start", lambda data: {"result": 0, "task_id": data["task_id"]})

        class _Msg:
            topic = client.topic_services
            payload = json.dumps(
                {
                    "method": "task_start",
                    "bid": "B1",
                    "tid": "T1",
                    "data": {"task_id": "TASK-1"},
                }
            ).encode("utf-8")

        try:
            client.executor.shutdown(wait=True)
            client.executor = None
            from concurrent.futures import ThreadPoolExecutor

            client.executor = ThreadPoolExecutor(max_workers=1)
            client._on_message(None, None, _Msg())
            client.executor.shutdown(wait=True)
            client._on_message(None, None, _Msg())
            self.assertEqual(len(replies), 2)
            self.assertEqual(replies[1][2]["result"], 0)
        finally:
            if client.executor is not None:
                client.executor.shutdown(wait=False)

    def test_unknown_method_replies_error(self):
        client = MQTTClient(_mqtt_config())
        replies = []
        client.send_service_reply = lambda bid, tid, result: replies.append(result)

        class _Msg:
            topic = client.topic_services
            payload = json.dumps(
                {"method": "robot_move", "bid": "B", "tid": "T", "data": {}}
            ).encode("utf-8")

        try:
            client._on_message(None, None, _Msg())
            self.assertEqual(replies[0]["result"], -1)
            self.assertIn("Unknown method", replies[0]["error"])
        finally:
            client.executor.shutdown(wait=False)

    def test_service_reply_envelope(self):
        client = MQTTClient(_mqtt_config())
        published = []
        client.is_connected = True
        client.publish = lambda topic, payload, qos=1: published.append(payload) or True
        try:
            client.send_service_reply(
                "BID-1",
                "TID-1",
                {"code": 0, "message": "task start accepted", "data": {"result": 0}},
            )
            reply = published[0]
            self.assertEqual(reply["bid"], "BID-1")
            self.assertEqual(reply["tid"], "TID-1")
            self.assertEqual(reply["code"], 0)
            self.assertEqual(reply["data"]["result"], 0)
            self.assertIn("timestamp", reply)
        finally:
            client.executor.shutdown(wait=False)

    def test_result_success_requires_code_and_data(self):
        self.assertTrue(result_succeeded({"code": 0, "data": {"result": 0}}))
        self.assertFalse(result_succeeded({"code": 0, "data": {"result": -1}}))
        self.assertEqual(
            normalize_service_result({"result": 0, "task_id": "T"})["code"], 0
        )


if __name__ == "__main__":
    unittest.main()
