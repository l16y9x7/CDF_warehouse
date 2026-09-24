import os
import sys
import unittest

PROJECT_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
APPS_ROOT = os.path.join(PROJECT_ROOT, "apps")
for _path in (PROJECT_ROOT, APPS_ROOT):
    if _path not in sys.path:
        sys.path.insert(0, _path)

from fastapi.testclient import TestClient

from zhongmian.http_app import create_app
from zhongmian.service import NavigationFacade


class FakeNav:
    def __init__(self) -> None:
        self.ok = True
        self.gotos = []
        self.terminal = "SUCCEEDED"

    def ready(self) -> bool:
        return bool(self.ok)

    def goto(self, body):
        self.gotos.append(dict(body))
        return {"accepted": True, "request_id": body["request_id"], "state": "ACCEPTED"}

    def status(self, request_id):
        del request_id
        return {"state": self.terminal, "terminal_state": self.terminal}


def _client(nav=None, **overrides):
    config = {
        "nav": {"timeout_sec": 1, "poll_sec": 0.01, "task_id": "ZHONGMIAN"},
        "waypoints": {"AGV_L": "A", "AGV_R": "B"},
    }
    config.update(overrides)
    facade = NavigationFacade(config, nav=nav or FakeNav())
    return TestClient(create_app(facade)), facade


class TestZhongmianNavigate(unittest.TestCase):
    def test_health_ready(self) -> None:
        client, _ = _client()
        response = client.get("/navigation/health")
        self.assertEqual(response.status_code, 200)
        self.assertEqual(response.json(), {"status": "READY"})

    def test_health_error_when_nav_down(self) -> None:
        nav = FakeNav()
        nav.ok = False
        client, _ = _client(nav)
        response = client.get("/navigation/health")
        self.assertEqual(response.status_code, 200)
        self.assertEqual(response.json(), {"status": "ERROR"})

    def test_navigate_requires_key_and_nav_id(self) -> None:
        client, _ = _client()
        missing_key = client.post("/navigation/navigate", json={"nav_id": "AGV_L"})
        self.assertEqual(missing_key.status_code, 400)
        self.assertEqual(missing_key.json(), {"error_code": "EXECUTION_FAILED"})
        missing_id = client.post(
            "/navigation/navigate",
            json={},
            headers={"Idempotency-Key": "k1"},
        )
        self.assertEqual(missing_id.status_code, 400)

    def test_unknown_nav_id(self) -> None:
        client, _ = _client()
        response = client.post(
            "/navigation/navigate",
            json={"nav_id": "NOPE"},
            headers={"Idempotency-Key": "k1"},
        )
        self.assertEqual(response.status_code, 400)
        self.assertEqual(response.json(), {"error_code": "EXECUTION_FAILED"})

    def test_navigate_maps_to_station_and_succeeds(self) -> None:
        nav = FakeNav()
        client, _ = _client(nav)
        response = client.post(
            "/navigation/navigate",
            json={"nav_id": "AGV_L"},
            headers={"Idempotency-Key": "task-1:nav"},
        )
        self.assertEqual(response.status_code, 200)
        self.assertEqual(response.json(), {"status": "SUCCEEDED"})
        self.assertEqual(nav.gotos[0]["station_id"], "A")
        self.assertEqual(nav.gotos[0]["idempotency_key"], "task-1:nav")

    def test_target_id_alias(self) -> None:
        nav = FakeNav()
        client, _ = _client(nav)
        response = client.post(
            "/navigation/navigate",
            json={"target_id": "AGV_R"},
            headers={"Idempotency-Key": "k-r"},
        )
        self.assertEqual(response.status_code, 200)
        self.assertEqual(nav.gotos[0]["station_id"], "B")

    def test_idempotency_replays_without_second_goto(self) -> None:
        nav = FakeNav()
        client, _ = _client(nav)
        headers = {"Idempotency-Key": "same"}
        first = client.post("/navigation/navigate", json={"nav_id": "AGV_L"}, headers=headers)
        second = client.post("/navigation/navigate", json={"nav_id": "AGV_L"}, headers=headers)
        self.assertEqual(first.json(), second.json())
        self.assertEqual(len(nav.gotos), 1)

    def test_nav_failure_is_non_2xx(self) -> None:
        nav = FakeNav()
        nav.terminal = "FAILED"
        client, _ = _client(nav)
        response = client.post(
            "/navigation/navigate",
            json={"nav_id": "AGV_L"},
            headers={"Idempotency-Key": "k-fail"},
        )
        self.assertEqual(response.status_code, 500)
        self.assertEqual(response.json(), {"error_code": "EXECUTION_FAILED"})


if __name__ == "__main__":
    unittest.main()
