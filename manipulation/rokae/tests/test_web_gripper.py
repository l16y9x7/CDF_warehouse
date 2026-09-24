from __future__ import annotations

import copy
import http.client
import json
import threading
import unittest
from pathlib import Path

from rokae_web.backends import MockChassisBackend, MockRobotBackend
from rokae_web.config import DEFAULT_CONFIG
from rokae_web.service import ControlService
from rokae_web.web import make_server


class GripperHTTPTests(unittest.TestCase):
    def setUp(self) -> None:
        self.service = ControlService(
            copy.deepcopy(DEFAULT_CONFIG), MockRobotBackend(), MockChassisBackend(), False
        )
        static = Path(__file__).resolve().parents[1] / "static"
        self.server = make_server("127.0.0.1", 0, self.service, static)
        self.thread = threading.Thread(target=self.server.serve_forever, daemon=True)
        self.thread.start()

    def tearDown(self) -> None:
        self.server.shutdown()
        self.server.server_close()
        self.thread.join(timeout=2)
        self.service.close()

    def request(self, method: str, path: str, body: dict | None = None):
        connection = http.client.HTTPConnection("127.0.0.1", self.server.server_port, timeout=3)
        payload = None if body is None else json.dumps(body)
        connection.request(method, path, body=payload, headers={"Content-Type": "application/json"})
        response = connection.getresponse()
        data = json.loads(response.read())
        connection.close()
        return response.status, data

    def test_lock_activation_and_position_routes(self) -> None:
        status, result = self.request("POST", "/api/gripper/position", {"position": 30})
        self.assertEqual(status, 400)
        self.assertFalse(result["ok"])

        self.request("POST", "/api/control/arm", {})
        self.request("POST", "/api/gripper/lock", {"unlocked": True})
        status, result = self.request("POST", "/api/gripper/activate", {})
        self.assertEqual(status, 200)
        self.assertEqual(result["data"]["activation_state"], 3)
        status, result = self.request("POST", "/api/gripper/position", {"position": 30})
        self.assertEqual(status, 200)
        self.assertEqual(result["data"]["measured_position"], 30)
        status, result = self.request("GET", "/api/gripper/status")
        self.assertEqual(status, 200)
        self.assertTrue(result["data"]["unlocked"])
        self.assertEqual(result["data"]["requested_position"], 30)


if __name__ == "__main__":
    unittest.main()
