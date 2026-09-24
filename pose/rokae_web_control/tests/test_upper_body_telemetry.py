"""No hardware: SDK doubles, local HTTP and process-ownership tests."""
import http.client
import json
import math
import os
from pathlib import Path
import tempfile
import threading
from types import SimpleNamespace
import unittest
from unittest.mock import patch

from rokae_web.backends import MockRobotBackend, XCoreRobotBackend
from rokae_web.hardware_owner import HardwareOwner
from rokae_web.telemetry import MODULES, UpperBodyTelemetry
from rokae_web.web import make_server
from rokae_web.telemetry_http import TelemetryHTTPServer


class TelemetryTests(unittest.TestCase):
    def setUp(self):
        self.robot = MockRobotBackend()
        self.cache = UpperBodyTelemetry(self.robot, {})

    def test_failures_are_per_module_and_never_filled_with_zeros(self):
        read = self.robot.read_telemetry_module
        def selective(module):
            if module == "right_arm":
                raise RuntimeError("offline")
            return read(module)
        self.robot.read_telemetry_module = selective
        self.cache.poll_once()
        s = self.cache.snapshot()
        self.assertFalse(s["all_valid"])
        self.assertTrue(s["modules"]["left_arm"]["valid"])
        self.assertIsNone(s["modules"]["right_arm"]["joint_positions_deg"])
        self.assertEqual(len(s["modules"]["trunk"]["joint_positions_deg"]), 4)

    def test_expired_or_failed_sample_is_not_returned_as_current(self):
        self.cache.poll_once()
        with self.cache._lock:
            self.cache._records["left_arm"]["monotonic"] -= 10
        s = self.cache.snapshot("left_arm")["modules"]["left_arm"]
        self.assertTrue(s["stale"])
        self.assertIsNone(s["end_pose"])
        self.assertIsNotNone(s["sampled_at"])

    def test_busy_motion_lock_skips_poll_without_waiting(self):
        entered, release = threading.Event(), threading.Event()
        def hold():
            with self.robot._lock:
                entered.set()
                release.wait(2)
        t = threading.Thread(target=hold)
        t.start()
        self.assertTrue(entered.wait(1))
        try:
            poll = threading.Thread(target=self.cache.poll_once)
            poll.start()
            poll.join(.2)
            self.assertFalse(poll.is_alive())
            self.assertEqual(self.cache.snapshot()["modules"]["trunk"]["sequence"], 0)
        finally:
            release.set()
            t.join()

    def test_http_reads_cache_and_supports_partial_module_query(self):
        self.cache.poll_once()
        self.robot.read_telemetry_module = lambda *_: self.fail("HTTP must not call SDK")
        self.robot.close = lambda: self.fail("telemetry does not disconnect the owner")
        server = TelemetryHTTPServer(("127.0.0.1", 0), self.cache)
        t = threading.Thread(target=server.serve_forever, daemon=True)
        t.start()
        try:
            for suffix in ("", "/left_arm", "/right_arm", "/trunk"):
                c = http.client.HTTPConnection("127.0.0.1", server.server_port)
                c.request("GET", "/api/telemetry/upper-body" + suffix)
                response = c.getresponse()
                data = json.loads(response.read())
                self.assertEqual(response.status, 200)
                self.assertEqual(len(data["data"]["modules"]), 1 if suffix else 3)
                self.assertEqual(data["data"]["mode"], "mock")
                c.close()
            with self.cache._lock:
                self.cache._records["trunk"]["error"] = "lost"
            c = http.client.HTTPConnection("127.0.0.1", server.server_port)
            c.request("GET", "/api/telemetry/upper-body")
            response = c.getresponse()
            self.assertEqual(response.status, 503)
            self.assertFalse(json.loads(response.read())["ok"])
            c.close()
            c = http.client.HTTPConnection("127.0.0.1", server.server_port)
            c.request("POST", "/api/joints/right_arm", body='{}')
            response = c.getresponse()
            self.assertEqual(response.status, 405)
            response.read()
            c.close()
        finally:
            server.shutdown()
            server.server_close()
            t.join()
            self.cache.close()

    def test_sdk_reads_actual_flange_base_and_excludes_head_axes(self):
        calls = []
        backend = XCoreRobotBackend({})
        backend._sdk = SimpleNamespace(CoordinateType=SimpleNamespace(flangeInBase="flange-base"))
        for module, count in MODULES.items():
            def cart(coordinate, ec):
                calls.append(coordinate)
                return SimpleNamespace(trans=[.1, .2, .3], rpy=[.1, .2, .3])
            robot = SimpleNamespace(jointPos=lambda ec: [math.pi/2]*10,
                cartPosture=cart, operationState=lambda ec: SimpleNamespace(name="moving"))
            backend._robots[module] = robot
            result = backend.read_telemetry_module(module)
            self.assertEqual(result["joint_positions_deg"], [90.0]*count)
            self.assertEqual(result["state"], "moving")
            self.assertEqual(result["end_pose"]["position_mm"], [100.0, 200.0, 300.0])
        self.assertEqual(calls, ["flange-base"]*3)

    @unittest.skipUnless(os.name == "posix", "hardware owner is Linux-only")
    def test_duplicate_owner_rejected_until_original_exits(self):
        with tempfile.TemporaryDirectory() as directory:
            first, second = HardwareOwner(Path(directory)/"owner"), HardwareOwner(Path(directory)/"owner")
            first.acquire()
            try:
                with self.assertRaisesRegex(RuntimeError, "已运行"):
                    second.acquire()
            finally:
                first.close()
            second.acquire()
            second.close()

    def test_occupied_port_fails_before_constructing_hardware(self):
        import server as app
        server = make_server("127.0.0.1", 0, None, Path("static"))
        args = SimpleNamespace(config=None, hardware=True, host="127.0.0.1",
                               port=server.server_port, wait_for_existing=False)
        try:
            with patch.object(app, "parse_args", return_value=args), \
                 patch.object(app, "BrokerRobotBackend") as backend:
                with self.assertRaises(OSError):
                    app.main()
                backend.assert_not_called()
        finally:
            server.server_close()


if __name__ == "__main__":
    unittest.main()
