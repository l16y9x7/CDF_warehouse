import os
import sys
import unittest


PROJECT_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
if PROJECT_ROOT not in sys.path:
    sys.path.insert(0, PROJECT_ROOT)

from navigation.adapters.tianji import TianjiNavigationAdapter
from navigation.adapters.tianji_telemetry import (
    aggregate_battery_channels,
    osd_from_get_state,
    parse_get_state_content,
)


SHANGHAI_CONTENT = {
    "isNormal": True,
    "isScram": False,
    "connectState": 1,
    "isParkingMode": True,
    "taskId": 0,
    "linearSpeedVec3": {"x": 0.0, "y": 0.0},
    "angularSpeedVec3": {"z": 0.0},
    "batteryStateList": [
        {
            "isConnected": True,
            "isInPlace": True,
            "errorCode": 0,
            "energy": 69,
            "voltage": 49.8,
            "temperature": 31,
            "isCharging": False,
        },
        {
            "isConnected": True,
            "isInPlace": True,
            "errorCode": 0,
            "energy": 66,
            "voltage": 49.7,
            "temperature": 33,
            "isCharging": False,
        },
    ],
}


class TestTianjiTelemetry(unittest.TestCase):
    def test_shanghai_fixture_matches_smt(self):
        fragment = osd_from_get_state(SHANGHAI_CONTENT)
        self.assertEqual(fragment["battery"]["capacity_percent"], 66.0)
        self.assertEqual(fragment["battery"]["temperature"], 33.0)
        self.assertEqual(fragment["battery"]["voltage"], 49.7)
        self.assertFalse(fragment["battery"]["charging"])
        self.assertEqual(fragment["battery"]["cycle"], 31)
        self.assertEqual(fragment["chassis_status"]["state"], "idle")
        self.assertTrue(fragment["chassis_status"]["motion"]["stopped"])
        self.assertEqual(fragment["chassis_status"]["motion"]["linear_speed_mps"], 0.0)
        self.assertNotIn("drive_mode", fragment["chassis_status"])

    def test_invalid_battery_channel_is_ignored(self):
        battery = aggregate_battery_channels(
            [
                {
                    "isConnected": False,
                    "isInPlace": True,
                    "errorCode": 0,
                    "energy": 90,
                    "temperature": 40,
                    "isCharging": False,
                }
            ]
        )
        self.assertEqual(battery, {})

    def test_parse_get_state_requires_error_code_zero(self):
        self.assertEqual(parse_get_state_content({"ErrorCode": 1, "Content": {}}), {})
        self.assertEqual(
            parse_get_state_content({"ErrorCode": 0, "Content": SHANGHAI_CONTENT})[
                "connectState"
            ],
            1,
        )


class TestTianjiAdapterGetState(unittest.TestCase):
    def test_snapshot_merges_get_state_osd_fields(self):
        import navigation.adapters.tianji as tianji

        def fake_http(url, **kwargs):
            if url.endswith("/GetState"):
                return 200, {"ErrorCode": 0, "Content": SHANGHAI_CONTENT}
            return 200, {"status": "READY"}

        original = tianji._http_json
        tianji._http_json = fake_http
        try:
            adapter = TianjiNavigationAdapter(
                {
                    "tianji": {
                        "stations_yaml": "",
                        "get_state": {
                            "enabled": True,
                            "base_url": "http://6.6.7.6:8080",
                        },
                    }
                }
            )
            snap = adapter.snapshot()
            self.assertEqual(snap["battery"]["capacity_percent"], 66.0)
            self.assertEqual(snap["chassis_status"]["state"], "idle")
        finally:
            tianji._http_json = original

    def test_get_state_disabled_without_config(self):
        import navigation.adapters.tianji as tianji

        calls = []

        def fake_http(url, **kwargs):
            calls.append(url)
            return 200, {"status": "READY"}

        original = tianji._http_json
        tianji._http_json = fake_http
        try:
            adapter = TianjiNavigationAdapter({"tianji": {"stations_yaml": ""}})
            snap = adapter.snapshot()
            self.assertNotIn("battery", snap)
            self.assertTrue(all("/GetState" not in url for url in calls))
        finally:
            tianji._http_json = original


if __name__ == "__main__":
    unittest.main()
