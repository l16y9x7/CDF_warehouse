import os
import sys
import unittest


PROJECT_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
if PROJECT_ROOT not in sys.path:
    sys.path.insert(0, PROJECT_ROOT)

from gateway.manipulator_port import ManipulatorStateReader, _install_xcore_joint_truncation
from gateway.mqtt.osd import StateCollector
from gateway.records import TaskStateStore
from gateway.state_sources import adapt_state_fragment, StateSource


class _Snapshot:
    def __init__(self, legacy, full):
        self._legacy = legacy
        self._full = full

    def to_legacy_dual_arm_mapping(self):
        return self._legacy

    def to_mapping(self):
        return self._full


class TestManipulatorPort(unittest.TestCase):
    def test_get_state_passed_through(self):
        reader = ManipulatorStateReader()

        class _Svc:
            def get_state(self):
                return _Snapshot(
                    {"left_arm": {"state": "idle"}, "right_arm": {"state": "idle"}},
                    {"left_arm": {"state": "idle", "power_state": "on"}},
                )

        reader._service = _Svc()
        blocks = reader.osd_blocks()
        self.assertEqual(blocks["manipulator_status"]["left_arm"]["state"], "idle")
        self.assertEqual(blocks["arm_action"]["left_arm"]["power_state"], "on")

    def test_collector_merges_get_state_blocks(self):
        class _Port:
            def osd_blocks(self):
                return {"manipulator_status": {"left_arm": {"state": "idle"}}}

        class _Client:
            def get_json(self, url):
                return None

        collector = StateCollector(
            {"state": {"navigation": "http://127.0.0.1:9"}},
            scenario_client=_Client(),
            task_state=TaskStateStore(),
        )
        collector.manipulator = _Port()
        self.assertEqual(collector.snapshot()["manipulator_status"]["left_arm"]["state"], "idle")

    def test_http_fragment_is_passthrough(self):
        body = {"position": {"x": 1}}
        self.assertEqual(
            adapt_state_fragment(StateSource(name="navigation", url="http://127.0.0.1:8001"), body),
            body,
        )

    def test_gateway_truncates_overlong_xcore_joints(self):
        class _Mod:
            ARM_DOF = 7

            @staticmethod
            def _read_vector(getter, label):
                return tuple(getter({}))

        _install_xcore_joint_truncation(_Mod)
        self.assertEqual(
            len(_Mod._read_vector(lambda _ec: [0.0] * 13, "JOINT_POSITION")),
            7,
        )
        self.assertEqual(
            len(_Mod._read_vector(lambda _ec: [0.0] * 6, "JOINT_POSITION")),
            6,
        )

    def test_config_url_reads_local_http(self):
        payload = {
            "ok": True,
            "data": {
                "left_arm": {
                    "state": "idle",
                    "joint_positions_deg": [1, 2, 3, 4, 5, 6, 7],
                    "end_pose": [1, 2, 3, 4, 5, 6],
                },
                "right_arm": {
                    "state": "idle",
                    "joint_positions_deg": [7, 6, 5, 4, 3, 2, 1],
                    "end_pose": [6, 5, 4, 3, 2, 1],
                },
                "body": {"state": "idle", "joint_positions_deg": [10, 11, 12, 13]},
            },
        }

        class _Resp:
            def read(self):
                import json

                return json.dumps(payload).encode("utf-8")

            def __enter__(self):
                return self

            def __exit__(self, *args):
                return False

        reader = ManipulatorStateReader(
            {"manipulator": {"url": "http://127.0.0.1:8092/api/telemetry/manipulator-state"}}
        )
        self.assertTrue(reader.start())
        reader._service._opener = lambda request, timeout=0.8: _Resp()
        blocks = reader.osd_blocks()
        self.assertEqual(
            blocks["manipulator_status"]["left_arm"]["joint_positions_deg"][-1], 7
        )
        self.assertEqual(blocks["arm_action"]["body"]["joint_positions_deg"][0], 10)


if __name__ == "__main__":
    unittest.main()
