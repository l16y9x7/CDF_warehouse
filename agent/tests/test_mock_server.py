import os
import unittest
from unittest.mock import patch

from agent.capabilities.mocks.server import services


class MockServerPortTest(unittest.TestCase):
    def test_defaults_do_not_overlap_production_ports(self):
        with patch.dict(os.environ, {}, clear=True):
            self.assertEqual(
                {service.name: service.port for service in services()},
                {
                    "navigation": 28081,
                    "pose": 28082,
                    "perception": 28083,
                    "estimation": 28084,
                    "camera": 28085,
                    "manipulation": 28086,
                    "vla": 28087,
                    "hand": 28088,
                },
            )

    def test_mock_port_override_is_independent(self):
        with patch.dict(os.environ, {"AGENT_MOCK_CAMERA_PORT": "29085"}, clear=True):
            ports = {service.name: service.port for service in services()}
        self.assertEqual(ports["camera"], 29085)
        self.assertEqual(ports["navigation"], 28081)


if __name__ == "__main__":
    unittest.main()
