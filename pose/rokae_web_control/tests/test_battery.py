import sys
import unittest
from types import SimpleNamespace
from unittest.mock import Mock, patch
import copy

from rokae_web.battery import BatteryTelemetry
from rokae_web.backends import BackendError, MockChassisBackend, Ros2ChassisBackend
from rokae_web.backends import MockRobotBackend
from rokae_web.config import DEFAULT_CONFIG
from rokae_web.service import ControlService


class BatteryTests(unittest.TestCase):
    def setUp(self):
        self.now = 100.0
        self.cache = BatteryTelemetry(clock=lambda: self.now)

    def message(self, level=16, state=1):
        return SimpleNamespace(remaining_percentage=level, state=state, voltage=49820)

    def test_empty_is_unknown_but_zero_is_a_real_percentage(self):
        self.assertIsNone(self.cache.snapshot()["percentage"])
        self.cache.update(self.message(0, 3))
        self.assertTrue(self.cache.snapshot()["available"])
        self.assertEqual(self.cache.snapshot()["percentage"], 0)
        self.assertIs(self.cache.snapshot()["charging"], False)

    def test_percentage_is_not_rescaled_and_charging_is_explicit(self):
        self.cache.update(self.message())
        self.assertEqual(self.cache.snapshot()["percentage"], 16)
        self.assertIs(self.cache.snapshot()["charging"], True)
        self.assertEqual(self.cache.snapshot()["voltage_v"], 49.82)
        self.cache.update(self.message(80, 32))
        self.assertIsNone(self.cache.snapshot()["charging"])

    def test_stale_values_are_hidden_and_fresh_message_recovers(self):
        self.cache.update(self.message())
        self.now += 15
        self.assertTrue(self.cache.snapshot()["stale"])
        self.assertIsNone(self.cache.snapshot()["percentage"])
        self.assertIsNone(self.cache.snapshot()["charging"])
        self.cache.update(self.message(17))
        self.assertEqual(self.cache.snapshot()["percentage"], 17)
        self.assertFalse(self.cache.snapshot()["stale"])

    def test_invalid_or_absent_battery_does_not_keep_previous_level(self):
        for level, state in ((101, 3), (-1, 3), (True, 3), (float("nan"), 3), (0, 0)):
            self.cache.update(self.message())
            self.cache.update(self.message(level, state))
            self.assertFalse(self.cache.snapshot()["available"])
            self.assertIsNone(self.cache.snapshot()["percentage"])

    def test_mock_and_ros_failure_never_fabricate_percentage(self):
        self.assertIsNone(MockChassisBackend().status()["battery"]["percentage"])
        backend = Ros2ChassisBackend({})
        with patch.object(backend, "_ensure_ros", side_effect=BackendError("ROS unavailable")):
            status = backend.status()
        self.assertFalse(status["ros_ready"])
        self.assertFalse(status["battery"]["available"])

    def test_status_creates_only_subscription_and_never_sends_control(self):
        node = Mock()
        node.create_client.return_value.service_is_ready.return_value = True
        modules = {
            "rclpy": SimpleNamespace(ok=lambda: True, create_node=lambda name: node),
            "rclpy.executors": SimpleNamespace(SingleThreadedExecutor=Mock()),
            "geometry_msgs.msg": SimpleNamespace(Twist=object),
            "std_srvs.srv": SimpleNamespace(SetBool=object, Trigger=object),
            "sr_amr_interfaces.msg": SimpleNamespace(BatteryState=object, SystemState=object),
        }
        backend = Ros2ChassisBackend({"cmd_vel_topic": "/cmd_vel", "remote_control_service": "/remote",
                                     "release_emergency_stop_service": "/release"})
        with patch.dict(sys.modules, modules), patch("rokae_web.backends.threading.Thread"):
            backend.status()
            subscription = next(call.args for call in node.create_subscription.call_args_list
                                if call.args[1] == "/sr_amr_control/battery_state")
            self.assertEqual(subscription[1], "/sr_amr_control/battery_state")
            subscription[2](self.message())
            self.assertEqual(backend.status()["battery"]["percentage"], 16)
        node.create_publisher.return_value.publish.assert_not_called()
        node.create_client.return_value.call_async.assert_not_called()
        self.assertEqual(node.create_subscription.call_count, 2)
        self.assertEqual({call.args[1] for call in node.create_subscription.call_args_list},
                         {"/sr_amr_control/battery_state", "/sr_amr_control/system_state"})

    def test_chassis_offline_does_not_block_upper_body_status_or_unlock(self):
        config = copy.deepcopy(DEFAULT_CONFIG)
        backend = Ros2ChassisBackend(config['chassis'])
        backend._client = Mock()
        backend._emergency_stop_client = Mock()
        backend._client.service_is_ready.return_value = False
        backend._emergency_stop_client.service_is_ready.return_value = False
        service = ControlService(config, MockRobotBackend(), backend, False)
        try:
            with patch.object(backend, '_ensure_ros'):
                status = service.status()
                self.assertFalse(status['chassis']['ros_ready'])
                self.assertIn('底盘', status['chassis']['error'])
                service.arm({})
                self.assertTrue(service.status()['armed'])
                backend._client.service_is_ready.side_effect = RuntimeError('ROS graph offline')
                status = service.status()
                self.assertEqual(status['chassis']['error'], 'ROS graph offline')
                self.assertTrue(status['armed'])
                backend._client.service_is_ready.side_effect = None
                backend._client.service_is_ready.return_value = True
                self.assertIsNone(service.status()['chassis']['error'])
        finally:
            service.close()


if __name__ == "__main__":
    unittest.main()
