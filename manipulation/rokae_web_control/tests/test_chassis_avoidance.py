import copy
import http.client
import json
import threading
import unittest
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import Mock, patch

from rokae_web.audit import MemoryAuditLogger
from rokae_web.backends import BackendError, MockChassisBackend, MockRobotBackend, Ros2ChassisBackend
from rokae_web.config import DEFAULT_CONFIG
from rokae_web.service import ControlService, ValidationError
from rokae_web.web import make_server


class AvoidanceBackendTests(unittest.TestCase):
    def setUp(self):
        self.backend = Ros2ChassisBackend(copy.deepcopy(DEFAULT_CONFIG['chassis']))
        self.backend._ensure_ros = Mock()
        self.backend._SetBool = SimpleNamespace(Request=SimpleNamespace)
        self.client = self.backend._obstacle_avoidance_client = Mock()
        self.client.wait_for_service.return_value = True
        self.future = self.client.call_async.return_value
        self.future.done.return_value = True
        self.future.result.return_value = SimpleNamespace(success=True, message='')
        self.backend._client = Mock()
        self.backend._publisher = Mock()

    def test_state_requires_fresh_boolean_and_false_is_valid(self):
        with patch('rokae_web.backends.time.monotonic', return_value=100) as clock:
            self.assertIsNone(self.backend._obstacle_status(True)['enabled'])
            self.backend._update_obstacle_avoidance(SimpleNamespace(remote_control_oba_active=False))
            self.assertIs(self.backend._obstacle_status(True)['enabled'], False)
            clock.return_value = 103
            self.assertTrue(self.backend._obstacle_status(True)['stale'])
            self.assertIsNone(self.backend._obstacle_status(True)['enabled'])
            self.backend._update_obstacle_avoidance(SimpleNamespace(remote_control_oba_active=True))
            self.assertIs(self.backend._obstacle_status(True)['enabled'], True)
            self.backend._update_obstacle_avoidance(SimpleNamespace(remote_control_oba_active=None))
            self.assertIsNone(self.backend._obstacle_status(True)['enabled'])

    def test_switch_uses_only_avoidance_service_and_confirms_result(self):
        for enabled in (False, True):
            self.backend.set_obstacle_avoidance(enabled)
            self.assertIs(self.client.call_async.call_args.args[0].data, enabled)
            state = self.backend.status()['obstacle_avoidance']
            self.assertIs(state['enabled'], enabled)
            self.assertEqual(state['source'], 'ros2_service_confirmed')
            self.assertFalse(state['stale'])
        self.assertEqual(self.client.call_async.call_count, 2)
        self.backend._client.call_async.assert_not_called()
        self.backend._publisher.publish.assert_not_called()

    def test_rejected_result_invalidates_old_state_and_does_not_retry(self):
        self.backend._update_obstacle_avoidance(SimpleNamespace(remote_control_oba_active=True))
        self.future.result.return_value = SimpleNamespace(success=False, message='controller refused')
        with self.assertRaisesRegex(BackendError, 'controller refused'):
            self.backend.set_obstacle_avoidance(False)
        self.assertIsNone(self.backend.status()['obstacle_avoidance']['enabled'])
        self.client.call_async.assert_called_once()

    def test_timeout_is_unconfirmed_and_does_not_retry(self):
        self.future.done.return_value = False
        with patch('rokae_web.backends.time.monotonic', side_effect=[0, 8]):
            with self.assertRaisesRegex(BackendError, '结果未确认'):
                self.backend.set_obstacle_avoidance(False)
        self.assertIsNone(self.backend.status()['obstacle_avoidance']['enabled'])
        self.client.call_async.assert_called_once()

    def test_missing_service_sends_nothing(self):
        self.client.wait_for_service.return_value = False
        with self.assertRaisesRegex(BackendError, '找不到'):
            self.backend.set_obstacle_avoidance(False)
        self.client.call_async.assert_not_called()


class AvoidanceHTTPTests(unittest.TestCase):
    def setUp(self):
        self.audit = MemoryAuditLogger()
        self.chassis = MockChassisBackend()
        self.service = ControlService(copy.deepcopy(DEFAULT_CONFIG), MockRobotBackend(),
                                      self.chassis, False, audit=self.audit)
        self.server = make_server('127.0.0.1', 0, self.service, Path(__file__).resolve().parents[1]/'static')
        self.thread = threading.Thread(target=self.server.serve_forever, daemon=True)
        self.thread.start()

    def tearDown(self):
        self.server.shutdown()
        self.server.server_close()
        self.thread.join(timeout=2)
        self.service.close()

    def post(self, payload):
        connection = http.client.HTTPConnection('127.0.0.1', self.server.server_port, timeout=3)
        connection.request('POST', '/api/chassis/obstacle-avoidance', json.dumps(payload),
                           {'Content-Type': 'application/json'})
        response = connection.getresponse()
        result = json.loads(response.read())
        connection.close()
        return response.status, result

    def test_locked_and_invalid_requests_do_not_switch(self):
        self.assertEqual(self.post({'enabled': False})[0], 400)
        self.service.arm({})
        for payload in ({}, {'enabled': 0}, {'enabled': 'false'}, {'enabled': None},
                        {'enabled': False, 'nav_stop_distance': 0}):
            self.assertEqual(self.post(payload)[0], 400)
        self.assertTrue(self.chassis.obstacle_avoidance_enabled)

    def test_round_trip_and_enable_remote_restores_avoidance(self):
        self.service.arm({})
        for enabled in (False, True, False):
            status, result = self.post({'enabled': enabled})
            self.assertEqual(status, 200)
            self.assertIs(result['data']['obstacle_avoidance']['enabled'], enabled)
            self.assertFalse(result['data']['remote_enabled'])
        result = self.service.set_chassis_enabled({'enabled': True})
        self.assertTrue(result['obstacle_avoidance']['enabled'])
        events = [record['event'] for record in self.audit.records]
        self.assertEqual(events.count('chassis_obstacle_avoidance_requested'), 3)
        self.assertEqual(events.count('chassis_obstacle_avoidance_changed'), 3)

    def test_failure_logged_and_not_reported_as_success(self):
        self.service.arm({})
        with patch.object(self.chassis, 'set_obstacle_avoidance', side_effect=BackendError('offline')) as switch:
            status, result = self.post({'enabled': False})
        self.assertEqual(status, 400)
        self.assertFalse(result['ok'])
        switch.assert_called_once_with(False)
        events = [record['event'] for record in self.audit.records]
        self.assertIn('chassis_obstacle_avoidance_failed', events)
        self.assertNotIn('chassis_obstacle_avoidance_changed', events)

    def test_active_motion_rejects_switch(self):
        self.service.arm({})
        with patch.object(self.service.memory, 'ensure_idle', side_effect=ValidationError('busy')):
            self.assertEqual(self.post({'enabled': False})[0], 400)
        self.assertTrue(self.chassis.obstacle_avoidance_enabled)


if __name__ == '__main__':
    unittest.main()
