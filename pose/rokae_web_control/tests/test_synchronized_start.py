"""Controller group validation and dispatch; all controllers are fakes."""
import threading
import unittest
from unittest.mock import Mock

from rokae_web.backends import BackendError, XCoreRobotBackend
from rokae_web.sdk_broker import BrokerCore
from rokae_web.sdk_wire import SDK
from rokae_web.synchronized_start import start, validate_modules
from test_sdk_broker import FakeController


class SynchronizedStartTests(unittest.TestCase):
    def test_supported_groups_include_left_trunk_and_preserve_single_arm(self):
        for modules in (['left_arm'], ['right_arm'], ['left_arm', 'right_arm'],
                        ['right_arm', 'left_arm'], ['left_arm', 'trunk'],
                        ['trunk', 'left_arm'], ['right_arm', 'trunk'],
                        ['trunk', 'right_arm']):
            with self.subTest(modules=modules):
                validate_modules(modules)

    def test_invalid_group_never_touches_controllers_or_broker_queues(self):
        backend = Mock()
        core = BrokerCore(backend, Mock())
        core.queued = {'left_arm', 'right_arm', 'trunk'}
        for modules in (None, [], 'left_arm', ('left_arm', 'trunk'), ['trunk'],
                        ['left_arm', 'left_arm'], ['trunk', 'trunk'],
                        ['left_arm', 'right_arm', 'trunk'], ['head', 'trunk'],
                        ['left_arm', 'unknown'], ['left_arm', {}]):
            with self.subTest(modules=modules):
                with self.assertRaises(BackendError):
                    start(backend, modules)
                with self.assertRaises(BackendError):
                    core.start_arms({'modules': modules})
                self.assertEqual(core.queued, {'left_arm', 'right_arm', 'trunk'})
                self.assertFalse(core.dirty)
        self.assertEqual(backend.mock_calls, [])

    def test_left_trunk_broker_starts_concurrently_without_touching_right_arm(self):
        backend = XCoreRobotBackend({})
        backend._sdk = SDK
        backend._robots = {m: FakeController(n) for m, n in
                           [('left_arm', 7), ('right_arm', 7), ('trunk', 4)]}
        core = BrokerCore(backend, Mock())
        entered = {m: threading.Event() for m in ('left_arm', 'trunk')}
        def begin(name, ec):
            entered[name].set()
            other = 'trunk' if name == 'left_arm' else 'left_arm'
            if not entered[other].wait(1):
                raise RuntimeError('controllers started serially')
        for name in entered:
            backend._robots[name].moveStart = lambda ec, name=name: begin(name, ec)
        core.queued = {'left_arm'}
        with self.assertRaisesRegex(BackendError, '未全部就绪'):
            core.start_arms({'modules': ['left_arm', 'trunk']})
        self.assertFalse(any(e.is_set() for e in entered.values()))
        core.queued.add('trunk')
        result = core.start_arms({'modules': ['left_arm', 'trunk']})
        self.assertTrue(result['ok'])
        self.assertTrue(all(e.is_set() for e in entered.values()))
        self.assertFalse(backend._robots['right_arm'].calls)
        self.assertFalse(core.queued)


if __name__ == '__main__':
    unittest.main()
