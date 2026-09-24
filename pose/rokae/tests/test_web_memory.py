import copy
import http.client
import json
import tempfile
import threading
import unittest
from pathlib import Path

from rokae_web.backends import MockChassisBackend, MockRobotBackend
from rokae_web.config import DEFAULT_CONFIG
from rokae_web.memory_points import MemoryPointStore
from rokae_web.service import ControlService
from rokae_web.web import make_server


class MemoryDeleteHTTPTests(unittest.TestCase):
    def test_create_delete_and_refresh_while_locked(self):
        with tempfile.TemporaryDirectory() as directory:
            config = copy.deepcopy(DEFAULT_CONFIG)
            config['memory_points'] = {'file': str(Path(directory)/'mock.json')}
            service = ControlService(config, MockRobotBackend(), MockChassisBackend(), False)
            server = make_server('127.0.0.1', 0, service, Path(__file__).resolve().parents[1]/'static')
            thread = threading.Thread(target=server.serve_forever, daemon=True)
            thread.start()

            def request(method, route, body=None):
                connection = http.client.HTTPConnection('127.0.0.1', server.server_port, timeout=3)
                connection.request(method, '/api/memory-points'+route,
                                   body=None if body is None else json.dumps(body),
                                   headers={'Content-Type': 'application/json'})
                response = connection.getresponse()
                result = response.status, json.loads(response.read())
                connection.close()
                return result

            try:
                status, created = request('POST', '/create', {'name': '删除测试点'})
                self.assertEqual(status, 200)
                point = created['data']['point']
                payload = {'id': point['id'], 'revision': point['revision']}
                status, deleted = request('POST', '/delete', payload)
                self.assertEqual(status, 200)
                self.assertEqual(deleted['data']['points'], [])
                self.assertEqual(request('GET', '')[1]['data']['points'], [])
                self.assertTrue(deleted['data']['can_delete'])
                self.assertFalse(service.armed)
                self.assertEqual(MemoryPointStore(config['memory_points']['file']).list(), [])
                self.assertEqual(request('POST', '/delete', payload)[0], 400)
            finally:
                server.shutdown()
                server.server_close()
                thread.join(timeout=2)
                service.close()


if __name__ == '__main__':
    unittest.main()
