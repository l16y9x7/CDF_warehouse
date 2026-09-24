import copy
import http.client
import json
import socket
import tempfile
import threading
import unittest
import time
from unittest.mock import Mock
from pathlib import Path
from rokae_web.control_proxy import CoreWebServer, make_proxy
from rokae_web.service import ControlService
from rokae_web.config import DEFAULT_CONFIG
from rokae_web.backends import MockRobotBackend, MockChassisBackend
from rokae_web.agent_actions import AgentActions
from wait_control_ready import wait_for_core


@unittest.skipUnless(hasattr(socket,'AF_UNIX'),'Unix runtime only')
class ProxyTests(unittest.TestCase):
    def test_startup_ready_does_not_query_status_or_hardware(self):
        with tempfile.TemporaryDirectory() as temp:
            root = Path(temp)
            service = Mock()
            service.status.side_effect = RuntimeError('chassis offline / motion lock busy')
            core = CoreWebServer(root/'core.sock', service, root)
            thread = threading.Thread(target=core.serve_forever, daemon=True)
            thread.start()
            try:
                start = time.monotonic()
                wait_for_core(root/'core.sock', .5)
                elapsed = time.monotonic()-start
                self.assertLess(elapsed, .5)
                self.assertEqual(service.mock_calls, [])
                print(f'core readiness without hardware reads: {elapsed*1000:.2f}ms')
            finally:
                core.shutdown();core.server_close();thread.join(2)

    def test_missing_core_is_not_reported_ready(self):
        with tempfile.TemporaryDirectory() as temp:
            with self.assertRaisesRegex(RuntimeError, '运控核心未就绪'):
                wait_for_core(Path(temp)/'missing.sock', .05)

    def test_frontend_restart_preserves_core_and_shared_speed_and_mutex(self):
        with tempfile.TemporaryDirectory() as temp:
            root=Path(temp);config=copy.deepcopy(DEFAULT_CONFIG)
            config['memory_points']={'file':str(root/'memory.json')}
            service=ControlService(config,MockRobotBackend(),MockChassisBackend(),False)
            actions=AgentActions(service,root/'agent')
            core=CoreWebServer(root/'core.sock',service,root)
            thread=threading.Thread(target=core.serve_forever,daemon=True);thread.start()
            try:
                for i in range(2):
                    proxy=make_proxy('127.0.0.1',0,str(root/'core.sock'),root)
                    worker=threading.Thread(target=proxy.serve_forever,daemon=True);worker.start()
                    try:
                        c=http.client.HTTPConnection(*proxy.server_address)
                        if i==0:
                            c.request('POST','/api/speed',json.dumps({'speed_mm_s':77,'rotation_deg_s':9}))
                            response=c.getresponse();self.assertEqual(response.status,200);response.read()
                        else:
                            self.assertEqual(service.speed_mm_s,77)
                            actions.active={'key':'busy'}
                            c.request('POST','/api/speed',json.dumps({'speed_mm_s':99,'rotation_deg_s':9}))
                            response=c.getresponse();self.assertEqual(response.status,409);response.read()
                            self.assertEqual(service.speed_mm_s,77)
                        c.close()
                    finally:
                        proxy.shutdown();proxy.server_close();worker.join(2)
            finally:
                actions.active=None
                core.shutdown();core.server_close();thread.join(2);service.close();actions.ledger.db.close()
