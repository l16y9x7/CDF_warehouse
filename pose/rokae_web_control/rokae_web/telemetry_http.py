"""Read-only public endpoint, independent of the development web process."""
import json
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import urlparse


class TelemetryHandler(BaseHTTPRequestHandler):
    def log_message(self, *args):
        pass

    def respond(self, status, payload):
        raw = json.dumps(payload, ensure_ascii=False, allow_nan=False).encode('utf-8')
        self.send_response(status)
        self.send_header('Content-Type', 'application/json; charset=utf-8')
        self.send_header('Content-Length', str(len(raw)))
        self.send_header('Cache-Control', 'no-store')
        self.end_headers()
        self.wfile.write(raw)

    def do_GET(self):
        path = urlparse(self.path).path
        root = '/api/telemetry/upper-body'
        routes = {root: None, **{root+'/'+m: m for m in ('left_arm','right_arm','trunk')}}
        try:
            if path in routes:
                data = self.server.telemetry.snapshot(routes[path])
                self.respond(200 if data['all_valid'] else 503, {'ok': data['all_valid'], 'data': data})
            elif path == '/api/telemetry/manipulator-state' and hasattr(self.server.telemetry, 'get_state'):
                data = self.server.telemetry.get_state().to_mapping()
                self.respond(200, {'ok': True, 'data': data})
            elif path == '/health':
                self.respond(200, {'ok': True, 'service': 'mui-sdk', 'version': 'sdk-broker-v1',
                                  'manipulator_state_version': 1 if hasattr(self.server.telemetry, 'get_state') else None,
                                  'read_only': True, 'telemetry': self.server.telemetry.snapshot()})
            else:
                self.respond(404, {'ok': False, 'error': '接口不存在'})
        except (BrokenPipeError, ConnectionResetError):
            pass

    def do_POST(self):
        self.close_connection = True
        self.respond(405, {'ok': False, 'error': '此端口只提供只读查询；不接受运控指令'})

    do_PUT = do_DELETE = do_PATCH = do_POST


class TelemetryHTTPServer(ThreadingHTTPServer):
    daemon_threads = True

    def __init__(self, address, telemetry):
        self.telemetry = telemetry
        super().__init__(address, TelemetryHandler)
