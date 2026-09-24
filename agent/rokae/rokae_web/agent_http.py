"""Synchronous HTTP contracts for BodyPose (8082) and Manipulation (8086)."""
import json
import uuid
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import urlparse, parse_qs
from .agent_actions import AgentError
from .control_trace import context


class AgentHandler(BaseHTTPRequestHandler):
    def _send(self, status, payload):
        data = json.dumps(payload, ensure_ascii=False, allow_nan=False).encode()
        self.send_response(status)
        self.send_header('Content-Type', 'application/json; charset=utf-8')
        self.send_header('Content-Length', str(len(data)))
        self.send_header('Cache-Control', 'no-store')
        self.end_headers()
        self.wfile.write(data)

    def do_GET(self):
        path = urlparse(self.path).path
        actions = self.server.actions
        try:
            if path == self.server.prefix + '/health':
                result = actions.health(pose=self.server.prefix == '/pose')
            elif path == '/pose/camera_transform' and self.server.prefix == '/pose':
                query = parse_qs(urlparse(self.path).query)
                if query.get('camera', ['head']) != ['head']:
                    raise AgentError('CAMERA_NOT_SUPPORTED', '当前仅提供头部相机外参')
                result = actions.geometry.extrinsics()
            else:
                raise AgentError('NOT_FOUND', '接口不存在', 404)
            self._send(200, result)
        except Exception as exc:
            actions.service.audit_event('agent_get_failed', path=path, error=str(exc))
            self._send(*actions.error(exc))

    def do_POST(self):
        actions = self.server.actions
        path = urlparse(self.path).path
        request_id = uuid.uuid4().hex
        token = context.set(dict(request_id=request_id, operation_id=request_id))
        try:
            if not path.startswith(self.server.prefix + '/'):
                raise AgentError('NOT_FOUND', '接口不存在', 404)
            if self.headers.get('Transfer-Encoding'):
                raise AgentError('INVALID_BODY', '不支持分块请求体')
            size = int(self.headers.get('Content-Length', '0'))
            if not 0 < size <= 4*1024*1024:
                raise AgentError('INVALID_BODY', '请求体必须为 JSON，最大 4 MiB')
            payload = json.loads(self.rfile.read(size), parse_constant=lambda _: (_ for _ in ()).throw(ValueError('非有限数值')))
            if not isinstance(payload, dict):
                raise AgentError('INVALID_BODY', '请求体必须为 JSON 对象')
            status, result = actions.run(path, payload, self.headers.get('Idempotency-Key'))
            self._send(status, result)
        except (BrokenPipeError, ConnectionResetError):
            # Disconnect does not resend or roll back completed physical actions.
            pass
        except Exception as exc:
            actions.service.audit_event('agent_http_failed', path=path, error=str(exc))
            self._send(*actions.error(exc))
        finally:
            context.reset(token)


def make_agent_server(host, port, prefix, actions):
    server = ThreadingHTTPServer((host, port), AgentHandler)
    server.actions, server.prefix = actions, prefix
    return server
