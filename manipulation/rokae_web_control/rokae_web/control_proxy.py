"""Restartable web front end; control and action lifetimes belong to the core."""
import http.client
import json
import os
import socket
from pathlib import Path
from socketserver import ThreadingUnixStreamServer
from .web import ControlRequestHandler, ControlHTTPServer


def socket_path(config):
    return str(Path(config['hardware_service']['socket_path']).parent / 'control.sock')


class UnixHTTPConnection(http.client.HTTPConnection):
    def __init__(self, path):
        super().__init__('localhost', timeout=3600)
        self.path = str(path)

    def connect(self):
        self.sock = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
        self.sock.settimeout(self.timeout)
        self.sock.connect(self.path)


class CoreRequestHandler(ControlRequestHandler):
    def do_GET(self):
        if self.path.split('?', 1)[0] == '/api/ready':
            # Startup readiness only: construction and initial SDK handshake
            # already succeeded. Do not acquire the motion lock or query ROS,
            # cameras, gripper, suction, or full robot status here.
            return self._send_json(200, {'ok': True, 'ready': True})
        return super().do_GET()


class CoreWebServer(ThreadingUnixStreamServer):
    daemon_threads = True

    def __init__(self, path, service, static_dir):
        self.service, self.static_dir = service, Path(static_dir).resolve()
        Path(path).parent.mkdir(parents=True, exist_ok=True)
        # Caller holds a process lock before removing its stale socket.
        Path(path).unlink(missing_ok=True)
        super().__init__(str(path), CoreRequestHandler)
        os.chmod(path, 0o600)

    def get_request(self):
        sock, _ = super().get_request()
        return sock, ('local-web', 0)


class ProxyHandler(ControlRequestHandler):
    def _proxy(self):
        connection = UnixHTTPConnection(self.server.core_socket)
        try:
            size = int(self.headers.get('Content-Length', '0'))
            if not 0 <= size <= 65536 or self.headers.get('Transfer-Encoding'):
                raise ValueError('请求体过大或编码不支持')
            body = self.rfile.read(size) if size else None
            headers = {k:v for k,v in self.headers.items()
                       if k.lower() not in ('host','connection','transfer-encoding','content-length')}
            connection.request(self.command, self.path, body=body, headers=headers)
            response = connection.getresponse()
            data = response.read()
            self.send_response(response.status)
            for key, value in response.getheaders():
                if key.lower() not in ('server','date','connection','transfer-encoding','content-length'):
                    self.send_header(key, value)
            self.send_header('Content-Length', str(len(data)))
            self.end_headers()
            self.wfile.write(data)
        except (BrokenPipeError, ConnectionResetError):
            pass
        except Exception as exc:
            self._send_json(503, dict(ok=False, error='常驻运控服务不可用；请求不会自动重发：'+str(exc)))
        finally:
            connection.close()

    def do_GET(self):
        if self.path.startswith('/api/'):
            return self._proxy()
        return self._serve_static(self.path.split('?',1)[0])

    def do_POST(self):
        return self._proxy()


def make_proxy(host, port, path, static_dir):
    server = ControlHTTPServer((host, port), None, static_dir)
    server.RequestHandlerClass, server.core_socket = ProxyHandler, path
    return server
