#!/usr/bin/env python3
"""Wait for core startup readiness without polling hardware status."""
import argparse
import http.client
import json
from pathlib import Path
import socket
import sys
import time

from rokae_web.config import load_config


class CoreConnection(http.client.HTTPConnection):
    def __init__(self, path, timeout):
        super().__init__('localhost', timeout=timeout)
        self.path = str(path)

    def connect(self):
        self.sock = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
        self.sock.settimeout(self.timeout)
        self.sock.connect(self.path)


def wait_for_core(path, timeout=40.0):
    deadline = time.monotonic() + timeout
    error = 'core not ready'
    while time.monotonic() < deadline:
        connection = CoreConnection(path, min(2.0, max(.01, deadline-time.monotonic())))
        try:
            connection.request('GET', '/api/ready')
            response = connection.getresponse()
            data = json.loads(response.read())
            if not isinstance(data, dict):
                raise ValueError('core readiness must be a JSON object')
            if response.status == 200 and data.get('ok') is True and data.get('ready') is True:
                return
            error = f'HTTP {response.status}: {data.get("error", "core not ready")}'
        except (OSError, ValueError, http.client.HTTPException) as exc:
            error = str(exc)
        finally:
            connection.close()
        time.sleep(min(.2, max(0, deadline-time.monotonic())))
    raise RuntimeError('运控核心未就绪，网页未启动：' + error)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--config', required=True)
    parser.add_argument('--timeout', type=float, default=40.0)
    args = parser.parse_args()
    if not 0 < args.timeout <= 300:
        parser.error('--timeout must be between 0 and 300 seconds')
    config = load_config(args.config)
    path = Path(config['hardware_service']['socket_path']).parent / 'control.sock'
    try:
        wait_for_core(path, args.timeout)
    except RuntimeError as exc:
        print(str(exc), file=sys.stderr)
        return 1
    print('MUI control core ready; starting web frontend.', flush=True)
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
