#!/usr/bin/env python3
"""One shared control core; 8091 front end may start/stop independently."""
import argparse
import fcntl
import os
import signal
import threading
from pathlib import Path
from rokae_web.audit import JsonlAuditLogger
from rokae_web.backends import Ros2ChassisBackend, MockChassisBackend, MockRobotBackend
from rokae_web.config import load_config
from rokae_web.sdk_broker import BrokerRobotBackend
from rokae_web.http_camera import HttpCameraManager
from rokae_web.camera_manager import DisabledCameraManager
from rokae_web.service import ControlService
from rokae_web.agent_actions import AgentActions
from rokae_web.agent_http import make_agent_server
from rokae_web.control_proxy import CoreWebServer, socket_path


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--config', required=True)
    parser.add_argument('--mock', action='store_true')
    args = parser.parse_args()
    root = Path(__file__).resolve().parent
    config = load_config(args.config)
    path = socket_path(config)
    Path(path).parent.mkdir(parents=True, exist_ok=True)
    lock = open(path+'.lock', 'a')
    fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
    audit = JsonlAuditLogger(config['logging']['directory'])
    audit.capture_assets(config)
    robot = MockRobotBackend() if args.mock else BrokerRobotBackend(config['xcore'], config['hardware_service']['socket_path'])
    # Read-only handshake: do not steal the old direct web's SDK session.
    robot.read_memory_state()
    chassis = MockChassisBackend() if args.mock else Ros2ChassisBackend(config['chassis'])
    camera = DisabledCameraManager() if args.mock else HttpCameraManager(config['ros_camera'], config['camera']['data_directory'], config['camera']['jpeg_quality'])
    service = ControlService(config, robot, chassis, not args.mock, audit, camera_backend=camera)
    actions = AgentActions(service, root / '.run' / ('agent-mock' if args.mock else 'agent'))
    servers, started_servers = [], []
    try:
        servers.append(CoreWebServer(path, service, root/'static'))
        servers.append(make_agent_server('0.0.0.0', int(os.environ.get('AGENT_POSE_PORT',8082)), '/pose', actions))
        servers.append(make_agent_server('0.0.0.0', int(os.environ.get('AGENT_MANIPULATION_PORT',8086)), '/manipulation', actions))
        stop = threading.Event()
        signal.signal(signal.SIGINT, lambda *_: stop.set())
        signal.signal(signal.SIGTERM, lambda *_: stop.set())
        for server in servers:
            threading.Thread(target=server.serve_forever, daemon=True).start()
            started_servers.append(server)
        print('MUI shared core ready: pose=8082 manipulation=8086; web uses '+path, flush=True)
        stop.wait()
    finally:
        actions.cancelled.set()
        # Allow synchronous Agent trunk stages to observe cancellation and issue
        # their own native stop before the broker session is closed.
        if actions.active:
            import time
            deadline = time.monotonic()+5
            while actions.active and time.monotonic() < deadline:
                time.sleep(.05)
        service.close()
        for server in servers:
            if server in started_servers:
                server.shutdown()
            server.server_close()
        Path(path).unlink(missing_ok=True)


if __name__ == '__main__':
    main()
