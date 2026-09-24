#!/usr/bin/env python3
from __future__ import annotations

import argparse
import signal
import threading
from pathlib import Path

from rokae_web.audit import JsonlAuditLogger
from rokae_web.backends import (
    MockChassisBackend,
    MockRobotBackend,
    Ros2ChassisBackend,
)
from rokae_web.camera_manager import DisabledCameraManager
from rokae_web.config import load_config
from rokae_web.ros_camera import RosCameraManager
from rokae_web.service import ControlService
from rokae_web.web import make_server
from rokae_web.sdk_broker import BrokerRobotBackend


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="ROKAE whole-body web controller")
    parser.add_argument("--config", default=None, help="JSON config path")
    parser.add_argument(
        "--hardware",
        action="store_true",
        help="Enable real SDK/ROS adapters. Without this flag the server is MOCK only.",
    )
    parser.add_argument("--host", default=None)
    parser.add_argument("--port", type=int, default=None)
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    root = Path(__file__).resolve().parent
    config_path = args.config or (root / "config.json" if (root / "config.json").exists() else None)
    config = load_config(config_path)
    host = args.host or config["web"]["host"]
    port = args.port or int(config["web"]["port"])
    display_host = config["web"].get("display_host") or host
    if display_host in ("0.0.0.0", "::"):
        display_host = "127.0.0.1"
    if args.hardware:
        from rokae_web.control_proxy import make_proxy, socket_path
        import subprocess
        server = make_proxy(host, port, socket_path(config), root / 'static')
        try:
            subprocess.run(['systemctl', '--user', 'start', 'mui-control.service'], check=True)
        except Exception:
            server.server_close()
            raise
        def stop_proxy(*_):
            threading.Thread(target=server.shutdown, daemon=True).start()
        signal.signal(signal.SIGINT, stop_proxy)
        signal.signal(signal.SIGTERM, stop_proxy)
        print(f'Web frontend: http://{display_host}:{port}/; persistent control core remains running', flush=True)
        try:
            server.serve_forever(poll_interval=.2)
        finally:
            server.server_close()
        return
    server = make_server(host, port, None, root / "static")
    if args.hardware:
        robot_backend = BrokerRobotBackend(config["xcore"], config["hardware_service"]["socket_path"])
        chassis_backend = Ros2ChassisBackend(config["chassis"])
        camera_backend = RosCameraManager(
            config["ros_camera"],
            config["camera"]["data_directory"],
            config["camera"]["jpeg_quality"],
        )
    else:
        robot_backend = MockRobotBackend()
        chassis_backend = MockChassisBackend()
        camera_backend = DisabledCameraManager()
    audit = JsonlAuditLogger(config["logging"]["directory"])
    audit.capture_assets(config)
    service = ControlService(
        config,
        robot_backend,
        chassis_backend,
        args.hardware,
        audit,
        camera_backend=camera_backend,
    )
    server.service = service

    def request_stop(*_: object) -> None:
        threading.Thread(target=server.shutdown, daemon=True).start()

    signal.signal(signal.SIGINT, request_stop)
    signal.signal(signal.SIGTERM, request_stop)
    mode = "HARDWARE（仍需网页解锁）" if args.hardware else "MOCK（绝不连接机器人）"
    print(f"ROKAE Web Control: {mode}")
    print(f"监听地址: {host}:{port}")
    print(f"访问链接: http://{display_host}:{port}/")
    print(f"运动日志: {config['logging']['directory']}")
    try:
        server.serve_forever(poll_interval=0.2)
    finally:
        service.close()
        server.server_close()


if __name__ == "__main__":
    main()
