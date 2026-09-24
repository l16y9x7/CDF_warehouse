#!/usr/bin/env python3
"""Persistent SDK owner and read-only telemetry; no web/planning process here."""
import argparse
import os
from pathlib import Path
import signal
import socket
import stat
import threading

from rokae_web.audit import JsonlAuditLogger
from rokae_web.config import load_config
from rokae_web.control_trace import fields
from rokae_web.hardware_owner import HardwareOwner
from rokae_web.sdk_broker import OwnedHardwareBackend, BrokerCore, BrokerServer
from rokae_web.manipulator_reader import ManipulatorReader
from rokae_web.manipulator_telemetry import ManipulatorTelemetry
from rokae_web.telemetry_http import TelemetryHTTPServer


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--config', required=True)
    args = parser.parse_args()
    config = load_config(args.config)
    settings = config['hardware_service']
    path = Path(settings['socket_path'])
    owner = HardwareOwner(path.with_name('sdk-owner.lock'))
    owner.acquire()
    path.parent.mkdir(parents=True, exist_ok=True, mode=0o700)
    if path.exists():
        if not stat.S_ISSOCK(path.stat().st_mode) or path.stat().st_uid != os.getuid():
            raise RuntimeError('Refusing to replace a non-owned Unix socket path')
        probe = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
        try:
            probe.connect(str(path))
        except ConnectionRefusedError:
            path.unlink()  # Only our stale socket, while holding the owner lock.
        else:
            raise RuntimeError('SDK socket already has a listener')
        finally:
            probe.close()
    backend = OwnedHardwareBackend(config['xcore'])
    audit = JsonlAuditLogger(str(Path(config['logging']['directory'])/'hardware'))
    def emit(event, **data):
        try:
            audit.record(event, **{**fields(), **data})
        except Exception as exc:
            print('Hardware audit error:', exc, flush=True)
    backend.audit_callback = emit
    core = BrokerCore(backend, emit)
    settings_state = config.get('manipulator_state', {})
    reader = ManipulatorReader(backend, settings_state, motion_pending=core.motion_pending)
    telemetry = ManipulatorTelemetry(reader, settings_state, config['telemetry'])
    broker = BrokerServer(path, core)
    public = TelemetryHTTPServer((settings['host'], int(settings['port'])), telemetry)
    stop = threading.Event()
    signal.signal(signal.SIGTERM, lambda *_: stop.set())
    signal.signal(signal.SIGINT, lambda *_: stop.set())
    threads = [threading.Thread(target=s.serve_forever, daemon=True) for s in (broker, public)]
    for t in threads:
        t.start()
    telemetry.start()
    emit('hardware_service_started', http_port=settings['port'], socket_path=str(path))
    print(f"SDK owner ready; read-only HTTP {settings['host']}:{settings['port']}; web is separate", flush=True)
    try:
        stop.wait()
    finally:
        core.stopping = True
        public.shutdown(); broker.shutdown()
        telemetry.close()
        if core.session is not None:
            core.release(core.session)
        backend.close()
        public.server_close(); broker.server_close()
        if path.exists():
            path.unlink()
        owner.close()
        emit('hardware_service_stopped')
        audit.close()


if __name__ == '__main__':
    main()
