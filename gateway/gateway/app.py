"""Gateway 进程：仅 MQTT 平台接入。"""

from __future__ import annotations

import argparse
import logging
import signal
import threading
from typing import Any, Dict, Optional

from gateway.config import load_config
from gateway.debug_http import (
    OsdTaskHttpServer,
    first_free_port,
    listen_ports_from_config,
)
from gateway.dispatcher import CommandDispatcher
from gateway.manipulator_port import ManipulatorStateReader
from gateway.map_sync import MapSyncService
from gateway.media_push import MediaPushController
from gateway.mqtt.client import MQTTClient
from gateway.mqtt.command_adapter import MqttCommandAdapter
from gateway.mqtt.osd import OsdReporter, StateCollector
from gateway.mqtt.trajectory import TrajectoryReporter
from gateway.mqtt.uplink import UplinkPublisher
from gateway.scenario_client import ScenarioClient


class GatewayApp:
    def __init__(self, config: Dict[str, Any]) -> None:
        self.config = config
        timeout = float(config.get("http_timeout_sec") or 8.0)
        poll_timeout = float(config.get("state_poll_timeout_sec") or 0.8)
        self.scenario_client = ScenarioClient(
            timeout_sec=timeout, poll_timeout_sec=poll_timeout
        )
        self.dispatcher = CommandDispatcher(
            config, scenario_client=self.scenario_client
        )
        self.command_log = self.dispatcher.command_log
        self.task_state = self.dispatcher.task_state
        self.collector = StateCollector(
            config,
            scenario_client=self.scenario_client,
            task_state=self.task_state,
        )
        self.manipulator = ManipulatorStateReader(config)
        self.collector.manipulator = self.manipulator
        self.map_sync = MapSyncService(config, collector=self.collector)
        self.collector.map_sync = self.map_sync
        self.media_push = MediaPushController(
            config, scenario_client=self.scenario_client
        )
        self.mqtt_client = MQTTClient(config)
        adapter = MqttCommandAdapter(self.dispatcher)
        for method, handler in adapter.handlers().items():
            self.mqtt_client.register_handler(method, handler)
        self.uplink = UplinkPublisher(
            self.mqtt_client, task_state=self.task_state
        )
        self.mqtt_client.add_connect_callback(self.uplink.flush_pending)
        self.mqtt_client.add_connect_callback(self._retry_map_sync_on_mqtt)
        self.osd_reporter = OsdReporter(
            config=config,
            mqtt_client=self.mqtt_client,
            collector=self.collector,
        )
        self.trajectory_reporter = TrajectoryReporter(
            config=config,
            mqtt_client=self.mqtt_client,
            collector=self.collector,
        )
        debug_http = config.get("debug_http") or {}
        self._osd_http: Optional[OsdTaskHttpServer] = None
        if bool(debug_http.get("enabled", False)):
            host = str(debug_http.get("host") or "0.0.0.0")
            try:
                port = first_free_port(host, listen_ports_from_config(debug_http))
            except OSError as exc:
                logging.getLogger(__name__).error(
                    "debug HTTP disabled, no free port: %s", exc
                )
            else:
                self._osd_http = OsdTaskHttpServer(self, host=host, port=port)
                logging.getLogger(__name__).info(
                    "debug HTTP enabled: http://%s:%s/osd/task /events /results /map/sync /media/push",
                    host,
                    port,
                )
        else:
            logging.getLogger(__name__).info("OSD task HTTP disabled")
        device = config.get("device") or {}
        logging.getLogger(__name__).info(
            "gateway assembled: sn=%s methods=%s scenarios=%s",
            device.get("sn") or "",
            ",".join(self.dispatcher.methods),
            ",".join(self.dispatcher.scenario_names) or "(none)",
        )

    @property
    def mqtt_connected(self) -> bool:
        return bool(getattr(self.mqtt_client, "is_connected", False))

    def start(self) -> None:
        log = logging.getLogger(__name__)
        log.info(
            "gateway MQTT starting: sn=%s broker=%s:%s client_id=%s",
            self.mqtt_client.device_sn,
            self.mqtt_client.broker_host,
            self.mqtt_client.broker_port,
            self.mqtt_client.client_id,
        )
        self.mqtt_client.start()
        self.manipulator.start()
        self.osd_reporter.start()
        self.trajectory_reporter.start()
        if self._osd_http is not None:
            self._osd_http.start()
        self.media_push.on_gateway_start()
        self.map_sync.on_gateway_start()
        log.info("gateway started")

    def _retry_map_sync_on_mqtt(self) -> None:
        thread = threading.Thread(
            target=self.map_sync.on_platform_ready,
            name="map-sync-mqtt-retry",
            daemon=True,
        )
        thread.start()

    def stop(self) -> None:
        logging.getLogger(__name__).info("gateway stopping")
        if self._osd_http is not None:
            self._osd_http.stop()
        self.trajectory_reporter.stop()
        self.osd_reporter.stop()
        self.manipulator.stop()
        self.mqtt_client.stop()
        logging.getLogger(__name__).info("gateway stopped")

    def serve_forever(self) -> None:
        self.start()
        stop = threading.Event()

        def _handle_stop(signum, frame):
            del frame
            logging.getLogger(__name__).info("received signal %s, shutting down", signum)
            stop.set()

        signal.signal(signal.SIGINT, _handle_stop)
        signal.signal(signal.SIGTERM, _handle_stop)
        if hasattr(signal, "SIGHUP"):
            signal.signal(signal.SIGHUP, signal.SIG_IGN)
        try:
            while not stop.is_set():
                stop.wait(0.5)
        finally:
            self.stop()


def main(argv: Optional[list[str]] = None) -> int:
    parser = argparse.ArgumentParser(description="Edge Gateway (MQTT)")
    parser.add_argument("--config", default="", help="gateway JSON config path")
    parser.add_argument(
        "--log-level",
        default="INFO",
        help="DEBUG / INFO / WARNING / ERROR",
    )
    args = parser.parse_args(argv)
    level_name = str(args.log_level or "INFO").upper()
    level = getattr(logging, level_name, logging.INFO)
    logging.basicConfig(
        level=level,
        format="%(asctime)s %(levelname)s %(name)s: %(message)s",
    )
    logging.getLogger(__name__).info("gateway process starting")
    config = load_config(args.config or None)
    GatewayApp(config).serve_forever()
    return 0
