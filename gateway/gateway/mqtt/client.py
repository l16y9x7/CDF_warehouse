"""MQTT 传输客户端：连接、收命令、回执和遥测发布。协议对齐 dog_device-SMT。"""

from __future__ import annotations

import inspect
import json
import logging
import time
import threading
from concurrent.futures import ThreadPoolExecutor
from typing import Any, Callable, Dict, Optional

from gateway.logging_utils import log_safe
from gateway.mqtt.envelopes import (
    normalize_service_result,
    normalize_task_event,
    result_succeeded,
    result_summary,
)

try:
    import paho.mqtt.client as mqtt
except ImportError as exc:
    raise ImportError("Please install paho-mqtt: pip install paho-mqtt") from exc


class MQTTClient:
    """持有唯一 MQTT 网络循环，并隔离传输重连与命令 handler。"""

    def __init__(
        self,
        config: Dict[str, Any],
        *,
        subscribe_services: bool = True,
    ):
        if not isinstance(subscribe_services, bool):
            raise ValueError("subscribe_services must be a boolean")
        self.config = config
        self.subscribe_services = subscribe_services
        device = config.get("device") or {}
        mqtt_cfg = config.get("mqtt") or {}
        topics = mqtt_cfg.get("topics") or {}

        self.device_sn = str(device.get("sn") or "")
        self.broker_host = str(mqtt_cfg.get("broker_host") or "")
        self.broker_port = int(mqtt_cfg.get("broker_port") or 1883)
        self.username = str(mqtt_cfg.get("username") or "")
        self.password = str(mqtt_cfg.get("password") or "")
        self.logger = logging.getLogger(__name__)
        configured_client_id = str(mqtt_cfg.get("client_id") or "").strip()
        self.client_id = configured_client_id or f"gateway_{self.device_sn}"
        if configured_client_id and self.device_sn and self.device_sn not in configured_client_id:
            self.logger.warning(
                "mqtt.client_id=%r does not contain device.sn=%r; using configured client_id",
                configured_client_id,
                self.device_sn,
            )
        self.reconnect_initial_sec = _positive_int_setting(
            mqtt_cfg, "reconnect_initial_sec", default=1
        )
        self.reconnect_max_sec = _positive_int_setting(
            mqtt_cfg, "reconnect_max_sec", default=60
        )
        if self.reconnect_max_sec < self.reconnect_initial_sec:
            raise ValueError(
                "mqtt.reconnect_max_sec must be >= mqtt.reconnect_initial_sec"
            )
        self.publish_ack_timeout_sec = _positive_float_setting(
            mqtt_cfg, "publish_ack_timeout_sec", default=3.0
        )

        self.topic_services = topics.get("services", "").format(sn=self.device_sn)
        self.topic_services_reply = topics.get("services_reply", "").format(
            sn=self.device_sn
        )
        self.topic_events = topics.get("events", "").format(sn=self.device_sn)
        self.topic_osd = topics.get("osd", "").format(sn=self.device_sn)
        self.topic_trajectory = topics.get("trajectory", "").format(
            sn=self.device_sn
        )

        self.handlers: Dict[str, Callable[[Any], Dict[str, Any]]] = {}
        self._connect_callbacks: list[Callable[[], None]] = []
        self.running = False
        self.is_connected = False
        self._loop_started = False
        self._executor_shutdown = False
        self._lifecycle_lock = threading.Lock()
        self._connection_state_lock = threading.Lock()
        self._connect_failure_count = 0
        self._offline_publish_topics: set[str] = set()
        self.command_dedupe_ttl_sec = float(
            mqtt_cfg.get("command_dedupe_ttl_sec", 300)
        )
        self.command_dedupe_max_entries = int(
            mqtt_cfg.get("command_dedupe_max_entries", 2048)
        )
        self._command_cache_lock = threading.Lock()
        self._command_cache: Dict[str, Dict[str, Any]] = {}
        self.executor: Optional[ThreadPoolExecutor] = None
        if self.subscribe_services:
            self.executor = ThreadPoolExecutor(
                max_workers=int(mqtt_cfg.get("command_worker_count") or 4),
                thread_name_prefix="mqtt-cmd",
            )

        callback_api = getattr(mqtt, "CallbackAPIVersion", None)
        if callback_api is None:
            self.client = mqtt.Client(client_id=self.client_id)
        else:
            self.client = mqtt.Client(
                callback_api_version=callback_api.VERSION2,
                client_id=self.client_id,
            )
        if self.username:
            self.client.username_pw_set(self.username, self.password)
        self.client.on_connect = self._on_connect
        self.client.on_connect_fail = self._on_connect_fail
        self.client.on_disconnect = self._on_disconnect
        self.client.on_message = (
            self._on_message if self.subscribe_services else None
        )
        self.logger.info("MQTT topic configuration:")
        if self.subscribe_services:
            self.logger.info("   - services subscribe: %s", self.topic_services)
        else:
            self.logger.info("   - services subscribe: disabled (outbound-only)")
        self.logger.info("   - services reply: %s", self.topic_services_reply)
        self.logger.info("   - events publish: %s", self.topic_events)
        self.logger.info("   - OSD publish: %s", self.topic_osd)
        self.logger.info("   - trajectory publish: %s", self.topic_trajectory)

    def register_handler(self, method: str, handler: Callable[[Any], Dict[str, Any]]):
        if not self.subscribe_services:
            raise RuntimeError("inbound MQTT services are disabled")
        self.handlers[method] = handler
        self.logger.info("registered MQTT handler: %s", method)

    def add_connect_callback(self, callback: Callable[[], None]) -> None:
        if callback not in self._connect_callbacks:
            self._connect_callbacks.append(callback)

    def start(self) -> None:
        with self._lifecycle_lock:
            if self.running:
                self.logger.info("MQTT network loop is already running")
                return
            if self._executor_shutdown:
                raise RuntimeError("MQTT client cannot restart after stop")
            self.logger.info(
                "starting MQTT network loop: broker=%s:%s client_id=%s "
                "reconnect_backoff_sec=%s..%s",
                self.broker_host,
                self.broker_port,
                self.client_id,
                self.reconnect_initial_sec,
                self.reconnect_max_sec,
            )
            try:
                self.client.reconnect_delay_set(
                    min_delay=self.reconnect_initial_sec,
                    max_delay=self.reconnect_max_sec,
                )
                self.client.connect_async(
                    self.broker_host,
                    self.broker_port,
                    keepalive=60,
                )
                self.running = True
                loop_rc = self.client.loop_start()
            except Exception:
                self.running = False
                self.logger.exception("failed to start MQTT network loop")
                raise

            if _reason_code_value(loop_rc) != _mqtt_success_code():
                self.running = False
                raise RuntimeError(
                    f"failed to start MQTT network loop: rc={loop_rc}"
                )
            self._loop_started = True

    def stop(self) -> None:
        with self._lifecycle_lock:
            self.running = False
            loop_started = self._loop_started
            self._loop_started = False
            should_shutdown_executor = (
                not self._executor_shutdown and self.executor is not None
            )
            self._executor_shutdown = True
        try:
            if loop_started:
                try:
                    self.client.disconnect()
                finally:
                    self.client.loop_stop()
        finally:
            self.is_connected = False
            if should_shutdown_executor and self.executor is not None:
                self.executor.shutdown(wait=False)

    def publish(self, topic: str, payload: Dict[str, Any], qos: int = 1) -> bool:
        if not self.is_connected:
            with self._connection_state_lock:
                first_skip = topic not in self._offline_publish_topics
                self._offline_publish_topics.add(topic)
            if first_skip:
                self.logger.warning(
                    "MQTT not connected; suppressing publishes until recovery: topic=%s",
                    topic,
                )
            return False
        try:
            result = self.client.publish(
                topic,
                json.dumps(
                    payload,
                    ensure_ascii=False,
                    allow_nan=False,
                    separators=(",", ":"),
                ),
                qos=qos,
            )
            rc = _reason_code_value(getattr(result, "rc", -1))
            if rc != _mqtt_success_code():
                self.logger.warning(
                    "MQTT publish rejected: topic=%s qos=%s rc=%s",
                    topic,
                    qos,
                    rc,
                )
                return False
            if qos > 0:
                result.wait_for_publish(timeout=self.publish_ack_timeout_sec)
                if not result.is_published():
                    self.logger.warning(
                        "MQTT publish confirmation timed out: topic=%s qos=%s "
                        "timeout_sec=%s",
                        topic,
                        qos,
                        self.publish_ack_timeout_sec,
                    )
                    return False
        except Exception:
            self.logger.warning(
                "MQTT publish failed: topic=%s qos=%s",
                topic,
                qos,
                exc_info=True,
            )
            return False
        return True

    def send_service_reply(self, bid: str, tid: str, result: Dict[str, Any]) -> None:
        normalized = normalize_service_result(result)
        reply = {
            "bid": bid,
            "tid": tid,
            "code": normalized["code"],
            "message": normalized["message"],
            "timestamp": int(time.time() * 1000),
            "data": normalized["data"],
        }
        self.publish(self.topic_services_reply, reply, qos=1)

    def publish_event(self, event: str, data: Dict[str, Any]) -> bool:
        event_name = str(event or "").strip()
        if event_name != "task_event":
            raise ValueError("only task_event is supported")
        event_data = normalize_task_event(data)
        return self.publish(
            self.topic_events,
            {
                "tid": "",
                "bid": "",
                "timestamp": int(time.time() * 1000),
                "sn": self.device_sn,
                "data": event_data,
            },
            qos=1,
        )

    def publish_osd(self, osd_data: Dict[str, Any]) -> bool:
        return self.publish(self.topic_osd, osd_data, qos=0)

    def publish_trajectory(self, trajectory_data: Dict[str, Any]) -> bool:
        if not self.topic_trajectory:
            self.logger.warning("trajectory topic is not configured; skip publish")
            return False
        return self.publish(self.topic_trajectory, trajectory_data, qos=0)

    def _on_connect(self, client, userdata, flags, reason_code, properties=None):
        code = _reason_code_value(reason_code)
        self.is_connected = code == 0
        if self.is_connected:
            with self._connection_state_lock:
                recovered_topics = len(self._offline_publish_topics)
                self._offline_publish_topics.clear()
                failed_attempts = self._connect_failure_count
                self._connect_failure_count = 0
            self.logger.info(
                "MQTT connected: %s:%s", self.broker_host, self.broker_port
            )
            if self.subscribe_services:
                client.subscribe(self.topic_services, qos=1)
                self.logger.info(
                    "subscribed MQTT command topic: %s", self.topic_services
                )
            if failed_attempts or recovered_topics:
                self.logger.info(
                    "MQTT connection recovered: failed_attempts=%s resumed_topics=%s",
                    failed_attempts,
                    recovered_topics,
                )
            for callback in tuple(self._connect_callbacks):
                try:
                    callback()
                except Exception:
                    self.logger.warning("MQTT connect callback failed", exc_info=True)
        else:
            with self._connection_state_lock:
                self._connect_failure_count += 1
                attempt = self._connect_failure_count
            self.logger.error(
                "MQTT connection rejected: rc=%s attempt=%s "
                "reconnect_backoff_sec=%s..%s",
                code,
                attempt,
                self.reconnect_initial_sec,
                self.reconnect_max_sec,
            )

    def _on_connect_fail(self, client, userdata) -> None:
        if not self.running:
            return
        with self._connection_state_lock:
            self._connect_failure_count += 1
            attempt = self._connect_failure_count
        self.logger.warning(
            "MQTT transport connection failed: attempt=%s "
            "reconnect_backoff_sec=%s..%s",
            attempt,
            self.reconnect_initial_sec,
            self.reconnect_max_sec,
        )

    def _on_disconnect(self, client, userdata, *callback_args):
        reason_code = (
            callback_args[1]
            if len(callback_args) >= 2
            else callback_args[0]
            if callback_args
            else 0
        )
        rc = _reason_code_value(reason_code)
        self.is_connected = False
        reason = _disconnect_reason(rc)
        self.logger.warning("MQTT disconnected: rc=%s reason=%s", rc, reason)
        if self.running:
            self.logger.info(
                "MQTT reconnect remains active: backoff_sec=%s..%s",
                self.reconnect_initial_sec,
                self.reconnect_max_sec,
            )
        if int(rc or 0) == 7:
            self.logger.warning(
                "duplicate client_id detected; check mqtt.client_id=%r",
                self.client_id,
            )

    def _on_message(self, client, userdata, msg):
        if not self.subscribe_services:
            return
        try:
            if msg.topic != self.topic_services:
                return
            message = json.loads(msg.payload.decode("utf-8"))
        except Exception as exc:
            self.logger.error("invalid MQTT message: %s", exc, exc_info=True)
            return

        method = str(message.get("method") or "")
        bid = str(message.get("bid") or "")
        tid = str(message.get("tid") or "")
        data = message.get("data", {})
        if isinstance(data, str):
            try:
                data = json.loads(data)
            except json.JSONDecodeError:
                data = {}

        handler = self.handlers.get(method)
        if handler is None:
            self.send_service_reply(
                bid,
                tid,
                {"result": -1, "error": f"Unknown method: {method}"},
            )
            return

        dedupe_key = self._build_command_dedupe_key(method=method, tid=tid, bid=bid)
        if dedupe_key:
            cached = self._check_duplicate_or_mark_processing(dedupe_key)
            if cached is not None:
                if cached.get("status") == "processing":
                    self.logger.warning(
                        "duplicate MQTT command is still processing: method=%s tid=%s bid=%s",
                        method,
                        tid,
                        bid,
                    )
                    self.send_service_reply(
                        bid,
                        tid,
                        {
                            "code": 0,
                            "message": "command is processing",
                            "data": {"result": 0, "processing": True},
                        },
                    )
                    return
                self.logger.warning(
                    "duplicate MQTT command hit cache: method=%s tid=%s bid=%s",
                    method,
                    tid,
                    bid,
                )
                self.send_service_reply(
                    bid,
                    tid,
                    cached.get(
                        "result",
                        {"result": -1, "error": "cached result missing"},
                    ),
                )
                return

        self.logger.info(
            "MQTT command received: method=%s bid=%s tid=%s data=%s",
            method,
            bid,
            tid,
            log_safe(data),
        )
        executor = self.executor
        if executor is None:
            self.logger.error("MQTT command executor is unavailable")
            return
        executor.submit(
            self._execute_handler,
            method,
            bid,
            tid,
            data,
            handler,
            dedupe_key,
        )

    def _execute_handler(
        self,
        method: str,
        bid: str,
        tid: str,
        data: Any,
        handler,
        dedupe_key: str = "",
    ):
        try:
            result = _invoke_command_handler(handler, data, bid=bid, tid=tid)
            if result is None:
                result = {"result": -1, "error": f"handler {method} returned None"}
        except Exception as exc:
            self.logger.error(
                "MQTT command handler failed: method=%s", method, exc_info=True
            )
            result = {"result": -1, "error": str(exc)}
        summary = result_summary(method, result)
        if result_succeeded(result):
            self.logger.info(
                "MQTT command succeeded: method=%s bid=%s tid=%s %s",
                method,
                bid,
                tid,
                summary,
            )
        else:
            self.logger.warning(
                "MQTT command failed: method=%s bid=%s tid=%s %s",
                method,
                bid,
                tid,
                summary,
            )
        self.send_service_reply(bid, tid, result)
        if dedupe_key:
            self._mark_command_done(dedupe_key, method, tid, bid, result)

    @staticmethod
    def _build_command_dedupe_key(method: str, tid: str, bid: str) -> str:
        tid_val = (tid or "").strip()
        bid_val = (bid or "").strip()
        if tid_val:
            return f"{method}|tid|{tid_val}"
        if bid_val:
            return f"{method}|bid|{bid_val}"
        return ""

    def _check_duplicate_or_mark_processing(
        self, dedupe_key: str
    ) -> Optional[Dict[str, Any]]:
        now_ts = time.time()
        with self._command_cache_lock:
            self._prune_command_cache_locked(now_ts)
            existing = self._command_cache.get(dedupe_key)
            if existing:
                existing["updated_at"] = now_ts
                return dict(existing)
            self._command_cache[dedupe_key] = {
                "status": "processing",
                "updated_at": now_ts,
                "result": None,
            }
            return None

    def _mark_command_done(
        self,
        dedupe_key: str,
        method: str,
        tid: str,
        bid: str,
        result: Dict[str, Any],
    ) -> None:
        now_ts = time.time()
        with self._command_cache_lock:
            self._command_cache[dedupe_key] = {
                "status": "done",
                "updated_at": now_ts,
                "method": method,
                "tid": tid,
                "bid": bid,
                "result": dict(result or {}),
            }
            self._prune_command_cache_locked(now_ts)

    def _prune_command_cache_locked(self, now_ts: float) -> None:
        ttl = max(1.0, float(self.command_dedupe_ttl_sec))
        expired_keys = [
            key
            for key, value in self._command_cache.items()
            if (now_ts - float(value.get("updated_at", now_ts))) > ttl
        ]
        for key in expired_keys:
            self._command_cache.pop(key, None)
        if len(self._command_cache) <= self.command_dedupe_max_entries:
            return
        sorted_items = sorted(
            self._command_cache.items(),
            key=lambda item: float(item[1].get("updated_at", 0.0)),
        )
        overflow = len(self._command_cache) - self.command_dedupe_max_entries
        for index in range(max(0, overflow)):
            self._command_cache.pop(sorted_items[index][0], None)


def _positive_int_setting(config: Dict[str, Any], key: str, *, default: int) -> int:
    raw_value = config.get(key, default)
    if isinstance(raw_value, bool):
        raise ValueError(f"mqtt.{key} must be a positive integer")
    try:
        value = int(raw_value)
    except (TypeError, ValueError) as exc:
        raise ValueError(f"mqtt.{key} must be a positive integer") from exc
    if value < 1:
        raise ValueError(f"mqtt.{key} must be a positive integer")
    return value


def _positive_float_setting(
    config: Dict[str, Any], key: str, *, default: float
) -> float:
    raw_value = config.get(key, default)
    if isinstance(raw_value, bool):
        raise ValueError(f"mqtt.{key} must be a positive number")
    try:
        value = float(raw_value)
    except (TypeError, ValueError) as exc:
        raise ValueError(f"mqtt.{key} must be a positive number") from exc
    if value <= 0:
        raise ValueError(f"mqtt.{key} must be a positive number")
    return value


def _mqtt_success_code() -> int:
    return _reason_code_value(getattr(mqtt, "MQTT_ERR_SUCCESS", 0))


def _disconnect_reason(rc: Any) -> str:
    reasons = {
        0: "normal disconnect",
        1: "incorrect protocol version",
        2: "invalid client identifier",
        3: "server unavailable",
        4: "bad username or password",
        5: "not authorized",
        7: "session taken over by duplicate client_id",
    }
    try:
        code = int(rc)
    except (TypeError, ValueError):
        return f"unknown({rc})"
    return reasons.get(code, f"unknown({code})")


def _invoke_command_handler(handler, data: Any, *, bid: str, tid: str) -> Any:
    try:
        parameters = inspect.signature(handler).parameters
    except (TypeError, ValueError):
        parameters = {}
    kwargs = {}
    if "bid" in parameters:
        kwargs["bid"] = bid
    if "tid" in parameters:
        kwargs["tid"] = tid
    return handler(data, **kwargs)


def _reason_code_value(reason_code: Any) -> int:
    value = getattr(reason_code, "value", reason_code)
    try:
        return int(value)
    except (TypeError, ValueError):
        return -1
