"""从能力 / 场景权威状态组装 RobotOSD。Gateway 不编造位姿或任务终态。"""

from __future__ import annotations

import logging
import math
import threading
import time
import uuid
from typing import Any, Dict, Mapping, Optional

from gateway.host_status import HostStateReader
from gateway.map_sync import rewrite_platform_map_id
from gateway.state_sources import adapt_state_fragment, iter_state_sources

LOGGER = logging.getLogger(__name__)

_CORE_OSD_KEYS = (
    "device_sn",
    "alias",
    "battery",
    "position",
    "host_status",
    "chassis_status",
    "manipulator_status",
    "map",
    "navigation_status",
    "task",
    "self_check",
)

_OPTIONAL_OSD_KEYS = (
    "alarm_status",
    "devices_info",
    "collector",
    "transport",
    "robot_mode",
    "motors",
    "motor_status",
    "shaft_status",
    "odometry",
    "locomotion",
    "mainboard",
    "arm_action",
    "safety",
    "imu_status",
    "imu",
    "stm32_status",
    "sensors_extrinsic",
)

_OSD_DATA_KEYS = _CORE_OSD_KEYS + _OPTIONAL_OSD_KEYS

_BATTERY_OSD_KEYS = (
    "capacity_percent",
    "voltage",
    "temperature",
    "charging",
    "cycle",
)


class StateCollector:
    """拉取各进程 GET /state 与 GET /health 碎片并合并。"""

    def __init__(
        self,
        config: Mapping[str, Any],
        *,
        scenario_client,
        task_state,
    ) -> None:
        self._config = config
        self._client = scenario_client
        self._task_state = task_state
        self.map_sync = None
        state_cfg = config.get("state") if isinstance(config.get("state"), Mapping) else {}
        raw_ttl = state_cfg.get("cache_ttl_sec")
        raw_backoff = state_cfg.get("skip_backoff_sec")
        self._cache_ttl_sec = max(
            float(raw_ttl if raw_ttl is not None else 0.8), 0.0
        )
        self._skip_backoff_sec = max(
            float(raw_backoff if raw_backoff is not None else 5.0), 0.0
        )
        self._lock = threading.Lock()
        self._cache: Optional[Dict[str, Any]] = None
        self._cache_at = 0.0
        self._skip_until: Dict[str, float] = {}
        self._resolved_url: Dict[str, str] = {}
        self._source_diag: Dict[str, Dict[str, Any]] = {}
        self.manipulator = None

    def snapshot(self) -> Dict[str, Any]:
        now = time.monotonic()
        with self._lock:
            if (
                self._cache is not None
                and self._cache_ttl_sec > 0
                and now - self._cache_at < self._cache_ttl_sec
            ):
                merged = dict(self._cache)
            else:
                merged = self._snapshot_uncached()
                self._cache = merged
                self._cache_at = now
                merged = dict(merged)
        if self._task_state.forced or "task" not in merged:
            merged["task"] = self._task_state.snapshot()
        return merged

    def diagnostics(self) -> Dict[str, Any]:
        with self._lock:
            sources = {
                name: dict(item) for name, item in self._source_diag.items()
            }
        if not sources:
            return {}
        return {
            "name": "gateway_state_collector",
            "schema_version": "1.0.0",
            "sources": sources,
        }

    def _snapshot_uncached(self) -> Dict[str, Any]:
        fragments: list[Dict[str, Any]] = []
        health_ok = True
        messages: list[str] = []
        now = time.monotonic()

        for source in iter_state_sources(self._config):
            if not source.required:
                skip_until = self._skip_until.get(source.name, 0.0)
                if now < skip_until:
                    continue
            fragment, base = self._get_source_fragment(source)
            health = None
            health_url = source.health_url_for(base) if base else source.health_url
            need_health = bool(health_url) and (
                source.required or fragment is not None
            )
            if need_health and not (
                isinstance(fragment, Mapping)
                and isinstance(fragment.get("self_check"), Mapping)
            ):
                health = self._client.get_json(health_url)
            adapted = adapt_state_fragment(source, fragment)
            if adapted:
                if not source.required:
                    adapted.pop("self_check", None)
                fragments.append(adapted)
            ready = _health_ok(health, adapted or fragment)
            self._source_diag[source.name] = {
                "connected": fragment is not None,
                "fresh": bool(ready and fragment is not None),
                "url": base or source.url,
            }
            if source.required and not ready:
                health_ok = False
                messages.append(f"{source.name} not ready")
            if source.required:
                continue
            if fragment is None:
                self._skip_until[source.name] = now + self._skip_backoff_sec
            else:
                self._skip_until.pop(source.name, None)

        merged = _merge_fragments(fragments)
        manipulator = getattr(self, "manipulator", None)
        if manipulator is not None:
            try:
                blocks = manipulator.osd_blocks()
            except Exception:
                blocks = {}
            if blocks:
                merged = _merge_fragments([merged, blocks])
                self._source_diag["manipulator"] = {
                    "connected": True,
                    "fresh": bool(
                        blocks.get("manipulator_status") or blocks.get("arm_action")
                    ),
                    "url": getattr(manipulator, "source_url", None)
                    or "rokae_robot_state.get_state",
                }
        if "self_check" not in merged:
            merged["self_check"] = {
                "status": 0 if health_ok else 1,
                "message": "就绪" if health_ok else ("；".join(messages) or "未就绪"),
            }
        return merged

    def _get_source_fragment(
        self, source
    ) -> tuple[Optional[Dict[str, Any]], str]:
        ordered: list[str] = []
        cached = self._resolved_url.get(source.name, "")
        bases = source.base_urls()
        if cached in bases:
            ordered.append(cached)
        for base in bases:
            if base not in ordered:
                ordered.append(base)
        for base in ordered:
            fragment = self._client.get_json(source.state_url_for(base))
            if fragment is not None:
                self._resolved_url[source.name] = base
                return fragment, base
        self._resolved_url.pop(source.name, None)
        return None, ""


class OsdReporter:
    def __init__(
        self,
        *,
        config: Mapping[str, Any],
        mqtt_client: Any,
        collector: StateCollector,
        host_reader: Any = None,
    ) -> None:
        self.config = config
        self.mqtt_client = mqtt_client
        self.collector = collector
        self._host_reader = host_reader if host_reader is not None else HostStateReader()
        mqtt_cfg = config.get("mqtt") or {}
        self.report_interval = max(
            float(mqtt_cfg.get("osd_report_interval") or 1.0), 0.1
        )
        self._summary_log_every = max(int(mqtt_cfg.get("osd_summary_log_every") or 3), 1)
        self._stop_event = threading.Event()
        self._thread: Optional[threading.Thread] = None
        self._logger = logging.getLogger(__name__)
        self._last_mqtt_warning_time = 0.0
        self._report_count = 0
        self._last_published_task: Optional[tuple[str, int, bool]] = None

    def start(self) -> None:
        if self._thread is not None and self._thread.is_alive():
            return
        self._stop_event.clear()
        self._thread = threading.Thread(
            target=self._report_loop, name="osd-reporter", daemon=True
        )
        self._thread.start()
        self._logger.info(
            "OSD reporter started: interval=%ss topic=%s",
            self.report_interval,
            getattr(self.mqtt_client, "topic_osd", ""),
        )

    def stop(self) -> None:
        self._stop_event.set()
        if self._thread is not None:
            self._thread.join(timeout=2.0)
        self._thread = None
        self._logger.info("OSD reporter stopped")

    def report_now(self) -> bool:
        if not bool(getattr(self.mqtt_client, "is_connected", False)):
            now = time.time()
            if now - self._last_mqtt_warning_time > 30.0:
                self._logger.warning("MQTT not connected; skip OSD publish")
                self._last_mqtt_warning_time = now
            return False
        payload = self.collect_osd_data()
        map_sync = getattr(self.collector, "map_sync", None)
        if map_sync is not None:
            try:
                map_sync.ensure_active_map()
            except Exception:
                self._logger.warning("map sync for active OSD map failed", exc_info=True)
        try:
            success = bool(self.mqtt_client.publish_osd(payload))
        except Exception as exc:
            self._logger.warning("OSD publish failed: %s", exc, exc_info=True)
            return False
        if success:
            self._report_count += 1
            data = payload.get("data") if isinstance(payload.get("data"), dict) else {}
            task = data.get("task") if isinstance(data.get("task"), dict) else {}
            forced = bool(getattr(self.collector._task_state, "forced", False))
            self._log_task_if_changed(task, forced)
            if (
                self._report_count == 1
                or self._report_count % self._summary_log_every == 0
            ):
                self._logger.debug(
                    "OSD published: count=%s task_id=%s task_status=%s "
                    "forced=%s self_check=%s has_position=%s map=%s "
                    "battery=%s host_temp=%s chassis=%s",
                    self._report_count,
                    task.get("task_id") or "",
                    task.get("status"),
                    forced,
                    (data.get("self_check") or {}).get("status")
                    if isinstance(data.get("self_check"), dict)
                    else None,
                    "position" in data,
                    (data.get("map") or {}).get("map_id")
                    if isinstance(data.get("map"), dict)
                    else "",
                    (data.get("battery") or {}).get("capacity_percent")
                    if isinstance(data.get("battery"), dict)
                    else None,
                    (data.get("host_status") or {}).get("temperature_c")
                    if isinstance(data.get("host_status"), dict)
                    else None,
                    (data.get("chassis_status") or {}).get("state")
                    if isinstance(data.get("chassis_status"), dict)
                    else "",
                )
        return success

    def collect_osd_data(self, *, snapshot: Optional[Mapping[str, Any]] = None) -> Dict[str, Any]:
        now_ms = int(time.time() * 1000)
        device = self.config.get("device") or {}
        source = dict(snapshot or self.collector.snapshot())
        data: Dict[str, Any] = {
            "device_sn": str(device.get("sn") or source.get("device_sn") or ""),
        }
        alias = str(device.get("alias") or device.get("name") or "").strip()
        if alias:
            data["alias"] = alias

        position = _compact_pose(source.get("position"))
        if position:
            data["position"] = position
        battery = _battery_payload(source.get("battery"))
        if battery:
            data["battery"] = battery
        host_status = _host_status_payload(
            self._read_host_status(), source.get("host_status")
        )
        if host_status:
            data["host_status"] = host_status
        chassis = _chassis_payload(source.get("chassis_status"))
        if chassis:
            data["chassis_status"] = chassis
        manipulator = _compact_dict(source.get("manipulator_status"))
        if manipulator:
            data["manipulator_status"] = manipulator
        map_payload = _map_payload(
            source.get("map"),
            map_sync=getattr(self.collector, "map_sync", None),
        )
        if map_payload:
            data["map"] = map_payload
        navigation_status = _navigation_payload(source.get("navigation_status"))
        if navigation_status:
            data["navigation_status"] = navigation_status
        data["task"] = _task_payload(source.get("task"))
        data["self_check"] = _self_check(source.get("self_check"))
        for key in _OPTIONAL_OSD_KEYS:
            if key in {"collector", "transport", "alarm_status"}:
                continue
            block = _optional_block_payload(key, source.get(key))
            if block:
                data[key] = block
        _enrich_xcore_blocks(data, self.config)
        # 仅透传上游告警；不自动补空 {has_alarm:false}
        alarm = _alarm_payload(source.get("alarm_status"))
        if alarm and (
            alarm.get("alarms")
            or alarm.get("has_alarm") is True
            or any(k not in {"has_alarm", "alarms"} for k in alarm)
        ):
            data["alarm_status"] = alarm
        # collector：仅当上游已带该块时透传，不强制注入诊断
        if isinstance(source.get("collector"), Mapping) and source.get("collector"):
            collector = _collector_payload(
                source.get("collector"),
                self.collector,
                data,
            )
            if collector:
                data["collector"] = collector
        transport = _transport_payload(
            source.get("transport"), self.mqtt_client, self.config, data
        )
        if transport:
            data["transport"] = transport

        payload = {
            "tid": str(uuid.uuid4()),
            "timestamp": now_ms,
            "deviceType": str(device.get("deviceType") or "term").strip() or "term",
            "data": data,
        }
        robot_type = str(device.get("robotType") or "").strip()
        if robot_type:
            payload["robotType"] = robot_type
        return payload

    def _log_task_if_changed(self, task: Mapping[str, Any], forced: bool) -> None:
        current = (str(task.get("task_id") or ""), int(task.get("status") or 0), forced)
        previous = self._last_published_task
        if previous == current:
            return
        self._last_published_task = current
        if previous is None and current[1] == 0 and not current[2]:
            return
        if previous is None:
            self._logger.info(
                "OSD published task: task_id=%s status=%s forced=%s",
                current[0],
                current[1],
                current[2],
            )
            return
        self._logger.info(
            "OSD published task change: task_id=%s status=%s -> %s forced=%s",
            current[0] or previous[0],
            previous[1],
            current[1],
            current[2],
        )

    def _report_loop(self) -> None:
        while not self._stop_event.is_set():
            try:
                self.report_now()
            except Exception as exc:
                self._logger.warning("OSD report failed: %s", exc, exc_info=True)
            self._stop_event.wait(self.report_interval)

    def _read_host_status(self) -> Dict[str, Any]:
        reader = self._host_reader
        if reader is None:
            return {}
        try:
            status = reader.get_status()
        except Exception:
            return {}
        return dict(status) if isinstance(status, Mapping) else {}


def current_pose(
    snapshot: Mapping[str, Any],
    *,
    map_sync: Any = None,
) -> Optional[Dict[str, Any]]:
    position = snapshot.get("position")
    if not isinstance(position, Mapping):
        return None
    x = _finite_float(position.get("x"))
    y = _finite_float(position.get("y"))
    yaw = _finite_float(position.get("yaw", position.get("theta")))
    if x is None or y is None or yaw is None:
        return None
    map_id = ""
    raw_map = snapshot.get("map")
    if isinstance(raw_map, Mapping):
        map_id = _publishable_map_id(
            rewrite_platform_map_id(
                str(raw_map.get("map_id") or "").strip(),
                map_sync,
            )
        )
    return {"x": x, "y": y, "yaw": yaw, "map_id": map_id}


def _state_endpoints(config: Mapping[str, Any]) -> Dict[str, str]:
    return {source.name: source.url for source in iter_state_sources(config)}


def _health_ok(health: Any, fragment: Any) -> bool:
    if isinstance(health, Mapping):
        if health.get("ok") is False:
            return False
        if health.get("ready") is False:
            return False
    if isinstance(fragment, Mapping):
        check = fragment.get("self_check")
        if isinstance(check, Mapping) and int(check.get("status") or 0) == 1:
            return False
    if health is None and fragment is None:
        return False
    return True


def _merge_fragments(fragments: list[Dict[str, Any]]) -> Dict[str, Any]:
    merged: Dict[str, Any] = {}
    for fragment in fragments:
        for key in _OSD_DATA_KEYS:
            if key in fragment and fragment[key] not in (None, "", {}, []):
                merged[key] = fragment[key]
    return merged


def _compact_pose(value: Any) -> Dict[str, float]:
    if not isinstance(value, Mapping):
        return {}
    x = _finite_float(value.get("x"))
    y = _finite_float(value.get("y"))
    yaw = _finite_float(value.get("yaw", value.get("theta")))
    if x is None or y is None or yaw is None:
        return {}
    return {"x": x, "y": y, "yaw": yaw}


def _compact_dict(value: Any) -> Dict[str, Any]:
    return dict(value) if isinstance(value, Mapping) and value else {}


def _battery_payload(value: Any) -> Dict[str, Any]:
    if not isinstance(value, Mapping) or not value:
        return {}
    payload: Dict[str, Any] = {}
    for key in _BATTERY_OSD_KEYS:
        if key not in value:
            continue
        item = value[key]
        if item is None:
            continue
        payload[key] = item
    for key, item in value.items():
        if key in payload or key in _BATTERY_OSD_KEYS:
            continue
        if item in (None, "", {}, []):
            continue
        payload[key] = item
    if payload and "cycle" not in payload:
        payload["cycle"] = 0
    return payload


def _host_status_payload(local: Any, fragment: Any) -> Dict[str, Any]:
    source = local if isinstance(local, Mapping) and local else fragment
    if not isinstance(source, Mapping) or not source:
        return {}
    payload: Dict[str, Any] = {}
    runtime = source.get("runtime_sec")
    if isinstance(runtime, int) and not isinstance(runtime, bool) and runtime >= 0:
        payload["runtime_sec"] = runtime
    temperature = _finite_float(source.get("temperature_c"))
    if temperature is not None:
        payload["temperature_c"] = temperature
    return payload


def _chassis_payload(value: Any) -> Dict[str, Any]:
    """对外 OSD：state + motion。linear_speed_mps 无实测时用 vx/vy 合成。"""

    if not isinstance(value, Mapping) or not value:
        return {}
    result: Dict[str, Any] = {}
    state = value.get("state")
    if isinstance(state, str) and state.strip():
        result["state"] = state.strip().lower()
    motion_src = value.get("motion") if isinstance(value.get("motion"), Mapping) else {}
    motion: Dict[str, Any] = {}
    stopped = motion_src.get("stopped")
    if isinstance(stopped, bool):
        motion["stopped"] = stopped
    for key in ("linear_x_mps", "linear_y_mps", "angular_radps"):
        number = _finite_float(motion_src.get(key))
        if number is not None:
            motion[key] = number
    speed = motion_src.get("linear_speed_mps")
    if (
        isinstance(speed, (int, float))
        and not isinstance(speed, bool)
        and math.isfinite(float(speed))
        and float(speed) >= 0.0
    ):
        motion["linear_speed_mps"] = float(speed)
    else:
        vx = motion.get("linear_x_mps")
        vy = motion.get("linear_y_mps")
        if vx is not None and vy is not None:
            motion["linear_speed_mps"] = math.hypot(float(vx), float(vy))
        else:
            motion["linear_speed_mps"] = None
    if motion:
        result["motion"] = motion
    return result


def _map_payload(value: Any, *, map_sync: Any = None) -> Dict[str, Any]:
    if not isinstance(value, Mapping):
        return {}
    map_id = _publishable_map_id(
        rewrite_platform_map_id(str(value.get("map_id") or "").strip(), map_sync)
    )
    if not map_id:
        return {}
    payload: Dict[str, Any] = {"map_id": map_id}
    payload["stations"] = _stations_payload(value.get("stations"))
    nodes = _graph_rows(value.get("nodes"))
    edges = _graph_rows(value.get("edges"))
    if nodes:
        payload["nodes"] = nodes
    if edges:
        payload["edges"] = edges
    return payload


def _graph_rows(value: Any) -> list[Dict[str, Any]]:
    if not isinstance(value, list):
        return []
    return [dict(item) for item in value if isinstance(item, Mapping)]


def _stations_payload(value: Any) -> list[Dict[str, Any]]:
    if not isinstance(value, list):
        return []
    stations: list[Dict[str, Any]] = []
    for item in value:
        if not isinstance(item, Mapping):
            continue
        row = dict(item)
        station_id = str(row.get("station_id") or "").strip()
        name = str(row.get("name") or "").strip()
        if station_id and not name:
            row["name"] = station_id
        x = _finite_float(row.get("x"))
        y = _finite_float(row.get("y"))
        yaw = _finite_float(row.get("yaw", row.get("theta")))
        if x is not None:
            row["x"] = x
        if y is not None:
            row["y"] = y
        if yaw is not None:
            row["yaw"] = yaw
        stations.append(row)
    return stations


def _navigation_payload(value: Any) -> Dict[str, Any]:
    payload = _compact_dict(value)
    if not payload:
        return {}
    target = str(payload.get("target_station_id") or "").strip()
    if not target and str(payload.get("state") or "").upper() not in {"IDLE", "", "0"}:
        target = str(payload.get("station_id") or "").strip()
    payload["target_station_id"] = target
    return payload


def _task_payload(value: Any) -> Dict[str, Any]:
    if not isinstance(value, Mapping):
        return {
            "task_id": "",
            "status": 0,
            "current_tray_index": 0,
            "has_tray_in_hand": 0,
        }
    status = int(value.get("status") or 0)
    task_id = str(value.get("task_id") or "").strip() if status != 0 else ""
    payload: Dict[str, Any] = {
        "task_id": task_id,
        "status": status,
    }
    tray_index = value.get("current_tray_index")
    if isinstance(tray_index, int) and not isinstance(tray_index, bool) and tray_index >= 0:
        payload["current_tray_index"] = tray_index
    else:
        payload["current_tray_index"] = 0
    in_hand = value.get("has_tray_in_hand")
    if in_hand in (0, 1, True, False):
        payload["has_tray_in_hand"] = int(in_hand)
    else:
        payload["has_tray_in_hand"] = 0
    if status == 3:
        error = str(value.get("error") or "").strip()
        if error:
            payload["error"] = error[:300]
    return payload


def _self_check(value: Any) -> Dict[str, Any]:
    if isinstance(value, Mapping):
        status = 0 if int(value.get("status") or 0) == 0 else 1
        message = str(value.get("message") or ("就绪" if status == 0 else "未就绪"))
        return {"status": status, "message": message}
    return {"status": 1, "message": "未就绪"}


def _optional_block_payload(key: str, value: Any) -> Dict[str, Any]:
    if key == "alarm_status":
        return _alarm_payload(value)
    return _compact_dict(value)


def _alarm_payload(value: Any) -> Dict[str, Any]:
    if not isinstance(value, Mapping) or not value:
        return {}
    alarms = value.get("alarms")
    if isinstance(alarms, list) and not alarms:
        return {"has_alarm": False, "alarms": []}
    payload = dict(value)
    if "has_alarm" not in payload:
        payload["has_alarm"] = bool(alarms) if isinstance(alarms, list) else True
    return payload


def _collector_payload(
    raw: Any,
    collector: Any,
    data: Mapping[str, Any],
) -> Dict[str, Any]:
    payload = dict(raw) if isinstance(raw, Mapping) and raw else {}
    diag = getattr(collector, "diagnostics", None)
    extra = diag() if callable(diag) else {}
    if not isinstance(extra, Mapping):
        extra = {}
    sources: Dict[str, Any] = {}
    raw_sources = payload.get("sources")
    if isinstance(raw_sources, Mapping):
        sources.update(raw_sources)
    extra_sources = extra.get("sources")
    if isinstance(extra_sources, Mapping):
        for name, item in extra_sources.items():
            sources.setdefault(str(name), item)
    manipulator = data.get("manipulator_status")
    if isinstance(manipulator, Mapping):
        for name, arm in manipulator.items():
            if not isinstance(arm, Mapping):
                continue
            sources.setdefault(
                str(name),
                {
                    "connected": True,
                    "fresh": True,
                    "state": str(arm.get("state") or "idle"),
                },
            )
    action = data.get("arm_action") if isinstance(data.get("arm_action"), Mapping) else {}
    if action.get("body_joints_deg") or (action.get("endpoints") or {}).get("body"):
        sources.setdefault("body", {"connected": True, "fresh": True})
    if action.get("head_joints_deg") or (action.get("endpoints") or {}).get("head"):
        sources.setdefault("head", {"connected": True, "fresh": True})
    if not payload and not sources:
        return {}
    return {
        "name": str(payload.get("name") or extra.get("name") or "gateway_state_collector"),
        "schema_version": str(
            payload.get("schema_version") or extra.get("schema_version") or "1.0.0"
        ),
        "sources": sources,
    }


def _transport_payload(
    raw: Any, mqtt_client: Any, config: Mapping[str, Any], data: Mapping[str, Any]
) -> Dict[str, Any]:
    payload = dict(raw) if isinstance(raw, Mapping) and raw else {}
    connected = getattr(mqtt_client, "is_connected", None)
    if connected is None and not payload:
        has_site = bool(_site_xcore(config) or _site_sros(config))
        if not has_site:
            return {}
    if connected is not None:
        payload.setdefault("mqtt_connected", bool(connected))
    host = str(getattr(mqtt_client, "broker_host", "") or "").strip()
    if host:
        payload.setdefault("broker_host", host)
    client_id = str(getattr(mqtt_client, "client_id", "") or "").strip()
    if client_id:
        payload.setdefault("client_id", client_id)
    endpoints = dict(payload.get("endpoints") or {}) if isinstance(payload.get("endpoints"), Mapping) else {}
    sros = _site_sros(config)
    if sros.get("host"):
        chassis = dict(endpoints.get("chassis") or {})
        chassis.setdefault("protocol", str(sros.get("protocol") or "srp"))
        chassis.setdefault("host", sros["host"])
        if sros.get("port"):
            chassis.setdefault("port", sros["port"])
        chassis.setdefault(
            "connected",
            bool(data.get("battery") or data.get("chassis_status") or data.get("robot_mode")),
        )
        endpoints["chassis"] = chassis
    xcore = _site_xcore(config)
    action = data.get("arm_action") if isinstance(data.get("arm_action"), Mapping) else {}
    action_eps = action.get("endpoints") if isinstance(action.get("endpoints"), Mapping) else {}
    manipulator = data.get("manipulator_status") if isinstance(data.get("manipulator_status"), Mapping) else {}
    for name, spec in xcore.items():
        if name in {"sdk_root", "sdk_version", "local_ip"} or str(name).endswith("_ip"):
            continue
        host_ip = str(spec or "").strip()
        if not host_ip:
            continue
        item = dict(endpoints.get(name) or {})
        item.setdefault("protocol", "xcore")
        item.setdefault("host", host_ip)
        item.setdefault(
            "connected",
            bool(manipulator.get(name) or action_eps.get(name)),
        )
        endpoints[name] = item
    if endpoints:
        payload["endpoints"] = endpoints
    return payload


def _enrich_xcore_blocks(data: Dict[str, Any], config: Mapping[str, Any]) -> None:
    xcore = _site_xcore(config)
    if not xcore:
        return
    action = dict(data.get("arm_action") or {}) if isinstance(data.get("arm_action"), Mapping) else {}
    endpoints = dict(action.get("endpoints") or {}) if isinstance(action.get("endpoints"), Mapping) else {}
    manipulator = data.get("manipulator_status") if isinstance(data.get("manipulator_status"), Mapping) else {}
    changed = False
    if xcore.get("sdk_version"):
        action.setdefault("sdk_version", xcore["sdk_version"])
        changed = True
    for name, host_ip in xcore.items():
        if name in {"sdk_root", "sdk_version", "local_ip"} or str(name).endswith("_ip"):
            continue
        host_text = str(host_ip or "").strip()
        if not host_text:
            continue
        item = dict(endpoints.get(name) or {})
        item.setdefault("protocol", "xcore")
        item.setdefault("host", host_text)
        item.setdefault("connected", bool(manipulator.get(name) or item.get("joint_positions_deg")))
        endpoints[name] = item
        changed = True
    if endpoints:
        action["endpoints"] = endpoints
        changed = True
    if changed:
        data["arm_action"] = action


def _site_xcore(config: Mapping[str, Any]) -> Dict[str, Any]:
    site = config.get("site") if isinstance(config.get("site"), Mapping) else {}
    raw = site.get("xcore") if isinstance(site, Mapping) else None
    if not isinstance(raw, Mapping):
        state = config.get("state") if isinstance(config.get("state"), Mapping) else {}
        manipulator = state.get("manipulator") if isinstance(state.get("manipulator"), Mapping) else {}
        raw = manipulator.get("xcore") if isinstance(manipulator, Mapping) else {}
    if not isinstance(raw, Mapping):
        return {}
    mapped: Dict[str, Any] = dict(raw)
    if mapped.get("left_arm_ip") and "left_arm" not in mapped:
        mapped["left_arm"] = mapped.get("left_arm_ip")
    if mapped.get("right_arm_ip") and "right_arm" not in mapped:
        mapped["right_arm"] = mapped.get("right_arm_ip")
    if mapped.get("trunk_ip") and "body" not in mapped:
        mapped["body"] = mapped.get("trunk_ip")
    return mapped


def _site_sros(config: Mapping[str, Any]) -> Dict[str, Any]:
    site = config.get("site") if isinstance(config.get("site"), Mapping) else {}
    raw = site.get("sros") if isinstance(site, Mapping) else {}
    return dict(raw) if isinstance(raw, Mapping) else {}


def _publishable_map_id(map_id: str) -> str:
    text = str(map_id or "").strip()
    if not text or text.lower() == "no_map":
        return ""
    return text


def _finite_float(value: Any) -> Optional[float]:
    if isinstance(value, bool) or value is None:
        return None
    try:
        number = float(value)
    except (TypeError, ValueError):
        return None
    if not math.isfinite(number):
        return None
    return number
