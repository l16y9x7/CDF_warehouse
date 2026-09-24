"""天机厂家 HTTP Adapter：去站走 :8081，不解释业务站名。"""

from __future__ import annotations

import json
import logging
import threading
import time
from pathlib import Path
from typing import Any, Dict, Optional
from urllib.error import HTTPError, URLError
from urllib.request import Request, urlopen

from navigation.adapters.tianji_telemetry import (
    DEFAULT_BATTERY_CYCLE,
    osd_from_get_state,
    parse_get_state_content,
)

LOGGER = logging.getLogger(__name__)


def _http_json(
    url: str,
    *,
    method: str = "GET",
    body: Optional[Dict[str, Any]] = None,
    headers: Optional[Dict[str, str]] = None,
    timeout_sec: float = 8.0,
) -> tuple[int, Optional[Dict[str, Any]]]:
    payload = None
    req_headers = {"Accept": "application/json"}
    if headers:
        req_headers.update(headers)
    if body is not None:
        payload = json.dumps(body, ensure_ascii=False, separators=(",", ":")).encode("utf-8")
        req_headers["Content-Type"] = "application/json; charset=utf-8"
    request = Request(url, data=payload, method=method, headers=req_headers)
    try:
        with urlopen(request, timeout=timeout_sec) as response:
            raw = response.read()
            status = int(getattr(response, "status", 200) or 200)
    except HTTPError as exc:
        raw = exc.read() if exc.fp is not None else b""
        status = int(exc.code)
    except (URLError, TimeoutError, OSError) as exc:
        LOGGER.warning("tianji navigation unreachable: url=%s error=%s", url, exc)
        return 0, None
    parsed = _parse_json(raw)
    return status, parsed


def _parse_json(raw: bytes) -> Optional[Dict[str, Any]]:
    if not raw:
        return {}
    try:
        value = json.loads(raw.decode("utf-8", "replace"))
    except json.JSONDecodeError:
        return None
    return value if isinstance(value, dict) else None


def load_stations_yaml(path: str) -> tuple[str, list[Dict[str, Any]]]:
    file_path = Path(path)
    if not str(path or "").strip():
        return "", []
    if not file_path.is_file():
        LOGGER.warning("stations yaml missing: %s", file_path)
        return "", []
    try:
        import yaml
    except ImportError:
        LOGGER.warning("PyYAML missing; skip stations file %s", path)
        return "", []
    doc = yaml.safe_load(file_path.read_text(encoding="utf-8")) or {}
    if not isinstance(doc, dict):
        return "", []
    map_name = str(doc.get("map_name") or "")
    raw_stations = doc.get("stations") or {}
    rows: list[Dict[str, Any]] = []
    if isinstance(raw_stations, dict):
        for station_id, item in raw_stations.items():
            row: Dict[str, Any] = {"station_id": str(station_id)}
            if isinstance(item, dict):
                for key in ("x", "y", "yaw"):
                    if item.get(key) is not None:
                        row[key] = item[key]
            rows.append(row)
    return map_name, rows


class TianjiNavigationAdapter:
    def __init__(self, config: Dict[str, Any]) -> None:
        tianji = config.get("tianji") or {}
        self.base_url = str(tianji.get("base_url") or "http://127.0.0.1:8081").rstrip("/")
        self.health_path = str(tianji.get("health_path") or "/navigation/health")
        self.navigate_path = str(tianji.get("navigate_path") or "/navigation/navigate")
        self.health_timeout_sec = float(config.get("http_timeout_sec") or 8.0)
        self.navigate_timeout_sec = float(tianji.get("navigate_timeout_sec") or 180.0)
        self._lock = threading.Lock()
        self._position: Optional[Dict[str, float]] = None
        self._nav_state = ""
        self._station_id = ""
        self._vendor_error = ""
        configured_map = str(tianji.get("map_id") or "")
        yaml_map, stations = load_stations_yaml(str(tianji.get("stations_yaml") or ""))
        self._map_id = configured_map or yaml_map
        self._stations = stations
        self._station_ids = {row["station_id"] for row in stations}
        get_state = tianji.get("get_state") or {}
        if not isinstance(get_state, dict):
            get_state = {}
        self._get_state_enabled = bool(get_state) and get_state.get("enabled") is not False
        self._get_state_base = str(
            get_state.get("base_url") or "http://6.6.7.6:8080"
        ).rstrip("/")
        self._get_state_path = str(get_state.get("path") or "/api/AMR/GetState")
        self._get_state_robot_id = int(get_state.get("robot_id") or 1)
        self._get_state_timeout_sec = float(get_state.get("timeout_sec") or 1.0)
        self._get_state_coalesce_sec = max(
            float(get_state.get("coalesce_sec") or 0.2), 0.0
        )
        cycle_default = get_state.get("battery_cycle_default", DEFAULT_BATTERY_CYCLE)
        self._battery_cycle_default = (
            None if cycle_default is None else int(cycle_default)
        )
        self._get_state_cache: Optional[Dict[str, Any]] = None
        self._get_state_cached_at = 0.0
        self._last_get_state_warning = 0.0
        self._get_state_ok = False
        self._osd_logged = False
        self._last_unready_warning = 0.0
        LOGGER.info(
            "tianji adapter: nav=%s%s map_id=%s stations=%s get_state=%s",
            self.base_url,
            self.navigate_path,
            self._map_id or "-",
            len(self._stations),
            (
                f"{self._get_state_base}{self._get_state_path}"
                if self._get_state_enabled
                else "disabled"
            ),
        )

    def note_pose(self, x: float, y: float, yaw: float) -> None:
        with self._lock:
            self._position = {"x": float(x), "y": float(y), "yaw": float(yaw)}

    def note_status(self, *, nav_state: str = "", station_id: str = "", error_msg: str = "") -> None:
        with self._lock:
            if nav_state:
                self._nav_state = str(nav_state)
            if station_id:
                self._station_id = str(station_id)
            if error_msg:
                self._vendor_error = str(error_msg)

    def ready(self) -> bool:
        status, body = _http_json(
            f"{self.base_url}{self.health_path}",
            timeout_sec=self.health_timeout_sec,
        )
        ok = status == 200 and bool(body) and str(body.get("status") or "").upper() == "READY"
        if not ok:
            now = time.monotonic()
            if now - self._last_unready_warning > 30.0:
                LOGGER.warning(
                    "tianji navigation not READY: url=%s http=%s status=%s",
                    f"{self.base_url}{self.health_path}",
                    status,
                    (body or {}).get("status") if isinstance(body, dict) else None,
                )
                self._last_unready_warning = now
        return ok

    def snapshot(self) -> Dict[str, Any]:
        ready = self.ready()
        with self._lock:
            position = dict(self._position) if self._position else None
            nav_state = self._nav_state
            station_id = self._station_id
            vendor_error = self._vendor_error
            map_id = self._map_id
            stations = list(self._stations)
        out: Dict[str, Any] = {
            "ready": ready,
            "nav_state": nav_state or ("IDLE" if ready else "UNREADY"),
            "station_id": station_id,
            "map_id": map_id,
            "stations": stations,
        }
        if position:
            out["position"] = position
        if vendor_error:
            out["message"] = vendor_error
        out.update(self._osd_from_get_state())
        return out

    def _osd_from_get_state(self) -> Dict[str, Any]:
        if not self._get_state_enabled:
            return {}
        content = self._get_state_content()
        if not content:
            return {}
        fragment = osd_from_get_state(
            content, cycle_default=self._battery_cycle_default
        )
        if fragment and not self._osd_logged:
            battery = fragment.get("battery") if isinstance(fragment.get("battery"), dict) else {}
            chassis = (
                fragment.get("chassis_status")
                if isinstance(fragment.get("chassis_status"), dict)
                else {}
            )
            LOGGER.info(
                "tianji OSD live: battery=%s chassis=%s",
                battery.get("capacity_percent"),
                chassis.get("state") or "-",
            )
            self._osd_logged = True
        return fragment

    def _get_state_content(self) -> Dict[str, Any]:
        now = time.monotonic()
        with self._lock:
            cached = self._get_state_cache
            cached_at = self._get_state_cached_at
        if (
            cached is not None
            and self._get_state_coalesce_sec > 0
            and (now - cached_at) <= self._get_state_coalesce_sec
        ):
            return dict(cached)
        status, body = _http_json(
            f"{self._get_state_base}{self._get_state_path}",
            method="POST",
            body={"id": self._get_state_robot_id},
            timeout_sec=self._get_state_timeout_sec,
        )
        content = parse_get_state_content(body) if status == 200 else {}
        if content:
            recovered = not self._get_state_ok
            with self._lock:
                self._get_state_cache = dict(content)
                self._get_state_cached_at = time.monotonic()
                self._get_state_ok = True
            if recovered:
                LOGGER.info(
                    "tianji GetState ok: url=%s keys=%s",
                    f"{self._get_state_base}{self._get_state_path}",
                    sorted(content.keys()),
                )
            return content
        with self._lock:
            self._get_state_cache = None
            self._get_state_ok = False
            self._osd_logged = False
            if now - self._last_get_state_warning > 30.0:
                error_code = (
                    body.get("ErrorCode") if isinstance(body, dict) else None
                )
                LOGGER.warning(
                    "tianji GetState unavailable: url=%s http=%s ErrorCode=%s",
                    f"{self._get_state_base}{self._get_state_path}",
                    status,
                    error_code,
                )
                self._last_get_state_warning = now
        return {}

    def goto(
        self,
        *,
        station_id: str,
        idempotency_key: str,
        timeout_sec: float,
    ) -> Dict[str, Any]:
        if self._station_ids and station_id not in self._station_ids:
            LOGGER.warning(
                "goto rejected unknown station: station_id=%s known=%s",
                station_id,
                sorted(self._station_ids),
            )
            return {
                "terminal_state": "REJECTED",
                "error_code": "NAVIGATION_UNKNOWN_STATION",
                "message": f"unknown station_id={station_id}",
            }
        timeout = max(float(timeout_sec or 0), 1.0)
        timeout = min(timeout, self.navigate_timeout_sec)
        LOGGER.info(
            "goto vendor POST: url=%s target_id=%s timeout_sec=%s key=%s",
            f"{self.base_url}{self.navigate_path}",
            station_id,
            timeout,
            idempotency_key,
        )
        status, body = _http_json(
            f"{self.base_url}{self.navigate_path}",
            method="POST",
            body={"target_id": station_id},
            headers={"Idempotency-Key": idempotency_key},
            timeout_sec=timeout,
        )
        if body is None:
            LOGGER.error(
                "goto vendor unreachable: station_id=%s url=%s",
                station_id,
                f"{self.base_url}{self.navigate_path}",
            )
            return {
                "terminal_state": "FAILED",
                "error_code": "NAVIGATION_VENDOR_UNREACHABLE",
                "message": "tianji navigation HTTP unreachable",
            }
        vendor_status = str((body or {}).get("status") or "").upper()
        error_code = str((body or {}).get("error_code") or "")
        if status == 200 and vendor_status == "SUCCEEDED":
            with self._lock:
                self._station_id = station_id
                self._nav_state = "IDLE"
                self._vendor_error = ""
            LOGGER.info("goto vendor SUCCEEDED: station_id=%s", station_id)
            return {
                "terminal_state": "SUCCEEDED",
                "evidence": {"arrived": True, "station_id": station_id},
            }
        if status in {400, 409} or error_code in {
            "MISSING_IDEMPOTENCY_KEY",
            "INVALID_REQUEST",
            "RESOURCE_BUSY",
        }:
            mapped = "RESOURCE_BUSY" if error_code == "RESOURCE_BUSY" else "NAVIGATION_REJECTED"
            LOGGER.warning(
                "goto vendor REJECTED: station_id=%s http=%s error=%s",
                station_id,
                status,
                error_code or vendor_status,
            )
            return {
                "terminal_state": "REJECTED",
                "error_code": mapped,
                "message": error_code or f"tianji rejected status={status}",
            }
        if status == 503 or error_code == "MODULE_NOT_READY":
            LOGGER.warning(
                "goto vendor not ready: station_id=%s http=%s error=%s",
                station_id,
                status,
                error_code or vendor_status,
            )
            return {
                "terminal_state": "REJECTED",
                "error_code": "NAVIGATION_NOT_READY",
                "message": error_code or "tianji not ready",
            }
        LOGGER.error(
            "goto vendor FAILED: station_id=%s http=%s status=%s error=%s body=%s",
            station_id,
            status,
            vendor_status,
            error_code or "-",
            body,
        )
        return {
            "terminal_state": "FAILED",
            "error_code": "NAVIGATION_VENDOR",
            "message": error_code or vendor_status or f"tianji status={status}",
        }

    def stop(self) -> Dict[str, Any]:
        LOGGER.warning("stop unavailable on tianji HTTP adapter")
        return {
            "accepted": False,
            "error_code": "NAVIGATION_UNAVAILABLE",
            "message": "tianji HTTP has no stop; use ROS Cancel when ros.enabled",
        }

    def load_map(self, map_id: str) -> Dict[str, Any]:
        LOGGER.warning("vendor map switch unavailable on tianji HTTP adapter: map_id=%s", map_id)
        return {
            "accepted": False,
            "error_code": "NAVIGATION_UNAVAILABLE",
            "message": "tianji HTTP does not switch vendor maps; POST /load_map with unified stations",
        }

    def apply_map(self, map_id: str, stations: list[Dict[str, Any]]) -> Dict[str, Any]:
        from navigation.unified_map import to_public_stations

        rows = to_public_stations({"map_id": map_id, "stations": stations})
        if not rows:
            return {
                "accepted": False,
                "error_code": "NAVIGATION_INVALID_REQUEST",
                "message": "stations is empty",
            }
        with self._lock:
            self._map_id = str(map_id)
            self._stations = rows
            self._station_ids = {str(row.get("station_id") or "") for row in rows}
            self._station_ids.discard("")
        LOGGER.info("tianji unified map applied: map_id=%s stations=%s", map_id, [row.get("station_id") for row in rows])
        return {"accepted": True, "map_id": str(map_id)}
