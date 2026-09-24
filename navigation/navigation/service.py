"""能力层核心：校验字段、幂等、同时只跑一条 goto、后台等 Adapter。

不直接碰厂家 SDK。厂家差异全在 adapters/（当前中免现场用 rokae）。
/goto 立刻返回句柄；真正移动在 _run_goto 线程里。zhongmian 靠轮询 /status 等终态。
"""

from __future__ import annotations

import logging
import threading
import time
from typing import Any, Dict, Optional

from navigation.adapters.fake import FakeNavigationAdapter
from navigation.adapters.rokae import RokaeNavigationAdapter
from navigation.adapters.rokae_telemetry import OSD_CHASSIS_KEYS
from navigation.occupancy import maybe_occupancy
from navigation.adapters.tianji import TianjiNavigationAdapter
from navigation.rokae_runtime.matrix_import import blank_pgm, fms_gzip, import_fms
from navigation.unified_map import canonicalize, from_snapshot, maps_dir, read_map, write_map

LOGGER = logging.getLogger(__name__)

_REQUIRED_CONTEXT = ("task_id", "request_id", "timeout_sec", "idempotency_key")
_TERMINAL = {"SUCCEEDED", "FAILED", "CANCELLED", "TIMED_OUT", "REJECTED"}


class NavigationService:
    def __init__(self, config: Dict[str, Any], *, adapter: Any = None) -> None:
        self.config = config
        self.adapter = adapter or build_adapter(config)
        self._lock = threading.Lock()
        self._actions: Dict[str, Dict[str, Any]] = {}  # request_id → 这次 goto 的句柄
        self._by_idempotency: Dict[str, str] = {}  # idempotency_key → request_id
        self._current_id: Optional[str] = None  # 正在跑或刚跑完的那条，用于忙锁

    def health(self) -> Dict[str, Any]:
        return {"ok": bool(self.adapter.ready())}

    def state(self) -> Dict[str, Any]:
        snap = self.adapter.snapshot()
        ready = bool(snap.get("ready"))
        fragment: Dict[str, Any] = {
            "self_check": {
                "status": 0 if ready else 1,
                "message": "就绪" if ready else "导航未就绪",
            }
        }
        position = snap.get("position")
        if isinstance(position, dict) and position.get("x") is not None and position.get("y") is not None:
            fragment["position"] = {
                "x": float(position["x"]),
                "y": float(position["y"]),
                "yaw": float(position.get("yaw") or 0.0),
            }
        map_id = str(snap.get("map_id") or "")
        if map_id:
            fragment["map"] = _map_fragment(snap)
        nav_status: Dict[str, Any] = {
            "state": self._public_state(str(snap.get("nav_state") or "")),
        }
        if snap.get("station_id"):
            nav_status["station_id"] = str(snap["station_id"])
        with self._lock:
            current = self._actions.get(self._current_id or "")
        if current:
            nav_status["request_id"] = current["request_id"]
            nav_status["state"] = current["state"] if current["state"] != "ACCEPTED" else "RUNNING"
        fragment["navigation_status"] = nav_status
        for key in OSD_CHASSIS_KEYS:
            value = snap.get(key)
            if isinstance(value, dict) and value:
                fragment[key] = value
        return fragment

    def current_status(self) -> Dict[str, Any]:
        with self._lock:
            if self._current_id and self._current_id in self._actions:
                return dict(self._actions[self._current_id])
        snap = self.adapter.snapshot()
        body: Dict[str, Any] = {
            "state": self._public_state(str(snap.get("nav_state") or "IDLE")),
        }
        if snap.get("station_id"):
            body["station_id"] = str(snap["station_id"])
        if snap.get("position"):
            body["position"] = snap["position"]
        if snap.get("map_id"):
            body["map_id"] = snap["map_id"]
        return body

    def status_of(self, request_id: str) -> Dict[str, Any]:
        with self._lock:
            action = self._actions.get(request_id)
            if action is None:
                return {"accepted": False, "error_code": "NAVIGATION_UNKNOWN_REQUEST", "request_id": request_id}
            return dict(action)

    def map_info(self) -> Dict[str, Any]:
        return _map_fragment(self.adapter.snapshot())

    def refresh_stations(self) -> Dict[str, Any]:
        refresh = getattr(self.adapter, "refresh_stations", None)
        if not callable(refresh):
            return _rejected("NAVIGATION_UNAVAILABLE", "adapter cannot refresh stations")
        result = refresh()
        if result.get("accepted") is False:
            return {
                "accepted": False,
                "terminal_state": "REJECTED",
                "error_code": str(result.get("error_code") or "NAVIGATION_UNAVAILABLE"),
                "message": str(result.get("message") or "refresh stations failed"),
            }
        snap = self.adapter.snapshot()
        body = _map_fragment(snap)
        body["accepted"] = True
        body["map_id"] = str(result.get("map_id") or body.get("map_id") or "")
        if result.get("occupancy_revision"):
            body["occupancy_revision"] = str(result["occupancy_revision"])
        return body

    def goto(self, body: Dict[str, Any]) -> Dict[str, Any]:
        # 拒单也 HTTP 200，看 accepted=false。zhongmian 会把这当成失败。
        error = _validate_context(body)
        if error is not None:
            return error
        station_id = str(body.get("station_id") or "").strip()
        if not station_id:
            return _rejected("NAVIGATION_INVALID_REQUEST", "station_id is required")
        request_id = str(body["request_id"])
        idempotency_key = str(body["idempotency_key"])
        timeout_sec = float(body["timeout_sec"])

        if not self.adapter.ready():
            return _rejected("NAVIGATION_NOT_READY", "navigation adapter not ready")

        known = getattr(self.adapter, "known_station", None)
        if callable(known) and not known(station_id):
            snap = self.adapter.snapshot()
            map_id = str(snap.get("map_id") or "")
            LOGGER.warning("goto rejected unknown station: station_id=%s map=%s", station_id, map_id)
            return _rejected(
                "NAVIGATION_UNKNOWN_STATION",
                f"station_id={station_id} not on map {map_id}".strip(),
            )

        with self._lock:
            existing_id = self._by_idempotency.get(idempotency_key)
            if existing_id:
                existing = self._actions[existing_id]
                if existing["station_id"] != station_id:
                    return _rejected(
                        "NAVIGATION_IDEMPOTENCY_CONFLICT",
                        "idempotency_key reused with different station_id",
                    )
                # 同一把 key 再来：直接回上次句柄，不再发车。
                return dict(existing)
            if self._current_id and self._actions[self._current_id]["state"] not in _TERMINAL:
                return _rejected("RESOURCE_BUSY", "chassis is busy")
            action = {
                "accepted": True,
                "request_id": request_id,
                "task_id": str(body["task_id"]),
                "station_id": station_id,
                "idempotency_key": idempotency_key,
                "state": "ACCEPTED",
                "terminal_state": None,
                "error_code": "",
                "message": "",
                "evidence": None,
            }
            self._actions[request_id] = action
            self._by_idempotency[idempotency_key] = request_id
            self._current_id = request_id

        LOGGER.info(
            "goto accepted: request_id=%s station_id=%s task_id=%s",
            request_id,
            station_id,
            body.get("task_id"),
        )
        # HTTP 马上返回；Adapter.goto 会堵到车到站，所以丢后台线程。
        worker = threading.Thread(
            target=self._run_goto,
            args=(request_id, station_id, idempotency_key, timeout_sec),
            name=f"nav-goto-{request_id}",
            daemon=True,
        )
        worker.start()
        with self._lock:
            return dict(self._actions[request_id])

    def stop(self, body: Optional[Dict[str, Any]] = None) -> Dict[str, Any]:
        del body
        vendor = self.adapter.stop()
        if vendor.get("accepted") is False:
            LOGGER.warning(
                "stop not executed by adapter: error=%s message=%s",
                vendor.get("error_code"),
                vendor.get("message"),
            )
            return {
                "accepted": False,
                "terminal_state": "REJECTED",
                "error_code": str(vendor.get("error_code") or "NAVIGATION_UNAVAILABLE"),
                "message": str(vendor.get("message") or "stop unavailable"),
            }
        with self._lock:
            if self._current_id and self._actions[self._current_id]["state"] not in _TERMINAL:
                action = self._actions[self._current_id]
                action["state"] = "CANCELLED"
                action["terminal_state"] = "CANCELLED"
                action["error_code"] = str(vendor.get("error_code") or "")
                action["message"] = str(vendor.get("message") or "stop accepted")
                LOGGER.info("goto cancelled by stop: request_id=%s", self._current_id)
                return dict(action)
        return {
            "accepted": True,
            "terminal_state": vendor.get("terminal_state") or "CANCELLED",
            "error_code": vendor.get("error_code") or "",
            "message": vendor.get("message") or "no active goto",
        }

    def cancel(self, body: Dict[str, Any]) -> Dict[str, Any]:
        request_id = str(body.get("request_id") or "").strip()
        if not request_id:
            return _rejected("NAVIGATION_INVALID_REQUEST", "request_id is required")
        with self._lock:
            action = self._actions.get(request_id)
            if action is None:
                return _rejected("NAVIGATION_UNKNOWN_REQUEST", "unknown request_id")
            if action["state"] in _TERMINAL:
                return dict(action)
        return self.stop(body)

    def load_map(self, body: Dict[str, Any]) -> Dict[str, Any]:
        map_id = str(body.get("map_id") or "").strip()
        if not map_id:
            return _rejected("NAVIGATION_INVALID_REQUEST", "map_id is required")
        snap = self.adapter.snapshot()
        current_id = str(snap.get("map_id") or "")
        same_map = bool(current_id) and current_id == map_id
        directory = maps_dir(self.config)
        applied = same_map
        payload: Optional[Dict[str, Any]] = None
        try:
            if body.get("save_current"):
                payload = from_snapshot(snap, map_id=map_id)
                write_map(directory, payload)
            elif isinstance(body.get("stations"), list):
                payload = canonicalize(map_id, body["stations"])
                write_map(directory, payload)
            elif same_map:
                try:
                    payload = from_snapshot(snap, map_id=map_id)
                except ValueError:
                    payload = {"map_id": map_id, "stations": []}
            else:
                payload = read_map(directory, map_id)
        except FileNotFoundError:
            if same_map:
                try:
                    payload = from_snapshot(snap, map_id=map_id)
                except ValueError:
                    payload = {"map_id": map_id, "stations": []}
            else:
                result = self.adapter.load_map(map_id)
                if not result.get("accepted"):
                    return {
                        "accepted": False,
                        "terminal_state": "REJECTED",
                        "error_code": result.get("error_code") or "NAVIGATION_UNKNOWN_MAP",
                        "message": result.get("message") or f"unified map not found: {map_id}",
                    }
                snap = self.adapter.snapshot()
                return _load_map_ok(str(result.get("map_id") or map_id), snap, self.adapter)
        except ValueError as exc:
            return _rejected("NAVIGATION_INVALID_REQUEST", str(exc))
        if payload is None:
            return _rejected("NAVIGATION_UNKNOWN_MAP", f"unified map not found: {map_id}")
        if not same_map or body.get("save_current") or isinstance(body.get("stations"), list):
            apply = getattr(self.adapter, "apply_map", None)
            if not callable(apply):
                return _rejected("NAVIGATION_UNAVAILABLE", "adapter cannot apply unified map")
            result = apply(payload["map_id"], payload["stations"])
            if not result.get("accepted"):
                return {
                    "accepted": False,
                    "terminal_state": "REJECTED",
                    "error_code": result.get("error_code") or "NAVIGATION_UNAVAILABLE",
                    "message": result.get("message") or "apply unified map failed",
                }
            applied = True
            map_id = str(result.get("map_id") or payload["map_id"])
        if not applied:
            return _rejected("NAVIGATION_UNAVAILABLE", "adapter cannot apply unified map")
        snap = self.adapter.snapshot()
        return _load_map_ok(map_id, snap, self.adapter, stations=payload.get("stations"))

    def import_map(self, body: Dict[str, Any]) -> Dict[str, Any]:
        # 新图名写进底盘目录。switch 缺省 false，不切当前图。
        if str(self.config.get("adapter") or "").strip().lower() != "rokae":
            return _rejected("NAVIGATION_UNAVAILABLE", "map import is only implemented for rokae")
        map_name = str(body.get("map_name") or "").strip()
        if not map_name or any(ch not in "abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789_-" for ch in map_name):
            return _rejected("NAVIGATION_INVALID_REQUEST", "map_name must be letters, digits, _ or -")
        if map_name[-1].isdigit():
            # 底盘从 {图名}0.pgm 认图名。图名末尾是数字时，nav_try_010.pgm 会被看成空名，回 90002。
            return _rejected("NAVIGATION_INVALID_REQUEST", "map_name must not end with a digit")
        topology = body.get("topology")
        if not isinstance(topology, dict):
            return _rejected("NAVIGATION_INVALID_REQUEST", "topology object is required")
        switch = _want_switch(body)
        if switch is None:
            return _rejected("NAVIGATION_INVALID_REQUEST", "switch must be a boolean")
        snap = self.adapter.snapshot()
        current = str(snap.get("map_id") or "")
        if current and map_name == current:
            return _rejected("NAVIGATION_INVALID_REQUEST", "refusing to overwrite the loaded map")
        pose = _switch_pose(snap) if switch else None
        if switch and pose is None:
            return _rejected("NAVIGATION_INVALID_REQUEST", "switch requires a current pose")
        with self._lock:
            if self._current_id and self._actions[self._current_id]["state"] not in _TERMINAL:
                return _rejected("RESOURCE_BUSY", "chassis is busy")
        matrix = (self.config.get("rokae") or {}).get("matrix") or {}
        base_url = str(matrix.get("base_url") or "").strip()
        if not base_url:
            return _rejected("NAVIGATION_UNAVAILABLE", "rokae.matrix.base_url is empty")
        meta = topology.get("meta") if isinstance(topology.get("meta"), dict) else {}
        version = str(body.get("base_version") or meta.get("version") or "1.13.0")
        try:
            raster = blank_pgm(topology)
            packed = fms_gzip(map_name, topology, raster)
            status, parsed = import_fms(
                base_url,
                packed,
                map_name=map_name,
                base_version=version,
                timeout=30,
            )
        except (OSError, ValueError) as exc:
            return _rejected("NAVIGATION_VENDOR_UNREACHABLE", str(exc))
        if status != 200:
            return {
                "accepted": False,
                "terminal_state": "REJECTED",
                "error_code": "NAVIGATION_VENDOR_REJECTED",
                "message": f"chassis import HTTP {status}",
                "chassis": parsed,
            }
        if switch:
            failed = self._switch_after_import(map_name, pose)
            if failed is not None:
                return failed
            return {
                "accepted": True,
                "map_name": map_name,
                "chassis_status": status,
                "pgm_bytes": len(raster),
                "switched": True,
                "message": "imported and switched",
            }
        return {
            "accepted": True,
            "map_name": map_name,
            "chassis_status": status,
            "pgm_bytes": len(raster),
            "switched": False,
            "message": "imported; current map was not switched",
        }

    def _switch_after_import(self, map_name: str, pose: tuple) -> Optional[Dict[str, Any]]:
        switch = getattr(self.adapter, "switch_map", None)
        if not callable(switch):
            return {
                "accepted": False,
                "terminal_state": "REJECTED",
                "error_code": "NAVIGATION_UNAVAILABLE",
                "imported": True,
                "switched": False,
                "map_name": map_name,
                "message": "map is in the chassis catalog; this backend cannot switch",
            }
        x_m, y_m, yaw = pose
        result = switch(map_name, x_m, y_m, yaw)
        if not isinstance(result, dict) or not result.get("accepted"):
            return {
                "accepted": False,
                "terminal_state": "REJECTED",
                "error_code": (result or {}).get("error_code") or "NAVIGATION_VENDOR",
                "imported": True,
                "switched": False,
                "map_name": map_name,
                "message": (result or {}).get("message") or "switch failed after import",
            }
        refresh = getattr(self.adapter, "refresh_stations", None)
        if callable(refresh):
            try:
                refresh(pull_occupancy=True)
            except Exception as exc:
                LOGGER.warning("refresh after map switch failed: %s", exc)
        return None

    def _run_goto(
        self,
        request_id: str,
        station_id: str,
        idempotency_key: str,
        timeout_sec: float,
    ) -> None:
        # 这里才会真正调厂家。结果写回 _actions，zhongmian 的轮询能读到。
        with self._lock:
            action = self._actions.get(request_id)
            if action is None or action["state"] in _TERMINAL:
                return
            action["state"] = "RUNNING"
        started = time.monotonic()
        try:
            result = self.adapter.goto(
                station_id=station_id,
                idempotency_key=idempotency_key,
                timeout_sec=timeout_sec,
            )
        except Exception as exc:
            LOGGER.exception("goto worker failed: request_id=%s", request_id)
            result = {
                "terminal_state": "FAILED",
                "error_code": "NAVIGATION_INTERNAL",
                "message": str(exc),
            }
        elapsed = time.monotonic() - started
        terminal = str(result.get("terminal_state") or "FAILED")
        if elapsed >= timeout_sec and terminal not in _TERMINAL:
            terminal = "TIMED_OUT"
        with self._lock:
            action = self._actions.get(request_id)
            if action is None or action["state"] in _TERMINAL:
                return
            action["state"] = terminal
            action["terminal_state"] = terminal
            action["error_code"] = str(result.get("error_code") or "")
            action["message"] = str(result.get("message") or "")
            action["evidence"] = result.get("evidence")
            LOGGER.info(
                "goto finished: request_id=%s station_id=%s state=%s error=%s",
                request_id,
                station_id,
                terminal,
                action["error_code"] or "-",
            )

    @staticmethod
    def _public_state(vendor_state: str) -> str:
        mapping = {
            "UNREADY": "IDLE",
            "IDLE": "IDLE",
            "READY": "IDLE",
            "NAVIGATING": "RUNNING",
            "LOCALIZING": "RUNNING",
            "MAPPING": "RUNNING",
            "HOLDING": "RUNNING",
            "PAUSED": "RUNNING",
            "ARRIVED": "SUCCEEDED",
            "ERROR": "FAILED",
            "CANCELLED": "CANCELLED",
        }
        return mapping.get(vendor_state.upper(), vendor_state.upper() or "IDLE")


def _map_fragment(snap: Dict[str, Any]) -> Dict[str, Any]:
    body: Dict[str, Any] = {
        "map_id": str(snap.get("map_id") or ""),
        "stations": snap.get("stations") if isinstance(snap.get("stations"), list) else [],
    }
    for key in ("nodes", "edges"):
        rows = snap.get(key)
        if isinstance(rows, list):
            body[key] = rows
    revision = str(snap.get("occupancy_revision") or "").strip()
    if revision:
        body["occupancy_revision"] = revision
    return body


def _load_map_ok(
    map_id: str,
    snap: Dict[str, Any],
    adapter: Any,
    stations: Optional[list] = None,
) -> Dict[str, Any]:
    rows = stations if isinstance(stations, list) and stations else (
        snap.get("stations") if isinstance(snap.get("stations"), list) else []
    )
    body: Dict[str, Any] = {
        "accepted": True,
        "map_id": str(map_id or snap.get("map_id") or ""),
        "stations": rows,
    }
    getter = getattr(adapter, "occupancy", None)
    if callable(getter):
        try:
            grid = maybe_occupancy(getter(str(map_id or snap.get("map_id") or "")))
        except Exception as exc:
            LOGGER.warning("occupancy unavailable: map_id=%s error=%s", map_id, exc)
            grid = None
        if grid:
            body["occupancy"] = grid
    return body


def build_adapter(config: Dict[str, Any]) -> Any:
    # 看 config/navigation.json 的 "adapter"。中免现场是 rokae；tianji 是另一厂家。
    name = str(config.get("adapter") or "tianji").strip().lower()
    if name == "fake":
        return FakeNavigationAdapter()
    if name == "tianji":
        return TianjiNavigationAdapter(config)
    if name == "rokae":
        return RokaeNavigationAdapter(config)
    raise ValueError(f"unknown navigation adapter: {name}")


def _validate_context(body: Dict[str, Any]) -> Optional[Dict[str, Any]]:
    for key in _REQUIRED_CONTEXT:
        if body.get(key) in (None, ""):
            return _rejected("NAVIGATION_INVALID_REQUEST", f"{key} is required")
    try:
        timeout = float(body.get("timeout_sec"))
    except (TypeError, ValueError):
        return _rejected("NAVIGATION_INVALID_REQUEST", "timeout_sec must be > 0")
    if timeout <= 0:
        return _rejected("NAVIGATION_INVALID_REQUEST", "timeout_sec must be > 0")
    return None


def _want_switch(body: Dict[str, Any]) -> Optional[bool]:
    if "switch" not in body or body.get("switch") is None:
        return False
    value = body.get("switch")
    if isinstance(value, bool):
        return value
    if isinstance(value, (int, float)) and not isinstance(value, bool) and value in (0, 1):
        return bool(int(value))
    if isinstance(value, str):
        token = value.strip().lower()
        if token in {"1", "true", "yes"}:
            return True
        if token in {"0", "false", "no", ""}:
            return False
    return None


def _switch_pose(snap: Dict[str, Any]) -> Optional[tuple]:
    pos = snap.get("position") if isinstance(snap.get("position"), dict) else None
    if not pos or "x" not in pos or "y" not in pos:
        return None
    try:
        return (float(pos["x"]), float(pos["y"]), float(pos.get("yaw") or 0.0))
    except (TypeError, ValueError):
        return None


def _rejected(error_code: str, message: str) -> Dict[str, Any]:
    # 调用方仍收到 HTTP 200，靠 accepted=false 判断没发车。
    return {
        "accepted": False,
        "terminal_state": "REJECTED",
        "error_code": error_code,
        "message": message,
    }
