"""珞石 Adapter：把能力层的 station_id（A/B）译成 MATRIX 站点号（1/2），再交给 backend。

没有单独 HTTP 口，和 :8001 同一进程。:8081 是中免业务层，不是本文件。
backend 由 navigation.json 的 rokae.backend 选：现场 sros，本机假车也是 sros + sim:true。
"""

from __future__ import annotations

import logging
import threading
import time
from typing import Any, Dict, Optional

from navigation.adapters.rokae_telemetry import osd_from_vendor
from navigation.rokae_runtime import build_backend, load_stations, resolve_station_id
from navigation.rokae_runtime.base import LOCATION_STATE_RUNNING, RokaeVendor

__all__ = ["RokaeNavigationAdapter", "resolve_station_id"]

LOGGER = logging.getLogger(__name__)


def _keep_previous_aliases(stations: list[Dict[str, Any]], old_rows: list[Dict[str, Any]]) -> None:
    # MATRIX 名可能是「站点1」，按厂家 id 保留 json 里的 A/B，中免 AGV_L→A 不用改。
    old_by_id: Dict[str, Dict[str, Any]] = {}
    for row in old_rows:
        key = str(row.get("id") or row.get("vendor_id") or "").strip()
        if key:
            old_by_id[key] = row
    for row in stations:
        old = old_by_id.get(str(row.get("vendor_id") or row.get("id") or "").strip())
        if old is None:
            continue
        aliases = row.setdefault("aliases", [])
        for token in (old.get("name"), old.get("station_id"), *(old.get("aliases") or [])):
            name = str(token or "").strip()
            if name and name not in aliases:
                aliases.append(name)


class RokaeNavigationAdapter:
    def __init__(self, config: Dict[str, Any], *, backend: Optional[RokaeVendor] = None) -> None:
        rokae = config.get("rokae") or {}
        if not isinstance(rokae, dict):
            rokae = {}
        self._map_id = str(rokae.get("map_id") or "AB_0619")
        self._localized_wait_sec = float(rokae.get("localized_wait_sec", 3.0))
        self._location_running = int(
            rokae.get("location_state_running", LOCATION_STATE_RUNNING)
        )
        self._stations = load_stations(rokae)
        self._nodes: list[Dict[str, Any]] = []
        self._edges: list[Dict[str, Any]] = []
        self._config = config
        self._vendor = backend if backend is not None else build_backend(config)
        self._lock = threading.Lock()
        self._station_id = ""  # 上次目标站名，给 /state 用
        self._nav_state = ""
        self._station_fetch_done = False
        self._occupancy_by_map: Dict[str, Dict[str, Any]] = {}
        self._occupancy_revision_by_map: Dict[str, str] = {}
        self._refresh_inflight = False
        self._refresh_backoff_until = 0.0
        self._matrix_fingerprint = ""
        self._map_file_revision = ""
        self._next_matrix_poll = 0.0
        matrix = rokae.get("matrix") if isinstance(rokae.get("matrix"), dict) else {}
        try:
            self._matrix_poll_sec = float(matrix.get("poll_sec", 10) or 0)
        except (TypeError, ValueError):
            self._matrix_poll_sec = 10.0
        LOGGER.info(
            "rokae adapter: backend=%s map_id=%s stations=%s",
            str(rokae.get("backend") or "ros"),
            self._map_id,
            [row.get("name") or row.get("id") for row in self._stations],
        )
        self.ensure_stations()

    def close(self) -> None:
        close = getattr(self._vendor, "close", None)
        if callable(close):
            close()

    def ready(self) -> bool:
        return self._is_localized(self._vendor.latest_state())

    def known_station(self, station_id: str) -> bool:
        with self._lock:
            return resolve_station_id(station_id, self._stations) is not None

    def ensure_stations(self) -> None:
        # 真机启动拉一次。之后只轮询网页地图目录的 md5；变了才下站点，换图号才下栅格。
        if self._station_fetch_done:
            return
        if not self._should_fetch_matrix():
            self._station_fetch_done = True
            return
        result = self.refresh_stations(pull_occupancy=True)
        self._station_fetch_done = True
        if not result.get("accepted"):
            LOGGER.warning(
                "matrix station refresh skipped, goto uses fallback A/B: %s",
                result.get("message"),
            )

    def _should_fetch_matrix(self) -> bool:
        rokae = self._config.get("rokae") if isinstance(self._config.get("rokae"), dict) else {}
        backend = str(rokae.get("backend") or "ros")
        if backend == "mock":
            return False
        sros = rokae.get("sros") if isinstance(rokae.get("sros"), dict) else {}
        if bool(sros.get("sim")):
            return False
        if backend != "sros":
            return False
        matrix = rokae.get("matrix") if isinstance(rokae.get("matrix"), dict) else {}
        if str(matrix.get("topology_path") or "").strip():
            return True
        return bool(str(matrix.get("base_url") or sros.get("host") or "").strip())

    def snapshot(self) -> Dict[str, Any]:
        state = self._vendor.latest_state() or {}
        ready = self._is_localized(state)
        with self._lock:
            station_id = self._station_id
            nav_state = self._nav_state
            stations = [
                {
                    "station_id": str(row.get("name") or row.get("id") or ""),
                    **{
                        key: row[key]
                        for key in ("x", "y", "yaw")
                        if key in row and row[key] is not None
                    },
                }
                for row in self._stations
            ]
            nodes = list(self._nodes)
            edges = list(self._edges)
        live_station = int(state.get("current_station_id") or 0)
        if live_station > 0:
            station_id = self._name_for_vendor_id(live_station) or str(live_station)
        if state.get("executing_movement_task"):
            live_nav = "NAVIGATING"
        elif not ready:
            live_nav = "UNREADY"
        else:
            live_nav = nav_state or "IDLE"
        live_map = str(state.get("map_name") or "").strip()
        if live_map:
            self._maybe_sync_matrix(live_map)
        out: Dict[str, Any] = {
            "ready": ready,
            "nav_state": live_nav,
            "station_id": station_id,
            "map_id": live_map or self._map_id,
            "stations": stations,
            "nodes": nodes,
            "edges": edges,
        }
        revision = self._revision_for(str(out["map_id"]))
        if revision:
            out["occupancy_revision"] = revision
        if state.get("x") is not None and state.get("y") is not None:
            out["position"] = {
                "x": float(state["x"]),
                "y": float(state["y"]),
                "yaw": float(state.get("yaw") or 0.0),
            }
        out.update(osd_from_vendor(state, self._vendor.latest_battery()))
        return out

    def goto(
        self,
        *,
        station_id: str,
        idempotency_key: str,
        timeout_sec: float,
    ) -> Dict[str, Any]:
        del idempotency_key  # 幂等在 NavigationService 做完了，厂家 SDK 不认这把 key。
        vendor_id = resolve_station_id(station_id, self._stations)
        if vendor_id is None:
            LOGGER.warning("goto rejected unknown station: station_id=%s map=%s", station_id, self._map_id)
            return {
                "terminal_state": "REJECTED",
                "error_code": "NAVIGATION_UNKNOWN_STATION",
                "message": f"station_id={station_id} not on map {self._map_id}",
            }
        if not self._wait_until_localized(self._localized_wait_sec):
            LOGGER.warning(
                "goto blocked: location_state is not RUNNING (%s)",
                self._location_running,
            )
            return {
                "terminal_state": "REJECTED",
                "error_code": "NAVIGATION_NOT_READY",
                "message": f"location_state is not RUNNING ({self._location_running})",
            }
        timeout = max(float(timeout_sec or 0), 1.0)
        LOGGER.info(
            "goto vendor: station_id=%s vendor_id=%s timeout_sec=%s",
            station_id,
            vendor_id,
            timeout,
        )
        with self._lock:
            self._station_id = str(station_id)
            self._nav_state = "NAVIGATING"
        result = self._vendor.move_to_station(vendor_id, timeout)
        # vendor_id 是 MATRIX 数字站号。A→1、B→2，见 config/navigation.json 的 rokae.stations。
        if result.get("unavailable"):
            with self._lock:
                self._nav_state = "IDLE"
            LOGGER.error("goto vendor unreachable: station_id=%s", station_id)
            return {
                "terminal_state": "FAILED",
                "error_code": "NAVIGATION_VENDOR_UNREACHABLE",
                "message": str(result.get("message") or "rokae vendor unavailable"),
            }
        if not result.get("accepted"):
            with self._lock:
                self._nav_state = "IDLE"
            LOGGER.warning("goto vendor REJECTED: station_id=%s vendor_id=%s", station_id, vendor_id)
            return {
                "terminal_state": "REJECTED",
                "error_code": "NAVIGATION_REJECTED",
                "message": str(result.get("message") or f"vendor rejected station_id={vendor_id}"),
            }
        if result.get("timed_out"):
            with self._lock:
                self._nav_state = "IDLE"
            return {
                "terminal_state": "TIMED_OUT",
                "error_code": "NAVIGATION_TIMEOUT",
                "message": str(result.get("message") or "move_to_station result timeout"),
            }
        if result.get("result"):
            with self._lock:
                self._station_id = str(station_id)
                self._nav_state = "IDLE"
            LOGGER.info("goto vendor SUCCEEDED: station_id=%s vendor_id=%s", station_id, vendor_id)
            return {
                "terminal_state": "SUCCEEDED",
                "evidence": {"arrived": True, "station_id": station_id, "vendor_id": vendor_id},
            }
        with self._lock:
            self._nav_state = "IDLE"
        error_code = str(result.get("error_code") or "NAVIGATION_VENDOR")
        LOGGER.error(
            "goto vendor FAILED: station_id=%s vendor_id=%s error=%s",
            station_id,
            vendor_id,
            error_code,
        )
        return {
            "terminal_state": "FAILED",
            "error_code": "NAVIGATION_VENDOR",
            "message": error_code,
        }

    def stop(self) -> Dict[str, Any]:
        stop_motion = getattr(self._vendor, "stop_motion", None)
        if callable(stop_motion):
            return stop_motion()
        LOGGER.warning("stop unavailable on current rokae backend")
        return {
            "accepted": False,
            "error_code": "NAVIGATION_UNAVAILABLE",
            "message": "rokae backend has no stop; sros uses cancel_movement_task",
        }

    def switch_map(self, map_name: str, x_m: float, y_m: float, yaw_rad: float) -> Dict[str, Any]:
        switch = getattr(self._vendor, "switch_map", None)
        if not callable(switch):
            return {
                "accepted": False,
                "error_code": "NAVIGATION_UNAVAILABLE",
                "message": "rokae backend cannot switch maps",
            }
        return switch(map_name, x_m, y_m, yaw_rad)

    def load_map(self, map_id: str) -> Dict[str, Any]:
        LOGGER.warning("vendor map switch unavailable on rokae; use unified apply_map: map_id=%s", map_id)
        return {
            "accepted": False,
            "error_code": "NAVIGATION_UNAVAILABLE",
            "message": "rokae does not switch MATRIX maps; POST /load_map with unified stations",
        }

    def occupancy(self, map_id: str) -> Optional[Dict[str, Any]]:
        name = str(map_id or "").strip()
        if not name:
            with self._lock:
                name = str(self._map_id or "")
        if not name:
            return None
        with self._lock:
            cached = self._occupancy_by_map.get(name)
        if cached:
            return dict(cached)
        from navigation.occupancy import occupancy_revision
        from navigation.rokae_runtime.matrix_occupancy import occupancy_from_rokae_config
        from navigation.unified_map import maps_dir, occupancy_path, read_occupancy, write_occupancy

        grid = None
        try:
            grid = read_occupancy(maps_dir(self._config), name)
        except (FileNotFoundError, ValueError):
            try:
                grid = occupancy_from_rokae_config(self._config, map_name=name)
            except Exception as exc:
                LOGGER.warning("rokae occupancy fetch failed: map_id=%s error=%s", name, exc)
                return None
        if not grid:
            return None
        grid = self._fit_occupancy(grid)
        stored = occupancy_path(maps_dir(self._config), name)
        if not stored.is_file():
            try:
                write_occupancy(maps_dir(self._config), name, grid)
            except Exception as exc:
                LOGGER.warning("occupancy not saved: map_id=%s error=%s", name, exc)
        revision = occupancy_revision(grid)
        with self._lock:
            self._occupancy_by_map[name] = dict(grid)
            if revision:
                self._occupancy_revision_by_map[name] = revision
        return dict(grid)

    def apply_map(self, map_id: str, stations: list[Dict[str, Any]]) -> Dict[str, Any]:
        # 只改本进程里的站点表，不会去切 MATRIX 网页上的原图。
        from navigation.unified_map import to_rokae_stations

        rows = to_rokae_stations({"stations": stations, "map_id": map_id})
        if not rows:
            return {
                "accepted": False,
                "error_code": "NAVIGATION_INVALID_REQUEST",
                "message": "stations is empty",
            }
        with self._lock:
            self._map_id = str(map_id)
            self._stations = rows
        apply_vendor = getattr(self._vendor, "apply_stations", None)
        if callable(apply_vendor):
            apply_vendor(rows)
        LOGGER.info("rokae unified map applied: map_id=%s stations=%s", map_id, [row.get("name") for row in rows])
        return {"accepted": True, "map_id": str(map_id)}

    def refresh_stations(
        self, *, topology: Optional[Dict[str, Any]] = None, pull_occupancy: bool = True
    ) -> Dict[str, Any]:
        from navigation.rokae_runtime.matrix_stations import (
            catalog_from_rokae_config,
            edges_from_topology,
            nodes_from_topology,
            stations_from_topology,
            topology_fingerprint,
            topology_from_rokae_config,
        )
        from navigation.unified_map import canonicalize, maps_dir, write_map

        live_map = ""
        for _ in range(5):
            state = self._vendor.latest_state() if self._vendor is not None else None
            if isinstance(state, dict):
                live_map = str(state.get("map_name") or "").strip()
            if live_map:
                break
            time.sleep(0.1)
        try:
            doc = (
                topology
                if topology is not None
                else topology_from_rokae_config(self._config, map_name=live_map)
            )
            if doc is None:
                return {
                    "accepted": False,
                    "error_code": "NAVIGATION_UNAVAILABLE",
                    "message": "no MATRIX topology (sim has no export)",
                }
            stations = stations_from_topology(doc)
            nodes = nodes_from_topology(doc)
            edges = edges_from_topology(doc)
        except Exception as exc:
            LOGGER.warning("matrix station refresh failed: %s", exc)
            return {
                "accepted": False,
                "error_code": "NAVIGATION_VENDOR_UNREACHABLE",
                "message": str(exc),
            }
        map_id = live_map or self._map_id
        with self._lock:
            old_rows = list(self._stations)
        _keep_previous_aliases(stations, load_stations(self._config.get("rokae") or {}))
        _keep_previous_aliases(stations, old_rows)
        applied = self.apply_map(map_id, stations)
        if not applied.get("accepted"):
            return applied
        with self._lock:
            self._nodes = nodes
            self._edges = edges
        try:
            write_map(maps_dir(self._config), canonicalize(map_id, stations))
        except Exception as exc:
            LOGGER.warning("matrix stations not saved to maps/: %s", exc)
        revision = self._store_occupancy(str(map_id), fetch=pull_occupancy)
        applied["stations"] = stations
        if revision:
            applied["occupancy_revision"] = revision
        file_rev = ""
        try:
            file_rev = catalog_from_rokae_config(self._config, map_name=str(map_id))
        except Exception as exc:
            LOGGER.debug("matrix catalog peek failed: %s", exc)
        with self._lock:
            self._station_fetch_done = True
            self._matrix_fingerprint = topology_fingerprint(doc, map_name=str(map_id))
            if file_rev:
                self._map_file_revision = file_rev
            self._next_matrix_poll = time.monotonic() + max(self._matrix_poll_sec, 0.0)
        LOGGER.info(
            "matrix stations refreshed: map_id=%s count=%s edges=%s names=%s occupancy=%s catalog=%s",
            self._map_id,
            len(stations),
            len(edges),
            [row.get("station_id") for row in stations],
            revision or "missing",
            file_rev or self._matrix_fingerprint,
        )
        return applied

    def _store_occupancy(self, map_id: str, *, fetch: bool = True) -> str:
        from navigation.occupancy import occupancy_revision
        from navigation.rokae_runtime.matrix_occupancy import occupancy_from_rokae_config
        from navigation.unified_map import maps_dir, read_occupancy, write_occupancy

        grid = None
        if fetch:
            try:
                grid = occupancy_from_rokae_config(self._config, map_name=map_id)
            except Exception as exc:
                LOGGER.warning("matrix occupancy refresh failed: %s", exc)
        if grid is None:
            try:
                grid = read_occupancy(maps_dir(self._config), map_id)
            except (FileNotFoundError, ValueError):
                return ""
        grid = self._fit_occupancy(grid)
        try:
            write_occupancy(maps_dir(self._config), map_id, grid)
        except Exception as exc:
            LOGGER.warning("occupancy not saved: map_id=%s error=%s", map_id, exc)
        revision = occupancy_revision(grid)
        with self._lock:
            self._occupancy_by_map[map_id] = dict(grid)
            if revision:
                self._occupancy_revision_by_map[map_id] = revision
        return revision

    def _fit_occupancy(self, grid: Dict[str, Any]) -> Dict[str, Any]:
        from navigation.occupancy import crop_unknown_border

        with self._lock:
            stations = list(self._stations)
        return crop_unknown_border(grid, extra_xy=stations)

    def _revision_for(self, map_id: str) -> str:
        with self._lock:
            cached = self._occupancy_revision_by_map.get(map_id) or ""
            grid = self._occupancy_by_map.get(map_id)
        if cached:
            return cached
        if grid:
            from navigation.occupancy import occupancy_revision

            return occupancy_revision(grid)
        from navigation.unified_map import maps_dir, read_occupancy

        try:
            stored = read_occupancy(maps_dir(self._config), map_id)
        except (FileNotFoundError, ValueError):
            return ""
        from navigation.occupancy import occupancy_revision

        revision = occupancy_revision(stored)
        with self._lock:
            self._occupancy_by_map[map_id] = dict(stored)
            if revision:
                self._occupancy_revision_by_map[map_id] = revision
        return revision

    def _maybe_sync_matrix(self, live_map: str) -> None:
        if not self._should_fetch_matrix():
            return
        now = time.monotonic()
        with self._lock:
            current = str(self._map_id or "")
            map_changed = live_map != current
            poll_due = self._matrix_poll_sec > 0 and now >= self._next_matrix_poll
            if self._refresh_inflight:
                return
            if map_changed:
                pass
            elif now < self._refresh_backoff_until:
                return
            elif poll_due:
                pass
            else:
                return
            self._refresh_inflight = True
        worker = threading.Thread(
            target=self._sync_matrix_worker,
            args=(live_map, map_changed),
            name=f"nav-matrix-sync-{live_map}",
            daemon=True,
        )
        worker.start()

    def _peek_map_revision(self, live_map: str) -> str:
        getter = getattr(self._vendor, "map_file_revision", None)
        if callable(getter):
            try:
                revision = str(getter(live_map) or "").strip()
            except Exception as exc:
                LOGGER.debug("vendor map revision failed: %s", exc)
                revision = ""
            if revision:
                return revision
        from navigation.rokae_runtime.matrix_stations import catalog_from_rokae_config

        try:
            return catalog_from_rokae_config(self._config, map_name=live_map)
        except Exception as exc:
            LOGGER.debug("matrix catalog poll failed: %s", exc)
            return ""

    def _sync_matrix_worker(self, live_map: str, map_changed: bool) -> None:
        from navigation.rokae_runtime.matrix_stations import (
            topology_fingerprint,
            topology_from_rokae_config,
        )

        try:
            catalog_rev = self._peek_map_revision(live_map)
            with self._lock:
                same_file = bool(
                    catalog_rev
                    and catalog_rev == self._map_file_revision
                    and not map_changed
                )
            if same_file:
                LOGGER.debug(
                    "matrix catalog unchanged: map=%s revision=%s",
                    live_map,
                    catalog_rev,
                )
                return
            if catalog_rev or map_changed:
                LOGGER.info(
                    "matrix catalog changed: map=%s map_changed=%s revision=%s->%s",
                    live_map,
                    map_changed,
                    self._map_file_revision,
                    catalog_rev,
                )
                if catalog_rev:
                    with self._lock:
                        self._map_file_revision = catalog_rev
                result = self.refresh_stations(pull_occupancy=map_changed)
            else:
                doc = topology_from_rokae_config(self._config, map_name=live_map)
                if doc is None:
                    return
                fingerprint = topology_fingerprint(doc, map_name=live_map)
                with self._lock:
                    unchanged = (
                        fingerprint == self._matrix_fingerprint
                        and live_map == str(self._map_id or "")
                    )
                if unchanged:
                    LOGGER.debug(
                        "matrix topology unchanged: map=%s fingerprint=%s",
                        live_map,
                        fingerprint,
                    )
                    return
                LOGGER.info(
                    "matrix topology changed: map=%s fingerprint=%s->%s",
                    live_map,
                    self._matrix_fingerprint,
                    fingerprint,
                )
                result = self.refresh_stations(topology=doc, pull_occupancy=False)
            if not result.get("accepted"):
                LOGGER.warning(
                    "matrix auto-refresh failed: map=%s error=%s",
                    live_map,
                    result.get("message"),
                )
                with self._lock:
                    self._refresh_backoff_until = time.monotonic() + 5.0
        except Exception as exc:
            LOGGER.warning("matrix auto-refresh crashed: map=%s error=%s", live_map, exc)
            with self._lock:
                self._refresh_backoff_until = time.monotonic() + 5.0
        finally:
            with self._lock:
                self._refresh_inflight = False
                self._next_matrix_poll = time.monotonic() + max(self._matrix_poll_sec, 0.0)

    def _is_localized(self, state: Optional[Dict[str, Any]]) -> bool:
        # MATRIX location_state==3 才算在路上；否则厂家错误 321005。
        return bool(
            state is not None
            and int(state.get("location_state") or -1) == self._location_running
        )

    def _wait_until_localized(self, timeout_sec: float) -> bool:
        deadline = time.monotonic() + max(0.0, float(timeout_sec))
        while time.monotonic() < deadline:
            if self._is_localized(self._vendor.latest_state()):
                return True
            time.sleep(0.05)
        return self._is_localized(self._vendor.latest_state())

    def _name_for_vendor_id(self, vendor_id: int) -> str:
        for row in self._stations:
            if int(row.get("id") or 0) == int(vendor_id):
                return str(row.get("name") or row.get("id") or vendor_id)
        return ""
