"""地图上云：换图时 POST /load_map 一次，把 occupancy 原样上报 robotDog/map。"""

from __future__ import annotations

import json
import logging
import math
import os
import tempfile
import threading
import time
import urllib.error
import urllib.parse
import urllib.request
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Callable, Dict, Mapping, Optional
from uuid import UUID, uuid4

from gateway.config import PACKAGE_ROOT
from gateway.platform_headers import json_request_headers

LOGGER = logging.getLogger(__name__)

_SCHEMA_VERSION = 1
DEFAULT_REPORT_URL = "https://robotsolution.cn/dj-prod-api/robotDog/map"
DEFAULT_IDENTITY_FILE = "runtime/map_index.json"


class MapSyncError(RuntimeError):
    def __init__(self, code: str, message: str, *, http_status: int = 400) -> None:
        super().__init__(message)
        self.code = str(code or "MAP_SYNC_ERROR")
        self.http_status = int(http_status)


@dataclass(frozen=True)
class MapIdentity:
    source_map_id: str
    map_id: str
    created_at_ms: int = 0
    last_upload_ok: bool = False
    last_upload_at_ms: int = 0
    last_error: str = ""
    source_path: str = ""

    def as_dict(self) -> Dict[str, Any]:
        payload = {
            "source_map_id": self.source_map_id,
            "map_id": self.map_id,
            "created_at_ms": self.created_at_ms,
            "last_upload_ok": self.last_upload_ok,
            "last_upload_at_ms": self.last_upload_at_ms,
            "last_error": self.last_error,
        }
        if self.source_path:
            payload["source_path"] = self.source_path
        return payload


@dataclass(frozen=True)
class OccupancyGrid:
    width: int
    height: int
    resolution: float
    origin_x: float
    origin_y: float
    data: tuple[int, ...]


class MapIdentityStore:
    """把天机本地图号（如 42）映射成云端要求的 UUID，并记住上次是否上传成功。"""

    def __init__(self, path: Path) -> None:
        self.path = Path(path)
        self._lock = threading.RLock()
        self._loaded = False
        self._by_source: Dict[str, MapIdentity] = {}

    def peek(self, source_map_id: str) -> Optional[MapIdentity]:
        source_id = _normalize_source_id(source_map_id)
        if not source_id:
            return None
        with self._lock:
            self._load_once()
            return self._by_source.get(source_id)

    def uploaded_map_id(self, source_map_id: str) -> str:
        identity = self.peek(source_map_id)
        if identity is None or not identity.last_upload_ok:
            return ""
        return identity.map_id

    def items(self) -> list[MapIdentity]:
        with self._lock:
            self._load_once()
            return [
                self._by_source[key]
                for key in sorted(self._by_source)
            ]

    def resolve_or_create(
        self,
        source_map_id: str,
        *,
        preferred_map_id: str = "",
        now_ms: int,
    ) -> MapIdentity:
        source_id = _normalize_source_id(source_map_id)
        if not source_id:
            raise MapSyncError("MAP_SYNC_INVALID_SOURCE", "source_map_id is required")
        with self._lock:
            self._load_once()
            existing = self._by_source.get(source_id)
            if existing is not None:
                return existing
            map_id = canonical_uuid(preferred_map_id) or canonical_uuid(uuid4())
            if not map_id:
                raise MapSyncError("MAP_SYNC_UUID", "cannot allocate map UUID")
            identity = MapIdentity(
                source_map_id=source_id,
                map_id=map_id,
                created_at_ms=max(int(now_ms), 0),
            )
            next_items = dict(self._by_source)
            next_items[source_id] = identity
            self._write(next_items)
            self._by_source = next_items
            LOGGER.info(
                "map identity created: source_map=%s map=%s",
                identity.source_map_id,
                identity.map_id,
            )
            return identity

    def mark_upload(
        self,
        source_map_id: str,
        *,
        ok: bool,
        now_ms: int,
        error: str = "",
        source_path: str = "",
    ) -> MapIdentity:
        source_id = _normalize_source_id(source_map_id)
        with self._lock:
            self._load_once()
            existing = self._by_source.get(source_id)
            if existing is None:
                raise MapSyncError("MAP_SYNC_NO_IDENTITY", "map identity is missing")
            updated = MapIdentity(
                source_map_id=existing.source_map_id,
                map_id=existing.map_id,
                created_at_ms=existing.created_at_ms,
                last_upload_ok=bool(ok),
                last_upload_at_ms=max(int(now_ms), 0),
                last_error="" if ok else str(error or "")[:300],
                source_path=str(source_path or existing.source_path or ""),
            )
            next_items = dict(self._by_source)
            next_items[source_id] = updated
            self._write(next_items)
            self._by_source = next_items
            return updated

    def _load_once(self) -> None:
        if self._loaded:
            return
        if not self.path.exists():
            self._loaded = True
            return
        try:
            value = json.loads(self.path.read_text(encoding="utf-8"))
        except (OSError, json.JSONDecodeError) as exc:
            raise MapSyncError(
                "MAP_SYNC_IDENTITY_READ",
                f"cannot read map identity index: {exc}",
                http_status=500,
            ) from exc
        if not isinstance(value, dict):
            raise MapSyncError("MAP_SYNC_IDENTITY_INVALID", "map identity index must be an object")
        items = value.get("items")
        if not isinstance(items, list):
            raise MapSyncError("MAP_SYNC_IDENTITY_INVALID", "map identity items must be an array")
        by_source: Dict[str, MapIdentity] = {}
        for item in items:
            if not isinstance(item, dict):
                continue
            source_id = _normalize_source_id(item.get("source_map_id"))
            map_id = canonical_uuid(item.get("map_id"))
            if not source_id or not map_id:
                continue
            by_source[source_id] = MapIdentity(
                source_map_id=source_id,
                map_id=map_id,
                created_at_ms=max(int(item.get("created_at_ms") or 0), 0),
                last_upload_ok=bool(item.get("last_upload_ok")),
                last_upload_at_ms=max(int(item.get("last_upload_at_ms") or 0), 0),
                last_error=str(item.get("last_error") or ""),
                source_path=str(item.get("source_path") or ""),
            )
        self._by_source = by_source
        self._loaded = True

    def _write(self, identities: Dict[str, MapIdentity]) -> None:
        self.path.parent.mkdir(parents=True, exist_ok=True)
        descriptor, temporary_name = tempfile.mkstemp(
            prefix=f".{self.path.name}.",
            suffix=".tmp",
            dir=str(self.path.parent),
        )
        temporary_path = Path(temporary_name)
        payload = {
            "schema_version": _SCHEMA_VERSION,
            "items": [
                item.as_dict()
                for item in sorted(identities.values(), key=lambda value: value.source_map_id)
            ],
        }
        try:
            with os.fdopen(descriptor, "w", encoding="utf-8") as stream:
                json.dump(payload, stream, ensure_ascii=False, sort_keys=True, indent=2)
                stream.write("\n")
                stream.flush()
                os.fsync(stream.fileno())
            os.replace(temporary_path, self.path)
        except Exception:
            try:
                os.close(descriptor)
            except OSError:
                pass
            temporary_path.unlink(missing_ok=True)
            raise


class MapSyncService:
    def __init__(
        self,
        config: Mapping[str, Any],
        *,
        collector: Any = None,
        opener: Optional[Callable[..., Any]] = None,
        clock: Callable[[], float] = time.time,
    ) -> None:
        self._config = config
        self._collector = collector
        self._opener = opener or urllib.request.urlopen
        self._clock = clock
        cfg = _map_sync_config(config)
        self.enabled = bool(cfg.get("enabled", True))
        self.report_url = str(cfg.get("report_url") or DEFAULT_REPORT_URL).strip()
        self.upload_timeout_sec = max(float(cfg.get("upload_timeout_sec") or 60.0), 1.0)
        self.default_source_map_id = str(cfg.get("default_source_map_id") or "42").strip()
        self.load_map_url = _resolve_load_map_url(config, cfg)
        self.sync_on_start = bool(cfg.get("sync_on_start", False))
        identity_file = str(cfg.get("identity_file") or DEFAULT_IDENTITY_FILE).strip()
        identity_path = Path(identity_file)
        if not identity_path.is_absolute():
            identity_path = PACKAGE_ROOT / identity_path
        self.identities = MapIdentityStore(identity_path)
        try:
            self._headers = json_request_headers(config)
        except ValueError as exc:
            raise MapSyncError("MAP_SYNC_AUTH_INVALID", str(exc)) from exc
        self._lock = threading.RLock()
        self._last: Dict[str, Any] = {}
        self._syncing = False
        self._ensure_skip_until = 0.0
        self._ensure_thread: Optional[threading.Thread] = None
        self._last_load_map_id = ""

    def on_gateway_start(self) -> None:
        self._sync_if_configured(reason="start", require_start_flag=True)

    def on_platform_ready(self) -> None:
        try:
            self._sync_if_configured(reason="mqtt-connect", require_start_flag=True)
        except Exception:
            LOGGER.warning(
                "map sync retry after MQTT connect failed",
                exc_info=True,
            )

    def ensure_active_map(self) -> None:
        if not self.enabled:
            return
        now = time.monotonic()
        with self._lock:
            if now < self._ensure_skip_until:
                return
            source = self._state_map_id()
            if not source or source == self._last_load_map_id:
                return
            if self._syncing:
                return
            thread = self._ensure_thread
            if thread is not None and thread.is_alive():
                return
            self._ensure_thread = threading.Thread(
                target=self._ensure_active_map_locked,
                kwargs={"source_map_id": source},
                name="map-sync-active",
                daemon=True,
            )
            thread = self._ensure_thread
        thread.start()

    def _ensure_active_map_locked(self, source_map_id: str = "") -> None:
        source = _normalize_source_id(source_map_id) or self._state_map_id()
        if not source:
            return
        try:
            result = self.sync(source_map_id=source)
        except Exception:
            self._ensure_skip_until = time.monotonic() + 30.0
            LOGGER.warning("map sync for active map failed", exc_info=True)
            return
        self._ensure_skip_until = 0.0
        with self._lock:
            self._last_load_map_id = str(result.get("source_map_id") or source)
        LOGGER.info(
            "map cloud upload requested: reason=active-map source=%s map=%s uploaded=%s",
            result.get("source_map_id"),
            result.get("map_id"),
            result.get("uploaded"),
        )

    def _sync_if_configured(
        self, *, reason: str, require_start_flag: bool = True
    ) -> None:
        if not self.enabled:
            return
        if require_start_flag and not self.sync_on_start:
            if reason == "start":
                LOGGER.info("map cloud upload not requested at gateway start")
            return
        with self._lock:
            if self._syncing:
                LOGGER.info("map sync already running; skip %s", reason)
                return
            source = self._upload_source_id()
            identity = self.identities.peek(source) if source else None
            # Start always refreshes occupancy. last_upload_ok only skips MQTT
            # reconnect retries so a successful start is not uploaded twice.
            if (
                reason != "start"
                and identity is not None
                and identity.last_upload_ok
            ):
                LOGGER.info(
                    "map cloud upload already ok: reason=%s source=%s map=%s",
                    reason,
                    source,
                    identity.map_id,
                )
                self._last_load_map_id = source
                return
            self._syncing = True
        try:
            result = self.sync()
        except MapSyncError as exc:
            self._ensure_skip_until = time.monotonic() + 30.0
            LOGGER.warning(
                "map sync on %s failed: code=%s error=%s",
                reason,
                exc.code,
                exc,
            )
            return
        finally:
            with self._lock:
                self._syncing = False
        self._ensure_skip_until = 0.0
        with self._lock:
            self._last_load_map_id = str(result.get("source_map_id") or source)
        LOGGER.info(
            "map cloud upload requested: reason=%s source=%s map=%s uploaded=%s",
            reason,
            result.get("source_map_id"),
            result.get("map_id"),
            result.get("uploaded"),
        )

    def platform_map_id(self, source_map_id: str) -> str:
        source = _normalize_source_id(source_map_id)
        mapped = self.identities.uploaded_map_id(source) if source else ""
        if self._navigation_ready():
            return mapped
        fallback = self.identities.uploaded_map_id(self.default_source_map_id)
        return fallback or mapped

    def status(self) -> Dict[str, Any]:
        current_source = self._upload_source_id()
        current = self.identities.peek(current_source)
        return {
            "enabled": self.enabled,
            "report_url": self.report_url,
            "load_map_url": self.load_map_url,
            "sync_on_start": self.sync_on_start,
            "identity_file": str(self.identities.path),
            "current_source_map_id": current_source,
            "current": current.as_dict() if current is not None else {},
            "items": [item.as_dict() for item in self.identities.items()],
            "last": dict(self._last),
        }

    def sync(
        self,
        *,
        source_map_id: str = "",
        resource: str = "",
        resource_token: str = "",
    ) -> Dict[str, Any]:
        if not self.enabled:
            raise MapSyncError("MAP_SYNC_DISABLED", "map sync is disabled")
        if not self.report_url:
            raise MapSyncError("MAP_SYNC_NO_URL", "map report_url is not configured")
        if not self.load_map_url:
            raise MapSyncError("MAP_SYNC_NO_URL", "nav load_map url is not configured")
        parsed = urllib.parse.urlparse(self.report_url)
        if parsed.hostname == "robotsolution.cn" and parsed.port == 1235:
            raise MapSyncError("MAP_SYNC_BAD_URL", "map report port 1235 is retired")

        source_id = self._upload_source_id(source_map_id)
        if not source_id:
            raise MapSyncError("MAP_SYNC_INVALID_SOURCE", "source_map_id is required")

        grid = self._fetch_nav_load_map(source_id)
        if grid is None:
            raise MapSyncError(
                "MAP_SYNC_BMAP_NOT_FOUND",
                f"nav load_map occupancy missing: source={source_id} url={self.load_map_url}",
                http_status=404,
            )
        source_ref = self.load_map_url
        now_ms = self._now_ms()
        identity = self.identities.resolve_or_create(
            source_id,
            now_ms=now_ms,
        )
        upload_payload = {
            "sn": str((self._config.get("device") or {}).get("sn") or ""),
            "timestamp": now_ms,
            "map_id": identity.map_id,
            "info": {
                "width": grid.width,
                "height": grid.height,
                "resolution": grid.resolution,
                "origin": {
                    "position": {"x": grid.origin_x, "y": grid.origin_y, "z": 0.0},
                    "orientation": {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0},
                },
            },
            "data": list(grid.data),
        }
        try:
            self._upload(
                upload_payload,
                headers=_merge_auth_headers(
                    self._headers,
                    resource=resource,
                    resource_token=resource_token,
                ),
            )
        except ValueError as exc:
            raise MapSyncError("MAP_SYNC_AUTH_INVALID", str(exc)) from exc
        except MapSyncError as exc:
            self.identities.mark_upload(
                source_id,
                ok=False,
                now_ms=now_ms,
                error=str(exc),
                source_path=source_ref,
            )
            self._set_last(
                {
                    "ok": False,
                    "source_map_id": source_id,
                    "map_id": identity.map_id,
                    "error": exc.code,
                    "message": str(exc),
                    "source_path": source_ref,
                }
            )
            raise

        updated = self.identities.mark_upload(
            source_id, ok=True, now_ms=now_ms, source_path=source_ref
        )
        result = {
            "source_map_id": source_id,
            "map_id": updated.map_id,
            "width": grid.width,
            "height": grid.height,
            "resolution": grid.resolution,
            "origin": {"x": grid.origin_x, "y": grid.origin_y},
            "cells": len(grid.data),
            "occupied": sum(1 for cell in grid.data if cell == 100),
            "uploaded": True,
            "source_path": source_ref,
        }
        self._set_last({"ok": True, **result})
        LOGGER.info(
            "map sync uploaded: sn=%s source=%s map=%s size=%sx%s resolution=%s path=%s",
            upload_payload["sn"],
            source_id,
            updated.map_id,
            grid.width,
            grid.height,
            grid.resolution,
            source_ref,
        )
        return result

    def _upload_source_id(self, requested: str = "") -> str:
        explicit = _normalize_source_id(requested)
        if explicit:
            return explicit
        return self._active_source_map_id()

    def _active_source_map_id(self) -> str:
        nav_id = self._state_map_id()
        if nav_id:
            return nav_id
        return _normalize_source_id(self.default_source_map_id)

    def _state_map_id(self) -> str:
        snapshot = self._collector_snapshot()
        raw_map = snapshot.get("map")
        if not isinstance(raw_map, Mapping):
            return ""
        map_id = _normalize_source_id(raw_map.get("map_id"))
        if not map_id or map_id.lower() == "no_map":
            return ""
        return map_id

    def _collector_snapshot(self) -> Mapping[str, Any]:
        if self._collector is None:
            return {}
        try:
            snapshot = self._collector.snapshot()
        except Exception:
            return {}
        return snapshot if isinstance(snapshot, Mapping) else {}

    def _navigation_ready(self, snapshot: Optional[Mapping[str, Any]] = None) -> bool:
        snap = self._collector_snapshot() if snapshot is None else snapshot
        check = snap.get("self_check")
        if not isinstance(check, Mapping):
            return False
        try:
            return int(check.get("status") or 0) == 0
        except (TypeError, ValueError):
            return False

    def _current_source_map_id(self) -> str:
        return self._active_source_map_id()

    def _fetch_nav_load_map(self, source_map_id: str) -> Optional[OccupancyGrid]:
        body = json.dumps({"map_id": source_map_id}).encode("utf-8")
        request = urllib.request.Request(
            self.load_map_url,
            data=body,
            method="POST",
            headers={
                "Accept": "application/json",
                "Content-Type": "application/json",
            },
        )
        LOGGER.info("nav load_map: %s map_id=%s", self.load_map_url, source_map_id)
        try:
            with self._opener(request, timeout=min(self.upload_timeout_sec, 30.0)) as response:
                raw = response.read()
        except urllib.error.HTTPError as exc:
            raise MapSyncError(
                "MAP_SYNC_BMAP_NOT_FOUND",
                f"nav load_map HTTP {exc.code}: {self.load_map_url}",
                http_status=404,
            ) from exc
        except urllib.error.URLError as exc:
            raise MapSyncError(
                "MAP_SYNC_BMAP_NOT_FOUND",
                f"nav load_map unreachable: {exc.reason}",
                http_status=502,
            ) from exc
        try:
            payload = json.loads(raw.decode("utf-8"))
        except (UnicodeDecodeError, json.JSONDecodeError) as exc:
            raise MapSyncError(
                "MAP_SYNC_BMAP_INVALID",
                "nav load_map returned non-JSON",
            ) from exc
        if not isinstance(payload, dict):
            raise MapSyncError("MAP_SYNC_BMAP_INVALID", "nav load_map returned non-object")
        if payload.get("accepted") is False:
            raise MapSyncError(
                "MAP_SYNC_BMAP_NOT_FOUND",
                str(payload.get("message") or payload.get("error_code") or "nav load_map rejected"),
                http_status=404,
            )
        grid = occupancy_from_nav_json(payload)
        if grid is not None:
            LOGGER.info(
                "nav load_map occupancy: source=%s size=%sx%s resolution=%s origin.x=%s origin.y=%s",
                source_map_id,
                grid.width,
                grid.height,
                grid.resolution,
                grid.origin_x,
                grid.origin_y,
            )
        return grid

    def _upload(self, map_payload: Mapping[str, Any], *, headers: Mapping[str, str]) -> None:
        body = {"data": json.dumps(dict(map_payload), ensure_ascii=False)}
        request = urllib.request.Request(
            self.report_url,
            data=json.dumps(body, ensure_ascii=False).encode("utf-8"),
            method="POST",
            headers=dict(headers),
        )
        try:
            with self._opener(request, timeout=self.upload_timeout_sec) as response:
                status = int(getattr(response, "status", None) or response.getcode() or 0)
                raw = response.read(64 * 1024 + 1)
        except urllib.error.HTTPError as exc:
            raw = exc.read() if exc.fp is not None else b""
            raise MapSyncError(
                "MAP_SYNC_UPLOAD_HTTP",
                f"map upload HTTP {exc.code}: {_preview(raw)}",
                http_status=502,
            ) from exc
        except (OSError, urllib.error.URLError, TimeoutError) as exc:
            raise MapSyncError(
                "MAP_SYNC_UPLOAD_FAILED",
                f"map upload failed: {exc}",
                http_status=502,
            ) from exc
        if status != 200:
            raise MapSyncError(
                "MAP_SYNC_UPLOAD_HTTP",
                f"map upload returned HTTP {status}",
                http_status=502,
            )
        if len(raw) > 64 * 1024:
            raise MapSyncError("MAP_SYNC_UPLOAD_HTTP", "map upload response is too large", http_status=502)
        try:
            payload = json.loads(raw.decode("utf-8")) if raw else {}
        except (UnicodeDecodeError, json.JSONDecodeError) as exc:
            raise MapSyncError(
                "MAP_SYNC_UPLOAD_HTTP",
                "map upload returned invalid JSON",
                http_status=502,
            ) from exc
        code = payload.get("code") if isinstance(payload, dict) else None
        if not isinstance(code, int) or isinstance(code, bool) or code != 0:
            raise MapSyncError(
                "MAP_SYNC_UPLOAD_REJECTED",
                f"map upload returned business error: {payload}",
                http_status=502,
            )

    def _now_ms(self) -> int:
        try:
            value = float(self._clock())
        except Exception:
            return 0
        return max(int(value * 1000), 0) if math.isfinite(value) else 0

    def _set_last(self, value: Mapping[str, Any]) -> None:
        with self._lock:
            self._last = dict(value)


def occupancy_from_nav_json(payload: Mapping[str, Any]) -> Optional[OccupancyGrid]:
    occ = payload.get("occupancy")
    if not isinstance(occ, Mapping):
        return None
    try:
        width = int(occ.get("width") or 0)
        height = int(occ.get("height") or 0)
        resolution = float(occ.get("resolution") or 0.0)
    except (TypeError, ValueError) as exc:
        raise MapSyncError(
            "MAP_SYNC_BMAP_INVALID",
            "nav occupancy geometry is invalid",
        ) from exc
    if width <= 0 or height <= 0 or resolution <= 0.0:
        raise MapSyncError("MAP_SYNC_BMAP_INVALID", "nav occupancy geometry is invalid")
    origin = occ.get("origin")
    if isinstance(origin, Mapping):
        origin_x = _required_finite(origin.get("x"), field="occupancy.origin.x")
        origin_y = _required_finite(origin.get("y"), field="occupancy.origin.y")
    elif isinstance(origin, (list, tuple)) and len(origin) >= 2:
        origin_x = _required_finite(origin[0], field="occupancy.origin.x")
        origin_y = _required_finite(origin[1], field="occupancy.origin.y")
    else:
        raise MapSyncError("MAP_SYNC_BMAP_INVALID", "nav occupancy origin is invalid")
    raw = occ.get("data")
    if not isinstance(raw, list) or len(raw) != width * height:
        raise MapSyncError("MAP_SYNC_BMAP_INVALID", "nav occupancy data size mismatch")
    cells: list[int] = []
    for item in raw:
        try:
            value = int(item)
        except (TypeError, ValueError):
            value = -1
        if value >= 50:
            cells.append(100)
        elif value == 0:
            cells.append(0)
        else:
            cells.append(-1)
    return OccupancyGrid(
        width=width,
        height=height,
        resolution=resolution,
        origin_x=origin_x,
        origin_y=origin_y,
        data=tuple(cells),
    )


def canonical_uuid(value: Any) -> str:
    try:
        return str(UUID(str(value or "").strip()))
    except (AttributeError, TypeError, ValueError):
        return ""


def rewrite_platform_map_id(source_map_id: str, service: Optional[MapSyncService]) -> str:
    source_id = str(source_map_id or "").strip()
    if not source_id or service is None:
        return source_id
    mapped = service.platform_map_id(source_id)
    return mapped or source_id


def _map_sync_config(config: Mapping[str, Any]) -> Mapping[str, Any]:
    cfg = config.get("map_sync") or {}
    return cfg if isinstance(cfg, Mapping) else {}


def _resolve_load_map_url(config: Mapping[str, Any], cfg: Mapping[str, Any]) -> str:
    raw = str(cfg.get("load_map_url") or "").strip()
    if raw.lower() not in {"", "auto", "*"}:
        return raw.rstrip("/")
    state = config.get("state") if isinstance(config.get("state"), Mapping) else {}
    nav = (state or {}).get("navigation")
    if isinstance(nav, str):
        base = nav.strip()
    elif isinstance(nav, Mapping):
        base = str(nav.get("url") or "").strip()
    else:
        base = ""
    if not base:
        return ""
    return f"{base.rstrip('/')}/load_map"


def _merge_auth_headers(
    base: Mapping[str, str],
    *,
    resource: str = "",
    resource_token: str = "",
) -> Dict[str, str]:
    headers = dict(base)
    res = str(resource or "").strip()
    token = str(resource_token or "").strip()
    if not res and not token:
        return headers
    overlay = json_request_headers(
        {"platform_api": {"resource": res, "resource_token": token}}
    )
    headers.update(overlay)
    return headers


def _normalize_source_id(value: Any) -> str:
    return str(value or "").strip()


def _optional_finite(value: Any) -> Optional[float]:
    if value is None or value == "" or isinstance(value, bool):
        return None
    try:
        number = float(value)
    except (TypeError, ValueError, OverflowError):
        return None
    return number if math.isfinite(number) else None


def _required_finite(value: Any, *, field: str) -> float:
    number = _optional_finite(value)
    if number is None:
        raise MapSyncError("MAP_SYNC_BMAP_INVALID", f"bmap {field} must be finite")
    return number


def _preview(raw: bytes) -> str:
    try:
        text = raw.decode("utf-8", errors="replace")
    except Exception:
        return ""
    return text[:200]
