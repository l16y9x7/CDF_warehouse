"""Station catalog loader / writer."""

from __future__ import annotations

from dataclasses import asdict, dataclass
from pathlib import Path
from typing import Any, Dict, Iterable, List, Optional

import yaml


@dataclass(frozen=True)
class Station:
    station_id: str
    display_name: str
    x: float
    y: float
    yaw: float
    arrive_tol_m: float = 0.08
    yaw_tol_rad: float = 0.17
    max_speed: float = 0.5
    vendor_target_id: int = 0
    align_enable: bool = True


def align_enabled_for_station(global_enable: bool, station: Station | None) -> bool:
    """Global switch plus optional per-station opt-out."""
    if not global_enable:
        return False
    if station is not None and not station.align_enable:
        return False
    return True


@dataclass
class StationCatalog:
    map_name: str
    stations: Dict[str, Station]

    def get(self, station_id: str) -> Optional[Station]:
        return self.stations.get(str(station_id or "").strip())

    def list(self) -> List[Station]:
        return list(self.stations.values())

    def ids(self) -> Iterable[str]:
        return self.stations.keys()


def _as_bool(value: Any, default: bool) -> bool:
    if value is None:
        return default
    if isinstance(value, bool):
        return value
    text = str(value).strip().lower()
    if text in {"1", "true", "yes", "on"}:
        return True
    if text in {"0", "false", "no", "off"}:
        return False
    return default


def _station_from_raw(station_id: str, raw: Optional[Dict[str, Any]]) -> Station:
    raw = raw or {}
    return Station(
        station_id=str(station_id),
        display_name=str(raw.get("display_name") or station_id),
        x=float(raw.get("x", 0.0)),
        y=float(raw.get("y", 0.0)),
        yaw=float(raw.get("yaw", 0.0)),
        arrive_tol_m=float(raw.get("arrive_tol_m", 0.08)),
        yaw_tol_rad=float(raw.get("yaw_tol_rad", 0.17)),
        max_speed=float(raw.get("max_speed", 0.5)),
        vendor_target_id=int(raw.get("vendor_target_id", 0)),
        align_enable=_as_bool(raw.get("align_enable"), True),
    )


def catalog_from_dict(data: Dict[str, Any]) -> StationCatalog:
    map_name = str(data.get("map_name") or "retail_default")
    raw_stations = data.get("stations") or {}
    stations: Dict[str, Station] = {}
    for station_id, raw in raw_stations.items():
        stations[str(station_id)] = _station_from_raw(str(station_id), raw)
    return StationCatalog(map_name=map_name, stations=stations)


def load_station_catalog(path: str | Path) -> StationCatalog:
    data = yaml.safe_load(Path(path).read_text(encoding="utf-8")) or {}
    if not isinstance(data, dict):
        raise ValueError(f"stations file must be a mapping: {path}")
    return catalog_from_dict(data)


def catalog_to_dict(catalog: StationCatalog) -> Dict[str, Any]:
    stations: Dict[str, Any] = {}
    for sid, st in catalog.stations.items():
        payload = asdict(st)
        payload.pop("station_id", None)
        if payload.get("align_enable", True) is True:
            payload.pop("align_enable", None)
        stations[sid] = payload
    return {"map_name": catalog.map_name, "stations": stations}


def save_station_catalog(path: str | Path, catalog: StationCatalog) -> Path:
    """Write catalog to YAML (atomic replace)."""
    target = Path(path)
    target.parent.mkdir(parents=True, exist_ok=True)
    body = yaml.safe_dump(
        catalog_to_dict(catalog),
        allow_unicode=True,
        sort_keys=False,
        default_flow_style=False,
    )
    header = (
        "# Auto-synced station catalog. Prefer SyncStations over manual edits "
        "while the bridge is running.\n"
    )
    tmp = target.with_suffix(target.suffix + ".tmp")
    tmp.write_text(header + body, encoding="utf-8")
    tmp.replace(target)
    return target
