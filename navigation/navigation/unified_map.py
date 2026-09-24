"""统一地图包：站点表与厂家格式解耦。"""

from __future__ import annotations

import json
import logging
from pathlib import Path
from typing import Any, Dict, List, Optional

LOGGER = logging.getLogger(__name__)

PACKAGE_ROOT = Path(__file__).resolve().parent.parent
DEFAULT_MAPS_DIR = PACKAGE_ROOT / "maps"


def maps_dir(config: Optional[Dict[str, Any]] = None) -> Path:
    raw = ""
    if isinstance(config, dict):
        raw = str(config.get("maps_dir") or "")
    return Path(raw) if raw else DEFAULT_MAPS_DIR


def canonicalize(map_id: str, stations: List[Any]) -> Dict[str, Any]:
    map_id = str(map_id or "").strip()
    if not map_id:
        raise ValueError("map_id is required")
    rows: List[Dict[str, Any]] = []
    for index, raw in enumerate(stations or [], start=1):
        if not isinstance(raw, dict):
            continue
        station_id = str(
            raw.get("station_id") or raw.get("name") or raw.get("id") or ""
        ).strip()
        if not station_id:
            continue
        vendor_id = str(raw.get("vendor_id") or raw.get("id") or index).strip()
        aliases = [str(item).strip() for item in (raw.get("aliases") or []) if str(item).strip()]
        row: Dict[str, Any] = {
            "station_id": station_id,
            "vendor_id": vendor_id,
            "x": float(raw.get("x") or 0.0),
            "y": float(raw.get("y") or 0.0),
            "yaw": float(raw.get("yaw") or 0.0),
        }
        if aliases:
            row["aliases"] = aliases
        rows.append(row)
    if not rows:
        raise ValueError("stations is empty")
    return {"map_id": map_id, "stations": rows}


def from_snapshot(snap: Dict[str, Any], *, map_id: str = "") -> Dict[str, Any]:
    return canonicalize(
        str(map_id or snap.get("map_id") or "").strip(),
        snap.get("stations") if isinstance(snap.get("stations"), list) else [],
    )


def to_rokae_stations(payload: Dict[str, Any]) -> List[Dict[str, Any]]:
    rows: List[Dict[str, Any]] = []
    for item in payload.get("stations") or []:
        station_id = str(item.get("station_id") or "")
        vendor_id = str(item.get("vendor_id") or station_id)
        aliases = list(item.get("aliases") or [])
        if station_id and station_id not in aliases:
            aliases.append(station_id)
        rows.append(
            {
                "id": vendor_id,
                "name": station_id,
                "aliases": aliases,
                "x": float(item.get("x") or 0.0),
                "y": float(item.get("y") or 0.0),
                "yaw": float(item.get("yaw") or 0.0),
            }
        )
    return rows


def to_public_stations(payload: Dict[str, Any]) -> List[Dict[str, Any]]:
    out: List[Dict[str, Any]] = []
    for item in payload.get("stations") or []:
        row = {
            "station_id": str(item.get("station_id") or ""),
            "x": float(item.get("x") or 0.0),
            "y": float(item.get("y") or 0.0),
            "yaw": float(item.get("yaw") or 0.0),
        }
        if item.get("vendor_id"):
            row["vendor_id"] = str(item["vendor_id"])
        out.append(row)
    return out


def map_path(directory: Path, map_id: str) -> Path:
    safe = "".join(ch if ch.isalnum() or ch in "._-" else "_" for ch in map_id)
    return directory / f"{safe}.json"


def read_map(directory: Path, map_id: str) -> Dict[str, Any]:
    path = map_path(directory, map_id)
    with path.open("r", encoding="utf-8") as handle:
        data = json.load(handle)
    if not isinstance(data, dict):
        raise ValueError("map file must be a JSON object")
    return canonicalize(str(data.get("map_id") or map_id), data.get("stations") or [])


def write_map(directory: Path, payload: Dict[str, Any]) -> Path:
    directory.mkdir(parents=True, exist_ok=True)
    path = map_path(directory, str(payload["map_id"]))
    with path.open("w", encoding="utf-8") as handle:
        json.dump(payload, handle, ensure_ascii=False, indent=2)
        handle.write("\n")
    LOGGER.info("unified map saved: %s", path)
    return path


def occupancy_path(directory: Path, map_id: str) -> Path:
    return map_path(directory, map_id).with_name(map_path(directory, map_id).stem + ".occupancy.json")


def read_occupancy(directory: Path, map_id: str) -> Dict[str, Any]:
    from navigation.occupancy import maybe_occupancy

    path = occupancy_path(directory, map_id)
    with path.open("r", encoding="utf-8") as handle:
        data = json.load(handle)
    grid = maybe_occupancy(data)
    if not grid:
        raise ValueError(f"invalid occupancy file: {path}")
    return grid


def write_occupancy(directory: Path, map_id: str, grid: Dict[str, Any]) -> Path:
    from navigation.occupancy import maybe_occupancy, occupancy_revision

    payload = maybe_occupancy(grid)
    if not payload:
        raise ValueError("occupancy payload is invalid")
    directory.mkdir(parents=True, exist_ok=True)
    path = occupancy_path(directory, map_id)
    body = dict(payload)
    body["map_id"] = str(map_id)
    body["revision"] = occupancy_revision(payload)
    with path.open("w", encoding="utf-8") as handle:
        json.dump(body, handle, ensure_ascii=False, separators=(",", ":"))
        handle.write("\n")
    LOGGER.info("occupancy saved: %s revision=%s cells=%s", path, body["revision"], len(payload["data"]))
    return path
