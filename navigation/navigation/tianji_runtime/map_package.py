"""Map package: map metadata + stations.yaml (sync is explicit, not on LoadMap)."""

from __future__ import annotations

import shutil
from dataclasses import dataclass
from datetime import datetime, timezone
from pathlib import Path
import re
from typing import Any, Dict, Optional

import yaml

from navigation.tianji_runtime.stations import (
    StationCatalog,
    catalog_to_dict,
    load_station_catalog,
    save_station_catalog,
)

MANIFEST_NAME = "manifest.yaml"
STATIONS_NAME = "stations.yaml"
VENDOR_MAP_FILES = {
    0: "map.bmap",
    1: "map.png",
    2: "map.xml",
    3: "map.pcd",
}


@dataclass
class MapPackageManifest:
    map_name: str = ""
    map_id: str = ""
    map_path: str = ""
    created_at: str = ""
    notes: str = ""

    @classmethod
    def from_dict(cls, data: Dict[str, Any]) -> "MapPackageManifest":
        data = data or {}
        return cls(
            map_name=str(data.get("map_name") or ""),
            map_id=str(data.get("map_id") or ""),
            map_path=str(data.get("map_path") or ""),
            created_at=str(data.get("created_at") or ""),
            notes=str(data.get("notes") or ""),
        )

    def to_dict(self) -> Dict[str, Any]:
        return {
            "map_name": self.map_name,
            "map_id": self.map_id,
            "map_path": self.map_path,
            "created_at": self.created_at,
            "notes": self.notes,
        }


def resolve_stations_path(package_or_stations: str | Path) -> Path:
    """Accept a map package directory or a stations.yaml path."""
    path = Path(package_or_stations).expanduser().resolve()
    if path.is_dir():
        candidate = path / STATIONS_NAME
        if not candidate.is_file():
            raise FileNotFoundError(f"package missing {STATIONS_NAME}: {path}")
        return candidate
    if path.is_file():
        return path
    raise FileNotFoundError(f"stations source not found: {path}")


def load_package_stations(package_or_stations: str | Path) -> StationCatalog:
    return load_station_catalog(resolve_stations_path(package_or_stations))


def load_package_manifest(package_dir: str | Path) -> Optional[MapPackageManifest]:
    path = Path(package_dir).expanduser().resolve()
    manifest_path = path / MANIFEST_NAME if path.is_dir() else path.parent / MANIFEST_NAME
    if not manifest_path.is_file():
        return None
    data = yaml.safe_load(manifest_path.read_text(encoding="utf-8")) or {}
    if not isinstance(data, dict):
        return None
    return MapPackageManifest.from_dict(data)


def export_map_package(
    output_dir: str | Path,
    catalog: StationCatalog,
    *,
    map_id: str = "",
    map_name: str = "",
    map_path: str = "",
    copy_map_files: bool = False,
    notes: str = "",
) -> Path:
    """Write a map package directory containing manifest.yaml + stations.yaml.

    If copy_map_files and map_path points to an existing file/dir, copies into
    package ``maps/`` and rewrites manifest.map_path to the relative copy.
    """
    out = Path(output_dir).expanduser().resolve()
    if out.exists() and not out.is_dir():
        raise NotADirectoryError(f"output_dir is not a directory: {out}")
    out.mkdir(parents=True, exist_ok=True)

    name = map_name or catalog.map_name or "retail_default"
    packaged_map_path = map_path
    if copy_map_files and map_path:
        src = Path(map_path).expanduser()
        maps_dir = out / "maps"
        maps_dir.mkdir(parents=True, exist_ok=True)
        if src.is_file():
            dest = maps_dir / src.name
            shutil.copy2(src, dest)
            packaged_map_path = str(Path("maps") / src.name)
        elif src.is_dir():
            dest = maps_dir / src.name
            if dest.exists():
                shutil.rmtree(dest)
            shutil.copytree(src, dest)
            packaged_map_path = str(Path("maps") / src.name)

    catalog_out = StationCatalog(map_name=name, stations=dict(catalog.stations))
    save_station_catalog(out / STATIONS_NAME, catalog_out)

    manifest = MapPackageManifest(
        map_name=name,
        map_id=str(map_id or ""),
        map_path=str(packaged_map_path or ""),
        created_at=datetime.now(timezone.utc).isoformat(),
        notes=notes,
    )
    (out / MANIFEST_NAME).write_text(
        yaml.safe_dump(manifest.to_dict(), allow_unicode=True, sort_keys=False),
        encoding="utf-8",
    )
    return out


def sync_stations_to_file(
    source_package_or_stations: str | Path,
    target_stations_file: str | Path,
) -> StationCatalog:
    """Load stations from package/file and write into target retail_stations.yaml."""
    catalog = load_package_stations(source_package_or_stations)
    save_station_catalog(target_stations_file, catalog)
    return catalog


def _safe_package_name(map_name: str) -> str:
    value = re.sub(r"[^A-Za-z0-9_.-]+", "_", map_name.strip())
    return value.strip("._") or "vendor_map"


def _write_bytes_atomic(path: Path, data: bytes) -> None:
    tmp = path.with_suffix(path.suffix + ".tmp")
    tmp.write_bytes(data)
    tmp.replace(path)


def export_vendor_map_package(
    output_root: str | Path,
    map_data: Dict[int, bytes],
    *,
    map_id: str = "",
    map_name: str = "",
    notes: str = "",
) -> tuple[Path, Dict[int, str]]:
    """Write native TX-S2 map data into a timestamped package directory.

    ``map_data`` keys follow server/GetMapInfo.map_type. bmap (type 0) is
    required because it is the vendor-native map artifact; PNG/XML/PCD are
    exported when the vendor stack provides them. This function never reads or
    writes business station YAML files.
    """
    if not map_data.get(0):
        raise ValueError("vendor bmap (map_type=0) is required")

    root = Path(output_root).expanduser().resolve()
    root.mkdir(parents=True, exist_ok=True)
    timestamp = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ")
    package_dir = root / f"{_safe_package_name(map_name)}_{timestamp}"
    suffix = 1
    while package_dir.exists():
        package_dir = root / f"{_safe_package_name(map_name)}_{timestamp}_{suffix}"
        suffix += 1
    package_dir.mkdir()

    statuses: Dict[int, str] = {}
    exported_files = []
    for map_type, filename in VENDOR_MAP_FILES.items():
        data = map_data.get(map_type)
        if data:
            _write_bytes_atomic(package_dir / filename, data)
            statuses[map_type] = "exported"
            exported_files.append(filename)
        else:
            statuses[map_type] = "unavailable"

    manifest = {
        "artifact_type": "tx_s2_vendor_map",
        "map_name": map_name,
        "map_id": str(map_id or ""),
        "created_at": datetime.now(timezone.utc).isoformat(),
        "notes": notes,
        "files": exported_files,
        "formats": {
            "bmap": statuses[0],
            "png": statuses[1],
            "xml": statuses[2],
            "pcd": statuses[3],
        },
        "point_data_note": (
            "The bmap is the vendor-native export. The vendor API does not "
            "document a separate navigation-point export or bmap schema."
        ),
    }
    (package_dir / MANIFEST_NAME).write_text(
        yaml.safe_dump(manifest, allow_unicode=True, sort_keys=False),
        encoding="utf-8",
    )
    return package_dir, statuses
