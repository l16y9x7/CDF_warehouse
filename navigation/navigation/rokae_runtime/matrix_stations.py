"""从 MATRIX 导出包抽出站点。启动拉一次，和 load_map / 切图分开。

GET {base}/api/v0/map/{map_name}/export?type=FMS&packed=0
现场是 gzip+tar，里面 sros/map/{map}.json 才有 data.station。
"""

from __future__ import annotations

import hashlib
import gzip
import io
import json
import logging
import tarfile
import zipfile
from pathlib import Path
from typing import Any, Dict, List, Optional
from urllib.error import HTTPError, URLError
from urllib.parse import quote, urlencode
from urllib.request import Request, urlopen

LOGGER = logging.getLogger(__name__)


def stations_from_topology(doc: Dict[str, Any]) -> List[Dict[str, Any]]:
    meta = doc.get("meta") if isinstance(doc.get("meta"), dict) else {}
    mm = str(meta.get("length_unit") or "").strip().lower() in {"mm", "millimeter", "millimetre"}
    data = doc.get("data") if isinstance(doc.get("data"), dict) else {}
    raw = data.get("station") if isinstance(data.get("station"), list) else []
    rows: List[Dict[str, Any]] = []
    for item in raw:
        if not isinstance(item, dict):
            continue
        vendor_id = str(item.get("id") or "").strip()
        vendor_name = str(item.get("name") or "").strip()
        if not vendor_id and not vendor_name:
            continue
        if not vendor_id:
            vendor_id = vendor_name
        station_id = _public_name(vendor_name, vendor_id)
        aliases = []
        for token in (station_id, vendor_name, vendor_id):
            if token and token not in aliases:
                aliases.append(token)
        x, y, yaw = _pose(item, mm=mm)
        rows.append(
            {
                "station_id": station_id,
                "vendor_id": vendor_id,
                "x": x,
                "y": y,
                "yaw": yaw,
                "aliases": aliases,
            }
        )
    if not rows:
        raise ValueError("MATRIX topology has no data.station")
    return rows


def nodes_from_topology(doc: Dict[str, Any]) -> List[Dict[str, Any]]:
    mm = _length_is_mm(doc)
    data = doc.get("data") if isinstance(doc.get("data"), dict) else {}
    raw = data.get("node") if isinstance(data.get("node"), list) else []
    rows: List[Dict[str, Any]] = []
    for item in raw:
        if not isinstance(item, dict) or item.get("id") is None:
            continue
        rows.append(
            {
                "id": item.get("id"),
                "x": _length(item.get("x"), mm=mm),
                "y": _length(item.get("y"), mm=mm),
                "yaw": _angle(item.get("yaw")),
                "desc": str(item.get("desc") or ""),
            }
        )
    return rows


def edges_from_topology(doc: Dict[str, Any]) -> List[Dict[str, Any]]:
    """路径。坐标、代价跟站点一样：毫米换成米。朝向绝对值大于 10 当千分之一弧度。"""
    mm = _length_is_mm(doc)
    data = doc.get("data") if isinstance(doc.get("data"), dict) else {}
    raw = data.get("edge") if isinstance(data.get("edge"), list) else []
    rows: List[Dict[str, Any]] = []
    for item in raw:
        if not isinstance(item, dict) or item.get("id") is None:
            continue
        row: Dict[str, Any] = {}
        for key in (
            "id",
            "type",
            "s_node",
            "e_node",
            "limit_v",
            "limit_w",
            "rotate_direction",
            "direction",
            "robot_direction",
            "param",
            "is_back_edge",
            "desc",
            "path_behavior",
            "user_define_properties",
        ):
            if key in item:
                row[key] = item[key]
        for key in ("sx", "sy", "ex", "ey", "cx", "cy", "dx", "dy", "cost", "radius"):
            if key in item:
                row[key] = _length(item[key], mm=mm)
        for key in ("s_facing", "e_facing", "orientation"):
            if key in item:
                row[key] = _angle(item[key])
        rows.append(row)
    return rows


def topology_fingerprint(doc: Dict[str, Any], *, map_name: str = "") -> str:
    """FMS 拓扑指纹：图号、meta 时间/尺寸、站点位姿。用来发现 MATRIX 改图改站。"""
    meta = doc.get("meta") if isinstance(doc.get("meta"), dict) else {}
    try:
        rows = stations_from_topology(doc)
    except ValueError:
        rows = []
    stations = tuple(
        (
            str(row.get("station_id") or ""),
            str(row.get("vendor_id") or ""),
            round(float(row.get("x") or 0.0), 4),
            round(float(row.get("y") or 0.0), 4),
            round(float(row.get("yaw") or 0.0), 4),
        )
        for row in rows
    )
    payload = (
        str(map_name or ""),
        str(meta.get("modified_timestamp") or ""),
        str(meta.get("created_timestamp") or ""),
        str(meta.get("resolution") or ""),
        str(meta.get("size.x") or meta.get("width") or ""),
        str(meta.get("size.y") or meta.get("height") or ""),
        stations,
        tuple(
            (
                row.get("id"),
                row.get("s_node"),
                row.get("e_node"),
                row.get("sx"),
                row.get("sy"),
                row.get("ex"),
                row.get("ey"),
                row.get("robot_direction"),
            )
            for row in edges_from_topology(doc)
        ),
    )
    return hashlib.sha256(repr(payload).encode("utf-8")).hexdigest()[:16]


def maps_from_catalog(doc: Any) -> List[Dict[str, Any]]:
    """MATRIX 网页地图目录：GET /api/v0/map → {maps:[{name, md5, modify_time, map_version}]}。"""
    if isinstance(doc, list):
        items = doc
    elif isinstance(doc, dict):
        raw = doc.get("maps") or doc.get("data") or doc.get("list") or []
        items = raw if isinstance(raw, list) else []
    else:
        items = []
    rows: List[Dict[str, Any]] = []
    for item in items:
        if isinstance(item, str):
            rows.append({"name": item, "md5": "", "modify_time": "", "map_version": ""})
            continue
        if not isinstance(item, dict):
            continue
        name = str(item.get("name") or item.get("map_name") or item.get("mapName") or "").strip()
        if not name:
            continue
        rows.append(
            {
                "name": name,
                "md5": str(item.get("md5") or item.get("md5sum") or "").strip(),
                "modify_time": str(
                    item.get("modify_time")
                    or item.get("last_write_time")
                    or item.get("modified_timestamp")
                    or ""
                ).strip(),
                "map_version": str(item.get("map_version") or item.get("version") or "").strip(),
                "size": item.get("size") or 0,
            }
        )
    return rows


def map_file_revision(entry: Dict[str, Any], *, map_name: str = "") -> str:
    """网页保存后 md5 / modify_time 会变；图名不变也能发现改站改图。"""
    payload = (
        str(map_name or entry.get("name") or ""),
        str(entry.get("md5") or entry.get("md5sum") or ""),
        str(entry.get("modify_time") or entry.get("last_write_time") or ""),
        str(entry.get("map_version") or ""),
        str(entry.get("size") or ""),
    )
    return hashlib.sha256(repr(payload).encode("utf-8")).hexdigest()[:16]


def catalog_revision(maps: List[Dict[str, Any]], *, map_name: str) -> str:
    want = str(map_name or "").strip()
    if not want:
        return ""
    for row in maps:
        name = str(row.get("name") or "").strip()
        if name == want or Path(name).stem == Path(want).stem:
            return map_file_revision(row, map_name=want)
    return ""


def fetch_map_catalog(*, base_url: str, timeout_sec: float = 5.0) -> List[Dict[str, Any]]:
    base = str(base_url or "").rstrip("/")
    if not base:
        raise ValueError("matrix base_url is required")
    url = f"{base}/api/v0/map"
    LOGGER.debug("matrix map catalog: %s", url)
    request = Request(url, method="GET", headers={"Accept": "application/json"})
    try:
        with urlopen(request, timeout=timeout_sec) as response:
            raw = response.read()
    except HTTPError as exc:
        raise RuntimeError(f"MATRIX catalog HTTP {exc.code}: {url}") from exc
    except (URLError, TimeoutError, OSError) as exc:
        raise RuntimeError(f"MATRIX catalog failed: {exc}") from exc
    try:
        doc = json.loads(raw.decode("utf-8"))
    except (UnicodeDecodeError, json.JSONDecodeError) as exc:
        raise ValueError("MATRIX catalog is not JSON") from exc
    return maps_from_catalog(doc)


def catalog_from_rokae_config(config: Dict[str, Any], *, map_name: str = "") -> str:
    rokae = config.get("rokae") if isinstance(config.get("rokae"), dict) else {}
    matrix = rokae.get("matrix") if isinstance(rokae.get("matrix"), dict) else {}
    if str(matrix.get("topology_path") or "").strip():
        return ""
    sros = rokae.get("sros") if isinstance(rokae.get("sros"), dict) else {}
    if bool(sros.get("sim")) or str(rokae.get("backend") or "") == "mock":
        return ""
    base = str(matrix.get("base_url") or sros.get("host") or "192.168.71.50").strip()
    if base and "://" not in base:
        base = "http://" + base
    name = str(matrix.get("map_name") or map_name or rokae.get("map_id") or "").strip()
    timeout = float(matrix.get("timeout_sec") or config.get("http_timeout_sec") or 8.0)
    return catalog_revision(fetch_map_catalog(base_url=base, timeout_sec=timeout), map_name=name)


def load_topology_file(path: str | Path) -> Dict[str, Any]:
    return decode_topology_bytes(Path(path).read_bytes())


def fetch_topology(
    *,
    base_url: str,
    map_name: str,
    export_type: str = "FMS",
    packed: bool = False,
    timeout_sec: float = 8.0,
) -> Dict[str, Any]:
    base = str(base_url or "").rstrip("/")
    name = quote(str(map_name or "").strip(), safe="")
    if not base or not name:
        raise ValueError("matrix base_url and map_name are required")
    query = urlencode({"type": export_type or "FMS", "packed": 1 if packed else 0})
    url = f"{base}/api/v0/map/{name}/export?{query}"
    LOGGER.debug("matrix export stations: %s", url)
    request = Request(url, method="GET", headers={"Accept": "application/json, application/octet-stream"})
    try:
        with urlopen(request, timeout=timeout_sec) as response:
            raw = response.read()
    except HTTPError as exc:
        raise RuntimeError(f"MATRIX export HTTP {exc.code}: {url}") from exc
    except (URLError, TimeoutError, OSError) as exc:
        raise RuntimeError(f"MATRIX export failed: {exc}") from exc
    return decode_topology_bytes(raw)


def decode_topology_bytes(raw: bytes) -> Dict[str, Any]:
    payload = _unwrap(raw)
    try:
        doc = json.loads(payload.decode("utf-8"))
    except (UnicodeDecodeError, json.JSONDecodeError) as exc:
        raise ValueError("MATRIX export is not JSON topology") from exc
    if not isinstance(doc, dict):
        raise ValueError("MATRIX topology must be a JSON object")
    return doc


def _unwrap(raw: bytes) -> bytes:
    if raw[:2] == b"\x1f\x8b":
        raw = gzip.decompress(raw)
    if raw[:2] == b"PK":
        with zipfile.ZipFile(io.BytesIO(raw)) as archive:
            names = [n for n in archive.namelist() if n.endswith(".json") and not n.endswith("/")]
            if not names:
                raise ValueError("MATRIX zip has no json")
            return archive.read(names[0])
    extracted = _tar_topology_json(raw)
    if extracted is not None:
        return extracted
    return raw


def _tar_topology_json(raw: bytes) -> Optional[bytes]:
    if len(raw) < 262:
        return None
    try:
        archive = tarfile.open(fileobj=io.BytesIO(raw), mode="r:")
    except tarfile.TarError:
        return None
    with archive:
        members = [item for item in archive.getmembers() if item.isfile() and item.name.endswith(".json")]
        if not members:
            return None
        chosen = None
        for member in members:
            handle = archive.extractfile(member)
            if handle is None:
                continue
            blob = handle.read()
            try:
                doc = json.loads(blob.decode("utf-8"))
            except (UnicodeDecodeError, json.JSONDecodeError):
                continue
            data = doc.get("data") if isinstance(doc, dict) else None
            if isinstance(data, dict) and isinstance(data.get("station"), list):
                return blob
            if chosen is None:
                chosen = blob
        return chosen


def _public_name(vendor_name: str, vendor_id: str) -> str:
    if "-" in vendor_name:
        tail = vendor_name.rsplit("-", 1)[-1].strip()
        if tail:
            return tail
    return vendor_name or vendor_id


def _length_is_mm(doc: Dict[str, Any]) -> bool:
    meta = doc.get("meta") if isinstance(doc.get("meta"), dict) else {}
    return str(meta.get("length_unit") or "").strip().lower() in {"mm", "millimeter", "millimetre"}


def _length(value: Any, *, mm: bool) -> float:
    return float(value or 0.0) * (0.001 if mm else 1.0)


def _angle(value: Any) -> float:
    yaw = float(value or 0.0)
    if abs(yaw) > 10:
        yaw = yaw / 1000.0
    return yaw


def _pose(item: Dict[str, Any], *, mm: bool) -> tuple[float, float, float]:
    return (
        _length(item.get("pos.x") or item.get("x"), mm=mm),
        _length(item.get("pos.y") or item.get("y"), mm=mm),
        _angle(item.get("pos.yaw") or item.get("yaw")),
    )


def topology_from_rokae_config(
    config: Dict[str, Any], *, map_name: str = ""
) -> Optional[Dict[str, Any]]:
    rokae = config.get("rokae") if isinstance(config.get("rokae"), dict) else {}
    matrix = rokae.get("matrix") if isinstance(rokae.get("matrix"), dict) else {}
    path = str(matrix.get("topology_path") or "").strip()
    if path:
        file_path = Path(path)
        if not file_path.is_absolute():
            file_path = Path(__file__).resolve().parent.parent.parent / path
        LOGGER.info("matrix stations from file: %s", file_path)
        return load_topology_file(file_path)
    sros = rokae.get("sros") if isinstance(rokae.get("sros"), dict) else {}
    if bool(sros.get("sim")):
        return None
    name = str(matrix.get("map_name") or map_name or rokae.get("map_id") or "").strip()
    base = str(matrix.get("base_url") or sros.get("host") or "192.168.71.50").strip()
    if base and "://" not in base:
        base = "http://" + base
    return fetch_topology(
        base_url=base,
        map_name=name,
        export_type=str(matrix.get("export_type") or "FMS"),
        packed=bool(matrix.get("packed")),
        timeout_sec=float(matrix.get("timeout_sec") or config.get("http_timeout_sec") or 15.0),
    )
