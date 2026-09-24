"""从 MATRIX 导出包抽出 ROS 占用栅格。站点走 FMS JSON；栅格也在同一份 FMS 包的 pgm 里。"""

from __future__ import annotations

import gzip
import io
import json
import logging
import tarfile
import zipfile
from pathlib import Path
from typing import Any, Dict, Optional
from urllib.error import HTTPError, URLError
from urllib.parse import quote, urlencode
from urllib.request import Request, urlopen

from navigation.occupancy import (
    occupancy_from_gray,
    occupancy_from_pgm,
    occupancy_from_png,
    sros_gray_to_occ,
)
from navigation.rokae_runtime.matrix_stations import decode_topology_bytes

LOGGER = logging.getLogger(__name__)


def occupancy_from_sros_map(doc: Dict[str, Any], image: bytes, *, image_name: str = "") -> Dict[str, Any]:
    meta = doc.get("meta") if isinstance(doc.get("meta"), dict) else {}
    width, height = _size(meta)
    resolution = _resolution_m(meta)
    origin = _origin_m(meta, width=width, height=height, resolution=resolution)
    unit = str(meta.get("length_unit") or "mm").strip().lower()
    sros_pgm = unit in {"mm", "millimeter", "millimetre"}
    suffix = Path(image_name).suffix.lower()
    if suffix == ".pgm" or image[:2] == b"P5":
        grid = occupancy_from_pgm(
            image,
            resolution=resolution,
            origin=origin,
            to_occ=sros_gray_to_occ if sros_pgm else None,
        )
    else:
        grid = occupancy_from_png(image, resolution=resolution, origin=origin)
    if width and height and (grid["width"] != width or grid["height"] != height):
        LOGGER.warning(
            "occupancy image size %sx%s != meta %sx%s",
            grid["width"],
            grid["height"],
            width,
            height,
        )
    return grid


def occupancy_from_export_bytes(raw: bytes) -> Dict[str, Any]:
    doc, image, name = _split_export(raw)
    LOGGER.info("matrix occupancy raster: %s (%s bytes)", name, len(image))
    return occupancy_from_sros_map(doc, image, image_name=name)


def occupancy_from_gray_and_meta(doc: Dict[str, Any], pixels: list[int]) -> Dict[str, Any]:
    meta = doc.get("meta") if isinstance(doc.get("meta"), dict) else {}
    width, height = _size(meta)
    if width * height != len(pixels):
        raise ValueError("gray pixels do not match meta size")
    resolution = _resolution_m(meta)
    origin = _origin_m(meta, width=width, height=height, resolution=resolution)
    return occupancy_from_gray(
        pixels,
        width=width,
        height=height,
        resolution=resolution,
        origin=origin,
        to_occ=sros_gray_to_occ,
    )


def fetch_occupancy(
    *,
    base_url: str,
    map_name: str,
    timeout_sec: float = 30.0,
    export_types: tuple[str, ...] = ("FMS", "NAV", "ALL"),
) -> Dict[str, Any]:
    base = str(base_url or "").rstrip("/")
    name = str(map_name or "").strip()
    if not base or not name:
        raise ValueError("matrix base_url and map_name are required")
    last_error: Optional[Exception] = None
    for export_type in export_types:
        query = urlencode({"type": export_type, "packed": 0 if export_type == "FMS" else 1})
        url = f"{base}/api/v0/map/{quote(name, safe='')}/export?{query}"
        LOGGER.info("matrix export occupancy: %s", url)
        try:
            with urlopen(
                Request(url, method="GET", headers={"Accept": "application/octet-stream, application/json"}),
                timeout=timeout_sec,
            ) as response:
                raw = response.read()
            return occupancy_from_export_bytes(raw)
        except HTTPError as exc:
            detail = _http_error_body(exc)
            last_error = RuntimeError(f"MATRIX occupancy HTTP {exc.code}: {url}{detail}")
        except (URLError, TimeoutError, OSError, ValueError) as exc:
            last_error = RuntimeError(f"MATRIX occupancy failed: {exc}")
    if last_error is not None:
        raise last_error
    raise RuntimeError("MATRIX occupancy export returned no raster")


def occupancy_from_rokae_config(config: Dict[str, Any], *, map_name: str = "") -> Optional[Dict[str, Any]]:
    rokae = config.get("rokae") if isinstance(config.get("rokae"), dict) else {}
    matrix = rokae.get("matrix") if isinstance(rokae.get("matrix"), dict) else {}
    path = str(matrix.get("occupancy_path") or "").strip()
    if path:
        file_path = Path(path)
        if not file_path.is_absolute():
            file_path = Path(__file__).resolve().parent.parent.parent / path
        return occupancy_from_local_path(file_path)
    sros = rokae.get("sros") if isinstance(rokae.get("sros"), dict) else {}
    if bool(sros.get("sim")) or str(rokae.get("backend") or "") == "mock":
        return None
    if str(matrix.get("topology_path") or "").strip():
        return None
    name = str(matrix.get("map_name") or map_name or rokae.get("map_id") or "").strip()
    base = str(matrix.get("base_url") or sros.get("host") or "192.168.71.50").strip()
    if base and "://" not in base:
        base = "http://" + base
    timeout = float(
        matrix.get("occupancy_timeout_sec")
        or matrix.get("timeout_sec")
        or config.get("http_timeout_sec")
        or 30.0
    )
    return fetch_occupancy(base_url=base, map_name=name, timeout_sec=max(timeout, 8.0))


def occupancy_from_local_path(path: Path) -> Dict[str, Any]:
    raw = path.read_bytes()
    try:
        return occupancy_from_export_bytes(raw)
    except ValueError:
        pass
    if path.suffix.lower() in {".pgm", ".png"}:
        meta_path = path.with_suffix(".json")
        if not meta_path.is_file():
            raise ValueError(f"occupancy image {path} needs sibling json meta")
        doc = json.loads(meta_path.read_text(encoding="utf-8"))
        return occupancy_from_sros_map(doc, raw, image_name=path.name)
    doc = decode_topology_bytes(raw)
    for suffix in (".pgm", ".png"):
        image_path = path.with_suffix(suffix)
        if image_path.is_file():
            return occupancy_from_sros_map(doc, image_path.read_bytes(), image_name=image_path.name)
    raise ValueError(f"no occupancy raster next to {path}")


def _split_export(raw: bytes) -> tuple[Dict[str, Any], bytes, str]:
    if raw[:2] == b"\x1f\x8b":
        raw = gzip.decompress(raw)
    if raw[:2] == b"PK":
        with zipfile.ZipFile(io.BytesIO(raw)) as archive:
            return _from_names(archive.namelist(), archive.read)
    extracted = _from_tar(raw)
    if extracted is not None:
        return extracted
    raise ValueError("MATRIX occupancy export has no map image")


def _from_tar(raw: bytes) -> Optional[tuple[Dict[str, Any], bytes, str]]:
    if len(raw) < 262 and raw[:2] != b"\x1f\x8b":
        return None
    try:
        archive = tarfile.open(fileobj=io.BytesIO(raw), mode="r:*")
    except tarfile.TarError:
        return None
    with archive:
        members = [item for item in archive.getmembers() if item.isfile()]
        blobs: Dict[str, bytes] = {}
        for member in members:
            handle = archive.extractfile(member)
            if handle is None:
                continue
            blobs[member.name] = handle.read()
        return _from_names(list(blobs), blobs.__getitem__)


def _from_names(names: list[str], reader: Any) -> tuple[Dict[str, Any], bytes, str]:
    json_name = None
    doc: Optional[Dict[str, Any]] = None
    rasters: list[tuple[str, bytes]] = []
    for name in names:
        suffix = Path(name).suffix.lower()
        if suffix in {".pgm", ".png"}:
            blob = reader(name)
            if blob:
                rasters.append((name, blob))
            continue
        if not name.lower().endswith(".json"):
            continue
        parsed = json.loads(reader(name).decode("utf-8"))
        if not isinstance(parsed, dict):
            continue
        meta = parsed.get("meta") if isinstance(parsed.get("meta"), dict) else {}
        if meta.get("size.x") or meta.get("resolution"):
            json_name, doc = name, parsed
            continue
        if json_name is None:
            json_name, doc = name, parsed
    if json_name is None or not rasters or not isinstance(doc, dict):
        raise ValueError("MATRIX occupancy export missing json or png/pgm")
    image_name, image = max(rasters, key=_raster_rank)
    return doc, image, image_name


def _raster_rank(item: tuple[str, bytes]) -> tuple[int, int]:
    name, blob = item
    pgm = 1 if Path(name).suffix.lower() == ".pgm" else 0
    return (pgm, len(blob))


def _http_error_body(exc: HTTPError) -> str:
    try:
        body = exc.read().decode("utf-8", errors="replace").strip()
    except Exception:
        return ""
    if not body:
        return ""
    return f" {body[:200]}"


def _size(meta: Dict[str, Any]) -> tuple[int, int]:
    width = int(meta.get("size.x") or meta.get("size_x") or meta.get("width") or 0)
    height = int(meta.get("size.y") or meta.get("size_y") or meta.get("height") or 0)
    if width <= 0 or height <= 0:
        raise ValueError("MATRIX map meta missing size.x/size.y")
    return width, height


def _resolution_m(meta: Dict[str, Any]) -> float:
    value = float(meta.get("resolution") or 0.0)
    if value <= 0:
        raise ValueError("MATRIX map meta missing resolution")
    unit = str(meta.get("length_unit") or "mm").strip().lower()
    if unit in {"cm", "centimeter", "centimetre"}:
        return value / 100.0
    if unit in {"mm", "millimeter", "millimetre"}:
        # length_unit=mm 只作用于 pos。建图格长跟网页一致：2 → 2cm/格。
        # 跟 pos 一起 /1000 会变成 2mm/格，742×906 只剩 1.5m；底盘是 ~16×20m。
        # 10 及以上按毫米（20、50 那种档）。
        if value < 10:
            return value / 100.0
        return value / 1000.0
    return value


def _origin_m(meta: Dict[str, Any], *, width: int, height: int, resolution: float) -> Dict[str, float]:
    zero_x = float(meta.get("zero_offset.x") or meta.get("zero_offset_x") or 0.0)
    zero_y = float(meta.get("zero_offset.y") or meta.get("zero_offset_y") or 0.0)
    # 图像左上、Y 向下；ROS 栅格左下、Y 向上。zero_offset 是世界原点所在像素。
    return {
        "x": round(-zero_x * resolution, 6),
        "y": round((zero_y - height) * resolution, 6),
    }
