"""把一张图打成底盘能收的 FMS 包，再 POST /api/v0/map/import。

包里只要两样：sros/map/{name}.json 和 sros/map/{name}0.pgm。
pgm 头三行注释是偏移和分辨率，缺了底盘回 90004。
"""

from __future__ import annotations

import hashlib
import gzip
import io
import json
import tarfile
from typing import Any, Dict, Optional, Tuple
from urllib.error import HTTPError, URLError
from urllib.request import Request, urlopen


def blank_pgm(doc: Dict[str, Any]) -> bytes:
    meta = doc.get("meta") if isinstance(doc.get("meta"), dict) else {}
    width = int(meta.get("size.x") or 0)
    height = int(meta.get("size.y") or 0)
    if width < 1 or height < 1:
        raise ValueError("meta.size.x and meta.size.y must be >= 1")
    zero_x = int(meta.get("zero_offset.x") or 0)
    zero_y = int(meta.get("zero_offset.y") or 0)
    resolution = int(meta.get("resolution") or 2)
    header = f"P5\n#{zero_x}\n#{zero_y}\n#{resolution}\n{width} {height}\n255\n"
    return header.encode("ascii") + bytes([254]) * (width * height)


def fms_gzip(map_name: str, doc: Dict[str, Any], pgm: Optional[bytes] = None) -> bytes:
    name = str(map_name or "").strip()
    if not name:
        raise ValueError("map_name is required")
    raster = pgm if pgm is not None else blank_pgm(doc)
    payload = json.dumps(doc, ensure_ascii=False, separators=(",", ":")).encode("utf-8")
    raw = io.BytesIO()
    with tarfile.open(fileobj=raw, mode="w") as tar:
        _add(tar, f"sros/map/{name}.json", payload)
        _add(tar, f"sros/map/{name}0.pgm", raster)
    out = io.BytesIO()
    with gzip.GzipFile(fileobj=out, mode="wb", mtime=0) as gz:
        gz.write(raw.getvalue())
    return out.getvalue()


def import_fms(
    base_url: str,
    packed: bytes,
    *,
    map_name: str,
    base_version: str = "1.13.0",
    lang: str = "zh-cn",
    timeout: float = 30.0,
) -> Tuple[int, Dict[str, Any]]:
    url = base_url.rstrip("/") + "/api/v0/map/import"
    filename = f"{map_name}.map_export"
    body, content_type = _multipart(packed, filename, base_version, lang)
    request = Request(
        url,
        data=body,
        headers={"Content-Type": content_type, "Content-Length": str(len(body))},
        method="POST",
    )
    try:
        with urlopen(request, timeout=timeout) as response:
            raw = response.read()
            status = int(response.status)
    except HTTPError as exc:
        raw = exc.read()
        status = int(exc.code)
    except URLError as exc:
        raise OSError(f"chassis import unreachable: {exc.reason}") from exc
    try:
        parsed = json.loads(raw.decode("utf-8")) if raw else {}
    except (UnicodeDecodeError, json.JSONDecodeError):
        parsed = {"raw": raw.decode("utf-8", errors="replace")}
    if not isinstance(parsed, dict):
        parsed = {"raw": parsed}
    return status, parsed


def _add(tar: tarfile.TarFile, name: str, payload: bytes) -> None:
    info = tarfile.TarInfo(name)
    info.size = len(payload)
    tar.addfile(info, io.BytesIO(payload))


def _multipart(packed: bytes, filename: str, base_version: str, lang: str) -> Tuple[bytes, str]:
    md5 = hashlib.md5(packed).hexdigest()
    boundary = "----NavMapImport" + md5
    body = bytearray()
    for name, value in (("md5", md5), ("base_version", base_version), ("lang", lang)):
        body.extend(f"--{boundary}\r\n".encode())
        body.extend(f'Content-Disposition: form-data; name="{name}"\r\n\r\n'.encode())
        body.extend(str(value).encode())
        body.extend(b"\r\n")
    body.extend(f"--{boundary}\r\n".encode())
    body.extend(
        (
            'Content-Disposition: form-data; name="file"; '
            f'filename="{filename}"\r\n'
            "Content-Type: application/octet-stream\r\n\r\n"
        ).encode()
    )
    body.extend(packed)
    body.extend(b"\r\n")
    body.extend(f"--{boundary}--\r\n".encode())
    return bytes(body), f"multipart/form-data; boundary={boundary}"
