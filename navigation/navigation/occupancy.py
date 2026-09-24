"""ROS OccupancyGrid 合同：0 空闲、100 占用、-1 未知；行序从左下原点往上。"""

from __future__ import annotations

import hashlib
import math
from typing import Any, Dict, Iterable, List, Mapping, Optional, Sequence, Tuple

OCC_FREE = 0
OCC_OCCUPIED = 100
OCC_UNKNOWN = -1
OCCUPIED_THRESH = 0.65
FREE_THRESH = 0.196


def occupancy_payload(
    *,
    width: int,
    height: int,
    resolution: float,
    origin: Mapping[str, Any] | Sequence[Any],
    data: Sequence[Any],
) -> Dict[str, Any]:
    w = int(width)
    h = int(height)
    if w <= 0 or h <= 0:
        raise ValueError("occupancy width/height must be positive")
    res = float(resolution)
    if res <= 0:
        raise ValueError("occupancy resolution must be positive")
    cells = [_cell(value) for value in data]
    if len(cells) != w * h:
        raise ValueError(f"occupancy data length {len(cells)} != width*height {w * h}")
    return {
        "width": w,
        "height": h,
        "resolution": res,
        "origin": _origin(origin),
        "data": cells,
    }


def crop_unknown_border(
    grid: Mapping[str, Any],
    *,
    extra_xy: Sequence[Any] = (),
    pad_cells: int = 8,
) -> Dict[str, Any]:
    """裁掉四周未知格，origin 跟着平移。给 FloorMap 整图 fit 用；世界坐标不变。"""
    width = int(grid.get("width") or 0)
    height = int(grid.get("height") or 0)
    resolution = float(grid.get("resolution") or 0.0)
    origin = _origin(grid.get("origin") or {"x": 0.0, "y": 0.0})
    data = [_cell(value) for value in (grid.get("data") or [])]
    if width <= 0 or height <= 0 or resolution <= 0.0 or len(data) != width * height:
        return dict(grid)
    min_col, min_row = width, height
    max_col, max_row = -1, -1
    for index, value in enumerate(data):
        if value == OCC_UNKNOWN:
            continue
        col = index % width
        row = index // width
        min_col = min(min_col, col)
        max_col = max(max_col, col)
        min_row = min(min_row, row)
        max_row = max(max_row, row)
    for item in extra_xy:
        x, y = _xy(item)
        if x is None or y is None:
            continue
        col = (x - origin["x"]) / resolution
        row = (y - origin["y"]) / resolution
        if not math.isfinite(col) or not math.isfinite(row):
            continue
        min_col = min(min_col, col)
        max_col = max(max_col, col)
        min_row = min(min_row, row)
        max_row = max(max_row, row)
    if max_col < min_col or max_row < min_row:
        return occupancy_payload(
            width=width,
            height=height,
            resolution=resolution,
            origin=origin,
            data=data,
        )
    pad = max(0, int(pad_cells))
    min_col_i = max(0, int(math.floor(min_col)) - pad)
    min_row_i = max(0, int(math.floor(min_row)) - pad)
    max_col_i = min(width - 1, int(math.ceil(max_col)) + pad)
    max_row_i = min(height - 1, int(math.ceil(max_row)) + pad)
    new_width = max_col_i - min_col_i + 1
    new_height = max_row_i - min_row_i + 1
    if new_width == width and new_height == height:
        return occupancy_payload(
            width=width,
            height=height,
            resolution=resolution,
            origin=origin,
            data=data,
        )
    cropped: List[int] = []
    for row in range(min_row_i, max_row_i + 1):
        start = row * width + min_col_i
        cropped.extend(data[start : start + new_width])
    return occupancy_payload(
        width=new_width,
        height=new_height,
        resolution=resolution,
        origin={
            "x": round(origin["x"] + min_col_i * resolution, 6),
            "y": round(origin["y"] + min_row_i * resolution, 6),
        },
        data=cropped,
    )


def _xy(item: Any) -> Tuple[Optional[float], Optional[float]]:
    if isinstance(item, Mapping):
        raw_x = item.get("x")
        raw_y = item.get("y")
    elif isinstance(item, Sequence) and not isinstance(item, (str, bytes)) and len(item) >= 2:
        raw_x, raw_y = item[0], item[1]
    else:
        return None, None
    try:
        x = float(raw_x)
        y = float(raw_y)
    except (TypeError, ValueError):
        return None, None
    if not math.isfinite(x) or not math.isfinite(y):
        return None, None
    return x, y


def occupancy_revision(grid: Mapping[str, Any] | None) -> str:
    if not isinstance(grid, Mapping) or not grid:
        return ""
    origin = grid.get("origin") if isinstance(grid.get("origin"), Mapping) else {}
    payload = {
        "width": int(grid.get("width") or 0),
        "height": int(grid.get("height") or 0),
        "resolution": float(grid.get("resolution") or 0.0),
        "origin": {
            "x": float(origin.get("x") or 0.0) if isinstance(origin, Mapping) else 0.0,
            "y": float(origin.get("y") or 0.0) if isinstance(origin, Mapping) else 0.0,
        },
        "data": [int(cell) for cell in (grid.get("data") or [])],
    }
    digest = hashlib.sha256(
        repr(
            (
                payload["width"],
                payload["height"],
                payload["resolution"],
                payload["origin"]["x"],
                payload["origin"]["y"],
                tuple(payload["data"]),
            )
        ).encode("ascii")
    ).hexdigest()
    return digest[:16]


def occupancy_from_gray(
    pixels: Sequence[int],
    *,
    width: int,
    height: int,
    resolution: float,
    origin: Mapping[str, Any] | Sequence[Any],
    top_left: bool = True,
    to_occ: Any = None,
) -> Dict[str, Any]:
    convert = to_occ or _gray_to_occ
    w = int(width)
    h = int(height)
    if len(pixels) != w * h:
        raise ValueError(f"gray pixels {len(pixels)} != width*height {w * h}")
    if top_left:
        cells = []
        for row in range(h):
            source_row = h - 1 - row
            start = source_row * w
            cells.extend(convert(pixels[start + col]) for col in range(w))
    else:
        cells = [convert(value) for value in pixels]
    return occupancy_payload(
        width=w,
        height=h,
        resolution=resolution,
        origin=origin,
        data=cells,
    )


def occupancy_from_pgm(
    raw: bytes,
    *,
    resolution: float,
    origin: Mapping[str, Any] | Sequence[Any],
    to_occ: Any = None,
) -> Dict[str, Any]:
    width, height, pixels = decode_pgm_gray(raw)
    return occupancy_from_gray(
        pixels,
        width=width,
        height=height,
        resolution=resolution,
        origin=origin,
        to_occ=to_occ,
    )


def occupancy_from_png(raw: bytes, *, resolution: float, origin: Mapping[str, Any] | Sequence[Any]) -> Dict[str, Any]:
    width, height, pixels = decode_png_gray(raw)
    return occupancy_from_gray(
        pixels,
        width=width,
        height=height,
        resolution=resolution,
        origin=origin,
    )


def maybe_occupancy(value: Any) -> Optional[Dict[str, Any]]:
    if not isinstance(value, Mapping):
        return None
    try:
        return occupancy_payload(
            width=int(value.get("width") or 0),
            height=int(value.get("height") or 0),
            resolution=float(value.get("resolution") or 0),
            origin=value.get("origin") or {"x": 0.0, "y": 0.0},
            data=value.get("data") or [],
        )
    except (TypeError, ValueError):
        return None


def decode_pgm_gray(raw: bytes) -> Tuple[int, int, List[int]]:
    if raw[:2] != b"P5":
        raise ValueError("not a P5 PGM")
    header, pixels = _split_pnm(raw)
    tokens = header.split()
    if len(tokens) < 4 or tokens[0] != b"P5":
        raise ValueError("invalid PGM header")
    width = int(tokens[1])
    height = int(tokens[2])
    maxval = int(tokens[3])
    if maxval <= 0 or maxval > 255:
        raise ValueError("PGM maxval must be 1..255")
    expected = width * height
    if len(pixels) < expected:
        raise ValueError("PGM pixel data truncated")
    if maxval == 255:
        values = list(pixels[:expected])
    else:
        values = [min(255, int(round(byte * 255 / maxval))) for byte in pixels[:expected]]
    return width, height, values


def decode_png_gray(raw: bytes) -> Tuple[int, int, List[int]]:
    try:
        from PIL import Image
    except Exception:
        Image = None  # type: ignore[assignment]
    if Image is not None:
        from io import BytesIO

        image = Image.open(BytesIO(raw)).convert("L")
        width, height = image.size
        return width, height, list(image.getdata())
    return _decode_png_gray8(raw)


def encode_pgm_gray(width: int, height: int, pixels: Iterable[int]) -> bytes:
    cells = [max(0, min(255, int(value))) for value in pixels]
    if len(cells) != int(width) * int(height):
        raise ValueError("PGM pixel count mismatch")
    header = f"P5\n{int(width)} {int(height)}\n255\n".encode("ascii")
    return header + bytes(cells)


def _origin(value: Mapping[str, Any] | Sequence[Any]) -> Dict[str, float]:
    if isinstance(value, Mapping):
        return {"x": float(value.get("x") or 0.0), "y": float(value.get("y") or 0.0)}
    if isinstance(value, Sequence) and not isinstance(value, (str, bytes)) and len(value) >= 2:
        return {"x": float(value[0]), "y": float(value[1])}
    raise ValueError("occupancy origin must be {x,y} or [x,y]")


def _cell(value: Any) -> int:
    cell = int(value)
    if cell < OCC_UNKNOWN:
        return OCC_UNKNOWN
    if cell > OCC_OCCUPIED:
        return OCC_OCCUPIED
    return cell


def _gray_to_occ(gray: int) -> int:
    occ = (255 - max(0, min(255, int(gray)))) / 255.0
    if occ >= OCCUPIED_THRESH:
        return OCC_OCCUPIED
    if occ <= FREE_THRESH:
        return OCC_FREE
    return OCC_UNKNOWN


def sros_gray_to_occ(gray: int) -> int:
    """MATRIX *0.pgm：0 未探索，1–79 空闲，>=80 占用（与网页 png 黑墙对齐）。"""
    value = max(0, min(255, int(gray)))
    if value <= 0:
        return OCC_UNKNOWN
    if value >= 80:
        return OCC_OCCUPIED
    return OCC_FREE


def _split_pnm(raw: bytes) -> Tuple[bytes, bytes]:
    index = 0
    tokens: List[bytes] = []
    while index < len(raw) and len(tokens) < 4:
        while index < len(raw) and raw[index : index + 1] in b" \t\r\n":
            index += 1
        if index < len(raw) and raw[index : index + 1] == b"#":
            newline = raw.find(b"\n", index)
            index = len(raw) if newline < 0 else newline + 1
            continue
        start = index
        while index < len(raw) and raw[index : index + 1] not in b" \t\r\n":
            index += 1
        if start == index:
            break
        tokens.append(raw[start:index])
    if len(tokens) < 4:
        raise ValueError("invalid PNM header")
    if index < len(raw) and raw[index : index + 1] in b" \t\r\n":
        index += 1
    return b" ".join(tokens), raw[index:]


def _decode_png_gray8(raw: bytes) -> Tuple[int, int, List[int]]:
    import struct
    import zlib

    if raw[:8] != b"\x89PNG\r\n\x1a\n":
        raise ValueError("not a PNG")
    width = height = bit_depth = color_type = None
    idat = bytearray()
    offset = 8
    while offset + 8 <= len(raw):
        length = struct.unpack(">I", raw[offset : offset + 4])[0]
        kind = raw[offset + 4 : offset + 8]
        start = offset + 8
        end = start + length
        if end + 4 > len(raw):
            raise ValueError("truncated PNG")
        chunk = raw[start:end]
        offset = end + 4
        if kind == b"IHDR":
            width, height, bit_depth, color_type, compression, filt, interlace = struct.unpack(
                ">IIBBBBB", chunk[:13]
            )
            if compression or filt or interlace:
                raise ValueError("unsupported PNG compression/filter/interlace")
            if bit_depth != 8 or color_type not in {0, 2, 4, 6}:
                raise ValueError("unsupported PNG color type")
        elif kind == b"IDAT":
            idat.extend(chunk)
        elif kind == b"IEND":
            break
    if width is None or height is None or color_type is None:
        raise ValueError("PNG missing IHDR")
    reconstructed = zlib.decompress(bytes(idat))
    channels = {0: 1, 2: 3, 4: 2, 6: 4}[color_type]
    stride = width * channels
    rows: List[bytes] = []
    cursor = 0
    prev = bytes(stride)
    for _ in range(height):
        filt = reconstructed[cursor]
        scan = reconstructed[cursor + 1 : cursor + 1 + stride]
        cursor += 1 + stride
        row = _paeth_unfilter(filt, scan, prev, channels)
        rows.append(row)
        prev = row
    pixels: List[int] = []
    for row in rows:
        for col in range(width):
            base = col * channels
            if channels == 1:
                pixels.append(row[base])
            else:
                pixels.append((row[base] + row[base + 1] + row[base + 2]) // 3)
    return width, height, pixels


def _paeth_unfilter(filt: int, scan: bytes, prev: bytes, bpp: int) -> bytes:
    out = bytearray(len(scan))
    for i, value in enumerate(scan):
        left = out[i - bpp] if i >= bpp else 0
        up = prev[i]
        up_left = prev[i - bpp] if i >= bpp else 0
        if filt == 0:
            recon = value
        elif filt == 1:
            recon = (value + left) & 255
        elif filt == 2:
            recon = (value + up) & 255
        elif filt == 3:
            recon = (value + ((left + up) // 2)) & 255
        elif filt == 4:
            recon = (value + _paeth(left, up, up_left)) & 255
        else:
            raise ValueError("unsupported PNG filter")
        out[i] = recon
    return bytes(out)


def _paeth(a: int, b: int, c: int) -> int:
    p = a + b - c
    pa = abs(p - a)
    pb = abs(p - b)
    pc = abs(p - c)
    if pa <= pb and pa <= pc:
        return a
    if pb <= pc:
        return b
    return c
