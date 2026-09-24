"""GET /camera/capture contract helpers for ROKAE Owner HTTP."""

from __future__ import annotations

import os
import shutil
import uuid
from typing import Any, Dict, Optional, Set, Tuple

# Contract role (Feishu) <-> Owner internal id.
_TO_INTERNAL = {
    "head": "head",
    "left_wrist": "hand_left",
    "left": "hand_left",
    "hand_left": "hand_left",
    "right_wrist": "hand_right",
    "hand_wrist": "hand_right",
    "right": "hand_right",
    "hand_right": "hand_right",
}
_TO_CONTRACT = {
    "head": "head",
    "hand_left": "left_wrist",
    "left_wrist": "left_wrist",
    "left": "left_wrist",
    "hand_right": "right_wrist",
    "hand_wrist": "right_wrist",
    "right_wrist": "right_wrist",
    "right": "right_wrist",
}

_VALID_STREAMS = {
    frozenset({"color"}),
    frozenset({"depth"}),
    frozenset({"color", "depth"}),
}


def capture_root() -> str:
    return str(os.environ.get("VISION_CAPTURE_ROOT") or "/shared/frames").rstrip("/")


def resolve_camera(raw: str) -> Optional[Tuple[str, str]]:
    """Return (contract_id, internal_id) or None if unknown."""
    key = str(raw or "").strip()
    internal = _TO_INTERNAL.get(key)
    contract = _TO_CONTRACT.get(key)
    if not internal or not contract:
        return None
    return contract, internal


def contract_camera_id(internal: str) -> str:
    return _TO_CONTRACT.get(str(internal or "").strip(), str(internal or "").strip())


def parse_streams(raw: Optional[str]) -> Tuple[Optional[Set[str]], Optional[str]]:
    """Return (stream_set, error_code). Default color when omitted."""
    if raw is None:
        return {"color"}, None
    text = str(raw).strip()
    if text == "":
        return None, "INVALID_STREAMS"
    parts = [p.strip().lower() for p in text.split(",") if p.strip()]
    if not parts:
        return None, "INVALID_STREAMS"
    streams = set(parts)
    if streams not in _VALID_STREAMS or len(parts) != len(streams):
        return None, "INVALID_STREAMS"
    return streams, None


def parse_format(raw: Optional[str], *, need_depth: bool) -> Tuple[Optional[str], Optional[str]]:
    """Return (format, error_code). Ignored when depth not requested."""
    if not need_depth:
        return None, None
    if raw is None or str(raw).strip() == "":
        return "raw", None
    value = str(raw).strip().lower()
    if value not in {"raw", "preview"}:
        return None, "INVALID_FORMAT"
    return value, None


def parse_capture_query(qs: Dict[str, str]) -> Tuple[Optional[Dict[str, Any]], Optional[Dict[str, Any]]]:
    """
    Parse capture query.
    Success: ({contract, internal, streams, format}, None)
    Failure: (None, error_payload)
    """
    camera_raw = str(qs.get("camera") or qs.get("camera_id") or "")
    resolved = resolve_camera(camera_raw)
    if resolved is None:
        return None, {
            "ok": False,
            "error_code": "CAMERA_NOT_FOUND",
            "message": "camera invalid or unsupported",
            "camera": camera_raw,
        }
    contract, internal = resolved
    streams, err = parse_streams(qs.get("streams") if "streams" in qs else None)
    if err:
        return None, {
            "ok": False,
            "error_code": err,
            "message": "streams invalid",
            "camera": contract,
        }
    depth_fmt, err = parse_format(qs.get("format") if "format" in qs else None,
                                  need_depth="depth" in streams)
    if err:
        return None, {
            "ok": False,
            "error_code": err,
            "message": "format invalid",
            "camera": contract,
        }
    return {
        "contract": contract,
        "internal": internal,
        "streams": streams,
        "format": depth_fmt,
    }, None


def new_capture_id() -> str:
    return f"capture-{uuid.uuid4().hex[:12]}"


def _jpeg_size(data: bytes) -> Tuple[int, int]:
    """Read width/height from a baseline JPEG without OpenCV."""
    if len(data) < 4 or data[:2] != b"\xff\xd8":
        raise ValueError("invalid color jpeg")
    i = 2
    while i + 9 < len(data):
        if data[i] != 0xFF:
            i += 1
            continue
        marker = data[i + 1]
        if marker in {0xC0, 0xC1, 0xC2}:  # SOF0/1/2
            height = int.from_bytes(data[i + 5:i + 7], "big")
            width = int.from_bytes(data[i + 7:i + 9], "big")
            if min(width, height) <= 0:
                raise ValueError("invalid color jpeg size")
            return width, height
        if marker == 0xD9:  # EOI
            break
        if marker in {0xD8, 0x01} or 0xD0 <= marker <= 0xD7:
            i += 2
            continue
        length = int.from_bytes(data[i + 2:i + 4], "big")
        i += 2 + length
    raise ValueError("invalid color jpeg")


def write_capture_dir(
    *,
    capture_id: str,
    color_jpeg: Optional[bytes],
    depth_mm,
    depth_format: Optional[str],
    depth_aligned: Optional[bool] = None,
    root: Optional[str] = None,
) -> Dict[str, Any]:
    """
    Atomically write requested streams under root/capture_id.
    depth_mm: HxW uint16 ndarray or None.
    Raises OSError/ValueError on failure after cleanup.
    """
    import numpy as np

    base = (root or capture_root()).rstrip("/")
    os.makedirs(base, exist_ok=True)
    out_dir = os.path.join(base, capture_id)
    if os.path.exists(out_dir):
        raise ValueError(f"capture_id already exists: {capture_id}")
    os.makedirs(out_dir, exist_ok=False)
    color_meta = None
    depth_meta = None
    try:
        if color_jpeg is not None:
            path = os.path.join(out_dir, "rgb.jpg")
            with open(path, "wb") as fh:
                fh.write(color_jpeg)
            w, h = _jpeg_size(color_jpeg)
            color_meta = {"path": path, "format": "jpeg", "width": int(w), "height": int(h)}
        if depth_mm is not None:
            depth = np.asarray(depth_mm)
            if depth.ndim != 2 or depth.size == 0:
                raise ValueError("invalid depth array")
            h, w = depth.shape[:2]
            if depth_format == "preview":
                import cv2

                path = os.path.join(out_dir, "depth_preview.jpg")
                norm = cv2.normalize(depth, None, 0, 255, cv2.NORM_MINMAX)
                ok, buf = cv2.imencode(".jpg", norm.astype("uint8"),
                                       [int(cv2.IMWRITE_JPEG_QUALITY), 80])
                if not ok:
                    raise ValueError("depth preview encode failed")
                with open(path, "wb") as fh:
                    fh.write(buf.tobytes())
                depth_meta = {
                    "path": path, "format": "preview",
                    "width": int(w), "height": int(h), "aligned": bool(depth_aligned),
                }
            else:
                path = os.path.join(out_dir, "depth_mm.npy")
                np.save(path, depth.astype(np.uint16, copy=False))
                depth_meta = {
                    "path": path, "format": "raw",
                    "width": int(w), "height": int(h), "aligned": bool(depth_aligned),
                }
    except Exception:
        shutil.rmtree(out_dir, ignore_errors=True)
        raise
    return {"color": color_meta, "depth": depth_meta}
