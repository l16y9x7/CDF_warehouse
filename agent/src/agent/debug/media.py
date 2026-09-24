from __future__ import annotations

from collections.abc import Sequence
from io import BytesIO
from pathlib import Path
from typing import Any

import numpy as np
from PIL import Image

from .overlay import attach_overlays, overlays_from_run

MAX_DEPTH_FILE_BYTES = 64 << 20
MAX_DEPTH_PIXELS = 16_777_216


def media_from_run(
    run: dict[str, Any],
    *,
    spans: Sequence[dict[str, Any]] | None = None,
) -> list[dict[str, Any]]:
    """Build a stable, de-duplicated media list without changing persisted run data."""
    captures = [
        event
        for event in (run.get("events") or [])
        if isinstance(event, dict) and event.get("event") == "camera.captured"
    ]
    result = run.get("result")
    if isinstance(result, dict) and result.get("capture_id") and result.get("camera"):
        captures.append(result)

    media: list[dict[str, Any]] = []
    seen: set[tuple[str, str, str]] = set()
    for capture in captures:
        capture_id = capture.get("capture_id")
        camera = capture.get("camera")
        if not isinstance(capture_id, str) or not isinstance(camera, str):
            continue
        for stream in ("color", "depth"):
            frame = capture.get(stream)
            if not isinstance(frame, dict) or not isinstance(frame.get("path"), str):
                continue
            key = (capture_id, stream, frame["path"])
            if key in seen:
                continue
            seen.add(key)
            item: dict[str, Any] = {
                "capture_id": capture_id,
                "camera": camera,
                "stream": stream,
                "format": frame.get("format"),
                "path": frame["path"],
                "width": frame.get("width"),
                "height": frame.get("height"),
                "skill": capture.get("skill"),
            }
            if capture.get("color_intrinsics") is not None:
                item["K"] = capture.get("color_intrinsics")
            media.append(item)
    return attach_overlays(media, overlays_from_run(run, spans))


def depth_preview_png(path: Path) -> bytes:
    if path.stat().st_size > MAX_DEPTH_FILE_BYTES:
        raise ValueError("depth file is too large to preview")
    try:
        depth = np.load(path, allow_pickle=False, mmap_mode="r")
    except Exception as exc:
        raise ValueError("depth file is not a safe NumPy array") from exc
    if depth.ndim != 2 or not np.issubdtype(depth.dtype, np.number):
        raise ValueError("depth preview requires a two-dimensional numeric array")
    if depth.size > MAX_DEPTH_PIXELS:
        raise ValueError("depth array is too large to preview")

    values = depth.astype(np.float64, copy=False)
    valid = np.isfinite(values) & (values > 0)
    pixels = np.zeros(values.shape, dtype=np.uint8)
    if np.any(valid):
        low, high = np.percentile(values[valid], (2, 98))
        if high <= low:
            pixels[valid] = 255
        else:
            normalized = 1.0 - np.clip((values - low) / (high - low), 0.0, 1.0)
            pixels[valid] = np.rint(normalized[valid] * 255).astype(np.uint8)

    output = BytesIO()
    Image.fromarray(pixels).save(output, format="PNG", optimize=True)
    return output.getvalue()
