"""Shared service and camera endpoint configuration for perception APIs."""

from __future__ import annotations

import os
from pathlib import Path

_PERCEPTION_ROOT = Path(__file__).resolve().parent


SERVICE_BIND_HOST = os.getenv("SERVICE_BIND_HOST", "0.0.0.0")

# Keep every non-loopback host in this file.  A deployment can either change
# these defaults or override them with environment variables.  Full URL
# overrides below remain supported for backward compatibility.
CAMERA_SERVICE_HOST = os.getenv("CAMERA_SERVICE_HOST", "192.168.200.66")
INFERENCE_SERVICE_HOST = os.getenv("INFERENCE_SERVICE_HOST", "192.168.200.65")

CAMERA_SERVICE_URL = os.getenv(
    "CAMERA_SERVICE_URL",
    f"http://{CAMERA_SERVICE_HOST}:8085",
).rstrip("/")

SKU_API_URL = os.getenv(
    "SKU_API_URL",
    os.getenv("SKU_BASE_URL", "http://127.0.0.1:25540"),
).rstrip("/")
SAM3_URL = os.getenv(
    "SAM3_URL",
    f"http://{INFERENCE_SERVICE_HOST}:25541/api/v1/segment",
)
# SAM3 segment 服务示例: http://211.137.21.33:25541/api/v1/segment
# 旧 infer 服务示例: http://192.168.130.97:18004/infer
# URL 以 /infer 结尾时自动使用 JSON+base64 协议；也可显式设置 SAM3_BACKEND=infer|segment
QWEN3_URL = os.getenv(
    "QWEN3_URL",
    f"http://{INFERENCE_SERVICE_HOST}:25542/v1/chat/completions",
)
QWEN3_MODEL = os.getenv(
    "QWEN3_MODEL",
    os.getenv("QWEN_MODEL", "Qwen3-VL-4B-Instruct"),
)

# recognize_sku_barcode: L1 center distance filter for multi-barcode scenes.
# bbox center must satisfy |cx-0.5| + |cy-0.5| <= this value to stay in the candidate pool.
RECOGNIZE_SKU_BARCODE_CENTER_DISTANCE_MAX = float(
    os.getenv("RECOGNIZE_SKU_BARCODE_CENTER_DISTANCE_MAX", "0.45")
)
# Expand SAM3 bbox before 1D barcode decode to include quiet zone around bars.
RECOGNIZE_SKU_BARCODE_BBOX_PADDING_RATIO = float(
    os.getenv("RECOGNIZE_SKU_BARCODE_BBOX_PADDING_RATIO", "0.15")
)
RECOGNIZE_SKU_BARCODE_LOG_PATH = os.getenv(
    "RECOGNIZE_SKU_BARCODE_LOG_PATH",
    str(_PERCEPTION_ROOT / "logs" / "recognize_sku_barcode.log"),
)


def camera_snapshot_url(camera: str) -> str:
    """Return the configured color snapshot URL for a camera name."""

    defaults = {
        "left": f"{CAMERA_SERVICE_URL}/camera/snapshot?camera=left_wrist&type=color",
        "right": f"{CAMERA_SERVICE_URL}/camera/snapshot?camera=right_wrist&type=color",
        "head": f"{CAMERA_SERVICE_URL}/camera/snapshot?camera=head&type=color",
    }
    environment_names = {
        "left": "LEFT_CAMERA_SNAPSHOT_URL",
        "right": "RIGHT_CAMERA_SNAPSHOT_URL",
        "head": "RECEIPT_CAMERA_URL",
    }
    normalized_camera = camera.strip().lower()
    if normalized_camera not in defaults:
        raise ValueError(f"unsupported camera: {camera}")

    legacy_url = os.getenv("CAMERA_SNAPSHOT_URL", "").strip()
    if normalized_camera == "left" and legacy_url:
        return os.getenv(environment_names[normalized_camera], "").strip() or legacy_url
    return (
        os.getenv(environment_names[normalized_camera], "").strip()
        or defaults[normalized_camera]
    )


def camera_depth_snapshot_url(camera: str) -> str:
    """Return the configured raw 16UC1 depth snapshot URL for a camera."""

    defaults = {
        "left": f"{CAMERA_SERVICE_URL}/camera/snapshot?camera=left_wrist&type=depth",
        "right": f"{CAMERA_SERVICE_URL}/camera/snapshot?camera=right_wrist&type=depth",
        "head": f"{CAMERA_SERVICE_URL}/camera/snapshot?camera=head&type=depth",
    }
    environment_names = {
        "left": "LEFT_CAMERA_DEPTH_SNAPSHOT_URL",
        "right": "RIGHT_CAMERA_DEPTH_SNAPSHOT_URL",
        "head": "HEAD_CAMERA_DEPTH_SNAPSHOT_URL",
    }
    normalized_camera = camera.strip().lower()
    if normalized_camera not in defaults:
        raise ValueError(f"unsupported depth camera: {camera}")
    return (
        os.getenv(environment_names[normalized_camera], "").strip()
        or defaults[normalized_camera]
    )


def hand_camera_snapshot_url(hand: str) -> str:
    """Resolve left/right robot hand names to their wrist camera URLs."""

    normalized_hand = hand.strip().lower()
    if normalized_hand not in {"left", "right"}:
        raise ValueError("hand must be left or right")
    return camera_snapshot_url(normalized_hand)
