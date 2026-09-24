"""Shared image loading helpers for vision APIs."""

from __future__ import annotations

import base64
import binascii
from pathlib import Path

import cv2
import numpy as np
from fastapi import HTTPException


def load_image_from_path(image_path: str) -> np.ndarray:
    path = Path(image_path).expanduser()
    if not path.is_file():
        raise HTTPException(status_code=400, detail=f"图片不存在: {image_path}")
    image = cv2.imread(str(path), cv2.IMREAD_COLOR)
    if image is None:
        raise HTTPException(status_code=400, detail=f"无法读取图片: {image_path}")
    return image


def load_image_from_base64(image_base64: str) -> np.ndarray:
    encoded = image_base64.strip()
    if not encoded:
        raise HTTPException(status_code=400, detail="图片不能为空")
    if encoded.startswith("data:"):
        encoded = encoded.partition(",")[2]
    try:
        image_bytes = base64.b64decode(encoded, validate=True)
    except (ValueError, binascii.Error) as error:
        raise HTTPException(status_code=400, detail="图片 Base64 格式错误") from error
    if not image_bytes:
        raise HTTPException(status_code=400, detail="图片不能为空")
    image = cv2.imdecode(np.frombuffer(image_bytes, np.uint8), cv2.IMREAD_COLOR)
    if image is None:
        raise HTTPException(status_code=400, detail="无法读取图片: Base64 内容不是有效图片")
    return image


def load_request_image(
    *,
    image_path: str | None = None,
    image_base64: str | None = None,
) -> np.ndarray:
    path = (image_path or "").strip()
    encoded = (image_base64 or "").strip()
    if path:
        return load_image_from_path(path)
    if encoded:
        return load_image_from_base64(encoded)
    raise HTTPException(status_code=400, detail="image_path 与 image_base64 必须提供一个")
