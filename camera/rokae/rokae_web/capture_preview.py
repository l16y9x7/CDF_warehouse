"""RGB-only preview through the camera/capture shared-file contract."""
from __future__ import annotations

import json
import re
from pathlib import Path
from urllib.error import HTTPError
from urllib.parse import urlencode
from urllib.request import urlopen

from .backends import BackendError


CAPTURE_URL = 'http://127.0.0.1:8085/camera/capture'
FRAMES_ROOT = Path('/shared/frames')
MAX_JSON = 65536
MAX_JPEG = 8 * 1024 * 1024


def capture_rgb(camera_id: str, timeout: float, *, url: str = CAPTURE_URL,
                frames_root: Path = FRAMES_ROOT, return_metadata=False):
    """Request one color capture; never fall back to a stale or ROS frame."""
    if camera_id not in ('head', 'left_wrist', 'right_wrist'):
        raise BackendError('CAMERA_NOT_FOUND: 未知摄像头')
    request_url = url + '?' + urlencode({'camera': camera_id, 'streams': 'color'})
    try:
        http_failed = False
        try:
            response = urlopen(request_url, timeout=max(0.1, min(timeout, 5.0)))
        except HTTPError as exc:
            response, http_failed = exc, True
        with response:
            raw = response.read(MAX_JSON + 1)
        if len(raw) > MAX_JSON:
            raise BackendError('单帧采集响应过大')
        payload = json.loads(raw)
        if not isinstance(payload, dict):
            raise BackendError('单帧采集未返回 JSON 对象')
        if payload.get('ok') is not True:
            code = str(payload.get('error_code') or 'CAPTURE_FAILED')[:80]
            message = str(payload.get('message') or '相机单帧采集失败')[:300]
            raise BackendError(f'{code}: {message}')
        if http_failed:
            raise BackendError('单帧采集 HTTP 状态与成功结果不一致')
        capture_id = payload.get('capture_id')
        if (not isinstance(capture_id, str)
                or not re.fullmatch(r'[A-Za-z0-9_-]{1,128}', capture_id)
                or payload.get('camera') != camera_id):
            raise BackendError('单帧采集标识或摄像头不匹配')
        color = payload.get('color')
        if (not isinstance(color, dict) or color.get('format') != 'jpeg'
                or payload.get('depth') is not None):
            raise BackendError('单帧采集未返回请求的纯 color JPEG')
        width, height = color.get('width'), color.get('height')
        if any(type(v) is not int or not 0 < v <= 8192 for v in (width, height)):
            raise BackendError('单帧采集图片尺寸无效')
        path_text = color.get('path')
        if not isinstance(path_text, str) or not Path(path_text).is_absolute():
            raise BackendError('单帧采集图片路径无效')
        root = frames_root.resolve()
        capture_dir = root / capture_id
        path = Path(path_text).resolve()
        if capture_dir.resolve() != capture_dir or path.parent != capture_dir:
            raise BackendError('单帧采集图片路径不属于本次 capture_id')
        with path.open('rb') as image:
            frame = image.read(MAX_JPEG + 1)
        if (len(frame) > MAX_JPEG or not frame.startswith(b'\xff\xd8')
                or not frame.endswith(b'\xff\xd9')):
            raise BackendError('单帧采集文件不是有效 JPEG 图像')
        import cv2
        import numpy as np

        decoded = cv2.imdecode(np.frombuffer(frame, dtype=np.uint8), cv2.IMREAD_COLOR)
        if decoded is None or decoded.shape[:2] != (height, width):
            raise BackendError('单帧采集 JPEG 解码或尺寸校验失败')
        return (frame, payload) if return_metadata else frame
    except (OSError, ValueError, RuntimeError) as exc:
        if isinstance(exc, BackendError):
            raise
        raise BackendError(f'获取 RGB 当前帧失败: {exc}') from exc
