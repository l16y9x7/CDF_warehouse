#!/usr/bin/env python3
"""SAM3 client communications, mask encoding/decoding, and detection proposal processing.

Supports both `legacy_18003` JSON payloads and `multipart_segment` HTTP backends.
Provides unified mask decoding (RLE/PNG), score filtering, proposal preparation,
and detection ranking.
"""
import base64
import time
from typing import Any, Dict, List, Optional, Tuple, Union
import cv2
import numpy as np
import requests

try:
    from config_loader import SAM3_BACKEND, SAM3_MASK_THRESHOLD, SAM3_TIMEOUT_S, SAM3_URL
    from request_codec import arr
except ImportError:
    from deploy.config_loader import SAM3_BACKEND, SAM3_MASK_THRESHOLD, SAM3_TIMEOUT_S, SAM3_URL
    from deploy.request_codec import arr

try:
    import pycocotools.mask as mask_utils
except ImportError:
    try:
        from module_loader import get_deploy_modules
        mask_utils = get_deploy_modules().fit_bottle_axis.mask_utils
    except Exception:
        mask_utils = None


def decode_detection_mask(detection: Union[Dict[str, Any], Any]) -> np.ndarray:
    """Decodes a mask from a detection proposal dictionary or segmentation structure."""
    if isinstance(detection, dict) and '_mask' in detection:
        return detection['_mask']
    seg = detection.get('segmentation', detection) if isinstance(detection, dict) else detection
    if mask_utils is None:
        raise RuntimeError("pycocotools.mask is required for mask decoding")
    counts = seg.get('counts') if isinstance(seg, dict) else None
    if counts is None:
        raise ValueError("Invalid segmentation format, missing 'counts'")
    if isinstance(counts, str):
        counts = counts.encode('ascii')
    rle_obj = {'size': seg['size'], 'counts': counts}
    decoded = mask_utils.decode(rle_obj)
    if decoded.ndim == 3:
        decoded = decoded[..., 0]
    return decoded.astype(bool)


def encode_mask(mask: np.ndarray) -> Dict[str, Any]:
    """Encodes a boolean mask array into standard COCO RLE dictionary format."""
    if mask_utils is None:
        raise RuntimeError("pycocotools.mask is required for mask encoding")
    encoded = mask_utils.encode(np.asfortranarray(mask.astype(np.uint8)))
    counts = encoded['counts']
    encoded['counts'] = counts.decode('ascii') if isinstance(counts, bytes) else counts
    encoded['size'] = [int(x) for x in encoded['size']]
    return encoded


def parse_multipart_segment(
    data: Dict[str, Any],
    rgb_shape: Tuple[int, ...],
    prompt: str,
    threshold: float,
    wall_ms: float
) -> Dict[str, Any]:
    """Parses multipart SAM3 service response into standard detection dictionary."""
    if not isinstance(data, dict):
        raise ValueError('multipart SAM3 response must be an object')
    instances = data.get('instances') or []
    detections = []
    for index, item in enumerate(instances, 1):
        score = float(item.get('score', 0))
        bbox = arr(item.get('bbox_xyxy'), (4,))
        raw = base64.b64decode(item.get('mask_png_base64', ''), validate=True)
        mask = cv2.imdecode(np.frombuffer(raw, np.uint8), cv2.IMREAD_GRAYSCALE)
        detections.append({
            'bbox': [float(bbox[0]), float(bbox[1]), float(bbox[2] - bbox[0]), float(bbox[3] - bbox[1])],
            'score': score,
            'segmentation': encode_mask(mask > 0),
            'upstream_instance_id': index,
            'remote_instance_id': item.get('instance_id'),
        })
    return {
        'ok': True,
        'prompt': data.get('prompt', prompt),
        'num_detections': len(detections),
        'detections': detections,
        'image_size': [int(rgb_shape[1]), int(rgb_shape[0])],
        'threshold': float(data.get('threshold', threshold)),
        'mask_threshold': float(data.get('mask_threshold', SAM3_MASK_THRESHOLD)),
        'backend': 'multipart_segment',
        '_sam3_backend': 'multipart_segment',
        '_service_call_wall_ms': round(float(wall_ms), 3),
    }


def call_sam3(
    rgb: np.ndarray,
    prompt: str,
    threshold: float,
    url: Optional[str] = None
) -> Dict[str, Any]:
    """Calls upstream SAM3 service (multipart or legacy JSON) and returns detection response."""
    target_url = url or SAM3_URL
    ok, encoded = cv2.imencode('.png', rgb)
    if not ok:
        raise ValueError('failed to encode upstream RGB as PNG')
    png_bytes = encoded.tobytes()
    started = time.perf_counter()

    if SAM3_BACKEND == 'multipart_segment':
        resp = requests.post(
            target_url,
            files={'image': ('frame.png', png_bytes, 'image/png')},
            data={
                'prompt': prompt,
                'threshold': str(float(threshold)),
                'mask_threshold': str(SAM3_MASK_THRESHOLD),
            },
            timeout=SAM3_TIMEOUT_S,
        )
        resp.raise_for_status()
        return parse_multipart_segment(
            resp.json(), rgb.shape, prompt, threshold, (time.perf_counter() - started) * 1000
        )
    else:
        payload = {
            'image_base64': base64.b64encode(png_bytes).decode('ascii'),
            'prompt': prompt,
            'threshold': float(threshold),
            'allow_empty': True,
        }
        resp = requests.post(target_url, json=payload, timeout=SAM3_TIMEOUT_S)
        resp.raise_for_status()
        data = resp.json()
        if not data.get('ok', False):
            raise ValueError('SAM3 returned not-ok: ' + str(data))
        data['_service_call_wall_ms'] = round((time.perf_counter() - started) * 1000, 3)
        data['_sam3_backend'] = 'legacy_18003'
        return data


def score_filter(
    detections: List[Dict[str, Any]],
    threshold: float
) -> Tuple[List[Dict[str, Any]], List[Dict[str, Any]]]:
    """Filters detection candidates by confidence score threshold."""
    kept = []
    rejected = []
    for i, item in enumerate(detections, 1):
        score = float(item.get('score', -1))
        if np.isfinite(score) and score >= threshold:
            d = dict(item)
            d['upstream_instance_id'] = i
            kept.append(d)
        else:
            rejected.append({'upstream_instance_id': i, 'score': score, 'reason': 'score_below_target_threshold'})
    return kept, rejected


def prepare_detections(
    items: List[Dict[str, Any]],
    shape: Tuple[int, ...]
) -> List[Dict[str, Any]]:
    """Decodes detection masks and validates dimensions match the RGB frame."""
    out = []
    for i, item in enumerate(items, 1):
        d = dict(item)
        d['upstream_instance_id'] = int(d.get('upstream_instance_id', i))
        m = d.get('_mask')
        if m is None:
            m = decode_detection_mask(d)
        if m.shape != tuple(shape[:2]):
            raise ValueError('SAM3 mask dimensions do not match inference image')
        d['_mask'] = m
        out.append(d)
    return out


def setup_category_detections(
    raw_detections: List[Dict[str, Any]],
    class_name: str
) -> List[Dict[str, Any]]:
    """Initializes per-category filtered detections with category metadata and decoded masks."""
    out = []
    for filtered_id, item in enumerate(raw_detections, 1):
        d = dict(item)
        d['class_name'] = class_name
        d['upstream_instance_id'] = int(d.get('upstream_instance_id', filtered_id))
        d['filtered_instance_id'] = filtered_id
        if '_mask' not in d:
            d['_mask'] = decode_detection_mask(d)
        out.append(d)
    return out


def rank_by_score(detections: List[Dict[str, Any]]) -> List[Tuple[int, float, int]]:
    """Ranks detection proposals by score descending, breaking ties by upstream ID ascending."""
    return sorted(
        [
            (int(d.get('upstream_instance_id', i)), float(d.get('score', -1)), i)
            for i, d in enumerate(detections, 1)
        ],
        key=lambda x: (-x[1], x[0]),
    )


def sam_summary(
    sam: Dict[str, Any],
    prompt: str,
    threshold: Optional[float] = None
) -> Dict[str, Any]:
    """Generates standardized diagnostic summary of upstream SAM3 call."""
    out = {
        k: sam.get(k)
        for k in (
            'ok', 'prompt', 'num_detections', 'elapsed_ms', 'wall_ms',
            'backend', 'engine', 'script_version', 'image_size', 'mask_threshold'
        )
    }
    out['requested_prompt'] = prompt
    if threshold is not None:
        out.update({
            'requested_threshold': float(threshold),
            'threshold_application': 'upstream_request_and_service_side_score_audit',
            'upstream_backend': sam.get('_sam3_backend', SAM3_BACKEND),
        })
    if '_service_call_wall_ms' in sam:
        out['service_call_wall_ms'] = sam['_service_call_wall_ms']
    return out
