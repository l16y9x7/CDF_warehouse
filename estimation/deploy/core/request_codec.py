#!/usr/bin/env python3
"""Request payload decoding, validation, and normalization utilities.

Handles base64 decoding for RGB/depth, camera intrinsics (K), extrinsics (T),
coordinate frame validation, target contract enforcement, and JSON serialization.
"""
import base64
import io
from typing import Any, Dict, Optional, Tuple
import cv2
import numpy as np

try:
    from config_loader import CLASS_CONFIG
except ImportError:
    from deploy.config_loader import CLASS_CONFIG


def arr(x: Any, shape: Optional[Tuple[int, ...]] = None) -> np.ndarray:
    """Converts input sequence to float numpy array with optional shape validation."""
    a = np.asarray(x, dtype=float)
    if shape and a.size != int(np.prod(shape)):
        raise ValueError('array has wrong size')
    return a.reshape(shape) if shape else a


def decode_b64_image(value: str) -> np.ndarray:
    """Decodes a base64 encoded image string to a BGR numpy array."""
    raw = base64.b64decode(value, validate=True)
    image = cv2.imdecode(np.frombuffer(raw, np.uint8), cv2.IMREAD_COLOR)
    if image is None:
        raise ValueError('rgb_base64 is not a decodable image')
    return image


def decode_npy_b64(value: str) -> np.ndarray:
    """Decodes a base64 encoded numpy array (depth) to a 2D float array."""
    depth = np.load(io.BytesIO(base64.b64decode(value, validate=True)), allow_pickle=False)
    if depth.ndim != 2 or depth.dtype.kind not in 'fc':
        raise ValueError('depth_npy_base64 must be a 2-D floating array')
    return depth.astype(float)


def parse_K(value: Any) -> Dict[str, float]:
    """Parses camera intrinsic matrix from dict or 3x3 array into fx, fy, cx, cy."""
    if isinstance(value, dict):
        out = {k: float(value[k]) for k in ('fx', 'fy', 'cx', 'cy')}
    else:
        a = arr(value, (3, 3))
        out = {'fx': float(a[0, 0]), 'fy': float(a[1, 1]), 'cx': float(a[0, 2]), 'cy': float(a[1, 2])}
    if out['fx'] <= 0 or out['fy'] <= 0:
        raise ValueError('invalid K')
    return out


def parse_T_m(req: Dict[str, Any]) -> np.ndarray:
    """Validates chassis<-camera extrinsics and normalizes its translation to metres."""
    T = arr(req.get('T_chassis_camera'), (4, 4))
    unit = req.get('T_unit')
    if unit not in ('m', 'mm'):
        raise ValueError('T_unit must be explicitly m or mm')
    if (
        req.get('camera_frame') != 'head_camera_color_optical_frame'
        or req.get('base_frame') != 'chassis_link'
    ):
        raise ValueError('camera_frame/base_frame must be head_camera_color_optical_frame -> chassis_link')
    if not np.allclose(T[3], [0, 0, 0, 1], atol=1e-8):
        raise ValueError('invalid homogeneous bottom row')
    R = T[:3, :3]
    if not np.allclose(R.T @ R, np.eye(3), atol=1e-5) or not np.isclose(np.linalg.det(R), 1.0, atol=1e-5):
        raise ValueError('invalid rotation')
    if unit == 'mm':
        T = T.copy()
        T[:3, 3] /= 1000.0
    return T


def common_input(req: Dict[str, Any]) -> Tuple[np.ndarray, np.ndarray, Dict[str, float], np.ndarray]:
    """Decodes and validates standard RGB, depth, K, and T_m inputs."""
    rgb = decode_b64_image(req['rgb_base64'])
    depth = decode_npy_b64(req['depth_npy_base64'])
    if req.get('depth_unit') != 'mm':
        raise ValueError('depth_unit must be explicitly mm')
    K = parse_K(req['K'])
    T_m = parse_T_m(req)
    if rgb.shape[:2] != depth.shape:
        raise ValueError('RGB/depth dimensions do not match')
    return rgb, depth, K, T_m


def normalize_request_target(req: Dict[str, Any]) -> str:
    """Normalizes the strict robot-facing target_type/sku_typ/side contract."""
    target_type = req.get('target_type', 'sku')
    if target_type not in ('sku', 'basket'):
        raise ValueError('target_type must be sku or basket')

    if target_type == 'basket':
        side = req.get('side')
        if side is not None and side not in ('LEFT', 'RIGHT'):
            raise ValueError('side must be LEFT or RIGHT when provided')
        if 'sku_typ' in req or 'sku_id' in req or 'class_name' in req:
            raise ValueError('basket requests must not include sku_typ, sku_id or class_name')
        req['target_type'] = 'basket'
        req['class_name'] = 'Basket'
        req['sku_typ'] = None
        req['_target_context'] = {
            'target_type': 'basket',
            'side_received': side,
            'side_applied': False,
            'side_note': 'ignored_for_basket_mode',
        }
        return 'Basket'

    if 'sku_id' in req or 'class_name' in req:
        raise ValueError('sku_id/class_name were removed; send sku_typ=bottle, box or tube')

    sku_typ = req.get('sku_typ')
    if not isinstance(sku_typ, str) or not sku_typ.strip():
        raise ValueError('sku_typ must be a non-empty string for target_type=sku')
    class_name = sku_typ.strip()
    if class_name not in CLASS_CONFIG:
        raise ValueError('sku_typ must be bottle, box or tube')

    side = req.get('side')
    if side is not None and side not in ('LEFT', 'RIGHT'):
        raise ValueError('side must be LEFT or RIGHT when provided')

    raw_box = req.get('box_selection')
    if raw_box is not None and not isinstance(raw_box, dict):
        raise ValueError('box_selection must be an object')
    if side is None and not (isinstance(raw_box, dict) and raw_box.get('target_box') in (1, 2)):
        raise ValueError('SKU requests require side=LEFT/RIGHT or diagnostic box_selection.target_box=1/2')

    req['target_type'] = 'sku'
    req['sku_typ'] = class_name
    req['class_name'] = class_name
    req['_target_context'] = {
        'target_type': 'sku',
        'sku_typ': class_name,
        'side_received': side,
        'side_applied': side is not None,
        'side_note': None,
    }
    if side is not None:
        target_box = 1 if side == 'LEFT' else 2
        raw = req.get('box_selection')
        if raw is None:
            raw = {}
            req['box_selection'] = raw
        if not isinstance(raw, dict):
            raise ValueError('box_selection must be an object')
        if raw.get('target_box') not in (None, target_box):
            raise ValueError('side conflicts with box_selection.target_box')
        raw.setdefault('target_box', target_box)

    return class_name


def jsonable(x: Any) -> Any:
    """JSON serialization default converter for numpy arrays and scalars."""
    if isinstance(x, np.ndarray):
        return x.tolist()
    if isinstance(x, (np.floating, np.integer)):
        return x.item()
    raise TypeError(type(x).__name__)
