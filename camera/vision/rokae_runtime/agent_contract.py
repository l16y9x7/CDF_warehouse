"""Main-plan camera contract; existing device ownership remains unchanged."""
import cv2
import numpy as np
from .capture_api import resolve_camera


def query(qs, *, rgbd=False, stream=False):
    resolved = resolve_camera(qs.get('camera', qs.get('camera_id', '')))
    if resolved is None:
        raise ValueError('CAMERA_NOT_FOUND')
    camera, internal = resolved
    kind = 'depth' if rgbd else qs.get('type', 'color' if stream else '')
    if kind not in ('color', 'depth'):
        raise ValueError('INVALID_TYPE')
    if kind == 'depth' and camera != 'head':
        raise ValueError('DEPTH_UNSUPPORTED')
    fmt = 'raw' if rgbd else qs.get('format')
    if kind == 'depth' and fmt not in ('raw', 'preview'):
        raise ValueError('INVALID_FORMAT')
    if stream and kind == 'depth' and fmt != 'preview':
        raise ValueError('DEPTH_STREAM_REQUIRES_PREVIEW')
    return camera, internal, kind, fmt


def snapshot(owner, qs, *, rgbd=False):
    camera, internal, kind, fmt = query(qs, rgbd=rgbd)
    result = owner.capture(contract=camera, internal=internal,
                           streams={'color', 'depth'} if rgbd else {kind}, format=fmt)
    if not result.get('ok'):
        return result
    out = dict(ok=True, camera=camera, capture_id=result['capture_id'],
               captured_at=result.get('captured_at'), timestamps=result.get('timestamps'),
               same_shot=result.get('same_shot'), color_intrinsics=result.get('color_intrinsics'))
    if rgbd:
        out.update(rgb=result['color']['path'], depth=result['depth']['path'], t_unit='mm')
    else:
        part = result[kind]
        out.update(type=kind, format=part['format'], image_path=part['path'],
                   width=part['width'], height=part['height'])
    return out


def stream_frame(owner, camera, kind):
    if kind == 'color':
        return owner.get_jpeg(camera)
    result = owner.get_depth_mm(camera)
    if result is None:
        return None
    depth, _ = result
    preview = cv2.applyColorMap(np.uint8(np.clip(depth / 3000.0 * 255, 0, 255)), cv2.COLORMAP_JET)
    preview[depth == 0] = 0
    ok, image = cv2.imencode('.jpg', preview)
    return image.tobytes() if ok else None
