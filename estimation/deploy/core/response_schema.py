#!/usr/bin/env python3
"""Response formatting, artifact assembly, empty fields schema, and disk persistence.

Ensures that all error responses, intermediate visualization artifacts,
and request summaries strictly follow the HTTP localization contract.
"""
import base64
from concurrent.futures import ThreadPoolExecutor
import hashlib
import json
from pathlib import Path
import threading
from typing import Any, Dict, List, Optional, Tuple, Union
import cv2
import numpy as np

try:
    from config_loader import SAVE_REQUEST_INPUTS
    from request_codec import jsonable
except ImportError:
    from deploy.config_loader import SAVE_REQUEST_INPUTS
    from deploy.request_codec import jsonable

_ASYNC_IO_POOL = ThreadPoolExecutor(max_workers=2, thread_name_prefix='deploy_async_io')

_EMBED = threading.local()


def set_embed_visualizations(enabled: bool) -> None:
    """Sets whether visualization base64 payloads are embedded in the response."""
    _EMBED.enabled = bool(enabled)


def is_embed_visualizations() -> bool:
    """Returns True if visualization base64 embedding is requested."""
    return bool(getattr(_EMBED, 'enabled', False))


CATEGORY_EMPTY_FIELDS: Dict[str, Dict[str, Any]] = {
    'bottle': {
        'localization_method': 'bottle_cylinder_axis',
        'axis_fit_valid': False,
        'reference_point_valid': False,
        'axis_point_camera_mm': None,
        'axis_direction_camera_up': None,
        'reference_point_camera_mm': None,
        'reference_point_chassis_mm': None,
    },
    'box': {
        'localization_method': 'box_top_surface',
        'top_plane_valid': False,
        'top_point_valid': False,
        'point_semantics': None,
        'point_source': None,
        'fallback_used': False,
        'fallback_reason': None,
        'top_point_camera_mm': None,
        'top_point_chassis_mm': None,
        'top_point_uv': None,
    },
    'tube': {
        'localization_method': 'tube_height_band_edge',
        'tube_fit_mode': 'height_band',
        'edge_valid': False,
        'point_valid': False,
        'top_edge_center_camera_mm': None,
        'top_edge_center_chassis_mm': None,
        'top_edge_endpoints_camera_mm': None,
        'edge_direction_camera': None,
    },
}


def image_b64(image: np.ndarray) -> str:
    """Encodes a BGR image array into a JPEG base64 string."""
    ok, encoded = cv2.imencode('.jpg', image, [cv2.IMWRITE_JPEG_QUALITY, 90])
    if not ok:
        raise ValueError('failed to encode overlay')
    return base64.b64encode(encoded.tobytes()).decode('ascii')


def file_b64(path: Union[str, Path]) -> str:
    """Reads a binary file and returns its base64 encoded string."""
    return base64.b64encode(Path(path).read_bytes()).decode('ascii')


def save_json(path: Path, data: Any) -> None:
    """Writes JSON serialized data with 2-space indentation and UTF-8 encoding."""
    path.write_text(json.dumps(data, ensure_ascii=False, indent=2, default=jsonable), encoding='utf-8')


def input_summary(
    req: Dict[str, Any],
    rgb: np.ndarray,
    depth: np.ndarray,
    K: Dict[str, float]
) -> Dict[str, Any]:
    """Generates standard input summary for diagnostic logging."""
    return {
        'rgb_shape': list(rgb.shape),
        'depth_shape': list(depth.shape),
        'depth_unit': req.get('depth_unit'),
        'K': K,
        'T_unit_input': req.get('T_unit'),
        'T_translation_normalized_unit': 'm',
        'geometry_internal_unit': 'mm',
        'camera_frame': req.get('camera_frame'),
        'base_frame': req.get('base_frame'),
    }


def save_all_overlay(
    rgb: np.ndarray,
    detections: List[Dict[str, Any]],
    path: Path
) -> np.ndarray:
    """Draws all detection masks and confidence scores onto an RGB copy and saves to disk."""
    overlay = rgb.copy()
    for idx, d in enumerate(detections, 1):
        m = d['_mask']
        overlay[m] = (0.65 * overlay[m] + 0.35 * np.array([0, 210, 255])).astype(np.uint8)
        x, y, w, h = map(int, d.get('bbox', [0, 0, 0, 0]))
        label = '#{} {:.3f}'.format(int(d.get('upstream_instance_id', idx)), float(d.get('score', 0)))
        cv2.putText(overlay, label, (x, max(15, y - 4)), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 0, 0), 3, cv2.LINE_AA)
        cv2.putText(overlay, label, (x, max(15, y - 4)), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (255, 255, 255), 1, cv2.LINE_AA)
    cv2.imwrite(str(path), overlay)
    return overlay


def artifact_payload(
    root: Path,
    all_overlay: Optional[np.ndarray],
    selected_overlay: Optional[Union[np.ndarray, Path]] = None,
    ply_path: Optional[Union[str, Path]] = None,
    extra: Optional[Dict[str, Any]] = None,
    embed: Optional[bool] = None
) -> Dict[str, Any]:
    """Assembles file paths and optional base64 payloads for request artifacts."""
    should_embed = is_embed_visualizations() if embed is None else embed
    sel_path = root / 'selected_overlay.jpg'
    out = {
        'all_instances_overlay': str(root / 'all_instances_overlay.jpg'),
        'json_log': str(root / 'response.json'),
    }
    if should_embed and all_overlay is not None:
        out['all_instances_overlay_jpeg_base64'] = image_b64(all_overlay)

    if selected_overlay is not None or sel_path.is_file():
        out['selected_instance_overlay'] = str(sel_path)
        if should_embed:
            sel_img = (
                selected_overlay
                if isinstance(selected_overlay, np.ndarray)
                else (cv2.imread(str(sel_path)) if sel_path.is_file() else None)
            )
            if sel_img is not None:
                out['selected_overlay_jpeg_base64'] = image_b64(sel_img)

    if ply_path is not None and Path(ply_path).exists():
        out['point_cloud'] = str(ply_path)
        if should_embed:
            out['point_cloud_ply_base64'] = file_b64(ply_path)

    if extra:
        out.update(extra)
    return out


def attach_two_stage_artifacts(
    response: Dict[str, Any],
    root: Path,
    images: Dict[str, Tuple[Path, Optional[np.ndarray]]],
    embed: Optional[bool] = None
) -> Dict[str, Any]:
    """Attaches visualization file paths and optional base64 payloads to response artifacts."""
    should_embed = is_embed_visualizations() if embed is None else embed
    artifacts = response.setdefault('artifacts', {})
    for key, (path, image) in images.items():
        artifacts[key] = str(path)
        if should_embed and image is not None:
            artifacts[key + '_jpeg_base64'] = image_b64(image)
    return response


def _do_persist_inputs(root: Path, raw_body: bytes, req: Dict[str, Any]) -> None:
    """Worker function for async persistence of raw replay inputs."""
    try:
        request_path = root / 'request.json'
        request_path.write_bytes(raw_body)
        metadata = {
            k: v for k, v in req.items()
            if k not in ('rgb_base64', 'depth_npy_base64') and not str(k).startswith('_')
        }
        save_json(root / 'request_metadata.json', metadata)
        manifest = {
            'schema_version': 1,
            'request_file': request_path.name,
            'request_sha256': hashlib.sha256(raw_body).hexdigest(),
            'request_bytes': len(raw_body),
            'metadata_file': 'request_metadata.json',
            'inputs': {},
            'decode_errors': [],
        }
        for key, default_name in (('rgb_base64', 'input_rgb.bin'), ('depth_npy_base64', 'input_depth.npy')):
            try:
                raw = base64.b64decode(req[key], validate=True)
                name = default_name
                if key == 'rgb_base64':
                    if raw.startswith(b'\xff\xd8\xff'):
                        name = 'input_rgb.jpg'
                    elif raw.startswith(b'\x89PNG\r\n\x1a\n'):
                        name = 'input_rgb.png'
                (root / name).write_bytes(raw)
                manifest['inputs'][key] = {
                    'file': name,
                    'bytes': len(raw),
                    'sha256': hashlib.sha256(raw).hexdigest(),
                }
            except Exception as exc:
                manifest['decode_errors'].append({
                    'field': key,
                    'error_type': type(exc).__name__,
                    'error': str(exc),
                })
        save_json(root / 'request_manifest.json', manifest)
    except Exception as exc:
        # Non-blocking structured error record
        try:
            err_log = {
                'stage': 'persist_request_inputs',
                'error_type': type(exc).__name__,
                'error_message': str(exc),
            }
            save_json(root / 'persist_error.json', err_log)
        except Exception:
            pass


def persist_request_inputs(root: Path, raw_body: bytes, req: Dict[str, Any]) -> Optional[bool]:
    """Asynchronously persists request body and decoded inputs for exact offline replay."""
    if not SAVE_REQUEST_INPUTS:
        return None
    _ASYNC_IO_POOL.submit(_do_persist_inputs, root, raw_body, req)
    return True


def _clean_vec(v: Any) -> Optional[List[Any]]:
    """Normalizes vector/list, converting ndarrays or sequences to standard Python float lists."""
    if v is None:
        return None
    if isinstance(v, np.ndarray):
        v = v.tolist()
    if isinstance(v, (list, tuple)):
        cleaned = []
        for x in v:
            if x is None:
                cleaned.append(None)
            elif isinstance(x, (int, float)):
                f = float(x)
                cleaned.append(None if np.isnan(f) or np.isinf(f) else f)
            elif isinstance(x, (list, tuple, np.ndarray)):
                cleaned.append(_clean_vec(x))
            else:
                cleaned.append(x)
        return cleaned
    return None


def _clean_mat(m: Any) -> Optional[List[List[Any]]]:
    """Normalizes matrix, converting ndarrays or nested sequences to lists of float lists."""
    if m is None:
        return None
    if isinstance(m, np.ndarray):
        m = m.tolist()
    if isinstance(m, (list, tuple)):
        return [_clean_vec(row) for row in m]
    return None


def _clean_float(val: Any) -> Optional[float]:
    """Safely converts a scalar to float, returning None on NaN, Inf, or failure."""
    if val is None:
        return None
    try:
        f = float(val)
        return None if np.isnan(f) or np.isinf(f) else f
    except (TypeError, ValueError):
        return None


def _clean_int(val: Any) -> Optional[int]:
    """Safely converts a scalar to int, returning None on failure."""
    if val is None:
        return None
    try:
        return int(val)
    except (TypeError, ValueError):
        return None


def _prune_heavy_objects(obj: Any, depth: int = 0) -> Any:
    """Recursively prunes point clouds, masks, and internal objects from diagnostics."""
    if depth > 6:
        return None
    if isinstance(obj, np.ndarray):
        if obj.size <= 25:
            return obj.tolist()
        return None
    if isinstance(obj, (int, float, bool, str)) or obj is None:
        return obj
    if isinstance(obj, dict):
        cleaned = {}
        for k, v in obj.items():
            k_str = str(k)
            if k_str.startswith('_'):
                continue
            if k_str in ('raw', 'fit', 'ins', 'fm', 'analyses', 'camera_points', 'top_mask',
                         'keep', 'mask', '_mask', 'center_fit', 'left_fit', 'right_fit',
                         'main_analysis', 'points', 'point_cloud'):
                continue
            pruned = _prune_heavy_objects(v, depth + 1)
            if pruned is not None:
                cleaned[k] = pruned
        return cleaned
    if isinstance(obj, (list, tuple)):
        if len(obj) > 100 and all(isinstance(x, (int, float, bool)) for x in obj[:10]):
            return None
        cleaned_list = []
        for x in obj:
            pruned = _prune_heavy_objects(x, depth + 1)
            if pruned is not None:
                cleaned_list.append(pruned)
        return cleaned_list
    return str(obj)


def sanitize_diagnostics(diag: Any) -> Dict[str, Any]:
    """Sanitizes diagnostic dict, stripping heavy point arrays and internal fit data."""
    if not isinstance(diag, dict):
        return {}
    res = _prune_heavy_objects(diag)
    return res if isinstance(res, dict) else {}


def sanitize_artifacts(artifacts: Any, return_visualizations: bool = False) -> Dict[str, Any]:
    """Sanitizes artifacts dict, stripping base64 payloads unless explicitly requested."""
    if not isinstance(artifacts, dict):
        return {}
    cleaned = {}
    for k, v in artifacts.items():
        k_str = str(k)
        if k_str.endswith('_base64'):
            if return_visualizations:
                cleaned[k_str] = v
        else:
            if isinstance(v, (str, Path)):
                cleaned[k_str] = str(v)
            elif v is not None:
                cleaned[k_str] = v
    return cleaned


def sanitize_box_selection(box_sel: Any) -> Dict[str, Any]:
    """Sanitizes box selection audit dict, ensuring no raw mask arrays leak."""
    if not isinstance(box_sel, dict):
        return {}
    out = {}
    for k, v in box_sel.items():
        k_str = str(k)
        if k_str in ('_mask', 'mask') or k_str.startswith('_'):
            continue
        if k_str in ('object_membership', 'rejected_objects'):
            if isinstance(v, list):
                cleaned_objs = []
                for obj in v:
                    if isinstance(obj, dict):
                        cleaned_objs.append({
                            ok: ov for ok, ov in obj.items()
                            if ok not in ('_mask', 'mask') and not str(ok).startswith('_')
                        })
                    else:
                        cleaned_objs.append(obj)
                out[k_str] = cleaned_objs
            else:
                out[k_str] = v
        else:
            pruned = _prune_heavy_objects(v)
            if pruned is not None:
                out[k_str] = pruned
    return out


def sanitize_targets(targets: Any) -> Dict[str, Any]:
    """Sanitizes multi-target slots (center, left, right) to retain only contract fields."""
    if not isinstance(targets, dict):
        return {}
    cleaned = {}
    for slot in ('center', 'left', 'right'):
        item = targets.get(slot)
        if isinstance(item, dict):
            cleaned[slot] = {
                'instance_id': _clean_int(item.get('instance_id')),
                'valid': bool(item.get('valid', False)),
                'point_camera_mm': _clean_vec(item.get('point_camera_mm')),
                'point_chassis_mm': _clean_vec(item.get('point_chassis_mm')),
                'axis_direction_camera_up': _clean_vec(item.get('axis_direction_camera_up')),
                'axis_direction_robot': _clean_vec(item.get('axis_direction_robot')),
                'score': _clean_float(item.get('score')),
                'reasons': list(item.get('reasons', [])),
                'reason': str(item.get('reason', '')),
            }
    return cleaned


def build_robot_response(result: Dict[str, Any], req: Optional[Dict[str, Any]] = None) -> Dict[str, Any]:
    """Constructs a strict, whitelisted HTTP response conforming to ROBOT_API_HANDOFF.md.

    Strips all internal point clouds, fit arrays, masks, and center_fit/left_fit/right_fit
    from wire responses, and only embeds base64 visualizations when return_visualizations is True.
    """
    if not isinstance(result, dict):
        return {'ok': False, 'error': 'Invalid result object'}

    req = req or {}
    return_viz = bool(req.get('return_visualizations', False))

    if not result.get('ok', False) and 'error' in result and 'sku_typ' not in result and 'target_type' not in result:
        return {
            'ok': False,
            'request_id': result.get('request_id'),
            'error': str(result.get('error')),
            'error_type': str(result.get('error_type', 'RuntimeError')),
        }

    target_type = result.get('target_type') or req.get('target_type') or 'sku'

    if target_type == 'basket':
        return _build_basket_response(result, req, return_viz)
    return _build_sku_response(result, req, return_viz)


def _build_basket_response(result: Dict[str, Any], req: Dict[str, Any], return_viz: bool) -> Dict[str, Any]:
    """Whitelisted serializer for target_type=basket."""
    return {
        'ok': bool(result.get('ok', False)),
        'request_id': result.get('request_id'),
        'target_type': 'basket',
        'pose_valid': bool(result.get('pose_valid', False)),
        'model_center_camera_mm': _clean_vec(result.get('model_center_camera_mm')),
        'reference_point_camera_mm': _clean_vec(result.get('reference_point_camera_mm')),
        'reference_point_chassis_mm': _clean_vec(result.get('reference_point_chassis_mm')),
        'xyz_camera_mm': _clean_vec(result.get('xyz_camera_mm')),
        'point_semantics': result.get('point_semantics'),
        'object_origin_camera_mm': _clean_vec(result.get('object_origin_camera_mm')),
        'model_center_offset_m': _clean_vec(result.get('model_center_offset_m')),
        'pose_4x4': _clean_mat(result.get('pose_4x4')),
        'pose_4x4_input_m': _clean_mat(result.get('pose_4x4_input_m')),
        'rotation_euler_zyx_rad': _clean_vec(result.get('rotation_euler_zyx_rad')),
        'xyzrxryrz_camera_mm_rad': _clean_vec(result.get('xyzrxryrz_camera_mm_rad')),
        'selected_instance_id': _clean_int(result.get('selected_instance_id')),
        'sam3_score': _clean_float(result.get('sam3_score')),
        'foundationpose_http_status': _clean_int(result.get('foundationpose_http_status')),
        'foundationpose_wall_ms': _clean_float(result.get('foundationpose_wall_ms')),
        'output_frame': result.get('output_frame', req.get('camera_frame')),
        'output_unit': result.get('output_unit', 'mm'),
        'reference_frame': result.get('reference_frame', req.get('base_frame')),
        'cad_id': result.get('cad_id'),
        'cad_path': result.get('cad_path'),
        'mesh_scale': result.get('mesh_scale'),
        'foundationpose_url': result.get('foundationpose_url'),
        'side_audit': result.get('side_audit'),
        'rejection_reasons': list(result.get('rejection_reasons', [])),
        'diagnostics': sanitize_diagnostics(result.get('diagnostics', {})),
        'artifacts': sanitize_artifacts(result.get('artifacts', {}), return_viz),
    }


def _build_sku_response(result: Dict[str, Any], req: Dict[str, Any], return_viz: bool) -> Dict[str, Any]:
    """Whitelisted serializer for target_type=sku (bottle, box, tube)."""
    sku_typ = result.get('sku_typ') or req.get('sku_typ') or 'bottle'
    class_name = result.get('class_name') or sku_typ

    resp: Dict[str, Any] = {
        'ok': bool(result.get('ok', False)),
        'request_id': result.get('request_id'),
        'target_type': 'sku',
        'sku_typ': sku_typ,
        'class_name': class_name,
        'sam3_call_count': _clean_int(result.get('sam3_call_count', 2 if result.get('ok') else 1)),
        'localization_method': result.get(
            'localization_method',
            CATEGORY_EMPTY_FIELDS.get(sku_typ, {}).get('localization_method')
        ),
        'selected_instance_id': _clean_int(result.get('selected_instance_id')),
        'upstream_instance_id': _clean_int(result.get('upstream_instance_id', result.get('selected_instance_id'))),
        'filtered_instance_id': _clean_int(result.get('filtered_instance_id')),
        'sam3_score': _clean_float(result.get('sam3_score')),
        'output_frame': result.get('output_frame', req.get('camera_frame', 'head_camera_color_optical_frame')),
        'output_unit': result.get('output_unit', 'mm'),
        'side_audit': result.get('side_audit'),
        'pipeline_steps': list(result.get('pipeline_steps', [])),
        'rejection_reasons': list(result.get('rejection_reasons', [])),
    }

    if 'box_selection' in result:
        resp['box_selection'] = sanitize_box_selection(result['box_selection'])

    # Front panel plane and edge fields
    resp['front_panel_valid'] = bool(result.get('front_panel_valid', False))
    resp['front_panel_top_edge_midpoint_camera_mm'] = _clean_vec(result.get('front_panel_top_edge_midpoint_camera_mm'))
    resp['front_panel_plane_point_camera_mm'] = _clean_vec(result.get('front_panel_plane_point_camera_mm'))
    resp['front_panel_plane_normal_camera'] = _clean_vec(result.get('front_panel_plane_normal_camera'))
    if 'front_panel_top_edge_midpoint_chassis_mm' in result:
        resp['front_panel_top_edge_midpoint_chassis_mm'] = _clean_vec(result.get('front_panel_top_edge_midpoint_chassis_mm'))
    if 'front_panel_plane_point_chassis_mm' in result:
        resp['front_panel_plane_point_chassis_mm'] = _clean_vec(result.get('front_panel_plane_point_chassis_mm'))
    if 'front_panel_plane_normal_chassis' in result:
        resp['front_panel_plane_normal_chassis'] = _clean_vec(result.get('front_panel_plane_normal_chassis'))

    # Category-specific formal fields
    if sku_typ == 'bottle':
        resp.update({
            'axis_fit_valid': bool(result.get('axis_fit_valid', False)),
            'reference_point_valid': bool(result.get('reference_point_valid', False)),
            'axis_point_camera_mm': _clean_vec(result.get('axis_point_camera_mm')),
            'axis_direction_camera_up': _clean_vec(result.get('axis_direction_camera_up')),
            'reference_point_camera_mm': _clean_vec(result.get('reference_point_camera_mm')),
            'reference_point_chassis_mm': _clean_vec(result.get('reference_point_chassis_mm')),
            'reference_z_mm': None,
            'reference_mode': result.get('reference_mode', 'visible_axis_midpoint'),
        })
        if 'body_radius_mm' in result:
            resp['body_radius_mm'] = _clean_float(result['body_radius_mm'])
        if 'fit_mode' in result:
            resp['fit_mode'] = result['fit_mode']
        if 'selection_reason' in result:
            resp['selection_reason'] = result['selection_reason']

    elif sku_typ == 'box':
        resp.update({
            'top_plane_valid': bool(result.get('top_plane_valid', False)),
            'top_point_valid': bool(result.get('top_point_valid', False)),
            'point_semantics': result.get('point_semantics'),
            'point_source': result.get('point_source'),
            'fallback_used': bool(result.get('fallback_used', False)),
            'fallback_reason': result.get('fallback_reason'),
            'top_point_camera_mm': _clean_vec(result.get('top_point_camera_mm')),
            'top_point_chassis_mm': _clean_vec(result.get('top_point_chassis_mm')),
            'top_point_uv': _clean_vec(result.get('top_point_uv')),
        })
        if 'top_plane_point_camera_mm' in result:
            resp['top_plane_point_camera_mm'] = _clean_vec(result.get('top_plane_point_camera_mm'))
        if 'top_plane_point_chassis_mm' in result:
            resp['top_plane_point_chassis_mm'] = _clean_vec(result.get('top_plane_point_chassis_mm'))
        if 'plane_normal_camera' in result:
            resp['plane_normal_camera'] = _clean_vec(result.get('plane_normal_camera'))
        if 'plane_normal_chassis' in result:
            resp['plane_normal_chassis'] = _clean_vec(result.get('plane_normal_chassis'))
        if 'plane_mode' in result:
            resp['plane_mode'] = result['plane_mode']
        if 'selection_reason' in result:
            resp['selection_reason'] = result['selection_reason']

    elif sku_typ == 'tube':
        resp.update({
            'tube_fit_mode': result.get('tube_fit_mode', 'height_band'),
            'edge_valid': bool(result.get('edge_valid', False)),
            'point_valid': bool(result.get('point_valid', False)),
            'point_semantics': result.get('point_semantics', 'visible_top_edge_midpoint' if result.get('point_valid') else None),
            'top_edge_center_camera_mm': _clean_vec(result.get('top_edge_center_camera_mm')),
            'top_edge_center_chassis_mm': _clean_vec(result.get('top_edge_center_chassis_mm')),
            'top_edge_endpoints_camera_mm': _clean_mat(result.get('top_edge_endpoints_camera_mm')),
            'edge_direction_camera': _clean_vec(result.get('edge_direction_camera')),
        })
        if 'selection_reason' in result:
            resp['selection_reason'] = result['selection_reason']

    # Optional multi-target / candidate slots
    if 'selection_strategy' in result:
        resp['selection_strategy'] = result['selection_strategy']
    if 'grasp_order' in result:
        resp['grasp_order'] = list(result['grasp_order'])
    if 'targets' in result and isinstance(result['targets'], dict):
        resp['targets'] = sanitize_targets(result['targets'])

    # Optional audit summaries
    if 'input_summary' in result and isinstance(result['input_summary'], dict):
        resp['input_summary'] = result['input_summary']
    if 'sam3_response_summary' in result and isinstance(result['sam3_response_summary'], dict):
        resp['sam3_response_summary'] = result['sam3_response_summary']

    # Diagnostics and artifacts
    resp['diagnostics'] = sanitize_diagnostics(result.get('diagnostics', {}))
    resp['artifacts'] = sanitize_artifacts(result.get('artifacts', {}), return_viz)

    return resp


sanitize_http_response = build_robot_response
