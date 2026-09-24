#!/usr/bin/env python3
"""Target localization pipeline orchestrator for SKU and Basket recognition.

Encapsulates:
1. Two-stage box selection then SKU candidate detection;
2. Concurrent SAM3 dispatch for dual-parallel inference mode;
3. Category-specific geometric fitting (bottle, box, tube);
4. Basket localization via FoundationPose.
"""
from concurrent.futures import ThreadPoolExecutor
import json
from pathlib import Path
import shutil
import threading
import time
from typing import Any, Dict, List, Optional, Tuple, Union

import cv2
import numpy as np
import requests

from config_loader import (
    ASYNC_3D_RENDER,
    BASKET_DEFAULT_PROMPT,
    BASKET_DEFAULT_THRESHOLD,
    BASKET_FP_REGISTERED_CAD,
    BASKET_FP_URL,
    BASKET_MESH_PATH,
    BASKET_MESH_SCALE,
    CLASS_CONFIG,
    CONTAINER_BOX_PROMPT,
    ENABLE_3D_RENDER,
    FRONT_RULE_DEFAULT,
    SAM3_URL,
)
from geometry_cache import GeometryCache
from module_loader import get_deploy_modules
from request_codec import (
    arr,
    common_input,
    normalize_request_target,
)
from response_schema import (
    CATEGORY_EMPTY_FIELDS,
    artifact_payload,
    attach_two_stage_artifacts,
    file_b64,
    input_summary,
    is_embed_visualizations,
    save_all_overlay,
    save_json,
    set_embed_visualizations,
)
from sam3_client import (
    call_sam3,
    decode_detection_mask,
    encode_mask,
    prepare_detections,
    rank_by_score,
    sam_summary,
    score_filter,
    setup_category_detections,
)

MODULES = get_deploy_modules()
FIT = MODULES.fit_bottle_axis
BOX = MODULES.fit_estee_box_top_surface
BOXSEL = MODULES.box_selection
TUBE = MODULES.fit_tube_top_edge
FRONT = MODULES.fit_front_panel_plane
BGC = MODULES.box_geometric_center
BOX_PIPE = MODULES.box_pipeline

_ASYNC_IO_POOL = ThreadPoolExecutor(max_workers=2, thread_name_prefix='deploy_async_io')


def dispatch_3d_render(fn, *args):
    """Dispatch 3D render function conditionally and asynchronously without blocking request."""
    if not ENABLE_3D_RENDER:
        return None
    if ASYNC_3D_RENDER:
        return _ASYNC_IO_POOL.submit(fn, *args)
    return fn(*args)


def parse_box_selection(req: Dict[str, Any], class_cfg: Optional[Dict[str, Any]] = None) -> Dict[str, Any]:
    """Delegates box selection configuration parsing to box_selection module."""
    return BOXSEL.parse_box_selection(req, class_cfg, CONTAINER_BOX_PROMPT)


def box_overlay(rgb: np.ndarray, detections: list, rois: list, indices: list, target_box: int, path: Path):
    """Delegates box selection overlay visualization to box_selection module."""
    return BOXSEL.box_overlay(rgb, detections, rois, indices, target_box, path, decoder=decode_detection_mask)


def select_front(detections: list, front_rule: Optional[dict]):
    """Delegates front row candidate selection to box_selection module."""
    return BOXSEL.select_front(detections, front_rule)


def apply_front_rule(items: list, shape: tuple, depth: np.ndarray, K: dict, T_m: np.ndarray, front_rule: Optional[dict], decoder, geometry_cache):
    """Delegates front rule spatial audit to box_selection module."""
    return BOXSEL.apply_front_rule(items, shape, depth, K, T_m, front_rule, decoder, geometry_cache)


def fit_selected_box_front_panel(selected_box, rgb, depth, K, T_m, req, geometry_cache, path):
    """Delegates container box front panel plane estimation to fit_front_panel_plane module."""
    return FRONT.fit_selected_box_front_panel(
        selected_box, rgb, depth, K, T_m, req, geometry_cache, path,
        estee_module=BOX, decoder=decode_detection_mask
    )


def box_surface_cfg(req: dict) -> dict:
    """Constructs box top planar surface fitting parameters from request overrides or defaults."""
    return {
        'tol': float(req.get('height_tolerance_mm', BOX.DEFAULT_HEIGHT_TOLERANCE_MM)),
        'min_points': int(req.get('min_top_points', BOX.DEFAULT_MIN_TOP_POINTS)),
        'min_span': float(req.get('min_tangent_span_mm', BOX.DEFAULT_MIN_TANGENT_SPAN_MM)),
        'min_area': float(req.get('min_top_area_mm2', BOX.DEFAULT_MIN_TOP_AREA_MM2)),
        'max_tilt': float(req.get('normal_max_tilt_deg', BOX.DEFAULT_NORMAL_MAX_TILT_DEG)),
        'max_med': float(req.get('max_residual_median_mm', BOX.DEFAULT_MAX_RESIDUAL_MEDIAN_MM)),
        'max_p90': float(req.get('max_residual_p90_mm', BOX.DEFAULT_MAX_RESIDUAL_P90_MM)),
        'min_margin': float(req.get('min_interior_margin_px', BOX.DEFAULT_MIN_INTERIOR_MARGIN_PX)),
    }


def process_bottle(req, request_id, root, rgb, depth, K, T_m, sam, prompt, geometry_cache=None, mark=None):
    """Coordinates bottle cylinder axis fitting and visible axis midpoint reference point."""
    mark = mark or (lambda name, t0: time.perf_counter())
    geometry_cache = geometry_cache or GeometryCache()
    detections = setup_category_detections(sam.get('detections', []), 'bottle')
    t0 = time.perf_counter()
    all_overlay = save_all_overlay(rgb, detections, root / 'all_instances_overlay.jpg')
    mark('all_instances_overlay', t0)

    for d in detections:
        if '_median_chassis_point' not in d:
            raw, _, _, _ = geometry_cache.get_mask(d['_mask'], depth, K, T_m)
            d['_median_chassis_point'] = (
                FIT.to_chassis(np.median(raw, axis=0), np.array([0.0, 0.0, 1.0]), T_m)[0].tolist()
                if len(raw) else None
            )

    candidates, front_info = select_front(detections, req.get('front_rule'))
    radius = float(req.get('body_radius_mm', 28.5))
    mode = req.get('fit_mode', 'cylinder_3d')
    if mode not in ('cylinder_3d', 'prior_2d'):
        raise ValueError('bottle fit_mode must be cylinder_3d or prior_2d')

    common = {
        'request_id': request_id,
        'sku_typ': 'bottle',
        'class_name': 'bottle',
        'localization_method': 'bottle_cylinder_axis',
        'sam3_response_summary': sam_summary(sam, prompt, sam.get('_requested_threshold')),
        'front_row': front_info,
        'input_summary': input_summary(req, rgb, depth, K),
        'reference_mode': 'visible_axis_midpoint',
        'reference_z_mm': None,
        'body_radius_mm': radius,
        'fit_mode': mode,
        'output_frame': req['camera_frame'],
        'output_unit': 'mm',
    }

    if not candidates:
        common.update({
            'ok': False,
            'selected_instance_id': None,
            **CATEGORY_EMPTY_FIELDS['bottle'],
            'rejection_reasons': ['no front-row bottle candidate'],
            'diagnostics': {'front_candidates': []},
            'artifacts': artifact_payload(root, all_overlay),
        })
        return common

    iid = candidates[0][0]
    filtered_id = candidates[0][3]
    selected = detections[filtered_id - 1]
    mask = selected['_mask']
    raw, _, _, _ = geometry_cache.get_mask(mask, depth, K, T_m)
    prior = FIT.norm(T_m[:3, :3].T @ np.array([0.0, 0.0, 1.0]))

    try:
        t0 = time.perf_counter()
        fm = FIT.body_filter(mask, depth, 4, 'bottle_body')
        mark('body_filter', t0)

        t0 = time.perf_counter()
        fit, _, _, _ = geometry_cache.get_mask(fm, depth, K, T_m)
        mark('body_mask_point_cloud', t0)

        t0 = time.perf_counter()
        p, a, res, ins, extra, stab = FIT.robust_fit(
            fit, radius, prior if mode == 'prior_2d' else None, mode, None
        )
        mark('robust_fit_total', t0)

        a = FIT.canonical(a, prior)
        ts = (fit - p) @ a
        q = np.percentile(ts[ins], [5, 95])
        s_center = float(np.mean(q))
        ref = p + s_center * a
        refch, _ = FIT.to_chassis(ref, np.zeros(3), T_m)
    except Exception as exc:
        fm = np.zeros_like(mask)
        fit = np.empty((0, 3))
        ins = np.zeros(0, bool)
        overlay = rgb.copy()
        overlay[mask] = (0.65 * overlay[mask] + 0.35 * np.array([255, 160, 0])).astype(np.uint8)
        cv2.imwrite(str(root / 'selected_overlay.jpg'), overlay)
        ply = root / 'selected_points.ply'
        FIT.write_ply(ply, raw, fit, ins)
        common.update({
            'ok': False,
            'selected_instance_id': iid,
            'upstream_instance_id': iid,
            'filtered_instance_id': filtered_id,
            'sam3_score': selected.get('score'),
            'selection_reason': 'bottle + configured front band + highest SAM3 score; no fallback after fit failure',
            'axis_fit_valid': False,
            'reference_point_valid': False,
            'axis_point_camera_mm': None,
            'axis_direction_camera_up': None,
            'reference_point_camera_mm': None,
            'reference_point_chassis_mm': None,
            'rejection_reasons': ['cylinder fitting failed: ' + str(exc)],
            'diagnostics': {
                'sam3_detections': len(detections),
                'front_candidates': candidates,
                'mask_area_pixels': int(mask.sum()),
                'mask_valid_depth_ratio': float(np.count_nonzero(mask & np.isfinite(depth) & (depth > 0)) / max(1, mask.sum())),
                'fit_error_type': type(exc).__name__,
            },
            'artifacts': artifact_payload(root, all_overlay, overlay, ply),
        })
        return common

    t0 = time.perf_counter()
    maxvis = max(stab['visible_common_section_center_mm'], default=float('inf'))
    maxang = max(stab['axis_angle_deg'], default=float('inf'))
    axis_ok = bool(
        len(fit) >= FIT.MIN_BODY_FIT_POINTS
        and q[1] - q[0] >= FIT.MIN_VISIBLE_AXIS_SPAN_MM
        and ins.mean() >= 0.5
        and np.median(abs(res)) <= 3
        and np.percentile(abs(res), 90) <= 8
        and maxvis <= FIT.MAX_BOOTSTRAP_VISIBLE_CENTER_MM
        and (mode != 'cylinder_3d' or maxang <= FIT.MAX_BOOTSTRAP_AXIS_ANGLE_DEG)
        and not extra.get('candidate_solution_ambiguity', False)
    )
    ref_ok = bool(axis_ok)
    mark('quality_gate', t0)

    t0 = time.perf_counter()
    overlay = rgb.copy()
    overlay[mask] = (0.65 * overlay[mask] + 0.35 * np.array([255, 160, 0])).astype(np.uint8)
    overlay[fm] = (0.35 * overlay[fm] + 0.65 * np.array([0, 210, 0])).astype(np.uint8)
    uv = FIT.project(p[None] + q[:, None] * a, K).astype(int)
    cv2.line(overlay, tuple(uv[0]), tuple(uv[1]), (0, 0, 255), 3)
    if ref_ok and ref[2] > 0:
        u = FIT.project(ref[None], K)[0]
        if 0 <= u[0] < rgb.shape[1] and 0 <= u[1] < rgb.shape[0]:
            cv2.drawMarker(overlay, tuple(u.astype(int)), (255, 0, 255), cv2.MARKER_CROSS, 18, 2)
    ply = root / 'selected_points.ply'
    FIT.write_ply(ply, raw, fit, ins)
    cv2.imwrite(str(root / 'selected_overlay.jpg'), overlay)
    mark('selected_overlay_ply_write', t0)

    common.update({
        'ok': bool(axis_ok and ref_ok),
        'selected_instance_id': iid,
        'upstream_instance_id': iid,
        'filtered_instance_id': filtered_id,
        'sam3_score': selected.get('score'),
        'selection_reason': 'bottle + configured front band + highest SAM3 score; visible fitted body-axis midpoint; no fallback after fit failure',
        'axis_fit_valid': axis_ok,
        'reference_point_valid': ref_ok,
        'axis_point_camera_mm': p,
        'axis_direction_camera_up': FIT.canonical(a, prior),
        'reference_point_camera_mm': ref if ref_ok else None,
        'reference_point_chassis_mm': refch if ref_ok else None,
        'reference_z_note': 'not used for bottle reference point; reference is visible_axis_midpoint',
        'rejection_reasons': ([] if axis_ok else ['cylinder axis quality gate failed']),
        'diagnostics': {
            'sam3_detections': len(detections),
            'front_candidates': candidates,
            'fit_inlier_count': int(ins.sum()),
            'fit_region_point_count': len(fit),
            'mask_area_pixels': int(mask.sum()),
            'mask_valid_depth_ratio': float(np.count_nonzero(mask & np.isfinite(depth) & (depth > 0)) / max(1, mask.sum())),
            'radial_median_mm': float(np.median(abs(res))),
            'radial_p90_mm': float(np.percentile(abs(res), 90)),
            'visible_axis_span_mm': float(q[1] - q[0]),
            'visible_axis_t_range_mm': q,
            'visible_axis_midpoint_offset_mm': s_center,
            'bootstrap': stab,
            'fit_candidates': extra,
            'reference_point_semantics': 'midpoint of the 5th-95th percentile projection range of robust cylinder inliers',
            'reference_chassis_z_mm': float(refch[2]),
        },
        'artifacts': artifact_payload(root, all_overlay, overlay, ply),
    })
    return common


def process_tube(req, request_id, root, rgb, depth, K, T_m, sam, prompt, geometry_cache=None, mark=None):
    """Coordinates tube top edge center fitting and erosion stability evaluation."""
    mark = mark or (lambda name, t0: time.perf_counter())
    geometry_cache = geometry_cache or GeometryCache()
    detections = setup_category_detections(sam.get('detections', []), 'tube')
    all_overlay = save_all_overlay(rgb, detections, root / 'all_instances_overlay.jpg')
    ranked = rank_by_score(detections)

    common = {
        'request_id': request_id,
        'sku_typ': 'tube',
        'class_name': 'tube',
        'localization_method': 'tube_height_band_edge',
        'tube_fit_mode': 'height_band',
        'sam3_prompt': prompt,
        'sam3_response_summary': sam_summary(sam, prompt, sam.get('_requested_threshold')),
        'input_summary': input_summary(req, rgb, depth, K),
        'output_frame': req['camera_frame'],
        'output_unit': 'mm',
    }

    if not ranked:
        common.update({
            'ok': False,
            'selected_instance_id': None,
            **CATEGORY_EMPTY_FIELDS['tube'],
            'rejection_reasons': ['no tube SAM3 candidate'],
            'diagnostics': {'selection_candidates': []},
            'artifacts': artifact_payload(root, all_overlay),
        })
        return common

    iid = ranked[0][0]
    filtered_id = ranked[0][2]
    selected = detections[filtered_id - 1]
    mask = selected['_mask']
    erosion = int(req.get('mask_erosion_pixels', 1))

    cfg = TUBE.make_height_band_cfg(req)
    analyses = {
        str(e): TUBE.analyse_height_band(mask, depth, K, T_m, e, cfg, geometry_cache=geometry_cache)
        for e in (0, 1, 2)
    }
    main = analyses[str(erosion)]
    points = [a['reference_camera_mm'] for a in analyses.values() if a.get('valid')]
    point_var = (
        float(max(np.linalg.norm(np.asarray(a) - np.asarray(b)) for a in points for b in points))
        if len(points) >= 2 else None
    )
    depth_ratio = main.get('mask_valid_depth_ratio', 0)
    edge_ok = bool(main.get('valid'))
    point_ok = bool(
        edge_ok
        and point_var is not None
        and point_var <= TUBE.DEFAULT_MAX_POINT_STABILITY_MM
        and depth_ratio >= cfg['min_mask_valid_depth_ratio']
    )

    reasons = [] if edge_ok else [main.get('reason', 'top edge invalid')]
    if depth_ratio < cfg['min_mask_valid_depth_ratio']:
        reasons.append('valid depth ratio below threshold')
    if point_var is None:
        reasons.append('insufficient successful erosion variants for stability check')
    elif point_var > TUBE.DEFAULT_MAX_POINT_STABILITY_MM:
        reasons.append('top-edge point unstable across 0/1/2 px erosion')
    reasons = list(dict.fromkeys(reasons))

    cv2.imwrite(str(root / 'mask_original.png'), mask.astype(np.uint8) * 255)
    TUBE.draw_overlay(rgb, detections, iid, mask, main, K, root / 'selected_overlay.jpg')
    if 'camera_points' in main:
        TUBE.write_ply(
            root / 'selected_points.ply',
            main['camera_points'],
            main.get('fit', {}).get('keep', np.zeros(len(main['camera_points']), bool))
        )
        dispatch_3d_render(TUBE.render_3d, main, T_m, root / 'top_edge_3d.png', False)

    diagnostics = {
        'sam3_detections': len(detections),
        'selection_candidates': ranked,
        'mask_area_pixels': int(mask.sum()),
        'valid_depth_ratio': depth_ratio,
        'edge_point_count': main.get('height_band_point_count'),
        'edge_depth_sampling_semantics': (
            'full valid mask point cloud transformed to chassis_link, then upper continuous height band; '
            'not a reconstructed hidden seal centre'
        ),
        'edge_residual_median_mm': main.get('fit', {}).get('residual_median_mm'),
        'edge_residual_p90_mm': main.get('fit', {}).get('residual_p90_mm'),
        'edge_segment_length_mm': main.get('fit', {}).get('segment_length_mm'),
        'edge_free_tilt_deg': main.get('fit', {}).get('free_tilt_deg'),
        'erosion_point_max_pairwise_mm': point_var,
        'valid_depth_points': main.get('valid_depth_points'),
        'mask_valid_depth_ratio': depth_ratio,
        'height_top_mm': main.get('height_top_mm'),
        'height_band_min_mm': main.get('height_band_min_mm'),
        'height_band_max_mm': main.get('height_band_max_mm'),
        'height_band_span_mm': main.get('height_band_span_mm'),
        'height_band_max_gap_mm': main.get('height_band_max_gap_mm'),
        'height_band_projection_coverage': main.get('height_band_projection_coverage'),
    }

    common.update({
        'ok': bool(edge_ok and point_ok),
        'selected_instance_id': iid,
        'upstream_instance_id': iid,
        'filtered_instance_id': filtered_id,
        'sam3_score': selected.get('score'),
        'selection_reason': 'height-band candidate + highest original SAM3 score; no fallback after geometry failure',
        'edge_valid': edge_ok,
        'point_valid': point_ok,
        'point_semantics': main.get('point_semantics') if point_ok else None,
        'top_edge_center_camera_mm': main.get('reference_camera_mm') if point_ok else None,
        'top_edge_center_chassis_mm': main.get('reference_chassis_mm') if point_ok else None,
        'top_edge_endpoints_camera_mm': main.get('endpoints_camera_mm') if point_ok else None,
        'edge_direction_camera': main.get('direction_camera') if point_ok else None,
        'rejection_reasons': reasons,
        'diagnostics': diagnostics,
        'artifacts': artifact_payload(
            root, all_overlay, root / 'selected_overlay.jpg',
            root / 'selected_points.ply' if (root / 'selected_points.ply').exists() else None,
            {'top_edge_3d': str(root / 'top_edge_3d.png') if (ENABLE_3D_RENDER or (root / 'top_edge_3d.png').exists()) else None}
        ),
    })
    return common


def process_box(req, request_id, root, rgb, depth, K, T_m, sam, prompt, geometry_cache=None, mark=None):
    """Delegates box top planar fitting and geometric center arbitration to box_pipeline module."""
    mark = mark or (lambda name, t0: time.perf_counter())
    return BOX_PIPE.fit_box(
        req, request_id, root, rgb, depth, K, T_m, sam, prompt,
        geometry_cache=geometry_cache or GeometryCache(), estee=BOX, bgc=BGC,
        dispatch_3d_render_fn=dispatch_3d_render,
        artifact_payload_fn=artifact_payload,
        setup_category_detections_fn=setup_category_detections,
        save_all_overlay_fn=save_all_overlay,
        rank_by_score_fn=rank_by_score,
        sam_summary_fn=sam_summary,
        input_summary_fn=input_summary,
        box_surface_cfg_fn=box_surface_cfg,
        empty_fields=CATEGORY_EMPTY_FIELDS['box'],
        enable_3d_render=ENABLE_3D_RENDER
    )


def process_basket(req, request_id, root, rgb, depth, K, T_m, mark=None):
    """Server-owned Basket mask plus local FoundationPose 25550 backend."""
    mark = mark or (lambda name, t0: time.perf_counter())
    prompt = BASKET_DEFAULT_PROMPT
    threshold = BASKET_DEFAULT_THRESHOLD
    t0 = time.perf_counter()
    sam = call_sam3(rgb, prompt, threshold)
    mark('basket_sam3_call', t0)

    raw = list(sam.get('detections', []))
    t0 = time.perf_counter()
    kept, rejected = score_filter(raw, threshold)
    prepared = prepare_detections(kept, rgb.shape)
    mark('basket_mask_prepare', t0)

    h, w = rgb.shape[:2]
    cx, cy = w / 2.0, h / 2.0
    candidates = []
    for d in prepared:
        x, y, bw, bh = map(float, d.get('bbox', [0, 0, 0, 0]))
        mx, my = x + bw / 2, y + bh / 2
        candidates.append((float((mx - cx) ** 2 + (my - cy) ** 2), -float(d.get('score', -1)), int(d['upstream_instance_id']), d))
    candidates.sort(key=lambda x: (x[0], x[1], x[2]))

    t0 = time.perf_counter()
    all_overlay = save_all_overlay(rgb, prepared, root / 'all_instances_overlay.jpg')
    mark('all_instances_overlay', t0)

    diagnostics = {
        'sam3': sam_summary(sam, prompt, threshold),
        'score_rejected': rejected,
        'candidate_count': len(candidates),
        'candidate_order': [
            {'instance_id': x[2], 'score': float(x[3].get('score', -1)), 'center_distance_px': float(np.sqrt(x[0]))}
            for x in candidates
        ],
    }
    base = {
        'request_id': request_id,
        'target_type': 'basket',
        'class_name': 'Basket',
        'sku_typ': None,
        'recognition_mode': 'basket_foundationpose',
        'sam3_call_count': 1,
        'mask_prompt': prompt,
        'mask_threshold': threshold,
        'output_frame': req['camera_frame'],
        'output_unit': 'mm',
        'reference_frame': req['base_frame'],
        'cad_id': 'Basket',
        'cad_path': str(BASKET_MESH_PATH),
        'mesh_scale': BASKET_MESH_SCALE,
        'foundationpose_url': BASKET_FP_URL,
        'side_audit': req.get('_target_context'),
        'diagnostics': diagnostics,
        'rejection_reasons': [],
        'model_center_camera_mm': None,
        'object_origin_camera_mm': None,
        'model_center_offset_m': None,
        'point_semantics': None,
        'rotation_euler_zyx_rad': None,
        'xyzrxryrz_camera_mm_rad': None,
    }

    if not candidates:
        base.update({
            'ok': False,
            'pose_valid': False,
            'selected_instance_id': None,
            'sam3_score': None,
            'pose_4x4': None,
            'xyz_camera_mm': None,
            'reference_point_camera_mm': None,
            'rejection_reasons': ['no Basket mask above configured threshold'],
            'artifacts': artifact_payload(root, all_overlay),
        })
        return base

    selected = candidates[0][3]
    mask = selected['_mask']
    iid = int(selected['upstream_instance_id'])
    x, y, bw, bh = map(int, selected.get('bbox', [0, 0, 0, 0]))

    t0 = time.perf_counter()
    selected_overlay = rgb.copy()
    selected_overlay[mask] = (0.45 * selected_overlay[mask] + 0.55 * np.array([0, 220, 80])).astype(np.uint8)
    cv2.rectangle(selected_overlay, (x, y), (x + bw, y + bh), (0, 255, 0), 3)
    cv2.putText(
        selected_overlay,
        f'Basket#{iid} score={float(selected.get("score", 0)):.3f}',
        (x, max(20, y - 8)),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.65,
        (255, 255, 255),
        1,
        cv2.LINE_AA,
    )
    cv2.imwrite(str(root / 'selected_overlay.jpg'), selected_overlay)
    mark('selected_overlay_write', t0)

    if not BASKET_MESH_PATH.is_file():
        base.update({
            'ok': False,
            'pose_valid': False,
            'selected_instance_id': iid,
            'sam3_score': float(selected['score']),
            'pose_4x4': None,
            'xyz_camera_mm': None,
            'rejection_reasons': ['server Basket CAD not found'],
            'artifacts': artifact_payload(root, all_overlay, selected_overlay),
        })
        return base

    t0 = time.perf_counter()
    ok_rgb, enc_rgb = cv2.imencode('.png', rgb)
    depth_u16 = np.clip(np.rint(np.nan_to_num(depth, nan=0.0, posinf=0.0, neginf=0.0)), 0, 65535).astype(np.uint16)
    ok_depth, enc_depth = cv2.imencode('.png', depth_u16)
    ok_mask, enc_mask = cv2.imencode('.png', mask.astype(np.uint8) * 255)
    mark('fp_input_encode', t0)

    if not (ok_rgb and ok_depth and ok_mask):
        raise ValueError('failed to encode Basket FoundationPose inputs')

    camera_json = json.dumps({'cam_K': [K['fx'], 0.0, K['cx'], 0.0, K['fy'], K['cy'], 0.0, 0.0, 1.0], 'depth_scale': 0.001})
    files = {
        'rgb': ('rgb.png', enc_rgb.tobytes(), 'image/png'),
        'depth': ('depth.png', enc_depth.tobytes(), 'image/png'),
        'camera': ('camera.json', camera_json.encode(), 'application/json'),
        'mask': ('mask.png', enc_mask.tobytes(), 'image/png'),
    }
    if not BASKET_FP_REGISTERED_CAD:
        files['mesh'] = (BASKET_MESH_PATH.name, BASKET_MESH_PATH.read_bytes(), 'application/octet-stream')

    started = time.perf_counter()
    try:
        fp_resp = requests.post(BASKET_FP_URL, files=files, data={'mesh_scale': str(BASKET_MESH_SCALE)}, timeout=240)
        fp_data = fp_resp.json()
    except Exception as exc:
        fp_resp = None
        fp_data = {'ok': False, 'error': str(exc)}

    fp_ms = round((time.perf_counter() - started) * 1000, 3)
    mark('foundationpose_call', started)
    t0 = time.perf_counter()
    save_json(root / 'foundationpose_response.json', fp_data)
    mark('fp_response_save', t0)

    fp_visual_extra = {}
    t0 = time.perf_counter()
    for source_key, target_name in (
        ('vis_pose_path', 'foundationpose_pose_overlay.png'),
        ('detection_pose_path', 'foundationpose_detection_pose.png'),
        ('vis_ism_path', 'foundationpose_instance_segmentation.png'),
    ):
        source = fp_data.get(source_key)
        if isinstance(source, str) and Path(source).is_file():
            target = root / target_name
            shutil.copyfile(source, target)
            fp_visual_extra[source_key] = str(target)
            if is_embed_visualizations():
                fp_visual_extra[source_key + '_base64'] = file_b64(target)
    mark('fp_visual_copy', t0)

    pose_m = np.asarray(fp_data.get('pose_4x4'), dtype=float) if fp_data.get('pose_4x4') is not None else None
    valid = bool(
        fp_resp is not None
        and fp_resp.status_code == 200
        and pose_m is not None
        and pose_m.shape == (4, 4)
        and np.isfinite(pose_m).all()
        and np.allclose(pose_m[3], [0, 0, 0, 1], atol=1e-5)
    )
    pose_mm = pose_m.copy() if valid else None
    if pose_mm is not None:
        pose_mm[:3, 3] *= 1000.0

    fp_det = (fp_data.get('detections') or [{}])[0] if isinstance(fp_data, dict) else {}
    center_off = fp_det.get('mesh_center_offset_m')
    origin_mm = pose_mm[:3, 3] if valid else None
    if valid and center_off is not None:
        center_mm = (pose_m[:3, :3] @ np.asarray(center_off, dtype=float)) * 1000.0 + np.asarray(origin_mm, dtype=float)
    else:
        center_mm = origin_mm

    euler = fp_det.get('rotation_euler_zyx_rad')
    ch = (T_m[:3, :3] @ center_mm + T_m[:3, 3] * 1000.0) if valid else None

    base.update({
        'ok': valid,
        'pose_valid': valid,
        'selected_instance_id': iid,
        'sam3_score': float(selected['score']),
        'pose_4x4': pose_mm if valid else None,
        'pose_4x4_input_m': pose_m if valid else None,
        'xyz_camera_mm': center_mm,
        'reference_point_camera_mm': center_mm,
        'reference_point_chassis_mm': ch,
        'model_center_camera_mm': center_mm,
        'object_origin_camera_mm': origin_mm,
        'model_center_offset_m': center_off,
        'point_semantics': 'basket_model_center' if valid else None,
        'rotation_euler_zyx_rad': euler,
        'xyzrxryrz_camera_mm_rad': (list(center_mm) + list(euler)) if (valid and center_mm is not None and euler is not None) else None,
        'foundationpose_http_status': int(fp_resp.status_code) if fp_resp is not None else None,
        'foundationpose_wall_ms': fp_ms,
        'diagnostics': {
            **diagnostics,
            'selection_reason': 'nearest image-center Basket candidate; tie-break score then instance id',
            'selected_bbox_xyxy': [x, y, x + bw, y + bh],
            'foundationpose_response_ok': valid,
            'foundationpose_cad_transport': 'registered_default' if BASKET_FP_REGISTERED_CAD else 'per_request_upload',
        },
        'rejection_reasons': [] if valid else ['FoundationPose Basket pose quality gate failed'],
        'artifacts': artifact_payload(
            root, all_overlay, selected_overlay,
            extra={'foundationpose_response': str(root / 'foundationpose_response.json'), **fp_visual_extra}
        ),
    })
    return base


def process_sku(req, request_id, root, rgb=None, depth=None, K=None, T_m=None, class_name=None, mark=None):
    """Executes the standard SKU localization pipeline (box -> ROI -> product SAM3 -> front row -> category fit)."""
    mark = mark or (lambda name, t0: time.perf_counter())
    if class_name is None:
        class_name = req.get('sku_typ') or normalize_request_target(req)

    if not req.get('front_rule'):
        req['front_rule'] = dict(FRONT_RULE_DEFAULT)

    class_cfg = CLASS_CONFIG[class_name]
    if rgb is None or depth is None or K is None or T_m is None:
        t0 = time.perf_counter()
        rgb, depth, K, T_m = common_input(req)
        mark('rgbd_decode', t0)
    geometry_cache = GeometryCache()

    prompt = req.get('sam3_prompt')
    box_cfg = parse_box_selection(req, class_cfg)
    if not isinstance(prompt, str) or not prompt.strip():
        prompt = class_cfg['sam3_prompt']
    prompt = prompt.strip()

    t0 = time.perf_counter()
    box_sam = call_sam3(rgb, box_cfg['box_prompt'], box_cfg['box_threshold'], SAM3_URL)
    mark('box_sam3_call', t0)

    box_raw = list(box_sam.get('detections', []))
    box_candidates, box_score_rejected = score_filter(box_raw, box_cfg['box_threshold'])
    box_audit = dict(box_cfg)
    box_audit.update({
        'valid': False,
        'stage_status': 'box_selection_failed',
        'reason': None,
        'selection_policy': 'largest_separable_horizontal_pair; image-position left/right only; not persistent identity or robot left/right',
        'roi_type': 'box_bbox',
        'box_candidates': [
            {'upstream_instance_id': d['upstream_instance_id'], 'score': float(d.get('score', -1)), 'bbox': d.get('bbox')}
            for d in box_candidates
        ],
        'box_score_rejected': box_score_rejected,
        'box_stage': sam_summary(box_sam, box_cfg['box_prompt'], box_cfg['box_threshold']),
        'target_prompt': prompt,
        'threshold_note': 'configured upstream applies threshold; service also audits returned scores but cannot recover proposals removed upstream',
    })

    rois = None
    indices = None
    t0 = time.perf_counter()
    try:
        indices, rois = BOXSEL.choose_pair(box_candidates, rgb.shape)
    except ValueError as exc:
        box_audit['reason'] = str(exc)
    mark('box_pair_selection', t0)

    box_vis = box_overlay(rgb, box_candidates, rois, indices, box_cfg['target_box'], root / 'box_selection_overlay.jpg')
    if rois is None:
        result = {
            'ok': False,
            'request_id': request_id,
            'target_type': 'sku',
            'sku_typ': class_name,
            'class_name': class_name,
            'side_audit': req.get('_target_context'),
            'selected_instance_id': None,
            'sam3_score': None,
            'sam3_call_count': 1,
            'box_selection': box_audit,
            'output_frame': req['camera_frame'],
            'output_unit': 'mm',
            'rejection_reasons': ['box selection failed: ' + str(box_audit['reason'])],
            'input_summary': input_summary(req, rgb, depth, K),
            'pipeline_steps': ['rgbd_decode', 'box_sam3', 'box_pair_selection', 'rejected'],
            'artifacts': {},
        }
        result.update(CATEGORY_EMPTY_FIELDS.get(class_name, {}))
        result.update({
            'front_panel_valid': False,
            'front_panel_top_edge_midpoint_camera_mm': None,
            'front_panel_plane_point_camera_mm': None,
            'front_panel_plane_normal_camera': None,
        })
        return attach_two_stage_artifacts(result, root, {'box_selection_overlay': (root / 'box_selection_overlay.jpg', box_vis)})

    if len(rois) == 1:
        side = 0
        target_semantics = 'single_box_adaptive: only 1 cardboard box detected in workspace'
    else:
        side = min(max(0, box_cfg['target_box'] - 1), len(rois) - 1)
        target_semantics = '1=image-left, 2=image-right within current selected pair'

    roi = rois[side]
    selected_box = box_candidates[indices[side]]
    box_audit.update({
        'valid': True,
        'stage_status': 'box_selected',
        'reason': 'ok',
        'selected_pair_upstream_ids': [box_candidates[i]['upstream_instance_id'] for i in indices],
        'left_right_rois_xyxy': rois,
        'selected_box_upstream_id': selected_box['upstream_instance_id'],
        'container_roi_xyxy': roi,
        'target_box_semantics': target_semantics,
    })

    t0 = time.perf_counter()
    front_panel, front_panel_overlay = fit_selected_box_front_panel(
        selected_box, rgb, depth, K, T_m, req, geometry_cache, root / 'front_panel_overlay.jpg'
    )
    mark('front_panel_fit', t0)
    box_audit['front_panel'] = front_panel

    stage_rgb = rgb
    t0 = time.perf_counter()
    cv2.imwrite(str(root / 'second_stage_input.jpg'), stage_rgb)
    mark('second_stage_input_write', t0)

    t0 = time.perf_counter()
    target_sam = call_sam3(stage_rgb, prompt, box_cfg['target_threshold'], SAM3_URL)
    mark('target_sam3_call', t0)

    target_stage_summary = sam_summary(target_sam, prompt, box_cfg['target_threshold'])
    raw_items = list(target_sam.get('detections', []))
    for i, d in enumerate(raw_items, 1):
        d['upstream_instance_id'] = i
        d['crop_boundary_touched'] = False

    t0 = time.perf_counter()
    raw_prepared = prepare_detections(raw_items, rgb.shape)
    raw_overlay = save_all_overlay(rgb, raw_prepared, root / 'raw_object_instances_overlay.jpg')
    mark('raw_mask_decode_overlay', t0)

    t0 = time.perf_counter()
    threshold_kept, target_score_rejected = score_filter(raw_items, box_cfg['target_threshold'])
    mark('target_score_filter', t0)

    t0 = time.perf_counter()
    kept, rejected, membership = BOXSEL.filter_instances_detailed(
        threshold_kept, roi, rgb.shape, decode_detection_mask, min_inside=box_cfg['min_inside_ratio'], reject_crop_boundary=True
    )
    mark('box_roi_filter', t0)

    t0 = time.perf_counter()
    kept, front_audit = apply_front_rule(kept, rgb.shape, depth, K, T_m, req.get('front_rule'), decode_detection_mask, geometry_cache)
    mark('front_row_filter', t0)

    t0 = time.perf_counter()
    filtered_prepared = prepare_detections(kept, rgb.shape)
    filtered_overlay = save_all_overlay(rgb, filtered_prepared, root / 'filtered_instances_overlay.jpg')
    mark('filtered_mask_decode_overlay', t0)

    target_sam = dict(target_sam)
    target_sam['detections'] = kept
    target_sam['num_detections'] = len(kept)
    target_sam['_requested_threshold'] = box_cfg['target_threshold']

    box_audit.update({
        'stage_status': 'target_instances_filtered' if kept else 'no_target_in_selected_box',
        'raw_object_count': len(raw_items),
        'score_qualified_object_count': len(threshold_kept),
        'inside_object_count': len(kept),
        'target_score_rejected': target_score_rejected,
        'object_membership': membership,
        'rejected_objects': rejected,
        'front_row_filter': front_audit,
        'target_stage': target_stage_summary,
        'crop_region': None,
    })

    geometry_started = time.perf_counter()
    if class_name == 'bottle':
        result = process_bottle(req, request_id, root, rgb, depth, K, T_m, target_sam, prompt, geometry_cache, mark=mark)
    elif class_name == 'box':
        result = process_box(req, request_id, root, rgb, depth, K, T_m, target_sam, prompt, geometry_cache, mark=mark)
    else:
        result = process_tube(req, request_id, root, rgb, depth, K, T_m, target_sam, prompt, geometry_cache, mark=mark)

    result.setdefault('diagnostics', {}).update({
        'geometry_wall_ms': round((time.perf_counter() - geometry_started) * 1000, 3),
        'geometry_preprocess': geometry_cache.stats,
    })

    fp_valid = bool(front_panel.get('valid'))
    result.update({
        'front_panel_valid': fp_valid,
        'front_panel_top_edge_midpoint_camera_mm': front_panel.get('top_edge_midpoint_camera_mm') if fp_valid else None,
        'front_panel_top_edge_midpoint_chassis_mm': front_panel.get('top_edge_midpoint_chassis_mm') if fp_valid else None,
        'front_panel_plane_point_camera_mm': front_panel.get('plane_point_camera_mm') if fp_valid else None,
        'front_panel_plane_point_chassis_mm': front_panel.get('plane_point_chassis_mm') if fp_valid else None,
        'front_panel_plane_normal_camera': front_panel.get('plane_normal_camera') if fp_valid else None,
        'front_panel_plane_normal_chassis': front_panel.get('plane_normal_chassis') if fp_valid else None,
    })

    result['pipeline_steps'] = [
        'rgbd_decode', 'box_sam3', 'box_pair_selection', 'shared_camera_to_chassis_geometry',
        'selected_box_front_panel_fit', 'target_sam3', 'box_roi_filter', 'front_row_filter',
        'category_geometry_fit', 'quality_gate'
    ]
    result.update({
        'sam3_call_count': 2,
        'box_selection': box_audit,
        'target_type': 'sku',
        'sku_typ': class_name,
        'side_audit': req.get('_target_context'),
    })

    if not kept:
        result['ok'] = False
        result['selected_instance_id'] = None
        result['rejection_reasons'] = list(dict.fromkeys(
            ['no target instance assigned to selected box'] + result.get('rejection_reasons', [])
        ))

    images = {
        'box_selection_overlay': (root / 'box_selection_overlay.jpg', box_vis),
        'second_stage_input': (root / 'second_stage_input.jpg', stage_rgb),
        'raw_object_instances_overlay': (root / 'raw_object_instances_overlay.jpg', raw_overlay),
        'filtered_instances_overlay': (root / 'filtered_instances_overlay.jpg', filtered_overlay),
    }
    if front_panel_overlay is not None:
        images['front_panel_overlay'] = (root / 'front_panel_overlay.jpg', front_panel_overlay)

    return attach_two_stage_artifacts(result, root, images)


def execute_pipeline(req: Dict[str, Any], request_id: str, root: Path, mark=None) -> Dict[str, Any]:
    """Top-level pipeline execution entry point."""
    mark = mark or (lambda name, t0: time.perf_counter())
    set_embed_visualizations(bool(req.get('return_visualizations', False)))
    class_name = normalize_request_target(req)

    t0 = time.perf_counter()
    rgb, depth, K, T_m = common_input(req)
    mark('rgbd_decode', t0)

    if req.get('target_type') == 'basket':
        return process_basket(req, request_id, root, rgb, depth, K, T_m, mark=mark)
    return process_sku(req, request_id, root, rgb, depth, K, T_m, class_name=class_name, mark=mark)
