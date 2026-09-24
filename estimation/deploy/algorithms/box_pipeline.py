#!/usr/bin/env python3
"""Box SKU geometry pipeline: multi-plane fitting, 3D OBB geometric center gating, interior fallback, and diagnostics."""
from pathlib import Path
import cv2
import numpy as np

try:
    import fit_estee_box_top_surface as BOX
    import box_geometric_center as BGC
except ImportError:
    BOX = None
    BGC = None

def analyze_box_planes(mask, depth, K, T_m, mode, cfg, geometry_cache, estee=None):
    """Compute 0/1/2 erosion plane fits lazily without computing interior points."""
    e_mod = estee or BOX
    return {str(e): e_mod.analyse(mask, depth, K, T_m, e, mode, cfg, geometry_cache=geometry_cache, compute_interior=False) for e in (0, 1, 2)}

def select_geometric_point(analyses, mask, K, T_m, erosion=1, cfg=None, bgc=None):
    b_mod = bgc or BGC
    cfg = cfg or {}
    key = str(erosion) if str(erosion) in analyses else ('1' if '1' in analyses else list(analyses.keys())[0])
    main = analyses[key]
    plane_ok = bool(main.get('fit', {}).get('valid'))
    max_support_dist = float(cfg.get('max_support_distance_px', b_mod.DEFAULT_MAX_SUPPORT_DISTANCE_PX))
    bgc_analysis = b_mod.analyze_existing_analyses(
        analyses, mask, K, T_m,
        selected_erosion=int(key),
        max_support_distance_px=max_support_dist, cfg=cfg
    )
    cand = bgc_analysis.get('candidate', bgc_analysis.get('candidate_geometric_center', {}))
    geo_usable = bool(plane_ok and cand.get('valid') and cand.get('geometric_center_supported') and cand.get('camera_mm') is not None)
    fb_reasons = []
    if not geo_usable:
        fb_reasons = list(cand.get('rejection_reasons', []))
        if not fb_reasons:
            if not plane_ok: fb_reasons = [main.get('reason', 'top_plane_invalid')]
            elif not cand.get('geometric_center_supported'):
                fb_reasons = [f"geometric_center_unsupported_distance_{cand.get('support_distance_px')}px>{cand.get('support_threshold_px', max_support_dist)}px"]
            elif not cand.get('valid'): fb_reasons = ["geometric_center_invalid"]
            else: fb_reasons = ["geometric_center_unavailable"]
    return geo_usable, cand, bgc_analysis, fb_reasons

def compute_interior_fallback(analyses, mask, K, T_m, cfg, erosion=1, estee=None):
    e_mod = estee or BOX
    key = str(erosion) if str(erosion) in analyses else ('1' if '1' in analyses else list(analyses.keys())[0])
    for a in analyses.values():
        e_mod.attach_interior_point(a, mask, K, T_m, cfg)
    main = analyses[key]
    plane_ok = bool(main.get('fit', {}).get('valid'))
    heights = [a['fit']['plane_point_chassis_mm'][2] for a in analyses.values() if a.get('fit', {}).get('valid')]
    points = [a['reference_camera_mm'] for a in analyses.values() if a.get('valid') and a.get('reference_camera_mm') is not None]
    height_var = float(max(heights) - min(heights)) if len(heights) >= 2 else None
    point_var = float(max(np.linalg.norm(np.asarray(a) - np.asarray(b)) for a in points for b in points)) if len(points) >= 2 else None

    interior_ok = bool(
        plane_ok and main.get('valid') and
        height_var is not None and point_var is not None and
        height_var <= e_mod.DEFAULT_MAX_HEIGHT_STABILITY_MM and
        point_var <= e_mod.DEFAULT_MAX_POINT_STABILITY_MM and
        main.get('valid_depth_ratio', 0) >= e_mod.DEFAULT_MIN_VALID_DEPTH_RATIO
    )
    fail_reasons = []
    if not plane_ok: fail_reasons.append(main.get('reason', 'top plane invalid'))
    if main.get('valid_depth_ratio', 0) < e_mod.DEFAULT_MIN_VALID_DEPTH_RATIO: fail_reasons.append('valid depth ratio below threshold')
    if height_var is None or point_var is None:
        fail_reasons.append('insufficient successful erosion variants for stability check')
    else:
        if height_var > e_mod.DEFAULT_MAX_HEIGHT_STABILITY_MM: fail_reasons.append('top height unstable across 0/1/2 px erosion')
        if point_var > e_mod.DEFAULT_MAX_POINT_STABILITY_MM: fail_reasons.append('top point unstable across 0/1/2 px erosion')
    if not main.get('valid') and plane_ok: fail_reasons.append(main.get('reason', 'top point unsupported'))

    int_dict = {
        'camera_mm': np.asarray(main['reference_camera_mm']).tolist() if interior_ok and main.get('reference_camera_mm') is not None else None,
        'chassis_mm': np.asarray(main['reference_chassis_mm']).tolist() if interior_ok and main.get('reference_chassis_mm') is not None else None,
        'pixel': np.asarray(main['reference_pixel']).tolist() if interior_ok and main.get('reference_pixel') is not None else None,
        'height_var_mm': height_var, 'point_var_mm': point_var
    }
    return bool(interior_ok and int_dict['camera_mm'] is not None), int_dict, fail_reasons

def summarize_box_diagnostics(detections, ranked, mask, main, bgc_analysis, plane_distance, height_var=None, point_var=None):
    raw_candidates = main.get('fit', {}).get('candidates', [])
    plane_candidates = [{k: v for k, v in c.items() if not k.startswith('_')} for c in raw_candidates if isinstance(c, dict)]
    return {
        'sam3_detections': len(detections), 'selection_candidates': ranked, 'mask_area_pixels': int(mask.sum()),
        'valid_depth_ratio': main.get('valid_depth_ratio'),
        'top_inlier_count': int(main.get('fit', {}).get('keep', np.array([], bool)).sum()),
        'top_residual_median_mm': main.get('fit', {}).get('residual_median_mm'),
        'top_residual_p90_mm': main.get('fit', {}).get('residual_p90_mm'),
        'normal_tilt_deg': main.get('fit', {}).get('normal_tilt_deg'),
        'top_support': main.get('fit', {}).get('metrics'),
        'reference_interior_margin_px': main.get('interior_margin_px'),
        'reference_plane_distance_mm': plane_distance,
        'erosion_height_range_mm': height_var, 'erosion_point_max_pairwise_mm': point_var,
        'plane_candidates': plane_candidates, 'box_geometric_analysis': bgc_analysis
    }

def fit_box(req, request_id, root, rgb, depth, K, T_m, sam, prompt,
            geometry_cache=None, estee=None, bgc=None,
            dispatch_3d_render_fn=None, artifact_payload_fn=None,
            setup_category_detections_fn=None, save_all_overlay_fn=None,
            rank_by_score_fn=None, sam_summary_fn=None, input_summary_fn=None,
            box_surface_cfg_fn=None, empty_fields=None, enable_3d_render=True):
    e_mod = estee or BOX
    b_mod = bgc or BGC
    detections = setup_category_detections_fn(sam.get('detections', []), 'box')
    all_overlay = save_all_overlay_fn(rgb, detections, root / 'all_instances_overlay.jpg')
    ranked = rank_by_score_fn(detections)
    mode = req.get('plane_mode', 'horizontal')
    if mode not in ('horizontal', 'constrained_tilt'):
        raise ValueError('box plane_mode must be horizontal or constrained_tilt')

    common = {
        'request_id': request_id,
        'sku_typ': 'box',
        'class_name': 'box',
        'localization_method': 'box_top_surface',
        'sam3_prompt': prompt,
        'sam3_response_summary': sam_summary_fn(sam, prompt, sam.get('_requested_threshold')),
        'input_summary': input_summary_fn(req, rgb, depth, K),
        'plane_mode': mode,
        'output_frame': req['camera_frame'],
        'output_unit': 'mm'
    }

    if not ranked:
        common.update({'ok': False, 'selected_instance_id': None, **(empty_fields or {}),
                       'rejection_reasons': ['no box SAM3 candidate'],
                       'diagnostics': {'selection_candidates': []},
                       'artifacts': artifact_payload_fn(root, all_overlay)})
        return common

    iid = ranked[0][0]
    filtered_id = ranked[0][2]
    selected = detections[filtered_id - 1]
    mask = selected['_mask']
    erosion = int(req.get('mask_erosion_pixels', e_mod.DEFAULT_MASK_EROSION_PIXELS))
    if erosion not in (0, 1, 2):
        erosion = 1
    cfg = box_surface_cfg_fn(req)

    # Step 1: 0/1/2 erosion plane fits (compute_interior=False, lazy!)
    analyses = analyze_box_planes(mask, depth, K, T_m, mode, cfg, geometry_cache, estee=e_mod)
    selected_key = str(erosion)
    main = analyses[selected_key]
    plane_ok = bool(main.get('fit', {}).get('valid'))

    # Step 2: Compute and gate 3D geometric center
    geo_usable, cand, bgc_analysis, fb_reasons = select_geometric_point(analyses, mask, K, T_m, erosion=erosion, cfg=cfg, bgc=b_mod)

    height_var = None
    point_var = None
    reasons = []

    if geo_usable:
        top_cam = cand.get('camera_mm')
        top_ch = cand.get('chassis_mm')
        top_uv = cand.get('pixel')
        point_semantics = 'top_surface_geometric_center'
        point_source = 'geometric'
        fallback_used = False
        fallback_reason = None
        point_valid = True
    else:
        fallback_used = True
        fallback_reason = fb_reasons
        int_usable, int_dict, int_fail_reasons = compute_interior_fallback(analyses, mask, K, T_m, cfg, erosion=erosion, estee=e_mod)
        height_var = int_dict.get('height_var_mm')
        point_var = int_dict.get('point_var_mm')

        if int_usable:
            top_cam = int_dict.get('camera_mm')
            top_ch = int_dict.get('chassis_mm')
            top_uv = int_dict.get('pixel')
            point_semantics = 'visible_top_interior_point'
            point_source = 'interior_fallback'
            point_valid = True
        else:
            top_cam = top_ch = top_uv = point_semantics = point_source = None
            point_valid = False
            reasons = list(dict.fromkeys(fb_reasons + int_fail_reasons))

    overall_ok = bool(plane_ok and point_valid)
    reasons = list(dict.fromkeys(reasons))

    # Save debug mask images
    cv2.imwrite(str(root / 'mask_original.png'), mask.astype(np.uint8) * 255)
    top_mask = main.get('fit', {}).get('top_mask', np.zeros_like(mask))
    cv2.imwrite(str(root / 'mask_top_surface.png'), top_mask.astype(np.uint8) * 255)

    # Draw overlays
    main_for_draw = {**main, 'reference_pixel': top_uv if point_valid else None,
                     'point_semantics': point_semantics if point_valid else None,
                     'valid': bool(point_valid and top_uv is not None)}
    e_mod.draw_overlay(rgb, detections, iid, mask, main_for_draw, K, root / 'selected_overlay.jpg')

    geo_overlay_path = root / 'geometric_center_overlay.jpg'
    b_mod.draw_candidate_overlay(rgb, mask, bgc_analysis, K, str(geo_overlay_path), top_mask=top_mask)

    ply = None
    if 'camera_points' in main:
        ply = root / 'selected_points.ply'
        e_mod.write_ply(ply, main['camera_points'], main.get('fit', {}).get('keep', np.zeros(len(main['camera_points']), bool)))
        if dispatch_3d_render_fn is not None:
            dispatch_3d_render_fn(e_mod.render_3d, main, T_m, root / 'top_surface_3d.png', False)

    plane_ch = main.get('fit', {}).get('plane_point_chassis_mm')
    normal_ch = main.get('fit', {}).get('normal_chassis')
    plane_cam = e_mod.chassis_to_camera(np.asarray(plane_ch)[None], T_m)[0] if plane_ch is not None else None
    normal_cam = T_m[:3, :3].T @ normal_ch if normal_ch is not None else None

    plane_distance = float(abs(np.dot(np.asarray(top_cam) - plane_cam, normal_cam))) if top_cam is not None and plane_cam is not None and normal_cam is not None else None

    diagnostics = summarize_box_diagnostics(
        detections, ranked, mask, main, bgc_analysis, plane_distance,
        height_var=height_var, point_var=point_var
    )

    common.update({
        'ok': overall_ok,
        'selected_instance_id': iid,
        'upstream_instance_id': iid,
        'filtered_instance_id': filtered_id,
        'sam3_score': selected.get('score'),
        'selection_reason': 'highest original SAM3 score; ties use lower upstream instance id',
        'top_plane_valid': plane_ok,
        'top_point_valid': point_valid,
        'point_semantics': point_semantics,
        'point_source': point_source,
        'fallback_used': fallback_used,
        'fallback_reason': fallback_reason,
        'top_point_camera_mm': top_cam,
        'top_point_chassis_mm': top_ch,
        'top_point_uv': top_uv,
        'top_plane_point_camera_mm': plane_cam,
        'top_plane_point_chassis_mm': plane_ch,
        'plane_normal_camera': normal_cam,
        'plane_normal_chassis': normal_ch,
        'rejection_reasons': reasons,
        'diagnostics': diagnostics,
        'artifacts': artifact_payload_fn(root, all_overlay, root / 'selected_overlay.jpg', ply, {
            'top_surface_3d': str(root / 'top_surface_3d.png') if (enable_3d_render or (root / 'top_surface_3d.png').exists()) else None,
            'geometric_center_overlay': str(geo_overlay_path)
        })
    })
    return common
