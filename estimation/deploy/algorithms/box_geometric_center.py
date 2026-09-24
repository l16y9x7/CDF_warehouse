"""Candidate 3D top-surface geometric center calculation and validation for box SKU.

Computes the metric-plane oriented bounding rectangle center of top-plane 3D inliers,
evaluates 4-edge observation support, checks top_mask support region proximity,
and verifies erosion stability across 0/1/2 px.
"""
from typing import Dict, Any, Tuple, Optional, List
import numpy as np
import cv2

DEFAULT_MAX_SUPPORT_DISTANCE_PX = 6.0
DEFAULT_MAX_POINT_STABILITY_MM = 5.0

def tangent_basis(normal: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
    """Return orthonormal tangent basis vectors e1, e2 for normal."""
    n = np.asarray(normal, float)
    norm = np.linalg.norm(n)
    if norm < 1e-6:
        n = np.array([0.0, 0.0, 1.0])
    else:
        n = n / norm
    ref = np.array([1.0, 0.0, 0.0]) if abs(n[0]) < 0.8 else np.array([0.0, 1.0, 0.0])
    e1 = np.cross(n, ref)
    e1 = e1 / np.linalg.norm(e1)
    e2 = np.cross(n, e1)
    e2 = e2 / np.linalg.norm(e2)
    return e1, e2

def fit_rectangle_uv(
    uv_points: np.ndarray,
    min_length_mm: float = 38.0,
    min_width_mm: float = 28.0,
    min_area_mm2: float = 1400.0,
    max_length_mm: float = 180.0,
    max_width_mm: float = 180.0,
    max_area_mm2: float = 20000.0,
    min_edge_coverage: float = 0.30,
    min_edge_points: int = 8,
    edge_margin_ratio: float = 0.12
) -> Dict[str, Any]:
    """
    Fit oriented bounding rectangle to metric UV points and evaluate 4-edge support.
    
    uv_points: (N, 2) array of coordinates in mm on the tangent plane.
    """
    if len(uv_points) < 20:
        return {
            'valid': False,
            'reasons': ['insufficient_uv_points'],
            'center_uv': None,
            'corners_uv': None,
            'length_mm': 0.0,
            'width_mm': 0.0,
            'area_mm2': 0.0,
            'angle_deg': 0.0,
            'edge_supports': {}
        }

    # Filter extreme 1.5% percentile outliers on UV to guard against flying points
    u_lo, u_hi = np.percentile(uv_points[:, 0], [1.5, 98.5])
    v_lo, v_hi = np.percentile(uv_points[:, 1], [1.5, 98.5])
    in_box = (uv_points[:, 0] >= u_lo) & (uv_points[:, 0] <= u_hi) & \
             (uv_points[:, 1] >= v_lo) & (uv_points[:, 1] <= v_hi)
    clean_uv = uv_points[in_box]
    outlier_mask = ~in_box
    if len(clean_uv) < 20:
        clean_uv = uv_points
        outlier_mask = np.zeros(len(uv_points), bool)

    hull = cv2.convexHull(clean_uv.astype(np.float32))
    rect = cv2.minAreaRect(hull)
    (center_u, center_v), (rect_w, rect_h), angle_deg = rect
    
    # Ensure consistent w >= h
    if rect_w < rect_h:
        rect_w, rect_h = rect_h, rect_w
        angle_deg = (angle_deg + 90.0) % 180.0
    
    corners = cv2.boxPoints(rect)  # (4, 2)
    
    # Transform points to rectangle local coordinates: x in [-rect_w/2, rect_w/2], y in [-rect_h/2, rect_h/2]
    rad = np.radians(angle_deg)
    cos_a, sin_a = np.cos(rad), np.sin(rad)
    
    # Vector from center to points
    du = clean_uv[:, 0] - center_u
    dv = clean_uv[:, 1] - center_v
    
    # Local coordinates
    lx = du * cos_a + dv * sin_a
    ly = -du * sin_a + dv * cos_a
    
    # Check if orientation aligned with rect_w vs rect_h
    span_x = lx.max() - lx.min()
    span_y = ly.max() - ly.min()
    if abs(span_x - rect_h) < abs(span_x - rect_w):
        lx, ly = ly, -lx
        rect_w, rect_h = rect_h, rect_w

    hw, hh = rect_w / 2.0, rect_h / 2.0
    area = rect_w * rect_h
    
    # Check for multi-component / internal large gap along major axis (x) or minor axis (y)
    sx = np.sort(lx)
    sy = np.sort(ly)
    max_gap_x = float(np.max(np.diff(sx))) if len(sx) > 1 else 0.0
    max_gap_y = float(np.max(np.diff(sy))) if len(sy) > 1 else 0.0
    
    reasons = []
    if max_gap_x > 18.0 and rect_w > 30.0:
        reasons.append(f'large_gap_along_major_axis_{max_gap_x:.1f}mm>18mm')
    if max_gap_y > 18.0 and rect_h > 30.0:
        reasons.append(f'large_gap_along_minor_axis_{max_gap_y:.1f}mm>18mm')

    # Dimensions Quality Gate
    if rect_w < min_length_mm or rect_h < min_width_mm:
        reasons.append(f'partial_box_dimension_too_small_{rect_w:.1f}x{rect_h:.1f}<{min_length_mm}x{min_width_mm}')
    if area < min_area_mm2:
        reasons.append(f'partial_box_area_too_small_{area:.1f}<{min_area_mm2}')
    if rect_w > max_length_mm or rect_h > max_width_mm or area > max_area_mm2:
        reasons.append(f'box_dimension_too_large_{rect_w:.1f}x{rect_h:.1f}')
    if rect_h / rect_w < 0.20:
        reasons.append(f'rectangle_aspect_ratio_too_elongated_{rect_h/rect_w:.2f}')

    # Segmented Edge Support Analysis (10 bins along each edge)
    margin_w = max(2.0, min(6.0, rect_w * edge_margin_ratio))
    margin_h = max(2.0, min(6.0, rect_h * edge_margin_ratio))

    def eval_edge_bins(dist_mask, pos_along_edge, edge_len):
        pts_pos = pos_along_edge[dist_mask]
        if len(pts_pos) < 2:
            return 0.0, len(pts_pos), float(edge_len)
        bins = np.linspace(-edge_len / 2.0, edge_len / 2.0, 11)
        hist, _ = np.histogram(pts_pos, bins)
        coverage = float(np.count_nonzero(hist > 0)) / 10.0
        sorted_pos = np.sort(pts_pos)
        max_gap = float(np.max(np.diff(sorted_pos))) if len(sorted_pos) > 1 else edge_len
        return coverage, len(pts_pos), max_gap

    cov_px, cnt_px, gap_px = eval_edge_bins(lx >= hw - margin_w, ly, rect_h)
    cov_nx, cnt_nx, gap_nx = eval_edge_bins(lx <= -hw + margin_w, ly, rect_h)
    cov_py, cnt_py, gap_py = eval_edge_bins(ly >= hh - margin_h, lx, rect_w)
    cov_ny, cnt_ny, gap_ny = eval_edge_bins(ly <= -hh + margin_h, lx, rect_w)

    edge_supports = {
        'right_coverage': cov_px, 'left_coverage': cov_nx,
        'top_coverage': cov_py, 'bottom_coverage': cov_ny,
        'right_count': cnt_px, 'left_count': cnt_nx,
        'top_count': cnt_py, 'bottom_count': cnt_ny,
        'right_max_gap_mm': gap_px, 'left_max_gap_mm': gap_nx,
        'top_max_gap_mm': gap_py, 'bottom_max_gap_mm': gap_ny
    }

    min_cov = min(cov_px, cov_nx, cov_py, cov_ny)
    min_cnt = min(cnt_px, cnt_nx, cnt_py, cnt_ny)
    max_gap = max(gap_px, gap_nx, gap_py, gap_ny)

    if min_cov < min_edge_coverage:
        missing = []
        if cov_px < min_edge_coverage: missing.append(f'+X({cov_px:.2f})')
        if cov_nx < min_edge_coverage: missing.append(f'-X({cov_nx:.2f})')
        if cov_py < min_edge_coverage: missing.append(f'+Y({cov_py:.2f})')
        if cov_ny < min_edge_coverage: missing.append(f'-Y({cov_ny:.2f})')
        reasons.append('missing_edge_observation_support:' + ','.join(missing))
    if min_cnt < min_edge_points:
        reasons.append(f'sparse_edge_points_min_{min_cnt}<{min_edge_points}')
    if max_gap > 0.55 * max(rect_w, rect_h):
        reasons.append(f'large_edge_occlusion_gap_{max_gap:.1f}mm')

    valid = len(reasons) == 0
    return {
        'valid': valid,
        'reasons': reasons,
        'center_uv': [float(center_u), float(center_v)],
        'corners_uv': corners.astype(float).tolist(),
        'length_mm': float(rect_w),
        'width_mm': float(rect_h),
        'area_mm2': float(area),
        'angle_deg': float(angle_deg),
        'edge_supports': edge_supports,
        'clean_point_count': int(len(clean_uv))
    }

def compute_box_geometric_center(
    chassis_points: np.ndarray,
    top_keep: np.ndarray,
    plane_point_chassis: np.ndarray,
    normal_chassis: np.ndarray,
    T_chassis_camera: np.ndarray,
    K: Dict[str, float],
    image_shape: Tuple[int, int],
    top_mask: Optional[np.ndarray] = None,
    max_support_distance_px: float = DEFAULT_MAX_SUPPORT_DISTANCE_PX,
    cfg: Optional[Dict[str, Any]] = None
) -> Dict[str, Any]:
    """
    Compute 3D geometric center for box top surface from chassis inliers, plane fit,
    and verify top_mask support region proximity.
    """
    cfg = cfg or {}
    pts = chassis_points[top_keep]
    p0 = np.asarray(plane_point_chassis, float)
    n0 = np.asarray(normal_chassis, float)
    n0 = n0 / np.linalg.norm(n0)
    
    e1, e2 = tangent_basis(n0)
    q = pts - p0
    uv = np.column_stack((q @ e1, q @ e2))
    
    rect_res = fit_rectangle_uv(
        uv,
        min_length_mm=float(cfg.get('min_box_length_mm', 38.0)),
        min_width_mm=float(cfg.get('min_box_width_mm', 28.0)),
        min_area_mm2=float(cfg.get('min_box_area_mm2', 1400.0)),
        min_edge_coverage=float(cfg.get('min_edge_coverage', 0.30)),
        min_edge_points=int(cfg.get('min_edge_points', 8))
    )
    if not rect_res['valid']:
        return {
            'valid': False,
            'reasons': rect_res['reasons'],
            'center_chassis_mm': None,
            'center_camera_mm': None,
            'center_pixel': None,
            'corners_chassis_mm': None,
            'corners_camera_mm': None,
            'corners_pixel': None,
            'fitted_rectangle': rect_res,
            'geometric_center_supported': False,
            'geometric_center_support_distance_px': None,
            'geometric_center_support_threshold_px': float(max_support_distance_px)
        }
        
    cu, cv_ = rect_res['center_uv']
    c_ch = p0 + cu * e1 + cv_ * e2
    
    # 4 corners in 3D chassis
    corners_ch = []
    for (u_c, v_c) in rect_res['corners_uv']:
        corners_ch.append(p0 + u_c * e1 + v_c * e2)
    corners_ch = np.asarray(corners_ch)
    
    # Convert center to camera optical frame:
    # T_chassis_camera maps camera point to chassis point: P_ch = T[:3, :3] @ P_cam + T[:3, 3] * 1000
    R = T_chassis_camera[:3, :3]
    t = T_chassis_camera[:3, 3] * 1000.0
    c_cam = R.T @ (c_ch - t)
    
    # Corners in camera frame
    corners_cam = (corners_ch - t) @ R
    
    # Project center and corners to pixel
    h, w = image_shape[:2]
    c_px = None
    corners_px = []
    if c_cam[2] > 10.0:
        c_px = np.array([
            K['fx'] * c_cam[0] / c_cam[2] + K['cx'],
            K['fy'] * c_cam[1] / c_cam[2] + K['cy']
        ])
    for c_pt in corners_cam:
        if c_pt[2] > 10.0:
            corners_px.append([
                float(K['fx'] * c_pt[0] / c_pt[2] + K['cx']),
                float(K['fy'] * c_pt[1] / c_pt[2] + K['cy'])
            ])
            
    reasons = []
    # Check 1: In front of camera
    if c_cam[2] <= 10.0:
        reasons.append('geometric_center_behind_camera')
        
    # Check 2: Finite pixel coordinates
    if c_px is None or not (np.isfinite(c_px[0]) and np.isfinite(c_px[1])):
        reasons.append('geometric_center_pixel_non_finite')
    else:
        # Check 3: Inside image boundaries
        if c_px[0] < 0 or c_px[0] >= w or c_px[1] < 0 or c_px[1] >= h:
            reasons.append('geometric_center_projection_outside_image')

    # Check 4: Support region verification with top_mask
    center_supported = False
    center_support_dist_px = 0.0
    if c_px is not None and np.isfinite(c_px[0]) and np.isfinite(c_px[1]) and top_mask is not None:
        ix = int(round(c_px[0]))
        iy = int(round(c_px[1]))
        if 0 <= iy < h and 0 <= ix < w and bool(top_mask[iy, ix]):
            center_support_dist_px = 0.0
        else:
            my, mx = np.where(top_mask)
            if len(mx) > 0:
                center_support_dist_px = float(np.sqrt(np.min((mx - c_px[0])**2 + (my - c_px[1])**2)))
            else:
                center_support_dist_px = float('inf')
                
        center_supported = (center_support_dist_px <= max_support_distance_px)
        if not center_supported:
            reasons.append(
                f'geometric_center_unsupported_distance_{center_support_dist_px:.1f}px>{max_support_distance_px:.1f}px'
            )
    elif top_mask is None:
        # Backward-compatible fallback if top_mask not supplied
        center_supported = True
        center_support_dist_px = 0.0
            
    valid = len(reasons) == 0
    return {
        'valid': valid,
        'reasons': reasons,
        'center_chassis_mm': c_ch.tolist() if valid else None,
        'center_camera_mm': c_cam.tolist() if valid else None,
        'center_pixel': c_px.tolist() if c_px is not None else None,
        'corners_chassis_mm': corners_ch.tolist(),
        'corners_camera_mm': corners_cam.tolist(),
        'corners_pixel': corners_px,
        'fitted_rectangle': rect_res,
        'geometric_center_supported': center_supported,
        'geometric_center_support_distance_px': center_support_dist_px,
        'geometric_center_support_threshold_px': float(max_support_distance_px)
    }

def analyze_existing_analyses(
    analyses: Dict[str, Any],
    mask: np.ndarray,
    K: Dict[str, float],
    T: np.ndarray,
    selected_erosion: int = 1,
    max_support_distance_px: float = DEFAULT_MAX_SUPPORT_DISTANCE_PX,
    cfg: Optional[Dict[str, Any]] = None
) -> Dict[str, Any]:
    """
    Directly consume precomputed erosion analyses (0/1/2) from server.py,
    reusing plane fits without redundant point cloud backprojections.
    """
    cfg = cfg or {}
    max_support_dist = float(cfg.get('max_support_distance_px', max_support_distance_px))
    max_point_stab = float(cfg.get('max_point_stability_mm', DEFAULT_MAX_POINT_STABILITY_MM))
    
    selected_key = str(selected_erosion)
    if selected_key not in analyses:
        selected_key = '1' if '1' in analyses else list(analyses.keys())[0]
        
    main = analyses[selected_key]
    
    # 1. Plane fit validity from selected erosion
    plane_ok = bool(main.get('fit', {}).get('valid'))
    
    # 2. Diagnostic 2D points on original mask
    my, mx = np.where(mask)
    mask_centroid_px = [float(mx.mean()), float(my.mean())] if len(mx) else None
    bx, by, bw, bh = (int(mx.min()), int(my.min()), int(mx.max() - mx.min() + 1), int(my.max() - my.min() + 1)) if len(mx) else (0, 0, 0, 0)
    bbox_center_px = [float(bx + bw / 2.0), float(by + bh / 2.0)]
    
    # 3. Candidate 3D Geometric Center across 0/1/2 erosions
    geo_variants = {}
    valid_centers_cam = []
    valid_centers_ch = []
    rect_sizes = []
    
    for e_str, a in analyses.items():
        fit = a.get('fit', {})
        if fit.get('valid') and ('chassis_points' in a):
            geo_res = compute_box_geometric_center(
                a['chassis_points'], fit['keep'], fit['plane_point_chassis_mm'],
                fit['normal_chassis'], T, K, mask.shape,
                top_mask=fit.get('top_mask'),
                max_support_distance_px=max_support_dist,
                cfg=cfg
            )
            geo_variants[e_str] = geo_res
            if geo_res['valid']:
                valid_centers_cam.append(np.asarray(geo_res['center_camera_mm']))
                valid_centers_ch.append(np.asarray(geo_res['center_chassis_mm']))
                rect_sizes.append((geo_res['fitted_rectangle']['length_mm'], geo_res['fitted_rectangle']['width_mm']))
        else:
            geo_variants[e_str] = {'valid': False, 'reasons': ['plane_fit_invalid']}
            
    # Check erosion stability for candidate geometric center
    geo_spread_mm = None
    len_spread_mm = None
    wid_spread_mm = None
    geo_reasons = []
    
    if not plane_ok:
        geo_reasons.append('top_plane_invalid')
    elif len(valid_centers_cam) < 2:
        geo_reasons.append('insufficient_valid_erosion_variants')
    else:
        geo_spread_mm = float(max(np.linalg.norm(a - b) for a in valid_centers_cam for b in valid_centers_cam))
        len_spread_mm = float(max(s[0] for s in rect_sizes) - min(s[0] for s in rect_sizes))
        wid_spread_mm = float(max(s[1] for s in rect_sizes) - min(s[1] for s in rect_sizes))
        if geo_spread_mm > max_point_stab:
            geo_reasons.append(f'geometric_center_unstable_across_erosion_{geo_spread_mm:.2f}mm>{max_point_stab:.1f}mm')
            
    main_geo = geo_variants.get(selected_key, {})
    if not main_geo.get('valid'):
        geo_reasons.extend(main_geo.get('reasons', []))
    geo_reasons = list(dict.fromkeys(geo_reasons))
    
    geo_valid = plane_ok and (len(geo_reasons) == 0) and bool(main_geo.get('valid', False))
    
    cand = {
        'valid': geo_valid,
        'camera_mm': main_geo.get('center_camera_mm') if geo_valid else None,
        'chassis_mm': main_geo.get('center_chassis_mm') if geo_valid else None,
        'pixel': main_geo.get('center_pixel') if geo_valid else None,
        'rejection_reasons': geo_reasons,
        'support_distance_px': main_geo.get('geometric_center_support_distance_px', 0.0),
        'support_threshold_px': main_geo.get('geometric_center_support_threshold_px', max_support_dist),
        'geometric_center_supported': main_geo.get('geometric_center_supported', False),
        'erosion_center_spread_mm': geo_spread_mm,
        'length_spread_mm': len_spread_mm,
        'width_spread_mm': wid_spread_mm,
        'fitted_rectangle': main_geo.get('fitted_rectangle'),
        'corners_camera_mm': main_geo.get('corners_camera_mm'),
        'corners_pixel': main_geo.get('corners_pixel')
    }
    
    return {
        'top_plane_valid': plane_ok,
        'candidate': cand,
        'candidate_geometric_center': cand,
        'diagnostic_points': {
            'bbox': [bx, by, bw, bh],
            'bbox_center_pixel': bbox_center_px,
            'whole_mask_centroid_pixel': mask_centroid_px
        }
    }

def analyze_box_multi_erosion(
    mask: np.ndarray,
    depth: np.ndarray,
    K: Dict[str, float],
    T: np.ndarray,
    cfg: Dict[str, Any],
    box_module,
    mode: str = 'horizontal',
    selected_erosion: int = 1,
    max_support_distance_px: float = DEFAULT_MAX_SUPPORT_DISTANCE_PX
) -> Dict[str, Any]:
    """
    Offline/standalone runner: computes 0/1/2 erosions using requested mode
    and passes to analyze_existing_analyses.
    """
    analyses = {}
    for e in (0, 1, 2):
        analyses[str(e)] = box_module.analyse(mask, depth, K, T, e, mode, cfg, compute_interior=False)
        
    return analyze_existing_analyses(
        analyses,
        mask,
        K,
        T,
        selected_erosion=selected_erosion,
        max_support_distance_px=max_support_distance_px,
        cfg=cfg
    )

def draw_candidate_overlay(
    rgb: np.ndarray,
    mask: np.ndarray,
    result: Dict[str, Any],
    K: Dict[str, float],
    output_path: Optional[str] = None,
    top_mask: Optional[np.ndarray] = None
) -> np.ndarray:
    """
    Render comprehensive diagnostic overlay showing 5 distinct markers and fitted 3D rectangle.
    
    Markers:
    - Bbox & Bbox Midpoint: Yellow 'X' (Prohibited)
    - Whole Mask Centroid: Cyan '+' (Diagnostic only)
    - Legacy Interior Point: Magenta 'O' (Safety suction)
    - Candidate 3D Geometric Center: Bright Green bullseye/crosshair
    - Fitted Rectangle: Bright Green 4-corner polygon
    - Top Surface Inliers: Light green tint
    """
    overlay = rgb.copy()
    h, w = overlay.shape[:2]
    
    # 1. Tint top surface mask if present
    if top_mask is None:
        plane_fit = result.get('plane_fit', {})
        if isinstance(plane_fit, dict):
            top_mask = plane_fit.get('top_mask')
    if top_mask is not None and top_mask.shape == (h, w):
        overlay[top_mask] = (0.70 * overlay[top_mask] + 0.30 * np.array([0, 220, 0])).astype(np.uint8)
    elif mask is not None and mask.shape == (h, w):
        overlay[mask] = (0.80 * overlay[mask] + 0.20 * np.array([255, 180, 0])).astype(np.uint8)
        
    diag = result.get('diagnostic_points', {})
    legacy = result.get('legacy_interior_point', {})
    cand = result.get('candidate', result.get('candidate_geometric_center', {}))
    
    # 2. Draw 2D Bbox and Bbox Center (Yellow X)
    bx, by, bw, bh = diag.get('bbox', [0, 0, 0, 0])
    if bw > 0 and bh > 0:
        cv2.rectangle(overlay, (bx, by), (bx + bw, by + bh), (0, 240, 240), 1, cv2.LINE_AA)
    bbox_c = diag.get('bbox_center_pixel')
    if bbox_c is not None:
        pt = (int(round(bbox_c[0])), int(round(bbox_c[1])))
        cv2.drawMarker(overlay, pt, (0, 240, 240), cv2.MARKER_TILTED_CROSS, 16, 2, cv2.LINE_AA)
        
    # 3. Draw Mask Centroid (Cyan +)
    c_mask = diag.get('whole_mask_centroid_pixel')
    if c_mask is not None:
        pt = (int(round(c_mask[0])), int(round(c_mask[1])))
        cv2.drawMarker(overlay, pt, (255, 230, 0), cv2.MARKER_CROSS, 16, 2, cv2.LINE_AA)
        
    # 4. Draw Legacy Interior Point (Magenta O)
    int_px = legacy.get('pixel') if isinstance(legacy, dict) else None
    if int_px is not None:
        pt = (int(round(int_px[0])), int(round(int_px[1])))
        cv2.circle(overlay, pt, 6, (255, 0, 255), 2, cv2.LINE_AA)
        cv2.circle(overlay, pt, 2, (255, 0, 255), -1, cv2.LINE_AA)
        
    # 5. Draw Fitted 3D Rectangle and Candidate Geometric Center (Bright Green)
    corners_px = cand.get('corners_pixel')
    if corners_px is not None and len(corners_px) == 4:
        poly = np.array(corners_px, dtype=np.int32).reshape((-1, 1, 2))
        cv2.polylines(overlay, [poly], True, (0, 255, 0), 2, cv2.LINE_AA)
        
    geo_px = cand.get('pixel')
    if geo_px is not None:
        pt = (int(round(geo_px[0])), int(round(geo_px[1])))
        cv2.circle(overlay, pt, 9, (0, 255, 0), 2, cv2.LINE_AA)
        cv2.circle(overlay, pt, 3, (0, 255, 0), -1, cv2.LINE_AA)
        cv2.drawMarker(overlay, pt, (0, 255, 0), cv2.MARKER_CROSS, 20, 1, cv2.LINE_AA)
        
    # 6. Draw HUD Info Box
    hud_h = 240
    hud_w = 430
    hud = overlay[10:10+hud_h, 10:10+hud_w].astype(float)
    overlay[10:10+hud_h, 10:10+hud_w] = (0.25 * hud + 0.75 * np.array([20, 20, 20])).astype(np.uint8)
    cv2.rectangle(overlay, (10, 10), (10+hud_w, 10+hud_h), (80, 80, 80), 1, cv2.LINE_AA)
    
    font = cv2.FONT_HERSHEY_SIMPLEX
    lines = [
        ("BOX TOP SURFACE POINT COMPARISON", (255, 255, 255), 0.45, 1),
        ("X Yellow : 2D Bbox Midpoint (Prohibited)", (0, 240, 240), 0.40, 1),
        ("+ Cyan   : Whole Mask Centroid (Diagnostic)", (255, 230, 0), 0.40, 1),
        ("O Magenta: Legacy Interior Pt (Safety Suction)", (255, 0, 255), 0.40, 1),
        ("o Green  : Candidate 3D Top Geometric Center", (0, 255, 0), 0.40, 1),
    ]
    
    # Add metrics
    int_dist = diag.get('interior_to_geometric_center_distance_mm')
    d_str = f"{int_dist:.2f} mm" if int_dist is not None else "N/A"
    lines.append((f"Delta(Interior -> Geometric Center): {d_str}", (200, 200, 200), 0.38, 1))
    
    spread = cand.get('erosion_sensitivity', {}).get('center_spread_mm')
    sp_str = f"{spread:.2f} mm (Gate <= 5.0mm)" if spread is not None else "N/A"
    lines.append((f"Candidate Erosion Spread (0/1/2): {sp_str}", (200, 200, 200), 0.38, 1))
    
    rect_fit = cand.get('fitted_rectangle')
    if rect_fit:
        lw_str = f"{rect_fit.get('length_mm', 0):.1f} x {rect_fit.get('width_mm', 0):.1f} mm"
        lines.append((f"Fitted Rectangle: {lw_str}", (200, 200, 200), 0.38, 1))
        
    sup_dist = cand.get('geometric_center_support_distance_px', 0.0)
    sup_status = "Supported (0.0px)" if cand.get('geometric_center_supported') else f"Dist {sup_dist:.1f}px (Gate <= {cand.get('geometric_center_support_threshold_px', DEFAULT_MAX_SUPPORT_DISTANCE_PX):.1f}px)"
    lines.append((f"Support Region: {sup_status}", (200, 200, 200), 0.38, 1))
        
    status_str = "VALID (PASS)" if cand.get('valid') else f"REJECTED: {','.join(cand.get('rejection_reasons', []))}"
    col = (0, 255, 0) if cand.get('valid') else (0, 0, 255)
    lines.append((f"Status: {status_str}", col, 0.40, 1))
    
    y = 28
    for txt, color, sc, th in lines:
        cv2.putText(overlay, txt, (18, y), font, sc, color, th, cv2.LINE_AA)
        y += 22
        
    if output_path is not None:
        cv2.imwrite(str(output_path), overlay)
        
    return overlay
