#!/usr/bin/env python3
"""Multi-target geometric pose fitting for front-row SKU candidates.

Delegates category-specific geometric calculations to:
- fit_bottle_axis.py (bottle)
- fit_tube_top_edge.py (tube)
- fit_estee_box_top_surface.py / box_pipeline.py / box_geometric_center.py (box)

Provides unified target fitting workflows, erosion analysis helpers,
point stability calculations, reason deduplication, and an explicit
CATEGORY_FITTERS registry.
"""
from pathlib import Path
from typing import Any, Callable, Dict, List, Optional, Tuple, Union
import cv2
import numpy as np

try:
    from module_loader import get_deploy_modules
except ImportError:
    from deploy.module_loader import get_deploy_modules

_DEPLOY_DIR = Path(__file__).resolve().parent


def _get_deploy_module(name: str, filename: Optional[str] = None) -> Any:
    """Retrieves deploy module from central loader (backward-compatibility wrapper)."""
    return get_deploy_modules().get(name)


def merge_rejection_reasons(*reason_lists: List[str]) -> List[str]:
    """Combines multiple rejection reason lists while strictly preserving order and deduplicating."""
    merged: List[str] = []
    for r_list in reason_lists:
        if r_list:
            for item in r_list:
                if item and item not in merged:
                    merged.append(item)
    return merged


def calculate_point_stability(
    points: List[Union[List[float], np.ndarray]],
    max_stability_thresh: Optional[float] = None
) -> Tuple[Optional[float], bool]:
    """Calculates maximum pairwise Euclidean distance across points and checks threshold."""
    if len(points) < 2:
        return None, False
    point_var = float(
        max(np.linalg.norm(np.asarray(a) - np.asarray(b)) for a in points for b in points)
    )
    if max_stability_thresh is not None:
        return point_var, bool(point_var <= max_stability_thresh)
    return point_var, True


def run_erosion_variants(
    analyse_fn: Callable[..., Dict[str, Any]],
    erosions: Tuple[int, ...] = (0, 1, 2)
) -> Dict[str, Dict[str, Any]]:
    """Runs a multi-erosion analysis function across specified pixel erosion levels."""
    return {str(e): analyse_fn(e) for e in erosions}


def pack_target_result(
    item: Optional[Dict[str, Any]],
    fit_res: Dict[str, Any],
    slot_name: str,
    default_reason: str
) -> Dict[str, Any]:
    """Assembles a single target slot (center, left, right) into standard schema."""
    if not item:
        return {
            "instance_id": None,
            "valid": False,
            "point_camera_mm": None,
            "point_chassis_mm": None,
            "score": None,
            "reasons": ["no target candidate"],
            "reason": default_reason,
        }
    reasons = fit_res.get("reasons", [])
    is_valid = bool(fit_res.get("valid", False))
    return {
        "instance_id": int(item["instance_id"]),
        "valid": is_valid,
        "point_camera_mm": fit_res.get("point_camera_mm"),
        "point_chassis_mm": fit_res.get("point_robot_mm"),
        "axis_direction_camera_up": fit_res.get("axis_direction_camera_up"),
        "axis_direction_robot": fit_res.get("axis_direction_robot"),
        "score": float(item.get("score", 0.0)),
        "reasons": reasons,
        "reason": f"front-row instance {slot_name}" if is_valid else (reasons[0] if reasons else "fitting failed"),
    }


def fit_single_bottle_target(
    selected: Optional[Dict[str, Any]],
    req: Dict[str, Any],
    depth: np.ndarray,
    K: Dict[str, float],
    T_m: np.ndarray,
    geometry_cache: Any,
    fit_module: Any = None
) -> Dict[str, Any]:
    """Fits 3D cylinder axis and reference point for a single bottle instance."""
    if not selected:
        return {"valid": False, "reasons": ["no target candidate"]}

    FIT = fit_module or _get_deploy_module("fit_bottle_axis")
    mask = selected["_mask"]
    raw, _, _, _ = geometry_cache.get_mask(mask, depth, K, T_m)
    radius = float(req.get("body_radius_mm", 28.5))
    mode = req.get("fit_mode", "cylinder_3d")
    prior = FIT.norm(T_m[:3, :3].T @ np.array([0.0, 0.0, 1.0]))

    try:
        fm = FIT.body_filter(mask, depth, 4, "bottle_body")
        fit, _, _, _ = geometry_cache.get_mask(fm, depth, K, T_m)
        p, a, res, ins, extra, stab = FIT.robust_fit(
            fit, radius, prior if mode == "prior_2d" else None, mode, None
        )
        a = FIT.canonical(a, prior)
        ts = (fit - p) @ a
        q = np.percentile(ts[ins], [5, 95])
        s_center = float(np.mean(q))
        ref = p + s_center * a
        refch, _ = FIT.to_chassis(ref, np.zeros(3), T_m)

        maxvis = max(stab["visible_common_section_center_mm"], default=float("inf"))
        maxang = max(stab["axis_angle_deg"], default=float("inf"))
        axis_ok = bool(
            len(fit) >= FIT.MIN_BODY_FIT_POINTS
            and q[1] - q[0] >= FIT.MIN_VISIBLE_AXIS_SPAN_MM
            and ins.mean() >= 0.5
            and np.median(abs(res)) <= 3
            and np.percentile(abs(res), 90) <= 8
            and maxvis <= FIT.MAX_BOOTSTRAP_VISIBLE_CENTER_MM
            and (mode != "cylinder_3d" or maxang <= FIT.MAX_BOOTSTRAP_AXIS_ANGLE_DEG)
            and not extra.get("candidate_solution_ambiguity", False)
        )
        ref_ok = bool(axis_ok)

        reasons = [] if axis_ok else ["cylinder axis quality gate failed"]
        chassis_axis = T_m[:3, :3] @ a

        return {
            "valid": bool(axis_ok and ref_ok),
            "axis_ok": axis_ok,
            "ref_ok": ref_ok,
            "point_robot_mm": refch.tolist() if ref_ok else None,
            "point_camera_mm": ref.tolist() if ref_ok else None,
            "axis_point_camera_mm": p.tolist(),
            "axis_direction_camera_up": a.tolist(),
            "axis_direction_robot": chassis_axis.tolist(),
            "reasons": reasons,
            "_raw": raw, "_fit": fit, "_ins": ins, "_fm": fm, "p": p.tolist(), "a": a.tolist(), "q": q.tolist(),
            "ref": ref.tolist(), "refch": refch.tolist(), "stab": stab, "extra": extra, "s_center": s_center, "res": res.tolist()
        }
    except Exception as exc:
        return {
            "valid": False, "axis_ok": False, "ref_ok": False,
            "point_robot_mm": None, "point_camera_mm": None,
            "axis_point_camera_mm": None, "axis_direction_camera_up": None,
            "reasons": [f"cylinder fitting failed: {exc}"]
        }


def fit_single_tube_target(
    selected: Optional[Dict[str, Any]],
    req: Dict[str, Any],
    depth: np.ndarray,
    K: Dict[str, float],
    T_m: np.ndarray,
    geometry_cache: Any,
    tube_module: Any = None
) -> Dict[str, Any]:
    """Fits top-edge position and stability for a single tube instance."""
    if not selected:
        return {"valid": False, "reasons": ["no target candidate"]}

    TUBE = tube_module or _get_deploy_module("fit_tube_top_edge")
    mask = selected["_mask"]
    erosion = int(req.get("mask_erosion_pixels", 1))
    cfg = TUBE.make_height_band_cfg(req)

    analyses = run_erosion_variants(
        lambda e: TUBE.analyse_height_band(mask, depth, K, T_m, e, cfg, geometry_cache=geometry_cache),
        erosions=(0, 1, 2)
    )
    main = analyses[str(erosion)]

    points = [a["reference_camera_mm"] for a in analyses.values() if a.get("valid")]
    point_var, is_stable = calculate_point_stability(points, TUBE.DEFAULT_MAX_POINT_STABILITY_MM)
    depth_ratio = main.get("mask_valid_depth_ratio", 0)
    edge_ok = bool(main.get("valid"))
    point_ok = bool(
        edge_ok
        and point_var is not None
        and is_stable
        and depth_ratio >= cfg["min_mask_valid_depth_ratio"]
    )

    reasons: List[str] = []
    if not edge_ok:
        reasons.append(main.get("reason", "top edge invalid"))
    if depth_ratio < cfg["min_mask_valid_depth_ratio"]:
        reasons.append("valid depth ratio below threshold")
    if point_var is None:
        reasons.append("insufficient successful erosion variants for stability check")
    elif not is_stable:
        reasons.append("top-edge point unstable across 0/1/2 px erosion")
    reasons = merge_rejection_reasons(reasons)

    ref_cam = main.get("reference_camera_mm") if point_ok else None
    ref_ch = main.get("reference_chassis_mm") if point_ok else None
    dir_cam = main.get("direction_camera") if point_ok else None
    dir_ch = (T_m[:3, :3] @ np.asarray(dir_cam)).tolist() if (point_ok and dir_cam is not None) else None

    return {
        "valid": bool(edge_ok and point_ok),
        "edge_ok": edge_ok,
        "point_ok": point_ok,
        "point_robot_mm": ref_ch,
        "point_camera_mm": ref_cam,
        "axis_direction_camera_up": dir_cam,
        "axis_direction_robot": dir_ch,
        "top_edge_endpoints_camera_mm": main.get("endpoints_camera_mm") if point_ok else None,
        "reasons": reasons,
        "main_analysis": main,
        "point_var": point_var,
        "depth_ratio": depth_ratio
    }


def fit_single_box_target(
    selected: Optional[Dict[str, Any]],
    req: Dict[str, Any],
    depth: np.ndarray,
    K: Dict[str, float],
    T_m: np.ndarray,
    geometry_cache: Any,
    box_module: Any = None,
    bgc_module: Any = None
) -> Dict[str, Any]:
    """Fits top planar surface and geometric center for a single box instance."""
    if not selected:
        return {"valid": False, "reasons": ["no target candidate"]}

    BOX = box_module or _get_deploy_module("fit_estee_box_top_surface")
    BGC = bgc_module or _get_deploy_module("box_geometric_center")

    mask = selected["_mask"]
    cfg = {
        "tol": float(req.get("height_tolerance_mm", BOX.DEFAULT_HEIGHT_TOLERANCE_MM)),
        "min_points": int(req.get("min_top_points", BOX.DEFAULT_MIN_TOP_POINTS)),
        "min_span": float(req.get("min_tangent_span_mm", BOX.DEFAULT_MIN_TANGENT_SPAN_MM)),
        "min_area": float(req.get("min_top_area_mm2", BOX.DEFAULT_MIN_TOP_AREA_MM2)),
        "max_tilt": float(req.get("normal_max_tilt_deg", BOX.DEFAULT_NORMAL_MAX_TILT_DEG)),
        "max_med": float(req.get("max_residual_median_mm", BOX.DEFAULT_MAX_RESIDUAL_MEDIAN_MM)),
        "max_p90": float(req.get("max_residual_p90_mm", BOX.DEFAULT_MAX_RESIDUAL_P90_MM)),
        "min_margin": float(req.get("min_interior_margin_px", BOX.DEFAULT_MIN_INTERIOR_MARGIN_PX))
    }
    mode = req.get("plane_mode", getattr(BOX, "DEFAULT_PLANE_MODE", "horizontal"))
    analyses = run_erosion_variants(
        lambda e: BOX.analyse(mask, depth, K, T_m, e, mode, cfg, geometry_cache=geometry_cache, compute_interior=False),
        erosions=(0, 1, 2)
    )
    main = analyses.get("1", analyses.get("0"))
    plane_ok = bool(main.get("fit", {}).get("valid"))

    bgc_analysis = BGC.analyze_existing_analyses(analyses, mask, K, T_m, selected_erosion=1, cfg=cfg)
    cand = bgc_analysis.get("candidate", bgc_analysis.get("candidate_geometric_center", {}))
    geo_usable = bool(plane_ok and cand.get("valid") and cand.get("geometric_center_supported") and cand.get("camera_mm") is not None)

    ref_cam = cand.get("camera_mm") if geo_usable else main.get("reference_camera_mm")
    ref_ch = cand.get("chassis_mm") if geo_usable else main.get("reference_chassis_mm")
    is_valid = bool(plane_ok and ref_cam is not None)

    reasons = [] if is_valid else (main.get("fit", {}).get("reasons", []) or ["box plane quality gate failed"])
    normal_cam = main.get("fit", {}).get("normal_camera")
    normal_ch = (T_m[:3, :3] @ np.asarray(normal_cam)).tolist() if normal_cam is not None else None

    return {
        "valid": is_valid,
        "plane_ok": plane_ok,
        "point_ok": is_valid,
        "point_robot_mm": ref_ch,
        "point_camera_mm": ref_cam,
        "axis_direction_camera_up": normal_cam,
        "axis_direction_robot": normal_ch,
        "top_point_uv": cand.get("uv") if geo_usable else main.get("reference_uv"),
        "reasons": reasons,
        "_analyses": analyses,
        "bgc_analysis": bgc_analysis,
    }


def adapt_bottle_response(center_fit: Dict[str, Any]) -> Dict[str, Any]:
    """Generates bottle-specific top-level backward compatibility fields."""
    return {
        "axis_fit_valid": center_fit.get("axis_ok", False),
        "reference_point_valid": center_fit.get("ref_ok", False),
        "axis_point_camera_mm": center_fit.get("axis_point_camera_mm"),
        "axis_direction_camera_up": center_fit.get("axis_direction_camera_up"),
    }


def adapt_tube_response(center_fit: Dict[str, Any]) -> Dict[str, Any]:
    """Generates tube-specific top-level backward compatibility fields."""
    return {
        "edge_valid": center_fit.get("edge_ok", False),
        "point_valid": center_fit.get("point_ok", False),
        "top_edge_center_camera_mm": center_fit.get("point_camera_mm"),
        "top_edge_center_chassis_mm": center_fit.get("point_robot_mm"),
        "top_edge_endpoints_camera_mm": center_fit.get("top_edge_endpoints_camera_mm"),
        "edge_direction_camera": center_fit.get("axis_direction_camera_up"),
    }


def adapt_box_response(center_fit: Dict[str, Any]) -> Dict[str, Any]:
    """Generates box-specific top-level backward compatibility fields."""
    return {
        "top_plane_valid": center_fit.get("plane_ok", False),
        "top_point_valid": center_fit.get("point_ok", False),
        "top_point_camera_mm": center_fit.get("point_camera_mm"),
        "top_point_chassis_mm": center_fit.get("point_robot_mm"),
        "top_point_uv": center_fit.get("top_point_uv"),
    }


CATEGORY_FITTERS: Dict[str, Callable[..., Dict[str, Any]]] = {
    "bottle": fit_single_bottle_target,
    "tube": fit_single_tube_target,
    "box": fit_single_box_target,
}

CATEGORY_ADAPTERS: Dict[str, Callable[[Dict[str, Any]], Dict[str, Any]]] = {
    "bottle": adapt_bottle_response,
    "tube": adapt_tube_response,
    "box": adapt_box_response,
}


def build_targets_dict(
    category: str,
    center_item: Optional[Dict[str, Any]],
    center_fit: Dict[str, Any],
    left_item: Optional[Dict[str, Any]],
    left_fit: Dict[str, Any],
    right_item: Optional[Dict[str, Any]],
    right_fit: Dict[str, Any]
) -> Dict[str, Any]:
    """Assembles the three targets (center, left, right) into a standardized response dictionary."""
    return {
        "center": pack_target_result(center_item, center_fit, "nearest box center", "no front-row instance nearest box center"),
        "left": pack_target_result(left_item, left_fit, "touching left box boundary", "no front-row instance touches the left box boundary"),
        "right": pack_target_result(right_item, right_fit, "touching right box boundary", "no front-row instance touches the right box boundary"),
    }


def fit_category_targets(
    selection: Dict[str, Any],
    category: str,
    rgb: np.ndarray,
    depth: np.ndarray,
    K: Dict[str, float],
    T_m: np.ndarray,
    req: Dict[str, Any],
    geometry_cache: Any,
    root: Optional[Path] = None,
    modules: Optional[Dict[str, Any]] = None
) -> Dict[str, Any]:
    """Computes poses for center, left, and right targets and returns response fields."""
    if category not in CATEGORY_FITTERS:
        raise ValueError(f"Unknown SKU category: {category}")

    modules = modules or {}
    center_item = selection.get("center_item")
    left_item = selection.get("left_item")
    right_item = selection.get("right_item")

    if category == "bottle":
        fit_fn = lambda item: fit_single_bottle_target(item, req, depth, K, T_m, geometry_cache, modules.get("fit_bottle_axis"))
    elif category == "tube":
        fit_fn = lambda item: fit_single_tube_target(item, req, depth, K, T_m, geometry_cache, modules.get("fit_tube_top_edge"))
    elif category == "box":
        fit_fn = lambda item: fit_single_box_target(item, req, depth, K, T_m, geometry_cache, modules.get("fit_estee_box_top_surface"), modules.get("box_geometric_center"))

    center_fit = fit_fn(center_item)
    left_fit = fit_fn(left_item)
    right_fit = fit_fn(right_item)

    targets = build_targets_dict(category, center_item, center_fit, left_item, left_fit, right_item, right_fit)

    # Top-level backward compatibility points to center
    ok = bool(center_fit.get("valid", False))
    selected_iid = center_item["instance_id"] if center_item else None
    ref_chassis = center_fit.get("point_robot_mm")
    ref_camera = center_fit.get("point_camera_mm")

    if not ok:
        if not center_item:
            rejection_reasons = ["no front-row candidate nearest box center"]
        else:
            rejection_reasons = center_fit.get("reasons", ["center candidate quality gate failed"])
    else:
        rejection_reasons = []

    res: Dict[str, Any] = {
        "ok": ok,
        "selection_strategy": "front_row_center_left_right",
        "grasp_order": ["center", "left", "right"],
        "targets": targets,
        "selected_instance_id": selected_iid,
        "reference_point_chassis_mm": ref_chassis,
        "reference_point_camera_mm": ref_camera,
        "rejection_reasons": rejection_reasons,
        "center_fit": center_fit,
        "left_fit": left_fit,
        "right_fit": right_fit,
    }

    # Category-specific top-level compatibility fields
    adapter = CATEGORY_ADAPTERS.get(category)
    if adapter:
        res.update(adapter(center_fit))

    return res
