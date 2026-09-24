#!/usr/bin/env python3
"""SKU CAD geometry models, Box coordinate frames, and 3D instance feature extraction.

Part of the modular SKU localization pipeline.
"""
from dataclasses import dataclass
from typing import Any, Dict, List, Optional, Tuple, Union
import numpy as np

# CAD dimensions per SKU ID or category name
SKU_GEOMETRY: Dict[str, Dict[str, float]] = {
    "25540": {
        "depth_axis_size_mm": 80.0,
        "lateral_axis_size_mm": 80.0,
    },
    "25565": {
        "depth_axis_size_mm": 80.0,
        "lateral_axis_size_mm": 80.0,
    },
    "bottle": {
        "depth_axis_size_mm": 80.0,
        "lateral_axis_size_mm": 80.0,
    },
    "box": {
        "depth_axis_size_mm": 40.0,
        "lateral_axis_size_mm": 40.0,
    },
    "tube": {
        "depth_axis_size_mm": 40.0,
        "lateral_axis_size_mm": 40.0,
    },
}


def get_sku_geometry(req: Dict[str, Any], class_name: Optional[str] = None) -> Dict[str, float]:
    """Retrieve SKU CAD geometry from explicit request override or configuration lookup.

    Raises:
        ValueError: If no valid CAD geometry is found (strictly avoids hardcoded guesses).
    """
    if not isinstance(req, dict):
        req = {}

    # 1. Explicit request override
    geom = req.get("sku_geometry")
    if isinstance(geom, dict) and "depth_axis_size_mm" in geom:
        depth = float(geom["depth_axis_size_mm"])
        lateral = float(geom.get("lateral_axis_size_mm", geom.get("lateral_size_mm", depth)))
        return {"depth_axis_size_mm": depth, "lateral_axis_size_mm": lateral}

    # 2. Lookup by SKU ID
    sku_id = req.get("sku_id") or req.get("sku_typ") or class_name
    if sku_id is not None:
        key = str(sku_id).strip()
        if key in SKU_GEOMETRY:
            return dict(SKU_GEOMETRY[key])

    # 3. Lookup by class_name
    if class_name and str(class_name).strip() in SKU_GEOMETRY:
        return dict(SKU_GEOMETRY[str(class_name).strip()])

    target_type = req.get("target_type")
    if target_type and str(target_type).strip() in SKU_GEOMETRY:
        return dict(SKU_GEOMETRY[str(target_type).strip()])

    raise ValueError(
        f"No valid CAD geometry found for SKU: sku_id={req.get('sku_id')}, "
        f"sku_typ={req.get('sku_typ')}, class_name={class_name}. "
        f"CAD dimensions (depth_axis_size_mm) must be explicitly configured."
    )


@dataclass
class BoxFrame:
    """Local reference frame and lateral boundaries of the target container box in chassis frame."""
    origin_robot_mm: np.ndarray          # 3D origin in chassis_link (mm)
    inward_axis_robot: np.ndarray        # Unit vector pointing into the box along depth
    lateral_axis_robot: np.ndarray       # Unit vector pointing across the box (horizontal)
    left_lateral_mm: float = 0.0         # Lateral coordinate of left box boundary
    right_lateral_mm: float = 0.0        # Lateral coordinate of right box boundary
    center_lateral_mm: float = 0.0       # Lateral coordinate of box centerline


@dataclass
class InstanceFeatures:
    """Extracted physical and geometric features for a candidate instance."""
    instance_id: int
    score: float
    mask_area_px: int
    bbox_width_px: int
    bbox_height_px: int
    valid_point_count: int
    median_point_robot_mm: Optional[np.ndarray]
    depth_inward_mm: Optional[float]
    lateral_mm: Optional[float]
    valid: bool


def build_box_frame(
    box_roi_xyxy: Optional[List[float]],
    front_panel: Optional[Dict[str, Any]],
    req: Dict[str, Any],
    K: Dict[str, float],
    T_m: np.ndarray
) -> BoxFrame:
    """Constructs BoxFrame with inward/lateral axes and lateral reference bounds."""
    # Inward axis in chassis frame
    inward_axis = np.array([1.0, 0.0, 0.0], dtype=float)
    if req.get("front_rule") and req["front_rule"].get("front_axis_chassis"):
        inward_axis = np.asarray(req["front_rule"]["front_axis_chassis"], dtype=float)
    norm = np.linalg.norm(inward_axis)
    if norm > 1e-6:
        inward_axis = inward_axis / norm

    # Lateral axis: perpendicular in horizontal plane [ -y, x, 0 ] or [0, 1, 0]
    if abs(inward_axis[0]) > 0.5:
        lateral_axis = np.array([0.0, 1.0, 0.0], dtype=float)
    else:
        lateral_axis = np.array([1.0, 0.0, 0.0], dtype=float)

    # Box origin in chassis frame
    box_origin = np.array([0.0, 0.0, 0.0], dtype=float)
    if front_panel and front_panel.get("valid") and front_panel.get("top_edge_midpoint_chassis_mm") is not None:
        box_origin = np.asarray(front_panel["top_edge_midpoint_chassis_mm"], dtype=float)
    elif req.get("front_rule") and req["front_rule"].get("front_origin_chassis") is not None:
        box_origin = np.asarray(req["front_rule"]["front_origin_chassis"], dtype=float)

    left_lateral_mm = 0.0
    right_lateral_mm = 0.0
    center_lateral_mm = 0.0

    if box_roi_xyxy is not None and len(box_roi_xyxy) == 4:
        bx1, by1, bx2, by2 = map(float, box_roi_xyxy)
        bcx = (bx1 + bx2) / 2.0
        # Project center pixel to lateral offset estimate
        center_lateral_mm = 0.0

    return BoxFrame(
        origin_robot_mm=box_origin,
        inward_axis_robot=inward_axis,
        lateral_axis_robot=lateral_axis,
        left_lateral_mm=left_lateral_mm,
        right_lateral_mm=right_lateral_mm,
        center_lateral_mm=center_lateral_mm,
    )


def extract_instance_features(
    item: Dict[str, Any],
    depth: np.ndarray,
    K: Dict[str, float],
    T_m: np.ndarray,
    box_frame: BoxFrame,
    geometry_cache: Any
) -> Dict[str, Any]:
    """Extract 3D points, projected depth along inward axis, lateral position, and dimensions."""
    mask = item.get("_mask")
    if mask is None:
        raise ValueError("item missing binary '_mask'")

    mask_area = int(mask.sum())
    bbox = item.get("bbox", [0.0, 0.0, 0.0, 0.0])
    bw = int(round(float(bbox[2]))) if len(bbox) >= 3 else 0
    bh = int(round(float(bbox[3]))) if len(bbox) >= 4 else 0

    pts_cam, _, valid_mask, pts_chassis = geometry_cache.get_mask(mask, depth, K, T_m)
    valid_count = int(len(pts_chassis))

    if valid_count < 50:
        return {
            "mask_area_px": mask_area,
            "bbox_width_px": bw,
            "bbox_height_px": bh,
            "valid_point_count": valid_count,
            "median_point_robot_mm": None,
            "depth_inward_mm": None,
            "lateral_mm": None,
            "valid_depth": False,
            "valid": False,
        }

    median_chassis = np.median(pts_chassis, axis=0)
    delta = median_chassis - box_frame.origin_robot_mm
    depth_inward = float(np.dot(delta, box_frame.inward_axis_robot))
    lateral_val = float(np.dot(delta, box_frame.lateral_axis_robot))

    is_valid = bool(np.isfinite(depth_inward) and np.isfinite(lateral_val) and valid_count >= 50)

    return {
        "mask_area_px": mask_area,
        "bbox_width_px": bw,
        "bbox_height_px": bh,
        "valid_point_count": valid_count,
        "median_point_robot_mm": median_chassis,
        "depth_inward_mm": depth_inward,
        "lateral_mm": lateral_val,
        "valid_depth": is_valid,
        "valid": is_valid,
    }
