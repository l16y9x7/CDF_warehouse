#!/usr/bin/env python3
"""Shared type definitions and TypedDict contracts for SKU spatial localization.

These annotations document cross-module data schemas without altering runtime dict behavior.
"""
from typing import Any, Dict, List, Optional, Tuple, TypedDict, Union
import numpy as np


class Detection(TypedDict, total=False):
    """Detection proposal returned from SAM3 or intermediate filter stage."""
    upstream_instance_id: int
    filtered_instance_id: Optional[int]
    class_name: Optional[str]
    score: float
    bbox: List[float]  # [x, y, w, h]
    segmentation: Dict[str, Any]  # COCO RLE format
    _mask: Optional[np.ndarray]  # 2D boolean numpy array
    crop_boundary_touched: Optional[bool]
    merged_suspect: Optional[bool]
    merged_area_ratio: Optional[float]
    merged_width_ratio: Optional[float]
    usable_for_selection: Optional[bool]
    touch_left: Optional[bool]
    touch_right: Optional[bool]
    left_touch_reason: Optional[str]
    right_touch_reason: Optional[str]
    row_label: Optional[str]
    depth_inward_mm: Optional[float]
    lateral_mm: Optional[float]


class TargetResult(TypedDict, total=False):
    """Localization and pose result for an individual target slot (center, left, or right)."""
    instance_id: Optional[int]
    valid: bool
    point_camera_mm: Optional[List[float]]
    point_chassis_mm: Optional[List[float]]
    axis_direction_camera_up: Optional[List[float]]
    axis_direction_robot: Optional[List[float]]
    score: Optional[float]
    reasons: List[str]
    reason: str


class FitResult(TypedDict, total=False):
    """Intermediate geometric fitting result for single SKU candidate."""
    valid: bool
    axis_ok: Optional[bool]
    ref_ok: Optional[bool]
    edge_ok: Optional[bool]
    plane_ok: Optional[bool]
    point_ok: Optional[bool]
    point_robot_mm: Optional[List[float]]
    point_camera_mm: Optional[List[float]]
    axis_point_camera_mm: Optional[List[float]]
    axis_direction_camera_up: Optional[List[float]]
    axis_direction_robot: Optional[List[float]]
    top_point_uv: Optional[List[float]]
    top_edge_endpoints_camera_mm: Optional[List[List[float]]]
    reasons: List[str]
    diagnostics: Optional[Dict[str, Any]]


class SelectionResult(TypedDict, total=False):
    """Candidate selection outcome from front-row multi-target logic."""
    front_row: List[Dict[str, Any]]
    center_item: Optional[Dict[str, Any]]
    left_item: Optional[Dict[str, Any]]
    right_item: Optional[Dict[str, Any]]
    audit: Dict[str, Any]


class AuditResult(TypedDict, total=False):
    """Audit summary for box selection or front-row spatial validation."""
    valid: bool
    stage_status: str
    reason: Optional[str]
    selection_policy: str
    roi_type: str
    box_candidates: List[Dict[str, Any]]
    box_score_rejected: List[Dict[str, Any]]
    left_right_rois_xyxy: Optional[List[List[float]]]
    selected_box_upstream_id: Optional[int]
    container_roi_xyxy: Optional[List[float]]


class ServiceConfig(TypedDict, total=False):
    """Per-category upstream SAM3 and box thresholds configuration."""
    sam3_prompt: str
    box_prompt: str
    box_threshold: float
    target_threshold: float
