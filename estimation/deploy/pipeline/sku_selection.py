#!/usr/bin/env python3
"""SKU Candidate Selection and Front-Row Multi-Target Logic (center, left, right).

Part of the modular SKU localization pipeline.
"""
from pathlib import Path
from typing import Any, Dict, List, Optional, Tuple, Union
import cv2
import numpy as np

# Ratios and thresholds per system specifications
ROW_GAP_RATIO       = 0.60
BOUNDARY_RATIO      = 0.20
MERGED_AREA_RATIO   = 1.80
MERGED_WIDTH_RATIO  = 1.60
MIN_VALID_POINTS    = 50


def detect_merged_instances(
    instances: List[Dict[str, Any]],
    min_count: int = 3,
    area_ratio_thresh: float = MERGED_AREA_RATIO,
    width_ratio_thresh: float = MERGED_WIDTH_RATIO
) -> List[Dict[str, Any]]:
    """Flags oversized or merged proposal masks as merged_suspect.

    Protects small sample sizes (< min_count).
    """
    valid_items = [x for x in instances if x.get("valid_depth") and x.get("mask_area_px", 0) > 0]
    if len(valid_items) < min_count:
        for x in instances:
            x["merged_suspect"] = False
            x["usable_for_selection"] = bool(x.get("valid_depth", False))
        return instances

    median_area = float(np.median([x["mask_area_px"] for x in valid_items]))
    median_width = float(np.median([x["bbox_width_px"] for x in valid_items]))

    for x in instances:
        if not x.get("valid_depth"):
            x["merged_suspect"] = False
            x["usable_for_selection"] = False
            continue

        area_ratio = float(x["mask_area_px"]) / max(1.0, median_area)
        width_ratio = float(x["bbox_width_px"]) / max(1.0, median_width)

        is_merged = bool(area_ratio > area_ratio_thresh or width_ratio > width_ratio_thresh)
        x["merged_suspect"] = is_merged
        x["merged_area_ratio"] = area_ratio
        x["merged_width_ratio"] = width_ratio
        x["usable_for_selection"] = not is_merged

    return instances


def group_rows_by_depth(
    instances: List[Dict[str, Any]],
    depth_axis_size_mm: float
) -> List[List[Dict[str, Any]]]:
    """Sorts instances by depth_inward_mm and partitions into rows by gap threshold.

    Row gap threshold = 0.60 * depth_axis_size_mm.
    """
    threshold = float(ROW_GAP_RATIO * depth_axis_size_mm)
    ordered = sorted(
        [x for x in instances if x.get("depth_inward_mm") is not None and np.isfinite(x["depth_inward_mm"])],
        key=lambda item: float(item["depth_inward_mm"])
    )

    rows: List[List[Dict[str, Any]]] = []
    current: List[Dict[str, Any]] = []

    for item in ordered:
        if not current:
            current = [item]
            continue

        gap = float(item["depth_inward_mm"]) - float(current[-1]["depth_inward_mm"])
        if gap <= threshold:
            current.append(item)
        else:
            rows.append(current)
            current = [item]

    if current:
        rows.append(current)

    return rows


def detect_boundary_touching(
    instances: List[Dict[str, Any]],
    box_roi_xyxy: Optional[List[float]],
    img_shape: Tuple[int, ...]
) -> Tuple[int, Tuple[int, int], Tuple[int, int]]:
    """Calculates dynamic boundary band and marks boundary touching flags on instances."""
    valid_items = [x for x in instances if x.get("bbox_width_px", 0) > 0]
    median_width = float(np.median([x["bbox_width_px"] for x in valid_items])) if valid_items else 50.0
    boundary_band_px = max(3, int(round(BOUNDARY_RATIO * median_width)))

    h, w = img_shape[:2]
    if box_roi_xyxy is not None and len(box_roi_xyxy) == 4:
        bx1, by1, bx2, by2 = map(int, box_roi_xyxy)
    else:
        bx1, by1, bx2, by2 = 0, 0, w, h

    left_band = (bx1, bx1 + boundary_band_px)
    right_band = (bx2 - boundary_band_px, bx2)

    for item in instances:
        mask = item.get("_mask")
        bbox = item.get("bbox", [0, 0, 0, 0])
        x, y, bw, bh = map(float, bbox)

        touch_left = False
        touch_right = False
        left_reason = None
        right_reason = None

        if mask is not None and mask.sum() > 0:
            item_cols = np.where(mask.any(axis=0))[0]
            if len(item_cols) > 0:
                min_c, max_c = item_cols[0], item_cols[-1]

                # Left boundary check
                gap_left = abs(min_c - bx1)
                left_band_mask = np.zeros_like(mask, dtype=bool)
                left_band_mask[:, max(0, left_band[0]):min(w, left_band[1])] = True
                contact_left = np.count_nonzero(mask & left_band_mask) / max(1, mask.sum())

                if gap_left <= boundary_band_px and contact_left >= 0.05:
                    touch_left = True
                else:
                    if contact_left < 0.05:
                        left_reason = "contact ratio below 0.05"
                    else:
                        left_reason = f"gap {gap_left}px exceeds boundary band {boundary_band_px}px"

                # Right boundary check
                gap_right = abs(bx2 - max_c)
                right_band_mask = np.zeros_like(mask, dtype=bool)
                right_band_mask[:, max(0, right_band[0]):min(w, right_band[1])] = True
                contact_right = np.count_nonzero(mask & right_band_mask) / max(1, mask.sum())

                if gap_right <= boundary_band_px and contact_right >= 0.05:
                    touch_right = True
                else:
                    if contact_right < 0.05:
                        right_reason = "contact ratio below 0.05"
                    else:
                        right_reason = f"gap {gap_right}px exceeds boundary band {boundary_band_px}px"

        item["touch_left"] = touch_left
        item["touch_right"] = touch_right
        item["left_touch_reason"] = left_reason
        item["right_touch_reason"] = right_reason

    return boundary_band_px, left_band, right_band


def select_front_row_targets(
    instances: List[Dict[str, Any]],
    box_frame: Any,
    box_mask: Optional[np.ndarray],
    sku_geometry: Dict[str, float],
    box_roi_xyxy: Optional[List[float]] = None,
    img_shape: Optional[Tuple[int, ...]] = None
) -> Dict[str, Any]:
    """Selects front-row center, left, and right targets with strict deduplication.

    Returns:
        {
            "front_row": List[Dict],
            "center_item": Optional[Dict],
            "left_item": Optional[Dict],
            "right_item": Optional[Dict],
            "audit": Dict[str, Any],
        }
    """
    D_inward = float(sku_geometry.get("depth_axis_size_mm", 80.0))
    row_gap_threshold_mm = float(ROW_GAP_RATIO * D_inward)

    # 1. Detect merged oversized instances
    instances = detect_merged_instances(instances, min_count=3, area_ratio_thresh=MERGED_AREA_RATIO, width_ratio_thresh=MERGED_WIDTH_RATIO)

    # 2. Boundary band detection
    if img_shape is None:
        first_mask = next((x["_mask"] for x in instances if "_mask" in x), None)
        img_shape = first_mask.shape if first_mask is not None else (480, 640)
    boundary_band_px, _, _ = detect_boundary_touching(instances, box_roi_xyxy, img_shape)

    # 3. Depth grouping
    groupable = [x for x in instances if x.get("valid_depth") and not x.get("merged_suspect")]
    rows = group_rows_by_depth(groupable, D_inward)
    front_row = rows[0] if rows else []
    front_row_ids = {x["instance_id"] for x in front_row}
    front_row_ref_depth_mm = float(np.median([x["depth_inward_mm"] for x in front_row])) if front_row else None

    # 4. Mark row labels
    for x in instances:
        if x.get("merged_suspect"):
            x["row_label"] = "merged_suspect"
        elif x["instance_id"] in front_row_ids:
            x["row_label"] = "front"
        else:
            x["row_label"] = "rear"
            x["usable_for_selection"] = False

    # 5. Candidate selection within front row
    usable_front = [x for x in front_row if x.get("usable_for_selection", True)]

    # Center candidates: not touching left or right boundary
    center_cands = [x for x in usable_front if not x.get("touch_left") and not x.get("touch_right")]
    if not center_cands:
        # Fallback to any usable front item if all touch boundaries
        center_cands = list(usable_front)

    center_item = None
    if center_cands:
        target_center_lateral = getattr(box_frame, "center_lateral_mm", 0.0)
        center_item = min(
            center_cands,
            key=lambda item: (
                abs(float(item.get("lateral_mm", 0.0)) - target_center_lateral),
                -float(item.get("score", 0.0)),
                int(item.get("instance_id", 0))
            )
        )

    # Left candidate: touching left boundary, deduplicated with center
    left_cands = [x for x in usable_front if x.get("touch_left") and (center_item is None or x["instance_id"] != center_item["instance_id"])]
    left_item = None
    if left_cands:
        left_item = min(
            left_cands,
            key=lambda item: (
                float(item.get("lateral_mm", 0.0)),
                -float(item.get("score", 0.0)),
                int(item.get("instance_id", 0))
            )
        )

    # Right candidate: touching right boundary, deduplicated with center and left
    excluded_ids = {x["instance_id"] for x in (center_item, left_item) if x is not None}
    right_cands = [x for x in usable_front if x.get("touch_right") and x["instance_id"] not in excluded_ids]
    right_item = None
    if right_cands:
        right_item = max(
            right_cands,
            key=lambda item: (
                float(item.get("lateral_mm", 0.0)),
                float(item.get("score", 0.0)),
                -int(item.get("instance_id", 0))
            )
        )

    # Build audit information
    audit = {
        "strategy": "front_row_center_left_right",
        "sku_geometry": sku_geometry,
        "depth_axis_size_mm": D_inward,
        "row_gap_threshold_mm": row_gap_threshold_mm,
        "boundary_band_px": boundary_band_px,
        "front_row_reference_depth_mm": front_row_ref_depth_mm,
        "front_row_instance_ids": [x["instance_id"] for x in front_row],
        "rear_instance_ids": [x["instance_id"] for x in instances if x.get("row_label") == "rear"],
        "merged_suspect_instance_ids": [x["instance_id"] for x in instances if x.get("merged_suspect")],
        "invalid_depth_instance_ids": [x["instance_id"] for x in instances if not x.get("valid_depth")],
        "selected_target_ids": {
            "center": center_item["instance_id"] if center_item else None,
            "left": left_item["instance_id"] if left_item else None,
            "right": right_item["instance_id"] if right_item else None,
        }
    }

    return {
        "front_row": front_row,
        "center_item": center_item,
        "left_item": left_item,
        "right_item": right_item,
        "audit": audit,
        "boundary_band_px": boundary_band_px,
    }


def save_all_instances_diagnostic_overlay(
    rgb: np.ndarray,
    instances: List[Dict[str, Any]],
    box_roi_xyxy: Optional[List[float]],
    boundary_band_px: int,
    center_item: Optional[Dict[str, Any]],
    left_item: Optional[Dict[str, Any]],
    right_item: Optional[Dict[str, Any]],
    path: Optional[Union[str, Path]] = None
) -> np.ndarray:
    """Draws box boundaries, centerline, depth annotations, labels, and color-coded masks."""
    overlay = rgb.copy()
    h, w = overlay.shape[:2]

    if box_roi_xyxy is not None and len(box_roi_xyxy) == 4:
        bx1, by1, bx2, by2 = map(int, box_roi_xyxy)
        bcx = int(round((bx1 + bx2) / 2.0))

        # Box left boundary and boundary band
        cv2.line(overlay, (bx1, by1), (bx1, by2), (255, 255, 0), 2)
        cv2.line(overlay, (bx1 + boundary_band_px, by1), (bx1 + boundary_band_px, by2), (255, 255, 0), 1, cv2.LINE_AA)

        # Box right boundary and boundary band
        cv2.line(overlay, (bx2, by1), (bx2, by2), (255, 255, 0), 2)
        cv2.line(overlay, (bx2 - boundary_band_px, by1), (bx2 - boundary_band_px, by2), (255, 255, 0), 1, cv2.LINE_AA)

        # Box centerline
        cv2.line(overlay, (bcx, by1), (bcx, by2), (0, 255, 255), 2)
        cv2.putText(overlay, 'box_center', (bcx - 30, max(20, by1 - 8)), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 255, 255), 1, cv2.LINE_AA)

    center_id = center_item["instance_id"] if center_item else None
    left_id = left_item["instance_id"] if left_item else None
    right_id = right_item["instance_id"] if right_item else None

    COLOR_CENTER = (0, 255, 0)      # Green
    COLOR_LEFT   = (255, 120, 0)    # Blue/Cyan
    COLOR_RIGHT  = (0, 0, 255)      # Red
    COLOR_MERGED = (0, 165, 255)    # Orange/Amber
    COLOR_GRAY   = (128, 128, 128)  # Gray

    for item in instances:
        iid = item.get("instance_id")
        mask = item.get("_mask")
        if mask is None:
            continue
        depth_val = item.get("depth_inward_mm")
        depth_str = f"{depth_val:.0f}" if depth_val is not None and np.isfinite(depth_val) else "N/A"

        if iid == center_id:
            color = COLOR_CENTER
            label = f"id={iid} depth={depth_str} center"
        elif iid == left_id:
            color = COLOR_LEFT
            label = f"id={iid} depth={depth_str} left"
        elif iid == right_id:
            color = COLOR_RIGHT
            label = f"id={iid} depth={depth_str} right"
        elif item.get("merged_suspect"):
            color = COLOR_MERGED
            label = f"id={iid} merged_suspect"
        elif item.get("row_label") == "rear":
            color = COLOR_GRAY
            label = f"id={iid} depth={depth_str} rear"
        else:
            color = COLOR_GRAY
            label = f"id={iid} depth={depth_str} front"

        color_arr = np.array(color, dtype=np.uint8)
        overlay[mask] = (0.60 * overlay[mask] + 0.40 * color_arr).astype(np.uint8)

        bbox = item.get("bbox", [0, 0, 0, 0])
        x, y, bw, bh = map(int, bbox)
        tx = max(5, x)
        ty = max(18, y - 4)

        cv2.putText(overlay, label, (tx, ty), cv2.FONT_HERSHEY_SIMPLEX, 0.48, (0, 0, 0), 3, cv2.LINE_AA)
        cv2.putText(overlay, label, (tx, ty), cv2.FONT_HERSHEY_SIMPLEX, 0.48, color, 1, cv2.LINE_AA)

    if path is not None:
        p = Path(path)
        p.parent.mkdir(parents=True, exist_ok=True)
        cv2.imwrite(str(p), overlay)

    return overlay
