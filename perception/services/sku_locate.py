"""SAM3-based SKU localization with center-distance filtering."""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from clients.sam3_client import SamInstance, SamLocateConfig, SamLocateResult, locate_sam3_instances
from config import RECOGNIZE_SKU_BARCODE_CENTER_DISTANCE_MAX
from models.sku_locate import SamLocateResponse, build_locate_response

SKU_LOCATE_TOP_K = 16


def bbox_center_distance(bbox: list[int], image_shape: tuple[int, ...]) -> float:
    height, width = image_shape[:2]
    x1, y1, x2, y2 = bbox
    center_x = ((x1 + x2) / 2.0) / max(1, width)
    center_y = ((y1 + y2) / 2.0) / max(1, height)
    return abs(center_x - 0.5) + abs(center_y - 0.5)


def instance_center_rank(instance: SamInstance, image_shape: tuple[int, ...]) -> float:
    return -bbox_center_distance(instance.bbox_xyxy, image_shape)


def select_best_center_candidate(
    instances: list[SamLocateResult],
    image_shape: tuple[int, ...],
    *,
    center_distance_max: float,
) -> SamLocateResult | None:
    filtered = [
        instance
        for instance in instances
        if bbox_center_distance(instance.bbox, image_shape) <= center_distance_max
    ]
    if not filtered:
        return None
    return min(
        filtered,
        key=lambda item: bbox_center_distance(item.bbox, image_shape),
    )


@dataclass(frozen=True)
class SkuLocateContext:
    instances: list[SamLocateResult]
    selected: SamLocateResult | None
    center_distance_max: float
    filtered_count: int
    failure_reason: str | None = None


def locate_sku_with_center_filter(
    image_bgr: np.ndarray,
    *,
    sam3_prompt: str,
    sam3_threshold: float = 0.5,
    center_distance_max: float,
    top_k: int = SKU_LOCATE_TOP_K,
    timings_ms: dict[str, float] | None = None,
    timing_key: str = "sam3",
) -> SkuLocateContext:
    config = SamLocateConfig(
        sam3_prompt=sam3_prompt,
        sam3_threshold=sam3_threshold,
    )
    instances = locate_sam3_instances(
        image_bgr,
        config=config,
        top_k=top_k,
        instance_ranker=lambda instance: instance_center_rank(instance, image_bgr.shape),
        timings_ms=timings_ms,
        timing_key=timing_key,
    )
    if not instances:
        return SkuLocateContext(
            instances=[],
            selected=None,
            center_distance_max=center_distance_max,
            filtered_count=0,
            failure_reason="sam3_empty",
        )

    filtered_count = sum(
        1
        for instance in instances
        if bbox_center_distance(instance.bbox, image_bgr.shape) <= center_distance_max
    )
    selected = select_best_center_candidate(
        instances,
        image_bgr.shape,
        center_distance_max=center_distance_max,
    )
    if selected is None:
        return SkuLocateContext(
            instances=instances,
            selected=None,
            center_distance_max=center_distance_max,
            filtered_count=filtered_count,
            failure_reason="center_filter_empty",
        )
    return SkuLocateContext(
        instances=instances,
        selected=selected,
        center_distance_max=center_distance_max,
        filtered_count=filtered_count,
    )


def locate_sku_qr_code(
    image_bgr: np.ndarray,
    *,
    sam3_prompt: str,
    sam3_threshold: float = 0.5,
    center_distance_max: float | None = None,
    timings_ms: dict[str, float] | None = None,
) -> SamLocateResponse:
    distance_max = (
        RECOGNIZE_SKU_BARCODE_CENTER_DISTANCE_MAX
        if center_distance_max is None
        else center_distance_max
    )
    context = locate_sku_with_center_filter(
        image_bgr,
        sam3_prompt=sam3_prompt,
        sam3_threshold=sam3_threshold,
        center_distance_max=distance_max,
        timings_ms=timings_ms,
    )
    if context.selected is None:
        return build_locate_response(bbox=None, mask=None)
    return build_locate_response(
        bbox=context.selected.bbox,
        mask=context.selected.mask,
    )
