"""End-to-end SKU barcode recognition orchestration."""

from __future__ import annotations

import time

import numpy as np

from config import (
    RECOGNIZE_SKU_BARCODE_BBOX_PADDING_RATIO,
    RECOGNIZE_SKU_BARCODE_CENTER_DISTANCE_MAX,
)
from core.recognize_trace import RecognizePipelineTrace
from services.qr_decode import decode_barcode_from_bbox
from services.sku_locate import bbox_center_distance, locate_sku_with_center_filter

def _find_candidate_index(instances: list[object], bbox: list[int]) -> int | None:
    for index, instance in enumerate(instances, start=1):
        if instance.bbox == bbox:
            return index
    return None


def recognize_sku_barcode(
    image_bgr: np.ndarray,
    *,
    sam3_prompt: str,
    sam3_threshold: float = 0.5,
    center_distance_max: float | None = None,
    timings_ms: dict[str, float] | None = None,
    trace: RecognizePipelineTrace | None = None,
) -> str | None:
    distance_max = (
        RECOGNIZE_SKU_BARCODE_CENTER_DISTANCE_MAX
        if center_distance_max is None
        else center_distance_max
    )
    height, width = image_bgr.shape[:2]
    if trace is not None:
        trace.image_shape = (width, height)
        trace.sam3_prompt = sam3_prompt
        trace.sam3_threshold = sam3_threshold
        trace.center_distance_max = distance_max
        trace.bbox_padding_ratio = RECOGNIZE_SKU_BARCODE_BBOX_PADDING_RATIO

    context = locate_sku_with_center_filter(
        image_bgr,
        sam3_prompt=sam3_prompt,
        sam3_threshold=sam3_threshold,
        center_distance_max=distance_max,
        timings_ms=timings_ms,
    )
    if trace is not None:
        trace.record_sam3_candidates(
            context.instances,
            image_bgr.shape,
            center_distance_max=distance_max,
            center_distance_fn=bbox_center_distance,
        )
        trace.filtered_count = context.filtered_count

    if context.selected is None:
        if trace is not None:
            trace.failure_reason = context.failure_reason
        return None

    result = context.selected
    selected_center_distance = bbox_center_distance(result.bbox, image_bgr.shape)
    if trace is not None:
        trace.selected_index = _find_candidate_index(context.instances, result.bbox)
        trace.selected_score = result.score
        trace.selected_bbox = list(result.bbox)
        trace.selected_center_dist = selected_center_distance

    decode_started = time.perf_counter()
    try:
        content = decode_barcode_from_bbox(
            image_bgr,
            result.bbox,
            padding_ratio=RECOGNIZE_SKU_BARCODE_BBOX_PADDING_RATIO,
            trace=trace,
        )
    except ValueError as error:
        content = None
        if trace is not None:
            trace.decode_error = str(error)
            trace.failure_reason = "decode_error"
    else:
        if content is None and trace is not None:
            trace.failure_reason = "decode_failed"

    if timings_ms is not None:
        timings_ms["decode"] = round((time.perf_counter() - decode_started) * 1000, 1)
    return content
