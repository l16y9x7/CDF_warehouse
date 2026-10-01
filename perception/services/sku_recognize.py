"""End-to-end SKU barcode recognition orchestration."""

from __future__ import annotations

import asyncio
import time

import numpy as np
from starlette.concurrency import run_in_threadpool

from config import (
    RECOGNIZE_SKU_BARCODE_BBOX_PADDING_RATIO,
    RECOGNIZE_SKU_BARCODE_CENTER_DISTANCE_MAX,
)
from core.recognize_trace import RecognizePipelineTrace
from services.barcode_compare import run_barcode_comparisons, skipped_comparisons
from services.qr_decode import decode_barcode_from_bbox
from services.sku_locate import bbox_center_distance, locate_sku_with_center_filter

def _find_candidate_index(instances: list[object], bbox: list[int]) -> int | None:
    for index, instance in enumerate(instances, start=1):
        if instance.bbox == bbox:
            return index
    return None


async def recognize_sku_barcode(
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

    context = await run_in_threadpool(
        locate_sku_with_center_filter,
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
            await run_barcode_comparisons(image_bgr, trace)
        return None

    result = context.selected
    selected_center_distance = bbox_center_distance(result.bbox, image_bgr.shape)
    if trace is not None:
        trace.selected_index = _find_candidate_index(context.instances, result.bbox)
        trace.selected_score = result.score
        trace.selected_bbox = list(result.bbox)
        trace.selected_center_dist = selected_center_distance

    # The baseline and both comparisons share one SAM3 selection. Native CPU
    # decoders run on Starlette's reusable, bounded worker pool, not the loop.
    started = time.perf_counter()
    jobs = [run_in_threadpool(
        _decode_business_barcode, image_bgr, result.bbox,
        timings_ms=timings_ms, trace=trace,
    )]
    if trace is not None:
        jobs.append(run_barcode_comparisons(image_bgr, trace))
    # Wait for every method, even if the baseline raises, so archive.finish
    # never serializes a trace that is still being written by a decoder.
    results = await asyncio.gather(*jobs, return_exceptions=True)
    if timings_ms is not None:
        timings_ms["decode_parallel"] = round((time.perf_counter() - started) * 1000, 1)
    if trace is not None and isinstance(results[1], Exception):
        # A diagnostic orchestration error must not replace the business result.
        trace.decode_comparisons = skipped_comparisons("comparison_execution_failed")
        for comparison in trace.decode_comparisons.values():
            comparison.status = "ERROR"
            comparison.error = f"{type(results[1]).__name__}: {results[1]}"
    if isinstance(results[0], BaseException):
        raise results[0]
    content = results[0]
    if trace is not None:
        trace.barcode_content = content
        # Match only after joining all tasks; comparison workers never read a
        # baseline result or trace fields that the baseline is still updating.
        for comparison in trace.decode_comparisons.values():
            if content and comparison.status in {"FOUND", "NOT_FOUND"}:
                comparison.matches_business_result = any(
                    code["content"] == content for code in comparison.codes
                )
    return content


def _decode_business_barcode(
    image_bgr: np.ndarray,
    bbox: list[int],
    *,
    timings_ms: dict[str, float] | None,
    trace: RecognizePipelineTrace | None,
) -> str | None:
    """Keep the original decode and error behavior inside one worker thread."""
    decode_started = time.perf_counter()
    try:
        content = decode_barcode_from_bbox(
            image_bgr,
            bbox,
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
