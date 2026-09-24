"""End-to-end SKU barcode recognition orchestration."""

from __future__ import annotations

import logging
import re
import time
from datetime import datetime
from pathlib import Path

import cv2
import numpy as np

from config import (
    RECOGNIZE_SKU_BARCODE_BBOX_PADDING_RATIO,
    RECOGNIZE_SKU_BARCODE_CENTER_DISTANCE_MAX,
    RECOGNIZE_SKU_BARCODE_LOG_PATH,
)
from core.recognize_trace import RecognizePipelineTrace
from services.qr_decode import decode_barcode_from_bbox
from services.sku_locate import bbox_center_distance, locate_sku_with_center_filter

_LOGGER_NAME = "perception.recognize_sku_barcode"
_logger_configured = False


def get_logger() -> logging.Logger:
    global _logger_configured
    logger = logging.getLogger(_LOGGER_NAME)
    if _logger_configured:
        return logger

    logger.setLevel(logging.INFO)
    logger.propagate = False
    formatter = logging.Formatter(
        "%(asctime)s %(levelname)s [%(name)s] %(message)s",
        datefmt="%Y-%m-%d %H:%M:%S",
    )

    log_path = Path(RECOGNIZE_SKU_BARCODE_LOG_PATH)
    log_path.parent.mkdir(parents=True, exist_ok=True)
    file_handler = logging.FileHandler(log_path, encoding="utf-8")
    file_handler.setFormatter(formatter)
    logger.addHandler(file_handler)

    stream_handler = logging.StreamHandler()
    stream_handler.setFormatter(formatter)
    logger.addHandler(stream_handler)

    _logger_configured = True
    logger.info("recognize_sku_barcode logger initialized log_path=%s", log_path)
    return logger


def _find_candidate_index(instances: list[object], bbox: list[int]) -> int | None:
    for index, instance in enumerate(instances, start=1):
        if instance.bbox == bbox:
            return index
    return None


def log_pipeline_summary(logger: logging.Logger, trace: RecognizePipelineTrace) -> None:
    logger.info("\n%s", trace.format_summary())


def _not_found_image_dir() -> Path:
    return Path(RECOGNIZE_SKU_BARCODE_LOG_PATH).parent


def _safe_filename_part(value: str, *, max_len: int = 48) -> str:
    cleaned = re.sub(r"[^0-9A-Za-z._-]+", "_", value.strip())
    cleaned = cleaned.strip("._-")
    if not cleaned:
        return "unknown"
    return cleaned[:max_len]


def save_not_found_image(
    image_bgr: np.ndarray,
    *,
    sku_id: str,
    failure_reason: str | None,
    logger: logging.Logger,
) -> Path | None:
    output_dir = _not_found_image_dir()
    output_dir.mkdir(parents=True, exist_ok=True)
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S_%f")
    filename = (
        f"not_found_{timestamp}_{_safe_filename_part(sku_id)}"
        f"_{_safe_filename_part(failure_reason or 'not_found')}.jpg"
    )
    output_path = output_dir / filename
    if not cv2.imwrite(str(output_path), image_bgr):
        logger.warning("step=save_not_found_image result=failed path=%s", output_path)
        return None

    logger.info("step=save_not_found_image result=ok path=%s", output_path)
    return output_path


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
