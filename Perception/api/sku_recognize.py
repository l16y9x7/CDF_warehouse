"""HTTP routes for end-to-end SKU barcode recognition."""

from __future__ import annotations

import time

import requests
from fastapi import APIRouter, HTTPException

from core.image_io import load_request_image
from core.recognize_trace import RecognizePipelineTrace
from core.sam3_prompts import RECOGNIZE_SKU_BARCODE, resolve_sam3_prompt
from models.sku_recognize import RecognizeSkuBarcodeRequest, RecognizeSkuBarcodeResponse
from services.sku_recognize import (
    get_logger,
    log_pipeline_summary,
    recognize_sku_barcode,
    save_not_found_image,
)

router = APIRouter()


@router.post("/perception/recognize_sku_barcode", response_model=RecognizeSkuBarcodeResponse)
def recognize_sku_barcode_api(
    request: RecognizeSkuBarcodeRequest,
) -> RecognizeSkuBarcodeResponse:
    logger = get_logger()
    request_started = time.perf_counter()
    timings_ms: dict[str, float] = {}
    uses_path = bool(request.image_path and request.image_path.strip())
    trace = RecognizePipelineTrace(
        sku_id=request.sku_id,
        name=request.name,
        image_path=request.image_path.strip() if uses_path else "",
        image_source="path" if uses_path else "base64",
        sam3_threshold=request.sam3_threshold,
    )

    read_started = time.perf_counter()
    image_bgr = load_request_image(
        image_path=request.image_path,
        image_base64=request.image_base64,
    )
    timings_ms["read_image"] = round((time.perf_counter() - read_started) * 1000, 1)

    sam3_prompt = resolve_sam3_prompt(RECOGNIZE_SKU_BARCODE, request.sam3_prompt)
    trace.sam3_prompt = sam3_prompt

    try:
        barcode_content = recognize_sku_barcode(
            image_bgr,
            sam3_prompt=sam3_prompt,
            sam3_threshold=request.sam3_threshold,
            timings_ms=timings_ms,
            trace=trace,
        )
    except requests.RequestException as error:
        trace.status = "ERROR"
        trace.failure_reason = "sam3_request_failed"
        trace.timings_ms = timings_ms
        trace.total_ms = round((time.perf_counter() - request_started) * 1000, 1)
        log_pipeline_summary(logger, trace)
        logger.exception("step=recognize result=error reason=sam3_request_failed detail=%s", error)
        raise HTTPException(status_code=502, detail=f"SAM3 调用失败: {error}") from error
    except ValueError as error:
        trace.status = "ERROR"
        trace.failure_reason = "invalid_pipeline_state"
        trace.timings_ms = timings_ms
        trace.total_ms = round((time.perf_counter() - request_started) * 1000, 1)
        log_pipeline_summary(logger, trace)
        logger.exception("step=recognize result=error reason=invalid_pipeline_state detail=%s", error)
        raise HTTPException(status_code=502, detail=str(error)) from error

    trace.timings_ms = timings_ms
    trace.total_ms = round((time.perf_counter() - request_started) * 1000, 1)
    trace.barcode_content = barcode_content

    if not barcode_content:
        trace.status = "NOT_FOUND"
        saved_path = save_not_found_image(
            image_bgr,
            sku_id=request.sku_id,
            failure_reason=trace.failure_reason,
            logger=logger,
        )
        if saved_path is not None:
            trace.saved_image_path = str(saved_path)
        log_pipeline_summary(logger, trace)
        return RecognizeSkuBarcodeResponse(status="NOT_FOUND")

    trace.status = "FOUND"
    log_pipeline_summary(logger, trace)
    return RecognizeSkuBarcodeResponse(status="FOUND", barcode_content=barcode_content)
