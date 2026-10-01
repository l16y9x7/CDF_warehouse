"""HTTP routes for end-to-end SKU barcode recognition."""

from __future__ import annotations

import time

import requests
from fastapi import APIRouter, HTTPException, Request

from core.request_archive import ArchivedRoute
from core.recognize_trace import RecognizePipelineTrace
from core.sam3_prompts import RECOGNIZE_SKU_BARCODE, resolve_sam3_prompt
from models.sku_recognize import RecognizeSkuBarcodeRequest, RecognizeSkuBarcodeResponse
from services.sku_recognize import recognize_sku_barcode
from services.barcode_compare import run_barcode_comparisons

router = APIRouter(route_class=ArchivedRoute)


@router.post("/perception/recognize_sku_barcode", response_model=RecognizeSkuBarcodeResponse)
def recognize_sku_barcode_api(
    request: RecognizeSkuBarcodeRequest,
    http_request: Request,
) -> RecognizeSkuBarcodeResponse:
    archive = http_request.state.archive
    timings_ms: dict[str, float] = {}
    uses_path = bool(request.image_path and request.image_path.strip())
    trace = RecognizePipelineTrace(
        request_id=archive.request_id,
        sku_id=request.sku_id,
        name=request.name,
        image_path=request.image_path.strip() if uses_path else "",
        image_source="path" if uses_path else "base64",
        sam3_threshold=request.sam3_threshold,
    )
    trace.timings_ms = timings_ms
    archive.trace = trace

    read_started = time.perf_counter()
    image_bgr = archive.load_image()
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
        trace.failure_reason = "sam3_request_failed"
        raise HTTPException(status_code=502, detail=f"SAM3 调用失败: {error}") from error
    except ValueError as error:
        trace.failure_reason = "invalid_pipeline_state"
        raise HTTPException(status_code=502, detail=str(error)) from error

    trace.barcode_content = barcode_content
    run_barcode_comparisons(image_bgr, trace)

    if not barcode_content:
        return RecognizeSkuBarcodeResponse(status="NOT_FOUND")

    return RecognizeSkuBarcodeResponse(status="FOUND", barcode_content=barcode_content)
