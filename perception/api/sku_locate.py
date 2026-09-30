"""HTTP routes for SKU QR/barcode localization."""

from __future__ import annotations

import time

import requests
from fastapi import APIRouter, HTTPException, Request

from core.recognize_trace import RecognizePipelineTrace
from core.request_archive import ArchivedRoute
from core.sam3_prompts import RECOGNIZE_SKU_BARCODE, resolve_sam3_prompt
from models.sku_locate import LocateSkuQrCodeRequest, SamLocateResponse
from services.sku_locate import locate_sku_qr_code

router = APIRouter(route_class=ArchivedRoute)


@router.post("/perception/locate_sku_qr_code", response_model=SamLocateResponse)
def locate_sku_qr_code_api(request: LocateSkuQrCodeRequest, http_request: Request) -> SamLocateResponse:
    archive = http_request.state.archive
    timings_ms: dict[str, float] = {}
    trace = RecognizePipelineTrace(
        request_id=archive.request_id, operation="locate_sku_qr_code",
        image_path=(request.image_path or "").strip(),
        image_source="path" if (request.image_path or "").strip() else "base64",
        sam3_threshold=request.sam3_threshold, timings_ms=timings_ms,
    )
    archive.trace = trace
    read_started = time.perf_counter()
    image_bgr = archive.load_image()
    timings_ms["read_image"] = round((time.perf_counter() - read_started) * 1000, 1)

    sam3_prompt = resolve_sam3_prompt(RECOGNIZE_SKU_BARCODE, request.sam3_prompt)
    trace.sam3_prompt = sam3_prompt
    try:
        return locate_sku_qr_code(
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
