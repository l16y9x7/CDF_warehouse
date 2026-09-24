"""HTTP routes for SKU QR/barcode localization."""

from __future__ import annotations

import time

import requests
from fastapi import APIRouter, HTTPException

from core.image_io import load_request_image
from core.sam3_prompts import RECOGNIZE_SKU_BARCODE, resolve_sam3_prompt
from models.sku_locate import LocateSkuQrCodeRequest, SamLocateResponse
from services.sku_locate import locate_sku_qr_code

router = APIRouter()


@router.post("/perception/locate_sku_qr_code", response_model=SamLocateResponse)
def locate_sku_qr_code_api(request: LocateSkuQrCodeRequest) -> SamLocateResponse:
    timings_ms: dict[str, float] = {}
    read_started = time.perf_counter()
    image_bgr = load_request_image(
        image_path=request.image_path,
        image_base64=request.image_base64,
    )
    timings_ms["read_image"] = round((time.perf_counter() - read_started) * 1000, 1)

    sam3_prompt = resolve_sam3_prompt(RECOGNIZE_SKU_BARCODE, request.sam3_prompt)
    try:
        return locate_sku_qr_code(
            image_bgr,
            sam3_prompt=sam3_prompt,
            sam3_threshold=request.sam3_threshold,
            timings_ms=timings_ms,
        )
    except requests.RequestException as error:
        raise HTTPException(status_code=502, detail=f"SAM3 调用失败: {error}") from error
    except ValueError as error:
        raise HTTPException(status_code=502, detail=str(error)) from error
