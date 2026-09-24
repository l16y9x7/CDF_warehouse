"""HTTP routes for QR / barcode parsing from a known region."""

from __future__ import annotations

from fastapi import APIRouter, HTTPException

from core.image_io import load_image_from_path
from models.qr_parse import ParseQrCodeRequest, ParseQrCodeResponse
from services.qr_decode import parse_qr_code

router = APIRouter()


@router.post("/perception/parse_qr_code", response_model=ParseQrCodeResponse)
def parse_qr_code_api(request: ParseQrCodeRequest) -> ParseQrCodeResponse:
    image_bgr = load_image_from_path(request.image_path)
    try:
        content = parse_qr_code(
            image_bgr,
            target_type=request.target_type,
            bbox=request.bbox,
            mask_b64=request.mask,
        )
    except ValueError as error:
        raise HTTPException(status_code=400, detail=str(error)) from error

    if not content:
        raise HTTPException(status_code=404, detail="未识别到二维码/条码内容")

    return ParseQrCodeResponse(content=content)
