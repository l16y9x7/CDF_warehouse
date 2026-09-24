"""Schemas for SKU barcode recognition APIs."""

from __future__ import annotations

from typing import Literal

from pydantic import BaseModel

from models.request_base import ImageSourceRequest, Sam3Options

RecognizeStatus = Literal["FOUND", "NOT_FOUND"]


class RecognizeSkuBarcodeRequest(ImageSourceRequest, Sam3Options):
    sku_id: str
    name: str


class RecognizeSkuBarcodeResponse(BaseModel):
    status: RecognizeStatus
    barcode_content: str | None = None
