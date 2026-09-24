"""Schemas for SKU locate APIs."""

from __future__ import annotations

from typing import Literal

from pydantic import BaseModel

from models.request_base import ImageSourceRequest, Sam3Options

LocateStatus = Literal["FOUND", "NOT_FOUND"]


class SamLocateResponse(BaseModel):
    status: LocateStatus
    bbox: list[int] | None = None
    mask: str | None = None


class LocateSkuQrCodeRequest(ImageSourceRequest, Sam3Options):
    pass


def build_locate_response(*, bbox: list[int] | None, mask: str | None) -> SamLocateResponse:
    if bbox is None or mask is None:
        return SamLocateResponse(status="NOT_FOUND")
    return SamLocateResponse(status="FOUND", bbox=bbox, mask=mask)
