"""Schemas for QR / barcode parse APIs."""

from __future__ import annotations

from typing import Literal

from pydantic import BaseModel, Field, field_validator

TargetType = Literal["sku", "carton"]


class ParseQrCodeRequest(BaseModel):
    target_type: TargetType
    image_path: str
    bbox: list[int] = Field(min_length=4, max_length=4)
    mask: str

    @field_validator("bbox")
    @classmethod
    def validate_bbox(cls, value: list[int]) -> list[int]:
        x1, y1, x2, y2 = value
        if x2 <= x1 or y2 <= y1:
            raise ValueError("bbox 必须满足 x2 > x1 且 y2 > y1")
        return value


class ParseQrCodeResponse(BaseModel):
    content: str
