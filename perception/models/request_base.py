"""Shared request field groups."""

from __future__ import annotations

from pydantic import BaseModel, Field, model_validator


class ImageSourceRequest(BaseModel):
    """Exactly one of image_path or image_base64 must be provided."""

    image_path: str | None = None
    image_base64: str | None = None

    @model_validator(mode="after")
    def validate_image_source(self) -> ImageSourceRequest:
        has_path = bool(self.image_path and self.image_path.strip())
        has_base64 = bool(self.image_base64 and self.image_base64.strip())
        if has_path == has_base64:
            raise ValueError("image_path 与 image_base64 必须且只能提供一个")
        return self


class Sam3Options(BaseModel):
    sam3_prompt: str | None = None
    sam3_threshold: float = Field(default=0.5, ge=0.0, le=1.0)
