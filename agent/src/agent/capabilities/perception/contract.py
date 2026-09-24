from dataclasses import dataclass
from enum import Enum
from typing import Protocol

from agent.capabilities.common import BoundingBox, Detection, HealthResult


@dataclass(frozen=True)
class ImageRequest:
    image_path: str


@dataclass(frozen=True)
class RecognizeBarcodeRequest:
    image_base64: str
    sku_id: str = ""
    name: str = ""

    def __post_init__(self) -> None:
        object.__setattr__(self, "sku_id", self.sku_id or "")
        object.__setattr__(self, "name", self.name or "")


@dataclass(frozen=True)
class BarcodeResult:
    barcode_content: str


class LocateStatus(str, Enum):
    FOUND = "FOUND"
    NOT_FOUND = "NOT_FOUND"


@dataclass(frozen=True)
class LocateResult:
    status: LocateStatus
    bbox: BoundingBox | None = None
    mask: str | None = None

    def __post_init__(self) -> None:
        if self.status is LocateStatus.FOUND and (self.bbox is None or not self.mask):
            raise ValueError("FOUND requires bbox and mask")
        if self.status is LocateStatus.NOT_FOUND and (
            self.bbox is not None or self.mask is not None
        ):
            raise ValueError("NOT_FOUND does not accept bbox or mask")

    def detection(self) -> Detection:
        if self.status is not LocateStatus.FOUND or self.bbox is None or self.mask is None:
            raise ValueError("target was not found")
        return Detection(self.bbox, self.mask)


class PerceptionCapability(Protocol):
    def health(self) -> HealthResult: ...
    def recognize_sku_barcode(self, request: RecognizeBarcodeRequest) -> BarcodeResult: ...
    def locate_basket_item(self, request: ImageRequest) -> LocateResult: ...
