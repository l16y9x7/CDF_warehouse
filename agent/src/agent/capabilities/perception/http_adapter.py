from agent.capabilities.common import BoundingBox, CapabilityError, HealthResult, HealthStatus
from agent.capabilities.http import HttpCapabilityClient

from .contract import (
    BarcodeResult,
    ImageRequest,
    LocateResult,
    LocateStatus,
    RecognizeBarcodeRequest,
)


class HttpPerceptionCapability:
    def __init__(self, client: HttpCapabilityClient):
        self.client = client

    def health(self) -> HealthResult:
        return HealthResult(HealthStatus(self.client.get("/perception/health")["status"]))

    def recognize_sku_barcode(self, request: RecognizeBarcodeRequest) -> BarcodeResult:
        body = {
            "image_base64": request.image_base64,
            "sku_id": request.sku_id or "",
            "name": request.name or "",
        }
        payload = self.client.post("/perception/recognize_sku_barcode", body)
        if payload.get("status") == "NOT_FOUND":
            raise CapabilityError("SKU_BARCODE_NOT_FOUND", "SKU barcode was not found")
        content = payload.get("barcode_content")
        if not isinstance(content, str) or not content:
            raise ValueError("barcode response must include non-empty barcode_content")
        return BarcodeResult(content)

    def locate_basket_item(self, request: ImageRequest) -> LocateResult:
        return _locate(
            self.client.post("/perception/basket/locate_item", {"image_path": request.image_path})
        )


def _locate(payload: dict) -> LocateResult:
    status = LocateStatus(payload["status"])
    if status is LocateStatus.NOT_FOUND:
        return LocateResult(status)
    return LocateResult(status, BoundingBox.from_value(payload["bbox"]), payload["mask"])
