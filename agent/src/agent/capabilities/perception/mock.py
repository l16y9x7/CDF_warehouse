from agent.capabilities.common import BoundingBox, CapabilityError, HealthResult, HealthStatus
from agent.capabilities.mocks.delay import pause_mock_processing

from .contract import (
    BarcodeResult,
    ImageRequest,
    LocateResult,
    LocateStatus,
    RecognizeBarcodeRequest,
)


LOCATE_FOUND_BBOX = (100, 200, 200, 400)


def locate_found_payload(name: str = "basket-item") -> dict[str, object]:
    return {
        "status": "FOUND",
        "bbox": list(LOCATE_FOUND_BBOX),
        "mask": f"mock-mask:{name}",
    }


def default_locate_status(image_path: str) -> LocateStatus:
    if "empty" in image_path:
        return LocateStatus.NOT_FOUND
    return LocateStatus.FOUND


class MockPerceptionCapability:
    def __init__(self) -> None:
        self.barcode_content = "sku-1"
        self.basket_item_statuses: list[LocateStatus] = []

    def health(self) -> HealthResult:
        pause_mock_processing()
        return HealthResult(HealthStatus.READY)

    def recognize_sku_barcode(self, request: RecognizeBarcodeRequest) -> BarcodeResult:
        pause_mock_processing()
        if not self.barcode_content:
            raise CapabilityError("SKU_BARCODE_NOT_FOUND", "SKU barcode was not found")
        return BarcodeResult(self.barcode_content)

    def locate_basket_item(self, request: ImageRequest) -> LocateResult:
        pause_mock_processing()
        if self.basket_item_statuses:
            status = self.basket_item_statuses.pop(0)
        else:
            status = default_locate_status(request.image_path)
        return (
            LocateResult(status) if status is LocateStatus.NOT_FOUND else self._found("basket-item")
        )

    @staticmethod
    def _found(name: str) -> LocateResult:
        return LocateResult(
            LocateStatus.FOUND,
            BoundingBox(*LOCATE_FOUND_BBOX),
            f"mock-mask:{name}",
        )
