from agent.capabilities.common import ActionResult, ActionStatus, Hand, HealthResult, HealthStatus
from agent.capabilities.mocks.delay import pause_mock_processing

from .contract import (
    BasketPickRequest,
    PickActionResult,
    PickRequest,
    PlaceRequest,
    RotateActionResult,
    PushRequest,
    ReviewItemPickRequest,
)


class MockManipulationCapability:
    def __init__(self) -> None:
        self.calls: list[str] = []
        self.hands: list[Hand] = []
        self.pick_requests: list[PickRequest] = []
        self.place_requests: list[PlaceRequest] = []
        self.push_requests: list[PushRequest] = []
        self.pick_basket_requests: list[BasketPickRequest] = []
        self.idempotency_keys: list[str | None] = []
        self.sku_typs: list[str] = []
        self.rotate_image_paths = tuple(
            f"/shared/frames/barcode-scan-{index}/rgb.jpg" for index in range(1, 6)
        )

    def health(self) -> HealthResult:
        pause_mock_processing()
        return HealthResult(HealthStatus.READY)

    def pick(
        self, request: PickRequest, *, idempotency_key: str | None = None
    ) -> PickActionResult:
        pause_mock_processing()
        self.hands.append(request.hand)
        self.pick_requests.append(request)
        self.idempotency_keys.append(idempotency_key)
        self.calls.append("pick")
        return PickActionResult(
            ActionStatus.SUCCEEDED,
            {"frame": "trunk_controller_ref", "unit": "mm"},
            6,
        )

    def pick_review_item(
        self, request: ReviewItemPickRequest, *, idempotency_key: str | None = None
    ) -> ActionResult:
        pause_mock_processing()
        self.hands.append(request.hand)
        return self._record("pick_review_item")

    def place(self, request: PlaceRequest, *, idempotency_key: str | None = None) -> ActionResult:
        pause_mock_processing()
        self.hands.append(request.hand)
        self.place_requests.append(request)
        self.idempotency_keys.append(idempotency_key)
        return self._record("place")

    def rotate(
        self, hand: Hand, sku_typ: str, *, idempotency_key: str | None = None
    ) -> RotateActionResult:
        pause_mock_processing()
        self.hands.append(hand)
        self.sku_typs.append(sku_typ)
        self.idempotency_keys.append(idempotency_key)
        self.calls.append("rotate")
        return RotateActionResult(ActionStatus.SUCCEEDED, "left_wrist", self.rotate_image_paths)

    def push(self, request: PushRequest, *, idempotency_key: str | None = None) -> ActionResult:
        pause_mock_processing()
        self.hands.append(request.hand)
        self.push_requests.append(request)
        self.idempotency_keys.append(idempotency_key)
        return self._record("push")

    def pick_basket(
        self, request: BasketPickRequest, *, idempotency_key: str | None = None
    ) -> ActionResult:
        pause_mock_processing()
        self.hands.append(request.hand)
        self.pick_basket_requests.append(request)
        self.idempotency_keys.append(idempotency_key)
        return self._record("pick_basket")

    def _record(self, action: str) -> ActionResult:
        self.calls.append(action)
        return ActionResult(ActionStatus.SUCCEEDED)
