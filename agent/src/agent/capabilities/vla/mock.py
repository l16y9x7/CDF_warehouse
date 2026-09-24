from agent.capabilities.common import HealthResult, HealthStatus
from agent.capabilities.mocks.delay import pause_mock_processing

from .contract import VlaPickRequest, VlaPickResult, VlaPickStatus


class MockVlaCapability:
    def __init__(self) -> None:
        self.requests: list[VlaPickRequest] = []
        self.next_status = VlaPickStatus.PICKED

    def health(self) -> HealthResult:
        pause_mock_processing()
        return HealthResult(HealthStatus.READY)

    def pick_review_item(
        self, request: VlaPickRequest, *, idempotency_key: str | None = None
    ) -> VlaPickResult:
        pause_mock_processing()
        self.requests.append(request)
        return VlaPickResult(self.next_status)
