from agent.capabilities.common import ActionResult, ActionStatus, HealthResult, HealthStatus
from agent.capabilities.mocks.delay import pause_mock_processing

from .contract import HandPickRequest


class MockHandCapability:
    def __init__(self) -> None:
        self.calls: list[HandPickRequest] = []

    def health(self) -> HealthResult:
        pause_mock_processing()
        return HealthResult(HealthStatus.READY)

    def pick(self, request: HandPickRequest, *, idempotency_key: str | None = None) -> ActionResult:
        pause_mock_processing()
        self.calls.append(request)
        return ActionResult(ActionStatus.SUCCEEDED)
