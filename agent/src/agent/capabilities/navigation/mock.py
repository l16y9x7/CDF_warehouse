from agent.capabilities.common import ActionResult, ActionStatus, HealthResult, HealthStatus
from agent.capabilities.mocks.delay import pause_mock_processing


class MockNavigationCapability:
    def __init__(self) -> None:
        self.calls: list[str] = []

    def health(self) -> HealthResult:
        pause_mock_processing()
        return HealthResult(HealthStatus.READY)

    def navigate(self, nav_id: str, *, idempotency_key: str | None = None) -> ActionResult:
        pause_mock_processing()
        self.calls.append(nav_id)
        return ActionResult(ActionStatus.SUCCEEDED)
