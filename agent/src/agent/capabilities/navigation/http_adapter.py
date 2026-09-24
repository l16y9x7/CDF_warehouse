from agent.capabilities.common import ActionResult, ActionStatus, HealthResult, HealthStatus
from agent.capabilities.http import HttpCapabilityClient


class HttpNavigationCapability:
    def __init__(self, client: HttpCapabilityClient):
        self.client = client

    def health(self) -> HealthResult:
        return HealthResult(HealthStatus(self.client.get("/navigation/health")["status"]))

    def navigate(self, nav_id: str, *, idempotency_key: str | None = None) -> ActionResult:
        payload = self.client.post_action(
            "/navigation/navigate",
            {"nav_id": nav_id},
            idempotency_key=idempotency_key,
        )
        return ActionResult(ActionStatus(payload["status"]))
