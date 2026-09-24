from agent.capabilities.common import ActionResult, ActionStatus, HealthResult, HealthStatus
from agent.capabilities.http import HttpCapabilityClient

from .contract import HandPickRequest


class HttpHandCapability:
    def __init__(self, client: HttpCapabilityClient):
        self.client = client

    def health(self) -> HealthResult:
        return HealthResult(HealthStatus(self.client.get("/hand/health")["status"]))

    def pick(self, request: HandPickRequest, *, idempotency_key: str | None = None) -> ActionResult:
        payload = self.client.post_action(
            "/hand/pick",
            {
                "task_type": request.task_type.value,
                "target_type": request.target_type.value,
                "sku_id": request.sku_id,
                "hand": request.hand.value,
            },
            idempotency_key=idempotency_key,
        )
        return ActionResult(ActionStatus(payload["status"]))
