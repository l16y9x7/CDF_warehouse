from agent.capabilities.common import HealthResult, HealthStatus
from agent.capabilities.http import HttpCapabilityClient

from .contract import VlaPickRequest, VlaPickResult, VlaPickStatus


class HttpVlaCapability:
    def __init__(self, client: HttpCapabilityClient):
        self.client = client

    def health(self) -> HealthResult:
        return HealthResult(HealthStatus(self.client.get("/vla/health")["status"]))

    def pick_review_item(
        self, request: VlaPickRequest, *, idempotency_key: str | None = None
    ) -> VlaPickResult:
        payload = self.client.post_action(
            "/vla/pick_review_item",
            {"hand": request.hand.value},
            idempotency_key=idempotency_key,
        )
        return VlaPickResult(VlaPickStatus(payload["status"]))
