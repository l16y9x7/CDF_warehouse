from dataclasses import dataclass
from typing import Protocol

from agent.capabilities.common import ActionResult, Hand, HealthResult, TargetType, TaskType


@dataclass(frozen=True)
class HandPickRequest:
    task_type: TaskType
    target_type: TargetType
    sku_id: str
    hand: Hand

    def __post_init__(self) -> None:
        if self.task_type is not TaskType.SORTING or self.target_type is not TargetType.SKU:
            raise ValueError("hand pick requires SORTING sku")
        if not self.sku_id:
            raise ValueError("hand pick requires sku_id")


class HandCapability(Protocol):
    def health(self) -> HealthResult: ...
    def pick(
        self, request: HandPickRequest, *, idempotency_key: str | None = None
    ) -> ActionResult: ...
