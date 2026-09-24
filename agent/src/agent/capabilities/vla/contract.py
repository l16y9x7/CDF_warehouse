from dataclasses import dataclass
from enum import Enum
from typing import Protocol

from agent.capabilities.common import Hand, HealthResult


@dataclass(frozen=True)
class VlaPickRequest:
    hand: Hand


class VlaPickStatus(str, Enum):
    PICKED = "PICKED"
    NOT_FOUND = "NOT_FOUND"


@dataclass(frozen=True)
class VlaPickResult:
    status: VlaPickStatus


class VlaCapability(Protocol):
    def health(self) -> HealthResult: ...
    def pick_review_item(
        self, request: VlaPickRequest, *, idempotency_key: str | None = None
    ) -> VlaPickResult: ...
