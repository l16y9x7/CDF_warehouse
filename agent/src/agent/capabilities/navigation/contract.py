from typing import Protocol

from agent.capabilities.common import ActionResult, HealthResult

NAVIGATION_TARGETS = frozenset(
    {
        "AGV_L",
        "AGV_C",
        "AGV_R",
        "SORTING_BASKET_1",
        "SORTING_BASKET_2",
        "SORTING_BASKET_3",
        "SORTING_BASKET_4",
        "SORTING_BASKET_5",
        "REVIEW_BASKET_1",
        "REVIEW_BASKET_2",
        "REVIEW_BASKET_3",
        "REVIEW_BASKET_4",
        "REVIEW_BASKET_5",
        "REVIEW_TABLE",
    }
)


class NavigationCapability(Protocol):
    def health(self) -> HealthResult: ...

    def navigate(self, nav_id: str, *, idempotency_key: str | None = None) -> ActionResult: ...
