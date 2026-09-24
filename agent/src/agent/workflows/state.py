from dataclasses import dataclass, field
from typing import Any

from agent.models import InspectedItem, ReviewItemCount, ReviewSummary


@dataclass
class SortingItemTaskState:
    task_id: str
    agv_row: str
    agv_column: str
    basket_row: str
    basket_column: str
    item: dict[str, str]
    sorting_policy: dict[str, str]
    current_node: str | None = None
    sku_barcode: str | None = None
    pick_result: dict[str, Any] | None = None
    place_result: dict[str, Any] | None = None
    status: str = "PENDING"


@dataclass
class SortingFinishTaskState:
    task_id: str
    basket_row: str
    basket_column: str
    sorting_policy: dict[str, str]
    pushed: bool = False
    last_node: str | None = None
    status: str = "PENDING"


@dataclass
class ReviewTaskState:
    task_id: str
    order_id: str
    basket_row: str
    basket_column: str
    expected_items: list[ReviewItemCount]
    review_policy: dict[str, str] = field(default_factory=dict)
    inspected_items: list[InspectedItem] = field(default_factory=list)
    current_item: dict[str, Any] | None = None
    basket_transfer_state: dict[str, bool] = field(
        default_factory=lambda: {"picked": False, "placed": False}
    )
    empty_observations: int = 0
    review_summary: ReviewSummary | None = None
    last_node: str | None = None
    status: str = "PENDING"
