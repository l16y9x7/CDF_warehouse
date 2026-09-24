from dataclasses import dataclass


@dataclass(frozen=True)
class ReviewItemCount:
    sku_id: str
    count: int


@dataclass(frozen=True)
class InspectedItem:
    sequence: int
    actual_sku_id: str
    barcode_status: str = "READABLE"
    place_status: str = "SUCCEEDED"


@dataclass(frozen=True)
class ReviewSummary:
    review_status: str
    missing: tuple[ReviewItemCount, ...]
    extra: tuple[ReviewItemCount, ...]
    wrong: tuple[ReviewItemCount, ...]
