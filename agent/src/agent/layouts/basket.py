from agent.capabilities.common import Hand

AGV_ROWS = frozenset(f"L{i}" for i in range(1, 6))
BASKET_ROWS = frozenset(f"L{i}" for i in range(1, 5))
AGV_COLUMNS = frozenset({"1", "2"})
BASKET_COLUMNS = frozenset(str(i) for i in range(1, 6))


def validate_agv_position(row: str, column: str) -> None:
    if row not in AGV_ROWS or column not in AGV_COLUMNS:
        raise ValueError("agv_row must be L1-L5 and agv_column must be 1 or 2")


def validate_basket_position(row: str, column: str) -> None:
    if row not in BASKET_ROWS or column not in BASKET_COLUMNS:
        raise ValueError("basket_row must be L1-L4 and basket_column must be 1-5")


def agv_nav_id(column: str, hand: Hand) -> str:
    if column not in AGV_COLUMNS:
        raise ValueError("agv_column must be 1 or 2")
    return {
        ("1", Hand.LEFT): "AGV_C",
        ("1", Hand.RIGHT): "AGV_L",
        ("2", Hand.LEFT): "AGV_R",
        ("2", Hand.RIGHT): "AGV_C",
    }[(column, hand)]


def agv_side(column: str) -> str:
    if column not in AGV_COLUMNS:
        raise ValueError("agv_column must be 1 or 2")
    return "LEFT" if column == "1" else "RIGHT"


def basket_nav_id(column: str, *, review: bool = False) -> str:
    if column not in BASKET_COLUMNS:
        raise ValueError("basket_column must be 1-5")
    return f"{'REVIEW' if review else 'SORTING'}_BASKET_{column}"
