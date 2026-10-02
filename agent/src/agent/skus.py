import math
from collections.abc import Mapping, Sequence
from dataclasses import dataclass
from typing import cast

from agent.capabilities.common import Hand

MEASURE_FIELDS = ("length_mm", "width_mm", "height_mm", "weight_g")


@dataclass(frozen=True)
class SkuSpec:
    sku_typ: str
    hand: Hand
    name: str = ""
    sku_code: str = ""
    length_mm: float | None = None
    width_mm: float | None = None
    height_mm: float | None = None
    weight_g: float | None = None

    def measures(self) -> tuple[float, float, float, float]:
        values = (self.length_mm, self.width_mm, self.height_mm, self.weight_g)
        if any(value is None for value in values):
            missing = ", ".join(
                name for name, value in zip(MEASURE_FIELDS, values, strict=True) if value is None
            )
            raise ValueError(f"未配置 {missing}")
        return cast(tuple[float, float, float, float], values)


def parse_sku_catalog(raw: object) -> dict[str, SkuSpec]:
    if raw in (None, {}):
        return {}
    if not isinstance(raw, dict):
        raise ValueError("skus must be a mapping of sku_id to sku_typ and hand")
    catalog: dict[str, SkuSpec] = {}
    for sku_id, value in raw.items():
        key = str(sku_id).strip()
        if not key:
            raise ValueError("skus keys must be non-empty sku_id values")
        if not isinstance(value, Mapping):
            raise ValueError(f"skus[{key!r}] must be an object with sku_typ and hand")
        sku_typ = str(value.get("sku_typ", "")).strip().lower()
        if not sku_typ:
            raise ValueError(f"skus[{key!r}].sku_typ must be a non-empty string")
        try:
            hand = Hand(str(value.get("hand", "")).upper())
        except ValueError as exc:
            raise ValueError(
                f"skus[{key!r}].hand must be LEFT or RIGHT, got {value.get('hand')!r}"
            ) from exc
        name = str(value.get("name") or "").strip()
        sku_code = str(value.get("sku_code") or "").strip()
        length_mm, width_mm, height_mm, weight_g = parse_measures(value, key)
        catalog[key] = SkuSpec(
            sku_typ, hand, name, sku_code, length_mm, width_mm, height_mm, weight_g
        )
    return catalog


def parse_measures(
    value: Mapping, sku_id: str
) -> tuple[float | None, float | None, float | None, float | None]:
    parsed: list[float | None] = []
    for name in MEASURE_FIELDS:
        raw = value.get(name)
        if raw is None or raw == "":
            parsed.append(None)
            continue
        if isinstance(raw, bool) or not isinstance(raw, (int, float)):
            raise ValueError(f"skus[{sku_id!r}].{name} must be a positive number")
        number = float(raw)
        if not math.isfinite(number) or number <= 0:
            raise ValueError(f"skus[{sku_id!r}].{name} must be a positive number")
        parsed.append(number)
    return parsed[0], parsed[1], parsed[2], parsed[3]


def sku_spec(catalog: Mapping[str, SkuSpec], sku_id: str) -> SkuSpec:
    spec = catalog.get(str(sku_id).strip())
    if spec is None:
        raise ValueError(f"sku_id {sku_id} 未配置，请在 configs/products.yaml 中补充 sku_typ 和 hand 字段")
    return spec


def sku_id_for_code(catalog: Mapping[str, SkuSpec], sku_code: str) -> str | None:
    code = str(sku_code).strip()
    if not code:
        return None
    matches = [sku_id for sku_id, spec in catalog.items() if spec.sku_code == code]
    if len(matches) != 1:
        return None
    return matches[0]


def shared_working_hand(catalog: Mapping[str, SkuSpec], sku_ids: Sequence[str]) -> Hand:
    hands = {sku_spec(catalog, sku_id).hand for sku_id in sku_ids}
    if len(hands) != 1:
        raise ValueError("expected_items 必须映射到同一只工作手")
    return next(iter(hands))
