from collections.abc import Mapping, Sequence
from dataclasses import dataclass

from agent.capabilities.common import Hand


@dataclass(frozen=True)
class SkuSpec:
    sku_typ: str
    hand: Hand
    name: str = ""


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
        catalog[key] = SkuSpec(sku_typ, hand, name)
    return catalog


def sku_spec(catalog: Mapping[str, SkuSpec], sku_id: str) -> SkuSpec:
    spec = catalog.get(str(sku_id).strip())
    if spec is None:
        raise ValueError(f"sku_id {sku_id} 未配置，请在 configs/workflows.yaml 的 skus 中补充")
    return spec


def shared_working_hand(catalog: Mapping[str, SkuSpec], sku_ids: Sequence[str]) -> Hand:
    hands = {sku_spec(catalog, sku_id).hand for sku_id in sku_ids}
    if len(hands) != 1:
        raise ValueError("expected_items 必须映射到同一只工作手")
    return next(iter(hands))
