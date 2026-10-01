from dataclasses import asdict, dataclass
from pathlib import Path
from typing import Any

import yaml

from agent.skus import SkuSpec, parse_sku_catalog


@dataclass(frozen=True)
class OrderProduct:
    sku_id: str
    name: str
    description: str
    image_url: str
    category: str

    def public_dict(self) -> dict[str, str]:
        return asdict(self)


def load_product_catalog(path: str | Path) -> tuple[dict[str, OrderProduct], dict[str, SkuSpec]]:
    """
    从 products.yaml 加载商品目录和 SKU 配置。

    返回: (商品目录字典, SKU 配置字典)
    """
    config_path = Path(path)
    if not config_path.exists():
        return {}, {}
    with config_path.open(encoding="utf-8") as stream:
        raw = yaml.safe_load(stream) or {}
    products = raw.get("products", [])
    if not isinstance(products, list):
        raise ValueError("products must be a list")

    product_catalog: dict[str, OrderProduct] = {}
    sku_specs: dict[str, SkuSpec] = {}

    for index, value in enumerate(products):
        if not isinstance(value, dict):
            raise ValueError(f"products[{index}] must be an object")
        product = _parse_product(value, index)

        if product.sku_id in product_catalog:
            raise ValueError(f"duplicate product sku_id: {product.sku_id}")
        product_catalog[product.sku_id] = product

        # 解析 SKU 规格（用于机器人执行）
        sku_spec = _parse_sku_spec(value, product.sku_id, index)
        if sku_spec:
            sku_specs[product.sku_id] = sku_spec

    return product_catalog, sku_specs


def _parse_product(value: dict[str, Any], index: int) -> OrderProduct:
    def required(name: str) -> str:
        result = str(value.get(name, "")).strip()
        if not result:
            raise ValueError(f"products[{index}].{name} must be a non-empty string")
        return result

    return OrderProduct(
        sku_id=required("sku_id"),
        name=required("name"),
        description=str(value.get("description", "")).strip(),
        image_url=str(value.get("image_url", "")).strip(),
        category=str(value.get("category", "商品")).strip() or "商品",
    )


def _parse_sku_spec(value: dict[str, Any], sku_id: str, index: int) -> SkuSpec | None:
    """
    解析商品的 SKU 规格配置（sku_typ 和 hand）。
    如果不存在这些字段，返回 None（商品仅用于展示，不能被机器人抓取）。
    """
    from agent.capabilities.common import Hand

    sku_typ = str(value.get("sku_typ", "")).strip().lower()
    hand_str = str(value.get("hand", "")).strip().upper()

    # 如果两个字段都不存在，说明这个商品不需要机器人执行配置
    if not sku_typ and not hand_str:
        return None

    # 如果只有其中一个字段，说明配置不完整
    if not sku_typ:
        raise ValueError(f"products[{index}] (sku_id={sku_id}): 指定了 hand 但缺少 sku_typ")
    if not hand_str:
        raise ValueError(f"products[{index}] (sku_id={sku_id}): 指定了 sku_typ 但缺少 hand")

    try:
        hand = Hand(hand_str)
    except ValueError as exc:
        raise ValueError(
            f"products[{index}] (sku_id={sku_id}): hand must be LEFT or RIGHT, got {value.get('hand')!r}"
        ) from exc

    return SkuSpec(sku_typ=sku_typ, hand=hand, name=str(value.get("name", "")).strip())

