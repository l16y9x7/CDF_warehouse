from dataclasses import asdict, dataclass
from pathlib import Path
from typing import Any, Mapping

import yaml

from agent.skus import SkuSpec


@dataclass(frozen=True)
class OrderProduct:
    sku_id: str
    name: str
    description: str
    image_url: str
    category: str

    def public_dict(self) -> dict[str, str]:
        return asdict(self)


def load_product_catalog(
    path: str | Path, sku_catalog: Mapping[str, SkuSpec]
) -> dict[str, OrderProduct]:
    config_path = Path(path)
    if not config_path.exists():
        return {}
    with config_path.open(encoding="utf-8") as stream:
        raw = yaml.safe_load(stream) or {}
    products = raw.get("products", [])
    if not isinstance(products, list):
        raise ValueError("products must be a list")
    catalog: dict[str, OrderProduct] = {}
    for index, value in enumerate(products):
        if not isinstance(value, dict):
            raise ValueError(f"products[{index}] must be an object")
        product = _parse_product(value, index)
        if product.sku_id not in sku_catalog:
            continue
        if product.sku_id in catalog:
            raise ValueError(f"duplicate product sku_id: {product.sku_id}")
        catalog[product.sku_id] = product
    return catalog


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
