from pathlib import Path
from typing import Any

import yaml

from agent.skus import SkuSpec, parse_sku_catalog
from agent.workflows.policies import PickPolicy

DEFAULT_WORKFLOWS_PATH = Path("configs/workflows.yaml")
DEFAULT_AGENT_CONFIG_PATH = Path("configs/agent.yaml")
DEFAULT_PRODUCTS_PATH = Path("configs/products.yaml")


def load_yaml_config(path: str | Path) -> dict[str, Any]:
    with Path(path).open(encoding="utf-8") as stream:
        return yaml.safe_load(stream) or {}


def load_pick_policy(path: str | Path = DEFAULT_WORKFLOWS_PATH) -> PickPolicy:
    config_path = Path(path)
    if not config_path.exists():
        return PickPolicy()
    return PickPolicy.from_config(load_yaml_config(config_path))


def load_callback_url(path: str | Path = DEFAULT_AGENT_CONFIG_PATH) -> str:
    callback_url = load_yaml_config(path).get("callback_url")
    if not isinstance(callback_url, str) or not callback_url.strip():
        raise ValueError("configs/agent.yaml 必须配置 callback_url")
    return callback_url.strip()


def load_sku_catalog(path: str | Path = DEFAULT_PRODUCTS_PATH) -> dict[str, SkuSpec]:
    """从 products.yaml 加载 SKU 配置，转换为 sku_id -> SkuSpec 的字典"""
    config_path = Path(path)
    if not config_path.exists():
        return {}

    config = load_yaml_config(config_path)
    products = config.get("products", [])

    if not isinstance(products, list):
        raise ValueError("products.yaml 中的 products 字段必须是列表")

    # 将列表格式转换为字典格式
    skus_dict = {}
    for product in products:
        if not isinstance(product, dict):
            continue
        sku_id = product.get("sku_id")
        if not sku_id:
            continue
        skus_dict[str(sku_id)] = {
            "sku_typ": product.get("sku_typ"),
            "hand": product.get("hand"),
            "name": product.get("name", ""),
        }

    return parse_sku_catalog(skus_dict)
