from pathlib import Path
from typing import Any

import yaml

from agent.skus import SkuSpec, parse_sku_catalog
from agent.workflows.policies import PickPolicy

DEFAULT_WORKFLOWS_PATH = Path("configs/workflows.yaml")
DEFAULT_AGENT_CONFIG_PATH = Path("configs/agent.yaml")


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


def load_sku_catalog(path: str | Path = DEFAULT_WORKFLOWS_PATH) -> dict[str, SkuSpec]:
    config_path = Path(path)
    if not config_path.exists():
        return {}
    return parse_sku_catalog(load_yaml_config(config_path).get("skus"))
