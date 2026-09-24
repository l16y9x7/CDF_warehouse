"""读与业务同目录的 zhongmian.json。相对路径相对 nav 仓库根。"""

from __future__ import annotations

import json
import logging
from pathlib import Path
from typing import Any, Dict

PACKAGE_ROOT = Path(__file__).resolve().parent.parent.parent
DEFAULT_CONFIG_PATH = Path(__file__).resolve().parent / "zhongmian.json"
LOGGER = logging.getLogger(__name__)


def load_config(path: str | None = None) -> Dict[str, Any]:
    config_path = Path(path) if path else DEFAULT_CONFIG_PATH
    if not config_path.is_absolute():
        config_path = PACKAGE_ROOT / config_path
    LOGGER.info("loading zhongmian config: %s", config_path)
    with config_path.open("r", encoding="utf-8") as handle:
        data = json.load(handle)
    if not isinstance(data, dict):
        raise ValueError("zhongmian config must be a JSON object")
    http_cfg = data.get("http") or {}
    LOGGER.info(
        "config loaded: http=%s:%s nav=%s",
        http_cfg.get("host") or "0.0.0.0",
        http_cfg.get("port") or 8081,
        (data.get("nav") or {}).get("base_url"),
    )
    return data
