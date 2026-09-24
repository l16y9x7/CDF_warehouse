"""读 config/navigation.json：adapter、:8001 端口、珞石底盘 IP、是否 sim。"""

from __future__ import annotations

import json
import logging
from pathlib import Path
from typing import Any, Dict

PACKAGE_ROOT = Path(__file__).resolve().parent.parent
DEFAULT_CONFIG_PATH = PACKAGE_ROOT / "config" / "navigation.json"
LOGGER = logging.getLogger(__name__)


def load_config(path: str | None = None) -> Dict[str, Any]:
    config_path = Path(path) if path else DEFAULT_CONFIG_PATH
    LOGGER.info("loading navigation config: %s", config_path)
    with config_path.open("r", encoding="utf-8") as handle:
        data = json.load(handle)
    if not isinstance(data, dict):
        raise ValueError("navigation config must be a JSON object")
    http_cfg = data.get("http") or {}
    LOGGER.info(
        "config loaded: adapter=%s http=%s:%s",
        data.get("adapter") or "tianji",
        http_cfg.get("host") or "0.0.0.0",
        http_cfg.get("port") or 8001,
    )
    return data
