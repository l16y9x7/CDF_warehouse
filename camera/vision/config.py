"""加载 vision JSON 配置。"""

from __future__ import annotations

import json
import logging
from pathlib import Path
from typing import Any, Dict

PACKAGE_ROOT = Path(__file__).resolve().parent.parent
DEFAULT_CONFIG_PATH = PACKAGE_ROOT / "config" / "vision.json"
LOGGER = logging.getLogger(__name__)


def load_config(path: str | None = None) -> Dict[str, Any]:
    config_path = Path(path) if path else DEFAULT_CONFIG_PATH
    LOGGER.info("loading vision config: %s", config_path)
    with config_path.open("r", encoding="utf-8") as handle:
        data = json.load(handle)
    if not isinstance(data, dict):
        raise ValueError("vision config must be a JSON object")
    camera = data.get("camera") or {}
    LOGGER.info(
        "config loaded: adapter=%s camera=%s:%s",
        data.get("adapter") or "tianji",
        camera.get("host") or "0.0.0.0",
        camera.get("port") or 8003,
    )
    return data
