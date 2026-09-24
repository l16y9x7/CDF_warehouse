"""加载 Gateway JSON 配置。"""

from __future__ import annotations

import json
import logging
import os
from pathlib import Path
from typing import Any, Dict, Mapping, MutableMapping, Optional

PACKAGE_ROOT = Path(__file__).resolve().parent.parent
DEFAULT_CONFIG_PATH = PACKAGE_ROOT / "config" / "gateway.json"

LOGGER = logging.getLogger(__name__)

_AUTO_VALUES = {"", "auto", "*"}


def load_config(path: str | None = None, *, environ: Optional[Mapping[str, str]] = None) -> Dict[str, Any]:
    config_path = Path(path) if path else DEFAULT_CONFIG_PATH
    LOGGER.info("loading gateway config: %s", config_path)
    with config_path.open("r", encoding="utf-8") as handle:
        data = json.load(handle)
    if not isinstance(data, dict):
        raise ValueError("gateway config must be a JSON object")
    apply_runtime_dir(data, config_path=config_path, environ=environ)
    _require_device_sn(data)
    device = data.get("device") or {}
    mqtt_cfg = data.get("mqtt") or {}
    LOGGER.info(
        "config loaded: sn=%s broker=%s:%s client_id=%s",
        device.get("sn") or "",
        mqtt_cfg.get("broker_host") or "",
        mqtt_cfg.get("broker_port") or 1883,
        mqtt_cfg.get("client_id") or f"gateway_{device.get('sn') or ''}",
    )
    return data


def _require_device_sn(data: MutableMapping[str, Any]) -> None:
    device = data.get("device")
    if not isinstance(device, dict):
        device = {}
        data["device"] = device
    sn = str(device.get("sn") or "").strip()
    if not sn or _is_auto(sn):
        raise ValueError("device.sn must be set in config/gateway.json for this robot")
    device["sn"] = sn


def apply_runtime_dir(
    data: MutableMapping[str, Any],
    *,
    config_path: Path,
    environ: Optional[Mapping[str, str]] = None,
) -> Path:
    """本机数据目录用仓库内 runtime/。gitignore 排除，拷代码时不要带上。"""

    env = environ if environ is not None else os.environ
    root = _repo_root(config_path)
    configured = str(data.get("runtime_dir") or "auto").strip()
    env_dir = str(env.get("GATEWAY_RUNTIME_DIR") or "").strip()
    if env_dir:
        runtime = _as_root_path(root, env_dir)
        source = "env:GATEWAY_RUNTIME_DIR"
    elif not _is_auto(configured):
        runtime = _as_root_path(root, configured)
        source = str(config_path)
    else:
        runtime = root / "runtime"
        source = "default:runtime"
    runtime.mkdir(parents=True, exist_ok=True)
    data["runtime_dir"] = str(runtime)
    map_cfg = data.get("map_sync")
    if not isinstance(map_cfg, dict):
        map_cfg = {}
        data["map_sync"] = map_cfg
    identity = str(map_cfg.get("identity_file") or "map_index.json").strip() or "map_index.json"
    map_cfg["identity_file"] = str(_rebase_runtime_path(runtime, identity, "map_index.json"))
    LOGGER.info("runtime dir: path=%s source=%s", runtime, source)
    return runtime


def _repo_root(config_path: Path) -> Path:
    if config_path.parent.name == "config":
        return config_path.parent.parent
    return PACKAGE_ROOT


def _as_root_path(root: Path, raw: str) -> Path:
    path = Path(raw.strip())
    if not path.is_absolute():
        path = root / path
    return path


def _rebase_runtime_path(runtime: Path, raw: str, default_name: str) -> Path:
    text = str(raw or "").strip() or default_name
    path = Path(text)
    if path.is_absolute():
        return path
    parts = path.parts
    if parts and parts[0] == "runtime":
        rest = parts[1:] or (default_name,)
        return runtime.joinpath(*rest)
    return runtime / path


def _is_auto(value: str) -> bool:
    return value.strip().lower() in _AUTO_VALUES
