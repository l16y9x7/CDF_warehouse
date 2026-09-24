"""臂状态：配置 url 走本机 HTTP；否则沿用 rokae_robot_state.get_state()。"""

from __future__ import annotations

import json
import logging
import sys
from typing import Any, Dict, Mapping, Optional
from urllib.request import Request, urlopen

LOGGER = logging.getLogger(__name__)

_DEFAULT_PYTHONPATH = "/home/admin/LuLihu/rokae_robot_python/src"
_ARM_KEYS = ("left_arm", "right_arm", "body", "head")
_LEGACY_FIELDS = ("state", "joint_positions_deg", "end_pose")


class ManipulatorStateReader:
    def __init__(self, config: Optional[Any] = None) -> None:
        self._config = config if isinstance(config, Mapping) else {}
        self._service = None
        self.source_url = ""

    def start(self) -> bool:
        if self._service is not None:
            return True
        cfg = _manipulator_cfg(self._config)
        url = str(cfg.get("url") or "").strip()
        if url:
            self._service = _HttpManipulatorService(url)
            self.source_url = url
            LOGGER.info("manipulator http started: url=%s", url)
            return True
        _apply_pythonpath(cfg)
        try:
            from rokae_robot_state import build_manipulator_state_service
            from rokae_robot_web.config import load_config

            _install_xcore_joint_truncation()
            service = build_manipulator_state_service(load_config())
            service.start()
        except Exception as exc:
            LOGGER.warning("manipulator get_state unavailable: %s", exc)
            return False
        self._service = service
        self.source_url = "rokae_robot_state.get_state"
        LOGGER.info("manipulator get_state started")
        return True

    def stop(self) -> None:
        service, self._service = self._service, None
        self.source_url = ""
        if service is not None and hasattr(service, "stop"):
            service.stop()

    def osd_blocks(self) -> Dict[str, Any]:
        if self._service is None:
            return {}
        snapshot = self._service.get_state()
        out: Dict[str, Any] = {}
        legacy = snapshot.to_legacy_dual_arm_mapping()
        if legacy:
            out["manipulator_status"] = legacy
        full = snapshot.to_mapping()
        if any(full.get(name) for name in _ARM_KEYS):
            out["arm_action"] = full
        return out


class _HttpManipulatorService:
    def __init__(self, url: str, *, opener: Any = None, timeout_sec: float = 0.8) -> None:
        self.url = url
        self._opener = opener or urlopen
        self._timeout_sec = timeout_sec

    def start(self) -> None:
        return None

    def stop(self) -> None:
        return None

    def get_state(self) -> "_HttpSnapshot":
        return _HttpSnapshot(_fetch_mapping(self.url, self._opener, self._timeout_sec))


class _HttpSnapshot:
    def __init__(self, mapping: Mapping[str, Any]) -> None:
        self._mapping = dict(mapping)

    def to_mapping(self) -> Dict[str, Any]:
        return dict(self._mapping)

    def to_legacy_dual_arm_mapping(self) -> Dict[str, Any]:
        out: Dict[str, Any] = {}
        for name in ("left_arm", "right_arm"):
            raw = self._mapping.get(name)
            if not isinstance(raw, Mapping):
                return {}
            if any(key not in raw for key in _LEGACY_FIELDS):
                return {}
            joints = raw.get("joint_positions_deg")
            if not isinstance(joints, list) or not joints or any(v is None for v in joints):
                return {}
            out[name] = {key: raw[key] for key in _LEGACY_FIELDS}
        return out


def _manipulator_cfg(config: Mapping[str, Any]) -> Dict[str, Any]:
    raw = config.get("manipulator")
    return dict(raw) if isinstance(raw, Mapping) else {}


def _install_xcore_joint_truncation(module: Any = None) -> None:
    """本机 xCore 关节向量是 13 个数；采集器仍要求正好 7 个。只在 Gateway 侧截断。"""

    if module is None:
        from rokae_robot_state import xcore_client as module
    reader = module._read_vector
    if getattr(reader, "_gateway_arm_trunc", False):
        return
    dof = int(getattr(module, "ARM_DOF", 7) or 7)
    labels = {"JOINT_POSITION", "JOINT_VELOCITY", "JOINT_TORQUE"}

    def _read_vector(getter: Any, label: str) -> Any:
        values = reader(getter, label)
        if label in labels and len(values) > dof:
            return values[:dof]
        return values

    _read_vector._gateway_arm_trunc = True
    module._read_vector = _read_vector


def _apply_pythonpath(cfg: Mapping[str, Any]) -> None:
    paths = cfg.get("pythonpath")
    if isinstance(paths, str):
        items = [paths]
    elif isinstance(paths, list):
        items = [str(item) for item in paths]
    else:
        items = [_DEFAULT_PYTHONPATH]
    for raw in reversed(items):
        path = str(raw or "").strip()
        if path and path not in sys.path:
            sys.path.insert(0, path)


def _fetch_mapping(url: str, opener: Any, timeout_sec: float) -> Dict[str, Any]:
    request = Request(url, method="GET", headers={"Accept": "application/json"})
    try:
        with opener(request, timeout=timeout_sec) as response:
            raw = response.read()
    except Exception as exc:
        LOGGER.debug("manipulator http failed: url=%s error=%s", url, exc)
        return {}
    try:
        parsed = json.loads(raw)
    except (TypeError, ValueError):
        return {}
    if not isinstance(parsed, Mapping):
        return {}
    data = parsed.get("data") if isinstance(parsed.get("data"), Mapping) else parsed
    if not isinstance(data, Mapping):
        return {}
    return dict(data)
