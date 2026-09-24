"""珞石（ROKAE）相机 Adapter（机型 Helios）。只消费 Owner HTTP，不打开第二份相机。

命名约定（唯一）：厂商=珞石（ROKAE），配置键=rokae。
"""

from __future__ import annotations

import json
import logging
from typing import Any, Dict, Optional
from urllib.error import HTTPError, URLError
from urllib.request import Request, urlopen
from urllib.parse import urlencode

from vision.adapters.fake import map_camera_id

LOGGER = logging.getLogger(__name__)

# 合同 id → 厂家 id（现场 hand_cameras.name / 头部约定）
_DEFAULT_CONTRACT_TO_VENDOR = {
    "head": "head",
    "hand_left": "left_wrist",
    "hand_right": "hand_wrist",
}

_LEFT_HTTP_ALIASES = {"left_wrist", "hand_left", "left"}
_RIGHT_HTTP_ALIASES = {"hand_wrist", "hand_right", "right_wrist", "right"}


def _http_json(url: str, timeout_sec: float) -> tuple[int, Optional[Dict[str, Any]]]:
    request = Request(url, method="GET", headers={"Accept": "application/json"})
    try:
        with urlopen(request, timeout=timeout_sec) as response:
            raw = response.read()
            status = int(getattr(response, "status", 200) or 200)
    except HTTPError as exc:
        raw = exc.read() if exc.fp is not None else b""
        status = int(exc.code)
    except (URLError, TimeoutError, OSError) as exc:
        LOGGER.warning("rokae camera unreachable: url=%s error=%s", url, exc)
        return 0, None
    if not raw:
        return status, {}
    try:
        value = json.loads(raw.decode("utf-8", "replace"))
    except json.JSONDecodeError:
        return status, None
    return status, value if isinstance(value, (dict, list)) else None


class RokaeCameraAdapter:
    """消费珞石（ROKAE）相机 Owner（默认 :8085），对齐 dog_device 分支 ROKAE@a4a9846 三路语义。"""

    def __init__(self, config: Dict[str, Any]) -> None:
        rokae = config.get("rokae") or {}
        if not isinstance(rokae, dict):
            raise ValueError("config.rokae must be an object")
        self.base_url = str(rokae.get("base_url") or "http://127.0.0.1:8085").rstrip("/")
        self.health_path = str(rokae.get("health_path") or "/camera/health")
        self.list_path = str(rokae.get("list_path") or "/camera/list")
        self.snapshot_path = str(rokae.get("snapshot_path") or "/camera/snapshot")
        self.stream_path = str(rokae.get("stream_path") or "/camera/stream")
        self.timeout_sec = float(config.get("http_timeout_sec") or 3.0)
        mapping = rokae.get("camera_id_map") or {}
        self._contract_to_vendor = dict(_DEFAULT_CONTRACT_TO_VENDOR)
        if isinstance(mapping, dict):
            for key, value in mapping.items():
                cid = map_camera_id(str(key))
                if cid and value:
                    self._contract_to_vendor[cid] = str(value)

    def ready(self) -> bool:
        status, body = _http_json(f"{self.base_url}{self.health_path}", self.timeout_sec)
        if status != 200 or not body:
            return False
        health = str(body.get("status") or body.get("state") or "").upper()
        if health in {"READY", "OK", "HEALTHY"}:
            return True
        return bool(body.get("ok"))

    def listing(self) -> Dict[str, Any]:
        status, body = _http_json(f"{self.base_url}{self.list_path}", self.timeout_sec)
        vendor_rows = (body if isinstance(body, list) else (body or {}).get("cameras", [])) if status == 200 else []
        by_vendor: Dict[str, Dict[str, Any]] = {}
        if isinstance(vendor_rows, list):
            for row in vendor_rows:
                if not isinstance(row, dict):
                    continue
                vendor_id = str(row.get("id") or row.get("name") or row.get("camera_id") or "")
                if vendor_id:
                    by_vendor[vendor_id] = row
                    aliases = (
                        _LEFT_HTTP_ALIASES if vendor_id in _LEFT_HTTP_ALIASES
                        else _RIGHT_HTTP_ALIASES if vendor_id in _RIGHT_HTTP_ALIASES
                        else ()
                    )
                    for alias in aliases:
                        by_vendor.setdefault(alias, row)
        cameras = []
        any_ready = False
        for contract_id, vendor_id in self._contract_to_vendor.items():
            info = by_vendor.get(vendor_id) or {}
            # An explicit negative freshness/readiness signal wins over online.
            signals = [info[key] for key in ("ready", "fresh", "online") if key in info]
            ready = bool(signals) and all(value is True for value in signals)
            # enabled:false 的手相机不算 ready
            if info.get("enabled") is False:
                ready = False
            error = ""
            if not ready:
                if info.get("enabled") is False:
                    error = "CAMERA_DISABLED"
                elif status != 200:
                    error = "OWNER_UNAVAILABLE"
                else:
                    error = str(info.get("error") or "CAMERA_NOT_READY")
            any_ready = any_ready or ready
            cameras.append(
                {
                    "camera_id": contract_id,
                    "enabled": info.get("enabled", True) is not False,
                    "ready": ready,
                    "error": error,
                }
            )
        ok = any_ready
        return {"ok": ok, "cameras": cameras}

    def frame(self, camera_id: str) -> Optional[Dict[str, Any]]:
        cid = map_camera_id(camera_id)
        if cid is None:
            return None
        vendor_id = self._contract_to_vendor[cid]
        listing = self.listing()
        by_id = {row["camera_id"]: row for row in listing.get("cameras") or []}
        row = by_id.get(cid) or {}
        if not row.get("ready"):
            return {
                "ok": False,
                "error_code": "CAMERA_NOT_READY",
                "camera_id": cid,
                "reason": row.get("error") or "CAMERA_NOT_READY",
                "message": f"camera not ready camera_id={cid}",
            }
        return {
            "ok": True,
            "camera_id": cid,
            "uri": f"{self.base_url}/camera/frame?{urlencode({'camera': vendor_id, 'type': 'color'})}",
            "timestamp": "ready",
        }

    def stream_uri(self, camera_id: str) -> Optional[str]:
        cid = map_camera_id(camera_id)
        if cid is None:
            return None
        vendor_id = self._contract_to_vendor[cid]
        return f"{self.base_url}{self.stream_path}?{urlencode({'camera': vendor_id, 'type': 'color'})}"
