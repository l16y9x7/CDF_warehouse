"""天机相机 Adapter：只消费 :8085 的 list/health，不打开第二份相机。"""

from __future__ import annotations

import json
import logging
from typing import Any, Dict, Optional
from urllib.error import HTTPError, URLError
from urllib.request import Request, urlopen

from vision.adapters.fake import map_camera_id

LOGGER = logging.getLogger(__name__)

_CONTRACT_TO_VENDOR = {
    "head": "head",
    "hand_left": "left_wrist",
    "hand_right": "right_wrist",
}


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
        LOGGER.warning("tianji camera unreachable: url=%s error=%s", url, exc)
        return 0, None
    if not raw:
        return status, {}
    try:
        value = json.loads(raw.decode("utf-8", "replace"))
    except json.JSONDecodeError:
        return status, None
    return status, value if isinstance(value, dict) else None


class TianjiCameraAdapter:
    def __init__(self, config: Dict[str, Any]) -> None:
        tianji = config.get("tianji") or {}
        self.base_url = str(tianji.get("base_url") or "http://127.0.0.1:8085").rstrip("/")
        self.health_path = str(tianji.get("health_path") or "/camera/health")
        self.list_path = str(tianji.get("list_path") or "/camera/list")
        self.snapshot_path = str(tianji.get("snapshot_path") or "/camera/snapshot")
        self.stream_path = str(tianji.get("stream_path") or "/camera/stream")
        self.timeout_sec = float(config.get("http_timeout_sec") or 3.0)

    def ready(self) -> bool:
        status, body = _http_json(f"{self.base_url}{self.health_path}", self.timeout_sec)
        return status == 200 and bool(body) and str(body.get("status") or "").upper() == "READY"

    def listing(self) -> Dict[str, Any]:
        status, body = _http_json(f"{self.base_url}{self.list_path}", self.timeout_sec)
        vendor_rows = (body or {}).get("cameras") if status == 200 and isinstance(body, dict) else []
        by_vendor: Dict[str, Dict[str, Any]] = {}
        if isinstance(vendor_rows, list):
            for row in vendor_rows:
                if not isinstance(row, dict):
                    continue
                vendor_id = str(row.get("id") or "")
                if vendor_id:
                    by_vendor[vendor_id] = row
        cameras = []
        any_ready = False
        for contract_id, vendor_id in _CONTRACT_TO_VENDOR.items():
            info = by_vendor.get(vendor_id) or {}
            ready = bool(info.get("online") or info.get("ready") or info.get("fresh"))
            any_ready = any_ready or ready
            cameras.append(
                {
                    "camera_id": contract_id,
                    "enabled": True,
                    "ready": ready,
                }
            )
        ok = any_ready or self.ready()
        return {"ok": ok, "cameras": cameras}

    def frame(self, camera_id: str) -> Optional[Dict[str, Any]]:
        cid = map_camera_id(camera_id)
        if cid is None:
            return None
        vendor_id = _CONTRACT_TO_VENDOR[cid]
        listing = self.listing()
        by_id = {row["camera_id"]: row for row in listing.get("cameras") or []}
        stamp = ""
        if by_id.get(cid, {}).get("ready"):
            stamp = "ready"
        return {
            "camera_id": cid,
            "uri": f"{self.base_url}{self.snapshot_path}?camera={vendor_id}&type=color",
            "timestamp": stamp,
        }

    def stream_uri(self, camera_id: str) -> Optional[str]:
        cid = map_camera_id(camera_id)
        if cid is None:
            return None
        vendor_id = _CONTRACT_TO_VENDOR[cid]
        return f"{self.base_url}{self.stream_path}?camera={vendor_id}&type=color"
