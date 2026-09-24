"""相机 Capability Service：发现相机、给 FrameRef，不做估姿。"""

from __future__ import annotations

import logging
from typing import Any, Dict, Optional

from vision.adapters.fake import FakeCameraAdapter, map_camera_id
from vision.adapters.rokae import RokaeCameraAdapter
from vision.adapters.tianji import TianjiCameraAdapter
from vision.push import PushRuntime

LOGGER = logging.getLogger(__name__)


class CameraService:
    def __init__(self, config: Dict[str, Any], *, adapter: Any = None) -> None:
        self.config = config
        self.adapter = adapter or build_adapter(config)

    def health(self) -> Dict[str, Any]:
        return {"ok": bool(self.adapter.ready())}

    def listing(self) -> Dict[str, Any]:
        return self.adapter.listing()

    def state(self) -> Dict[str, Any]:
        listing = self.adapter.listing()
        ready = bool(listing.get("ok"))
        return {
            "self_check": {
                "status": 0 if ready else 1,
                "message": "就绪" if ready else "相机未就绪",
            },
            "cameras": listing.get("cameras") or [],
        }

    def frame(self, camera_id: str) -> Dict[str, Any]:
        if map_camera_id(camera_id) is None:
            return {"ok": False, "error_code": "CAMERA_NOT_FOUND", "message": f"unknown camera_id={camera_id}"}
        ref = self.adapter.frame(camera_id)
        if ref is None:
            return {"ok": False, "error_code": "CAMERA_NOT_FOUND", "message": f"unknown camera_id={camera_id}"}
        if ref.get("error_code"):
            return ref
        LOGGER.info("frame ref: camera_id=%s uri=%s", ref.get("camera_id"), ref.get("uri"))
        return ref

    def stream(self, camera_id: str) -> Dict[str, Any]:
        uri = self.adapter.stream_uri(camera_id)
        cid = map_camera_id(camera_id)
        if uri is None or cid is None:
            return {"ok": False, "error_code": "CAMERA_NOT_FOUND", "message": f"unknown camera_id={camera_id}"}
        return {"camera_id": cid, "uri": uri}


class MediaService:
    def __init__(self, camera_service: CameraService, config: Optional[Dict[str, Any]] = None) -> None:
        self.camera = camera_service
        self.push = PushRuntime(config or camera_service.config, source_uri=self._source_uri)

    def health(self) -> Dict[str, Any]:
        return {"ok": True}

    def stream(self, camera_id: str) -> Dict[str, Any]:
        return self.camera.stream(camera_id)

    def state(self) -> Dict[str, Any]:
        status = self.push.status()
        return {
            "self_check": {"status": 0, "message": "就绪"},
            "push": status,
        }

    def start_push(self, *, camera_id: str = "", device_sn: str = "", stream_key: str = "") -> Dict[str, Any]:
        result = self.push.start(camera_id=camera_id, device_sn=device_sn, stream_key=stream_key)
        LOGGER.info(
            "media push start: camera=%s sn=%s ok=%s",
            camera_id or "*",
            device_sn or self.push.device_sn,
            result.get("ok"),
        )
        return result

    def stop_push(self, *, camera_id: str = "") -> Dict[str, Any]:
        result = self.push.stop(camera_id=camera_id)
        LOGGER.info("media push stop: camera=%s ok=%s", camera_id or "*", result.get("ok"))
        return result

    def push_status(self, *, camera_id: str = "") -> Dict[str, Any]:
        return {"ok": True, **self.push.status(camera_id=camera_id)}

    def _source_uri(self, camera_id: str) -> str:
        if isinstance(self.camera.adapter, RokaeCameraAdapter):
            rows = self.camera.listing().get("cameras") or []
            cid = map_camera_id(camera_id)
            if not any(row.get("camera_id") == cid and row.get("ready") is True
                       and row.get("enabled") is not False for row in rows):
                return ""
        body = self.camera.stream(camera_id)
        if body.get("error_code"):
            return ""
        return str(body.get("uri") or "")


def build_adapter(config: Dict[str, Any]) -> Any:
    name = str(config.get("adapter") or "tianji").strip().lower()
    if name == "fake":
        return FakeCameraAdapter()
    if name == "tianji":
        return TianjiCameraAdapter(config)
    if name == "rokae":
        return RokaeCameraAdapter(config)
    raise ValueError(f"unknown camera adapter: {name}")
