"""Gateway 只下发推流起停，不碰相机、不跑 ffmpeg。"""

from __future__ import annotations

import logging
from typing import Any, Dict, Mapping

LOGGER = logging.getLogger(__name__)


class MediaPushController:
    def __init__(self, config: Mapping[str, Any], *, scenario_client) -> None:
        media = config.get("media") or {}
        if not isinstance(media, Mapping):
            media = {}
        self.url = str(media.get("url") or "").strip().rstrip("/")
        self.push_on_start = bool(media.get("push_on_start", True)) and bool(self.url)
        self.device_sn = str((config.get("device") or {}).get("sn") or "").strip()
        self.stream_key = str(media.get("stream_key") or "").strip()
        self._client = scenario_client

    def status(self) -> Dict[str, Any]:
        if not self.url:
            return {"ok": False, "error": "MEDIA_NOT_CONFIGURED"}
        body = self._client.get_json(f"{self.url}/push")
        if not body:
            return {"ok": False, "error": "MEDIA_UNREACHABLE", "url": self.url}
        return body

    def start(self, *, camera_id: str = "", stream_key: str = "") -> Dict[str, Any]:
        return self._call(
            "/push/start",
            {
                "device_sn": self.device_sn,
                "camera_id": camera_id,
                "stream_key": str(stream_key or "").strip() or self.stream_key,
            },
        )

    def stop(self, *, camera_id: str = "") -> Dict[str, Any]:
        return self._call("/push/stop", {"camera_id": camera_id})

    def on_gateway_start(self) -> None:
        if not self.push_on_start:
            LOGGER.info("media cloud push not requested at gateway start")
            return
        result = self.start()
        LOGGER.info(
            "media cloud push requested: url=%s sn=%s ok=%s error=%s",
            self.url,
            self.device_sn,
            result.get("ok"),
            result.get("error") or result.get("error_code") or "",
        )

    def _call(self, path: str, body: Mapping[str, Any]) -> Dict[str, Any]:
        if not self.url:
            return {"ok": False, "error": "MEDIA_NOT_CONFIGURED"}
        response = self._client.post(self.url, path, body)
        payload = dict(response.body or {})
        if not response.reached:
            return {
                "ok": False,
                "error": "MEDIA_UNREACHABLE",
                "url": self.url,
                "message": response.error,
            }
        if payload:
            payload.setdefault("ok", response.accepted)
            return payload
        return {
            "ok": response.accepted,
            "error": response.error_code or ("MEDIA_HTTP" if not response.accepted else ""),
            "message": response.error,
        }
