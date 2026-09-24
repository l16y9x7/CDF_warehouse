"""联调用 Fake 相机。"""

from __future__ import annotations

from typing import Any, Dict, Optional

CAMERA_IDS = ("head", "hand_left", "hand_right")


class FakeCameraAdapter:
    def __init__(self) -> None:
        self._ready = {
            "head": True,
            "hand_left": True,
            "hand_right": False,
        }
        self._stamp = {
            "head": "t-head",
            "hand_left": "t-left",
            "hand_right": "",
        }

    def set_ready(self, camera_id: str, ready: bool) -> None:
        self._ready[camera_id] = bool(ready)

    def ready(self) -> bool:
        return any(self._ready.values())

    def listing(self) -> Dict[str, Any]:
        cameras = []
        for camera_id in CAMERA_IDS:
            cameras.append(
                {
                    "camera_id": camera_id,
                    "enabled": True,
                    "ready": bool(self._ready.get(camera_id)),
                }
            )
        return {"ok": self.ready(), "cameras": cameras}

    def frame(self, camera_id: str) -> Optional[Dict[str, Any]]:
        cid = map_camera_id(camera_id)
        if cid is None:
            return None
        return {
            "camera_id": cid,
            "uri": f"fake://camera/{cid}/color",
            "timestamp": self._stamp.get(cid) or "",
        }

    def stream_uri(self, camera_id: str) -> Optional[str]:
        cid = map_camera_id(camera_id)
        if cid is None:
            return None
        return f"fake://camera/{cid}/stream"


def map_camera_id(raw: str) -> Optional[str]:
    aliases = {
        "head": "head",
        "hand_left": "hand_left",
        "left_wrist": "hand_left",
        "left": "hand_left",
        "hand_right": "hand_right",
        "right_wrist": "hand_right",
        "right": "hand_right",
    }
    return aliases.get(str(raw or "").strip())
