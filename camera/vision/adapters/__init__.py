"""相机 Adapter。"""

from __future__ import annotations

from typing import Any, Dict, Optional, Protocol

# 已实现：fake / tianji / rokae（珞石（ROKAE），配置键仅 rokae）


class CameraAdapter(Protocol):
    def ready(self) -> bool: ...

    def listing(self) -> Dict[str, Any]: ...

    def frame(self, camera_id: str) -> Optional[Dict[str, Any]]: ...

    def stream_uri(self, camera_id: str) -> Optional[str]: ...
