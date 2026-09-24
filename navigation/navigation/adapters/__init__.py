"""厂家 Adapter 合同。能力层只认这一套，不认厂家 HTTP / SDK。

goto 是同步的：堵到到站或失败才返回。HTTP 层用后台线程包一层，所以 /goto 能马上回 ACCEPTED。
"""

from __future__ import annotations

from typing import Any, Dict, Protocol


class NavigationAdapter(Protocol):
    def ready(self) -> bool: ...

    def snapshot(self) -> Dict[str, Any]: ...

    def goto(
        self,
        *,
        station_id: str,
        idempotency_key: str,
        timeout_sec: float,
    ) -> Dict[str, Any]: ...

    def stop(self) -> Dict[str, Any]: ...

    def load_map(self, map_id: str) -> Dict[str, Any]: ...

    def apply_map(self, map_id: str, stations: list[Dict[str, Any]]) -> Dict[str, Any]: ...
