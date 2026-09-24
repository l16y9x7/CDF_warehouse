"""Vendor-agnostic adapter protocol and shared result types."""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Optional, Protocol, runtime_checkable

from navigation.tianji_runtime.constants import ErrorCode, ResultCode
from navigation.tianji_runtime.stations import Station


@dataclass
class Pose2D:
    x: float = 0.0
    y: float = 0.0
    yaw: float = 0.0
    confidence: int = 100
    map_id: str = ""


@dataclass
class AdapterResult:
    error_code: int = ErrorCode.OK
    error_msg: str = ""
    result_code: str = ResultCode.DONE.value
    pose: Optional[Pose2D] = None
    progress: float = 0.0
    map_id: str = ""
    map_name: str = ""
    session_id: str = ""

    @property
    def ok(self) -> bool:
        return self.error_code == ErrorCode.OK


@dataclass
class NavGoal:
    request_id: str = ""
    station_id: str = ""
    x: float = 0.0
    y: float = 0.0
    yaw: float = 0.0
    arrive_tol_m: float = 0.08
    yaw_tol_rad: float = 0.17
    max_speed: float = 0.5
    timeout_sec: float = 120.0
    relative: bool = False


@dataclass
class HealthInfo:
    runtime_ready: bool = False
    mapping_ready: bool = False
    navigation_ready: bool = False
    localization_ready: bool = False
    motion_ready: bool = False
    detail: str = ""

    @property
    def ready(self) -> bool:
        return all(
            (
                self.runtime_ready,
                self.mapping_ready,
                self.navigation_ready,
                self.localization_ready,
                self.motion_ready,
            )
        )


@runtime_checkable
class NavAdapter(Protocol):
    def get_health(self) -> HealthInfo: ...

    def get_pose(self) -> Pose2D: ...

    def start_mapping(self, map_name: str = "") -> AdapterResult: ...

    def tick_stop_mapping(self, map_name: str = "", dt: float = 0.1) -> AdapterResult: ...

    def cancel_mapping(self, reason: str = "") -> AdapterResult: ...

    def load_map(self, map_id: str = "", map_name: str = "") -> AdapterResult: ...

    def relocate(
        self,
        *,
        station: Optional[Station] = None,
        x: float = 0.0,
        y: float = 0.0,
        yaw: float = 0.0,
    ) -> AdapterResult: ...

    def start_nav(self, goal: NavGoal) -> AdapterResult: ...

    def tick_nav(self, dt: float = 0.1) -> AdapterResult: ...

    def cancel_nav(self, reason: str = "") -> AdapterResult: ...

    def pause_nav(self) -> AdapterResult: ...

    def resume_nav(self) -> AdapterResult: ...

    def hold(self, reason: str = "") -> AdapterResult: ...

    def release_hold(self) -> AdapterResult: ...
