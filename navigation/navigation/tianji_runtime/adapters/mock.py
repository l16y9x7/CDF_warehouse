"""Mock chassis/navigation adapter for simulation and unit tests."""

from __future__ import annotations

import math
import time
import uuid
from typing import Optional

from navigation.tianji_runtime.adapters.base import (
    AdapterResult,
    HealthInfo,
    NavGoal,
    Pose2D,
)
from navigation.tianji_runtime.constants import ErrorCode, ResultCode
from navigation.tianji_runtime.stations import Station


def _wrap_angle(angle: float) -> float:
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle


class MockNavAdapter:
    """Simulates pose motion with constant linear/angular speed."""

    def __init__(
        self,
        *,
        linear_speed: float = 0.6,
        angular_speed: float = 0.8,
        mapping_duration_sec: float = 2.0,
        inject_result: str = "",
    ) -> None:
        self.linear_speed = max(linear_speed, 0.01)
        self.angular_speed = max(angular_speed, 0.01)
        self.mapping_duration_sec = max(mapping_duration_sec, 0.1)
        self.inject_result = inject_result

        self._pose = Pose2D()
        self._goal: Optional[NavGoal] = None
        self._nav_started_at = 0.0
        self._paused = False
        self._holding = False
        self._mapping = False
        self._mapping_started_at = 0.0
        self._mapping_name = ""
        self._map_id = "mock-map-1"
        self._map_name = "retail_default"
        self._localized = False

    def get_health(self) -> HealthInfo:
        return HealthInfo(
            runtime_ready=True,
            mapping_ready=True,
            navigation_ready=True,
            localization_ready=True,
            motion_ready=not self._holding,
            detail="mock adapter",
        )

    def get_pose(self) -> Pose2D:
        pose = Pose2D(
            x=self._pose.x,
            y=self._pose.y,
            yaw=self._pose.yaw,
            confidence=100 if self._localized else 0,
            map_id=self._map_id if self._localized else "",
        )
        return pose

    def start_mapping(self, map_name: str = "") -> AdapterResult:
        if self._goal is not None:
            return AdapterResult(
                error_code=ErrorCode.INVALID_STATE,
                error_msg="cannot map while navigating",
            )
        self._mapping = True
        self._mapping_started_at = time.monotonic()
        self._mapping_name = map_name or f"map_{int(time.time())}"
        self._localized = False
        return AdapterResult(session_id=str(uuid.uuid4()), map_name=self._mapping_name)

    def tick_stop_mapping(self, map_name: str = "", dt: float = 0.1) -> AdapterResult:
        if not self._mapping:
            # Treat as already finishing
            progress = 1.0
        else:
            elapsed = time.monotonic() - self._mapping_started_at
            progress = min(1.0, elapsed / self.mapping_duration_sec)
        if progress < 1.0:
            return AdapterResult(progress=progress, result_code="")
        name = map_name or self._mapping_name or self._map_name
        self._mapping = False
        self._map_name = name
        self._map_id = f"mock-{name}"
        return AdapterResult(
            progress=1.0,
            result_code=ResultCode.DONE.value,
            map_id=self._map_id,
            map_name=self._map_name,
        )

    def cancel_mapping(self, reason: str = "") -> AdapterResult:
        self._mapping = False
        return AdapterResult(error_msg=reason)

    def load_map(self, map_id: str = "", map_name: str = "") -> AdapterResult:
        if map_id:
            self._map_id = map_id
        if map_name:
            self._map_name = map_name
        return AdapterResult(map_id=self._map_id, map_name=self._map_name)

    def relocate(
        self,
        *,
        station: Optional[Station] = None,
        x: float = 0.0,
        y: float = 0.0,
        yaw: float = 0.0,
    ) -> AdapterResult:
        if station is not None:
            x, y, yaw = station.x, station.y, station.yaw
        self._pose = Pose2D(x=x, y=y, yaw=yaw, confidence=100, map_id=self._map_id)
        self._localized = True
        return AdapterResult(pose=self.get_pose())

    def start_nav(self, goal: NavGoal) -> AdapterResult:
        if self._holding:
            return AdapterResult(
                error_code=ErrorCode.INVALID_STATE,
                error_msg="holding chassis",
            )
        if self._mapping:
            return AdapterResult(
                error_code=ErrorCode.INVALID_STATE,
                error_msg="mapping in progress",
            )
        self._goal = goal
        self._nav_started_at = time.monotonic()
        self._paused = False
        if goal.relative:
            goal.x = self._pose.x + goal.x
            goal.y = self._pose.y + goal.y
            goal.yaw = _wrap_angle(self._pose.yaw + goal.yaw)
            goal.relative = False
            self._goal = goal
        return AdapterResult()

    def tick_nav(self, dt: float = 0.1) -> AdapterResult:
        if self._goal is None:
            return AdapterResult(
                error_code=ErrorCode.NOT_READY,
                error_msg="no active goal",
                result_code=ResultCode.ERROR.value,
            )
        if self.inject_result == ResultCode.BLOCKED.value:
            return AdapterResult(
                error_code=ErrorCode.UNREACHABLE,
                error_msg="injected blocked",
                result_code=ResultCode.BLOCKED.value,
                pose=self.get_pose(),
            )
        if self._paused:
            return AdapterResult(progress=0.0, pose=self.get_pose())

        goal = self._goal
        elapsed = time.monotonic() - self._nav_started_at
        if goal.timeout_sec > 0 and elapsed > goal.timeout_sec:
            self._goal = None
            return AdapterResult(
                error_code=ErrorCode.TIMEOUT,
                error_msg="navigation timeout",
                result_code=ResultCode.TIMEOUT.value,
                pose=self.get_pose(),
            )

        dx = goal.x - self._pose.x
        dy = goal.y - self._pose.y
        dist = math.hypot(dx, dy)
        speed = max(goal.max_speed, 0.05)
        step = speed * dt

        if dist > goal.arrive_tol_m:
            if dist <= step:
                self._pose.x = goal.x
                self._pose.y = goal.y
            else:
                self._pose.x += dx / dist * step
                self._pose.y += dy / dist * step
            remaining = math.hypot(goal.x - self._pose.x, goal.y - self._pose.y)
            total = max(dist, remaining, 1e-6)
            progress = max(0.0, min(1.0, 1.0 - remaining / (remaining + step + 1e-6)))
            return AdapterResult(
                progress=progress,
                pose=self.get_pose(),
                result_code="",
            )

        dyaw = _wrap_angle(goal.yaw - self._pose.yaw)
        yaw_step = self.angular_speed * dt
        if abs(dyaw) > goal.yaw_tol_rad:
            if abs(dyaw) <= yaw_step:
                self._pose.yaw = goal.yaw
            else:
                self._pose.yaw = _wrap_angle(
                    self._pose.yaw + math.copysign(yaw_step, dyaw)
                )
            return AdapterResult(progress=0.95, pose=self.get_pose(), result_code="")

        self._pose.x = goal.x
        self._pose.y = goal.y
        self._pose.yaw = goal.yaw
        self._goal = None
        return AdapterResult(
            progress=1.0,
            result_code=ResultCode.ARRIVED.value,
            pose=self.get_pose(),
        )

    def cancel_nav(self, reason: str = "") -> AdapterResult:
        self._goal = None
        self._paused = False
        return AdapterResult(
            result_code=ResultCode.CANCELLED.value,
            error_msg=reason,
            pose=self.get_pose(),
        )

    def pause_nav(self) -> AdapterResult:
        self._paused = True
        return AdapterResult(pose=self.get_pose())

    def resume_nav(self) -> AdapterResult:
        self._paused = False
        return AdapterResult(pose=self.get_pose())

    def hold(self, reason: str = "") -> AdapterResult:
        self._holding = True
        self._paused = True
        return AdapterResult(error_msg=reason, pose=self.get_pose())

    def release_hold(self) -> AdapterResult:
        self._holding = False
        self._paused = False
        return AdapterResult(pose=self.get_pose())

    def run_body_moves(self, steps, timeout_sec: float = 12.0) -> AdapterResult:
        del timeout_sec
        if self._holding:
            return AdapterResult(
                error_code=ErrorCode.INVALID_STATE,
                error_msg="holding chassis",
            )
        if self._goal is not None:
            return AdapterResult(
                error_code=ErrorCode.INVALID_STATE,
                error_msg="navigating",
            )
        yaw = float(self._pose.yaw)
        cosine = math.cos(yaw)
        sine = math.sin(yaw)
        for dx, dy in steps or ():
            self._pose.x += float(dx) * cosine - float(dy) * sine
            self._pose.y += float(dx) * sine + float(dy) * cosine
        return AdapterResult(pose=self.get_pose())

    def align_to_station(self, station: Station) -> AdapterResult:
        self._pose.x = float(station.x)
        self._pose.y = float(station.y)
        self._pose.yaw = float(station.yaw)
        return AdapterResult(pose=self.get_pose())
