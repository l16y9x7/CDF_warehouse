"""Navigation facade state machine (vendor-agnostic)."""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Optional, Set, Tuple

from navigation.tianji_runtime.constants import ErrorCode, Mode, NavState


@dataclass
class FacadeSnapshot:
    nav_state: NavState = NavState.UNREADY
    mode: Mode = Mode.NONE
    request_id: str = ""
    active_station_id: str = ""
    map_id: str = ""
    map_name: str = ""
    mapping_status: int = 1
    holding: bool = False
    localized: bool = False
    paused: bool = False
    error_code: int = 0
    error_msg: str = ""
    progress: float = 0.0
    hold_reason: str = ""
    resume_state: Optional[NavState] = None


# Allowed transitions: from_state -> set of to_states
_ALLOWED: dict[NavState, Set[NavState]] = {
    NavState.UNREADY: {NavState.IDLE, NavState.ERROR},
    NavState.IDLE: {
        NavState.MAPPING,
        NavState.LOCALIZING,
        NavState.READY,
        NavState.HOLDING,
        NavState.ERROR,
    },
    NavState.MAPPING: {NavState.SAVING, NavState.IDLE, NavState.ERROR},
    NavState.SAVING: {NavState.IDLE, NavState.READY, NavState.ERROR},
    NavState.LOCALIZING: {NavState.READY, NavState.ERROR},
    NavState.READY: {
        NavState.NAVIGATING,
        NavState.LOCALIZING,
        NavState.HOLDING,
        NavState.MAPPING,
        NavState.IDLE,
        NavState.ERROR,
    },
    NavState.NAVIGATING: {
        NavState.ARRIVED,
        NavState.PAUSED,
        NavState.READY,
        NavState.ERROR,
    },
    NavState.ARRIVED: {NavState.READY, NavState.HOLDING, NavState.NAVIGATING},
    NavState.PAUSED: {NavState.NAVIGATING, NavState.READY, NavState.ERROR},
    NavState.HOLDING: {NavState.READY, NavState.IDLE, NavState.ERROR},
    NavState.ERROR: {NavState.READY, NavState.IDLE, NavState.UNREADY},
}


@dataclass
class NavStateMachine:
    snap: FacadeSnapshot = field(default_factory=FacadeSnapshot)

    @property
    def state(self) -> NavState:
        return self.snap.nav_state

    def can_transition(self, to: NavState) -> bool:
        return to in _ALLOWED.get(self.snap.nav_state, set())

    def transition(self, to: NavState) -> Tuple[bool, int, str]:
        if to == self.snap.nav_state:
            return True, ErrorCode.OK, ""
        if not self.can_transition(to):
            return (
                False,
                ErrorCode.INVALID_STATE,
                f"cannot transition {self.snap.nav_state.value} -> {to.value}",
            )
        self.snap.nav_state = to
        self.snap.paused = to == NavState.PAUSED
        self.snap.holding = to == NavState.HOLDING
        return True, ErrorCode.OK, ""

    def require_states(self, *states: NavState) -> Tuple[bool, int, str]:
        if self.snap.nav_state in states:
            return True, ErrorCode.OK, ""
        allowed = ",".join(s.value for s in states)
        return (
            False,
            ErrorCode.INVALID_STATE,
            f"nav_state={self.snap.nav_state.value} not in [{allowed}]",
        )

    def set_error(self, code: int, msg: str) -> None:
        self.snap.error_code = code
        self.snap.error_msg = msg
        self.snap.nav_state = NavState.ERROR
        self.snap.paused = False

    def clear_error(self, fallback: NavState = NavState.READY) -> Tuple[bool, int, str]:
        ok, code, msg = self.require_states(NavState.ERROR)
        if not ok:
            return ok, code, msg
        self.snap.error_code = 0
        self.snap.error_msg = ""
        target = fallback if self.snap.localized else NavState.IDLE
        self.snap.nav_state = target
        self.snap.holding = False
        self.snap.paused = False
        return True, ErrorCode.OK, ""
