"""Grasp nudge policy: 5 cm body-frame steps, max 2, then return.

HTTP and the facade share this module. It does not talk to the chassis.
Body frame matches station_align: dx>0 forward, dy>0 left.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Dict, List, Optional, Tuple

from navigation.tianji_runtime.station_align import (
    AlignConfig,
    AlignHardware,
    AlignResult,
    BodyError,
)

STEP_M = 0.05
MAX_APPROACHES = 2
XY_MAX_M = 0.25
MIN_CONFIDENCE = 70

DIRECTIONS: Dict[str, Tuple[float, float]] = {
    "forward": (STEP_M, 0.0),
    "back": (-STEP_M, 0.0),
    "left": (0.0, STEP_M),
    "right": (0.0, -STEP_M),
}


@dataclass(frozen=True)
class NudgeRequest:
    action: str
    direction: str = ""


@dataclass(frozen=True)
class NudgePlan:
    ok: bool
    message: str = ""
    error_http: str = ""
    dx: float = 0.0
    dy: float = 0.0
    station_id: str = ""


@dataclass
class NudgeSession:
    station_id: str = ""
    approaches: List[str] = field(default_factory=list)

    @property
    def count(self) -> int:
        return len(self.approaches)

    @property
    def off_station(self) -> bool:
        return self.count > 0

    def reverse_delta(self) -> Tuple[float, float]:
        sx = 0.0
        sy = 0.0
        for name in self.approaches:
            dx, dy = DIRECTIONS[name]
            sx += dx
            sy += dy
        return (-sx, -sy)

    def commit_approach(self, direction: str, station_id: str) -> None:
        self.station_id = str(station_id or "").strip()
        self.approaches.append(str(direction))

    def clear(self) -> None:
        self.station_id = ""
        self.approaches.clear()


def parse_nudge_body(body: object) -> Tuple[Optional[NudgeRequest], str]:
    if not isinstance(body, dict):
        return None, "INVALID_REQUEST"
    action = str(body.get("action", "")).strip().lower()
    if action == "approach":
        if set(body) != {"action", "direction"}:
            return None, "INVALID_REQUEST"
        direction = str(body.get("direction", "")).strip().lower()
        if direction not in DIRECTIONS:
            return None, "INVALID_REQUEST"
        return NudgeRequest(action=action, direction=direction), ""
    if action == "return":
        if set(body) != {"action"}:
            return None, "INVALID_REQUEST"
        return NudgeRequest(action=action), ""
    return None, "INVALID_REQUEST"


def plan_approach(
    session: NudgeSession,
    direction: str,
    station_id: str,
    body_error: Optional[BodyError],
    *,
    xy_max_m: float = XY_MAX_M,
    max_count: int = MAX_APPROACHES,
) -> NudgePlan:
    direction = str(direction or "").strip().lower()
    station_id = str(station_id or "").strip()
    if direction not in DIRECTIONS:
        return NudgePlan(ok=False, error_http="INVALID_REQUEST", message="invalid direction")
    if not station_id:
        return NudgePlan(ok=False, error_http="NOT_AT_STATION", message="not at station")
    if session.count >= max_count:
        return NudgePlan(
            ok=False,
            error_http="NUDGE_LIMIT_EXCEEDED",
            message="nudge limit exceeded",
        )
    dx, dy = DIRECTIONS[direction]
    if body_error is not None:
        new_dx = float(body_error.dx) - dx
        new_dy = float(body_error.dy) - dy
        if abs(new_dx) > xy_max_m or abs(new_dy) > xy_max_m:
            return NudgePlan(
                ok=False,
                error_http="NUDGE_LIMIT_EXCEEDED",
                message="nudge limit exceeded: over 25cm from station",
            )
    return NudgePlan(ok=True, dx=dx, dy=dy, station_id=station_id)


def plan_return(session: NudgeSession) -> NudgePlan:
    if session.count <= 0:
        return NudgePlan(ok=False, error_http="NOT_AT_STATION", message="not approached")
    dx, dy = session.reverse_delta()
    return NudgePlan(ok=True, dx=dx, dy=dy, station_id=session.station_id)


def run_manual_body_moves(
    hw: AlignHardware,
    steps: List[Tuple[float, float]],
    cfg: Optional[AlignConfig] = None,
    log=print,
) -> AlignResult:
    """Soft-manual, one-axis translations, then restore auto+parking.

    Same restore contract as run_align. Does not change run_align itself.
    """
    cfg = cfg or AlignConfig()
    mode_changed = False
    unparked = False
    moves: List[str] = []
    result: Optional[AlignResult] = None

    def _fail(status: str, message: str) -> AlignResult:
        return AlignResult(status=status, message=message, moves=list(moves))

    try:
        if not steps:
            result = AlignResult(status="skipped", message="no body move")
            return result

        safe, why = hw.check_safe()
        if not safe:
            result = _fail("blocked", why)
            return result

        pose = hw.get_pose()
        if int(pose.confidence) < cfg.min_confidence:
            result = _fail(
                "blocked",
                f"pose confidence {pose.confidence} < {cfg.min_confidence}",
            )
            return result

        log("nudge mode: soft-manual")
        hw.set_control_mode(1)
        mode_changed = True
        hw.wait_soft_auto(want_auto=False, timeout_sec=cfg.mode_wait_sec)
        log("nudge parking: off")
        hw.set_parking(False)
        unparked = True
        hw.settle(cfg.unpark_settle_sec)

        for dx, dy in steps:
            if abs(dx) < 1e-6 and abs(dy) < 1e-6:
                continue
            hw.move_relative(float(dx), float(dy), 0.0, cfg.translate_timeout_sec)
            moves.append(f"body:{dx:.4f},{dy:.4f}")
            hw.settle(cfg.unpark_settle_sec)

        result = AlignResult(
            status="aligned",
            message=f"nudge moved {len(moves)} step(s)",
            moves=moves,
        )
        return result
    except Exception as exc:  # noqa: BLE001
        log(f"nudge move error: {exc}")
        result = _fail("failed", str(exc))
        return result
    finally:
        if result is None:
            result = _fail("failed", "nudge move aborted")
        if mode_changed and cfg.restore_auto:
            try:
                log("nudge restore: soft-auto")
                hw.set_control_mode(0)
                hw.wait_soft_auto(want_auto=True, timeout_sec=cfg.mode_wait_sec)
                result.restored_auto = True
            except Exception as exc:  # noqa: BLE001
                log(f"nudge restore auto failed: {exc}")
                result.message = f"{result.message}; restore auto failed: {exc}"
                if result.status == "aligned":
                    result.status = "failed"
        if unparked and cfg.restore_parking:
            try:
                log("nudge restore: parking on")
                hw.set_parking(True)
                result.restored_parking = True
            except Exception as exc:  # noqa: BLE001
                log(f"nudge restore parking failed: {exc}")
                result.message = f"{result.message}; restore parking failed: {exc}"
                if result.status == "aligned":
                    result.status = "failed"
