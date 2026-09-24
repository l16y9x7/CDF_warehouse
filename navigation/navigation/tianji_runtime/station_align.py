"""Station centimeter-align policy and executor.

S2NavAdapter calls run_align() after patrol arrive, before GoToStation
reports ARRIVED. HTTP / nav_goto.py are unchanged.
Standalone tests: scripts/run_station_align.sh.

Vendor MoveRelativeCmd.orientation.w is yaw in radians (not a quaternion).
w=0 means no rotation. Never send cos(yaw/2).

Three independent checks, in this order. Each: skip / correct / reject.
  1. yaw      |dyaw| > 2°  and <= 20°  → rotate
  2. lateral  |dy|   > 2 cm and <= 25 cm → strafe
  3. forward  |dx|   > 2 cm and <= 25 cm → drive
Need that axis → send that move; inside deadband → skip that axis.
At most two passes of the three checks (forward may leave a leftover lateral).
One vendor goal never mixes x, y, and yaw.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import List, Optional, Protocol, Tuple

TWO_CM = 0.02
TWO_DEG = math.radians(2.0)
TWENTY_DEG = math.radians(20.0)


def wrap_angle(rad: float) -> float:
    return math.atan2(math.sin(rad), math.cos(rad))


@dataclass(frozen=True)
class AlignConfig:
    xy_tol_m: float = TWO_CM
    yaw_tol_rad: float = TWO_DEG
    xy_max_m: float = 0.25
    yaw_max_rad: float = TWENTY_DEG
    min_confidence: int = 70
    translate_timeout_sec: float = 12.0
    rotate_timeout_sec: float = 8.0
    mode_wait_sec: float = 3.0
    unpark_settle_sec: float = 1.0
    restore_parking: bool = True
    restore_auto: bool = True
    max_passes: int = 2


@dataclass(frozen=True)
class Pose2D:
    x: float
    y: float
    yaw: float
    confidence: int = 100


@dataclass(frozen=True)
class BodyError:
    dx: float
    dy: float
    dyaw: float
    plane_m: float


@dataclass(frozen=True)
class AlignDecision:
    action: str  # skip | align | reject
    need_dx: bool
    need_dy: bool
    need_yaw: bool
    dx: float
    dy: float
    dyaw: float
    reason: str

    @property
    def need_xy(self) -> bool:
        return self.need_dx or self.need_dy


@dataclass
class AlignResult:
    status: str  # skipped | aligned | rejected | blocked | failed
    message: str = ""
    before: Optional[BodyError] = None
    after: Optional[BodyError] = None
    moves: List[str] = field(default_factory=list)
    restored_auto: bool = False
    restored_parking: bool = False

    @property
    def ok(self) -> bool:
        return self.status in ("skipped", "aligned")


class AlignHardware(Protocol):
    def get_pose(self) -> Pose2D: ...

    def check_safe(self) -> Tuple[bool, str]: ...

    def set_control_mode(self, mode: int) -> None: ...

    def set_parking(self, enable: bool) -> None: ...

    def wait_soft_auto(self, want_auto: bool, timeout_sec: float) -> None: ...

    def settle(self, sec: float) -> None: ...

    def move_relative(
        self, dx: float, dy: float, dyaw: float, timeout_sec: float
    ) -> None: ...


def map_error_to_body(target: Pose2D, current: Pose2D) -> BodyError:
    """Map-frame target minus current, rotated into the robot frame.

    dx > 0 forward, dy > 0 left, dyaw > 0 left turn. Yaw is wrapped to ±pi.
    """
    mx = float(target.x) - float(current.x)
    my = float(target.y) - float(current.y)
    yaw = float(current.yaw)
    c = math.cos(yaw)
    s = math.sin(yaw)
    dx = mx * c + my * s
    dy = -mx * s + my * c
    dyaw = wrap_angle(float(target.yaw) - yaw)
    return BodyError(dx=dx, dy=dy, dyaw=dyaw, plane_m=math.hypot(dx, dy))


def classify_axis(value: float, tol: float, max_v: float) -> str:
    """skip if |v| <= tol, correct if tol < |v| <= max, else reject."""
    mag = abs(float(value))
    if mag > max_v:
        return "reject"
    if mag > tol:
        return "correct"
    return "skip"


def translation_steps(
    dx: float, dy: float, *, axis_eps: float = 0.005
) -> List[Tuple[float, float]]:
    """Axis-aligned steps: lateral first, then forward/back. Never both in one goal."""
    steps: List[Tuple[float, float]] = []
    if abs(dy) >= axis_eps:
        steps.append((0.0, float(dy)))
    if abs(dx) >= axis_eps:
        steps.append((float(dx), 0.0))
    if not steps and math.hypot(dx, dy) > 0.0:
        steps.append((float(dx), float(dy)))
    return steps


def decide(err: BodyError, cfg: AlignConfig) -> AlignDecision:
    """Decide skip / align / reject per axis. Does not touch the chassis."""
    dx = float(err.dx)
    dy = float(err.dy)
    dyaw = wrap_angle(float(err.dyaw))
    abs_dx = abs(dx)
    abs_dy = abs(dy)
    abs_yaw = abs(dyaw)

    if abs_dx > cfg.xy_max_m or abs_dy > cfg.xy_max_m or abs_yaw > cfg.yaw_max_rad:
        return AlignDecision(
            action="reject",
            need_dx=False,
            need_dy=False,
            need_yaw=False,
            dx=0.0,
            dy=0.0,
            dyaw=0.0,
            reason=(
                f"error too large for relative move "
                f"(dx={dx*100:+.1f}cm dy={dy*100:+.1f}cm "
                f"yaw={math.degrees(dyaw):+.1f}°, "
                f"max {cfg.xy_max_m*100:.0f}cm / {math.degrees(cfg.yaw_max_rad):.0f}°)"
            ),
        )

    # Strict: exactly 2 cm / 2° does not move that axis.
    need_dx = abs_dx > cfg.xy_tol_m
    need_dy = abs_dy > cfg.xy_tol_m
    need_yaw = abs_yaw > cfg.yaw_tol_rad
    if not need_dx and not need_dy and not need_yaw:
        return AlignDecision(
            action="skip",
            need_dx=False,
            need_dy=False,
            need_yaw=False,
            dx=0.0,
            dy=0.0,
            dyaw=0.0,
            reason=(
                f"inside deadband "
                f"(|dx|={abs_dx*100:.1f}cm |dy|={abs_dy*100:.1f}cm "
                f"|yaw|={math.degrees(abs_yaw):.1f}°; "
                f"tol {cfg.xy_tol_m*100:.0f}cm / {math.degrees(cfg.yaw_tol_rad):.0f}°)"
            ),
        )

    return AlignDecision(
        action="align",
        need_dx=need_dx,
        need_dy=need_dy,
        need_yaw=need_yaw,
        dx=dx if need_dx else 0.0,
        dy=dy if need_dy else 0.0,
        dyaw=dyaw if need_yaw else 0.0,
        reason=(
            f"correct "
            f"dx={dx*100:+.1f}cm dy={dy*100:+.1f}cm "
            f"dyaw={math.degrees(dyaw):+.1f}° "
            f"(check yaw, then lateral, then forward)"
        ),
    )


def run_align(
    hw: AlignHardware,
    target: Pose2D,
    cfg: Optional[AlignConfig] = None,
    log=print,
) -> AlignResult:
    """Run CHECK then ALIGN. Restores auto+parking if this call changed mode.

    After unpark, check three axes in order (yaw, lateral, forward).
    Each axis is skip / correct / reject on the pose at that check.
    """
    cfg = cfg or AlignConfig()
    mode_changed = False
    unparked = False
    moves: List[str] = []
    before: Optional[BodyError] = None
    result: Optional[AlignResult] = None

    def _fail(status: str, message: str, after: Optional[BodyError] = None) -> AlignResult:
        return AlignResult(
            status=status,
            message=message,
            before=before,
            after=after,
            moves=list(moves),
        )

    def _err() -> BodyError:
        return map_error_to_body(target, hw.get_pose())

    try:
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

        before = map_error_to_body(target, pose)
        decision = decide(before, cfg)
        log(f"align check: {decision.reason}")
        if decision.action == "skip":
            result = AlignResult(status="skipped", message=decision.reason, before=before, after=before)
            return result
        if decision.action == "reject":
            result = _fail("rejected", decision.reason, after=before)
            return result

        log("align mode: soft-manual")
        hw.set_control_mode(1)
        mode_changed = True
        hw.wait_soft_auto(want_auto=False, timeout_sec=cfg.mode_wait_sec)
        log("align parking: off")
        hw.set_parking(False)
        unparked = True
        hw.settle(cfg.unpark_settle_sec)

        err = _err()
        decision = decide(err, cfg)
        log(f"align after mode switch: {decision.reason}")
        if decision.action == "skip":
            result = AlignResult(
                status="skipped",
                message="inside deadband after mode switch",
                before=before,
                after=err,
                moves=moves,
            )
            return result
        if decision.action == "reject":
            result = _fail("rejected", decision.reason, after=err)
            return result

        axes = (
            (
                "yaw",
                lambda e: e.dyaw,
                lambda v: (0.0, 0.0, v, cfg.rotate_timeout_sec),
                lambda v: f"|dyaw|={abs(math.degrees(v)):.1f}°",
                cfg.yaw_tol_rad,
                cfg.yaw_max_rad,
            ),
            (
                "lateral",
                lambda e: e.dy,
                lambda v: (0.0, v, 0.0, cfg.translate_timeout_sec),
                lambda v: f"dy={v*100:+.1f}cm",
                cfg.xy_tol_m,
                cfg.xy_max_m,
            ),
            (
                "forward",
                lambda e: e.dx,
                lambda v: (v, 0.0, 0.0, cfg.translate_timeout_sec),
                lambda v: f"dx={v*100:+.1f}cm",
                cfg.xy_tol_m,
                cfg.xy_max_m,
            ),
        )
        last_after: Optional[BodyError] = None
        for pass_i in range(1, max(int(cfg.max_passes), 1) + 1):
            log(f"align pass {pass_i}")
            moved = False
            for name, getter, to_cmd, fmt, tol, max_v in axes:
                err = _err()
                value = getter(err)
                kind = classify_axis(value, tol, max_v)
                log(f"align {name}: {kind} {fmt(value)}")
                if kind == "reject":
                    result = _fail(
                        "failed",
                        f"{name} {fmt(value)} over max",
                        after=err,
                    )
                    return result
                if kind == "correct":
                    dx, dy, dyaw, timeout = to_cmd(value)
                    hw.move_relative(dx, dy, dyaw, timeout)
                    moves.append(f"{name}:{value:.4f}")
                    hw.settle(cfg.unpark_settle_sec)
                    moved = True
            last_after = _err()
            still = decide(last_after, cfg)
            if still.action == "skip":
                result = AlignResult(
                    status="aligned",
                    message=f"inside deadband after {len(moves)} move(s)",
                    before=before,
                    after=last_after,
                    moves=moves,
                )
                return result
            if still.action == "reject":
                result = _fail(
                    "failed",
                    f"after move still too far: {still.reason}",
                    after=last_after,
                )
                return result
            if not moved:
                break
        result = _fail(
            "failed",
            f"after {cfg.max_passes} pass(es) still outside deadband: {still.reason}",
            after=last_after,
        )
        return result
    except Exception as exc:  # noqa: BLE001
        log(f"align error: {exc}")
        after = None
        try:
            after = map_error_to_body(target, hw.get_pose())
        except Exception:  # noqa: BLE001
            pass
        result = _fail("failed", str(exc), after=after)
        return result
    finally:
        if result is None:
            result = _fail("failed", "align aborted")
        if mode_changed and cfg.restore_auto:
            try:
                log("align restore: soft-auto")
                hw.set_control_mode(0)
                hw.wait_soft_auto(want_auto=True, timeout_sec=cfg.mode_wait_sec)
                result.restored_auto = True
            except Exception as exc:  # noqa: BLE001
                log(f"align restore auto failed: {exc}")
                result.message = f"{result.message}; restore auto failed: {exc}"
                if result.status == "aligned":
                    result.status = "failed"
        if unparked and cfg.restore_parking:
            try:
                log("align restore: parking on")
                hw.set_parking(True)
                result.restored_parking = True
            except Exception as exc:  # noqa: BLE001
                log(f"align restore parking failed: {exc}")
                result.message = f"{result.message}; restore parking failed: {exc}"
                if result.status == "aligned":
                    result.status = "failed"
