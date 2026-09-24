"""Independent torso-plane / arm-angle planner. No SDK import or motion calls.

All transforms and FK/IK poses use one common frame (normally chassis_link),
millimetres and degrees. The protection plane is attached to Chest_link.
Returned paths have sampled kinematic validation, not full-body collision
certification or a guarantee about a controller's unsampled interpolation.
"""
from __future__ import annotations

import math
import time
from dataclasses import dataclass
from typing import Callable, Protocol, Sequence

import numpy as np

if __package__:
    from .geometry import inverse, rigid
    from .planner import interpolate, rotation_log
else:
    from geometry import inverse, rigid
    from planner import interpolate, rotation_log


class ArmKinematics(Protocol):
    # Return None only for an infeasible IK candidate; transport/model errors raise.
    def ik(self, flange_world_mm: np.ndarray, arm_angle_deg: float, seed_deg: np.ndarray) -> np.ndarray | None: ...
    def fk(self, joints_deg: np.ndarray) -> tuple[np.ndarray, np.ndarray]: ...
    def within_limits(self, joints_deg: np.ndarray) -> bool: ...


def _finite(value, label: str, minimum=None, maximum=None) -> float:
    if isinstance(value, (bool, np.bool_)) or not np.isscalar(value):
        raise ValueError(f"{label}: expected a finite number")
    number = float(value)
    if not math.isfinite(number) or (minimum is not None and number < minimum) or (maximum is not None and number > maximum):
        raise ValueError(f"{label}: invalid range or nonfinite value")
    return number


def _vector(value, size: int, label: str) -> np.ndarray:
    raw = np.asarray(value)
    if raw.dtype.kind not in "iuf":
        raise ValueError(f"{label}: expected numeric values, not booleans/strings/complex values")
    result = raw.astype(float)
    if result.shape != (size,) or not np.isfinite(result).all():
        raise ValueError(f"{label}: expected {size} finite values")
    return result.copy()


def _matrix_tuple(value) -> tuple[tuple[float, ...], ...]:
    return tuple(tuple(float(v) for v in row) for row in value)


@dataclass(frozen=True)
class PlaneMeasurement:
    elbow_torso_mm: tuple[float, float, float]
    distance_outside_plane_mm: float
    clearance_mm: float  # After subtracting elbow radius and margin.

    @property
    def safe(self) -> bool:
        return self.clearance_mm >= 0.0


@dataclass(frozen=True)
class TorsoPlane:
    """Right: y <= -offset-radius-margin; left: y >= offset+radius+margin.

    offset_mm is measured OUTWARD from the torso XZ plane. Increasing it is
    more restrictive. A radius of zero implements a point-only elbow proxy.
    """
    side: str
    offset_mm: float
    margin_mm: float = 10.0
    elbow_radius_mm: float = 0.0

    def __post_init__(self):
        if self.side not in ("right", "left"):
            raise ValueError("side must be right or left")
        for name in ("offset_mm", "margin_mm", "elbow_radius_mm"):
            object.__setattr__(self, name, _finite(getattr(self, name), name, 0))

    @property
    def outward_sign(self) -> int:
        return -1 if self.side == "right" else 1

    @property
    def plane_y_mm(self) -> float:
        return self.outward_sign * self.offset_mm

    def measure(self, elbow_world_mm, world_from_torso_mm) -> PlaneMeasurement:
        torso_from_world = inverse(rigid(world_from_torso_mm, "world_from_torso_mm"))
        elbow = _vector(elbow_world_mm, 3, "elbow")
        local = torso_from_world[:3, :3] @ elbow + torso_from_world[:3, 3]
        distance = float(self.outward_sign * local[1] - self.offset_mm)
        return PlaneMeasurement(tuple(local.tolist()), distance,
                                distance - self.elbow_radius_mm - self.margin_mm)


@dataclass(frozen=True)
class SearchOptions:
    scan_step_deg: float = 5.0
    max_arm_angle_change_deg: float = 90.0
    arm_angle_min_deg: float = -180.0
    arm_angle_max_deg: float = 180.0
    linear_sample_mm: float = 2.0
    rotation_sample_deg: float = 1.0
    arm_angle_sample_deg: float = 1.0
    max_joint_step_deg: float = 5.0
    joint_probe_step_deg: float = 0.5
    fk_position_tolerance_mm: float = 2.0
    fk_rotation_tolerance_deg: float = 1.0
    max_ik_calls: int = 20000
    max_fk_calls: int = 100000
    time_limit_seconds: float = 60.0

    def __post_init__(self):
        for name in ("scan_step_deg", "linear_sample_mm", "rotation_sample_deg", "arm_angle_sample_deg",
                     "max_joint_step_deg", "joint_probe_step_deg", "fk_position_tolerance_mm",
                     "fk_rotation_tolerance_deg", "time_limit_seconds"):
            number = _finite(getattr(self, name), name, 0)
            if number == 0:
                raise ValueError(f"{name} must be positive")
            object.__setattr__(self, name, number)
        for name, low, high in (("max_arm_angle_change_deg", 0, 180),
                                 ("arm_angle_min_deg", -180, 180), ("arm_angle_max_deg", -180, 180)):
            object.__setattr__(self, name, _finite(getattr(self, name), name, low, high))
        if self.arm_angle_min_deg >= self.arm_angle_max_deg or self.scan_step_deg > 180:
            raise ValueError("invalid arm-angle scan limits")
        if math.floor(self.max_arm_angle_change_deg / self.scan_step_deg) > 1000:
            raise ValueError("arm-angle scan would contain too many candidates")
        for name in ("max_ik_calls", "max_fk_calls"):
            value = getattr(self, name)
            if type(value) is not int or value <= 0:
                raise ValueError(f"{name} must be a positive integer")


def arm_angle_scan(current_deg: float, options: SearchOptions | None = None) -> tuple[float, ...]:
    """Current, current-5, current+5, current-10, current+10 ...; no angle wrapping."""
    options = options or SearchOptions()
    current = _finite(current_deg, "current arm angle", options.arm_angle_min_deg, options.arm_angle_max_deg)
    result = [current]
    for index in range(1, math.floor(options.max_arm_angle_change_deg / options.scan_step_deg) + 1):
        delta = index * options.scan_step_deg
        for offset in (-delta, delta):
            value = current + offset
            if options.arm_angle_min_deg <= value <= options.arm_angle_max_deg:
                result.append(value)
    return tuple(result)


@dataclass(frozen=True)
class GuardedWaypoint:
    flange_world_mm: tuple[tuple[float, ...], ...]
    joints_deg: tuple[float, ...]
    arm_angle_deg: float
    elbow_world_mm: tuple[float, float, float]
    clearance_mm: float
    segment_index: int


@dataclass(frozen=True)
class CandidateAttempt:
    arm_angle_deg: float
    reason: str
    segment_index: int = -1
    sample_index: int = -1
    clearance_mm: float | None = None


@dataclass(frozen=True)
class GuardedPlan:
    start_joints_deg: tuple[float, ...]
    start_arm_angle_deg: float
    world_from_torso_mm: tuple[tuple[float, ...], ...]
    plane: TorsoPlane
    selected_arm_angle_deg: float
    waypoints: tuple[GuardedWaypoint, ...]
    minimum_sampled_clearance_mm: float
    validation: str = "sampled_ik_fk_and_elbow_plane"


@dataclass(frozen=True)
class PlanningResult:
    status: str
    plan: GuardedPlan | None
    attempts: tuple[CandidateAttempt, ...]
    ik_calls: int
    fk_calls: int
    message: str


class _Rejected(Exception):
    def __init__(self, reason, segment=-1, sample=-1, clearance=None):
        self.reason, self.segment, self.sample, self.clearance = reason, segment, sample, clearance


class _Interrupted(Exception):
    pass


class _Budget:
    def __init__(self, options, cancelled):
        self.options, self.cancelled = options, cancelled
        self.deadline = time.monotonic() + options.time_limit_seconds
        self.ik_calls = self.fk_calls = 0

    def check(self):
        if self.cancelled and self.cancelled():
            raise _Interrupted("cancelled")
        if time.monotonic() >= self.deadline:
            raise _Interrupted("time_limit")

    def call(self, kind, function, *args):
        self.check()
        field = kind + "_calls"
        count = getattr(self, field)
        if count >= getattr(self.options, "max_" + field):
            raise _Interrupted("computation_limit")
        setattr(self, field, count + 1)
        value = function(*args)  # Do not turn network/model exceptions into "IK infeasible".
        self.check()
        return value


def _fk(kinematics, joints, budget):
    pose, elbow = budget.call("fk", kinematics.fk, joints.copy())
    return rigid(pose, "FK pose").copy(), _vector(elbow, 3, "FK elbow")


def plan_guarded_movel(
    kinematics: ArmKinematics,
    current_joints_deg: Sequence[float],
    current_arm_angle_deg: float,
    targets_world_mm: Sequence[np.ndarray],
    *,
    world_from_torso_mm: np.ndarray,
    plane: TorsoPlane,
    options: SearchOptions | None = None,
    cancelled: Callable[[], bool] | None = None,
    search_seed_arm_angle_deg: float | None = None,
    validate_candidate: Callable[[GuardedPlan], str | None] | None = None,
) -> PlanningResult:
    """Return the FIRST passing complete candidate, without dispatching motion.

    Torso is fixed during this arm plan. On the first Cartesian segment, the
    arm angle changes smoothly from its actual value to the candidate value;
    subsequent segments keep that value. This transition is checked too.
    A hold-pose first segment is allowed for a separately checked elbow change.
    IK None rejects a candidate; model/transport exceptions abort the call.
    search_seed_arm_angle_deg selects the first scanned candidate without
    replacing the measured start angle. validate_candidate can reject a whole
    candidate after sampling (e.g. native SDK checkPath); exceptions abort.
    """
    options = options or SearchOptions()
    current = _vector(current_joints_deg, 7, "current joints")
    torso = rigid(world_from_torso_mm, "world_from_torso_mm").copy()
    targets = tuple(rigid(p, "target pose").copy() for p in targets_world_mm)
    if not targets:
        raise ValueError("at least one target pose is required")
    current_arm_angle_deg = _finite(current_arm_angle_deg, "current arm angle", -180, 180)
    candidates = arm_angle_scan(current_arm_angle_deg if search_seed_arm_angle_deg is None
                                else search_seed_arm_angle_deg, options)
    budget = _Budget(options, cancelled)
    attempts = []

    def result(status, plan=None, message=""):
        return PlanningResult(status, plan, tuple(attempts), budget.ik_calls, budget.fk_calls, message)

    try:
        budget.check()
        if not kinematics.within_limits(current.copy()):
            return result("invalid_start", message="当前关节不在模型限位内")
        start_pose, start_elbow = _fk(kinematics, current, budget)
        start_measurement = plane.measure(start_elbow, torso)
        if not start_measurement.safe:
            return result("start_inside_protection", message=f"当前肘点已进入保护余量区，净余量 {start_measurement.clearance_mm:.3f} mm；不自动规划脱离运动")
        for candidate in candidates:
            budget.check()
            previous_pose, previous_joints = start_pose.copy(), current.copy()
            previous_angle = float(current_arm_angle_deg)
            waypoints = []
            minimum_clearance = start_measurement.clearance_mm
            try:
                for segment, goal in enumerate(targets):
                    distance = float(np.linalg.norm(goal[:3, 3] - previous_pose[:3, 3]))
                    turn = math.degrees(float(np.linalg.norm(rotation_log(goal[:3, :3] @ previous_pose[:3, :3].T))))
                    count = max(1, math.ceil(distance / options.linear_sample_mm),
                                math.ceil(turn / options.rotation_sample_deg),
                                math.ceil(abs(candidate - previous_angle) / options.arm_angle_sample_deg))
                    segment_start, angle_start = previous_pose.copy(), previous_angle
                    for index in range(1, count + 1):
                        fraction = index / count
                        pose = interpolate(segment_start, goal, fraction)
                        angle = angle_start + fraction * (candidate - angle_start)
                        solved = budget.call("ik", kinematics.ik, pose.copy(), angle, previous_joints.copy())
                        if solved is None:
                            raise _Rejected("ik_unreachable", segment, index)
                        joints = _vector(solved, 7, "IK joints")
                        if not kinematics.within_limits(joints.copy()):
                            raise _Rejected("joint_limit", segment, index)
                        maximum_step = float(np.max(np.abs(joints - previous_joints)))
                        if maximum_step > options.max_joint_step_deg:
                            raise _Rejected("joint_discontinuity", segment, index)
                        actual, elbow = _fk(kinematics, joints, budget)
                        position_error = float(np.linalg.norm(actual[:3, 3] - pose[:3, 3]))
                        rotation_error = math.degrees(float(np.linalg.norm(rotation_log(pose[:3, :3] @ actual[:3, :3].T))))
                        if position_error > options.fk_position_tolerance_mm or rotation_error > options.fk_rotation_tolerance_deg:
                            raise _Rejected("fk_pose_mismatch", segment, index)
                        measured = plane.measure(elbow, torso)
                        if not measured.safe:
                            raise _Rejected("elbow_plane", segment, index, measured.clearance_mm)
                        minimum_clearance = min(minimum_clearance, measured.clearance_mm)
                        # Additional probes catch joint-space bulges between IK samples.
                        # They do NOT certify arbitrary controller interpolation.
                        probes = max(2, math.ceil(maximum_step / options.joint_probe_step_deg))
                        for probe in range(1, probes):
                            middle = previous_joints + (joints - previous_joints) * (probe / probes)
                            if not kinematics.within_limits(middle.copy()):
                                raise _Rejected("intermediate_joint_limit", segment, index)
                            _, elbow_middle = _fk(kinematics, middle, budget)
                            measurement_middle = plane.measure(elbow_middle, torso)
                            if not measurement_middle.safe:
                                raise _Rejected("intermediate_elbow_plane", segment, index, measurement_middle.clearance_mm)
                            minimum_clearance = min(minimum_clearance, measurement_middle.clearance_mm)
                        waypoints.append(GuardedWaypoint(_matrix_tuple(pose), tuple(joints.tolist()), float(angle),
                                                         tuple(elbow.tolist()), measured.clearance_mm, segment))
                        previous_joints, previous_pose, previous_angle = joints, pose, angle
            except _Rejected as rejection:
                attempts.append(CandidateAttempt(candidate, rejection.reason, rejection.segment,
                                                  rejection.sample, rejection.clearance))
                continue
            budget.check()
            plan = GuardedPlan(tuple(current.tolist()), float(current_arm_angle_deg), _matrix_tuple(torso), plane,
                               candidate, tuple(waypoints), minimum_clearance)
            if validate_candidate:
                rejection = validate_candidate(plan)
                budget.check()
                if rejection:
                    attempts.append(CandidateAttempt(candidate, rejection))
                    continue
            attempts.append(CandidateAttempt(candidate, "passed"))
            return result("candidate_found", plan, "已找到首个全路径采样通过的臂角方案；未发送运动")
        return result("no_solution", message="臂角搜索范围内没有全路径通过的方案；禁止无保护回退")
    except _Interrupted as interruption:
        return result(str(interruption), message="规划已取消或达到计算上限；未发送运动")


@dataclass(frozen=True)
class RuntimeDecision:
    stop_required: bool
    reason: str
    measurement: PlaneMeasurement


def inspect_runtime_sample(plan: GuardedPlan, kinematics: ArmKinematics, joints_deg,
                           world_from_torso_mm, *, torso_translation_tolerance_mm=0.5,
                           torso_rotation_tolerance_deg=0.1) -> RuntimeDecision:
    """Read-only decision for future executors; does not stop/replan by itself.

    Callers must stop and obtain a fresh stationary snapshot before replanning.
    This cannot replace a controller-side collision or emergency-stop system.
    """
    translation_tolerance = _finite(torso_translation_tolerance_mm, "torso translation tolerance", 0)
    rotation_tolerance = _finite(torso_rotation_tolerance_deg, "torso rotation tolerance", 0)
    actual_torso = rigid(world_from_torso_mm, "actual torso")
    expected_torso = np.asarray(plan.world_from_torso_mm)
    joints = _vector(joints_deg, 7, "actual joints")
    _, elbow = kinematics.fk(joints.copy())
    measured = plan.plane.measure(elbow, actual_torso)
    translation = np.linalg.norm(actual_torso[:3, 3] - expected_torso[:3, 3])
    rotation = math.degrees(float(np.linalg.norm(rotation_log(actual_torso[:3, :3] @ expected_torso[:3, :3].T))))
    if translation > translation_tolerance or rotation > rotation_tolerance:
        return RuntimeDecision(True, "torso_changed_replan_required", measured)
    if not kinematics.within_limits(joints.copy()):
        return RuntimeDecision(True, "joint_limit", measured)
    return RuntimeDecision(not measured.safe, "clear" if measured.safe else "elbow_protection_margin", measured)


def start_matches(plan: GuardedPlan, joints_deg, world_from_torso_mm, *, joint_tolerance_deg=0.1,
                  torso_translation_tolerance_mm=0.5, torso_rotation_tolerance_deg=0.1) -> bool:
    """Recheck immediately before future dispatch; stale plans must be discarded."""
    joints = _vector(joints_deg, 7, "actual joints")
    torso = rigid(world_from_torso_mm, "actual torso")
    expected = np.asarray(plan.world_from_torso_mm)
    tolerances = [_finite(v, "start tolerance", 0) for v in
                  (joint_tolerance_deg, torso_translation_tolerance_mm, torso_rotation_tolerance_deg)]
    rotation = math.degrees(float(np.linalg.norm(rotation_log(torso[:3, :3] @ expected[:3, :3].T))))
    return bool(np.max(np.abs(joints - plan.start_joints_deg)) <= tolerances[0]
                and np.linalg.norm(torso[:3, 3] - expected[:3, 3]) <= tolerances[1]
                and rotation <= tolerances[2])
