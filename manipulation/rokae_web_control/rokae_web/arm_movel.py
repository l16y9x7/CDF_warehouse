"""Single-arm MoveL jobs. Planning is read-only; dispatch is a separate phase.

Protected moves filter only the ENDPOINT elbow of native checkPath candidates.
The first accepted candidate is dispatched as ONE MoveL command. The elbow
along the path is deliberately not checked by this endpoint-only feature.
"""
from __future__ import annotations

import copy
from functools import lru_cache
import json
import math
from pathlib import Path
import sys
import threading
import time

import numpy as np

from .backends import BackendError, JOINT_COUNTS
from .head_kinematics import rpy_rotation, rotation_to_rpy_deg
from .memory_motion import checked_ik, poses_match, tool_signature
from .memory_points import joint_error, require_idle, vector
from .pose_frames import POSE_FRAMES, world_pose_to_ref


def number(value, label, low, high):
    if value is None or isinstance(value, bool) or value == "":
        raise BackendError(f"请填写{label}")
    try:
        result = float(value)
    except (ValueError, TypeError):
        raise BackendError(f"{label}必须是有限数值") from None
    if not math.isfinite(result) or not low <= result <= high:
        raise BackendError(f"{label}范围为 {low:g}–{high:g}")
    return result


def transform(pose):
    pose = vector(list(pose), 6, "Pose")
    result = np.eye(4)
    result[:3, 3] = pose[:3]
    result[:3, :3] = rpy_rotation(np.radians(pose[3:]))
    return result


def pose_values(matrix):
    return matrix[:3, 3].tolist() + rotation_to_rpy_deg(matrix[:3, :3])


@lru_cache(maxsize=8)
def _cached_model(path, side, mtime_ns, size):
    """Cache URDF parsing; file identity changes invalidate the cache."""
    from arm_motion_control.model import RobotModel
    return RobotModel(path, side)


def arm_model(path, side):
    root = str(Path(__file__).resolve().parents[2])
    if root not in sys.path:
        sys.path.append(root)
    source = Path(path).resolve()
    stat = source.stat()
    return _cached_model(str(source), side, stat.st_mtime_ns, stat.st_size)


def load_guard_plane(config):
    path = Path(config.get("torso_guard_file", Path(__file__).resolve().parents[1] / "torso_guard.json"))
    try:
        settings = json.loads(path.read_text(encoding="utf-8"))
        plane = {"offset_mm": number(settings.get("plane_offset_mm"), "torso_guard.json 中的 plane_offset_mm", 0, 1000),
                "elbow_radius_mm": number(settings.get("elbow_radius_mm"), "肘部半径 mm", 0, 300),
                "margin_mm": number(settings.get("margin_mm"), "额外余量 mm", 0, 100)}
        if settings.get("front_plane_x_mm") is not None:
            plane["front_plane_x_mm"] = number(settings["front_plane_x_mm"], "胸部前平面 X mm", 0, 1000)
        return plane
    except (OSError, ValueError, KeyError, TypeError, AttributeError) as exc:
        raise BackendError(f"躯干保护配置读取失败: {exc}") from exc


from .control_trace import fields, operation, worker_thread


class ArmMoveL:
    def __init__(self, service):
        self.service = service
        self.cancel = threading.Event()
        self.thread = None
        self.job = {"active": False, "phase": "idle", "message": "MoveL 待命"}

    def status(self):
        with self.service._lock:
            return {**copy.deepcopy(self.job), "planner_version": "endpoint-v1", "guard_rule_version": "chest-y-or-front-x-v1"}

    def ensure_idle(self):
        if self.job.get("stop_unconfirmed"):
            raise BackendError("上次 MoveL 停止未确认，请现场确认控制器状态后重启网页服务")
        if self.job["active"]:
            raise BackendError("MoveL 正在规划或运动，请先停止或等待完成")

    def _set(self, **fields):
        with self.service._lock:
            self.job.update(fields)
        self.service.audit_event("arm_movel_progress", **fields)

    @operation("arm_movel")
    def execute(self, module, payload, protected):
        requested_at = time.perf_counter()
        if module not in ("left_arm", "right_arm"):
            raise BackendError("MoveL 仅支持左右手臂")
        values = vector(payload.get("values"), 6, "目标 Pose")
        if payload.get("frame") != POSE_FRAMES[module]:
            raise BackendError("手臂坐标系不一致，请刷新网页并重新回读")
        seed = payload.get("elbow_deg")
        seed = None if seed is None else number(seed, "臂角 °", -180, 180)
        plane = load_guard_plane(self.service.config) if protected else None
        with self.service._lock:
            self.service._require_armed()
            # HardwareMoveL captures and checks a fresh idle snapshot in the worker.
            # Avoid a duplicate whole-body read here on the request's critical path.
            if not self.service.hardware_enabled:
                require_idle(self.service.robot.read_state())
            self.service.chassis.stop()
            speed = self.service.speed_mm_s
            rotation = self.service.rotation_deg_s
            self.cancel.clear()
            self.job = {"active": True, "phase": "planning", "module": module,
                        **fields(),
                        "protected": protected, "speed_mm_s": speed, "rotation_deg_s": rotation, "plane": plane,
                        "seed_arm_angle_deg": seed, "protection_scope": "endpoint_only" if protected else "none",
                        "message": "正在演算 MoveL 并筛选终点肘点" if protected else "正在演算 MoveL"}
            self.thread = worker_thread(target=self._run, args=(module, values, seed, plane, speed, rotation, requested_at),
                                           name="arm-movel", daemon=True)
            self.thread.start()
            return self.status()

    def _check(self):
        if self.cancel.is_set():
            raise BackendError("MoveL 已取消")

    def _run(self, module, values, seed, plane, speed, rotation, requested_at):
        executor = None
        started = False
        try:
            if not self.service.hardware_enabled:
                self._check()
                # Mock has no physical kinematics: never claim plane validation.
                with self.service._lock:
                    self._check()
                    angle = self.service.robot.read_state()["arm_elbow_deg"][module] if seed is None else seed
                    self.service.robot.move_pose(module, values, speed, angle)
                self._set(active=False, phase="completed", selected_arm_angle_deg=angle,
                          message="MOCK 已到位（仅模拟界面流程，未做实机几何验证）")
                return
            executor = HardwareMoveL(self.service.robot, module, self.service.config, self.cancel)
            result = executor.plan(values, seed, plane)
            self._check()
            self._set(selected_arm_angle_deg=result["angle"],
                      endpoint_clearance_mm=result.get("clearance"), segments=len(executor.steps),
                      endpoint_accepted_by=result.get("accepted_by"),
                      endpoint_elbow_chest_mm=result.get("elbow_chest_mm"),
                      endpoint_front_plane_x_mm=result.get("front_plane_x_mm"),
                      endpoint_front_clearance_mm=result.get("front_clearance_mm"),
                      planning_ms=result.get("planning_ms"), check_path_calls=result.get("check_path_calls"),
                      message="终点筛选通过，准备执行单条 MoveL" if plane else "准备执行单条 MoveL")
            self.service.audit_event("arm_movel_preflight", module=module, protected=bool(plane),
                                     speed_mm_s=speed, rotation_deg_s=rotation, plane=plane, result=result, start_state=getattr(executor, "start", None),
                                     target_pose=values, toolset=getattr(executor, "signature", None), steps=executor.steps)
            if len(executor.steps) != 1:
                raise BackendError("MoveL 必须只有一个目标，禁止分段下发")
            self._check()
            started = True  # Even a failing append/start may have affected the controller.
            executor.start_step(0, speed, rotation)  # Includes exactly one freshness check, before dispatch.
            self._set(phase="moving", message="单条 MoveL 运动中",
                      dispatch_ms=round((time.perf_counter() - requested_at) * 1000, 3))
            executor.wait_step(0)
            self._check()
            self._set(active=False, phase="completed", message="MoveL 已到位")
            self.service.audit_event("arm_movel_completed", module=module)
        except Exception as exc:
            try:
                errors = executor.stop_and_verify() if started and executor else []
            except Exception as stop_error:
                errors = [str(stop_error)]
            message = str(exc) + ("；停止确认失败：" + "; ".join(errors) if errors else "")
            if errors:
                with self.service._lock:
                    self.service.armed = False
            self._set(active=False, phase="cancelled" if self.cancel.is_set() and not errors else "failed",
                      message=message, stop_unconfirmed=bool(errors))
            self.service.audit_event("arm_movel_failed", module=module, error=message)

    def stop(self):
        self.cancel.set()  # Do not wait for a service/SDK lock before signalling cancellation.
        self.service.audit_event("arm_movel_stop_requested", target_operation_id=self.job.get("operation_id"))
        with self.service._lock:
            if self.job["active"]:
                self.cancel.set()
                self.job.update(phase="stopping", message="正在停止 MoveL")
            return self.status()

    def close(self):
        self.stop()
        if self.thread and self.thread.is_alive():
            self.thread.join(timeout=10)


class MotionPlanUnavailable(BackendError):
    """A geometric candidate failed; cancellation/communication errors are distinct."""


class HardwareMoveL:
    def __init__(self, backend, module, config, cancel):
        self.backend, self.module, self.config, self.cancel = backend, module, config, cancel
        self.endpoint_plane = None
        self.steps = []
        self.deadline = time.monotonic() + 60
        with backend._lock:
            self.sdk, self.robot = backend._load_sdk(), backend._robot(module)
            self.start = backend.read_state()
            require_idle(self.start)
            self.toolset = backend._call("读取 MoveL 工具工件", self.robot.toolset)
            self.signature = tool_signature(self.toolset)
            self.current = backend._call("读取 MoveL 起点构型", self.robot.cartPosture, self.sdk.CoordinateType.endInRef)
            self.conf = list(self.current.confData)
            self.limits = backend.soft_limit_status()["joint_limits_deg"][module]

    def check(self, planning=False):
        if self.cancel.is_set():
            raise BackendError("MoveL 已取消")
        if planning and time.monotonic() > self.deadline:
            raise BackendError("MoveL 规划超过 60 秒，未发送运动")

    def cartesian(self, pose, angle):
        cart = self.sdk.CartesianPosition(world_pose_to_ref(self.backend._pose_to_sdk(pose),
                                                           self.backend._world_from_ref(self.toolset)))
        cart.elbow = math.radians(angle)
        cart.hasElbow = True
        cart.confData = self.conf
        return cart

    def _path(self, start, joints, goal):
        self.check(planning=True)
        with self.backend._lock:
            result, reason = checked_ik(self.backend, "MoveL checkPath", self.robot.checkPath,
                                       start, np.radians(joints).tolist(), goal)
        self.check(planning=True)
        return None if result is None else np.degrees(result), reason

    def plan(self, values, seed, plane):
        started = time.perf_counter()
        if getattr(self.backend, "audit_callback", None):
            self.backend.audit_callback("arm_plan_input", module=self.module, start=self.start,
                                        target_pose_mm_deg=values, seed_arm_angle_deg=seed,
                                        plane=plane, toolset=self.signature, soft_limits_deg=self.limits)
        actual_angle = self.start["arm_elbow_deg"][self.module]
        angle = actual_angle if seed is None else seed
        self.steps = []
        if plane is None:
            cart = self.cartesian(values, angle)
            q, error = self._path(self.current, self.start["joints_deg"][self.module], cart)
            if q is None:
                raise BackendError(error)
            if not self.within_soft_limits(q):
                raise BackendError("MoveL 目标超出关节软限位")
            self.steps = [{"cart": cart, "q": q.tolist(), "pose": values, "angle": angle}]
            self._check_max_step()
            return {"angle": angle, "planning_ms": round((time.perf_counter() - started) * 1000, 3),
                    "check_path_calls": 1}

        side = self.module.split("_")[0]
        self.model = arm_model(self.config["pose_estimation"]["urdf_file"], side)
        from arm_motion_control.torso_guard import TorsoPlane, SearchOptions, arm_angle_scan
        plane_settings = dict(plane)
        front_x = plane_settings.pop("front_plane_x_mm", None)
        if front_x is not None:
            front_x = number(front_x, "胸部前平面 X mm", 0, 1000)
        self.endpoint_plane = TorsoPlane(side, **plane_settings)
        # Chest-relative elbow FK is invariant to the torso configuration. Using
        # the same zero torso for FK and the plane avoids unnecessary torso math
        # per candidate; SDK MoveL still uses the original target TCP/ref/tool.
        zero_trunk = [0.0] * 4
        torso = self.model.torso_world(zero_trunk)
        attempts = []
        for candidate in arm_angle_scan(angle, SearchOptions()):
            cart = self.cartesian(values, candidate)
            q, reason = self._path(self.current, self.start["joints_deg"][self.module], cart)
            if q is None:
                attempts.append({"angle": candidate, "reason": "native_path_unreachable"})
                continue
            if not self.within_soft_limits(q) or not self.model.arm_joints_within_limits(q):
                attempts.append({"angle": candidate, "reason": "endpoint_joint_limit"})
                continue
            _, elbow = self.model.arm_frames_world(zero_trunk, q)
            measurement = self.endpoint_plane.measure(elbow, torso)
            # X exemption uses the elbow CENTER. Do not add the Y-plane radius
            # or margin again: the user explicitly chose center X > threshold.
            front_clearance = None if front_x is None else float(measurement.elbow_torso_mm[0] - front_x)
            front_pass = front_clearance is not None and front_clearance > 0.0
            if not measurement.safe and not front_pass:
                attempts.append({"angle": candidate, "reason": "endpoint_elbow_plane",
                                 "clearance_mm": measurement.clearance_mm,
                                 "elbow_chest_mm": list(measurement.elbow_torso_mm),
                                 "front_plane_x_mm": front_x, "front_clearance_mm": front_clearance})
                continue
            accepted_by = "y_plane" if measurement.safe else "front_plane"
            self.check(planning=True)
            self.steps = [{"cart": cart, "q": q.tolist(), "pose": list(values), "angle": candidate}]
            self._check_max_step()
            attempts.append({"angle": candidate, "reason": "passed", "accepted_by": accepted_by})
            return {"angle": candidate, "clearance": measurement.clearance_mm,
                    "elbow_chest_mm": list(measurement.elbow_torso_mm),
                    "accepted_by": accepted_by, "front_plane_x_mm": front_x,
                    "front_clearance_mm": front_clearance,
                    "guard_rule_version": "chest-y-or-front-x-v1",
                    "attempts": attempts, "check_path_calls": len(attempts),
                    "planning_ms": round((time.perf_counter() - started) * 1000, 3),
                    "protection_scope": "endpoint_only"}
        if getattr(self.backend, "audit_callback", None):
            self.backend.audit_callback("arm_plan_exhausted", module=self.module, attempts=attempts,
                                        duration_ms=(time.perf_counter() - started) * 1000)
        raise MotionPlanUnavailable(f"{len(attempts)} 个臂角均未找到可行且终点肘部不过界的 MoveL；未发送运动")

    def within_soft_limits(self, q):
        return all(lo <= value <= hi for value, (lo, hi) in zip(vector(list(q), 7, "关节"), self.limits))

    def _check_max_step(self):
        maximum = self.config["motion"]["max_joint_step_deg"][self.module]
        if maximum is not None:
            start = np.asarray(self.start["joints_deg"][self.module])
            if any(np.max(np.abs(np.asarray(s["q"]) - start)) > float(maximum) for s in self.steps):
                raise BackendError("MoveL 超出网页配置的单次关节变化上限")

    def _snapshot(self):
        state = self.backend.read_state()
        signature = tool_signature(self.backend._call("回读 MoveL 工具工件", self.robot.toolset))
        if signature != self.signature:
            raise BackendError("工具或工件坐标系改变，已中止 MoveL")
        return state

    def _inspect(self, state):
        if any(state.get("dragging", {}).values()):
            raise BackendError("检测到拖拽状态，已中止 MoveL")
        for name in ("left_arm", "right_arm", "trunk"):
            operation = str(state["operation_state"].get(name)).lower()
            allowed = ("idle", "moving") if name == self.module else ("idle",)
            if operation not in allowed:
                raise BackendError(f"{name} 运行状态改变: {operation}")
        fixed = [m for m in JOINT_COUNTS if m != self.module]
        if joint_error(state, self.start, fixed) > 0.1:
            raise BackendError("躯干/头部/另一手臂位置改变，需要重新规划")
        # Endpoint protection intentionally does not sample or stop on elbow
        # position during this one MoveL. Fault/cancel/fixed-body checks remain.

    def check_fresh(self):
        with self.backend._lock:
            self.check()
            state = self._snapshot()
            require_idle(state)
            if joint_error(state, self.start, JOINT_COUNTS) > 0.1:
                raise BackendError("规划期间机器人位置改变，请重新执行")
            if abs(state["arm_elbow_deg"][self.module] - self.start["arm_elbow_deg"][self.module]) > 0.1:
                raise BackendError("规划期间臂角改变，请重新执行")
            self._inspect(state)

    def start_step(self, index, speed, rotation_deg_s):
        with self.backend._lock:
            self.check()
            if index != 0 or len(self.steps) != 1:
                raise BackendError("MoveL 必须单条执行")
            self.check_fresh()
            self.backend._prepare_motion(self.robot, self.module, speed)
            self.check()
            command = self.sdk.MoveLCommand(self.steps[index]["cart"], float(speed), 0.0)
            command.rotSpeed = math.radians(rotation_deg_s)
            self.backend._call("下发 MoveL", self.robot.moveAppend, [command], self.sdk.PyString())
            self.check()
            self.backend._call("启动 MoveL", self.robot.moveStart)

    def reached(self, state, step):
        return (max(abs(a - b) for a, b in zip(state["joints_deg"][self.module], step["q"])) <= 0.5
                and poses_match(state["poses"][self.module], step["pose"])
                and abs(state["arm_elbow_deg"][self.module] - step["angle"]) <= 0.5)

    def wait_step(self, index):
        step = self.steps[index]
        observed = False
        deadline, stable, idle_since = time.monotonic() + 180, 0, None
        while time.monotonic() < deadline:
            self.check()
            with self.backend._lock:
                state = self._snapshot()
                self._inspect(state)
            idle = str(state["operation_state"][self.module]).lower() == "idle"
            if not observed and (not idle or joint_error(state, self.start, (self.module,)) > .02):
                observed = True
                if getattr(self.backend, "audit_callback", None):
                    self.backend.audit_callback("motion_first_observed", module=self.module, state=state)
            reached = self.reached(state, step)
            stable = stable + 1 if idle and reached else 0
            if stable >= 2:
                return
            idle_since = (idle_since or time.monotonic()) if idle and not reached else None
            if idle_since and time.monotonic() - idle_since > 3:
                raise BackendError("控制器静止但未到达 MoveL 目标")
            self.cancel.wait(0.1)
        raise BackendError("MoveL 到位等待超时")

    def stop_and_verify(self):
        errors = []
        with self.backend._lock:
            for name, call in (("停止 MoveL", self.robot.stop), ("清除 MoveL 队列", self.robot.moveReset)):
                try:
                    self.backend._call(name, call)
                except Exception as exc:
                    errors.append(str(exc))
        deadline = time.monotonic() + 3
        while time.monotonic() < deadline:
            with self.backend._lock:
                if self.backend._operation_name(self.robot, self.module).lower() == "idle":
                    return errors
            time.sleep(0.1)
        return errors + ["无法确认手臂静止"]
