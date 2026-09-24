"""Persistent teaching points and an interruptible, monitored upper-body sequence."""
from __future__ import annotations

import copy
import json
import math
import os
import threading
import time
import uuid
from datetime import datetime, timezone
from pathlib import Path

from .backends import BackendError, JOINT_COUNTS, POSE_MODULES
from .pose_frames import POSE_FRAMES

ARMS = ("left_arm", "right_arm")


def vector(value, count, label):
    if (not isinstance(value, list) or len(value) != count
            or any(isinstance(v, bool) or not isinstance(v, (int, float))
                   or not math.isfinite(v) for v in value)):
        raise BackendError(f"{label} 数据不完整或不是有限数值")
    return value


def validate_snapshot(state):
    if not isinstance(state, dict):
        raise BackendError("记忆点状态数据损坏")
    for name, count in JOINT_COUNTS.items():
        vector(state.get("joints_deg", {}).get(name), count, name)
    for name in (*POSE_MODULES, "head"):
        vector(state.get("poses", {}).get(name), 6, f"{name} Pose")
    for name in ARMS:
        vector([state.get("arm_elbow_deg", {}).get(name)], 1, f"{name} 臂角")
    if any(state.get("pose_frames", {}).get(k) != v for k, v in POSE_FRAMES.items()):
        raise BackendError("记忆点的位姿坐标系不匹配")


def joint_error(state, target, modules):
    return max(abs(a - b) for name in modules
               for a, b in zip(vector(state["joints_deg"][name], JOINT_COUNTS[name], name),
                               target["joints_deg"][name]))


def require_idle(state):
    if any(state.get("dragging", {}).values()):
        raise BackendError("请先关闭双臂拖拽，再执行记忆点")
    for name in (*ARMS, "trunk"):
        if str(state.get("operation_state", {}).get(name)).lower() not in ("idle", "mock-idle"):
            raise BackendError(f"{name} 尚未静止或状态未知，不能执行记忆点")


class MemoryPointStore:
    def __init__(self, path):
        self.path = Path(path)

    def list(self):
        if not self.path.exists():
            return []
        try:
            doc = json.loads(self.path.read_text(encoding="utf-8"))
            if doc["version"] != 1 or not isinstance(doc["points"], list):
                raise ValueError("版本或列表不正确")
            ids, names = set(), set()
            for point in doc["points"]:
                if (not isinstance(point["id"], str) or not isinstance(point["name"], str)
                        or not point["name"].strip() or point["id"] in ids or point["name"] in names
                        or not isinstance(point["revision"], int) or point["revision"] < 1):
                    raise ValueError("点位标识、名称或版本损坏")
                validate_snapshot(point["state"])
                ids.add(point["id"])
                names.add(point["name"])
            return doc["points"]
        except (OSError, ValueError, KeyError, TypeError, BackendError) as exc:
            raise BackendError(f"记忆点文件读取失败，原文件已保留: {exc}") from exc

    def get(self, point_id, revision=None):
        for point in self.list():
            if point["id"] == point_id:
                if revision is not None and revision != point["revision"]:
                    raise BackendError("该记忆点已被更新，请刷新列表后重试")
                return point
        raise BackendError("所选记忆点不存在，请刷新列表")

    def save(self, name, state, mode, point_id=None, revision=None):
        validate_snapshot(state)
        if not isinstance(name, str):
            raise BackendError("请输入记忆点名称")
        name = name.strip()
        if not name or len(name) > 80 or any(ord(c) < 32 for c in name):
            raise BackendError("记忆点名称须为 1–80 个字符，不能包含控制字符")
        points = self.list()
        old = self.get(point_id, revision) if point_id else None
        if any(p["name"] == name and p["id"] != point_id for p in points):
            raise BackendError("记忆点名称已存在，请使用覆盖按钮或其他名称")
        now = datetime.now(timezone.utc).isoformat()
        point = {"id": point_id or uuid.uuid4().hex, "name": name,
                 "revision": old["revision"] + 1 if old else 1,
                 "created_at": old["created_at"] if old else now, "updated_at": now,
                 "mode": mode, "state": copy.deepcopy(state)}
        points = [point if p["id"] == point_id else p for p in points] if old else [*points, point]
        self._write(points)
        return point

    def delete(self, point_id, revision):
        if not isinstance(point_id, str) or not point_id:
            raise BackendError("请选择要删除的记忆点")
        if type(revision) is not int or revision < 1:
            raise BackendError("记忆点版本无效，请刷新列表后重试")
        points = self.list()
        for index, point in enumerate(points):
            if point["id"] == point_id:
                if point["revision"] != revision:
                    raise BackendError("该记忆点已被更新，请刷新列表后重试")
                self._write(points[:index] + points[index + 1:])
                return point
        raise BackendError("所选记忆点不存在，请刷新列表")

    def _write(self, points):
        self.path.parent.mkdir(parents=True, exist_ok=True)
        temporary = self.path.with_name(f".{self.path.name}.{uuid.uuid4().hex}.tmp")
        try:
            with temporary.open("x", encoding="utf-8") as stream:
                json.dump({"version": 1, "points": points}, stream, ensure_ascii=False, indent=2, allow_nan=False)
                stream.flush()
                os.fsync(stream.fileno())
            os.replace(temporary, self.path)
        finally:
            temporary.unlink(missing_ok=True)


from .control_trace import fields, operation, worker_thread


class MemoryPoints:
    def __init__(self, service):
        self.service = service
        config = service.config.get("memory_points", {})
        default = Path(__file__).resolve().parent.parent / "memory_points"
        mode = "hardware" if service.hardware_enabled else "mock"
        self.store = MemoryPointStore(config.get("file", default / f"{mode}.json"))
        self.mode = mode
        self.job = {"active": False, "phase": "idle", "message": "请选择或新增记忆点"}
        self.cancelled = threading.Event()
        self.thread = None
        self.kinematics = None
        self.poll_seconds = 0.1
        self.timeout_seconds = 600.0

    def status(self):
        # The caller holds service._lock; no SDK calls here.
        speed_mm_s = (self.job["execution_speed_mm_s"] if self.job["active"]
                      else self.service.speed_mm_s)
        rotation = (self.job["execution_rotation_deg_s"] if self.job["active"]
                    else self.service.rotation_deg_s)
        return {**copy.deepcopy(self.job), "speed": {"linear_mm_s": float(speed_mm_s),
                                                    "rotation_deg_s": float(rotation)}}

    def ensure_idle(self):
        if self.job["active"]:
            raise BackendError("记忆点正在执行，请等待完成或先停止")

    def listing(self):
        with self.service._lock:
            points = [{k: p[k] for k in ("id", "name", "revision", "updated_at")}
                      for p in self.store.list()]
            return {"points": points, "execution": self.status(), "can_delete": True}

    def _capture(self):
        first = self.service.robot.read_memory_state()
        time.sleep(0.1)
        state = self.service.robot.read_memory_state()
        if joint_error(state, first, JOINT_COUNTS) > 0.1:
            raise BackendError("关节还在移动，请停稳后保存记忆点")
        for name in (*ARMS, "trunk"):
            if str(state["operation_state"].get(name)).lower() not in ("idle", "drag", "mock-idle", "mock-drag"):
                raise BackendError(f"{name} 未静止，不能保存记忆点")
        if self.service.hardware_enabled:
            from .head_kinematics import UpperBodySixDofKinematics, rotation_to_rpy_deg
            if self.kinematics is None:
                self.kinematics = UpperBodySixDofKinematics(self.service.config["pose_estimation"]["urdf_file"])
            transform = self.kinematics.forward_deg(state["joints_deg"]["trunk"] + state["joints_deg"]["head"])
            state["poses"]["head"] = (transform[:3, 3] * 1000).tolist() + rotation_to_rpy_deg(transform[:3, :3])
            state["head_pose_source"] = "URDF FK: chassis_link -> Head_link"
        else:
            state["poses"]["head"] = [0.0] * 6
            state["head_pose_source"] = "mock (not physical FK)"
        state["pose_frames"]["head"] = "chassis_link"
        state["units"] = {"joints": "deg", "pose": "mm/deg", "rotation": "RPY (Rz Ry Rx)"}
        state["captured_at"] = datetime.now(timezone.utc).isoformat()
        validate_snapshot(state)
        return state

    def save(self, payload, overwrite=False):
        with self.service._lock:
            self.ensure_idle()
            self.service.arm_movel.ensure_idle()
            self.service.grasp_test.ensure_idle()
            self.service.scan_sequence.ensure_idle()
            old = self.store.get(payload.get("id"), payload.get("revision")) if overwrite else None
            state = self._capture()  # Never accept browser form values as a teaching point.
            point = self.store.save(old["name"] if old else payload.get("name"), state, self.mode,
                                    old["id"] if old else None, payload.get("revision"))
            self.service.audit_event("memory_point_overwritten" if overwrite else "memory_point_created",
                                     point=point)
            return {"point": point, **self.listing()}

    def delete(self, payload):
        with self.service._lock:
            self.ensure_idle()
            self.service.arm_movel.ensure_idle()
            self.service.grasp_test.ensure_idle()
            self.service.scan_sequence.ensure_idle()
            point = self.store.delete(payload.get("id"), payload.get("revision"))
            self.service.audit_event("memory_point_deleted", point=point)
            return {"point": point, **self.listing()}

    @operation("memory_motion")
    def execute(self, payload, *, prepared_point=None):
        with self.service._lock:
            self.service._require_armed()
            self.ensure_idle()
            point = (copy.deepcopy(prepared_point) if prepared_point is not None else
                     self.store.get(payload.get("id"), payload.get("revision")))
            validate_snapshot(point["state"])
            if point["mode"] != self.mode:
                raise BackendError("MOCK 与实机记忆点不能混用")
            current = self.service.robot.read_state()
            require_idle(current)
            for name, maximum in self.service.config["motion"]["max_joint_step_deg"].items():
                if maximum is not None and joint_error(current, point["state"], (name,)) > float(maximum):
                    raise BackendError(f"{name} 超过当前配置的单次关节变化上限")
            speed_mm_s = float(self.service.speed_mm_s)
            rotation = float(self.service.rotation_deg_s)
            self.service.chassis.stop()
            self.cancelled.clear()
            self.job = {"active": True, "phase": "planning", "id": point["id"], "name": point["name"],
                        **fields(),
                        "message": "正在预检双臂轨迹", "arm_modes": {},
                        "execution_speed_mm_s": speed_mm_s, "execution_rotation_deg_s": rotation}
            self.thread = worker_thread(target=self._run, args=(copy.deepcopy(point), speed_mm_s, rotation),
                                           name="upper-body-memory", daemon=True)
            self.thread.start()
            return self.status()

    def _set(self, **fields):
        with self.service._lock:
            self.job.update(fields)
        self.service.audit_event("memory_motion_progress", **fields)

    def _check_cancel(self):
        if self.cancelled.is_set():
            raise BackendError("记忆点执行已停止")

    def _wait_reached(self, target, moving, fixed):
        deadline = time.monotonic() + self.timeout_seconds
        stable = 0
        idle_since = None
        while time.monotonic() < deadline:
            self._check_cancel()
            state = self.service.robot.read_state()
            for name in (*ARMS, "trunk"):
                operation = str(state.get("operation_state", {}).get(name)).lower()
                if operation not in ("idle", "mock-idle", "moving"):
                    raise BackendError(f"{name} 运行状态异常: {operation}")
                if name not in moving and operation == "moving":
                    raise BackendError(f"{name} 在等待期间意外运动")
            if fixed and joint_error(state, fixed, [m for m in JOINT_COUNTS if m not in moving]) > 0.3:
                raise BackendError("非当前阶段的关节发生变化，已中止顺序执行")
            idle = all(str(state["operation_state"].get("trunk" if m == "head" else m)).lower()
                       in ("idle", "mock-idle") for m in moving)
            reached = joint_error(state, target, moving) <= 0.5
            if any(m in ARMS for m in moving):
                from .memory_motion import poses_match
                reached = reached and all(poses_match(state["poses"][m], target["poses"][m]) for m in moving if m in ARMS)
            stable = stable + 1 if idle and reached else 0
            if stable >= 3:
                return state
            idle_since = (idle_since or time.monotonic()) if idle and not reached else None
            if idle_since and time.monotonic() - idle_since > 5.0:
                raise BackendError("控制器已静止但尚未到达记忆点，后续阶段已取消")
            if self.cancelled.wait(self.poll_seconds):
                self._check_cancel()
        raise BackendError("记忆点到位等待超时，后续阶段已取消")

    def _run(self, point, speed_mm_s, rotation):
        started = False
        try:
            speeds = {"linear_mm_s": speed_mm_s, "rotation_deg_s": rotation}
            plan = self.service.robot.prepare_memory_motion(point["state"], speeds, self.cancelled)
            self.service.audit_event("memory_motion_preflight", point=point, plan=plan)
            self._check_cancel()
            self._set(phase="arms", message="双臂运动中；等待两臂到位", arm_modes=plan["modes"])
            self.service.audit_event("memory_motion_started", point_id=point["id"], modes=plan["modes"], speed=speeds)
            started = True  # A start can fail after one controller accepted its command.
            self.service.robot.start_memory_arms(plan, self.cancelled)
            self.service._request_motion_sampling("memory_arms", "upper_body")
            after_arms = self._wait_reached(point["state"], ARMS, plan["start"])
            self.service.audit_event("memory_motion_stage_completed", phase="arms", state=after_arms)
            self._check_cancel()
            self._set(phase="head_trunk", message="双臂已到位，头部与躯干运动中")
            self.service.robot.start_memory_head_trunk(plan, self.cancelled)
            self.service._request_motion_sampling("memory_head_trunk", "upper_body")
            final_state = self._wait_reached(point["state"], ("trunk", "head"), after_arms)
            self._check_cancel()
            self._set(active=False, phase="completed", message="记忆点已到位")
            self.service.audit_event("memory_motion_completed", point_id=point["id"], state=final_state)
        except Exception as exc:
            errors = self._stop_and_verify() if started else []
            message = str(exc) + (f"；停止指令失败: {'; '.join(errors)}" if errors else "")
            if errors:
                with self.service._lock:
                    self.service.armed = False
            self._set(active=False, phase="cancelled" if self.cancelled.is_set() and not errors else "failed",
                      message=message)
            self.service.audit_event("memory_motion_stopped", point_id=point["id"], error=message)

    def _stop_and_verify(self):
        try:
            errors = self.service.robot.stop_memory_motion()
        except Exception as exc:
            errors = [str(exc)]
        deadline = time.monotonic() + 3.0
        while time.monotonic() < deadline:
            try:
                state = self.service.robot.read_state()
                require_idle(state)
                return errors
            except Exception as exc:
                last_error = str(exc)
            time.sleep(self.poll_seconds)
        return [*errors, f"无法确认所有控制器已停止: {last_error}"]

    def stop(self):
        # Set the event BEFORE waiting on the SDK/service lock, including during preflight.
        self.cancelled.set()
        with self.service._lock:
            if self.job["active"]:
                self.cancelled.set()
                self.job.update(phase="stopping", message="正在停止记忆点执行")
            return self.status()

    def close(self):
        self.stop()
        if self.thread and self.thread.is_alive():
            self.thread.join(timeout=10.0)
