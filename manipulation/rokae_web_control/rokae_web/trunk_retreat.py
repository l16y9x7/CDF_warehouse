"""PCB4 reference-frame translation with fixed arms/head.

Planning computes the SDK reference-X target without SDK IK or path validation. The
controller executes each leg as one native MoveL. Live feedback verifies the
reached pose and the return-to-start joint state.
"""
from __future__ import annotations

import copy
import math
import time

import numpy as np

from .arm_movel import HardwareMoveL, pose_values, transform
from .backends import BackendError, JOINT_COUNTS
from .memory_motion import poses_match, tool_signature
from .memory_points import joint_error, require_idle


def cart_pose(cart):
    return [v * 1000 for v in cart.trans] + [math.degrees(v) for v in cart.rpy]


class HardwareTrunkRetreat:
    """Two native MoveLs, preserving the start chest orientation and head axes."""
    def __init__(self, backend, config, cancel, sequence_start, distance_mm):
        self.backend, self.config, self.cancel = backend, config, cancel
        self.module = "trunk"
        self.moves = {}
        self.preflight_stats = {}
        with backend._lock:
            self.sdk, self.robot = backend._load_sdk(), backend._robot("trunk")
            self.start = backend.read_state()
            require_idle(self.start)
            if joint_error(self.start, sequence_start, JOINT_COUNTS) > 0.1:
                raise BackendError("读取躯干预检时起始状态改变")
            if self.start.get("toolsets", {}).get("trunk") != sequence_start.get("toolsets", {}).get("trunk"):
                raise BackendError("准备躯干目标时 SDK 工具或参考系改变")
            self.toolset = backend._call("读取躯干工具工件", self.robot.toolset)
            self.signature = tool_signature(self.toolset)
            self.current = backend._call("读取躯干笛卡尔位姿", self.robot.cartPosture,
                                         self.sdk.CoordinateType.endInRef)
            external = list(self.current.external)
            if len(external) < 2 or max(abs(math.degrees(a) - b) for a, b in
                    zip(external[:2], self.start["joints_deg"]["head"])) > .1:
                raise BackendError("躯干笛卡尔外部轴与头部回读不一致")
            self.limits = backend.soft_limit_status()["joint_limits_deg"]
        self.plan(distance_mm)

    def check(self):
        if self.cancel.is_set():
            raise BackendError("抓取测试已停止")

    def plan(self, distance_mm):
        self.check()
        self.preflight_stats = {"mode": "controller_move_with_live_feedback", "ik_calls": 0, "fk_calls": 0,
                                "path_samples": 0, "return_preflight": False}
        q0 = self.start["joints_deg"]["trunk"]
        for value, (lo, hi) in zip(q0, self.limits["trunk"]):
            if not lo <= value <= hi:
                raise BackendError("躯干起点超过控制器软限位")
        if not math.isfinite(distance_mm) or distance_mm <= 0:
            raise BackendError("躯干后退距离无效")
        tcp = transform(cart_pose(self.current))
        goal = tcp.copy()
        # The user says "躯干": displace in the same reference as endInRef,
        # independent of the flange/Chest_link orientation and TCP offset.
        goal[0, 3] -= distance_mm
        target = self.sdk.CartesianPosition(self.backend._pose_to_sdk(pose_values(goal)))
        target.confData = list(self.current.confData)
        target.external = list(self.current.external)
        self.moves = {
            "trunk_retreat": {"cart": target, "q": None, "from_q": list(q0)},
            "trunk_return": {"cart": self.current, "q": list(q0), "from_q": None},
        }
        if getattr(self.backend, "audit_callback", None):
            self.backend.audit_callback("trunk_preflight_efficiency", **self.preflight_stats)

    def start_move(self, key, expected, speed, rotation_deg_s):
        self.step = self.moves[key]
        if self.step["from_q"] is None:
            raise BackendError("躯干后退尚未确认到位，不能执行返回")
        self.fixed = copy.deepcopy(expected)
        with self.backend._lock:
            self.check()
            state = self.backend.read_state()
            require_idle(state)
            if (joint_error(state, expected, JOINT_COUNTS) > .5
                    or max(abs(a - b) for a, b in zip(state["joints_deg"]["trunk"], self.step["from_q"])) > .1):
                raise BackendError("躯干运动前机器人位置改变")
            if tool_signature(self.backend._call("回读躯干工具", self.robot.toolset)) != self.signature:
                raise BackendError("躯干工具或工件坐标系改变")
            self.backend._prepare_motion(self.robot, "trunk", speed)
            self.check()
            command = self.sdk.MoveLCommand(self.step["cart"], float(speed), 0.0)
            command.rotSpeed = math.radians(rotation_deg_s)
            self.backend._call("下发躯干 MoveL", self.robot.moveAppend, [command], self.sdk.PyString())
            self.check()
            self.backend._call("启动躯干 MoveL", self.robot.moveStart)

    def wait_move(self):
        deadline, stable, idle_since = time.monotonic() + 180, 0, None
        observed = False
        while time.monotonic() < deadline:
            self.check()
            with self.backend._lock:
                state = self.backend.read_state()
                if tool_signature(self.backend._call("回读躯干工具", self.robot.toolset)) != self.signature:
                    raise BackendError("躯干运动期间工具或工件坐标系改变")
            if any(state.get("dragging", {}).values()):
                raise BackendError("躯干运动期间检测到拖拽")
            for module in ("left_arm", "right_arm", "trunk"):
                allowed = ("idle", "moving") if module == "trunk" else ("idle",)
                if str(state["operation_state"][module]).lower() not in allowed:
                    raise BackendError(f"躯干运动期间 {module} 状态异常")
            if joint_error(state, self.fixed, ("left_arm", "right_arm", "head")) > .1:
                raise BackendError("躯干运动期间双臂或头部关节改变")
            q = state["joints_deg"]["trunk"]
            for value, (lo, hi) in zip(q, self.limits["trunk"]):
                if not lo <= value <= hi:
                    raise BackendError("躯干运动期间超过控制器软限位")
            maximum = self.config["motion"]["max_joint_step_deg"]["trunk"]
            if maximum is not None and max(abs(a - b) for a, b in zip(q, self.step["from_q"])) > float(maximum):
                raise BackendError("躯干运动超过网页单次关节变化上限")
            idle = str(state["operation_state"]["trunk"]).lower() == "idle"
            if not observed and (not idle or joint_error(state, self.fixed, ("trunk",)) > .02):
                observed = True
                if getattr(self.backend, "audit_callback", None):
                    self.backend.audit_callback("motion_first_observed", module="trunk", state=state)
            reached = (poses_match(state["poses"]["trunk"], cart_pose(self.step["cart"]))
                       and (self.step["q"] is None or max(abs(a - b) for a, b in zip(q, self.step["q"])) <= .5))
            stable = stable + 1 if idle and reached else 0
            if stable >= 2:
                if self.step is self.moves["trunk_retreat"]:
                    self.step["q"] = list(q)
                    self.moves["trunk_return"]["from_q"] = list(q)
                return state
            idle_since = (idle_since or time.monotonic()) if idle and not reached else None
            if idle_since and time.monotonic() - idle_since > 3:
                raise BackendError("躯干已静止但未到达目标")
            self.cancel.wait(.1)
        raise BackendError("躯干运动等待超时")

    def stop_and_verify(self):
        # Same native stop/reset/idle acknowledgement as the arm adapter.
        return HardwareMoveL.stop_and_verify(self)
