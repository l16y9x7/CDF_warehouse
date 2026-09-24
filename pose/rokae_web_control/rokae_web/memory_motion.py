"""xCore memory motion adapter. Preflight cannot power or start a robot."""
from __future__ import annotations

import math
import numpy as np

from .backends import BackendError, JOINT_COUNTS
from .head_kinematics import rpy_rotation
from .memory_points import ARMS, joint_error, require_idle, vector
from .pose_frames import world_pose_to_ref
from .control_trace import invoke

# Documented model errors only. Network, state and unknown errors MUST NOT fall back.
IK_UNREACHABLE = {-50102, -50114, -50519, -50002}


def poses_match(actual, target):
    vector(actual, 6, "当前 Pose")
    vector(target, 6, "记忆 Pose")
    position = np.linalg.norm(np.asarray(actual[:3]) - target[:3])
    rotation = rpy_rotation(np.radians(actual[3:])).T @ rpy_rotation(np.radians(target[3:]))
    angle = math.degrees(math.acos(float(np.clip((np.trace(rotation) - 1) / 2, -1, 1))))
    return position <= 2.0 and angle <= 1.0


def tool_signature(toolset):
    return {name: vector(list(getattr(toolset, name).trans) + list(getattr(toolset, name).rpy), 6, name)
            for name in ("end", "ref")}


def capture(backend):
    with backend._lock:
        state = backend.read_state()
        state["arm_conf_data"] = {}
        state["toolsets"] = {}
        for name in (*ARMS, "trunk"):
            robot = backend._robot(name)
            state["toolsets"][name] = tool_signature(backend._call(f"读取 {name} 工具工件", robot.toolset))
            if name in ARMS:
                cart = backend._call(f"读取 {name} 构型", robot.cartPosture, backend._load_sdk().CoordinateType.endInRef)
                state["arm_conf_data"][name] = list(cart.confData)
        return state


def check_cancel(cancel):
    if cancel.is_set():
        raise BackendError("记忆点执行已停止")


def checked_ik(backend, action, call, *args):
    ec = {}
    result = invoke(getattr(backend, "audit_callback", None), action, call, args, ec)
    if ec.get("ec", 0) in IK_UNREACHABLE:
        return None, f"{action}: {ec.get('message', '')} (ec={ec['ec']})"
    backend._check_ec(action, ec)
    return vector(list(result), 7, action), None


def prepare(backend, target, speeds, cancel):
    with backend._lock:
        sdk = backend._load_sdk()
        start = capture(backend)
        require_idle(start)
        if target.get("toolsets") != start["toolsets"]:
            raise BackendError("工具/TCP 或工件坐标系已改变，请重新回读覆盖记忆点")
        limits = backend.soft_limit_status()["joint_limits_deg"]
        for name, count in JOINT_COUNTS.items():
            for index, value in enumerate(vector(target["joints_deg"][name], count, name)):
                low, high = limits[name][index]
                if not low <= value <= high:
                    raise BackendError(f"记忆点 {name} 第 {index + 1} 轴超出当前软限位")
        commands, modes = {}, {}
        for name in ARMS:
            check_cancel(cancel)
            robot = backend._robot(name)
            toolset = backend._call(f"读取 {name} 工具工件", robot.toolset)
            current = backend._call(f"读取 {name} 当前 Pose", robot.cartPosture, sdk.CoordinateType.endInRef)
            cart = sdk.CartesianPosition(world_pose_to_ref(backend._pose_to_sdk(target["poses"][name]),
                                                          backend._world_from_ref(toolset)))
            cart.elbow = math.radians(target["arm_elbow_deg"][name])
            cart.hasElbow = True
            conf = target.get("arm_conf_data", {}).get(name)
            if not isinstance(conf, list) or len(conf) != len(current.confData) or any(type(c) is not int for c in conf):
                raise BackendError(f"{name} 保存的构型数据无效")
            cart.confData = conf
            expected = target["joints_deg"][name]
            if joint_error(start, target, (name,)) <= 0.1 and poses_match(start["poses"][name], target["poses"][name]):
                modes[name] = {"motion": "已到位", "reason": ""}
                continue
            endpoint, reason = checked_ik(backend, f"{name} 目标逆解", robot.model().calcIk, cart, toolset)
            if endpoint is not None and max(abs(math.degrees(a) - b) for a, b in zip(endpoint, expected)) > 0.3:
                endpoint, reason = None, "笛卡尔逆解与记录的关节构型不同"
            if endpoint is None:
                # Explicit joint replay preserves ALL seven joints even at an IK singularity.
                command = sdk.MoveAbsJCommand(sdk.JointPosition([math.radians(v) for v in expected]), speeds["linear_mm_s"], 0.0)
                mode = "MoveJ（关节回放）"
            else:
                check_cancel(cancel)
                path_joints, reason = checked_ik(backend, f"{name} MoveL 轨迹预检", robot.checkPath,
                                               current, [math.radians(v) for v in start["joints_deg"][name]], cart)
                if path_joints is not None and max(abs(math.degrees(a) - b) for a, b in zip(path_joints, expected)) > 0.3:
                    reason = "直线路径终点构型与记录关节不同"
                    path_joints = None
                if path_joints is None:
                    command = sdk.MoveJCommand(cart, speeds["linear_mm_s"], 0.0)
                    mode = "MoveJ"
                else:
                    command = sdk.MoveLCommand(cart, speeds["linear_mm_s"], 0.0)
                    command.rotSpeed = math.radians(speeds["rotation_deg_s"])
                    mode = "MoveL"
            commands[name] = command
            modes[name] = {"motion": mode, "reason": reason or ""}
        head_trunk = sdk.JointPosition([math.radians(v) for v in target["joints_deg"]["trunk"]])
        head_trunk.external = [math.radians(v) for v in target["joints_deg"]["head"]]
        body_command = sdk.MoveAbsJCommand(head_trunk, speeds["linear_mm_s"], 0.0)
        return {"start": start, "target": target, "commands": commands, "modes": modes,
                "body_command": body_command, "speed": speeds["linear_mm_s"],
                "rotation_deg_s": speeds["rotation_deg_s"]}


def start_arms(backend, plan, cancel):
    with backend._lock:
        current = capture(backend)
        require_idle(current)
        if joint_error(current, plan["start"], JOINT_COUNTS) > 0.1 or current["toolsets"] != plan["start"]["toolsets"]:
            raise BackendError("预检期间机器人状态改变，请重新执行")
        sdk = backend._load_sdk()
        # Prepare AND enqueue both commands before starting either controller.
        for name, command in plan["commands"].items():
            check_cancel(cancel)
            robot = backend._robot(name)
            backend._prepare_motion(robot, name, plan["speed"])
            check_cancel(cancel)
            backend._call(f"{name} 下发记忆点", robot.moveAppend, [command], sdk.PyString())
        if plan.get('synchronized') and plan['commands']:
            check_cancel(cancel)
            modules = list(plan['commands'])
            if hasattr(backend, 'start_arms_synchronized'):
                plan['dispatch'] = backend.start_arms_synchronized(modules)
            else:
                from .synchronized_start import start
                plan['dispatch'] = start(backend, modules)
            return
        for name in plan["commands"]:
            check_cancel(cancel)
            backend._call(f"{name} 启动记忆点", backend._robot(name).moveStart)


def start_head_trunk(backend, plan, cancel):
    with backend._lock:
        check_cancel(cancel)
        current = capture(backend)
        require_idle(current)
        if joint_error(current, plan["target"], ARMS) > 0.5:
            raise BackendError("双臂尚未到位，禁止启动头部与躯干")
        if joint_error(current, plan["start"], ("head", "trunk")) > 0.3 or current["toolsets"] != plan["start"]["toolsets"]:
            raise BackendError("头躯干或工具配置在等待期间改变，已停止执行")
        if joint_error(current, plan["target"], ("head", "trunk")) <= 0.1:
            return
        robot = backend._robot("trunk")
        backend._prepare_motion(robot, "trunk", plan["speed"])
        check_cancel(cancel)
        backend._call("下发头部与躯干记忆点", robot.moveAppend, [plan["body_command"]], backend._load_sdk().PyString())
        check_cancel(cancel)
        backend._call("同时启动头部与躯干", robot.moveStart)


def stop(backend):
    errors = []
    with backend._lock:
        for name in (*ARMS, "trunk"):
            robot = backend._robots.get(name)
            if robot is not None:
                try:
                    backend._call(f"停止 {name}", robot.stop)
                    backend._call(f"清除 {name} 记忆点队列", robot.moveReset)
                except Exception as exc:
                    errors.append(str(exc))
    return errors
