from __future__ import annotations

import importlib
import math
import sys
import threading
import time
from pathlib import Path
from typing import Any
from .battery import BatteryTelemetry
from .control_trace import fields, invoke, span

from .gripper import GripperError, Robotiq2F85
from .pose_frames import POSE_FRAMES, ref_pose_to_world, world_pose_to_ref
from .telemetry import TelemetryBusy, module_data


JOINT_COUNTS = {"left_arm": 7, "right_arm": 7, "trunk": 4, "head": 2}
POSE_MODULES = ("left_arm", "right_arm", "trunk")


class BackendError(RuntimeError):
    pass


def _zero_state() -> dict[str, Any]:
    return {
        "joints_deg": {name: [0.0] * count for name, count in JOINT_COUNTS.items()},
        "poses": {name: [0.0] * 6 for name in POSE_MODULES},
        "arm_elbow_deg": {"left_arm": 0.0, "right_arm": 0.0},
        "dragging": {"left_arm": False, "right_arm": False},
        "operation_state": {
            "left_arm": "mock-idle",
            "right_arm": "mock-idle",
            "trunk": "mock-idle",
        },
    }


class MockRobotBackend:
    """In-memory backend. It never imports the SDK or opens a network connection."""

    def __init__(self) -> None:
        self._state = _zero_state()
        self._lock = threading.RLock()
        self._gripper_position = 0
        self._gripper_activated = False

    def read_state(self) -> dict[str, Any]:
        with self._lock:
            return {
                "joints_deg": {k: list(v) for k, v in self._state["joints_deg"].items()},
                "poses": {k: list(v) for k, v in self._state["poses"].items()},
                "pose_frames": dict(POSE_FRAMES),
                "toolsets": {"trunk": {"end": [0.0] * 6, "ref": [0.0] * 6}},
                "arm_elbow_deg": dict(self._state["arm_elbow_deg"]),
                "dragging": dict(self._state["dragging"]),
                "operation_state": dict(self._state["operation_state"]),
            }

    def read_memory_state(self) -> dict[str, Any]:
        return self.read_state()

    def read_telemetry_module(self, module):
        if not self._lock.acquire(blocking=False):
            raise TelemetryBusy()
        try:
            s = self.read_state()
            return module_data(module, s["joints_deg"][module], s["poses"][module], s["operation_state"][module])
        finally:
            self._lock.release()

    def prepare_memory_motion(self, target, speeds, cancel):
        from .memory_points import ARMS, require_idle
        require_idle(self.read_state())
        return {"start": self.read_state(), "target": target,
                "modes": {m: {"motion": "MoveL (MOCK)", "reason": ""} for m in ARMS}}

    def start_memory_arms(self, plan, cancel):
        from .memory_motion import check_cancel
        with self._lock:
            check_cancel(cancel)
            for name in ("left_arm", "right_arm"):
                self._state["joints_deg"][name] = list(plan["target"]["joints_deg"][name])
                self._state["poses"][name] = list(plan["target"]["poses"][name])
                self._state["arm_elbow_deg"][name] = plan["target"]["arm_elbow_deg"][name]

    def start_memory_head_trunk(self, plan, cancel):
        from .memory_motion import check_cancel
        with self._lock:
            check_cancel(cancel)
            for name in ("trunk", "head"):
                self._state["joints_deg"][name] = list(plan["target"]["joints_deg"][name])
            self._state["poses"]["trunk"] = list(plan["target"]["poses"]["trunk"])

    def stop_memory_motion(self):
        return []

    def move_joints(self, module: str, values_deg: list[float], speed_mm_s: float) -> None:
        del speed_mm_s
        with self._lock:
            self._state["joints_deg"][module] = list(values_deg)

    def move_pose(
        self,
        module: str,
        pose_mm_deg: list[float],
        speed_mm_s: float,
        elbow_deg: float | None = None,
    ) -> None:
        del speed_mm_s
        with self._lock:
            self._state["poses"][module] = list(pose_mm_deg)
            if module in self._state["arm_elbow_deg"] and elbow_deg is not None:
                self._state["arm_elbow_deg"][module] = float(elbow_deg)

    def set_drag(self, side: str, enabled: bool) -> None:
        with self._lock:
            self._state["dragging"][side] = enabled
            self._state["operation_state"][side] = "mock-drag" if enabled else "mock-idle"

    def suction_set(self, enabled, config):
        from .suction import validate_config
        validate_config(config)
        return {'commanded_open': enabled, 'confirmed': True}

    def gripper_status(self) -> dict[str, Any]:
        with self._lock:
            return {
                "model": "2F-85",
                "activated": self._gripper_activated,
                "going_to_position": self._gripper_activated,
                "activation_state": 3 if self._gripper_activated else 0,
                "object_state": 3 if self._gripper_activated else 0,
                "fault_code": 0,
                "requested_position": self._gripper_position,
                "measured_position": self._gripper_position,
                "motor_current_ma": 0,
            }

    def gripper_activate(self) -> dict[str, Any]:
        with self._lock:
            self._gripper_activated = True
            self._gripper_position = 0
            return self.gripper_status()

    def gripper_move(self, position: int) -> dict[str, Any]:
        with self._lock:
            if not self._gripper_activated:
                raise BackendError("夹爪尚未初始化")
            self._gripper_position = position
            return self.gripper_status()

    def gripper_start_move(self, position: int) -> None:
        self.gripper_move(position)

    def gripper_stop(self) -> dict[str, Any]:
        return {**self.gripper_status(), "going_to_position": False}

    def close(self) -> None:
        return


class XCoreRobotBackend:
    """Lazy AR xCore SDK adapter for two AR arms and one PCB4 trunk.

    Constructing this class does not connect to any controller. Connections are
    opened only when a readback or a user-authorized control operation is handled.
    """

    def __init__(self, config: dict[str, Any]) -> None:
        self._config = config
        self._sdk: Any | None = None
        self._robots: dict[str, Any] = {}
        self._connected: set[str] = set()
        self._dragging = {"left_arm": False, "right_arm": False}
        self._soft_limits_cache: dict[str, Any] | None = None
        self._lock = threading.RLock()

    def _load_sdk(self) -> Any:
        if self._sdk is not None:
            return self._sdk
        root = Path(self._config["sdk_root"]).expanduser().resolve()
        candidates = [
            root / "rokae_xcore",
            root / "Release" / "linux" / "arm",
            root / "Release" / "linux",
            root,
        ]
        for candidate in candidates:
            if candidate.exists() and str(candidate) not in sys.path:
                sys.path.insert(0, str(candidate))
        try:
            self._sdk = importlib.import_module("xCoreSDK_python")
        except Exception as exc:
            raise BackendError(
                f"无法加载 AR xCore SDK: {exc}。请确认 aarch64 Python 库位于 "
                f"{root}/rokae_xcore（或直接将 sdk_root 指向 rokae_xcore）"
            ) from exc
        missing = [name for name in ("ArRobot", "PCB4Robot") if not hasattr(self._sdk, name)]
        if missing:
            self._sdk = None
            raise BackendError(
                "已加载的 xCore SDK 不是所需的 AR 版本，缺少接口: " + ", ".join(missing)
            )
        return self._sdk

    def read_memory_state(self):
        from .memory_motion import capture
        return capture(self)

    def prepare_memory_motion(self, target, speeds, cancel):
        from .memory_motion import prepare
        return prepare(self, target, speeds, cancel)

    def start_memory_arms(self, plan, cancel):
        from .memory_motion import start_arms
        return start_arms(self, plan, cancel)

    def start_memory_head_trunk(self, plan, cancel):
        from .memory_motion import start_head_trunk
        return start_head_trunk(self, plan, cancel)

    def stop_memory_motion(self):
        from .memory_motion import stop
        return stop(self)

    @staticmethod
    def _check_ec(action: str, ec: dict[str, Any]) -> None:
        code = ec.get("ec", 0)
        if code:
            message = ec.get("message", "未知错误")
            raise BackendError(f"{action} 失败: {message} (ec={code})")

    def _call(self, action: str, function: Any, *args: Any) -> Any:
        ec: dict[str, Any] = {}
        result = invoke(getattr(self, "audit_callback", None), action, function, args, ec)
        self._check_ec(action, ec)
        return result

    def _robot(self, module: str) -> Any:
        sdk = self._load_sdk()
        controller = "trunk" if module in ("trunk", "head") else module
        if controller not in self._robots:
            try:
                with span(getattr(self, "audit_callback", None) or (lambda *a, **k: None),
                          "controller_connection", controller=controller):
                    if controller == "trunk":
                        robot = sdk.PCB4Robot(self._config["trunk_ip"])
                    else:
                        robot = sdk.ArRobot(
                            self._config[f"{controller}_ip"],
                            self._config["local_ip"],
                        )
            except Exception as exc:
                raise BackendError(f"连接 {controller} 失败: {exc}") from exc
            # AR SDK 的带地址构造函数会立即连接；不要再调用 connectToRobot()。
            self._robots[controller] = robot
            self._connected.add(controller)
        return self._robots[controller]

    @staticmethod
    def _rad_to_deg(values: list[float]) -> list[float]:
        return [round(math.degrees(float(value)), 6) for value in values]

    @staticmethod
    def _pose_to_ui(values: list[float]) -> list[float]:
        if len(values) != 6:
            raise BackendError(f"SDK 返回了非 6 维 Pose: {values}")
        return [
            round(float(values[0]) * 1000.0, 6),
            round(float(values[1]) * 1000.0, 6),
            round(float(values[2]) * 1000.0, 6),
            round(math.degrees(float(values[3])), 6),
            round(math.degrees(float(values[4])), 6),
            round(math.degrees(float(values[5])), 6),
        ]

    @staticmethod
    def _pose_to_sdk(values: list[float]) -> list[float]:
        return [
            float(values[0]) / 1000.0,
            float(values[1]) / 1000.0,
            float(values[2]) / 1000.0,
            math.radians(float(values[3])),
            math.radians(float(values[4])),
            math.radians(float(values[5])),
        ]

    @staticmethod
    def _world_from_ref(toolset: Any) -> list[float]:
        # SDK toolset.ref is the work-object pose in the SDK world, NOT in base.
        return list(toolset.ref.trans) + list(toolset.ref.rpy)

    def _arm_pose_in_world(self, robot: Any, cart: Any, module: str) -> list[float]:
        toolset = self._call(f"读取 {module} 当前工具工件", robot.toolset)
        try:
            return ref_pose_to_world(
                list(cart.trans) + list(cart.rpy), self._world_from_ref(toolset)
            )
        except (TypeError, ValueError, AttributeError) as exc:
            raise BackendError(f"{module} SDK 世界系坐标转换失败: {exc}") from exc

    def _read_trunk_head_rad(self, robot: Any) -> tuple[list[float], list[float]]:
        values = list(self._call("读取躯干/头部关节", robot.jointPos))
        trunk = values[:4]
        head = values[4:6]
        if len(trunk) != 4:
            raise BackendError(f"躯干控制器返回关节数异常: {len(values)}")
        if len(head) < 2:
            sdk = self._load_sdk()
            cart = self._call("读取躯干外部轴", robot.cartPosture, sdk.CoordinateType.endInRef)
            head = list(getattr(cart, "external", []))[:2]
        if len(head) != 2:
            raise BackendError("未从 PCB4 控制器读取到 2 个头部外部轴")
        return trunk, head

    def _operation_name(self, robot: Any, module: str) -> str:
        try:
            state = self._call(f"读取 {module} 运行状态", robot.operationState)
            return str(getattr(state, "name", state))
        except BackendError:
            return "unknown"

    def read_state(self) -> dict[str, Any]:
        with self._lock:
            sdk = self._load_sdk()
            left = self._robot("left_arm")
            right = self._robot("right_arm")
            trunk_robot = self._robot("trunk")
            trunk_rad, head_rad = self._read_trunk_head_rad(trunk_robot)
            trunk_tool = self._call("读取躯干工具工件", trunk_robot.toolset)
            left_cart = self._call(
                "读取左臂七轴笛卡尔状态",
                left.cartPosture,
                sdk.CoordinateType.endInRef,
            )
            right_cart = self._call(
                "读取右臂七轴笛卡尔状态",
                right.cartPosture,
                sdk.CoordinateType.endInRef,
            )
            state = {
                "joints_deg": {
                    "left_arm": self._rad_to_deg(list(self._call("读取左臂关节", left.jointPos))[:7]),
                    "right_arm": self._rad_to_deg(list(self._call("读取右臂关节", right.jointPos))[:7]),
                    "trunk": self._rad_to_deg(trunk_rad),
                    "head": self._rad_to_deg(head_rad),
                },
                "poses": {
                    "left_arm": self._pose_to_ui(self._arm_pose_in_world(left, left_cart, "left_arm")),
                    "right_arm": self._pose_to_ui(self._arm_pose_in_world(right, right_cart, "right_arm")),
                    "trunk": self._pose_to_ui(list(self._call("读取躯干 Pose", trunk_robot.posture, sdk.CoordinateType.endInRef))),
                },
                "pose_frames": dict(POSE_FRAMES),
                "toolsets": {"trunk": {
                    "end": list(trunk_tool.end.trans) + list(trunk_tool.end.rpy),
                    "ref": list(trunk_tool.ref.trans) + list(trunk_tool.ref.rpy),
                }},
                "arm_elbow_deg": {
                    "left_arm": round(math.degrees(float(left_cart.elbow)), 6),
                    "right_arm": round(math.degrees(float(right_cart.elbow)), 6),
                },
                "dragging": dict(self._dragging),
                "operation_state": {
                    "left_arm": self._operation_name(left, "left_arm"),
                    "right_arm": self._operation_name(right, "right_arm"),
                    "trunk": self._operation_name(trunk_robot, "trunk"),
                },
            }
            for module, count in JOINT_COUNTS.items():
                if len(state["joints_deg"][module]) != count:
                    raise BackendError(f"{module} 关节回读数量不正确")
            if getattr(self, "audit_callback", None):
                self.audit_callback("robot_state_observed", state=state)
            return state

    def read_telemetry_module(self, module):
        if module not in POSE_MODULES:
            raise BackendError("未知上身遥测模块")
        if not self._lock.acquire(blocking=False):
            raise TelemetryBusy()
        try:
            sdk, robot = self._load_sdk(), self._robot(module)
            joints = list(self._call(f"遥测 {module} 关节", robot.jointPos))[:JOINT_COUNTS[module]]
            cart = self._call(f"遥测 {module} 法兰基座位姿", robot.cartPosture, sdk.CoordinateType.flangeInBase)
            state = self._call(f"遥测 {module} 运行状态", robot.operationState)
            return module_data(module, self._rad_to_deg(joints),
                self._pose_to_ui(list(cart.trans) + list(cart.rpy)), str(getattr(state, "name", state)))
        finally:
            self._lock.release()

    @staticmethod
    def _copy_soft_limit_status(status: dict[str, Any]) -> dict[str, Any]:
        return {
            "joint_limits_deg": {
                module: [list(pair) for pair in pairs]
                for module, pairs in status["joint_limits_deg"].items()
            },
            "joint_soft_limit_enabled": dict(status["joint_soft_limit_enabled"]),
            "joint_limit_source": status["joint_limit_source"],
        }

    def soft_limit_status(self) -> dict[str, Any]:
        """Read controller soft limits once per backend lifetime without changing them."""
        with self._lock:
            if self._soft_limits_cache is not None:
                return self._copy_soft_limit_status(self._soft_limits_cache)

            sdk = self._load_sdk()
            limits_deg: dict[str, list[list[float]]] = {}
            enabled: dict[str, bool] = {}
            for module in ("left_arm", "right_arm", "trunk"):
                robot = self._robot(module)
                holder = sdk.PyTypeVectorArrayDouble2()
                enabled[module] = bool(
                    self._call(f"读取 {module} 控制器软限位", robot.getSoftLimit, holder)
                )
                raw_limits = list(holder.content())
                expected = JOINT_COUNTS[module]
                if len(raw_limits) != expected:
                    raise BackendError(
                        f"{module} 控制器返回软限位数量异常: {len(raw_limits)}"
                    )
                limits_deg[module] = [
                    [round(math.degrees(float(pair[0])), 6), round(math.degrees(float(pair[1])), 6)]
                    for pair in raw_limits
                ]

            trunk_robot = self._robot("trunk")
            axes = sdk.PyTypeVectorString()
            self._call("读取头部外部轴列表", trunk_robot.getMechUnit, "u1", "axes_info", axes)
            axis_names = list(axes.content())
            if len(axis_names) != JOINT_COUNTS["head"]:
                raise BackendError(f"头部外部轴数量异常: {len(axis_names)}")
            head_limits: list[list[float]] = []
            for axis_name in axis_names:
                lower = sdk.PyTypeDouble()
                upper = sdk.PyTypeDouble()
                self._call(
                    f"读取 {axis_name} 软限位下限",
                    trunk_robot.getExtAxisInfo,
                    axis_name,
                    "soft_limit_lower",
                    lower,
                )
                self._call(
                    f"读取 {axis_name} 软限位上限",
                    trunk_robot.getExtAxisInfo,
                    axis_name,
                    "soft_limit_upper",
                    upper,
                )
                # getExtAxisInfo returns rotational external-axis limits in degrees.
                head_limits.append([round(float(lower.content()), 6), round(float(upper.content()), 6)])
            unit_enabled = sdk.PyTypeBool()
            self._call(
                "读取头部机械单元启用状态",
                trunk_robot.getMechUnit,
                "u1",
                "enable",
                unit_enabled,
            )
            limits_deg["head"] = head_limits
            enabled["head"] = bool(unit_enabled.content())

            status = {
                "joint_limits_deg": limits_deg,
                "joint_soft_limit_enabled": enabled,
                "joint_limit_source": "xCore 控制器软限位（本次服务启动首次读取）",
            }
            self._soft_limits_cache = status
            return self._copy_soft_limit_status(status)

    def _prepare_motion(self, robot: Any, module: str, speed_mm_s: float) -> None:
        sdk = self._load_sdk()
        current_state = self._operation_name(robot, module)
        if current_state.lower() == "drag":
            raise BackendError(f"{module} 仍处于拖拽状态")
        self._call(f"{module} 切换自动模式", robot.setOperateMode, sdk.OperateMode.automatic)
        self._call(f"{module} 上电", robot.setPowerState, True)
        self._call(
            f"{module} 设置非实时指令模式",
            robot.setMotionControlMode,
            sdk.MotionControlMode.NrtCommandMode,
        )
        self._call(f"{module} 设置速度", robot.setDefaultSpeed, float(speed_mm_s))
        self._call(f"{module} 运动重置", robot.moveReset)

    def _send_command(self, robot: Any, module: str, command: Any) -> None:
        sdk = self._load_sdk()
        command_id = sdk.PyString()
        self._call(f"{module} 下发运动指令", robot.moveAppend, [command], command_id)
        self._call(f"{module} 启动运动", robot.moveStart)

    def move_joints(self, module: str, values_deg: list[float], speed_mm_s: float) -> None:
        with self._lock:
            sdk = self._load_sdk()
            robot = self._robot(module)
            self._prepare_motion(robot, module, speed_mm_s)
            radians = [math.radians(value) for value in values_deg]
            if module in ("left_arm", "right_arm"):
                target = sdk.JointPosition(radians)
            else:
                trunk_rad, head_rad = self._read_trunk_head_rad(robot)
                if module == "trunk":
                    trunk_rad = radians
                else:
                    head_rad = radians
                target = sdk.JointPosition(trunk_rad)
                target.external = head_rad
            command = sdk.MoveAbsJCommand(target, float(speed_mm_s), 0.0)
            self._send_command(robot, module, command)

    def move_pose(
        self,
        module: str,
        pose_mm_deg: list[float],
        speed_mm_s: float,
        elbow_deg: float | None = None,
    ) -> None:
        with self._lock:
            sdk = self._load_sdk()
            robot = self._robot(module)
            if module in ("left_arm", "right_arm"):
                current = self._call(
                    f"读取 {module} 当前臂角",
                    robot.cartPosture,
                    sdk.CoordinateType.endInRef,
                )
                toolset = self._call(f"读取 {module} 当前工具工件", robot.toolset)
                try:
                    target_values = world_pose_to_ref(
                        self._pose_to_sdk(pose_mm_deg), self._world_from_ref(toolset)
                    )
                except (TypeError, ValueError, AttributeError) as exc:
                    raise BackendError(f"{module} SDK 世界系目标转换失败: {exc}") from exc
                # MoveJ and calcIk both consume the SAME current reference frame.
                # Preserve the controller's calibrated work object, TCP and load.
                target = sdk.CartesianPosition(target_values)
                target.elbow = (
                    float(current.elbow)
                    if elbow_deg is None
                    else math.radians(float(elbow_deg))
                )
                target.hasElbow = True
                target.confData = list(current.confData)
                ec: dict[str, Any] = {}
                robot.model().calcIk(target, toolset, ec)
                self._check_ec(f"{module} Pose 逆解预检", ec)
            else:
                target = sdk.CartesianPosition(self._pose_to_sdk(pose_mm_deg))
                _, head_rad = self._read_trunk_head_rad(robot)
                target.external = head_rad
            self._prepare_motion(robot, module, speed_mm_s)
            command = sdk.MoveJCommand(target, float(speed_mm_s), 0.0)
            self._send_command(robot, module, command)

    def set_drag(self, side: str, enabled: bool) -> None:
        with self._lock:
            sdk = self._load_sdk()
            robot = self._robot(side)
            if enabled:
                self._call(f"{side} 下电", robot.setPowerState, False)
                self._call(f"{side} 切换手动模式", robot.setOperateMode, sdk.OperateMode.manual)
                self._call(f"{side} 运动重置", robot.moveReset)
                ec: dict[str, Any] = {}
                def enable_drag(space, kind, non_button, error):
                    return robot.enableDrag(space, kind, error, non_button)
                invoke(getattr(self, "audit_callback", None), f"{side} 开启末端按钮门控自由拖拽",
                       enable_drag, (sdk.DragParameterSpace.cartesianSpace, sdk.DragParameterType.freely, False), ec)
                self._check_ec(f"{side} 开启末端按钮门控自由拖拽", ec)
            else:
                self._call(f"{side} 关闭拖拽", robot.disableDrag)
            self._dragging[side] = enabled

    def suction_set(self, enabled, config):
        from .suction import set_output, SuctionError
        with self._lock:
            try:
                return set_output(self._load_sdk(), self._robot('left_arm'), config, enabled,
                                  getattr(self, 'audit_callback', None))
            except SuctionError as exc:
                raise BackendError(str(exc)) from exc

    def gripper_status(self) -> dict[str, Any]:
        with self._lock:
            try:
                return Robotiq2F85(self._load_sdk(), self._robot("right_arm"), getattr(self, "audit_callback", None)).status()
            except GripperError as exc:
                raise BackendError(str(exc)) from exc

    def gripper_activate(self) -> dict[str, Any]:
        with self._lock:
            try:
                return Robotiq2F85(self._load_sdk(), self._robot("right_arm"), getattr(self, "audit_callback", None)).activate()
            except GripperError as exc:
                raise BackendError(str(exc)) from exc

    def gripper_move(self, position: int) -> dict[str, Any]:
        with self._lock:
            try:
                return Robotiq2F85(self._load_sdk(), self._robot("right_arm"), getattr(self, "audit_callback", None)).move(position)
            except GripperError as exc:
                raise BackendError(str(exc)) from exc

    def gripper_start_move(self, position: int) -> None:
        with self._lock:
            try:
                Robotiq2F85(self._load_sdk(), self._robot("right_arm"), getattr(self, "audit_callback", None)).start_move(position)
            except GripperError as exc:
                raise BackendError(str(exc)) from exc

    def gripper_stop(self) -> dict[str, Any]:
        with self._lock:
            try:
                return Robotiq2F85(self._load_sdk(), self._robot("right_arm"), getattr(self, "audit_callback", None)).stop()
            except GripperError as exc:
                raise BackendError(str(exc)) from exc

    def close(self) -> None:
        with self._lock:
            for name, robot in self._robots.items():
                if name not in self._connected:
                    continue
                try:
                    self._call(f"断开 {name}", robot.disconnectFromRobot)
                except Exception:
                    pass
            self._connected.clear()


class MockChassisBackend:
    def __init__(self) -> None:
        self.enabled = False
        self.obstacle_avoidance_enabled = True
        self.last_velocity = [0.0, 0.0, 0.0]

    def status(self) -> dict[str, Any]:
        return {"available": True, "ros_ready": True, "remote_enabled": self.enabled,
                "obstacle_avoidance": {"enabled": self.obstacle_avoidance_enabled,
                                       "service_ready": True, "stale": False,
                                       "age_seconds": 0.0, "source": "mock", "error": None},
                "battery": {"available": False, "percentage": None, "charging": None,
                            "source": "mock", "error": "MOCK 模式无真实电池数据"}}

    def set_enabled(self, enabled: bool) -> None:
        self.enabled = enabled
        # The AMR service restores avoidance when switching remote mode.
        self.obstacle_avoidance_enabled = True
        if not enabled:
            self.stop()

    def set_obstacle_avoidance(self, enabled: bool) -> None:
        self.obstacle_avoidance_enabled = enabled

    def release_emergency_stop(self) -> dict[str, Any]:
        self.stop()
        self.enabled = False
        return {"released": True, "message": "MOCK 底盘急停已解除"}

    def command(self, vx: float, vy: float, wz: float) -> None:
        if not self.enabled:
            raise BackendError("底盘遥控尚未启用")
        self.last_velocity = [vx, vy, wz]

    def stop(self) -> None:
        self.last_velocity = [0.0, 0.0, 0.0]

    def close(self) -> None:
        self.stop()


class Ros2ChassisBackend:
    """ROS2 publisher/client with a server-side velocity command lease."""

    def __init__(self, config: dict[str, Any]) -> None:
        self._config = config
        self._lock = threading.RLock()
        self._node: Any | None = None
        self._publisher: Any | None = None
        self._client: Any | None = None
        self._emergency_stop_client: Any | None = None
        self._obstacle_avoidance_client: Any | None = None
        self._system_state_subscription: Any | None = None
        self._obstacle_enabled: bool | None = None
        self._obstacle_updated_at: float | None = None
        self._obstacle_source = "ros2"
        self._obstacle_error: str | None = None
        self._executor: Any | None = None
        self._spin_thread: threading.Thread | None = None
        self._watchdog_thread: threading.Thread | None = None
        self._running = False
        self._remote_enabled = False
        self._last_command = 0.0
        self._last_velocity = (0.0, 0.0, 0.0)
        self._battery = BatteryTelemetry(config.get("battery_stale_seconds", 15.0))
        self._battery_subscription = None

    def _ensure_ros(self) -> None:
        with self._lock:
            if self._node is not None:
                return
            try:
                import rclpy
                from geometry_msgs.msg import Twist
                from rclpy.executors import SingleThreadedExecutor
                from std_srvs.srv import SetBool, Trigger
            except Exception as exc:
                raise BackendError(
                    f"ROS2 Python 环境不可用: {exc}。请先 source ROS Humble 和 AMR 工作空间。"
                ) from exc
            if not rclpy.ok():
                rclpy.init(args=None)
            node = rclpy.create_node(f"rokae_web_chassis_{int(time.time())}")
            publisher = node.create_publisher(Twist, self._config["cmd_vel_topic"], 10)
            client = node.create_client(SetBool, self._config["remote_control_service"])
            emergency_stop_client = node.create_client(
                Trigger,
                self._config["release_emergency_stop_service"],
            )
            obstacle_avoidance_client = node.create_client(
                SetBool, self._config.get("remote_obstacle_avoidance_service",
                                          "/sr_amr_control/remote_control_oba_enabled"),
            )
            try:
                from sr_amr_interfaces.msg import SystemState
                self._system_state_subscription = node.create_subscription(
                    SystemState, self._config.get("system_state_topic", "/sr_amr_control/system_state"),
                    self._update_obstacle_avoidance, 10,
                )
            except Exception as exc:
                self._obstacle_error = f"遥控避障状态订阅不可用: {exc}"
            try:
                from sr_amr_interfaces.msg import BatteryState
                self._battery_subscription = node.create_subscription(
                    BatteryState, self._config.get("battery_topic", "/sr_amr_control/battery_state"),
                    self._battery.update, 10,
                )
            except Exception as exc:
                # A telemetry dependency failure must not disable existing chassis controls.
                self._battery.unavailable(f"电池订阅不可用: {exc}")
            executor = SingleThreadedExecutor()
            executor.add_node(node)
            self._node = node
            self._publisher = publisher
            self._client = client
            self._emergency_stop_client = emergency_stop_client
            self._obstacle_avoidance_client = obstacle_avoidance_client
            self._executor = executor
            self._Twist = Twist
            self._SetBool = SetBool
            self._Trigger = Trigger
            self._running = True
            self._spin_thread = threading.Thread(target=executor.spin, daemon=True)
            self._spin_thread.start()
            self._watchdog_thread = threading.Thread(target=self._watchdog, daemon=True)
            self._watchdog_thread.start()

    def _update_obstacle_avoidance(self, message: Any) -> None:
        value = getattr(message, "remote_control_oba_active", None)
        with self._lock:
            if type(value) is not bool:
                self._obstacle_updated_at = None
                self._obstacle_error = "遥控避障状态无效"
                return
            self._obstacle_enabled = value
            self._obstacle_updated_at = time.monotonic()
            self._obstacle_source = "ros2_topic"
            self._obstacle_error = None

    def _obstacle_status(self, service_ready: bool, error: str | None = None) -> dict[str, Any]:
        with self._lock:
            age = None if self._obstacle_updated_at is None else max(0.0, time.monotonic()-self._obstacle_updated_at)
            stale = age is None or age >= float(self._config.get("state_stale_seconds", 3.0))
            return {"enabled": None if stale or error else self._obstacle_enabled,
                    "service_ready": service_ready, "stale": stale or bool(error),
                    "age_seconds": age, "source": self._obstacle_source,
                    "error": error or self._obstacle_error or ("遥控避障状态已过期或尚未收到" if stale else None)}

    def status(self) -> dict[str, Any]:
        try:
            # Discover the chassis service without changing its control state.
            self._ensure_ros()
            ready = bool(self._client and self._client.service_is_ready())
            emergency_stop_ready = bool(
                self._emergency_stop_client
                and self._emergency_stop_client.service_is_ready()
            )
            obstacle_ready = bool(self._obstacle_avoidance_client
                                  and self._obstacle_avoidance_client.service_is_ready())
        except Exception as exc:
            self._battery.unavailable(str(exc))
            return {
                "available": True,
                "ros_ready": False,
                "release_emergency_stop_ready": False,
                "remote_enabled": self._remote_enabled,
                "error": str(exc),
                "obstacle_avoidance": self._obstacle_status(False, str(exc)),
                "battery": self._battery.snapshot(),
            }
        return {
            "available": True,
            "ros_ready": ready,
            "release_emergency_stop_ready": emergency_stop_ready,
            "remote_enabled": self._remote_enabled,
            "error": None if ready else "底盘服务未就绪或已离线；上半身可独立使用",
            "obstacle_avoidance": self._obstacle_status(obstacle_ready),
            "battery": self._battery.snapshot(),
        }

    @staticmethod
    def _remote_control_error(message: Any) -> str:
        code = str(message).strip()
        if code == "70002":
            return "底盘处于急停状态（70002）；请先物理释放急停并解除底盘急停"
        if code == "70003":
            return "底盘尚未解抱闸（70003）"
        if code == "70004":
            return "底盘仍有移动任务执行，不能切换遥控（70004）"
        if code == "70005":
            return "底盘仍有 Mission 执行，不能切换遥控（70005）"
        return code

    def set_enabled(self, enabled: bool) -> None:
        self._ensure_ros()
        if not self._client.wait_for_service(timeout_sec=2.0):
            raise BackendError(f"找不到服务 {self._config['remote_control_service']}")
        if not enabled:
            self.stop()
        request = self._SetBool.Request()
        request.data = bool(enabled)
        future = self._client.call_async(request)
        deadline = time.monotonic() + 7.0
        while not future.done() and time.monotonic() < deadline:
            time.sleep(0.05)
        if not future.done():
            raise BackendError("切换底盘遥控状态超时")
        response = future.result()
        if response is None or not response.success:
            message = getattr(response, "message", "无响应") if response else "无响应"
            raise BackendError(
                f"切换底盘遥控失败: {self._remote_control_error(message)}"
            )
        self._remote_enabled = bool(enabled)

    def release_emergency_stop(self) -> dict[str, Any]:
        self._ensure_ros()
        client = self._emergency_stop_client
        service_name = self._config["release_emergency_stop_service"]
        if not client.wait_for_service(timeout_sec=2.0):
            raise BackendError(f"找不到服务 {service_name}；请确认统一硬件服务中的底盘节点已启动")
        self.stop()
        self._remote_enabled = False
        future = client.call_async(self._Trigger.Request())
        deadline = time.monotonic() + 7.0
        while not future.done() and time.monotonic() < deadline:
            time.sleep(0.05)
        if not future.done():
            raise BackendError("解除底盘急停超时")
        response = future.result()
        message = getattr(response, "message", "") if response else "无响应"
        if response is None or not response.success:
            raise BackendError(f"解除底盘急停失败: {message or '控制器拒绝'}")
        return {"released": True, "message": message or "底盘急停已解除"}

    def set_obstacle_avoidance(self, enabled: bool) -> None:
        self._ensure_ros()
        client = self._obstacle_avoidance_client
        if client is None or not client.wait_for_service(timeout_sec=2.0):
            raise BackendError("找不到手动遥控避障服务")
        request = self._SetBool.Request()
        request.data = enabled
        try:
            future = client.call_async(request)
            deadline = time.monotonic()+7.0
            while not future.done() and time.monotonic() < deadline:
                time.sleep(.05)
            if not future.done():
                raise BackendError("切换遥控避障超时，结果未确认；请等待状态回读")
            response = future.result()
            if response is None or not response.success:
                detail = getattr(response, "message", "无响应") if response else "无响应"
                raise BackendError(f"切换遥控避障失败: {detail}")
        except Exception:
            with self._lock:
                self._obstacle_updated_at = None
                self._obstacle_error = "上次遥控避障切换结果未确认，请等待状态回读"
            raise
        # The existing AMR service confirms its live SystemState matches the
        # request before returning success; dispatch alone is not confirmation.
        with self._lock:
            self._obstacle_enabled = enabled
            self._obstacle_updated_at = time.monotonic()
            self._obstacle_source = "ros2_service_confirmed"
            self._obstacle_error = None

    def _publish(self, vx: float, vy: float, wz: float) -> None:
        message = self._Twist()
        message.linear.x = float(vx)
        message.linear.y = float(vy)
        message.angular.z = float(wz)
        self._publisher.publish(message)
        if getattr(self, "audit_callback", None):
            self.audit_callback("chassis_velocity_published", linear_x=vx, linear_y=vy, angular_z=wz,
                                topic=self._config.get("cmd_vel_topic"))

    def command(self, vx: float, vy: float, wz: float) -> None:
        self._ensure_ros()
        if not self._remote_enabled:
            raise BackendError("底盘遥控尚未启用")
        with self._lock:
            self._publish(vx, vy, wz)
            self._last_control_trace = fields()
            self._last_velocity = (vx, vy, wz)
            self._last_command = time.monotonic()

    def stop(self) -> None:
        with self._lock:
            if self._publisher is not None:
                self._publish(0.0, 0.0, 0.0)
            self._last_velocity = (0.0, 0.0, 0.0)
            self._last_command = 0.0

    def _watchdog(self) -> None:
        lease = float(self._config["lease_seconds"])
        while self._running:
            time.sleep(min(0.1, lease / 3.0))
            with self._lock:
                moving = any(abs(value) > 1e-9 for value in self._last_velocity)
                expired = self._last_command and time.monotonic() - self._last_command > lease
                if moving and expired:
                    if getattr(self, "audit_callback", None):
                        self.audit_callback("chassis_lease_expired", source_context=getattr(self, "_last_control_trace", {}))
                    self._publish(0.0, 0.0, 0.0)
                    self._last_velocity = (0.0, 0.0, 0.0)
                    self._last_command = 0.0

    def close(self) -> None:
        self._running = False
        try:
            self.stop()
        except Exception:
            pass
        if self._executor is not None:
            self._executor.shutdown(timeout_sec=1.0)
        if self._node is not None:
            self._node.destroy_node()
