#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
import math
import os
import signal
import sys
import time
from datetime import datetime
from pathlib import Path
from typing import Any


TRUNK_IP = "192.168.71.162"
J2_INDEX = 1
MAX_SOFT_LIMIT_OVERRUN_DEG = 10.0
MAX_JOINT_SPEED_RATIO = 0.10
PROJECT_ROOT = Path(__file__).resolve().parents[1]
BACKUP_DIR = PROJECT_ROOT / "logs" / "recovery_backups"


class RecoveryError(RuntimeError):
    pass


def check_ec(action: str, ec: dict[str, Any]) -> None:
    code = ec.get("ec", 0)
    if code:
        raise RecoveryError(f"{action}失败: {ec.get('message', '未知错误')} (ec={code})")


def call(action: str, function: Any, *args: Any) -> Any:
    ec: dict[str, Any] = {}
    result = function(*args, ec)
    check_ec(action, ec)
    return result


def set_soft_limits(robot: Any, enabled: bool, limits: list[list[float]]) -> None:
    ec: dict[str, Any] = {}
    robot.setSoftLimit(enabled, ec, limits)
    check_ec("设置躯干软限位", ec)


def read_soft_limits(robot: Any, sdk: Any) -> tuple[bool, list[list[float]]]:
    holder = sdk.PyTypeVectorArrayDouble2()
    ec: dict[str, Any] = {}
    enabled = bool(robot.getSoftLimit(holder, ec))
    check_ec("读取躯干软限位", ec)
    limits = [[float(pair[0]), float(pair[1])] for pair in holder.content()]
    if len(limits) != 4:
        raise RecoveryError(f"控制器返回软限位数量异常: {len(limits)}")
    return enabled, limits


def degrees(values: list[float]) -> list[float]:
    return [math.degrees(float(value)) for value in values]


def limits_degrees(values: list[list[float]]) -> list[list[float]]:
    return [[math.degrees(pair[0]), math.degrees(pair[1])] for pair in values]


def hardware_server_pids() -> list[int]:
    found: list[int] = []
    for entry in Path("/proc").iterdir():
        if not entry.name.isdigit() or int(entry.name) == os.getpid():
            continue
        try:
            command = (entry / "cmdline").read_bytes().replace(b"\0", b" ").decode(errors="ignore")
        except (FileNotFoundError, PermissionError, ProcessLookupError):
            continue
        if "server.py" in command and "--hardware" in command:
            found.append(int(entry.name))
    return sorted(found)


def import_sdk() -> Any:
    try:
        import xCoreSDK_python as sdk
    except Exception as exc:
        raise RecoveryError(f"无法加载 xCore SDK: {exc}") from exc
    return sdk


def connect(sdk: Any) -> Any:
    try:
        return sdk.PCB4Robot(TRUNK_IP)
    except Exception as exc:
        raise RecoveryError(f"连接躯干控制器 {TRUNK_IP} 失败: {exc}") from exc


def disconnect(robot: Any) -> None:
    try:
        call("断开躯干控制器", robot.disconnectFromRobot)
    except Exception as exc:
        print(f"警告：断开控制器时出错：{exc}", file=sys.stderr)


def print_state(robot: Any, sdk: Any) -> tuple[list[float], bool, list[list[float]]]:
    joint_rad = list(call("读取躯干/头部关节", robot.jointPos))
    if len(joint_rad) < 4:
        raise RecoveryError(f"控制器返回关节数异常: {len(joint_rad)}")
    enabled, soft_rad = read_soft_limits(robot, sdk)
    joint_deg = degrees(joint_rad)
    soft_deg = limits_degrees(soft_rad)
    power = call("读取上电状态", robot.powerState)
    mode = call("读取操作模式", robot.operateMode)
    state = call("读取运行状态", robot.operationState)

    print(f"控制器: {TRUNK_IP}")
    print(f"上电={getattr(power, 'name', power)}  模式={getattr(mode, 'name', mode)}  状态={getattr(state, 'name', state)}")
    print(f"躯干角度: {[round(value, 4) for value in joint_deg[:4]]}")
    print(f"头部/外部轴角度: {[round(value, 4) for value in joint_deg[4:]]}")
    print(f"软限位启用: {enabled}")
    for index, pair in enumerate(soft_deg, 1):
        marker = "  <-- 当前越界" if not pair[0] <= joint_deg[index - 1] <= pair[1] else ""
        print(f"J{index}: {pair[0]:.4f}° ～ {pair[1]:.4f}°{marker}")
    return joint_rad, enabled, soft_rad


def save_backup(
    joint_rad: list[float],
    enabled: bool,
    limits_rad: list[list[float]],
    target_deg: float,
) -> Path:
    BACKUP_DIR.mkdir(parents=True, exist_ok=True)
    stamp = datetime.now().astimezone().strftime("%Y%m%d-%H%M%S")
    path = BACKUP_DIR / f"trunk-j2-soft-limits-{stamp}.json"
    payload = {
        "created_at": datetime.now().astimezone().isoformat(timespec="seconds"),
        "controller_ip": TRUNK_IP,
        "soft_limit_enabled": enabled,
        "soft_limits_rad": limits_rad,
        "soft_limits_deg": limits_degrees(limits_rad),
        "joint_position_rad": joint_rad,
        "joint_position_deg": degrees(joint_rad),
        "planned_target_j2_deg": target_deg,
        "recovery_method": "temporarily_disable_soft_limit",
    }
    with path.open("x", encoding="utf-8") as handle:
        json.dump(payload, handle, ensure_ascii=False, indent=2)
        handle.write("\n")
    return path


def validate_plan(
    joint_rad: list[float],
    enabled: bool,
    limits_rad: list[list[float]],
    target_deg: float,
    joint_speed_ratio: float,
) -> None:
    joint_deg = degrees(joint_rad[:4])
    soft_deg = limits_degrees(limits_rad)
    current = joint_deg[J2_INDEX]
    lower, original_upper = soft_deg[J2_INDEX]
    if not enabled:
        raise RecoveryError("当前软限位已经关闭，脚本拒绝继续")
    for index, value in enumerate(joint_deg):
        if index == J2_INDEX:
            continue
        axis_lower, axis_upper = soft_deg[index]
        if not axis_lower <= value <= axis_upper:
            raise RecoveryError(f"J{index + 1} 也已越过软限位，脚本拒绝自动恢复")
    if current <= original_upper:
        raise RecoveryError(f"J2 当前 {current:.3f}° 已在上限 {original_upper:.3f}° 内，无需执行")
    if not lower + 1.0 <= target_deg <= original_upper - 1.0:
        raise RecoveryError(
            f"目标角必须至少离原软限位两端 1°；当前允许 {lower + 1:.3f}° ～ {original_upper - 1:.3f}°"
        )
    if current > original_upper + MAX_SOFT_LIMIT_OVERRUN_DEG:
        raise RecoveryError(
            f"J2 超出原软上限 {current - original_upper:.3f}°，超过脚本允许自动恢复的 "
            f"{MAX_SOFT_LIMIT_OVERRUN_DEG:g}°"
        )
    if not 0 < joint_speed_ratio <= MAX_JOINT_SPEED_RATIO:
        raise RecoveryError(f"关节速度比例必须在 0～{MAX_JOINT_SPEED_RATIO:g} 之间")


def show_controller_errors(robot: Any, sdk: Any) -> None:
    try:
        ec: dict[str, Any] = {}
        logs = robot.queryControllerLog(
            5,
            {sdk.LogInfoLevel.error, sdk.LogInfoLevel.warning},
            ec,
        )
        check_ec("读取控制器错误日志", ec)
        for item in logs:
            print(f"控制器日志 {item.id} {item.timestamp}: {item.content}", file=sys.stderr)
    except Exception as exc:
        print(f"读取控制器错误日志失败: {exc}", file=sys.stderr)


def safe_manual_power_off(robot: Any, sdk: Any) -> None:
    try:
        call("下电", robot.setPowerState, False)
    except Exception as exc:
        print(f"警告：下电失败：{exc}", file=sys.stderr)
    try:
        call("切换手动模式", robot.setOperateMode, sdk.OperateMode.manual)
    except Exception as exc:
        print(f"警告：切换手动模式失败：{exc}", file=sys.stderr)


def run_status() -> int:
    sdk = import_sdk()
    robot = connect(sdk)
    try:
        print_state(robot, sdk)
        pids = hardware_server_pids()
        if pids:
            print(f"硬件网页服务正在运行，PID: {pids}")
        return 0
    finally:
        disconnect(robot)


def run_recovery(args: argparse.Namespace) -> int:
    pids = hardware_server_pids()
    if pids:
        raise RecoveryError(
            f"硬件网页服务仍在运行（PID: {pids}）。请先在运行网页服务的终端按 Ctrl+C，再重新执行恢复。"
        )

    sdk = import_sdk()
    robot = connect(sdk)
    original_limits: list[list[float]] | None = None
    original_enabled = False
    soft_limit_disabled = False
    motion_started = False
    motion_succeeded = False
    restore_error: Exception | None = None
    try:
        joint_rad, original_enabled, original_limits = print_state(robot, sdk)
        validate_plan(
            joint_rad,
            original_enabled,
            original_limits,
            args.target_deg,
            args.joint_speed_ratio,
        )
        backup_path = save_backup(
            joint_rad,
            original_enabled,
            original_limits,
            args.target_deg,
        )
        print(f"原软限位备份: {backup_path}")
        print(
            "计划：临时关闭躯干软限位，"
            f"以 {args.joint_speed_ratio * 100:g}% 关节速度只将 J2 退到 {args.target_deg:g}°，"
            "然后恢复原软限位。"
        )
        phrase = input("确认现场安全后，输入 RECOVER J2 继续：").strip()
        if phrase != "RECOVER J2":
            print("已取消，控制器未修改。")
            return 1

        call("下电", robot.setPowerState, False)
        call("切换手动模式", robot.setOperateMode, sdk.OperateMode.manual)
        set_soft_limits(robot, False, original_limits)
        soft_limit_disabled = True
        temp_enabled, _ = read_soft_limits(robot, sdk)
        if temp_enabled:
            raise RecoveryError("临时关闭软限位后回读仍为启用状态")
        print("软限位已临时关闭并回读确认。")

        call("切换自动模式", robot.setOperateMode, sdk.OperateMode.automatic)
        call("上电", robot.setPowerState, True)
        call("设置非实时指令模式", robot.setMotionControlMode, sdk.MotionControlMode.NrtCommandMode)
        call("设置默认速度", robot.setDefaultSpeed, 5.0)
        call("运动重置", robot.moveReset)

        target_joints = list(joint_rad[:4])
        target_joints[J2_INDEX] = math.radians(args.target_deg)
        target = sdk.JointPosition(target_joints)
        target.external = list(joint_rad[4:])
        command = sdk.MoveAbsJCommand(target, 5.0, 0.0)
        command.jointSpeed = float(args.joint_speed_ratio)
        command_id = sdk.PyString()
        call("下发 J2 恢复指令", robot.moveAppend, [command], command_id)
        call("启动 J2 恢复运动", robot.moveStart)
        motion_started = True
        print(f"指令已启动，ID={command_id.content()}")

        deadline = time.monotonic() + args.timeout_seconds
        last_print = 0.0
        while time.monotonic() < deadline:
            current_rad = list(call("回读 J2", robot.jointPos))
            current_deg = math.degrees(current_rad[J2_INDEX])
            state = call("读取运动状态", robot.operationState)
            state_name = str(getattr(state, "name", state))
            now = time.monotonic()
            if now - last_print >= 0.5:
                print(f"J2={current_deg:.3f}°  状态={state_name}")
                last_print = now
            if abs(current_deg - args.target_deg) <= 0.5 and state_name.lower() == "idle":
                motion_succeeded = True
                break
            time.sleep(0.1)
        if not motion_succeeded:
            show_controller_errors(robot, sdk)
            raise RecoveryError("J2 未在超时前到达目标角")
        print(f"J2 已到达 {args.target_deg:g}° 附近。")
    finally:
        if soft_limit_disabled and original_limits is not None:
            if motion_started and not motion_succeeded:
                try:
                    call("停止恢复运动", robot.stop)
                except Exception as exc:
                    print(f"警告：停止运动失败：{exc}", file=sys.stderr)
            safe_manual_power_off(robot, sdk)
            try:
                set_soft_limits(robot, original_enabled, original_limits)
                restored_enabled, restored_limits = read_soft_limits(robot, sdk)
                restored_upper = math.degrees(restored_limits[J2_INDEX][1])
                expected_upper = math.degrees(original_limits[J2_INDEX][1])
                if restored_enabled != original_enabled or abs(restored_upper - expected_upper) > 0.05:
                    raise RecoveryError(
                        f"原软限位回读不一致：启用={restored_enabled}, J2上限={restored_upper:.3f}°"
                    )
                print(f"原软限位已恢复并回读确认：J2 上限 {restored_upper:.3f}°")
            except Exception as exc:
                restore_error = exc
                print(f"严重警告：恢复原软限位失败：{exc}", file=sys.stderr)
        disconnect(robot)
    if restore_error is not None:
        raise RecoveryError(f"J2 恢复流程结束，但原软限位恢复失败：{restore_error}")
    return 0


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="安全恢复越过软上限的躯干 J2")
    subparsers = parser.add_subparsers(dest="action", required=True)
    subparsers.add_parser("status", help="只读显示当前角度、状态和软限位")
    recover = subparsers.add_parser("recover", help="临时关闭软限位并低速退回 J2")
    recover.add_argument("--target-deg", type=float, default=45.0)
    recover.add_argument("--joint-speed-ratio", type=float, default=0.05)
    recover.add_argument("--timeout-seconds", type=float, default=30.0)
    return parser.parse_args()


def main() -> int:
    signal.signal(signal.SIGTERM, lambda *_: (_ for _ in ()).throw(KeyboardInterrupt()))
    args = parse_args()
    try:
        if args.action == "status":
            return run_status()
        return run_recovery(args)
    except KeyboardInterrupt:
        print("\n已中断；脚本将尝试停止运动并恢复原软限位。", file=sys.stderr)
        return 130
    except RecoveryError as exc:
        print(f"错误：{exc}", file=sys.stderr)
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
