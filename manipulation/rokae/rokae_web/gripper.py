"""Robotiq 2F-85 Modbus RTU control through the ROKAE AR right xPanel."""

from __future__ import annotations

import time
from typing import Any
from .control_trace import invoke


class GripperError(RuntimeError):
    pass


class Robotiq2F85:
    SLAVE_ID = 9
    STATUS_REGISTER = 0x07D0
    COMMAND_REGISTER = 0x03E8
    SPEED = 20
    FORCE = 10

    def __init__(self, sdk: Any, right_arm: Any, audit=None) -> None:
        self.sdk = sdk
        self.right_arm = right_arm
        self.audit = audit

    @staticmethod
    def _check_ec(action: str, ec: dict[str, Any]) -> None:
        code = ec.get("ec", 0)
        if code:
            raise GripperError(f"{action}失败: {ec.get('message', '未知错误')} (ec={code})")

    def _read_once(self) -> dict[str, Any]:
        data = self.sdk.PyTypeVectorInt([0, 0, 0])
        ec: dict[str, Any] = {}
        invoke(self.audit, "读取夹爪寄存器", self.right_arm.XPRWModbusRTUReg,
               (self.SLAVE_ID, 0x03, self.STATUS_REGISTER, "uint16", 3, data, False), ec)
        self._check_ec("读取夹爪状态", ec)
        words = [int(value) for value in data.content()]
        if len(words) != 3 or any(value < 0 or value > 0xFFFF for value in words):
            raise GripperError(f"夹爪状态寄存器返回异常: {words}")
        flags = words[0] >> 8
        return {
            "model": "2F-85",
            "activated": bool(flags & 0x01),
            "going_to_position": bool(flags & 0x08),
            "activation_state": (flags >> 4) & 0x03,
            "object_state": (flags >> 6) & 0x03,
            "fault_code": words[1] >> 8,
            "requested_position": words[1] & 0xFF,
            "measured_position": words[2] >> 8,
            "motor_current_ma": (words[2] & 0xFF) * 10,
        }

    def status(self) -> dict[str, Any]:
        # The gripper reports 0x09 after one idle second; continuous reads
        # clear it. Do not mistake that first response for a persistent fault.
        status = self._read_once()
        for _ in range(5):
            if status["fault_code"] != 0x09:
                break
            time.sleep(0.1)
            status = self._read_once()
        return status

    def _write(self, action: int, position: int = 0) -> None:
        # Six Robotiq bytes: action, reserved, reserved, position, speed, force.
        speed = self.SPEED if action == 0x09 else 0
        force = self.FORCE if action == 0x09 else 0
        data = self.sdk.PyTypeVectorInt(
            [action << 8, position, (speed << 8) | force]
        )
        ec: dict[str, Any] = {}
        invoke(self.audit, "发送夹爪寄存器", self.right_arm.XPRWModbusRTUReg,
               (self.SLAVE_ID, 0x10, self.COMMAND_REGISTER, "uint16", 3, data, False), ec)
        self._check_ec("发送夹爪指令", ec)

    def _wait(self, ready: Any, timeout_seconds: float) -> dict[str, Any]:
        deadline = time.monotonic() + timeout_seconds
        while True:
            status = self.status()
            if status["fault_code"]:
                raise GripperError(f"夹爪故障码 0x{status['fault_code']:02X}")
            if ready(status):
                return status
            if time.monotonic() >= deadline:
                raise GripperError(f"夹爪动作等待超时，最后状态: {status}")
            time.sleep(0.1)

    def activate(self) -> dict[str, Any]:
        before = self.status()
        if before["fault_code"] not in (0, 0x07):
            raise GripperError(f"夹爪当前故障码 0x{before['fault_code']:02X}")
        if before["activation_state"] == 3:
            return before
        self._write(0)
        time.sleep(0.1)
        self._write(1)
        # On this 2F-85, activation first reached position 210 and then
        # returned to the open end near 3. Wait for the full calibration.
        return self._wait(
            lambda s: s["activation_state"] == 3 and s["measured_position"] <= 15,
            25.0,
        )

    def start_move(self, position: int) -> None:
        if isinstance(position, bool) or not isinstance(position, int) or not 0 <= position <= 255:
            raise ValueError("夹爪目标位置必须是 0–255 的整数")
        before = self.status()
        if before["fault_code"]:
            raise GripperError(f"夹爪当前故障码 0x{before['fault_code']:02X}")
        if before["activation_state"] != 3:
            raise GripperError("夹爪尚未初始化，请先点击“初始化夹爪”")
        self._write(0x09, position)

    def stop(self) -> dict[str, Any]:
        # rACT=1, rGTO=0: stop, preserving activation; never auto-release.
        self._write(0x01)
        return self._wait(lambda s: s["activated"] and not s["going_to_position"], 3.0)

    def move(self, position: int) -> dict[str, Any]:
        self.start_move(position)
        return self._wait(
            lambda s: s["going_to_position"]
            and s["requested_position"] == position
            and s["object_state"] != 0,
            30.0,
        )
