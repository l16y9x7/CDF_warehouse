"""Explicitly gated, low-speed, monitored right-arm MOVE L execution.

Importing this module or running it without --execute cannot command motion.
The elbow plane is NOT a complete collision model. Use only with a person at
the robot, the scene checked, and the physical emergency stop in reach.
"""

from __future__ import annotations

import argparse
import json
import math
import os
import time
from datetime import datetime, timezone
from pathlib import Path

import numpy as np

from planner import CandidatePlan, Waypoint, plan
from sdk_readonly import RightArmReadOnly, _call


def hardware_web_server_pids() -> list[int]:
    found = []
    for entry in Path("/proc").iterdir():
        if not entry.name.isdigit() or int(entry.name) == os.getpid():
            continue
        try:
            command = (entry / "cmdline").read_bytes().replace(b"\0", b" ").decode(errors="ignore")
        except (OSError, ProcessLookupError):
            continue
        if "server.py" in command and "--hardware" in command:
            found.append(int(entry.name))
    return found


def _state(arm: RightArmReadOnly) -> tuple[np.ndarray, np.ndarray, str]:
    trunk = np.degrees(list(_call("read trunk joints", arm.trunk.jointPos))[:4])
    joints = np.degrees(list(_call("read arm joints", arm.right.jointPos))[:7])
    operation = _call("read operation state", arm.right.operationState)
    return trunk, joints, str(getattr(operation, "name", operation)).lower()


def _verify_idle_start(arm: RightArmReadOnly) -> None:
    trunk, joints, operation = _state(arm)
    if operation != "idle":
        raise RuntimeError(f"right arm is not idle: {operation}")
    if np.max(np.abs(trunk - arm.trunk_joints_deg)) > .1:
        raise RuntimeError("torso moved after planning")
    if np.max(np.abs(joints - arm.current_joints_deg)) > .1:
        raise RuntimeError("right arm moved after planning")


def _move_one(arm: RightArmReadOnly, waypoint: Waypoint, speed_mm_s: float) -> None:
    sdk = arm.sdk
    cart = arm.target_cartesian(waypoint.flange_world_mm, waypoint.arm_angle_deg)
    command = sdk.MoveLCommand(cart, speed_mm_s, 0.0)
    command_id = sdk.PyString()
    _call("clear completed motion queue", arm.right.moveReset)
    _call("append one MOVE L waypoint", arm.right.moveAppend, [command], command_id)
    _call("start one MOVE L waypoint", arm.right.moveStart)
    deadline = time.monotonic() + 90.0
    started_at = time.monotonic()
    seen_running = False
    while time.monotonic() < deadline:
        trunk, joints, operation = _state(arm)
        if np.max(np.abs(trunk - arm.trunk_joints_deg)) > .2:
            raise RuntimeError("torso moved during arm motion")
        if operation == "unknown":
            raise RuntimeError("controller operation state unknown")
        _, actual_elbow = arm.fk(joints)
        if actual_elbow[1] > arm.protection_plane_y_mm - arm.elbow_margin_mm:
            raise RuntimeError("live elbow crossed protection plane")
        if operation != "idle":
            seen_running = True
        if operation == "idle":
            joint_error = float(np.max(np.abs(joints - waypoint.joints_deg)))
            actual_flange, _ = arm.fk(joints)
            position_error = float(np.linalg.norm(actual_flange[:3, 3] - waypoint.flange_world_mm[:3, 3]))
            if joint_error <= 1.5 and position_error <= 5.0:
                return
            if seen_running or time.monotonic() - started_at > 10:
                raise RuntimeError(f"MOVE L waypoint mismatch: {joint_error:.2f} deg, {position_error:.2f} mm")
        time.sleep(.1)
    raise TimeoutError("MOVE L waypoint did not finish within 90 seconds")


def execute(arm: RightArmReadOnly, candidate: CandidatePlan, speed_mm_s: float, elbow_margin_mm: float) -> None:
    if not 0 < speed_mm_s <= 5.0:
        raise ValueError("execution speed must be within (0, 5] mm/s")
    arm.protection_plane_y_mm = candidate.plane_y_mm
    arm.elbow_margin_mm = elbow_margin_mm
    _verify_idle_start(arm)
    sdk = arm.sdk
    _call("switch automatic mode", arm.right.setOperateMode, sdk.OperateMode.automatic)
    _call("power right arm", arm.right.setPowerState, True)
    _call("select non-real-time command mode", arm.right.setMotionControlMode, sdk.MotionControlMode.NrtCommandMode)
    _call("set low speed", arm.right.setDefaultSpeed, speed_mm_s)
    try:
        for segment_name, segment in (("approach", candidate.approach), ("grasp", candidate.grasp)):
            for index, waypoint in enumerate(segment, start=1):
                _move_one(arm, waypoint, speed_mm_s)
                print(f"{segment_name} {index}/{len(segment)} reached", flush=True)
    except BaseException:
        try:
            _call("stop right arm", arm.right.stop)
        finally:
            raise


def main() -> None:
    parser = argparse.ArgumentParser(description="Guarded MOVE L; never moves without all explicit gates")
    parser.add_argument("target", type=Path)
    parser.add_argument("config", type=Path)
    parser.add_argument("--execute", action="store_true")
    parser.add_argument("--operator-present", action="store_true")
    parser.add_argument("--scene-clearance-verified", action="store_true")
    parser.add_argument("--base-stationary-confirmed", action="store_true")
    parser.add_argument("--speed-mm-s", type=float, default=2.0)
    args = parser.parse_args()
    if not (args.execute and args.operator_present and args.scene_clearance_verified and args.base_stationary_confirmed):
        parser.error("execution requires --execute, operator/scene/base confirmations")
    config = json.loads(args.config.read_text(encoding="utf-8"))
    if config.get("execution_enabled") is not True:
        parser.error("config execution_enabled is false")
    other_pids = hardware_web_server_pids()
    if other_pids:
        raise RuntimeError(f"hardware web service is running {other_pids}; stop it before direct SDK motion")
    target = json.loads(args.target.read_text(encoding="utf-8"))
    if target.get("frame") != "chassis_link" or target.get("unit") != "mm":
        raise ValueError("target frame/unit mismatch")
    capture_age = (datetime.now(timezone.utc) - datetime.fromisoformat(target["capture_time"])).total_seconds()
    if not 0 <= capture_age <= 600:
        raise ValueError("4090 observation is older than 10 minutes or in the future; recapture")
    arm = RightArmReadOnly(
        config["sdk_root"], config["urdf_zip"], config["local_ip"],
        config["right_arm_ip"], config["trunk_ip"],
    )
    try:
        candidate = plan(
            arm, arm.current_flange_world,
            np.asarray(target["flange_pregrasp_world"], dtype=float),
            np.asarray(target["flange_grasp_world"], dtype=float),
            arm.current_joints_deg, arm.arm_angle_deg,
            plane_y_mm=float(config["torso_plane_y_mm"]),
            elbow_margin_mm=float(config["elbow_plane_margin_mm"]),
            sample_mm=float(config["linear_sample_mm"]),
            max_joint_step_deg=float(config["max_joint_step_deg"]),
            max_arm_angle_change_deg=float(config["max_arm_angle_change_deg"]),
        )
        if candidate is None:
            raise RuntimeError("MOVE L sampled IK / elbow-plane check failed; no motion")
        print(f"candidate: {len(candidate.approach)} approach + {len(candidate.grasp)} grasp waypoints; "
              f"arm angle {candidate.selected_arm_angle_deg:.2f} deg; plane Y={candidate.plane_y_mm:.1f} mm")
        print("WARNING: this does not verify gripper, forearm, box, shelf or dynamic collision.")
        if input("Type EXECUTE MOVE L to start the monitored low-speed sequence: ").strip() != "EXECUTE MOVE L":
            print("cancelled; no motion sent")
            return
        execute(arm, candidate, args.speed_mm_s, float(config["elbow_plane_margin_mm"]))
        print("MOVE L sequence completed. Gripper was not closed.")
    finally:
        arm.close()


if __name__ == "__main__":
    main()
