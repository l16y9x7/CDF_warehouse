"""Read current torso/arm and evaluate a guarded MOVE L candidate. No motion.

The base, box, and target must remain fixed from 4090 capture. A plan marked
PASS here is only IK + sampled elbow-plane feasibility, not collision safety.
"""

from __future__ import annotations

import argparse
import json
from datetime import datetime, timezone
from pathlib import Path

import numpy as np

from geometry import world_goal_in_arm_reference
from planner import plan
from sdk_readonly import RightArmReadOnly


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("target", type=Path, help="frozen chassis_link target JSON")
    parser.add_argument("config", type=Path)
    parser.add_argument("--base-stationary-confirmed", action="store_true")
    parser.add_argument("--output", type=Path, help="write a new JSON trajectory report; never overwrite")
    args = parser.parse_args()
    if not args.base_stationary_confirmed:
        parser.error("confirm base, box and target have not moved since capture")
    target = json.loads(args.target.read_text(encoding="utf-8"))
    config = json.loads(args.config.read_text(encoding="utf-8"))
    if target.get("frame") != "chassis_link" or target.get("unit") != "mm":
        raise ValueError("frozen target is not in chassis_link millimetres")
    if target.get("not_collision_verified") is not True:
        raise ValueError("unexpected frozen-target provenance")
    capture_age_s = (datetime.now(timezone.utc) - datetime.fromisoformat(target["capture_time"])).total_seconds()
    if capture_age_s < -10:
        raise ValueError("4090 capture timestamp is in the future")
    arm = RightArmReadOnly(
        config["sdk_root"], config["urdf_zip"], config["local_ip"],
        config["right_arm_ip"], config["trunk_ip"],
    )
    try:
        pregrasp = np.asarray(target["flange_pregrasp_world"], dtype=float)
        grasp = np.asarray(target["flange_grasp_world"], dtype=float)
        result = plan(
            arm,
            arm.current_flange_world,
            pregrasp,
            grasp,
            arm.current_joints_deg,
            arm.arm_angle_deg,
            plane_y_mm=float(config["torso_plane_y_mm"]),
            elbow_margin_mm=float(config["elbow_plane_margin_mm"]),
            sample_mm=float(config["linear_sample_mm"]),
            max_joint_step_deg=float(config["max_joint_step_deg"]),
            max_arm_angle_change_deg=float(config["max_arm_angle_change_deg"]),
        )
        summary = {
            "status": "SAMPLED_IK_PASS_NOT_COLLISION_CERTIFIED" if result else "PLAN_FAILED",
            "capture_time": target["capture_time"],
            "capture_age_seconds": round(capture_age_s, 1),
            "capture_older_than_10_minutes": capture_age_s > 600,
            "current_trunk_joints_deg": arm.trunk_joints_deg.tolist(),
            "current_arm_joints_deg": arm.current_joints_deg.tolist(),
            "current_arm_angle_deg": arm.arm_angle_deg,
            "right_sdk_world_origin_chassis_mm": arm.world_from_shoulder[:3, 3].tolist(),
            "pregrasp_flange_right_sdk_world_mm": world_goal_in_arm_reference(pregrasp, arm.world_from_shoulder)[:3, 3].tolist(),
            "grasp_flange_right_sdk_world_mm": world_goal_in_arm_reference(grasp, arm.world_from_shoulder)[:3, 3].tolist(),
            "torso_plane_y_mm": config["torso_plane_y_mm"],
            "selected_arm_angle_deg": result.selected_arm_angle_deg if result else None,
            "approach_samples": len(result.approach) if result else 0,
            "grasp_samples": len(result.grasp) if result else 0,
            "execution_allowed": False,
            "reason": "No validated collision model for torso, full arm, 255mm tool, box or shelf",
        }
        if result and args.output:
            if args.output.exists():
                raise FileExistsError(f"refusing to overwrite existing plan: {args.output}")
            summary["approach"] = [
                {"flange_world_mm": waypoint.flange_world_mm.tolist(),
                 "joints_deg": waypoint.joints_deg.tolist(),
                 "elbow_world_mm": waypoint.elbow_world_mm.tolist(),
                 "arm_angle_deg": waypoint.arm_angle_deg}
                for waypoint in result.approach
            ]
            summary["grasp"] = [
                {"flange_world_mm": waypoint.flange_world_mm.tolist(),
                 "joints_deg": waypoint.joints_deg.tolist(),
                 "elbow_world_mm": waypoint.elbow_world_mm.tolist(),
                 "arm_angle_deg": waypoint.arm_angle_deg}
                for waypoint in result.grasp
            ]
            args.output.write_text(json.dumps(summary, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
        print(json.dumps(summary, ensure_ascii=False, indent=2))
    finally:
        arm.close()


if __name__ == "__main__":
    main()
