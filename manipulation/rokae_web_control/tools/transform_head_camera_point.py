#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

import numpy as np

PROJECT_ROOT = Path(__file__).resolve().parents[1]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from rokae_web.head_kinematics import UpperBodySixDofKinematics, rotation_to_rpy_deg  # noqa: E402


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="将头部相机光学坐标系中的三维点换算到底盘 chassis_link 坐标系；不连接或控制机器人"
    )
    parser.add_argument("--point-mm", type=float, nargs=3, required=True, metavar=("X", "Y", "Z"))
    parser.add_argument("--state", required=True, help="包含躯干和头部关节角的 robot_state.json")
    parser.add_argument(
        "--calibration",
        default=str(PROJECT_ROOT / "calibration" / "head_camera_handeye_20260914.json"),
    )
    parser.add_argument(
        "--urdf",
        default="/home/admin/mui/1.5整机urdf-0.8AR5-20260520.zip",
    )
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    state = json.loads(Path(args.state).expanduser().read_text(encoding="utf-8"))
    calibration = json.loads(
        Path(args.calibration).expanduser().read_text(encoding="utf-8")
    )
    joint_values_deg = [
        *state["joints_deg"]["trunk"],
        *state["joints_deg"]["head"],
    ]
    head_from_camera = np.asarray(
        calibration["transform_head_link_from_camera_optical"]["matrix_4x4"],
        dtype=np.float64,
    )
    kinematics = UpperBodySixDofKinematics(args.urdf)
    chassis_from_camera = kinematics.camera_to_base(joint_values_deg, head_from_camera)
    point_camera_m = np.asarray(args.point_mm, dtype=np.float64) / 1000.0
    point_chassis_m = kinematics.point_camera_to_base(
        point_camera_m,
        joint_values_deg,
        head_from_camera,
    )
    result = {
        "input_point_camera_optical_mm": np.round(point_camera_m * 1000.0, 6).tolist(),
        "output_point_chassis_link_mm": np.round(point_chassis_m * 1000.0, 6).tolist(),
        "joint_order": [
            "trunk.J1",
            "trunk.J2",
            "trunk.J3",
            "trunk.J4",
            "head.O1",
            "head.O2",
        ],
        "joint_values_deg": joint_values_deg,
        "camera_pose_in_chassis": {
            "matrix_4x4": np.round(chassis_from_camera, 12).tolist(),
            "translation_mm": np.round(chassis_from_camera[:3, 3] * 1000.0, 6).tolist(),
            "rpy_deg": np.round(rotation_to_rpy_deg(chassis_from_camera[:3, :3]), 9).tolist(),
        },
    }
    print(json.dumps(result, ensure_ascii=False, indent=2))


if __name__ == "__main__":
    main()
