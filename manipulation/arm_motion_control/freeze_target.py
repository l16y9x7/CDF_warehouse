"""Freeze a 4090 observation in chassis_link before the torso moves.

Usage: python3 freeze_target.py REQUEST.json RESPONSE.json OUT.json
This is geometry only. It never imports the robot SDK or sends motion.
"""

from __future__ import annotations

import argparse
import json
from datetime import datetime, timezone
from pathlib import Path

from localization import from_4090


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("request", type=Path)
    parser.add_argument("response", type=Path)
    parser.add_argument("output", type=Path)
    args = parser.parse_args()
    request = json.loads(args.request.read_text(encoding="utf-8"))
    response = json.loads(args.response.read_text(encoding="utf-8"))
    result = from_4090(request, response)
    data = {
        "frame": "chassis_link",
        "unit": "mm",
        "created_at": datetime.now(timezone.utc).isoformat(),
        "capture_time": result.capture_time,
        "capture_upper_body_joints_deg": request["upper_body_joints_deg"],
        "source_request_id": response.get("request_id"),
        "visible_top_world_mm": result.geometry.visible_top_world_mm.tolist(),
        "axis_up_world": result.geometry.axis_up_world.tolist(),
        "grasp_world_mm": result.geometry.grasp_world_mm.tolist(),
        "flange_grasp_world": result.geometry.flange_grasp_world.tolist(),
        "flange_pregrasp_world": result.geometry.flange_pregrasp_world.tolist(),
        "cylinder_band_points": result.cylinder_band_points,
        "camera_reference_error_mm": result.reference_error_mm,
        "rule": "99.5-percentile visible cylinder top -20mm; flange +Z 255mm; pregrasp +100mm",
        "not_collision_verified": True,
    }
    if args.output.exists():
        raise FileExistsError(f"refusing to overwrite existing target: {args.output}")
    args.output.write_text(json.dumps(data, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    print(json.dumps({key: value for key, value in data.items() if key != "flange_grasp_world" and key != "flange_pregrasp_world"}, ensure_ascii=False, indent=2))


if __name__ == "__main__":
    main()
