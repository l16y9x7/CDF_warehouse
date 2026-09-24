#!/usr/bin/env python3
"""Read controller identity only; this script never powers or moves the robot."""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path
from typing import Any


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Read ROKAE controller UID and version")
    parser.add_argument(
        "--sdk-root",
        default="/home/admin/mui/xCoreSDK-Python-AR-v0.7.1.ar_4",
    )
    parser.add_argument("--local-ip", default="192.168.71.51")
    return parser.parse_args()


def check_ec(label: str, ec: dict[str, Any]) -> None:
    code = ec.get("ec", 0)
    if code:
        raise RuntimeError(f"{label}: {ec.get('message', 'unknown error')} (ec={code})")


def main() -> int:
    args = parse_args()
    sdk_path = Path(args.sdk_root).expanduser().resolve() / "rokae_xcore"
    sys.path.insert(0, str(sdk_path))
    import xCoreSDK_python as sdk

    controllers = (
        ("right_arm", "arm", "192.168.71.160"),
        ("left_arm", "arm", "192.168.71.161"),
        ("trunk", "pcb4", "192.168.71.162"),
    )
    failures = 0
    for name, kind, remote_ip in controllers:
        robot = None
        try:
            robot = (
                sdk.ArRobot(remote_ip, args.local_ip)
                if kind == "arm"
                else sdk.PCB4Robot(remote_ip)
            )
            ec: dict[str, Any] = {}
            info = robot.robotInfo(ec)
            check_ec(name, ec)
            print(
                json.dumps(
                    {
                        "controller": name,
                        "ip": remote_ip,
                        "uid": info.id,
                        "type": info.type,
                        "mac": info.mac,
                        "version": info.version,
                        "joint_num": info.joint_num,
                    },
                    ensure_ascii=False,
                )
            )
        except Exception as exc:
            failures += 1
            print(json.dumps({"controller": name, "ip": remote_ip, "error": str(exc)}))
        finally:
            if robot is not None:
                try:
                    ec = {}
                    robot.disconnectFromRobot(ec)
                except Exception:
                    pass
    return 1 if failures else 0


if __name__ == "__main__":
    raise SystemExit(main())
