#!/usr/bin/env bash
set -eo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
# Stable SDK, chassis and camera services have independent boot lifetimes.
# This entry point neither starts nor stops those hardware services.
source /opt/ros/humble/setup.bash || echo "警告：ROS 环境不可用；上半身继续启动。" >&2
source /home/admin/agvsdk/standard_robots_amr_ros2-v1.3.0/install/setup.bash || echo "警告：底盘环境不可用；上半身继续启动。" >&2
export ROS_DOMAIN_ID=0
set -u
SDK_AR_DIR="/home/admin/mui/xCoreSDK-Python-AR-v0.7.1.ar_4/rokae_xcore"
export LD_LIBRARY_PATH="$SDK_AR_DIR:${LD_LIBRARY_PATH:-}"
if ! pgrep -f '/sr_amr_control/[c]ontrol_node' >/dev/null 2>&1; then
  echo "警告：独立底盘服务节点未运行；请检查 sr-amr-control.service，接口服务不会代启底盘。" >&2
fi
echo "启动网页共享业务核心与 Agent 接口；复用常驻 SDK、底盘和摄像头。"
exec /usr/bin/python3 "$SCRIPT_DIR/control_runtime.py" --config "$SCRIPT_DIR/config.json"
