#!/usr/bin/env bash
set -eo pipefail

source /opt/ros/humble/setup.bash
source /home/admin/agvsdk/standard_robots_amr_ros2-v1.3.0/install/setup.bash
set -u

exec ros2 launch sr_amr_control amr_control.launch.py connect_ip:=192.168.71.50 lidar:=false
