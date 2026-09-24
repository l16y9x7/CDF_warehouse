# Source this file before ROS CLI tools or the SMT RGB-D capture consumer.
source /opt/ros/humble/setup.bash || return 1
while IFS= read -r vision_ros_setting || [[ -n "$vision_ros_setting" ]]; do
    [[ -z "$vision_ros_setting" || "$vision_ros_setting" == \#* ]] && continue
    export "$vision_ros_setting"
done < "$(dirname "${BASH_SOURCE[0]}")/ros.env"
unset vision_ros_setting
# Whole-host ROS domain (not vision-specific). Prefer an already-exported
# ROS_DOMAIN_ID; otherwise load /etc/ros-domain.env if present.
if [[ -z "${ROS_DOMAIN_ID:-}" && -f /etc/ros-domain.env ]]; then
    # shellcheck disable=SC1091
    source /etc/ros-domain.env
fi
if [[ -n "${ROS_DOMAIN_ID:-}" ]]; then
    export VISION_ROS_DOMAIN_ID="${ROS_DOMAIN_ID}"
fi
vision_ros_domain=$(/usr/bin/python3 "$(dirname "${BASH_SOURCE[0]}")/domain.py") || return 1
export ROS_DOMAIN_ID="$vision_ros_domain"
unset vision_ros_domain
