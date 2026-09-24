"""Native RGB-D capture with Tianji ROS names and explicit hardware serials."""
import json
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def build(context):
    config = json.loads(Path(LaunchConfiguration("vision_config").perform(context)).read_text())
    camera = LaunchConfiguration("camera").perform(context)
    if camera not in {"head", "hand_left", "hand_right"}:
        raise ValueError("Unsupported camera role")
    cfg = config["rokae"]["owner"]["cameras"].get(camera, {})
    if cfg.get("enabled") is not True:
        return []
    match = cfg.get("match", {})
    serial = str(match.get("value", "")).strip()
    if not serial:
        raise ValueError("Explicit camera serial is required")
    width, height = int(cfg.get("width", 640)), int(cfg.get("height", 480))
    fps = int(cfg.get("fps", cfg.get("ros_fps", 15)))
    if min(width, height, fps) <= 0:
        raise ValueError("Invalid color profile")
    if camera == "head":
        if match.get("type") not in {"orbbec", "orbbec_sdk"}:
            raise ValueError("Head requires explicit Orbbec serial")
        depth_width = int(cfg.get("depth_width", width))
        depth_height = int(cfg.get("depth_height", height))
        if min(depth_width, depth_height) <= 0:
            raise ValueError("Invalid RGB-D profile")
        return [Node(
            package="orbbec_camera", executable="orbbec_camera_node",
            name="head", namespace="/camera/head", output="screen", respawn=False,
            parameters=[{
                "camera_name": "head", "serial_number": serial,
                "color_width": width, "color_height": height, "color_fps": fps,
                "color_format": "MJPG", "enable_color": True,
                "depth_width": depth_width, "depth_height": depth_height, "depth_fps": fps,
                "depth_format": "Y16", "enable_depth": True,
                "enable_left_ir": False, "enable_right_ir": False,
                "enable_point_cloud": False, "enable_colored_point_cloud": False,
                # Gemini 335L: align native depth to the color optical frame.
                # Software alignment supports the configured unequal resolutions.
                "depth_registration": True, "align_mode": "SW",
                "enable_frame_sync": True, "frame_aggregate_mode": "full_frame",
                # default=RELIABLE matches synced_rgbd; Media BEST_EFFORT can still subscribe.
                "color_qos": "default", "depth_qos": "default",
                "color_camera_info_qos": "default", "depth_camera_info_qos": "default",
                "time_domain": "system", "enable_sync_host_time": False,
                "enumerate_net_device": False, "connection_delay": 100,
            }],
            remappings=[
                ("depth/image_raw", "aligned_depth_to_color/image_raw"),
                ("depth/camera_info", "depth/camera_info"),
            ],
        )]
    if match.get("type") != "realsense_serial":
        raise ValueError("Wrist camera requires explicit RealSense SDK serial")
    if str(cfg.get("backend", "realsense")).strip().lower() != "realsense":
        raise ValueError("Wrist ROS source requires backend=realsense")
    ros_name = "left_wrist" if camera == "hand_left" else "right_wrist"
    enable_depth_value = cfg.get("enable_depth", True)
    require_depth_value = cfg.get("require_depth", enable_depth_value)
    require_synced_value = cfg.get("require_synced", require_depth_value)
    if not all(isinstance(value, bool) for value in (
        enable_depth_value, require_depth_value, require_synced_value,
    )):
        raise ValueError("Depth and synchronization flags must be booleans")
    enable_depth = enable_depth_value
    require_depth = require_depth_value
    require_synced = require_synced_value
    if require_depth and not enable_depth:
        raise ValueError("Required depth must be enabled")
    if require_synced and not enable_depth:
        raise ValueError("Synchronized RGB-D requires depth")
    depth_width = int(cfg.get("depth_width", width))
    depth_height = int(cfg.get("depth_height", height))
    if enable_depth and min(depth_width, depth_height) <= 0:
        raise ValueError("Invalid RGB-D profile")
    if not enable_depth:
        # Keep disabled RealSense parameters syntactically valid even when a
        # site overlay carries zero/unused depth dimensions.
        depth_width, depth_height = width, height
    launch_file = Path(get_package_share_directory("realsense2_camera")) / "launch/rs_launch.py"
    return [IncludeLaunchDescription(PythonLaunchDescriptionSource(str(launch_file)), launch_arguments={
        "camera_namespace": "camera", "camera_name": ros_name, "serial_no": "_" + serial,
        "enable_color": "true", "rgb_camera.color_profile": f"{width}x{height}x{fps}",
        "enable_depth": str(enable_depth).lower(),
        "depth_module.depth_profile": f"{depth_width}x{depth_height}x{fps}",
        "align_depth.enable": str(enable_depth).lower(),
        "enable_sync": str(require_synced).lower(), "pointcloud.enable": "false",
        "enable_infra1": "false", "enable_infra2": "false", "initial_reset": "false",
    }.items())]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("vision_config"), DeclareLaunchArgument("camera"),
        OpaqueFunction(function=build),
    ])
