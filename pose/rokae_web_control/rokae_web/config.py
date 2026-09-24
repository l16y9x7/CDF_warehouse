from __future__ import annotations

import copy
import json
from pathlib import Path
from typing import Any


DEFAULT_CONFIG: dict[str, Any] = {
    "web": {
        "host": "0.0.0.0",
        "port": 8088,
        "display_host": None,
    },
    "xcore": {
        "sdk_root": "/home/admin/mui/xCoreSDK-Python-AR-v0.7.1.ar_4",
        "local_ip": "192.168.71.51",
        "left_arm_ip": "192.168.71.161",
        "right_arm_ip": "192.168.71.160",
        "trunk_ip": "192.168.71.162",
    },
    "hardware_service": {"socket_path": "/home/admin/mui/rokae_web_control/.run/sdk.sock", "host": "0.0.0.0", "port": 8092},
    "telemetry": {
        "poll_interval_seconds": 1.0,
        "stale_after_seconds": 3.0,
    },
    "manipulator_state": {
        "sample_hz": 10,
        "freshness_ms": 500,
        "log_query_interval_sec": 5,
        "gripper_query_interval_sec": 1,
        "include_grippers": True,
    },
    "motion": {
        "default_speed_mm_s": 50.0,
        "min_speed_mm_s": 5.0,
        "max_speed_mm_s": 1000.0,
        "default_rotation_deg_s": 6.0,
        "min_rotation_deg_s": 0.1,
        "max_rotation_deg_s": 200.0,
        # 控制器软限位的备用快照。硬件模式会在服务启动后的首次状态请求中读取并覆盖；
        # 这里只用于 MOCK 模式或控制器暂时无法读取时的页面提示。
        "joint_limits_deg": {
            "left_arm": [
                [-178.0, 178.0],
                [-120.0, 120.0],
                [-178.0, 178.0],
                [-60.0, 145.0],
                [-178.0, 178.0],
                [-50.0, 50.0],
                [-50.0, 50.0],
            ],
            "right_arm": [
                [-178.0, 178.0],
                [-120.0, 120.0],
                [-178.0, 178.0],
                [-60.0, 145.0],
                [-178.0, 178.0],
                [-50.0, 50.0],
                [-50.0, 50.0],
            ],
            "trunk": [
                [-68.0, 3.0],
                [-175.0, 49.0],
                [-128.0, 88.0],
                [-180.0, 90.0],
            ],
            "head": [
                [-185.0, 95.0],
                [-38.0, 26.0],
            ],
        },
        "joint_soft_limit_enabled": {
            "left_arm": True,
            "right_arm": True,
            "trunk": True,
            "head": True,
        },
        "joint_limit_source": "配置中的控制器软限位备用快照",
        "max_joint_step_deg": {
            "left_arm": None,
            "right_arm": None,
            "trunk": None,
            "head": None,
        },
    },
    "chassis": {
        "available": True,
        "remote_control_service": "/sr_amr_control/remote_control_enabled",
        "remote_obstacle_avoidance_service": "/sr_amr_control/remote_control_oba_enabled",
        "system_state_topic": "/sr_amr_control/system_state",
        "state_stale_seconds": 3.0,
        "release_emergency_stop_service": "/sr_amr_control/release_emergency_stop",
        "cmd_vel_topic": "/sr_amr_control/remote_control_cmd_vel",
        "lease_seconds": 0.6,
        "default_linear_m_s": 0.15,
        "default_angular_rad_s": 0.30,
        "min_linear_m_s": 0.02,
        "min_angular_rad_s": 0.05,
        "max_linear_m_s": 0.50,
        "max_angular_rad_s": 0.80,
    },
    "camera": {
        "data_directory": "/home/admin/mui/rokae_web_control/data",
        "rgb_width": 1280,
        "rgb_height": 720,
        "capture_fps": 15,
        "display_fps": 15,
        "preview_width": 1280,
        "preview_height": 720,
        "jpeg_quality": 80,
        "max_depth_mm": 10000,
    },
    "ros_camera": {
        # Resolve the camera domain without changing the chassis ROS environment.
        "setup_file": "/home/admin/vision/config/ros/setup.bash",
        "domain_id": None,
        "stale_seconds": 2.0,
        "capture_timeout_seconds": 3.0,
        "cameras": {
            "head": {
                "enabled": True,
                "model": "Orbbec Gemini 335L (ROS2)",
                "serial_number": "CP26363000CH",
                "width": 1280,
                "height": 720,
                "fps": 15,
                "topics": {
                    "color": "/camera/head/synced/color/image_raw",
                    "depth": "/camera/head/synced/depth/image_raw",
                    "info": "/camera/head/synced/color/camera_info",
                },
            },
        },
    },
    "realsense": {
        "serial_numbers": {
            "left_wrist": "261822073502",
            "right_wrist": "261722071442",
        },
        "rgb_width": 1280,
        "rgb_height": 720,
        "depth_width": 640,
        "depth_height": 480,
        "capture_fps": 15,
        "display_fps": 15,
        "preview_width": 1280,
        "preview_height": 720,
        "jpeg_quality": 80,
    },
    "pose_estimation": {
        "service_base_url": "http://211.137.21.33:25540",
        "health_timeout_seconds": 5.0,
        "request_timeout_seconds": 240.0,
        "fresh_frame_timeout_seconds": 3.0,
        "calibration_file": "/home/admin/mui/rokae_web_control/calibration/head_camera_handeye_20260914.json",
        "urdf_file": "/home/admin/mui/1.5整机urdf-0.8AR5-20260520.zip",
        "sku_typ": "bottle",
        "return_visualizations": True,
        "side": "RIGHT",
        "front_axis_source": "trunk_sdk_x",
        "grasp_height_trunk_mm_by_sku": {},
        "z_ref_mm": 1200.0,
        "body_radius_mm": 28.5,
        "fit_mode": "cylinder_3d",
        "front_rule": {
            "front_axis_chassis": [1.0, 0.0, 0.0],
            "front_origin_chassis": [0.0, 0.0, 0.0],
            "front_band_mm": 100000.0,
        },
        "stationary_tolerance_deg": 0.1,
        "jpeg_quality": 95,
        "axis_overlay_length_mm": 120.0,
    },
    "logging": {
        "directory": "/home/admin/mui/rokae_web_control/logs",
        "state_sample_interval_seconds": 0.25,
        "state_sample_max_seconds": 180.0,
    },
}


def _merge(target: dict[str, Any], source: dict[str, Any]) -> dict[str, Any]:
    for key, value in source.items():
        if isinstance(value, dict) and isinstance(target.get(key), dict):
            _merge(target[key], value)
        else:
            target[key] = value
    return target


def load_config(path: str | Path | None = None) -> dict[str, Any]:
    config = copy.deepcopy(DEFAULT_CONFIG)
    if path is None:
        return config
    config_path = Path(path)
    if not config_path.exists():
        raise FileNotFoundError(f"配置文件不存在: {config_path}")
    with config_path.open("r", encoding="utf-8") as handle:
        user_config = json.load(handle)
    if not isinstance(user_config, dict):
        raise ValueError("配置文件顶层必须是 JSON 对象")
    return _merge(config, user_config)
