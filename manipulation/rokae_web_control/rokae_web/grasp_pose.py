"""Read-only grasp-point geometry in chassis_link and the SDK right shoulder.

Lengths in this module are millimetres. The grasp uses assigned trunk Z+
through the recognized point and its calibrated trunk-height plane; object radius is not part of this geometry.
"""

from __future__ import annotations

import math
from typing import Any

import numpy as np

from .head_kinematics import rpy_rotation, rotation_to_rpy_deg
from .trunk_frame import TRUNK_FRAME
from .pose_protocol import sku_type


WORLD_GRASP_RPY_DEG = [180.0, -90.0, 0.0]
ASSIGNED_GRASP_RPY_TRUNK_DEG = [180.0, -90.0, 0.0]
GRIPPER_LENGTH_MM = 175.0
GRASP_Y_OFFSET_TRUNK_MM = 10.0
LEFT_GRIPPER_LENGTH_MM = 165.0


def _vector(values: Any, name: str) -> np.ndarray:
    vector = np.asarray(values, dtype=np.float64)
    if vector.shape != (3,) or not np.isfinite(vector).all():
        raise ValueError(f"{name} 必须是三个有限数值")
    return vector


def _transform(values: Any) -> np.ndarray:
    matrix = np.asarray(values, dtype=np.float64)
    if matrix.shape != (4, 4) or not np.isfinite(matrix).all():
        raise ValueError("拍照时相机到 chassis_link 的变换无效")
    rotation = matrix[:3, :3]
    if (not np.allclose(matrix[3], [0.0, 0.0, 0.0, 1.0], atol=1e-8)
            or not np.allclose(rotation.T @ rotation, np.eye(3), atol=2e-3)
            or not math.isclose(float(np.linalg.det(rotation)), 1.0, abs_tol=2e-3)):
        raise ValueError("拍照时相机到 chassis_link 的刚体变换无效")
    matrix = matrix.copy()
    matrix[:3, 3] *= 1000.0
    return matrix


def _point(transform_mm: np.ndarray, point_mm: np.ndarray) -> np.ndarray:
    return transform_mm[:3, :3] @ point_mm + transform_mm[:3, 3]


def world_grasp_from_4090(request: dict[str, Any], response: dict[str, Any]) -> dict[str, Any]:
    """Assign trunk Z+ through the recognized point and intersect the calibrated plane."""
    kind = sku_type(request.get('sku_typ', 'bottle'))
    if kind not in ('bottle', 'box', 'tube'):
        raise ValueError('当前仅支持 bottle、box 和 tube 抓取几何')
    # Consume the returned coordinates; upstream quality flags are diagnostic only.
    if (request.get("base_frame") != "chassis_link" or request.get("T_unit") != "m"
            or response.get("output_frame") != request.get("camera_frame")
            or response.get("output_unit") != "mm"):
        raise ValueError("位姿接口返回坐标系或单位与本次拍照请求不一致")
    transform = _transform(request.get("T_chassis_camera"))
    trunk_ref = _transform(request.get("T_chassis_trunk_ref_m"))
    height = request.get("grasp_height_trunk_mm")
    if isinstance(height, bool) or not isinstance(height, (int, float)) or not math.isfinite(height):
        raise ValueError("缺少有效的商品固定抓取高度（躯干 SDK Z，mm）")
    point_key = {'bottle': 'reference_point', 'box': 'top_point', 'tube': 'top_edge_center'}[kind]
    camera_key = point_key + '_camera_mm'
    chassis_key = point_key + '_chassis_mm'
    reference_camera = _vector(response.get(camera_key), '相机识别点')
    axis_world = _point(transform, reference_camera)
    reference_error = None
    if response.get(chassis_key) is not None:
        try:
            reference_world = _vector(response[chassis_key], '底盘系识别点')
            reference_error = float(np.linalg.norm(axis_world - reference_world))
        except (TypeError, ValueError):
            pass  # Optional duplicate coordinates never veto the camera point.
    axis_trunk = trunk_ref[:3, :3].T @ (axis_world - trunk_ref[:3, 3])
    direction_trunk = np.array([0.0, 0.0, 1.0])
    direction_world = trunk_ref[:3, 2]
    distance = float(height - axis_trunk[2])
    axis_intersection_trunk = axis_trunk + distance * direction_trunk
    grasp_offset_trunk = np.array([0.0, GRASP_Y_OFFSET_TRUNK_MM if kind == 'bottle' else 0.0, 0.0])
    grasp_trunk = axis_intersection_trunk + grasp_offset_trunk
    grasp_world = _point(trunk_ref, grasp_trunk)
    if not np.isfinite(grasp_world).all():
        raise ValueError("固定高度与目标轴线交点无效")
    world_rpy = rotation_to_rpy_deg(
        trunk_ref[:3, :3] @ rpy_rotation(np.radians(ASSIGNED_GRASP_RPY_TRUNK_DEG)))
    return {
        "frame": "chassis_link",
        "sku_typ": kind,
        "direction_source": "assigned_trunk_z_plus",
        "recognition_point_trunk_mm": axis_trunk.tolist(),
        "recognition_height_trunk_mm": float(axis_trunk[2]),
        "axis_point_world_mm": axis_world.tolist(),
        "axis_direction_world_up": direction_world.tolist(),
        "axis_point_trunk_mm": axis_trunk.tolist(),
        "axis_direction_trunk": direction_trunk.tolist(),
        "axis_intersection_trunk_mm": axis_intersection_trunk.tolist(),
        "grasp_offset_trunk_mm": grasp_offset_trunk.tolist(),
        "grasp_offset_frame": TRUNK_FRAME,
        "grasp_point_trunk_mm": grasp_trunk.tolist(),
        "grasp_pose_world_mm_deg": grasp_world.tolist() + world_rpy,
        "grasp_point_rule": "fixed_trunk_height_vertical_reference_v2",
        "grasp_height_frame": TRUNK_FRAME,
        "grasp_height_trunk_mm": float(height),
        "grasp_height_sku_typ": kind,
        "tube_height_calibration": request.get("tube_height_calibration") if kind == "tube" else None,
        "box_height_calibration": request.get("box_height_calibration") if kind == "box" else None,
        "axis_intersection_parameter_mm": distance,
        "reference_consistency_error_mm": reference_error,
        "capture_time": request.get("camera_frame_captured_at"),
        "capture_upper_body_joints_deg": request.get("upper_body_joints_deg"),
    }


def _world_grasp_to_shoulder(
    world_grasp: dict[str, Any], world_from_shoulder_m: np.ndarray,
    box_clearance: dict[str, Any],
    world_from_trunk_ref_m: np.ndarray,
    arm: str,
) -> dict[str, Any]:
    """Convert the object point to shoulder-frame flange grasp/pregrasp poses."""
    gripper_length = LEFT_GRIPPER_LENGTH_MM if arm == 'left' else GRIPPER_LENGTH_MM
    if world_grasp.get("frame") != "chassis_link":
        raise ValueError("抓取点不是 chassis_link 世界坐标")
    pose = np.asarray(world_grasp.get("grasp_pose_world_mm_deg"), dtype=np.float64)
    if pose.shape != (6,) or not np.isfinite(pose).all():
        raise ValueError("保存的世界抓取位姿无效")
    shoulder = np.asarray(world_from_shoulder_m, dtype=np.float64)
    if shoulder.shape != (4, 4) or not np.isfinite(shoulder).all():
        raise ValueError("当前肩部变换无效")
    rotation = shoulder[:3, :3]
    if (not np.allclose(shoulder[3], [0.0, 0.0, 0.0, 1.0], atol=1e-8)
            or not np.allclose(rotation.T @ rotation, np.eye(3), atol=2e-3)):
        raise ValueError("当前肩部刚体变换无效")
    shoulder_origin_world_mm = shoulder[:3, 3] * 1000.0
    object_grasp_shoulder_mm = rotation.T @ (pose[:3] - shoulder_origin_world_mm)
    trunk = _transform(world_from_trunk_ref_m)
    shoulder_from_trunk = rotation.T @ trunk[:3, :3]
    flange_grasp_shoulder_mm = object_grasp_shoulder_mm - shoulder_from_trunk[:, 0] * gripper_length
    if box_clearance.get("valid") is not True or box_clearance.get("frame") != TRUNK_FRAME:
        raise ValueError("预抓取点需要本次定位的有效箱体距离")
    length = float(box_clearance["pregrasp_virtual_length_mm"])
    if not math.isfinite(length) or length < gripper_length:
        raise ValueError("预抓取虚拟夹爪长度无效")
    flange_pregrasp_shoulder_mm = object_grasp_shoulder_mm - shoulder_from_trunk[:, 0] * length
    # Assign the attitude in the PCB4 reference, then express it in the arm SDK
    # world. Rotating the chest must not rotate the requested grasp in this frame.
    grasp_rpy_shoulder_deg = rotation_to_rpy_deg(
        shoulder_from_trunk @ rpy_rotation(np.radians(ASSIGNED_GRASP_RPY_TRUNK_DEG)))
    result = {
        "frame": f"{arm}_arm_sdk_world",
        "assigned_orientation_frame": TRUNK_FRAME,
        f"R_{arm}_shoulder_from_trunk_ref": shoulder_from_trunk.tolist(),
        "T_chassis_trunk_ref_m": np.asarray(world_from_trunk_ref_m).tolist(),
        f"object_grasp_pose_{arm}_shoulder_mm_deg": (
            object_grasp_shoulder_mm.tolist() + grasp_rpy_shoulder_deg
        ),
        f"grasp_pose_{arm}_shoulder_mm_deg": (
            flange_grasp_shoulder_mm.tolist() + grasp_rpy_shoulder_deg
        ),
        f"pregrasp_pose_{arm}_shoulder_mm_deg": (
            flange_pregrasp_shoulder_mm.tolist() + grasp_rpy_shoulder_deg
        ),
        "gripper_length_mm": gripper_length,
        "pregrasp_retraction_mm": length - gripper_length,
        "pregrasp_virtual_length_mm": length,
        "box_clearance": box_clearance,
        f"{arm}_shoulder_origin_world_mm": shoulder_origin_world_mm.tolist(),
        f"{arm}_shoulder_rpy_world_deg": rotation_to_rpy_deg(rotation),
    }
    if arm == 'left':
        calibration = world_grasp.get('box_height_calibration') or {}
        heights = calibration.get('heights_mm', {})
        for key in ('grasp', 'descend', 'lift'):
            value = heights.get(key)
            if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value):
                raise ValueError('缺少有效的盒子固定高度标定：' + key)
        if (calibration.get('frame') != TRUNK_FRAME or calibration.get('unit') != 'mm'
                or not heights['descend'] < heights['grasp'] < heights['lift']):
            raise ValueError('盒子固定高度标定顺序或坐标系不正确')
        flange_world = rotation @ flange_grasp_shoulder_mm + shoulder_origin_world_mm
        flange_trunk = trunk[:3, :3].T @ (flange_world - trunk[:3, 3])
        if abs(flange_trunk[2] - heights['grasp']) > 1e-5:
            raise ValueError('盒子抓取目标与独立标定高度不一致')
        for key in ('descend', 'lift'):
            position = flange_grasp_shoulder_mm + shoulder_from_trunk[:, 2] * (heights[key] - flange_trunk[2])
            result[key + '_height_trunk_mm'] = heights[key]
            result[key + '_pose_left_shoulder_mm_deg'] = position.tolist() + grasp_rpy_shoulder_deg
        result['box_height_calibration'] = calibration
    return result


def world_grasp_to_right_shoulder(world_grasp, shoulder, clearance, trunk):
    return _world_grasp_to_shoulder(world_grasp, shoulder, clearance, trunk, 'right')


def world_grasp_to_left_shoulder(world_grasp, shoulder, clearance, trunk):
    return _world_grasp_to_shoulder(world_grasp, shoulder, clearance, trunk, 'left')
