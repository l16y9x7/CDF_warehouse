"""Front-panel geometry from the same RGB-D result as the target object.

No controller calls. All points/distances below are in millimetres.
"""
from __future__ import annotations

import numpy as np

from .grasp_pose import _point, _transform, _vector
from .trunk_frame import TRUNK_FRAME


PREGRASP_EXTRA_MM = 250.0


def front_panel_from_response(request, response):
    """Use the capture transform, never the current head pose."""
    if (request.get("base_frame") != "chassis_link" or request.get("T_unit") != "m"
            or response.get("output_frame") != request.get("camera_frame")
            or response.get("output_unit") != "mm"):
        raise ValueError("前挡板坐标系或单位与拍照请求不一致")
    T = _transform(request.get("T_chassis_camera"))
    p = _vector(response.get("front_panel_plane_point_camera_mm"), "前挡板平面点")
    n = _vector(response.get("front_panel_plane_normal_camera"), "前挡板法向量")
    top = _vector(response.get("front_panel_top_edge_midpoint_camera_mm"), "前挡板上沿中点")
    length = float(np.linalg.norm(n))
    if length < 1e-12:
        raise ValueError("前挡板法向量不能为零")
    point = _point(T, p)
    normal = T[:3, :3] @ (n / length)
    midpoint = _point(T, top)
    return {"valid": True, "frame": "chassis_link", "point_mm": point.tolist(),
            "normal": normal.tolist(), "top_edge_midpoint_mm": midpoint.tolist(),
            "request_id": response.get("request_id"),
            "capture_time": request.get("camera_frame_captured_at")}


def clearance_in_trunk(panel, world_grasp, world_from_trunk_ref_m):
    """Intersect the target's SDK reference-X line with the fitted plane."""
    if panel.get("valid") is not True or panel.get("frame") != "chassis_link":
        raise ValueError(panel.get("error") or "缺少有效前挡板平面")
    if world_grasp.get("frame") != "chassis_link":
        raise ValueError("物体与前挡板必须处于同一底盘坐标系")
    T = _transform(world_from_trunk_ref_m)
    R, origin = T[:3, :3], T[:3, 3]
    point = R.T @ (_vector(panel.get("point_mm"), "前挡板平面点") - origin)
    normal = R.T @ _vector(panel.get("normal"), "前挡板法向量")
    obj_world = np.asarray(world_grasp.get("grasp_pose_world_mm_deg"), dtype=float)
    if obj_world.shape != (6,) or not np.isfinite(obj_world).all():
        raise ValueError("物品抓取点无效")
    obj = R.T @ (obj_world[:3] - origin)
    if abs(normal[0]) < 1e-6:
        raise ValueError("前挡板平面与躯干 SDK X 轴平行，无法计算后退距离")
    offset = float(normal @ point)
    D = offset / float(normal[0])
    # Positive d means the target is behind the panel along trunk SDK X+.
    d = float(normal @ (obj - point)) / float(normal[0])
    if D <= 0:
        raise ValueError("前挡板不在躯干 SDK 原点 X+ 方向，不能执行当前抓取流程")
    if d < -1e-6:
        raise ValueError("物品位于前挡板外侧（d < 0），不能执行当前抓取流程")
    d = max(0.0, d)
    intersection = obj.copy()
    intersection[0] -= d
    return {"valid": True, "frame": TRUNK_FRAME, "unit": "mm",
            "rule": "target_x_line_plane_intersection_v1",
            "D_mm": D, "d_mm": d,
            "plane_perpendicular_distance_mm": abs(offset) / float(np.linalg.norm(normal)),
            "object_point_mm": obj.tolist(), "edge_intersection_mm": intersection.tolist(),
            "plane_point_mm": point.tolist(), "plane_normal": normal.tolist(),
            "trunk_ref_origin_world_mm": origin.tolist(),
            "pregrasp_virtual_length_mm": d + (230.0 if world_grasp.get("sku_typ") == "box" else PREGRASP_EXTRA_MM),
            "sku_typ": world_grasp.get("sku_typ", "bottle"),
            "request_id": panel.get("request_id"), "capture_time": panel.get("capture_time")}
