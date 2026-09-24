#!/usr/bin/env python3
"""Canonical geometric transformation utilities between camera and chassis frames.

Maintains strict internal units in millimetres (mm) for 3D coordinates,
and radians for angular parameters. Extrinsics matrix T_m has translation in metres (m).
"""
from typing import Any, Dict, Optional, Tuple, Union
import numpy as np


def valid_depth_mask(depth: np.ndarray) -> np.ndarray:
    """Returns boolean mask where depth values are finite and strictly positive."""
    z = np.asarray(depth, dtype=float)
    return np.isfinite(z) & (z > 0.0)


def project_camera_points(pts: np.ndarray, K: Dict[str, float]) -> np.ndarray:
    """Projects 3D camera coordinates (mm) to 2D image pixel coordinates (u, v)."""
    pts = np.asarray(pts, dtype=float)
    if pts.ndim == 1:
        pts = pts.reshape(1, 3)
    z = np.clip(pts[:, 2], 1e-6, None)
    u = pts[:, 0] * K['fx'] / z + K['cx']
    v = pts[:, 1] * K['fy'] / z + K['cy']
    return np.column_stack([u, v])


def backproject_pixels(
    pixels_xy: np.ndarray,
    depths_mm: np.ndarray,
    K: Dict[str, float]
) -> np.ndarray:
    """Backprojects 2D pixel coordinates and corresponding depths (mm) to 3D camera points (mm)."""
    pixels = np.asarray(pixels_xy, dtype=float)
    z = np.asarray(depths_mm, dtype=float)
    x = (pixels[:, 0] - K['cx']) * z / K['fx']
    y = (pixels[:, 1] - K['cy']) * z / K['fy']
    return np.column_stack([x, y, z])


def backproject_depth(
    depth: np.ndarray,
    K: Dict[str, float],
    mask: Optional[np.ndarray] = None
) -> np.ndarray:
    """Backprojects a full 2D depth map (mm) to an (N, 3) camera point cloud (mm)."""
    h, w = depth.shape
    yy, xx = np.indices((h, w), dtype=float)
    z = np.asarray(depth, dtype=float)
    valid = valid_depth_mask(z)
    if mask is not None:
        valid = valid & np.asarray(mask, dtype=bool)

    zv = z[valid]
    xv = (xx[valid] - K['cx']) * zv / K['fx']
    yv = (yy[valid] - K['cy']) * zv / K['fy']
    return np.column_stack([xv, yv, zv])


def camera_to_chassis(
    pts_cam_mm: np.ndarray,
    T_m: np.ndarray,
    direction_cam: Optional[np.ndarray] = None
) -> Union[np.ndarray, Tuple[np.ndarray, np.ndarray]]:
    """Transforms 3D points (mm) and optional unit direction vectors from camera frame to chassis_link.

    T_m translation is normalized to metres; converted here to mm by multiplying by 1000.0.
    """
    pts = np.asarray(pts_cam_mm, dtype=float)
    R = T_m[:3, :3]
    t_mm = T_m[:3, 3] * 1000.0

    single_pt = pts.ndim == 1
    if single_pt:
        pts = pts.reshape(1, 3)

    pts_ch = pts @ R.T + t_mm
    if single_pt:
        pts_ch = pts_ch[0]

    if direction_cam is not None:
        v = np.asarray(direction_cam, dtype=float)
        single_v = v.ndim == 1
        if single_v:
            v = v.reshape(1, 3)
        v_ch = v @ R.T
        if single_v:
            v_ch = v_ch[0]
        return pts_ch, v_ch

    return pts_ch


def chassis_to_camera(
    pts_ch_mm: np.ndarray,
    T_m: np.ndarray,
    direction_ch: Optional[np.ndarray] = None
) -> Union[np.ndarray, Tuple[np.ndarray, np.ndarray]]:
    """Transforms 3D points (mm) and optional unit direction vectors from chassis_link to camera frame."""
    pts = np.asarray(pts_ch_mm, dtype=float)
    R = T_m[:3, :3]
    t_mm = T_m[:3, 3] * 1000.0

    single_pt = pts.ndim == 1
    if single_pt:
        pts = pts.reshape(1, 3)

    pts_cam = (pts - t_mm) @ R
    if single_pt:
        pts_cam = pts_cam[0]

    if direction_ch is not None:
        v = np.asarray(direction_ch, dtype=float)
        single_v = v.ndim == 1
        if single_v:
            v = v.reshape(1, 3)
        v_cam = v @ R
        if single_v:
            v_cam = v_cam[0]
        return pts_cam, v_cam

    return pts_cam
