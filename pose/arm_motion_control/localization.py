"""Convert one valid 4090 observation into a fixed chassis_link grasp point.

The 4090 reference_point_chassis_mm is on a diagnostic Z plane.  It is never
used as the grasp height; the visible top is estimated from the saved PLY.
"""

from __future__ import annotations

import base64
import io
from dataclasses import dataclass

import numpy as np

from geometry import GraspGeometry, direction, grasp_geometry, point, rigid


@dataclass(frozen=True)
class Localization:
    geometry: GraspGeometry
    cylinder_band_points: int
    reference_error_mm: float
    capture_time: str


def _ply_xyz_mm(encoded: str) -> np.ndarray:
    try:
        decoded = base64.b64decode(encoded, validate=True)
        header, data = decoded.split(b"end_header\n", 1)
    except (ValueError, TypeError) as exc:
        raise ValueError("4090 PLY missing or malformed") from exc
    if b"format ascii 1.0" not in header:
        raise ValueError("only ASCII PLY is supported")
    vertices = None
    for line in header.splitlines():
        if line.startswith(b"element vertex "):
            vertices = int(line.split()[-1])
    if vertices is None or vertices < 100:
        raise ValueError("4090 PLY has too few vertices")
    cloud = np.loadtxt(io.BytesIO(data), usecols=(0, 1, 2), max_rows=vertices)
    if cloud.shape != (vertices, 3) or not np.isfinite(cloud).all():
        raise ValueError("4090 PLY vertex count or coordinates invalid")
    return cloud


def from_4090(request: dict, response: dict) -> Localization:
    if response.get("ok") is not True or response.get("axis_fit_valid") is not True or response.get("reference_point_valid") is not True:
        raise ValueError("4090 result is not fully valid; refusing to reuse it")
    if request.get("base_frame") != "chassis_link":
        raise ValueError("4090 request base_frame is not chassis_link")
    if request.get("T_unit") != "m":
        raise ValueError("4090 camera transform unit is not m")
    if response.get("output_unit") != "mm":
        raise ValueError("4090 result unit is not mm")
    if response.get("output_frame") != request.get("camera_frame"):
        raise ValueError("4090 result camera frame does not match the request")
    capture_time = request.get("camera_frame_captured_at")
    if not isinstance(capture_time, str) or not capture_time:
        raise ValueError("capture timestamp missing")
    captured_upper_body = np.asarray(request.get("upper_body_joints_deg"), dtype=float)
    if captured_upper_body.shape != (6,) or not np.isfinite(captured_upper_body).all():
        raise ValueError("captured trunk/head joint state missing or invalid")
    transform = rigid(request["T_chassis_camera"], "T_chassis_camera").copy()
    transform[:3, 3] *= 1000
    axis_camera = np.asarray(response["axis_point_camera_mm"], dtype=float)
    up_camera = np.asarray(response["axis_direction_camera_up"], dtype=float)
    if axis_camera.shape != (3,) or up_camera.shape != (3,) or not np.isfinite(axis_camera).all() or not np.isfinite(up_camera).all():
        raise ValueError("invalid 4090 axis")
    norm = float(np.linalg.norm(up_camera))
    if not 0.95 <= norm <= 1.05:
        raise ValueError("4090 up-axis is not unit length")
    up_camera /= norm
    reference_camera = np.asarray(response["reference_point_camera_mm"], dtype=float)
    reference_world = np.asarray(response["reference_point_chassis_mm"], dtype=float)
    if reference_camera.shape != (3,) or reference_world.shape != (3,):
        raise ValueError("invalid 4090 reference point")
    reference_error = float(np.linalg.norm(point(transform, reference_camera) - reference_world))
    if not np.isfinite(reference_error) or reference_error > 5.0:
        raise ValueError(f"4090 camera/world transform disagrees by {reference_error:.1f} mm")
    radius_mm = float(request["body_radius_mm"])
    if not 5 <= radius_mm <= 150:
        raise ValueError("implausible cylinder radius")
    if abs(float(response.get("body_radius_mm", radius_mm)) - radius_mm) > 0.01:
        raise ValueError("4090 result radius does not match the request")
    cloud = _ply_xyz_mm(response["artifacts"]["point_cloud_ply_base64"])
    projection = (cloud - axis_camera) @ up_camera
    radial = np.linalg.norm(cloud - axis_camera - np.outer(projection, up_camera), axis=1)
    band = np.abs(radial - radius_mm) < 6.0
    count = int(np.count_nonzero(band))
    if count < 100:
        raise ValueError(f"only {count} cylinder-band points; visible top unreliable")
    top_projection = float(np.percentile(projection[band], 99.5))
    top_camera = axis_camera + top_projection * up_camera
    top_world = point(transform, top_camera)
    up_world = direction(transform, up_camera)
    return Localization(grasp_geometry(top_world, up_world), count, reference_error, capture_time)
