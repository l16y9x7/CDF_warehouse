#!/usr/bin/env python3
"""Tube upper-edge localization from a chassis-frame height band.

The full valid-depth mask cloud is transformed to ``chassis_link`` before a
high, spatially continuous band is selected and robustly fitted.  Public
``top_edge_*`` names are retained for robot API compatibility.  All geometry
is millimetres and this diagnostic never commands a robot.
"""
import argparse
import json
import math
import tempfile
from datetime import datetime
from pathlib import Path

import cv2
import numpy as np
from pycocotools import mask as mask_utils


PROJECT = Path(__file__).resolve().parents[2]
DEFAULT_RESULT_JSON = PROJECT / r"sam3\test\results\prompt_evaluation\20260916_145332\all_individual_gray_green_package_ends\132223928.json"
DEFAULT_RGB_PATH = PROJECT / r"data\20260915\132223928\head_rgb.jpg"
DEFAULT_DEPTH_PATH = PROJECT / r"data\20260915\132223928\head_depth_aligned.npy"
DEFAULT_CAMERA_PATH = PROJECT / r"data\20260915\132223928\camera.json"
DEFAULT_HANDEYE_PATH = PROJECT / r"sam3\test\axis_fit_support\head_camera_transform_20260915\calibration\head_camera_handeye_20260914.json"
DEFAULT_EXTRINSICS_PATH = PROJECT / r"sam3\test\axis_fit_support\head_camera_transform_20260915\transforms_20260915.json"
DEFAULT_METADATA_PATH = PROJECT / r"data\20260915\132223928\head_camera_metadata.json"
DEFAULT_SAMPLE_ID = "132223928"
DEFAULT_CLASS_NAME = "tube"
DEFAULT_INSTANCE_ID = None                  # None => highest score; ties use lower 1-based ID
DEFAULT_OUTPUT_ROOT = PROJECT / r"sam3\test\tube_top_edge_results"
DEFAULT_MASK_EROSION_PIXELS = 1
DEFAULT_MASK_CROP_ORIGIN = [0, 0]            # saved evaluation masks are 640x720 left-bin crop space
DEFAULT_MIN_EDGE_POINTS = 15
DEFAULT_EDGE_TRIM_MM = 4.0                   # robust residual cut during line refinement
DEFAULT_MIN_EDGE_LENGTH_MM = 30.0
DEFAULT_MAX_EDGE_LENGTH_MM = 250.0
DEFAULT_EDGE_MAX_TILT_DEG = 10.0
DEFAULT_MAX_RESIDUAL_MEDIAN_MM = 3.0
DEFAULT_MAX_RESIDUAL_P90_MM = 8.0
DEFAULT_MAX_POINT_STABILITY_MM = 10.0
DEFAULT_SHOW_3D = False
# Height-band geometry validated against one front-view and three side-view
# robot captures on 2026-09-22.
DEFAULT_HEIGHT_AXIS_CHASSIS = [0.0, 0.0, 1.0]
DEFAULT_HEIGHT_ORIGIN_CHASSIS = [0.0, 0.0, 0.0]
DEFAULT_HEIGHT_BAND_DOWN_MM = 24.0
DEFAULT_HEIGHT_BAND_UP_MM = 4.0
DEFAULT_HEIGHT_TOP_QUANTILE = 0.97
DEFAULT_MIN_MASK_VALID_DEPTH_RATIO = 0.08
DEFAULT_MIN_HEIGHT_BAND_POINTS = 15
DEFAULT_MIN_HEIGHT_BAND_SPAN_MM = 10.0
DEFAULT_MAX_HEIGHT_BAND_GAP_MM = 28.0
DEFAULT_MIN_HEIGHT_BAND_COVERAGE = 0.55
DEPTH_SCALE = 1.0                            # verified aligned NPY already stores millimetres
PINHOLE_LIMITATION = "raw aligned RGB-D is backprojected/projected with K only; calibrated distortion is ignored and no repeated undistortion is applied"


def unit(v):
    v = np.asarray(v, float)
    n = np.linalg.norm(v)
    if not np.isfinite(n) or n < 1e-12:
        raise ValueError("zero/invalid vector")
    return v / n


def decode_rle(detection):
    seg = detection.get("segmentation", detection)
    rle = {"size": seg["size"], "counts": seg["counts"]}
    if isinstance(rle["counts"], str):
        rle["counts"] = rle["counts"].encode("ascii")
    return mask_utils.decode(rle).astype(bool)


def erode_mask(mask, pixels):
    if pixels <= 0:
        return mask.copy()
    k = 2 * pixels + 1
    return cv2.erode(mask.astype(np.uint8), np.ones((k, k), np.uint8), iterations=1).astype(bool)


def load_camera(path, handeye_path, metadata_path, rgb_shape):
    """Read K from camera.json; verified captures without one fall back to the handeye file."""
    path = Path(path)
    if path.exists():
        d = json.loads(path.read_text(encoding="utf-8"))
        a = np.asarray(d.get("cam_K") or d.get("camera_matrix"), float).reshape(3, 3)
        source = str(path)
    else:
        h = json.loads(Path(handeye_path).read_text(encoding="utf-8"))
        m = json.loads(Path(metadata_path).read_text(encoding="utf-8"))
        hc, mc = h.get("camera", {}), m.get("camera", {})
        if hc.get("serial_number") != mc.get("serial_number") or hc.get("model") != mc.get("model"):
            raise ValueError("capture and calibration camera identity differ")
        intr = h.get("intrinsics", {})
        size = intr.get("image_size")
        if size != [rgb_shape[1], rgb_shape[0]]:
            raise ValueError("calibration image size differs from RGB")
        a = np.asarray(intr.get("camera_matrix"), float).reshape(3, 3)
        source = str(handeye_path) + " (fallback; capture has no camera.json)"
    if a[0, 0] <= 0 or a[1, 1] <= 0 or not np.allclose(a[2], [0, 0, 1]):
        raise ValueError("invalid pinhole K")
    return {"fx": float(a[0, 0]), "fy": float(a[1, 1]), "cx": float(a[0, 2]), "cy": float(a[1, 2])}, source


def flatten_joints(state):
    j = state["joints_deg"]
    return np.asarray(list(j["trunk"]) + list(j["head"]), float)


def load_extrinsic(path, sample_id, state_path):
    d = json.loads(Path(path).read_text(encoding="utf-8"))
    rows = [r for r in d.get("rows", []) if str(r.get("sample")) == str(sample_id)]
    if len(rows) != 1:
        raise ValueError("sample {} must have exactly one extrinsic row; got {}".format(sample_id, len(rows)))
    r = rows[0]
    if r.get("camera_frame") != "head_camera_color_optical_frame" or r.get("base_frame") != "chassis_link":
        raise ValueError("unexpected extrinsic frames")
    T = np.asarray(r["matrix_4x4"], float)
    if T.shape != (4, 4) or not np.allclose(T[3], [0, 0, 0, 1], atol=1e-8):
        raise ValueError("invalid homogeneous transform")
    R = T[:3, :3]
    if not np.allclose(R.T @ R, np.eye(3), atol=1e-5) or not np.isclose(np.linalg.det(R), 1.0, atol=1e-5):
        raise ValueError("invalid rotation")
    saved = np.asarray(r.get("joint_values_deg", []), float)
    live = flatten_joints(json.loads(Path(state_path).read_text(encoding="utf-8")))
    if saved.shape != (6,) or not np.allclose(saved, live, atol=1e-6):
        raise ValueError("extrinsic joint values do not match capture robot_state.json")
    return T, r


def backproject_pixels(pixels_xy, depth_mm, K):
    z = depth_mm[pixels_xy[:, 1], pixels_xy[:, 0]].astype(float)
    p = np.column_stack(((pixels_xy[:, 0] - K["cx"]) * z / K["fx"],
                         (pixels_xy[:, 1] - K["cy"]) * z / K["fy"], z))
    return p


def camera_to_chassis(points_camera, T):
    p = np.asarray(points_camera, float)
    return (T[:3, :3] @ p.T).T + T[:3, 3] * 1000.0


def chassis_to_camera(points_chassis, T):
    p = np.asarray(points_chassis, float)
    return (T[:3, :3].T @ (p - T[:3, 3] * 1000.0).T).T


def project(points_camera, K):
    p = np.asarray(points_camera, float)
    return np.column_stack((K["fx"] * p[:, 0] / p[:, 2] + K["cx"], K["fy"] * p[:, 1] / p[:, 2] + K["cy"]))


def fit_top_edge(points_chassis, cfg):
    """Robust horizontal line fit through preselected chassis-frame points."""
    p = np.asarray(points_chassis, float)
    out = {"n_points": int(len(p))}
    if len(p) < cfg["min_edge_points"]:
        out.update({"valid": False, "reasons": ["insufficient top-edge points"]})
        return out
    keep = np.ones(len(p), bool)
    c2 = d2 = None
    z0 = None
    res_all = None
    for _ in range(3):
        q = p[keep]
        xy = q[:, :2]
        c2 = np.median(xy, axis=0)
        _, _, vt = np.linalg.svd(xy - c2, full_matrices=False)
        d2 = unit(vt[0])
        z0 = float(np.median(q[:, 2]))
        t = (p[:, :2] - c2) @ d2
        perp = np.linalg.norm(p[:, :2] - c2 - t[:, None] * d2, axis=1)
        res_all = np.sqrt(perp ** 2 + (p[:, 2] - z0) ** 2)
        new = res_all <= cfg["trim_mm"]
        if new.sum() < cfg["min_edge_points"]:
            break
        if np.array_equal(new, keep):
            keep = new
            break
        keep = new
    q = p[keep]
    if len(q) < cfg["min_edge_points"]:
        out.update({"valid": False, "reasons": ["insufficient inlier top-edge points"], "keep": keep})
        return out
    # Free 3-D PCA tilt of the kept points against the horizontal plane (diagnostic).
    _, _, vt3 = np.linalg.svd(q - np.median(q, axis=0), full_matrices=False)
    free_tilt = math.degrees(math.asin(min(1.0, abs(unit(vt3[0])[2]))))
    t_kept = (q[:, :2] - c2) @ d2
    lo, hi = np.percentile(t_kept, [2, 98])
    t_mid = float((lo + hi) / 2.0)
    center = np.array([c2[0] + t_mid * d2[0], c2[1] + t_mid * d2[1], z0])
    res = res_all[keep]
    endpoints = np.array([[c2[0] + lo * d2[0], c2[1] + lo * d2[1], z0],
                         [c2[0] + hi * d2[0], c2[1] + hi * d2[1], z0]])
    length = float(hi - lo)
    med = float(np.median(res))
    p90 = float(np.percentile(res, 90))
    reasons = []
    if length < cfg["min_len"]:
        reasons.append("visible edge segment shorter than plausible tube width/length")
    if length > cfg["max_len"]:
        reasons.append("visible edge segment longer than plausible tube width/length")
    if med > cfg["max_med"] or p90 > cfg["max_p90"]:
        reasons.append("top-edge residual gate failed")
    if free_tilt > cfg["max_tilt"]:
        reasons.append("free edge direction exceeds tilt allowance from horizontal")
    out.update({"valid": not reasons, "reasons": reasons, "keep": keep,
                "anchor_xy_chassis_mm": c2, "direction_chassis": np.array([d2[0], d2[1], 0.0]),
                "height_chassis_mm": z0, "center_chassis_mm": center,
                "endpoints_chassis_mm": endpoints, "segment_length_mm": length,
                "residual_median_mm": med, "residual_p90_mm": p90,
                "free_tilt_deg": float(free_tilt), "n_kept": int(keep.sum())})
    return out


def make_cfg(args):
    """Build one geometry config from argparse Namespace or an HTTP request dict."""
    def value(name, default):
        return args.get(name, default) if isinstance(args, dict) else getattr(args, name, default)
    return {"min_edge_points": int(value("min_edge_points", DEFAULT_MIN_EDGE_POINTS)),
            "trim_mm": float(value("edge_trim_mm", DEFAULT_EDGE_TRIM_MM)),
            "min_len": float(value("min_edge_length_mm", DEFAULT_MIN_EDGE_LENGTH_MM)),
            "max_len": float(value("max_edge_length_mm", DEFAULT_MAX_EDGE_LENGTH_MM)),
            "max_tilt": float(value("edge_max_tilt_deg", DEFAULT_EDGE_MAX_TILT_DEG)),
            "max_med": float(value("max_residual_median_mm", DEFAULT_MAX_RESIDUAL_MEDIAN_MM)),
            "max_p90": float(value("max_residual_p90_mm", DEFAULT_MAX_RESIDUAL_P90_MM))}


def make_height_band_cfg(args):
    """Build the height-band configuration from a request or Namespace."""
    def value(name, default):
        return args.get(name, default) if isinstance(args, dict) else getattr(args, name, default)
    axis = np.asarray(value("height_axis_chassis", DEFAULT_HEIGHT_AXIS_CHASSIS), float)
    if axis.shape != (3,) or not np.all(np.isfinite(axis)) or np.linalg.norm(axis) < 1e-9:
        raise ValueError("height_axis_chassis must be a finite non-zero 3-vector")
    axis = axis / np.linalg.norm(axis)
    if not np.allclose(axis, [0.0, 0.0, 1.0], atol=1e-4):
        raise ValueError("height-band candidate currently requires chassis +Z as the height axis")
    origin = np.asarray(value("height_origin_chassis", DEFAULT_HEIGHT_ORIGIN_CHASSIS), float)
    if origin.shape != (3,) or not np.all(np.isfinite(origin)):
        raise ValueError("height_origin_chassis must be a finite 3-vector")
    band_down = float(value("height_band_down_mm", DEFAULT_HEIGHT_BAND_DOWN_MM))
    band_up = float(value("height_band_up_mm", DEFAULT_HEIGHT_BAND_UP_MM))
    quantile = float(value("height_top_quantile", DEFAULT_HEIGHT_TOP_QUANTILE))
    if band_down <= 0 or band_up < 0 or not 0.5 < quantile < 1.0:
        raise ValueError("invalid height-band limits or top quantile")
    return {
        "height_axis": axis, "height_origin": origin,
        "height_band_down_mm": band_down, "height_band_up_mm": band_up,
        "height_top_quantile": quantile,
        "min_mask_valid_depth_ratio": float(value("min_mask_valid_depth_ratio", DEFAULT_MIN_MASK_VALID_DEPTH_RATIO)),
        "min_height_band_points": int(value("min_height_band_points", DEFAULT_MIN_HEIGHT_BAND_POINTS)),
        "min_height_band_span_mm": float(value("min_height_band_span_mm", DEFAULT_MIN_HEIGHT_BAND_SPAN_MM)),
        "max_height_band_gap_mm": float(value("max_height_band_gap_mm", DEFAULT_MAX_HEIGHT_BAND_GAP_MM)),
        "min_height_band_coverage": float(value("min_height_band_coverage", DEFAULT_MIN_HEIGHT_BAND_COVERAGE)),
        "height_band_min_line_length_mm": float(value("height_band_min_line_length_mm", DEFAULT_MIN_HEIGHT_BAND_SPAN_MM)),
    }


def analyse_height_band(mask, depth, K, T, erosion, cfg, geometry_cache=None):
    """Fit a visible upper edge from the full chassis-frame mask point cloud."""
    inner = erode_mask(mask, erosion)
    mask_pixels = int(inner.sum())
    if geometry_cache is not None:
        camera_points, pixels, valid_map, chassis_points = geometry_cache.get_mask(inner, depth, K, T)
    else:
        valid_map = inner & np.isfinite(depth) & (depth > 0)
        yy, xx = np.where(valid_map)
        pixels = np.column_stack((xx, yy))
        camera_points = backproject_pixels(pixels, depth, K) if len(pixels) else np.empty((0, 3), float)
        chassis_points = camera_to_chassis(camera_points, T) if len(pixels) else np.empty((0, 3), float)
    out = {
        "erosion_pixels": int(erosion), "mask_area_pixels": int(mask.sum()),
        "eroded_mask_area_pixels": mask_pixels, "valid_depth_points": int(len(chassis_points)),
        "mask_valid_depth_ratio": float(len(chassis_points) / max(1, mask_pixels)),
        "height_axis_chassis": cfg["height_axis"], "height_origin_chassis": cfg["height_origin"],
        "camera_points": camera_points, "pixels_xy": pixels,
    }
    if len(chassis_points) < cfg["min_height_band_points"]:
        out.update({"valid": False, "reason": "insufficient valid mask point cloud", "rejection_reasons": ["insufficient valid mask point cloud"]})
        return out
    if out["mask_valid_depth_ratio"] < cfg["min_mask_valid_depth_ratio"]:
        out.update({"valid": False, "reason": "mask valid-depth ratio below threshold", "rejection_reasons": ["mask valid-depth ratio below threshold"]})
        return out
    heights = (chassis_points - cfg["height_origin"]) @ cfg["height_axis"]
    top_height = float(np.percentile(heights, cfg["height_top_quantile"] * 100.0))
    band = (heights >= top_height - cfg["height_band_down_mm"]) & (heights <= top_height + cfg["height_band_up_mm"])
    candidate = chassis_points[band]
    candidate_pixels = pixels[band]
    candidate_camera_points = camera_points[band]
    out.update({"height_top_mm": top_height, "height_band_min_mm": top_height - cfg["height_band_down_mm"],
                "height_band_max_mm": top_height + cfg["height_band_up_mm"],
                "height_band_point_count": int(len(candidate)), "height_band_pixels_xy": candidate_pixels,
                "camera_points": candidate_camera_points, "edge_pixels_xy": candidate_pixels})
    if len(candidate) < cfg["min_height_band_points"]:
        out.update({"valid": False, "reason": "insufficient points in height band", "rejection_reasons": ["insufficient points in height band"]})
        return out
    # Measure continuity along the dominant horizontal direction.  A dense
    # side face can have many points but should not pass as a single broken line.
    horizontal = candidate - np.outer((candidate - cfg["height_origin"]) @ cfg["height_axis"], cfg["height_axis"])
    center = np.median(horizontal, axis=0)
    _, _, vt = np.linalg.svd(horizontal - center, full_matrices=False)
    direction = unit(vt[0])
    direction = direction - np.dot(direction, cfg["height_axis"]) * cfg["height_axis"]
    if np.linalg.norm(direction) < 1e-9:
        out.update({"valid": False, "reason": "height-band horizontal direction is degenerate", "rejection_reasons": ["height-band horizontal direction is degenerate"]})
        return out
    direction = unit(direction)
    projection = (candidate - center) @ direction
    lo, hi = np.percentile(projection, [2, 98])
    span = float(hi - lo)
    sorted_projection = np.sort(projection)
    gaps = np.diff(sorted_projection)
    max_gap = float(np.max(gaps)) if len(gaps) else float("inf")
    coverage = float((hi - lo) / max(1e-6, sorted_projection[-1] - sorted_projection[0]))
    out.update({"height_band_span_mm": span, "height_band_max_gap_mm": max_gap,
                "height_band_projection_coverage": coverage})
    reasons = []
    if span < cfg["min_height_band_span_mm"]:
        reasons.append("height-band span below threshold")
    if max_gap > cfg["max_height_band_gap_mm"]:
        reasons.append("height-band continuity gap above threshold")
    if coverage < cfg["min_height_band_coverage"]:
        reasons.append("height-band projection coverage below threshold")
    edge_cfg = make_cfg({})
    edge_cfg["min_len"] = cfg["height_band_min_line_length_mm"]
    fit = fit_top_edge(candidate, edge_cfg)
    out["fit"] = fit
    if not fit.get("valid"):
        reasons.extend(fit.get("reasons", ["height-band line fit failed"]))
    reasons = list(dict.fromkeys(reasons))
    out.update({"valid": not reasons, "rejection_reasons": reasons,
                "reason": "; ".join(reasons) if reasons else None,
                "height_band_candidate_points_chassis": candidate})
    if not reasons:
        out.update({"reference_camera_mm": chassis_to_camera(np.asarray(fit["center_chassis_mm"])[None], T)[0],
                    "reference_chassis_mm": fit["center_chassis_mm"],
                    "direction_camera": T[:3, :3].T @ fit["direction_chassis"],
                    "endpoints_camera_mm": chassis_to_camera(np.asarray(fit["endpoints_chassis_mm"]), T),
                    "point_semantics": "visible_top_edge_midpoint"})
    return out


def write_ply(path, camera_points, keep):
    colors = np.tile(np.array([120, 120, 120], int), (len(camera_points), 1))
    colors[keep] = [40, 210, 60]
    with Path(path).open("w", encoding="ascii") as f:
        f.write("ply\nformat ascii 1.0\nelement vertex {}\nproperty float x\nproperty float y\nproperty float z\nproperty uchar red\nproperty uchar green\nproperty uchar blue\nend_header\n".format(len(camera_points)))
        for p, c in zip(camera_points, colors):
            f.write("{:.6f} {:.6f} {:.6f} {} {} {}\n".format(*p, *c))


def set_equal(ax, points):
    p = np.asarray(points); lo = p.min(axis=0); hi = p.max(axis=0); center = (lo + hi) / 2; radius = max((hi - lo).max() / 2, 1.0)
    ax.set_xlim(center[0] - radius, center[0] + radius); ax.set_ylim(center[1] - radius, center[1] + radius); ax.set_zlim(center[2] - radius, center[2] + radius)
    ax.set_box_aspect((1, 1, 1))


def render_3d(result, T, path, show=False):
    import matplotlib.pyplot as plt
    fig = plt.figure(figsize=(11, 8)); ax = fig.add_subplot(111, projection="3d")
    pc = result.get("camera_points")
    if pc is not None and len(pc):
        keep = np.asarray(result.get("fit", {}).get("keep", np.zeros(len(pc), bool)), dtype=bool)
        if keep.shape != (len(pc),):
            keep = np.zeros(len(pc), bool)
        ax.scatter(pc[~keep, 0], pc[~keep, 1], pc[~keep, 2], s=8, c="#999999", alpha=.5, label="trimmed edge points")
        if keep.any():
            ax.scatter(pc[keep, 0], pc[keep, 1], pc[keep, 2], s=8, c="#22bb55", alpha=.9, label="edge inliers")
    fit = result.get("fit", {})
    if result.get("endpoints_camera_mm") is not None:
        e = np.asarray(result["endpoints_camera_mm"])
        ax.plot(e[:, 0], e[:, 1], e[:, 2], c="red", linewidth=2, label="fitted horizontal top edge")
    if result.get("reference_camera_mm") is not None:
        ax.scatter(*result["reference_camera_mm"], s=110, c="#d000ff", marker="*", label="top-edge midpoint")
    ax.set_xlabel("camera X (mm)"); ax.set_ylabel("camera Y (mm)"); ax.set_zlabel("camera Z (mm)")
    ax.legend(loc="best"); ax.set_title("tube top edge — camera optical frame, mm")
    set_equal(ax, pc if pc is not None and len(pc) else np.zeros((3, 3)))
    fig.tight_layout(); fig.savefig(str(path), dpi=180)
    if show:
        plt.show()
    plt.close(fig)


def draw_overlay(rgb, detections, selected_id, mask, result, K, out):
    im = rgb.copy()
    for i, d in enumerate(detections, 1):
        x, y, w, h = map(int, d.get("bbox", [0, 0, 0, 0])); color = (0, 255, 255) if i == selected_id else (210, 210, 210)
        cv2.rectangle(im, (x, y), (x + w, y + h), color, 1)
        cv2.putText(im, "#{} {:.3f}".format(i, float(d.get("score", 0))), (x, max(15, y - 4)), cv2.FONT_HERSHEY_SIMPLEX, .42, color, 1, cv2.LINE_AA)
    im[mask] = (.55 * im[mask] + .45 * np.array([255, 150, 0])).astype(np.uint8)
    pix = result.get("edge_pixels_xy")
    if pix is not None and len(pix):
        for x, y in pix:
            cv2.circle(im, (int(x), int(y)), 1, (0, 220, 40), -1)
    fit = result.get("fit", {})
    if result.get("endpoints_camera_mm") is not None:
        uv = project(np.asarray(result["endpoints_camera_mm"]), K).astype(int)
        cv2.line(im, tuple(uv[0]), tuple(uv[1]), (0, 0, 255), 3)
    if result.get("reference_camera_mm") is not None:
        u = project(np.asarray(result["reference_camera_mm"])[None], K)[0]
        if 0 <= u[0] < rgb.shape[1] and 0 <= u[1] < rgb.shape[0]:
            cv2.drawMarker(im, tuple(u.astype(int)), (255, 0, 255), cv2.MARKER_STAR, 18, 2)
    status = "edge={} point={} {}".format(bool(fit.get("valid")), bool(result.get("valid")), result.get("point_semantics", "no formal point"))
    cv2.putText(im, status, (18, im.shape[0] - 18), cv2.FONT_HERSHEY_SIMPLEX, .55, (0, 0, 0), 3, cv2.LINE_AA)
    cv2.putText(im, status, (18, im.shape[0] - 18), cv2.FONT_HERSHEY_SIMPLEX, .55, (255, 255, 255), 1, cv2.LINE_AA)
    cv2.imwrite(str(out), im)


def serialise_result(x):
    if isinstance(x, np.ndarray):
        return x.tolist()
    if isinstance(x, (np.floating, np.integer)):
        return x.item()
    raise TypeError(type(x).__name__)


def synthetic_test():
    cfg = {"min_edge_points": 15, "trim_mm": 4.0, "min_len": 30.0, "max_len": 250.0,
           "max_tilt": 10.0, "max_med": 3.0, "max_p90": 8.0}
    rng = np.random.default_rng(11)
    # 1. Straight horizontal edge: midpoint, direction, height recovered.
    x = np.linspace(0, 150, 120)
    line = np.c_[x, 20 + rng.normal(0, .3, x.size), 100 + rng.normal(0, .4, x.size)]
    fit = fit_top_edge(line, cfg)
    assert fit["valid"], fit
    assert np.linalg.norm(fit["center_chassis_mm"] - [75, 20, 100]) < 2.0, fit["center_chassis_mm"]
    assert abs(fit["direction_chassis"][0]) > .99 and abs(fit["direction_chassis"][2]) < 1e-9
    assert fit["free_tilt_deg"] < 2.0 and 140 < fit["segment_length_mm"] < 155
    # 2. Cap-shaped outliers at one end are trimmed away; midpoint stays on the body edge.
    bump = np.c_[np.linspace(155, 182, 26), 20 + rng.normal(0, .3, 26), 108 + rng.normal(0, .4, 26)]
    fit2 = fit_top_edge(np.vstack((line, bump)), cfg)
    assert fit2["valid"], fit2
    assert np.linalg.norm(fit2["center_chassis_mm"] - [75, 20, 100]) < 4.0, fit2["center_chassis_mm"]
    assert fit2["segment_length_mm"] < 160
    # 3. A tilted edge stays valid under the allowance and reports its tilt.
    tilted = np.c_[x, 20 + rng.normal(0, .3, x.size), 100 + .0875 * x + rng.normal(0, .3, x.size)]
    fit3 = fit_top_edge(tilted, cfg)
    assert fit3["valid"] and 3.0 < fit3["free_tilt_deg"] < 7.0, fit3.get("free_tilt_deg")
    # 4. Too few points and too-short segments are rejected.
    assert not fit_top_edge(line[:8], cfg)["valid"]
    short = np.c_[np.linspace(0, 10, 40), 20 + rng.normal(0, .2, 40), 100 + rng.normal(0, .3, 40)]
    assert not fit_top_edge(short, cfg)["valid"]
    # 5. End-to-end height-band analysis over the full valid mask point cloud.
    K = {"fx": 600.0, "fy": 600.0, "cx": 320.0, "cy": 240.0}
    h, w = 480, 640
    band = np.zeros((h, w), bool); band[100:140, 200:340] = True   # ~140 mm band at 800 mm depth
    dep = np.full((h, w), 800.0)
    T = np.eye(4)
    height_cfg = make_height_band_cfg({"height_top_quantile": 0.9, "height_band_down_mm": 24.0})
    a = analyse_height_band(band, dep, K, T, 0, height_cfg)
    assert a["valid"], a.get("reason")
    assert abs(a["reference_camera_mm"][2] - 800) < 1
    assert a["point_semantics"] == "visible_top_edge_midpoint"
    # 6. Metre-translation normalisation and camera<->chassis round trip.
    ay = math.radians(20); az = math.radians(-12)
    Ry = np.array([[math.cos(ay), 0, math.sin(ay)], [0, 1, 0], [-math.sin(ay), 0, math.cos(ay)]])
    Rz = np.array([[math.cos(az), -math.sin(az), 0], [math.sin(az), math.cos(az), 0], [0, 0, 1]])
    T2 = np.eye(4); T2[:3, :3] = Rz @ Ry; T2[:3, 3] = [.15, -.05, 1.2]
    p = np.array([[10., 20., 800.]])
    assert np.linalg.norm(chassis_to_camera(camera_to_chassis(p, T2), T2) - p) < 1e-9
    # 7. Diagnostics and renders survive rejected fits without optional fields.
    with tempfile.TemporaryDirectory() as td:
        bad = {"camera_points": p.repeat(6, axis=0), "fit": {"valid": False, "reasons": ["synthetic rejection"]}}
        png = Path(td) / "bad.png"
        render_3d(bad, T2, png, False)
        assert png.exists() and png.stat().st_size > 0
        write_ply(Path(td) / "bad.ply", bad["camera_points"], np.zeros(6, bool))
    print(json.dumps({"synthetic_test": "PASS", "straight_edge_center_mm": True, "cap_bump_trimmed": True,
                      "tilt_reported_deg": fit3["free_tilt_deg"], "few_points_rejected": True,
                      "short_segment_rejected": True, "height_band_end_to_end_ok": True,
                      "transform_roundtrip_ok": True,
                      "invalid_fit_render_saved": True}, indent=2))


def main():
    ap = argparse.ArgumentParser(description="Offline tube chassis height-band edge diagnostic")
    ap.add_argument("--result-json", type=Path, default=DEFAULT_RESULT_JSON)
    ap.add_argument("--rgb-path", type=Path, default=DEFAULT_RGB_PATH)
    ap.add_argument("--depth-path", type=Path, default=DEFAULT_DEPTH_PATH)
    ap.add_argument("--camera-path", type=Path, default=DEFAULT_CAMERA_PATH)
    ap.add_argument("--handeye-path", type=Path, default=DEFAULT_HANDEYE_PATH)
    ap.add_argument("--metadata-path", type=Path, default=DEFAULT_METADATA_PATH)
    ap.add_argument("--extrinsics-path", type=Path, default=DEFAULT_EXTRINSICS_PATH)
    ap.add_argument("--sample-id", default=DEFAULT_SAMPLE_ID)
    ap.add_argument("--instance-id", type=int, default=DEFAULT_INSTANCE_ID)
    ap.add_argument("--mask-crop-origin", nargs=2, type=int, default=DEFAULT_MASK_CROP_ORIGIN, metavar=("X", "Y"))
    ap.add_argument("--output-root", type=Path, default=DEFAULT_OUTPUT_ROOT)
    ap.add_argument("--mask-erosion-pixels", type=int, default=DEFAULT_MASK_EROSION_PIXELS)
    ap.add_argument("--min-edge-points", type=int, default=DEFAULT_MIN_EDGE_POINTS)
    ap.add_argument("--edge-trim-mm", type=float, default=DEFAULT_EDGE_TRIM_MM)
    ap.add_argument("--min-edge-length-mm", type=float, default=DEFAULT_MIN_EDGE_LENGTH_MM)
    ap.add_argument("--max-edge-length-mm", type=float, default=DEFAULT_MAX_EDGE_LENGTH_MM)
    ap.add_argument("--edge-max-tilt-deg", type=float, default=DEFAULT_EDGE_MAX_TILT_DEG)
    ap.add_argument("--max-residual-median-mm", type=float, default=DEFAULT_MAX_RESIDUAL_MEDIAN_MM)
    ap.add_argument("--max-residual-p90-mm", type=float, default=DEFAULT_MAX_RESIDUAL_P90_MM)
    ap.add_argument("--height-band-down-mm", type=float, default=DEFAULT_HEIGHT_BAND_DOWN_MM)
    ap.add_argument("--height-band-up-mm", type=float, default=DEFAULT_HEIGHT_BAND_UP_MM)
    ap.add_argument("--height-top-quantile", type=float, default=DEFAULT_HEIGHT_TOP_QUANTILE)
    ap.add_argument("--show-3d", action="store_true", default=DEFAULT_SHOW_3D)
    ap.add_argument("--synthetic-test", action="store_true")
    args = ap.parse_args()
    if args.synthetic_test:
        synthetic_test()
        return 0
    required = [args.result_json, args.rgb_path, args.depth_path, args.extrinsics_path, args.rgb_path.parent / "robot_state.json"]
    for p in required:
        if not Path(p).exists():
            raise FileNotFoundError(p)
    result_doc = json.loads(args.result_json.read_text(encoding="utf-8"))
    detections = result_doc.get("detections", [])
    if not detections:
        raise ValueError("no detections")
    selected_id = args.instance_id if args.instance_id is not None else min(range(1, len(detections) + 1), key=lambda i: (-float(detections[i - 1].get("score", -1)), i))
    if not 1 <= selected_id <= len(detections):
        raise ValueError("instance-id out of range")
    rgb = cv2.imread(str(args.rgb_path), cv2.IMREAD_COLOR)
    depth_raw = np.load(str(args.depth_path), allow_pickle=False)
    if rgb is None or depth_raw.ndim != 2 or rgb.shape[:2] != depth_raw.shape:
        raise ValueError("RGB/aligned depth size mismatch")
    depth = depth_raw.astype(float) * DEPTH_SCALE
    K, camera_source = load_camera(args.camera_path, args.handeye_path, args.metadata_path, rgb.shape[:2])
    T, row = load_extrinsic(args.extrinsics_path, args.sample_id, args.rgb_path.parent / "robot_state.json")
    ox, oy = args.mask_crop_origin
    masks = []
    for d in detections:
        m = decode_rle(d)
        if m.shape != rgb.shape[:2]:
            if m.shape[0] + oy > rgb.shape[0] or m.shape[1] + ox > rgb.shape[1]:
                raise ValueError("mask does not fit image at crop origin")
            full = np.zeros(rgb.shape[:2], bool)
            full[oy:oy + m.shape[0], ox:ox + m.shape[1]] = m
            m = full
        masks.append(m)
    mask = masks[selected_id - 1]
    if (ox, oy) != (0, 0):
        for d in detections:
            d["bbox"] = [float(d["bbox"][0]) + ox, float(d["bbox"][1]) + oy, float(d["bbox"][2]), float(d["bbox"][3])]
    cfg = make_height_band_cfg(args)
    analyses = {str(e): analyse_height_band(mask, depth, K, T, e, cfg) for e in (0, 1, 2)}
    main_result = analyses[str(args.mask_erosion_pixels)]
    points = [a["reference_camera_mm"] for a in analyses.values() if a.get("valid")]
    point_var = float(max(np.linalg.norm(np.asarray(a) - np.asarray(b)) for a in points for b in points)) if len(points) >= 2 else None
    edge_ok = bool(main_result.get("valid"))
    point_ok = bool(edge_ok and point_var is not None and point_var <= DEFAULT_MAX_POINT_STABILITY_MM
                    and main_result.get("mask_valid_depth_ratio", 0) >= cfg["min_mask_valid_depth_ratio"])
    reasons = [] if edge_ok else [main_result.get("reason", "top edge invalid")]
    if main_result.get("mask_valid_depth_ratio", 0) < cfg["min_mask_valid_depth_ratio"]:
        reasons.append("valid depth ratio below threshold")
    if point_var is None:
        reasons.append("insufficient successful erosion variants for stability check")
    elif point_var > DEFAULT_MAX_POINT_STABILITY_MM:
        reasons.append("top-edge point unstable across 0/1/2 px erosion")
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S_%f")
    root = args.output_root / timestamp / "instance_{:03d}".format(selected_id)
    root.mkdir(parents=True, exist_ok=False)
    cv2.imwrite(str(root / "mask_original.png"), mask.astype(np.uint8) * 255)
    draw_overlay(rgb, detections, selected_id, mask, main_result, K, root / "rgb_top_edge_overlay.png")
    if "camera_points" in main_result:
        write_ply(root / "point_cloud.ply", main_result["camera_points"],
                  main_result.get("fit", {}).get("keep", np.zeros(len(main_result["camera_points"]), bool)))
        render_3d(main_result, T, root / "top_edge_3d.png", args.show_3d)
    fit = main_result.get("fit", {})
    report = {"class_name": DEFAULT_CLASS_NAME, "localization_method": "tube_height_band_edge",
              "inputs": {"result_json": str(args.result_json.resolve()), "rgb": str(args.rgb_path.resolve()),
                         "depth": str(args.depth_path.resolve()), "camera_source": camera_source,
                         "extrinsics": str(args.extrinsics_path.resolve())},
              "source_verification": {"sample_id": args.sample_id, "mask_crop_origin_xy": [ox, oy],
                                       "depth_dtype": str(depth_raw.dtype), "depth_unit": "mm", "depth_scale": DEPTH_SCALE},
              "instance_id": selected_id, "selection": "manual override" if args.instance_id is not None else "highest original SAM3 score; ties by lower 1-based ID",
              "sam3_score": float(detections[selected_id - 1]["score"]), "prompt": result_doc.get("prompt"),
              "output_frame": "head_camera_color_optical_frame", "output_unit": "mm", "reference_frame": "chassis_link",
              "extrinsics": {"sample": row["sample"], "camera_frame": row["camera_frame"], "base_frame": row["base_frame"],
                             "translation_input_unit": "m", "translation_internal_unit": "mm", "joint_match_verified": True},
              "camera_model": {"K": K, "limitation": PINHOLE_LIMITATION},
              "thresholds": {**cfg, "max_point_stability_mm": DEFAULT_MAX_POINT_STABILITY_MM},
              "mask_area_pixels": int(mask.sum()), "edge_point_count": main_result.get("height_band_point_count"),
              "valid_depth_ratio": main_result.get("mask_valid_depth_ratio"), "edge_valid": edge_ok, "point_valid": point_ok,
              "edge_depth_sampling_semantics": "full valid mask cloud in chassis_link, then upper continuous height band; not a reconstructed hidden seal centre",
              "edge_free_tilt_deg": fit.get("free_tilt_deg"), "edge_height_chassis_mm": fit.get("height_chassis_mm"),
              "edge_segment_length_mm": fit.get("segment_length_mm"),
              "edge_residual_median_mm": fit.get("residual_median_mm"), "edge_residual_p90_mm": fit.get("residual_p90_mm"),
              "top_edge_center_camera_mm": main_result.get("reference_camera_mm") if point_ok else None,
              "top_edge_center_chassis_mm": main_result.get("reference_chassis_mm") if point_ok else None,
              "top_edge_direction_camera": main_result.get("direction_camera") if point_ok else None,
              "erosion_point_max_pairwise_mm": point_var, "point_semantics": main_result.get("point_semantics"),
              "rejection_reasons": reasons}
    (root / "top_edge_result.json").write_text(json.dumps(report, ensure_ascii=False, indent=2, default=serialise_result), encoding="utf-8")
    print(json.dumps({"class_name": DEFAULT_CLASS_NAME, "localization_method": "tube_height_band_edge",
                      "output_dir": str(root.resolve()), "instance_id": selected_id,
                      "edge_valid": edge_ok, "point_valid": point_ok,
                      "top_edge_center_camera_mm": report["top_edge_center_camera_mm"],
                      "edge_height_chassis_mm": fit.get("height_chassis_mm"),
                      "edge_segment_length_mm": fit.get("segment_length_mm"),
                      "edge_free_tilt_deg": fit.get("free_tilt_deg"),
                      "erosion_point_max_pairwise_mm": point_var, "rejection_reasons": reasons},
                     ensure_ascii=False, indent=2, default=serialise_result))
    return 0 if point_ok else 3


if __name__ == "__main__":
    raise SystemExit(main())
