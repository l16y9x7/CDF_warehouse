#!/usr/bin/env python3
"""Offline box top-surface separation and supported suction reference.

All geometry is millimetres.  The formal output is in
head_camera_color_optical_frame.  This diagnostic does not command a robot.
"""
import argparse
import json
import math
import sys
import tempfile
from datetime import datetime
from pathlib import Path

import cv2
import numpy as np
from pycocotools import mask as mask_utils
from scipy import ndimage
from scipy.optimize import least_squares
from scipy.spatial import ConvexHull


PROJECT = Path(__file__).resolve().parents[2]
DEFAULT_RESULT_JSON = PROJECT / r"sam3\test\results\20260915_174340\head_rgb.json"
DEFAULT_RGB_PATH = PROJECT / r"data\20260915\125311198\head_rgb.jpg"
DEFAULT_DEPTH_PATH = PROJECT / r"data\20260915\125311198\head_depth_aligned.npy"
DEFAULT_CAMERA_PATH = PROJECT / r"data\20260915\125311198\camera.json"
DEFAULT_EXTRINSICS_PATH = PROJECT / r"sam3\test\axis_fit_support\head_camera_transform_20260915\transforms_20260915.json"
DEFAULT_HANDEYE_PATH = PROJECT / r"sam3\test\axis_fit_support\head_camera_transform_20260915\calibration\head_camera_handeye_20260914.json"
DEFAULT_SAMPLE_ID = "125311198"
DEFAULT_CLASS_NAME = "box"
DEFAULT_INSTANCE_ID = None                 # None => highest score; ties use lower 1-based ID
DEFAULT_OUTPUT_ROOT = PROJECT / r"sam3\test\estee_box_top_results"
DEFAULT_PLANE_MODE = "horizontal"
DEFAULT_MASK_EROSION_PIXELS = 1
DEFAULT_HEIGHT_TOLERANCE_MM = 2.5
DEFAULT_NORMAL_MAX_TILT_DEG = 12.0
DEFAULT_MIN_TOP_POINTS = 120
DEFAULT_MIN_VALID_DEPTH_RATIO = 0.60
DEFAULT_MIN_TANGENT_SPAN_MM = 8.0
DEFAULT_MIN_TOP_AREA_MM2 = 250.0
DEFAULT_MIN_INTERIOR_MARGIN_PX = 2.0
DEFAULT_MAX_RESIDUAL_MEDIAN_MM = 1.5
DEFAULT_MAX_RESIDUAL_P90_MM = 2.5
DEFAULT_MAX_HEIGHT_STABILITY_MM = 3.0
DEFAULT_MAX_POINT_STABILITY_MM = 8.0
DEFAULT_SHOW_3D = False
DEPTH_SCALE = 1.0                         # verified aligned NPY already stores millimetres
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


def load_camera(path):
    d = json.loads(Path(path).read_text(encoding="utf-8"))
    a = np.asarray(d.get("cam_K") or d.get("camera_matrix"), float).reshape(3, 3)
    if a[0, 0] <= 0 or a[1, 1] <= 0 or not np.allclose(a[2], [0, 0, 1]):
        raise ValueError("invalid pinhole K")
    return {"fx": float(a[0, 0]), "fy": float(a[1, 1]), "cx": float(a[0, 2]), "cy": float(a[1, 2])}, d


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


def validate_calibration(handeye_path, metadata_path, camera_doc, rgb_shape):
    h = json.loads(Path(handeye_path).read_text(encoding="utf-8"))
    m = json.loads(Path(metadata_path).read_text(encoding="utf-8"))
    if h.get("base_frame") != "chassis_link" or h.get("camera_frame") != "head_camera_color_optical_frame":
        raise ValueError("calibration frame metadata is inconsistent")
    hc, mc = h.get("camera", {}), m.get("camera", {})
    if hc.get("serial_number") != mc.get("serial_number") or hc.get("model") != mc.get("model"):
        raise ValueError("capture and calibration camera identity differ")
    size = h.get("intrinsics", {}).get("image_size")
    if size != [rgb_shape[1], rgb_shape[0]] or camera_doc.get("width") != rgb_shape[1] or camera_doc.get("height") != rgb_shape[0]:
        raise ValueError("calibration/camera.json/RGB dimensions differ")
    hk = np.asarray(h.get("intrinsics", {}).get("camera_matrix"), float).reshape(3, 3)
    ck = np.asarray(camera_doc.get("cam_K") or camera_doc.get("camera_matrix"), float).reshape(3, 3)
    if not np.allclose(hk, ck, atol=1e-9):
        raise ValueError("capture K and calibration K differ")
    return {"schema": h.get("schema"), "calibration_type": h.get("calibration_type"), "camera_model": hc.get("model"),
            "camera_serial_number": hc.get("serial_number"), "image_size": size,
            "distortion_coefficients": h.get("intrinsics", {}).get("distortion_coefficients"), "rms_px": h.get("intrinsics", {}).get("rms_px")}


def backproject(mask, depth_mm, K):
    valid = mask & np.isfinite(depth_mm) & (depth_mm > 0)
    y, x = np.where(valid)
    z = depth_mm[valid]
    p = np.column_stack(((x - K["cx"]) * z / K["fx"], (y - K["cy"]) * z / K["fy"], z))
    return p, np.column_stack((x, y)), valid


def camera_to_chassis(points_camera, T):
    p = np.asarray(points_camera, float)
    return (T[:3, :3] @ p.T).T + T[:3, 3] * 1000.0


def chassis_to_camera(points_chassis, T):
    p = np.asarray(points_chassis, float)
    return (T[:3, :3].T @ (p - T[:3, 3] * 1000.0).T).T


def project(points_camera, K):
    p = np.asarray(points_camera, float)
    return np.column_stack((K["fx"] * p[:, 0] / p[:, 2] + K["cx"], K["fy"] * p[:, 1] / p[:, 2] + K["cy"]))


def largest_component(point_mask):
    labels, count = ndimage.label(point_mask, structure=np.ones((3, 3), int))
    if count == 0:
        return point_mask.copy(), 0
    sizes = np.bincount(labels.ravel()); sizes[0] = 0
    label = int(np.argmax(sizes))
    return labels == label, int(sizes[label])


def support_metrics(points_xy):
    p = np.asarray(points_xy, float)
    if len(p) < 3:
        return {"span_major_mm": 0.0, "span_minor_mm": 0.0, "area_mm2": 0.0}
    q = p - np.median(p, axis=0)
    _, _, vt = np.linalg.svd(q, full_matrices=False)
    uv = q @ vt.T
    lo, hi = np.percentile(uv, [5, 95], axis=0)
    spans = np.sort(hi - lo)[::-1]
    try:
        area = float(ConvexHull(p).volume)
    except Exception:
        area = 0.0
    return {"span_major_mm": float(spans[0]), "span_minor_mm": float(spans[1]), "area_mm2": area}


def height_candidates(points_chassis, pixels, shape, tolerance_mm, min_points, min_span, min_area):
    z = points_chassis[:, 2]
    lo, hi = np.percentile(z, [1, 99])
    if hi - lo < 1.0:
        seeds = [float(np.median(z))]
    else:
        step = max(0.5, tolerance_mm / 2.0)
        edges = np.arange(lo - step, hi + 2 * step, step)
        hist, edges = np.histogram(z, edges)
        smooth = ndimage.gaussian_filter1d(hist.astype(float), 1.0)
        peaks = [i for i in range(1, len(smooth) - 1) if smooth[i] >= smooth[i - 1] and smooth[i] >= smooth[i + 1]]
        peaks = sorted(peaks, key=lambda i: smooth[i], reverse=True)[:12]
        seeds = [float((edges[i] + edges[i + 1]) * 0.5) for i in peaks]
        seeds += [float(np.percentile(z, q)) for q in (50, 65, 75, 85, 90)]
    out = []
    used = []
    for seed in seeds:
        if any(abs(seed - x) < tolerance_mm * 0.4 for x in used):
            continue
        raw = np.abs(z - seed) <= tolerance_mm
        pm = np.zeros(shape, bool); pm[pixels[raw, 1], pixels[raw, 0]] = True
        comp, n = largest_component(pm)
        keep = raw & comp[pixels[:, 1], pixels[:, 0]]
        if n:
            h = float(np.median(z[keep]))
            keep = np.abs(z - h) <= tolerance_mm
            pm[:] = False; pm[pixels[keep, 1], pixels[keep, 0]] = True
            comp, n = largest_component(pm)
            keep &= comp[pixels[:, 1], pixels[:, 0]]
        metrics = support_metrics(points_chassis[keep, :2])
        qualified = bool(n >= min_points and metrics["span_minor_mm"] >= min_span and metrics["area_mm2"] >= min_area)
        out.append({"seed_height_mm": seed, "height_mm": float(np.median(z[keep])) if n else seed,
                    "point_count": int(n), **metrics, "qualified": qualified, "_keep": keep})
        used.append(seed)
    out.sort(key=lambda c: (not c["qualified"], -c["height_mm"], -c["area_mm2"], -c["point_count"]))
    return out


def tangent_basis(normal):
    n = unit(normal)
    ref = np.array([1.0, 0.0, 0.0]) if abs(n[0]) < 0.8 else np.array([0.0, 1.0, 0.0])
    e1 = unit(np.cross(n, ref)); e2 = unit(np.cross(n, e1))
    return e1, e2


def plane_residual(points, plane_point, normal):
    return (np.asarray(points) - plane_point) @ unit(normal)


def constrained_plane(points, initial_keep, max_tilt_deg, tolerance_mm):
    p = np.asarray(points, float)
    seed = p[initial_keep]
    x0, y0 = np.median(seed[:, :2], axis=0)
    h0 = float(np.median(seed[:, 2]))
    slope = math.tan(math.radians(max_tilt_deg))

    def residual(v, data):
        return data[:, 2] - (v[0] + v[1] * (data[:, 0] - x0) + v[2] * (data[:, 1] - y0))

    current = initial_keep.copy(); sol = None
    for _ in range(4):
        sol = least_squares(lambda v: residual(v, p[current]), [h0, 0.0, 0.0], bounds=([-np.inf, -slope, -slope], [np.inf, slope, slope]), loss="soft_l1", f_scale=1.0)
        rr = residual(sol.x, p)
        new = np.abs(rr) <= tolerance_mm
        if np.array_equal(new, current):
            break
        current = new
    h, ax, ay = sol.x
    normal = unit(np.array([-ax, -ay, 1.0]))
    point = np.array([x0, y0, h])
    return point, normal, residual(sol.x, p), current, {"optimizer_success": bool(sol.success), "optimizer_cost": float(sol.cost)}


def fit_top(points_chassis, pixels, image_shape, mode, cfg):
    candidates = height_candidates(points_chassis, pixels, image_shape, cfg["tol"], cfg["min_points"], cfg["min_span"], cfg["min_area"])
    qualified = [c for c in candidates if c["qualified"]]
    if not qualified:
        return {"valid": False, "reason": "no height candidate has sufficient connected two-dimensional support", "candidates": strip_candidates(candidates)}
    chosen = qualified[0]
    ambiguity = False
    for other in qualified[1:]:
        if abs(other["height_mm"] - chosen["height_mm"]) > cfg["tol"] and other["area_mm2"] >= 0.8 * chosen["area_mm2"]:
            ambiguity = True
            break
    initial = chosen["_keep"].copy()
    if mode == "horizontal":
        h = float(np.median(points_chassis[initial, 2]))
        point = np.array([np.median(points_chassis[initial, 0]), np.median(points_chassis[initial, 1]), h])
        normal = np.array([0.0, 0.0, 1.0])
        residual = points_chassis[:, 2] - h
        keep0 = np.abs(residual) <= cfg["tol"]
        extra = {"optimizer_success": True, "optimizer_cost": None}
    elif mode == "constrained_tilt":
        point, normal, residual, keep0, extra = constrained_plane(points_chassis, initial, cfg["max_tilt"], cfg["tol"])
    else:
        raise ValueError("unknown plane mode")
    pm = np.zeros(image_shape, bool); pm[pixels[keep0, 1], pixels[keep0, 0]] = True
    comp, _ = largest_component(pm)
    keep = keep0 & comp[pixels[:, 1], pixels[:, 0]]
    if mode == "horizontal" and keep.any():
        point[2] = float(np.median(points_chassis[keep, 2])); residual = points_chassis[:, 2] - point[2]
        keep = (np.abs(residual) <= cfg["tol"]) & comp[pixels[:, 1], pixels[:, 0]]
    final_top_mask = np.zeros(image_shape, bool)
    final_top_mask[pixels[keep, 1], pixels[keep, 0]] = True
    metrics = support_metrics(project_to_plane(points_chassis[keep], point, normal))
    absr = np.abs(plane_residual(points_chassis[keep], point, normal)) if keep.any() else np.array([np.inf])
    tilt = math.degrees(math.acos(np.clip(normal[2], -1, 1)))
    valid = bool(len(absr) >= cfg["min_points"] and metrics["span_minor_mm"] >= cfg["min_span"] and metrics["area_mm2"] >= cfg["min_area"] and np.median(absr) <= cfg["max_med"] and np.percentile(absr, 90) <= cfg["max_p90"] and tilt <= cfg["max_tilt"] and not ambiguity)
    reasons = []
    if len(absr) < cfg["min_points"]: reasons.append("insufficient top-plane inlier points")
    if metrics["span_minor_mm"] < cfg["min_span"] or metrics["area_mm2"] < cfg["min_area"]: reasons.append("top support is too narrow or too small in plane")
    if np.median(absr) > cfg["max_med"] or np.percentile(absr, 90) > cfg["max_p90"]: reasons.append("top-plane residual gate failed")
    if tilt > cfg["max_tilt"]: reasons.append("plane normal exceeds chassis-up tilt allowance")
    if ambiguity: reasons.append("multiple separated height candidates have comparable support")
    return {"valid": valid, "reasons": reasons, "plane_point_chassis_mm": point, "normal_chassis": normal,
            "residual_all_mm": plane_residual(points_chassis, point, normal), "keep": keep, "top_mask": final_top_mask,
            "metrics": metrics, "residual_median_mm": float(np.median(absr)), "residual_p90_mm": float(np.percentile(absr, 90)),
            "normal_tilt_deg": tilt, "ambiguity": ambiguity, "chosen_candidate": {k: v for k, v in chosen.items() if k != "_keep"},
            "candidates": strip_candidates(candidates), **extra}


def strip_candidates(candidates):
    return [{k: v for k, v in c.items() if k != "_keep"} for c in candidates]


def project_to_plane(points, plane_point, normal):
    e1, e2 = tangent_basis(normal)
    q = np.asarray(points) - plane_point
    return np.column_stack((q @ e1, q @ e2))


def ray_plane_pixel(pixel, K, plane_point_chassis, normal_chassis, T):
    u, v = map(float, pixel)
    ray_cam = np.array([(u - K["cx"]) / K["fx"], (v - K["cy"]) / K["fy"], 1.0])
    ray_ch = T[:3, :3] @ ray_cam
    origin = T[:3, 3] * 1000.0
    den = float(np.dot(normal_chassis, ray_ch))
    if abs(den) < 1e-8:
        raise ValueError("camera ray is nearly parallel to top plane")
    lam = float(np.dot(normal_chassis, plane_point_chassis - origin) / den)
    if lam <= 0:
        raise ValueError("top-plane intersection is behind camera")
    pc = lam * ray_cam
    return pc, origin + lam * ray_ch, lam


def mask_centroid(mask):
    y, x = np.where(mask)
    if len(x) == 0:
        raise ValueError("empty mask")
    return np.array([x.mean(), y.mean()])


def interior_pixel(top_mask):
    dist = cv2.distanceTransform(top_mask.astype(np.uint8), cv2.DIST_L2, 5)
    maximum = float(dist.max())
    y, x = np.where(dist >= maximum - 1e-6)
    if len(x) == 0:
        raise ValueError("empty top support")
    # Select one real maximum-distance pixel.  Never synthesize x/y medians
    # from different maxima: on a dumbbell-shaped support that synthetic point
    # can land on the narrow connector and overstate its actual margin.
    cy, cx = np.mean(np.where(top_mask), axis=1)
    order = np.lexsort((x, y, (x - cx) ** 2 + (y - cy) ** 2))
    chosen_x, chosen_y = int(x[order[0]]), int(y[order[0]])
    margin = float(dist[chosen_y, chosen_x])
    return np.array([float(chosen_x), float(chosen_y)]), margin, dist


def attach_interior_point(analysis, mask, K, T, cfg):
    fit = analysis.get("fit", {})
    if not fit.get("valid"):
        analysis.update({"valid": False, "reason": "; ".join(fit.get("reasons", [fit.get("reason", "plane invalid")]))})
        return analysis
    baseline_px = mask_centroid(mask)
    try:
        recommended_px, margin, _ = interior_pixel(fit["top_mask"])
        baseline_cam, baseline_ch, _ = ray_plane_pixel(baseline_px, K, fit["plane_point_chassis_mm"], fit["normal_chassis"], T)
        ref_cam, ref_ch, _ = ray_plane_pixel(recommended_px, K, fit["plane_point_chassis_mm"], fit["normal_chassis"], T)
    except ValueError as exc:
        analysis.update({"valid": False, "reason": str(exc), "baseline_pixel": baseline_px})
        return analysis
    ui = np.rint(recommended_px).astype(int)
    supported = bool(0 <= ui[0] < mask.shape[1] and 0 <= ui[1] < mask.shape[0] and fit["top_mask"][ui[1], ui[0]])
    analysis.update({
        "valid": bool(supported and margin >= cfg["min_margin"]),
        "baseline_pixel": baseline_px,
        "baseline_camera_mm": baseline_cam,
        "baseline_chassis_mm": baseline_ch,
        "reference_pixel": recommended_px,
        "reference_camera_mm": ref_cam,
        "reference_chassis_mm": ref_ch,
        "interior_margin_px": margin,
        "reference_supported": supported,
        "point_semantics": "visible_top_interior_point"
    })
    if not analysis["valid"]:
        analysis["reason"] = "reference point is not sufficiently inside supported top region"
    return analysis


def analyse(mask, depth, K, T, erosion, mode, cfg, geometry_cache=None, compute_interior=True):
    inner = erode_mask(mask, erosion)
    if geometry_cache is None:
        pc, pixels, valid = backproject(inner, depth, K)
    else:
        pc, pixels, valid, ch = geometry_cache.get_mask(inner, depth, K, T)
    out = {"erosion_pixels": erosion, "mask_area_pixels": int(mask.sum()), "eroded_mask_area_pixels": int(inner.sum()),
           "valid_depth_points": int(len(pc)), "valid_depth_ratio": float(len(pc) / max(1, inner.sum()))}
    if len(pc) < cfg["min_points"]:
        out.update({"valid": False, "reason": "insufficient valid depth points"}); return out
    if geometry_cache is None:
        ch = camera_to_chassis(pc, T)
    fit = fit_top(ch, pixels, mask.shape, mode, cfg)
    out.update({"camera_points": pc, "chassis_points": ch, "pixels": pixels, "valid_mask": valid, "inner_mask": inner, "fit": fit})
    if not fit["valid"]:
        out.update({"valid": False, "reason": "; ".join(fit.get("reasons", [fit.get("reason", "plane invalid")]))}); return out
    if compute_interior:
        return attach_interior_point(out, mask, K, T, cfg)
    out.update({"plane_valid": True, "valid": False, "reason": "interior point not computed"})
    return out


def write_ply(path, camera_points, top_keep):
    colors = np.tile(np.array([120, 120, 120], int), (len(camera_points), 1)); colors[top_keep] = [40, 210, 60]
    with Path(path).open("w", encoding="ascii") as f:
        f.write("ply\nformat ascii 1.0\nelement vertex {}\nproperty float x\nproperty float y\nproperty float z\nproperty uchar red\nproperty uchar green\nproperty uchar blue\nend_header\n".format(len(camera_points)))
        for p, c in zip(camera_points, colors): f.write("{:.6f} {:.6f} {:.6f} {} {} {}\n".format(*p, *c))


def set_equal(ax, points):
    p = np.asarray(points); lo = p.min(axis=0); hi = p.max(axis=0); center = (lo + hi) / 2; radius = max((hi - lo).max() / 2, 1.0)
    ax.set_xlim(center[0] - radius, center[0] + radius); ax.set_ylim(center[1] - radius, center[1] + radius); ax.set_zlim(center[2] - radius, center[2] + radius)
    ax.set_box_aspect((1, 1, 1))


def render_3d(result, T, path, show=False):
    import matplotlib.pyplot as plt
    pc = result["camera_points"]
    fit = result.get("fit", {})
    keep = np.asarray(fit.get("keep", np.zeros(len(pc), bool)), dtype=bool)
    if keep.shape != (len(pc),):
        keep = np.zeros(len(pc), bool)
    fig = plt.figure(figsize=(11, 8)); ax = fig.add_subplot(111, projection="3d")
    ax.scatter(pc[~keep, 0], pc[~keep, 1], pc[~keep, 2], s=3, c="#777777", alpha=.35, label="non-top / abnormal")
    if keep.any(): ax.scatter(pc[keep, 0], pc[keep, 1], pc[keep, 2], s=5, c="#22bb55", alpha=.8, label="top inliers")
    if result.get("baseline_camera_mm") is not None: ax.scatter(*result["baseline_camera_mm"], s=80, c="#ff9900", marker="x", label="whole-mask centroid ray")
    if result.get("reference_camera_mm") is not None: ax.scatter(*result["reference_camera_mm"], s=90, c="#d000ff", marker="*", label="supported interior point")
    scale_points = pc
    if fit.get("plane_point_chassis_mm") is not None and fit.get("normal_chassis") is not None and fit.get("metrics") is not None:
        pp = chassis_to_camera(np.asarray(fit["plane_point_chassis_mm"])[None], T)[0]
        nc = unit(T[:3, :3].T @ fit["normal_chassis"])
        e1, e2 = tangent_basis(nc); metrics = fit["metrics"]; r1 = metrics["span_major_mm"] / 2; r2 = metrics["span_minor_mm"] / 2
        uu, vv = np.meshgrid(np.linspace(-r1, r1, 8), np.linspace(-r2, r2, 8)); surf = pp + uu[..., None] * e1 + vv[..., None] * e2
        ax.plot_surface(surf[..., 0], surf[..., 1], surf[..., 2], alpha=.18, color="#33cc66", edgecolor="#228844")
        ax.quiver(*pp, *nc, length=30, color="blue", linewidth=2, label="top normal")
        scale_points = np.vstack((pc, pp))
    ax.set_xlabel("camera X (mm)"); ax.set_ylabel("camera Y (mm)"); ax.set_zlabel("camera Z (mm)"); ax.legend(loc="best")
    ax.set_title("Box top separation — camera optical frame, mm\nmouse drag rotates; wheel zooms")
    set_equal(ax, scale_points); fig.tight_layout(); fig.savefig(str(path), dpi=180)
    if show: plt.show()
    plt.close(fig)


def draw_overlay(rgb, detections, selected_id, mask, result, K, out):
    im = rgb.copy()
    for i, d in enumerate(detections, 1):
        x, y, w, h = map(int, d.get("bbox", [0, 0, 0, 0])); color = (0, 255, 255) if i == selected_id else (210, 210, 210)
        cv2.rectangle(im, (x, y), (x + w, y + h), color, 1); cv2.putText(im, "#{} {:.3f}".format(i, float(d.get("score", 0))), (x, max(15, y - 4)), cv2.FONT_HERSHEY_SIMPLEX, .42, color, 1, cv2.LINE_AA)
    im[mask] = (.55 * im[mask] + .45 * np.array([255, 150, 0])).astype(np.uint8)
    if "fit" in result and result["fit"].get("top_mask") is not None:
        tm = result["fit"]["top_mask"] & result["inner_mask"]; rejected = result["inner_mask"] & ~tm
        im[rejected] = (.55 * im[rejected] + .45 * np.array([80, 80, 80])).astype(np.uint8)
        im[tm] = (.30 * im[tm] + .70 * np.array([0, 220, 40])).astype(np.uint8)
    if result.get("baseline_pixel") is not None:
        cv2.drawMarker(im, tuple(np.rint(result["baseline_pixel"]).astype(int)), (0, 150, 255), cv2.MARKER_CROSS, 15, 2)
    if result.get("reference_pixel") is not None:
        cv2.drawMarker(im, tuple(np.rint(result["reference_pixel"]).astype(int)), (255, 0, 255), cv2.MARKER_STAR, 18, 2)
    status = "plane={} point={} {}".format(bool(result.get("fit", {}).get("valid")), bool(result.get("valid")), result.get("point_semantics", "no formal point"))
    cv2.putText(im, status, (18, im.shape[0] - 18), cv2.FONT_HERSHEY_SIMPLEX, .55, (0, 0, 0), 3, cv2.LINE_AA); cv2.putText(im, status, (18, im.shape[0] - 18), cv2.FONT_HERSHEY_SIMPLEX, .55, (255, 255, 255), 1, cv2.LINE_AA)
    cv2.imwrite(str(out), im)


def serialise_result(x):
    if isinstance(x, np.ndarray): return x.tolist()
    if isinstance(x, (np.floating, np.integer)): return x.item()
    raise TypeError(type(x).__name__)


def synthetic_test():
    rng = np.random.default_rng(7)
    cfg = make_cfg(argparse.Namespace(height_tolerance_mm=2.5, min_top_points=100, min_tangent_span_mm=8.0, min_top_area_mm2=250.0, normal_max_tilt_deg=12.0, max_residual_median_mm=1.5, max_residual_p90_mm=2.5, min_interior_margin_px=2.0))
    shape = (240, 320)
    # A broad horizontal top plus a denser vertical side. Pixel support is separate and connected.
    xt, yt = np.meshgrid(np.linspace(0, 70, 30), np.linspace(0, 35, 18)); top = np.c_[xt.ravel(), yt.ravel(), 100 + rng.normal(0, .45, xt.size)]
    xs, zs = np.meshgrid(np.linspace(0, 70, 45), np.linspace(60, 98, 25)); side = np.c_[xs.ravel(), np.full(xs.size, 36.0), zs.ravel() + rng.normal(0, .25, xs.size)]
    points = np.vstack((top, side)); pt = np.c_[20 + np.tile(np.arange(30), 18), 20 + np.repeat(np.arange(18), 30)]; ps = np.c_[20 + np.tile(np.arange(45), 25), 39 + np.repeat(np.arange(25), 45)]; pixels = np.vstack((pt, ps)).astype(int)
    fit = fit_top(points, pixels, shape, "horizontal", cfg)
    assert fit["valid"] and abs(fit["plane_point_chassis_mm"][2] - 100) < 1.0, fit
    # Side only must fail the two-dimensional horizontal support gate.
    side_fit = fit_top(side, ps.astype(int), shape, "horizontal", cfg)
    assert not side_fit["valid"]
    # Highest sparse outliers cannot replace the supported top.
    outliers = np.c_[rng.uniform(0, 70, 30), rng.uniform(0, 35, 30), rng.uniform(120, 150, 30)]
    opix = np.c_[rng.integers(150, 210, 30), rng.integers(150, 210, 30)]
    fit2 = fit_top(np.vstack((points, outliers)), np.vstack((pixels, opix)), shape, "horizontal", cfg)
    assert fit2["valid"] and abs(fit2["plane_point_chassis_mm"][2] - 100) < 1.0
    # Constrained tilt recovers a small tilt and tolerates a missing patch.
    tilted = top.copy(); tilted[:, 2] = 100 + .04 * tilted[:, 0] - .025 * tilted[:, 1] + rng.normal(0, .35, len(tilted)); missing = ~((tilted[:, 0] > 25) & (tilted[:, 0] < 45) & (tilted[:, 1] > 10) & (tilted[:, 1] < 25))
    tf = fit_top(tilted[missing], pt[missing].astype(int), shape, "constrained_tilt", cfg)
    assert tf["valid"] and 1 < tf["normal_tilt_deg"] < 6
    # Transform metres->millimetres, round trip, projection, parallel and behind rejection.
    ay = math.radians(25); az = math.radians(15); Ry = np.array([[math.cos(ay), 0, math.sin(ay)], [0, 1, 0], [-math.sin(ay), 0, math.cos(ay)]]); Rz = np.array([[math.cos(az), -math.sin(az), 0], [math.sin(az), math.cos(az), 0], [0, 0, 1]])
    T = np.eye(4); T[:3, :3] = Rz @ Ry; T[:3, 3] = [.2, -.1, 1.4]
    p = np.array([[10., 20., 800.]]); assert np.linalg.norm(chassis_to_camera(camera_to_chassis(p, T), T) - p) < 1e-9
    K = {"fx": 600., "fy": 600., "cx": 320., "cy": 240.}; pc, ch, _ = ray_plane_pixel([320, 240], K, np.array([0., 0., 1800.]), np.array([0., 0., 1.]), T); assert abs(ch[2] - 1800) < 1e-9 and pc[2] > 0
    parallel = behind = False
    try: ray_plane_pixel([320, 240], K, np.zeros(3), unit(T[:3, :3] @ np.array([1., 0., 0.])), T)
    except ValueError: parallel = True
    try: ray_plane_pixel([320, 240], K, np.array([0., 0., 1000.]), np.array([0., 0., 1.]), T)
    except ValueError: behind = True
    assert parallel and behind
    # Regression: two wide lobes joined by a one-pixel bridge.  The selected
    # point must be an actual maximum-distance pixel and report its own margin.
    dumbbell = np.zeros((40, 80), np.uint8); dumbbell[5:30, 5:25] = 1; dumbbell[5:30, 55:75] = 1; dumbbell[17, 25:55] = 1
    ip, margin, distance = interior_pixel(dumbbell.astype(bool)); ix, iy = np.rint(ip).astype(int)
    assert dumbbell[iy, ix] and abs(margin - float(distance[iy, ix])) < 1e-9 and margin == float(distance.max())
    # Regression: diagnostic rendering must survive a rejected plane with raw
    # points but without keep/plane/normal fields.
    with tempfile.TemporaryDirectory() as td:
        invalid = {"camera_points": p.repeat(8, axis=0), "fit": {"valid": False, "reason": "synthetic rejection"}}
        invalid_png = Path(td) / "invalid_plane.png"; render_3d(invalid, T, invalid_png, False); assert invalid_png.exists() and invalid_png.stat().st_size > 0
    print(json.dumps({"synthetic_test": "PASS", "mixed_side_more_than_top": True, "side_only_rejected": True, "sparse_high_outliers_rejected": True, "tilted_missing_top_valid": True, "transform_roundtrip_mm": float(np.linalg.norm(chassis_to_camera(camera_to_chassis(p, T), T) - p)), "parallel_rejected": parallel, "behind_rejected": behind, "interior_pixel_is_real_maximum": True, "invalid_plane_render_saved": True}, indent=2))


def make_cfg(args):
    return {"tol": args.height_tolerance_mm, "min_points": args.min_top_points, "min_span": args.min_tangent_span_mm,
            "min_area": args.min_top_area_mm2, "max_tilt": args.normal_max_tilt_deg, "max_med": args.max_residual_median_mm,
            "max_p90": args.max_residual_p90_mm, "min_margin": args.min_interior_margin_px}


def main():
    ap = argparse.ArgumentParser(description="Offline box top-surface reference-point diagnostic")
    ap.add_argument("--result-json", type=Path, default=DEFAULT_RESULT_JSON); ap.add_argument("--rgb-path", type=Path, default=DEFAULT_RGB_PATH); ap.add_argument("--depth-path", type=Path, default=DEFAULT_DEPTH_PATH); ap.add_argument("--camera-path", type=Path, default=DEFAULT_CAMERA_PATH); ap.add_argument("--extrinsics-path", type=Path, default=DEFAULT_EXTRINSICS_PATH); ap.add_argument("--handeye-path", type=Path, default=DEFAULT_HANDEYE_PATH); ap.add_argument("--sample-id", default=DEFAULT_SAMPLE_ID); ap.add_argument("--instance-id", type=int, default=DEFAULT_INSTANCE_ID); ap.add_argument("--output-root", type=Path, default=DEFAULT_OUTPUT_ROOT); ap.add_argument("--plane-mode", choices=["horizontal", "constrained_tilt"], default=DEFAULT_PLANE_MODE); ap.add_argument("--mask-erosion-pixels", type=int, default=DEFAULT_MASK_EROSION_PIXELS); ap.add_argument("--height-tolerance-mm", type=float, default=DEFAULT_HEIGHT_TOLERANCE_MM); ap.add_argument("--normal-max-tilt-deg", type=float, default=DEFAULT_NORMAL_MAX_TILT_DEG); ap.add_argument("--min-top-points", type=int, default=DEFAULT_MIN_TOP_POINTS); ap.add_argument("--min-tangent-span-mm", type=float, default=DEFAULT_MIN_TANGENT_SPAN_MM); ap.add_argument("--min-top-area-mm2", type=float, default=DEFAULT_MIN_TOP_AREA_MM2); ap.add_argument("--min-interior-margin-px", type=float, default=DEFAULT_MIN_INTERIOR_MARGIN_PX); ap.add_argument("--max-residual-median-mm", type=float, default=DEFAULT_MAX_RESIDUAL_MEDIAN_MM); ap.add_argument("--max-residual-p90-mm", type=float, default=DEFAULT_MAX_RESIDUAL_P90_MM); ap.add_argument("--show-3d", action="store_true", default=DEFAULT_SHOW_3D); ap.add_argument("--synthetic-test", action="store_true"); args = ap.parse_args()
    if args.synthetic_test: synthetic_test(); return 0
    required = [args.result_json, args.rgb_path, args.depth_path, args.camera_path, args.extrinsics_path, args.handeye_path, args.rgb_path.parent / "head_camera_metadata.json", args.rgb_path.parent / "robot_state.json"]
    for p in required:
        if not Path(p).exists(): raise FileNotFoundError(p)
    result_doc = json.loads(args.result_json.read_text(encoding="utf-8")); detections = result_doc.get("detections", [])
    if not detections: raise ValueError("no detections")
    selected_id = args.instance_id if args.instance_id is not None else min(range(1, len(detections) + 1), key=lambda i: (-float(detections[i - 1].get("score", -1)), i))
    if not 1 <= selected_id <= len(detections): raise ValueError("instance-id out of range")
    rgb = cv2.imread(str(args.rgb_path), cv2.IMREAD_COLOR); depth_raw = np.load(str(args.depth_path), allow_pickle=False)
    if rgb is None or depth_raw.ndim != 2 or rgb.shape[:2] != depth_raw.shape: raise ValueError("RGB/aligned depth size mismatch")
    if depth_raw.dtype != np.float32: raise ValueError("verified capture depth must be float32 millimetres")
    depth = depth_raw.astype(float) * DEPTH_SCALE; K, camera_doc = load_camera(args.camera_path); T, row = load_extrinsic(args.extrinsics_path, args.sample_id, args.rgb_path.parent / "robot_state.json")
    calibration = validate_calibration(args.handeye_path, args.rgb_path.parent / "head_camera_metadata.json", camera_doc, rgb.shape[:2]); mask = decode_rle(detections[selected_id - 1])
    if mask.shape != rgb.shape[:2]: raise ValueError("mask/RGB dimensions differ")
    cfg = make_cfg(args); analyses = {str(e): analyse(mask, depth, K, T, e, args.plane_mode, cfg) for e in (0, 1, 2)}
    main_result = analyses[str(args.mask_erosion_pixels)]
    heights = [a["fit"]["plane_point_chassis_mm"][2] for a in analyses.values() if a.get("fit", {}).get("valid")]
    points = [a["reference_camera_mm"] for a in analyses.values() if a.get("valid")]
    height_var = float(max(heights) - min(heights)) if len(heights) >= 2 else None
    point_var = float(max(np.linalg.norm(np.asarray(a) - np.asarray(b)) for a in points for b in points)) if len(points) >= 2 else None
    plane_ok = bool(main_result.get("fit", {}).get("valid")); point_ok = bool(main_result.get("valid") and height_var is not None and point_var is not None and height_var <= DEFAULT_MAX_HEIGHT_STABILITY_MM and point_var <= DEFAULT_MAX_POINT_STABILITY_MM and main_result["valid_depth_ratio"] >= DEFAULT_MIN_VALID_DEPTH_RATIO)
    reasons = [] if plane_ok else [main_result.get("reason", "top plane invalid")]
    if main_result.get("valid_depth_ratio", 0) < DEFAULT_MIN_VALID_DEPTH_RATIO: reasons.append("valid depth ratio below threshold")
    if height_var is None or point_var is None: reasons.append("insufficient successful erosion variants for stability check")
    else:
        if height_var > DEFAULT_MAX_HEIGHT_STABILITY_MM: reasons.append("top height unstable across 0/1/2 px erosion")
        if point_var > DEFAULT_MAX_POINT_STABILITY_MM: reasons.append("top point unstable across 0/1/2 px erosion")
    if not main_result.get("valid") and plane_ok: reasons.append(main_result.get("reason", "top point unsupported"))
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S_%f"); root = args.output_root / timestamp / "instance_{:03d}".format(selected_id); root.mkdir(parents=True, exist_ok=False)
    cv2.imwrite(str(root / "mask_original.png"), mask.astype(np.uint8) * 255)
    cv2.imwrite(str(root / "mask_eroded.png"), main_result.get("inner_mask", np.zeros_like(mask)).astype(np.uint8) * 255)
    missing_depth_mask = main_result.get("inner_mask", mask) & ~(np.isfinite(depth) & (depth > 0))
    cv2.imwrite(str(root / "mask_missing_depth.png"), missing_depth_mask.astype(np.uint8) * 255)
    top_mask = main_result.get("fit", {}).get("top_mask", np.zeros_like(mask)); cv2.imwrite(str(root / "mask_top_surface.png"), top_mask.astype(np.uint8) * 255); cv2.imwrite(str(root / "mask_rejected.png"), (main_result.get("inner_mask", mask) & ~top_mask).astype(np.uint8) * 255)
    draw_overlay(rgb, detections, selected_id, mask, main_result, K, root / "rgb_top_surface_overlay.png")
    if "camera_points" in main_result:
        write_ply(root / "point_cloud.ply", main_result["camera_points"], main_result["fit"].get("keep", np.zeros(len(main_result["camera_points"]), bool)))
        np.savez_compressed(root / "point_cloud_classified.npz", camera_points_mm=main_result["camera_points"], chassis_points_mm=main_result["chassis_points"], pixels_xy=main_result["pixels"], top_inlier=main_result["fit"].get("keep", np.zeros(len(main_result["camera_points"]), bool)))
        render_3d(main_result, T, root / "top_surface_3d.png", args.show_3d)
    baseline_delta = float(np.linalg.norm(main_result["baseline_camera_mm"] - main_result["reference_camera_mm"])) if main_result.get("baseline_camera_mm") is not None and main_result.get("reference_camera_mm") is not None else None
    report = {"class_name": DEFAULT_CLASS_NAME, "localization_method": "box_top_surface", "inputs": {"result_json": str(args.result_json.resolve()), "rgb": str(args.rgb_path.resolve()), "depth": str(args.depth_path.resolve()), "camera": str(args.camera_path.resolve()), "extrinsics": str(args.extrinsics_path.resolve()), "handeye": str(args.handeye_path.resolve())}, "source_verification": {"sample_id": args.sample_id, "pixel_exact_unique_match": str(args.rgb_path.resolve()), "capture_timestamp": json.loads((args.rgb_path.parent / "head_camera_metadata.json").read_text(encoding="utf-8"))["camera_frame_captured_at"], "depth_dtype": str(depth_raw.dtype), "depth_unit": "mm", "depth_scale": DEPTH_SCALE, "rgb_shape": list(rgb.shape), "depth_shape": list(depth.shape)}, "instance_id": selected_id, "array_index": selected_id - 1, "selection": "manual override" if args.instance_id is not None else "highest original SAM3 score; ties by lower 1-based ID", "sam3_score": float(detections[selected_id - 1]["score"]), "prompt": result_doc.get("prompt"), "plane_mode": args.plane_mode, "output_frame": "head_camera_color_optical_frame", "output_unit": "mm", "reference_frame": "chassis_link", "extrinsics": {"sample": row["sample"], "camera_frame": row["camera_frame"], "base_frame": row["base_frame"], "translation_input_unit": "m", "translation_internal_unit": "mm", "joint_match_verified": True}, "camera_model": {"K": K, "calibration_validation": calibration, "distortion_coefficients_available_in_handeye": bool(calibration.get("distortion_coefficients")), "limitation": PINHOLE_LIMITATION}, "thresholds": {**cfg, "min_valid_depth_ratio": DEFAULT_MIN_VALID_DEPTH_RATIO, "max_height_stability_mm": DEFAULT_MAX_HEIGHT_STABILITY_MM, "max_point_stability_mm": DEFAULT_MAX_POINT_STABILITY_MM}, "mask_area_pixels": int(mask.sum()), "eroded_mask_area_pixels": main_result.get("eroded_mask_area_pixels"), "valid_depth_points": main_result.get("valid_depth_points"), "missing_depth_pixels_after_erosion": int(missing_depth_mask.sum()), "valid_depth_ratio": main_result.get("valid_depth_ratio"), "top_plane_valid": plane_ok, "top_point_valid": point_ok, "rejection_reasons": reasons, "top_plane_point_chassis_mm": main_result.get("fit", {}).get("plane_point_chassis_mm"), "top_height_chassis_mm": main_result.get("fit", {}).get("plane_point_chassis_mm", [None, None, None])[2], "plane_normal_chassis": main_result.get("fit", {}).get("normal_chassis"), "plane_normal_camera": T[:3, :3].T @ main_result["fit"]["normal_chassis"] if plane_ok else None, "normal_tilt_from_chassis_up_deg": main_result.get("fit", {}).get("normal_tilt_deg"), "top_inlier_count": int(main_result.get("fit", {}).get("keep", np.array([], bool)).sum()), "top_residual_median_mm": main_result.get("fit", {}).get("residual_median_mm"), "top_residual_p90_mm": main_result.get("fit", {}).get("residual_p90_mm"), "top_support": main_result.get("fit", {}).get("metrics"), "plane_candidate_ambiguity": main_result.get("fit", {}).get("ambiguity"), "plane_candidates": main_result.get("fit", {}).get("candidates"), "point_semantics": main_result.get("point_semantics") if point_ok else None, "top_point_camera_mm": main_result.get("reference_camera_mm") if point_ok else None, "top_point_chassis_mm": main_result.get("reference_chassis_mm") if point_ok else None, "diagnostic_point_camera_mm": main_result.get("reference_camera_mm"), "diagnostic_point_chassis_mm": main_result.get("reference_chassis_mm"), "whole_mask_centroid_pixel": main_result.get("baseline_pixel"), "whole_mask_centroid_ray_plane_camera_mm": main_result.get("baseline_camera_mm"), "baseline_to_recommended_distance_mm": baseline_delta, "reference_pixel": main_result.get("reference_pixel"), "reference_interior_margin_px": main_result.get("interior_margin_px"), "erosion_sensitivity": {"height_range_mm": height_var, "point_max_pairwise_mm": point_var, "variants": {k: {"plane_valid": v.get("fit", {}).get("valid", False), "point_valid_before_stability_gate": v.get("valid", False), "height_chassis_mm": v.get("fit", {}).get("plane_point_chassis_mm", [None, None, None])[2], "point_camera_mm": v.get("reference_camera_mm"), "top_inlier_count": int(v.get("fit", {}).get("keep", np.array([], bool)).sum())} for k, v in analyses.items()}}, "limitations": ["single-frame diagnostic without external geometric ground truth", "chassis +Z is assumed approximately physically vertical", "visible supported interior point is not a reconstructed hidden full-box centre", "stability across correlated same-frame regions is not measured accuracy"]}
    (root / "top_surface_result.json").write_text(json.dumps(report, ensure_ascii=False, indent=2, default=serialise_result), encoding="utf-8")
    md = "# Box top-surface offline result\n\n- Class: `box`; localization method: `box_top_surface`.\n- Instance: `#{}` (score `{:.7f}`), selection: {}.\n- Mode: `{}`; sample: `{}`; output: camera optical frame, mm.\n- Plane valid: `{}`; point valid: `{}`.\n- Top height in chassis: `{}` mm; normal tilt: `{}` deg.\n- Formal point camera XYZ: `{}`.\n- Diagnostic supported point camera XYZ: `{}`.\n- Whole-mask centroid-ray to supported-point distance: `{}` mm.\n- Erosion 0/1/2 height range: `{}` mm; point spread: `{}` mm.\n- Rejection reasons: `{}`.\n\n## Limits\n\n- `chassis +Z` is an approximate horizontal prior, not a measured ground plane.\n- K-only pinhole projection ignores calibrated distortion; no repeated undistortion is performed.\n- The point means `visible_top_interior_point`; it is not a hidden full-box centre or robot-validated suction point.\n".format(selected_id, report["sam3_score"], report["selection"], args.plane_mode, args.sample_id, plane_ok, point_ok, report["top_height_chassis_mm"], report["normal_tilt_from_chassis_up_deg"], report["top_point_camera_mm"], report["diagnostic_point_camera_mm"], baseline_delta, height_var, point_var, reasons)
    (root / "report.md").write_text(md, encoding="utf-8")
    print(json.dumps({"class_name": DEFAULT_CLASS_NAME, "localization_method": "box_top_surface", "output_dir": str(root.resolve()), "instance_id": selected_id, "plane_mode": args.plane_mode, "top_plane_valid": plane_ok, "top_point_valid": point_ok, "top_height_chassis_mm": report["top_height_chassis_mm"], "top_point_camera_mm": report["top_point_camera_mm"], "diagnostic_point_camera_mm": report["diagnostic_point_camera_mm"], "baseline_to_recommended_distance_mm": baseline_delta, "erosion_height_range_mm": height_var, "erosion_point_spread_mm": point_var, "rejection_reasons": reasons}, ensure_ascii=False, indent=2, default=serialise_result))
    return 0 if point_ok else 3


if __name__ == "__main__":
    raise SystemExit(main())
