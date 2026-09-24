#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
import math
import sys
from datetime import datetime
from pathlib import Path
from typing import Any

import cv2
import numpy as np

PROJECT_ROOT = Path(__file__).resolve().parents[1]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from rokae_web.head_kinematics import (  # noqa: E402
    UpperBodySixDofKinematics,
    invert_transform,
    make_transform,
    rotation_to_rpy_deg,
)


METHODS = {
    "Tsai": cv2.CALIB_HAND_EYE_TSAI,
    "Park": cv2.CALIB_HAND_EYE_PARK,
    "Horaud": cv2.CALIB_HAND_EYE_HORAUD,
    "Andreff": cv2.CALIB_HAND_EYE_ANDREFF,
    "Daniilidis": cv2.CALIB_HAND_EYE_DANIILIDIS,
}


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="用 0914 头部 RGB-D 记录标定 Head_link 到头部相机光学坐标系的眼在手变换"
    )
    parser.add_argument("--data", default=str(PROJECT_ROOT / "data" / "20260914"))
    parser.add_argument(
        "--urdf",
        default="/home/admin/mui/1.5整机urdf-0.8AR5-20260520.zip",
        help="整机 URDF 文件或只含一个 URDF 的 zip",
    )
    parser.add_argument("--output", default=str(PROJECT_ROOT / "calibration" / "head_camera_handeye_20260914.json"))
    parser.add_argument("--board-cols", type=int, default=13, help="编码棋盘横向内角点数")
    parser.add_argument("--board-rows", type=int, default=8, help="编码棋盘纵向内角点数")
    parser.add_argument("--square-size-mm", type=float, default=20.0, help="方格边长")
    parser.add_argument("--marker-size-mm", type=float, default=15.0, help="ArUco 标记边长")
    parser.add_argument("--dictionary", default="DICT_5X5_100", help="OpenCV ArUco 字典名")
    return parser.parse_args()


def robust_limit(values: np.ndarray, floor: float) -> float:
    values = np.asarray(values, dtype=np.float64)
    median = float(np.median(values))
    mad = float(np.median(np.abs(values - median)))
    return max(float(floor), median + 3.5 * 1.4826 * mad)


def transform_dict(transform: np.ndarray) -> dict[str, Any]:
    matrix = np.asarray(transform, dtype=np.float64).reshape(4, 4)
    return {
        "matrix_4x4": np.round(matrix, 12).tolist(),
        "translation_m": np.round(matrix[:3, 3], 12).tolist(),
        "translation_mm": np.round(matrix[:3, 3] * 1000.0, 6).tolist(),
        "rpy_deg": np.round(rotation_to_rpy_deg(matrix[:3, :3]), 9).tolist(),
    }


def canonical_corners(corners: np.ndarray, cols: int, rows: int) -> np.ndarray:
    grid = np.asarray(corners, dtype=np.float32).reshape(rows, cols, 2)
    if float(grid[0, 0].sum()) > float(grid[-1, -1].sum()):
        grid = grid[::-1, ::-1]
    return np.ascontiguousarray(grid.reshape(-1, 1, 2), dtype=np.float32)


def detect_board(
    gray: np.ndarray,
    board: Any,
    cols: int,
    rows: int,
) -> tuple[np.ndarray | None, int | None, str | None]:
    upscale = 3.0
    enlarged = cv2.resize(gray, None, fx=upscale, fy=upscale, interpolation=cv2.INTER_CUBIC)
    detector_parameters = cv2.aruco.DetectorParameters()
    detector_parameters.minMarkerPerimeterRate = 0.005
    detector_parameters.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
    detector = cv2.aruco.CharucoDetector(
        board,
        cv2.aruco.CharucoParameters(),
        detector_parameters,
    )
    charuco_corners, charuco_ids, _, marker_ids = detector.detectBoard(enlarged)
    expected_ids = np.arange(cols * rows, dtype=np.int32)
    if charuco_ids is not None and len(charuco_ids) == len(expected_ids):
        order = np.argsort(charuco_ids.reshape(-1))
        sorted_ids = charuco_ids.reshape(-1)[order]
        if np.array_equal(sorted_ids, expected_ids):
            corners = np.ascontiguousarray(
                charuco_corners.reshape(-1, 2)[order].reshape(-1, 1, 2) / upscale,
                dtype=np.float32,
            )
            cv2.cornerSubPix(
                gray,
                corners,
                (5, 5),
                (-1, -1),
                (cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_MAX_ITER, 50, 1e-4),
            )
            return corners, 0 if marker_ids is None else int(len(marker_ids)), "charuco"

    # 低分辨率画面上若 ArUco 解码失败，去除格内编码后仍可按完整棋盘回退提取。
    _, black = cv2.threshold(gray, 0, 255, cv2.THRESH_BINARY_INV | cv2.THRESH_OTSU)
    scale = gray.shape[1] / 1280.0
    kernels: list[int] = []
    for nominal in (9, 11, 13, 15, 7, 17):
        size = max(3, int(round(nominal * scale)))
        if size % 2 == 0:
            size += 1
        if size not in kernels:
            kernels.append(size)
    flags = cv2.CALIB_CB_NORMALIZE_IMAGE | cv2.CALIB_CB_EXHAUSTIVE | cv2.CALIB_CB_ACCURACY
    for size in kernels:
        kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (size, size))
        cleaned = 255 - cv2.morphologyEx(black, cv2.MORPH_OPEN, kernel)
        found, corners = cv2.findChessboardCornersSB(cleaned, (cols, rows), flags)
        if not found:
            continue
        corners = canonical_corners(corners, cols, rows)
        cv2.cornerSubPix(
            gray,
            corners,
            (5, 5),
            (-1, -1),
            (cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_MAX_ITER, 50, 1e-4),
        )
        return corners, size, "morphology_fallback"
    return None, None, None


def object_points(cols: int, rows: int, square_size: float = 1.0) -> np.ndarray:
    points = np.zeros((rows * cols, 3), dtype=np.float32)
    points[:, :2] = np.mgrid[0:cols, 0:rows].T.reshape(-1, 2)
    points *= float(square_size)
    return points


def calibrate_intrinsics(
    observations: list[dict[str, Any]],
    obj: np.ndarray,
    image_size: tuple[int, int],
) -> tuple[np.ndarray, np.ndarray, list[int], dict[str, float]]:
    def run(indices: list[int]) -> tuple[float, np.ndarray, np.ndarray, np.ndarray]:
        rms, matrix, distortion, rvecs, tvecs = cv2.calibrateCamera(
            [obj for _ in indices],
            [observations[index]["corners"] for index in indices],
            image_size,
            None,
            None,
        )
        errors = []
        for index, rvec, tvec in zip(indices, rvecs, tvecs):
            projected, _ = cv2.projectPoints(obj, rvec, tvec, matrix, distortion)
            residual = projected.reshape(-1, 2) - observations[index]["corners"].reshape(-1, 2)
            errors.append(float(np.sqrt(np.mean(np.sum(residual * residual, axis=1)))))
        return float(rms), matrix, distortion, np.asarray(errors, dtype=np.float64)

    indices = list(range(len(observations)))
    rms, matrix, distortion, errors = run(indices)
    limit = robust_limit(errors, floor=0.65)
    filtered = [index for index, error in zip(indices, errors) if error <= limit]
    if len(filtered) >= 12 and len(filtered) < len(indices):
        indices = filtered
        rms, matrix, distortion, errors = run(indices)
    metrics = {
        "rms_px": rms,
        "median_view_rmse_px": float(np.median(errors)),
        "max_view_rmse_px": float(np.max(errors)),
    }
    return matrix, distortion, indices, metrics


def sample_depth_mm(depth: np.ndarray, pixel: np.ndarray, radius: int = 3) -> float | None:
    x = int(round(float(pixel[0])))
    y = int(round(float(pixel[1])))
    x0, x1 = max(0, x - radius), min(depth.shape[1], x + radius + 1)
    y0, y1 = max(0, y - radius), min(depth.shape[0], y + radius + 1)
    values = np.asarray(depth[y0:y1, x0:x1], dtype=np.float64)
    valid = values[(values > 100.0) & (values < 5000.0) & np.isfinite(values)]
    if valid.size < 5:
        return None
    return float(np.median(valid))


def estimate_square_size_m(
    observations: list[dict[str, Any]],
    indices: list[int],
    camera_matrix: np.ndarray,
    distortion: np.ndarray,
    cols: int,
    rows: int,
) -> tuple[float, dict[str, float]]:
    lengths: list[float] = []
    for index in indices:
        observation = observations[index]
        depth = np.load(observation["depth_path"], allow_pickle=False)
        pixels = observation["corners"].reshape(-1, 2)
        normalized = cv2.undistortPoints(
            pixels.reshape(-1, 1, 2), camera_matrix, distortion
        ).reshape(-1, 2)
        points: list[np.ndarray | None] = []
        for pixel, ray in zip(pixels, normalized):
            depth_value = sample_depth_mm(depth, pixel)
            if depth_value is None:
                points.append(None)
            else:
                z_m = depth_value / 1000.0
                points.append(np.asarray([ray[0] * z_m, ray[1] * z_m, z_m], dtype=np.float64))
        grid = np.empty(len(points), dtype=object)
        grid[:] = points
        grid = grid.reshape(rows, cols)
        for row in range(rows):
            for col in range(cols - 1):
                a, b = grid[row, col], grid[row, col + 1]
                if a is not None and b is not None:
                    lengths.append(float(np.linalg.norm(a - b)))
        for row in range(rows - 1):
            for col in range(cols):
                a, b = grid[row, col], grid[row + 1, col]
                if a is not None and b is not None:
                    lengths.append(float(np.linalg.norm(a - b)))
    values = np.asarray(lengths, dtype=np.float64)
    if values.size < 100:
        raise RuntimeError("有效深度边长不足，无法自动估计标定板方格尺寸")
    median = float(np.median(values))
    mad = float(np.median(np.abs(values - median)))
    keep = np.abs(values - median) <= max(0.002, 3.5 * 1.4826 * mad)
    filtered = values[keep]
    estimate = float(np.median(filtered))
    return estimate, {
        "raw_edge_count": int(values.size),
        "used_edge_count": int(filtered.size),
        "median_mm": estimate * 1000.0,
        "mad_mm": float(np.median(np.abs(filtered - estimate))) * 1000.0,
    }


def rotation_mean(rotations: list[np.ndarray]) -> np.ndarray:
    total = np.sum(np.stack(rotations), axis=0)
    u, _, vt = np.linalg.svd(total)
    result = u @ vt
    if np.linalg.det(result) < 0.0:
        u[:, -1] *= -1.0
        result = u @ vt
    return result


def rotation_error_deg(a: np.ndarray, b: np.ndarray) -> float:
    relative = np.asarray(a).T @ np.asarray(b)
    cosine = float(np.clip((np.trace(relative) - 1.0) / 2.0, -1.0, 1.0))
    return math.degrees(math.acos(cosine))


def evaluate_handeye(
    transform_head_from_camera: np.ndarray,
    observations: list[dict[str, Any]],
    indices: list[int],
) -> tuple[dict[str, float], list[np.ndarray], np.ndarray]:
    base_targets = [
        observations[index]["base_from_head"]
        @ transform_head_from_camera
        @ observations[index]["camera_from_target"]
        for index in indices
    ]
    center_t = np.median(np.stack([item[:3, 3] for item in base_targets]), axis=0)
    center_r = rotation_mean([item[:3, :3] for item in base_targets])
    center = make_transform(center_r, center_t)
    translation_errors = np.asarray(
        [np.linalg.norm(item[:3, 3] - center_t) * 1000.0 for item in base_targets]
    )
    rotation_errors = np.asarray(
        [rotation_error_deg(center_r, item[:3, :3]) for item in base_targets]
    )
    metrics = {
        "target_translation_rms_mm": float(np.sqrt(np.mean(translation_errors * translation_errors))),
        "median_target_translation_error_mm": float(np.median(translation_errors)),
        "p95_target_translation_error_mm": float(np.percentile(translation_errors, 95)),
        "max_target_translation_error_mm": float(np.max(translation_errors)),
        "target_rotation_rms_deg": float(np.sqrt(np.mean(rotation_errors * rotation_errors))),
        "median_target_rotation_error_deg": float(np.median(rotation_errors)),
        "p95_target_rotation_error_deg": float(np.percentile(rotation_errors, 95)),
        "max_target_rotation_error_deg": float(np.max(rotation_errors)),
    }
    metrics["score"] = (
        metrics["median_target_translation_error_mm"]
        + metrics["p95_target_translation_error_mm"]
        + 2.0 * metrics["median_target_rotation_error_deg"]
        + 2.0 * metrics["p95_target_rotation_error_deg"]
    )
    residuals = [
        np.asarray([translation, rotation], dtype=np.float64)
        for translation, rotation in zip(translation_errors, rotation_errors)
    ]
    return metrics, residuals, center


def solve_handeye_methods(
    observations: list[dict[str, Any]],
    indices: list[int],
) -> dict[str, dict[str, Any]]:
    rotations_gripper_to_base = [observations[index]["base_from_head"][:3, :3] for index in indices]
    translations_gripper_to_base = [observations[index]["base_from_head"][:3, 3] for index in indices]
    rotations_target_to_camera = [observations[index]["camera_from_target"][:3, :3] for index in indices]
    translations_target_to_camera = [observations[index]["camera_from_target"][:3, 3] for index in indices]
    results: dict[str, dict[str, Any]] = {}
    for name, method in METHODS.items():
        try:
            rotation, translation = cv2.calibrateHandEye(
                rotations_gripper_to_base,
                translations_gripper_to_base,
                rotations_target_to_camera,
                translations_target_to_camera,
                method=method,
            )
            transform = make_transform(rotation, np.asarray(translation).reshape(3))
            if not np.isfinite(transform).all() or np.linalg.det(transform[:3, :3]) < 0.9:
                continue
            metrics, residuals, base_from_target = evaluate_handeye(
                transform, observations, indices
            )
            results[name] = {
                "transform": transform,
                "metrics": metrics,
                "residuals": residuals,
                "base_from_target": base_from_target,
            }
        except cv2.error:
            continue
    if not results:
        raise RuntimeError("所有 OpenCV 手眼标定方法都失败")
    return results


def main() -> None:
    args = parse_args()
    data_root = Path(args.data).expanduser().resolve()
    output_path = Path(args.output).expanduser().resolve()
    sample_dirs = sorted(path for path in data_root.iterdir() if path.is_dir())
    if args.square_size_mm <= 0.0 or args.marker_size_mm <= 0.0:
        raise ValueError("标定板方格和标记边长都必须大于零")
    if args.marker_size_mm >= args.square_size_mm:
        raise ValueError("ArUco 标记边长必须小于方格边长")
    if not hasattr(cv2.aruco, args.dictionary):
        raise ValueError(f"未知 ArUco 字典: {args.dictionary}")
    dictionary = cv2.aruco.getPredefinedDictionary(getattr(cv2.aruco, args.dictionary))
    board = cv2.aruco.CharucoBoard(
        (args.board_cols + 1, args.board_rows + 1),
        float(args.square_size_mm) / 1000.0,
        float(args.marker_size_mm) / 1000.0,
        dictionary,
    )
    board_points_exact = np.asarray(board.getChessboardCorners(), dtype=np.float32)
    board_points_unit = board_points_exact / (float(args.square_size_mm) / 1000.0)
    observations: list[dict[str, Any]] = []
    detection_failures: list[str] = []
    image_size: tuple[int, int] | None = None
    for sample_dir in sample_dirs:
        rgb_path = sample_dir / "head_rgb.jpg"
        depth_path = sample_dir / "head_depth_aligned.npy"
        state_path = sample_dir / "robot_state.json"
        if not (rgb_path.exists() and depth_path.exists() and state_path.exists()):
            detection_failures.append(f"{sample_dir.name}: 文件不完整")
            continue
        image = cv2.imread(str(rgb_path), cv2.IMREAD_GRAYSCALE)
        if image is None:
            detection_failures.append(f"{sample_dir.name}: RGB 读取失败")
            continue
        current_size = (int(image.shape[1]), int(image.shape[0]))
        if image_size is None:
            image_size = current_size
        elif current_size != image_size:
            detection_failures.append(f"{sample_dir.name}: RGB 尺寸不一致 {current_size}")
            continue
        corners, detector_detail, detector_method = detect_board(
            image,
            board,
            args.board_cols,
            args.board_rows,
        )
        if corners is None:
            detection_failures.append(f"{sample_dir.name}: 编码棋盘角点提取失败")
            continue
        state = json.loads(state_path.read_text(encoding="utf-8"))
        joint_values_deg = [
            *state["joints_deg"]["trunk"],
            *state["joints_deg"]["head"],
        ]
        observations.append(
            {
                "name": sample_dir.name,
                "rgb_path": str(rgb_path),
                "depth_path": str(depth_path),
                "state": state,
                "joints_deg": joint_values_deg,
                "corners": corners,
                "detector_method": detector_method,
                "detector_detail": detector_detail,
            }
        )
    if image_size is None or len(observations) < 12:
        raise RuntimeError(f"有效样本不足: {len(observations)}，失败: {detection_failures}")

    kinematics = UpperBodySixDofKinematics(args.urdf)
    fk_translation_errors_mm: list[float] = []
    fk_rotation_errors_deg: list[float] = []
    for observation in observations:
        base_from_head = kinematics.forward_deg(observation["joints_deg"])
        base_from_chest = kinematics.forward_deg(observation["joints_deg"], tip_link="Chest_link")
        trunk_pose = observation["state"]["poses"]["trunk"]
        recorded_translation = np.asarray(trunk_pose[:3], dtype=np.float64) / 1000.0
        recorded_rpy = np.asarray(trunk_pose[3:], dtype=np.float64)
        fk_translation_errors_mm.append(
            float(np.linalg.norm(base_from_chest[:3, 3] - recorded_translation) * 1000.0)
        )
        fk_rotation_errors_deg.append(
            float(np.linalg.norm(np.asarray(rotation_to_rpy_deg(base_from_chest[:3, :3])) - recorded_rpy))
        )
        observation["base_from_head"] = base_from_head

    camera_matrix, distortion, intrinsic_indices, intrinsic_metrics = calibrate_intrinsics(
        observations,
        board_points_unit,
        image_size,
    )
    measured_square_size_m, depth_scale_metrics = estimate_square_size_m(
        observations,
        intrinsic_indices,
        camera_matrix,
        distortion,
        args.board_cols,
        args.board_rows,
    )
    square_size_m = float(args.square_size_mm) / 1000.0
    depth_scale_metrics["specified_mm"] = float(args.square_size_mm)
    depth_scale_metrics["difference_mm"] = (
        measured_square_size_m - square_size_m
    ) * 1000.0
    square_size_source = "board_specification"
    board_points = board_points_exact
    pnp_errors: list[float] = []
    usable_indices: list[int] = []
    for index in intrinsic_indices:
        observation = observations[index]
        ok, rvec, tvec = cv2.solvePnP(
            board_points,
            observation["corners"],
            camera_matrix,
            distortion,
            flags=cv2.SOLVEPNP_ITERATIVE,
        )
        if not ok:
            continue
        rotation, _ = cv2.Rodrigues(rvec)
        camera_from_target = make_transform(rotation, np.asarray(tvec).reshape(3))
        if camera_from_target[2, 3] <= 0.0:
            continue
        projected, _ = cv2.projectPoints(
            board_points, rvec, tvec, camera_matrix, distortion
        )
        residual = projected.reshape(-1, 2) - observation["corners"].reshape(-1, 2)
        pnp_error = float(np.sqrt(np.mean(np.sum(residual * residual, axis=1))))
        observation["camera_from_target"] = camera_from_target
        observation["pnp_rmse_px"] = pnp_error
        usable_indices.append(index)
        pnp_errors.append(pnp_error)
    pnp_limit = robust_limit(np.asarray(pnp_errors), floor=0.8)
    usable_indices = [
        index for index in usable_indices if observations[index]["pnp_rmse_px"] <= pnp_limit
    ]
    if len(usable_indices) < 10:
        raise RuntimeError(f"PnP 后有效手眼样本不足: {len(usable_indices)}")

    initial_results = solve_handeye_methods(observations, usable_indices)
    initial_best_name = min(initial_results, key=lambda name: initial_results[name]["metrics"]["score"])
    initial_residuals = np.stack(initial_results[initial_best_name]["residuals"])
    translation_limit = robust_limit(initial_residuals[:, 0], floor=5.0)
    rotation_limit = robust_limit(initial_residuals[:, 1], floor=0.5)
    final_indices = [
        index
        for index, residual in zip(usable_indices, initial_residuals)
        if residual[0] <= translation_limit and residual[1] <= rotation_limit
    ]
    if len(final_indices) < 10:
        final_indices = usable_indices
    final_results = solve_handeye_methods(observations, final_indices)
    best_name = min(final_results, key=lambda name: final_results[name]["metrics"]["score"])
    best = final_results[best_name]
    transform_head_from_camera = best["transform"]
    transform_camera_from_head = invert_transform(transform_head_from_camera)
    used_names = [observations[index]["name"] for index in final_indices]
    rejected_names = sorted(set(item["name"] for item in observations) - set(used_names))

    first_metadata = json.loads(
        (data_root / observations[0]["name"] / "head_camera_metadata.json").read_text(encoding="utf-8")
    )
    result = {
        "schema": "rokae.head_camera_handeye.v1",
        "created_at": datetime.now().astimezone().isoformat(timespec="seconds"),
        "calibration_type": "eye_in_hand",
        "base_frame": "chassis_link",
        "gripper_frame": "Head_link",
        "camera_frame": "head_camera_color_optical_frame",
        "target_frame": "charuco_board_outer_corner",
        "camera": first_metadata["camera"],
        "dataset": str(data_root),
        "urdf": {
            "source": kinematics.urdf_source,
            "joint_order": list(kinematics.joint_names),
            "recorded_joint_order": ["trunk.J1", "trunk.J2", "trunk.J3", "trunk.J4", "head.O1", "head.O2"],
            "fk_validation": {
                "max_chest_translation_error_mm": float(np.max(fk_translation_errors_mm)),
                "max_chest_rpy_error_deg": float(np.max(fk_rotation_errors_deg)),
            },
        },
        "board": {
            "squares": [args.board_cols + 1, args.board_rows + 1],
            "inner_corners": [args.board_cols, args.board_rows],
            "dictionary": args.dictionary,
            "square_size_m": square_size_m,
            "square_size_mm": square_size_m * 1000.0,
            "marker_size_m": float(args.marker_size_mm) / 1000.0,
            "marker_size_mm": float(args.marker_size_mm),
            "square_size_source": square_size_source,
            "depth_scale_estimation": depth_scale_metrics,
            "corner_extraction": "3x upsample + OpenCV ChArUco IDs; morphology chessboard fallback",
        },
        "intrinsics": {
            "image_size": list(image_size),
            "camera_matrix": np.round(camera_matrix, 12).tolist(),
            "distortion_coefficients": np.round(distortion.reshape(-1), 12).tolist(),
            **intrinsic_metrics,
        },
        "samples": {
            "found_directories": len(sample_dirs),
            "board_detected": len(observations),
            "used": len(final_indices),
            "used_names": used_names,
            "rejected_names": rejected_names,
            "detection_failures": detection_failures,
            "pnp_median_rmse_px": float(np.median([observations[index]["pnp_rmse_px"] for index in final_indices])),
            "pnp_max_rmse_px": float(np.max([observations[index]["pnp_rmse_px"] for index in final_indices])),
        },
        "selected_method": best_name,
        "method_metrics": {
            name: values["metrics"] for name, values in final_results.items()
        },
        "transform_head_link_from_camera_optical": transform_dict(transform_head_from_camera),
        "transform_camera_optical_from_head_link": transform_dict(transform_camera_from_head),
        "estimated_transform_chassis_from_target": transform_dict(best["base_from_target"]),
        "usage": {
            "point_camera_to_chassis": "p_chassis = T_chassis_head(q) @ T_head_camera @ [x_camera,y_camera,z_camera,1]",
            "warning": "该矩阵只适用于本次固定安装位置；相机支架发生位移或拆装后必须重新标定。",
        },
    }
    output_path.parent.mkdir(parents=True, exist_ok=True)
    output_path.write_text(json.dumps(result, ensure_ascii=False, indent=2), encoding="utf-8")

    print(f"样本目录: {len(sample_dirs)}")
    print(f"棋盘检测: {len(observations)}/{len(sample_dirs)}")
    print(f"最终使用: {len(final_indices)}")
    print(f"内参 RMS: {intrinsic_metrics['rms_px']:.4f} px")
    print(
        f"方格边长: {square_size_m * 1000.0:.3f} mm ({square_size_source}); "
        f"对齐深度交叉验证 {measured_square_size_m * 1000.0:.3f} mm"
    )
    print(f"手眼方法: {best_name}")
    print(
        "目标一致性: "
        f"平移中位 {best['metrics']['median_target_translation_error_mm']:.3f} mm, "
        f"P95 {best['metrics']['p95_target_translation_error_mm']:.3f} mm; "
        f"旋转中位 {best['metrics']['median_target_rotation_error_deg']:.4f}°, "
        f"P95 {best['metrics']['p95_target_rotation_error_deg']:.4f}°"
    )
    print("T_HeadLink_CameraOptical =")
    print(np.array2string(transform_head_from_camera, precision=9, suppress_small=True))
    print(f"结果: {output_path}")


if __name__ == "__main__":
    main()
