from __future__ import annotations

import base64
import binascii
import io
import json
import math
import shutil
import time
import urllib.error
import urllib.request
import uuid
from datetime import datetime
from pathlib import Path
from typing import Any

import numpy as np

from .backends import BackendError
from .camera import CameraSnapshot
from .head_kinematics import UpperBodySixDofKinematics
from .grasp_pose import world_grasp_from_4090, world_grasp_to_right_shoulder, world_grasp_to_left_shoulder
from .box_clearance import front_panel_from_response, clearance_in_trunk
from .pose_targets import interpret, basket_right_shoulder, basket_left_shoulder, LABELS
from .pose_protocol import sku_type, historical_type, shared_grasp_height, validate_response_type
from .trunk_frame import stable_trunk_reference, TRUNK_FRAME
from .action_poses import box_heights, tube_height


class PoseEstimationError(RuntimeError):
    pass


class PoseEstimationClient:
    """Prepare synchronized head RGB-D requests and call the 位姿估计 locator."""

    CAMERA_FRAME = "head_camera_color_optical_frame"
    BASE_FRAME = "chassis_link"

    def __init__(self, config: dict[str, Any]) -> None:
        self.config = dict(config["pose_estimation"])
        self.data_root = Path(config["camera"]["data_directory"]).expanduser().resolve()
        self._calibration: dict[str, Any] | None = None
        self._kinematics: UpperBodySixDofKinematics | None = None

    @property
    def fresh_frame_timeout(self) -> float:
        return float(self.config["fresh_frame_timeout_seconds"])

    def _request_json(
        self,
        url: str,
        *,
        method: str,
        payload: dict[str, Any] | None,
        timeout: float,
    ) -> dict[str, Any]:
        data = None
        headers = {"Accept": "application/json"}
        if payload is not None:
            data = json.dumps(payload, ensure_ascii=False, separators=(",", ":")).encode(
                "utf-8"
            )
            headers["Content-Type"] = "application/json"
        request = urllib.request.Request(url, data=data, headers=headers, method=method)
        try:
            with urllib.request.urlopen(request, timeout=timeout) as response:
                raw = response.read()
        except urllib.error.HTTPError as exc:
            raw = exc.read()
            detail = raw.decode("utf-8", errors="replace")[:1000]
            try:
                body = json.loads(detail)
                detail = str(body.get("error") or body.get("detail") or detail)
            except json.JSONDecodeError:
                pass
            raise PoseEstimationError(
                f"位姿估计 定位服务返回 HTTP {exc.code}: {detail}"
            ) from exc
        except (urllib.error.URLError, TimeoutError, OSError) as exc:
            reason = getattr(exc, "reason", exc)
            raise PoseEstimationError(f"无法连接 位姿估计 定位服务: {reason}") from exc
        try:
            decoded = json.loads(raw.decode("utf-8"))
        except (UnicodeDecodeError, json.JSONDecodeError) as exc:
            raise PoseEstimationError("位姿估计 定位服务返回了非 JSON 数据") from exc
        if not isinstance(decoded, dict):
            raise PoseEstimationError("位姿估计 定位服务返回值不是 JSON 对象")
        return decoded

    def health(self, target=None) -> dict[str, Any]:
        url = self.config["service_base_url"].rstrip("/") + "/health"
        try:
            response = self._request_json(
                url,
                method="GET",
                payload=None,
                timeout=float(self.config["health_timeout_seconds"]),
            )
            available = response.get("ok") is True
            configured = target or self.config.get("sku_typ", "bottle")
            class_name = 'basket' if configured == 'basket' else sku_type(configured)
            classes = response.get("supported_sku_types")
            class_supported = class_name == 'basket' or (isinstance(classes, list) and class_name in classes)
            if available and not class_supported:
                return {
                    "available": False,
                    "error": f"位姿估计 supported_sku_types 未声明支持类别 {class_name}",
                    "response": response,
                }
            return {"available": available, "response": response}
        except (PoseEstimationError, ValueError) as exc:
            return {"available": False, "error": str(exc)}

    def _load_calibration(self) -> tuple[np.ndarray, np.ndarray, list[int]]:
        if self._calibration is None or self._kinematics is None:
            path = Path(self.config["calibration_file"]).expanduser().resolve()
            try:
                calibration = json.loads(path.read_text(encoding="utf-8"))
            except (OSError, json.JSONDecodeError) as exc:
                raise PoseEstimationError(f"读取头部相机标定失败: {exc}") from exc
            if calibration.get("camera_frame") != self.CAMERA_FRAME:
                raise PoseEstimationError("头部相机标定的 camera_frame 与接口约定不一致")
            if calibration.get("base_frame") != self.BASE_FRAME:
                raise PoseEstimationError("头部相机标定的 base_frame 与接口约定不一致")
            try:
                camera_matrix = np.asarray(
                    calibration["intrinsics"]["camera_matrix"], dtype=np.float64
                ).reshape(3, 3)
                transform = np.asarray(
                    calibration["transform_head_link_from_camera_optical"]["matrix_4x4"],
                    dtype=np.float64,
                ).reshape(4, 4)
                image_size = [int(value) for value in calibration["intrinsics"]["image_size"]]
            except (KeyError, TypeError, ValueError) as exc:
                raise PoseEstimationError("头部相机标定文件缺少内参或手眼矩阵") from exc
            if not np.isfinite(camera_matrix).all() or not np.isfinite(transform).all():
                raise PoseEstimationError("头部相机标定包含非有限数值")
            self._calibration = {
                "camera_matrix": camera_matrix,
                "transform_head_camera": transform,
                "image_size": image_size,
            }
            try:
                self._kinematics = UpperBodySixDofKinematics(self.config["urdf_file"])
            except Exception as exc:
                self._calibration = None
                raise PoseEstimationError(f"加载上身 URDF 运动学失败: {exc}") from exc
        return (
            self._calibration["camera_matrix"],
            self._calibration["transform_head_camera"],
            list(self._calibration["image_size"]),
        )

    @staticmethod
    def _upper_body_joints(state: dict[str, Any]) -> np.ndarray:
        try:
            values = list(state["joints_deg"]["trunk"]) + list(
                state["joints_deg"]["head"]
            )
            joints = np.asarray(values, dtype=np.float64)
        except (KeyError, TypeError, ValueError) as exc:
            raise PoseEstimationError("机器人状态缺少躯干或头部关节角") from exc
        if joints.shape != (6,) or not np.isfinite(joints).all():
            raise PoseEstimationError("躯干和头部关节状态无效")
        return joints

    def _synchronized_transform(
        self,
        state_before: dict[str, Any],
        state_after: dict[str, Any],
        transform_head_camera: np.ndarray,
    ) -> tuple[np.ndarray, list[float], float]:
        before = self._upper_body_joints(state_before)
        after = self._upper_body_joints(state_after)
        max_delta = float(np.max(np.abs(after - before)))
        tolerance = float(self.config["stationary_tolerance_deg"])
        if max_delta > tolerance:
            raise PoseEstimationError(
                f"采集期间躯干或头部发生运动（最大变化 {max_delta:.4f}°，允许 {tolerance:.4f}°）"
            )
        active_tokens = ("moving", "drag", "jog", "execution", "rtcontrolling")
        operation_states = state_after.get("operation_state", {})
        if isinstance(operation_states, dict):
            active = [
                f"{name}={value}"
                for name, value in operation_states.items()
                if any(token in str(value).lower() for token in active_tokens)
            ]
            if active:
                raise PoseEstimationError("采集时机器人仍在运动: " + ", ".join(active))
        joints_deg = ((before + after) / 2.0).tolist()
        assert self._kinematics is not None
        transform = self._kinematics.camera_to_base(joints_deg, transform_head_camera)
        rotation = transform[:3, :3]
        if not np.allclose(rotation.T @ rotation, np.eye(3), atol=1e-6):
            raise PoseEstimationError("计算得到的相机外参旋转矩阵不正交")
        if not math.isclose(float(np.linalg.det(rotation)), 1.0, abs_tol=1e-6):
            raise PoseEstimationError("计算得到的相机外参旋转矩阵行列式异常")
        return transform, [float(value) for value in joints_deg], max_delta

    @staticmethod
    def _encode_snapshot(snapshot: CameraSnapshot, jpeg_quality: int) -> tuple[bytes, bytes]:
        import cv2

        rgb = np.asarray(snapshot.rgb_bgr)
        depth = np.asarray(snapshot.depth_aligned_mm)
        if rgb.ndim != 3 or rgb.shape[2] != 3:
            raise PoseEstimationError("头部 RGB 图像尺寸无效")
        if depth.ndim != 2 or depth.shape != rgb.shape[:2]:
            raise PoseEstimationError("头部深度没有与 RGB 对齐")
        ok, encoded = cv2.imencode(
            ".jpg", rgb, [cv2.IMWRITE_JPEG_QUALITY, int(jpeg_quality)]
        )
        if not ok:
            raise PoseEstimationError("编码头部 RGB 图像失败")
        depth_buffer = io.BytesIO()
        np.save(
            depth_buffer,
            depth.astype(np.float32, copy=False),
            allow_pickle=False,
        )
        return encoded.tobytes(), depth_buffer.getvalue()

    @staticmethod
    def _project(point_mm: Any, camera_matrix: np.ndarray) -> tuple[int, int] | None:
        try:
            point = np.asarray(point_mm, dtype=np.float64).reshape(3)
        except (TypeError, ValueError):
            return None
        if not np.isfinite(point).all() or point[2] <= 1e-6:
            return None
        pixel = camera_matrix @ point
        return int(round(float(pixel[0] / pixel[2]))), int(
            round(float(pixel[1] / pixel[2]))
        )

    def _make_overlay(
        self,
        snapshot: CameraSnapshot,
        camera_matrix: np.ndarray | None,
        response: dict[str, Any] | None,
        status: str,
        message: str,
    ) -> bytes:
        import cv2

        image = np.asarray(snapshot.rgb_bgr).copy()
        response = response or {}
        artifacts = response.get("artifacts")
        if isinstance(artifacts, dict):
            for key in (
                "selected_overlay_jpeg_base64",
                "all_instances_overlay_jpeg_base64",
            ):
                encoded_overlay = artifacts.get(key)
                if not isinstance(encoded_overlay, str) or not encoded_overlay:
                    continue
                try:
                    decoded_bytes = base64.b64decode(encoded_overlay, validate=True)
                    decoded_image = cv2.imdecode(
                        np.frombuffer(decoded_bytes, dtype=np.uint8), cv2.IMREAD_COLOR
                    )
                except (binascii.Error, ValueError, TypeError):
                    decoded_image = None
                if decoded_image is not None and decoded_image.shape == image.shape:
                    image = decoded_image
                    break
        height, width = image.shape[:2]
        usable = status == "success"
        color = (70, 210, 120) if usable else (60, 100, 255)
        point = next((response.get(key) for key in ('model_center_camera_mm', 'top_point_camera_mm',
                     'top_edge_center_camera_mm', 'reference_point_camera_mm') if response.get(key) is not None), None)
        if point is None:
            point = response.get("axis_point_camera_mm")
        axis = response.get("axis_direction_camera_up")
        if camera_matrix is not None and point is not None:
            start = self._project(point, camera_matrix)
            if start is not None and 0 <= start[0] < width and 0 <= start[1] < height:
                cv2.circle(image, start, 10, color, 3, cv2.LINE_AA)
                if axis is not None:
                    try:
                        end_point = np.asarray(point, dtype=np.float64) + float(
                            self.config["axis_overlay_length_mm"]
                        ) * np.asarray(axis, dtype=np.float64)
                        end = self._project(end_point, camera_matrix)
                    except (TypeError, ValueError):
                        end = None
                    if end is not None:
                        cv2.arrowedLine(image, start, end, color, 4, cv2.LINE_AA, tipLength=0.18)
        cv2.rectangle(image, (0, 0), (width, 76), (5, 14, 18), thickness=-1)
        title = "POSE VALID" if usable else "POSE INVALID"
        cv2.putText(image, title, (18, 31), cv2.FONT_HERSHEY_SIMPLEX, 0.82, color, 2, cv2.LINE_AA)
        safe_message = str(message).replace("\n", " ")[:95]
        cv2.putText(
            image,
            safe_message,
            (18, 60),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.48,
            (220, 230, 232),
            1,
            cv2.LINE_AA,
        )
        ok, encoded = cv2.imencode(".jpg", image, [cv2.IMWRITE_JPEG_QUALITY, 95])
        if not ok:
            raise BackendError("保存位姿估计标注图失败")
        return encoded.tobytes()

    @staticmethod
    def _response_for_web(response: dict[str, Any] | None) -> dict[str, Any] | None:
        if response is None:
            return None
        result = dict(response)
        artifacts = response.get("artifacts")
        if isinstance(artifacts, dict):
            result["artifacts"] = {
                key: value
                for key, value in artifacts.items()
                if not key.endswith("_base64")
            }
            result["artifact_payloads"] = {
                "selected_overlay_jpeg": bool(
                    artifacts.get("selected_overlay_jpeg_base64")
                ),
                "all_instances_overlay_jpeg": bool(
                    artifacts.get("all_instances_overlay_jpeg_base64")
                ),
                "point_cloud_ply": bool(artifacts.get("point_cloud_ply_base64")),
            }
        return result

    def _store_result(
        self,
        *,
        snapshot: CameraSnapshot,
        rgb_jpeg: bytes,
        depth_npy: bytes,
        state_before: dict[str, Any],
        state_after: dict[str, Any],
        request_summary: dict[str, Any],
        response: dict[str, Any] | None,
        status: str,
        message: str,
        camera_matrix: np.ndarray | None,
        world_grasp: dict[str, Any] | None,
        box_clearance: dict[str, Any],
    ) -> dict[str, Any]:
        recorded_at = datetime.now().astimezone()
        day = recorded_at.strftime("%Y%m%d")
        leaf = (
            "pose_"
            + recorded_at.strftime("%H%M%S")
            + f"{recorded_at.microsecond // 1000:03d}_{uuid.uuid4().hex[:6]}"
        )
        day_dir = self.data_root / day
        target = day_dir / leaf
        temporary = day_dir / f".{leaf}.tmp"
        day_dir.mkdir(parents=True, exist_ok=True)
        temporary.mkdir()
        try:
            (temporary / "head_rgb.jpg").write_bytes(rgb_jpeg)
            (temporary / "head_depth_aligned.npy").write_bytes(depth_npy)
            (temporary / "robot_state_before.json").write_text(
                json.dumps(state_before, ensure_ascii=False, indent=2), encoding="utf-8"
            )
            (temporary / "robot_state_after.json").write_text(
                json.dumps(state_after, ensure_ascii=False, indent=2), encoding="utf-8"
            )
            (temporary / "pose_estimation_request.json").write_text(
                json.dumps(request_summary, ensure_ascii=False, indent=2), encoding="utf-8"
            )
            stored_response = response or {}
            (temporary / "pose_estimation_response.json").write_text(
                json.dumps(stored_response, ensure_ascii=False, indent=2), encoding="utf-8"
            )
            summary = {
                "recorded_at": recorded_at.isoformat(timespec="milliseconds"),
                "camera_frame_captured_at": snapshot.captured_at,
                "camera_sequence": snapshot.sequence,
                "selected_target": request_summary.get('selected_target'),
                "localization": request_summary.get('localization'),
                "status": status,
                "message": message,
                "usable": status == "success",
                "world_grasp": world_grasp,
                "box_clearance": box_clearance,
                "grasp_test_usable": status == "success" and box_clearance.get("valid") is True,
                "files": {
                    "rgb": "head_rgb.jpg",
                    "depth": "head_depth_aligned.npy",
                    "overlay": "pose_estimation_overlay.jpg",
                    "request": "pose_estimation_request.json",
                    "response": "pose_estimation_response.json",
                    "state_before": "robot_state_before.json",
                    "state_after": "robot_state_after.json",
                },
            }
            (temporary / "pose_estimation_summary.json").write_text(
                json.dumps(summary, ensure_ascii=False, indent=2), encoding="utf-8"
            )
            overlay = self._make_overlay(
                snapshot, camera_matrix, response, status, message
            )
            (temporary / "pose_estimation_overlay.jpg").write_bytes(overlay)
            temporary.replace(target)
        except Exception:
            shutil.rmtree(temporary, ignore_errors=True)
            raise
        result_id = f"{day}/{leaf}"
        return {
            "result_id": result_id,
            "directory": str(target),
            "relative_directory": f"{self.data_root.name}/{result_id}",
            "image_url": f"/api/pose-estimation/results/{day}/{leaf}/image",
        }

    def estimate(
        self,
        snapshot: CameraSnapshot,
        state_before: dict[str, Any],
        state_after: dict[str, Any],
        selected=None,
    ) -> dict[str, Any]:
        started_at = time.monotonic()
        rgb_jpeg, depth_npy = self._encode_snapshot(
            snapshot, int(self.config["jpeg_quality"])
        )
        camera_matrix: np.ndarray | None = None
        request_summary: dict[str, Any] = {
            "camera_frame_captured_at": snapshot.captured_at,
            "camera_sequence": snapshot.sequence,
            "color_timestamp_ms": snapshot.color_timestamp_ms,
            "depth_timestamp_ms": snapshot.depth_timestamp_ms,
            "rgb_file": "head_rgb.jpg",
            "depth_file": "head_depth_aligned.npy",
            "depth_unit": "mm",
            "rgb_byte_count": len(rgb_jpeg),
            "depth_npy_byte_count": len(depth_npy),
        }
        if selected:
            request_summary['selected_target'] = selected['target']
        localization = None
        response: dict[str, Any] | None = None
        world_grasp: dict[str, Any] | None = None
        box_clearance = {"valid": False, "error": "尚无有效商品及前挡板平面"}
        status = "input_invalid"
        message = "定位输入无效"
        max_joint_delta_deg: float | None = None
        joints_deg: list[float] | None = None
        try:
            camera_matrix, transform_head_camera, image_size = self._load_calibration()
            height, width = np.asarray(snapshot.rgb_bgr).shape[:2]
            if image_size != [width, height]:
                raise PoseEstimationError(
                    f"当前 RGB 尺寸 {width}x{height} 与标定尺寸 {image_size[0]}x{image_size[1]} 不一致"
                )
            if snapshot.camera_info.get("source") in ("ros2", "http8085"):
                # Keep the existing hand-eye extrinsics; use the subscribed color
                # CameraInfo for the actual ROS driver image rather than SDK K.
                try:
                    intrinsics = snapshot.camera_info["color_intrinsics"]
                    current_matrix = np.asarray(intrinsics["camera_matrix"], dtype=np.float64)
                    if ([intrinsics["width"], intrinsics["height"]] != [width, height]
                            or current_matrix.shape != (3, 3)
                            or not np.isfinite(current_matrix).all()
                            or current_matrix[0, 0] <= 0 or current_matrix[1, 1] <= 0):
                        raise ValueError("invalid ROS color intrinsics")
                    camera_matrix = current_matrix
                except (KeyError, TypeError, ValueError) as exc:
                    raise PoseEstimationError("ROS2 当前 RGB 内参无效") from exc
                request_summary["intrinsics_source"] = "ros2_color_camera_info"
            transform, joints_deg, max_joint_delta_deg = self._synchronized_transform(
                state_before, state_after, transform_head_camera
            )
            side = selected["side"] if selected else str(self.config.get("side", "RIGHT")).upper()
            if side not in ("LEFT", "RIGHT"):
                raise PoseEstimationError("选箱 side 必须为 LEFT 或 RIGHT")
            front_rule = dict(self.config["front_rule"])
            try:
                trunk_ref = stable_trunk_reference(state_before, state_after, self._kinematics)
            except ValueError as exc:
                raise PoseEstimationError(str(exc)) from exc
            request_summary["T_chassis_trunk_ref_m"] = trunk_ref.tolist()
            request_summary["grasp_planning_frame"] = TRUNK_FRAME
            if self.config.get("front_axis_source", "trunk_sdk_x") == "trunk_sdk_x":
                axis = trunk_ref[:3, 0].copy()
                axis[2] = 0.0
                if np.linalg.norm(axis) < 1e-6:
                    raise PoseEstimationError("躯干 SDK X 轴没有有效的水平前向分量")
                front_rule["front_axis_chassis"] = (axis / np.linalg.norm(axis)).tolist()
            else:
                raise PoseEstimationError("抓取前向轴配置须为 trunk_sdk_x，请更新配置后重启")
            request_payload = {
                "rgb_base64": base64.b64encode(rgb_jpeg).decode("ascii"),
                "depth_npy_base64": base64.b64encode(depth_npy).decode("ascii"),
                "depth_unit": "mm",
                "K": camera_matrix.tolist(),
                "T_chassis_camera": transform.tolist(),
                "T_unit": "m",
                "camera_frame": self.CAMERA_FRAME,
                "base_frame": self.BASE_FRAME,
                "target_type": "sku",
                "sku_typ": sku_type(self.config.get("sku_typ", "bottle")),
                "side": side,
                "body_radius_mm": float(self.config["body_radius_mm"]),
                "fit_mode": str(self.config["fit_mode"]),
                "front_rule": front_rule,
                # Keep the existing web overlay/point-cloud diagnostic features
                # after the upstream default changed to paths-only responses.
                "return_visualizations": bool(self.config.get("return_visualizations", True)),
            }
            if selected:
                target = selected['target']
                for key in ('sku_typ', 'side', 'body_radius_mm', 'fit_mode', 'front_rule'):
                    request_payload.pop(key, None)
                request_payload['target_type'] = 'basket' if target == 'basket' else 'sku'
                if target != 'basket':
                    request_payload.update(sku_typ=sku_type(target), side=side, front_rule=front_rule)
                request_summary['selected_target'] = target
            request_summary.update(
                {
                    key: value
                    for key, value in request_payload.items()
                    if key not in ("rgb_base64", "depth_npy_base64")
                }
            )
            request_summary["upper_body_joints_deg"] = joints_deg
            request_summary["max_joint_delta_deg"] = max_joint_delta_deg
            request_summary["front_axis_source"] = "trunk_sdk_x"

            if selected is None:
                request_summary = self._height_request(request_summary, trunk_ref)
            health = self.health(selected['target']) if selected else self.health()
            request_summary["health"] = health
            if not health.get("available"):
                raise PoseEstimationError(
                    str(health.get("error") or "位姿估计 定位服务健康检查失败")
                )
            infer_url = self.config["service_base_url"].rstrip("/") + "/infer"
            response = self._request_json(
                infer_url,
                method="POST",
                payload=request_payload,
                timeout=float(self.config["request_timeout_seconds"]),
            )
            if selected is None:
                try:
                    validate_response_type(response, request_payload['sku_typ'])
                except ValueError as exc:
                    raise PoseEstimationError(str(exc)) from exc
            if selected:
                localization = interpret(response, selected['target'], transform)
                if localization['valid']:
                    status, message = 'success', LABELS[selected['target']] + '定位成功'
                    if selected['target'] == 'basket':
                        try:
                            shoulder = self._kinematics.right_shoulder_sdk_world(joints_deg[:4])
                            localization.update(basket_right_shoulder(localization, shoulder))
                        except ValueError as exc:
                            localization['right_shoulder_error'] = str(exc)
                            message += '；右肩参考点不可用：' + str(exc)
                        try:
                            shoulder = self._kinematics.left_shoulder_sdk_world(joints_deg[:4])
                            localization.update(basket_left_shoulder(localization, shoulder))
                        except ValueError as exc:
                            localization['left_shoulder_error'] = str(exc)
                            message += '；左肩参考点不可用：' + str(exc)
                    if selected['target'] in ('bottle', 'box', 'tube'):
                        # Assign trunk Z+ through the category's recognized point;
                        # both arms intersect the same calibrated height plane.
                        try:
                            request_summary = self._height_request(request_summary, trunk_ref)
                            world_grasp = world_grasp_from_4090(request_summary, response)
                            panel = front_panel_from_response(request_summary, response)
                            box_clearance = clearance_in_trunk(panel, world_grasp, trunk_ref)
                        except (ValueError, TypeError, KeyError, PoseEstimationError) as exc:
                            box_clearance = {'valid': False, 'error': str(exc)}
                            message += '；原抓取规划不可用：' + str(exc)
                else:
                    status, message = 'invalid', LABELS[selected['target']] + '：' + localization['error']
            if selected is None:
                try:
                    world_grasp = world_grasp_from_4090(request_summary, response)
                except (ValueError, TypeError, KeyError) as exc:
                    status = "invalid"
                    message = f"定位返回数据无法计算世界抓取点：{exc}"
                else:
                    status = "success"
                    message = "被抓取物品定位及世界抓取点计算成功"
                    try:
                        panel = front_panel_from_response(request_summary, response)
                        box_clearance = clearance_in_trunk(panel, world_grasp, trunk_ref)
                    except (TypeError, ValueError, KeyError) as exc:
                        box_clearance = {"valid": False, "error": str(exc)}
                        message += f"；抓取测试不可用：{exc}"
        except PoseEstimationError as exc:
            message = str(exc)
            status = (
                "connection_error"
                if "位姿估计" in message or "连接" in message or "健康检查" in message
                else "input_invalid"
            )

        if selected:
            request_summary['localization'] = localization
        request_summary["elapsed_seconds_before_save"] = round(
            time.monotonic() - started_at, 3
        )
        stored = self._store_result(
            snapshot=snapshot,
            rgb_jpeg=rgb_jpeg,
            depth_npy=depth_npy,
            state_before=state_before,
            state_after=state_after,
            request_summary=request_summary,
            response=response,
            status=status,
            message=message,
            camera_matrix=camera_matrix,
            world_grasp=world_grasp,
            box_clearance=box_clearance,
        )
        return {
            "status": status,
            "usable": status == "success",
            "message": message,
            "response": self._response_for_web(response),
            "selected_target": selected['target'] if selected else None,
            "localization": localization,
            "world_grasp": world_grasp,
            "box_clearance": box_clearance,
            "grasp_test_usable": status == "success" and box_clearance.get("valid") is True,
            "capture": {
                "camera_frame_captured_at": snapshot.captured_at,
                "camera_sequence": snapshot.sequence,
                "upper_body_joints_deg": joints_deg,
                "max_joint_delta_deg": max_joint_delta_deg,
            },
            "elapsed_seconds": round(time.monotonic() - started_at, 3),
            **stored,
        }

    def _height_request(self, request, trunk_ref=None):
        request = dict(request)
        try:
            sku = historical_type(request.get('sku_typ', request.get('sku_id', request.get('class_name'))))
        except ValueError as exc:
            raise PoseEstimationError(str(exc)) from exc
        if sku not in ('bottle', 'box', 'tube'):
            raise PoseEstimationError('当前仅支持 bottle、box 和 tube 抓取')
        if sku == 'box':
            calibration = box_heights(self.config)
            request['box_height_calibration'] = calibration
            height = calibration['heights_mm']['grasp']
        elif sku == 'tube':
            try:
                calibration = tube_height(self.config)
            except BackendError as exc:
                raise PoseEstimationError(str(exc)) from exc
            request['tube_height_calibration'] = calibration
            height = calibration['pregrasp_flange_height_mm']
        else:
            height = shared_grasp_height(self.config)
        if isinstance(height, bool) or not isinstance(height, (int, float)) or not math.isfinite(height):
            raise PoseEstimationError(f"商品 {sku} 尚未设置有效的躯干 SDK 固定抓取高度")
        request["grasp_height_trunk_mm"] = float(height)
        request["grasp_height_frame"] = TRUNK_FRAME
        request["grasp_height_sku_typ"] = sku
        request["sku_typ"] = sku
        if trunk_ref is not None:
            request["T_chassis_trunk_ref_m"] = np.asarray(trunk_ref).tolist()
        return request

    def latest_world_grasp(self, trunk_ref=None, expected_type="bottle") -> dict[str, Any]:
        """Load the newest saved usable world result, including after a restart."""
        if not self.data_root.is_dir():
            raise PoseEstimationError("尚无已保存的有效世界抓取位姿")
        day_dirs = sorted(
            (path for path in self.data_root.iterdir()
             if path.is_dir() and len(path.name) == 8 and path.name.isdigit()),
            key=lambda path: path.name,
            reverse=True,
        )
        for day in day_dirs:
            results = sorted(
                (path for path in day.iterdir()
                 if path.is_dir() and path.name.startswith("pose_")
                 and not path.name.startswith(".pose_")),
                key=lambda path: path.name,
                reverse=True,
            )
            for target in results:
                try:
                    summary = json.loads(
                        (target / "pose_estimation_summary.json").read_text(encoding="utf-8")
                    )
                except (OSError, ValueError):
                    continue
                selected = summary.get('selected_target')
                if selected is not None:
                    try:
                        selected = historical_type(selected)
                    except ValueError:
                        selected = None
                    if selected != expected_type:
                        raise PoseEstimationError('最近定位结果为其他类别，请重新识别 ' + expected_type)
                if summary.get('usable') is not True:
                    raise PoseEstimationError('最近一次定位无效，请重新识别，不能复用旧坐标')
                # Saved summaries may contain a grasp point calculated with an
                # older rule. Recompute from this result's original camera
                # transform and recognized point with the shared fixed height, without rewriting it.
                try:
                    request = json.loads(
                        (target / "pose_estimation_request.json").read_text(encoding="utf-8")
                    )
                    response = json.loads(
                        (target / "pose_estimation_response.json").read_text(encoding="utf-8")
                    )
                    historical = 'sku_typ' not in request
                    request = self._height_request(request, trunk_ref)
                    if request['sku_typ'] != expected_type:
                        raise ValueError('保存结果的物品类别与所选手臂不一致')
                    expected_side = 'LEFT' if expected_type == 'box' else 'RIGHT'
                    if request.get('side') != expected_side:
                        raise ValueError(f'{expected_type} 必须重新识别 {expected_side} 箱')
                    validate_response_type(response, expected_type, historical=historical)
                    world_grasp = world_grasp_from_4090(request, response)
                except (OSError, ValueError, KeyError, TypeError) as exc:
                    raise PoseEstimationError(
                        f"最近一次有效定位无法按固定高度交点规则重算：{exc}；请重新估计"
                    ) from exc
                try:
                    panel = front_panel_from_response(request, response)
                except (ValueError, KeyError, TypeError) as exc:
                    panel = {"valid": False, "error": str(exc)}
                return {
                    "source_result_id": f"{day.name}/{target.name}",
                    "source_recorded_at": summary.get("recorded_at"),
                    "sku_typ": expected_type,
                    "world_grasp": world_grasp,
                    "front_panel": panel,
                }
        raise PoseEstimationError("尚无已保存的有效世界抓取位姿；请先重新估计")

    def reproject_latest_to_right_shoulder(self, state_before, state_after):
        return self._reproject_latest_to_shoulder(state_before, state_after, 'right')

    def reproject_latest_to_left_shoulder(self, state_before, state_after):
        return self._reproject_latest_to_shoulder(state_before, state_after, 'left')

    def reproject_latest_tube_to_right_shoulder(self, state_before, state_after):
        return self._reproject_latest_to_shoulder(state_before, state_after, 'right', 'tube')

    def _reproject_latest_to_shoulder(self, state_before, state_after, arm, expected_type=None):
        expected_type = expected_type or ('box' if arm == 'left' else 'bottle')
        try:
            before = np.asarray(state_before["joints_deg"]["trunk"], dtype=np.float64)
            after = np.asarray(state_after["joints_deg"]["trunk"], dtype=np.float64)
        except (KeyError, TypeError, ValueError) as exc:
            raise PoseEstimationError("当前机器人状态缺少躯干关节角") from exc
        if (before.shape != (4,) or after.shape != (4,)
                or not np.isfinite(before).all() or not np.isfinite(after).all()):
            raise PoseEstimationError("当前躯干关节角无效")
        max_delta = float(np.max(np.abs(after - before)))
        if max_delta > float(self.config["stationary_tolerance_deg"]):
            raise PoseEstimationError(f"回读期间躯干仍在运动（变化 {max_delta:.4f}°）")
        operation = state_after.get("operation_state", {}).get("trunk", "")
        if any(token in str(operation).lower()
               for token in ("moving", "drag", "jog", "execution", "rtcontrolling")):
            raise PoseEstimationError("当前躯干仍在运动，不能生成稳定的肩部坐标")
        trunk_joints_deg = after.tolist()
        try:
            if self._kinematics is None:
                self._kinematics = UpperBodySixDofKinematics(self.config["urdf_file"])
            shoulder_transform = getattr(self._kinematics, arm + "_shoulder_sdk_world")(trunk_joints_deg)
            trunk_ref = stable_trunk_reference(state_before, state_after, self._kinematics)
            saved = self.latest_world_grasp(trunk_ref, expected_type)
            box_clearance = clearance_in_trunk(saved["front_panel"], saved["world_grasp"], trunk_ref)
            box_clearance["source_result_id"] = saved["source_result_id"]
            projector = world_grasp_to_left_shoulder if arm == "left" else world_grasp_to_right_shoulder
            shoulder_grasp = projector(
                saved["world_grasp"], shoulder_transform, box_clearance, trunk_ref
            )
        except (OSError, ValueError) as exc:
            raise PoseEstimationError(f"肩部坐标转换失败：{exc}") from exc
        return {
            **saved,
            "current_trunk_joints_deg": trunk_joints_deg,
            "max_trunk_joint_delta_deg": max_delta,
            "shoulder_grasp": shoulder_grasp,
            "box_clearance": box_clearance,
            "planning_frame": TRUNK_FRAME,
            "T_chassis_trunk_ref_m": trunk_ref.tolist(),
        }

    def result_image(self, day: str, leaf: str) -> bytes:
        if len(day) != 8 or not day.isdigit():
            raise BackendError("位姿估计结果编号无效")
        if not leaf.startswith("pose_") or any(
            character not in "0123456789abcdef_" for character in leaf[5:]
        ):
            raise BackendError("位姿估计结果编号无效")
        candidate = (self.data_root / day / leaf / "pose_estimation_overlay.jpg").resolve()
        try:
            candidate.relative_to(self.data_root)
        except ValueError as exc:
            raise BackendError("位姿估计结果路径无效") from exc
        if not candidate.is_file():
            raise BackendError("位姿估计结果图片不存在")
        return candidate.read_bytes()
