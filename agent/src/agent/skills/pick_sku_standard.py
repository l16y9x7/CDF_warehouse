import base64
import io
from collections.abc import Callable, Mapping
from dataclasses import dataclass
from typing import Any

import numpy as np

from agent.capabilities.camera import (
    CameraCapability,
    CameraStream,
    CaptureResult,
    capture_with_event,
)
from agent.capabilities.common import Hand, Pose6D, TargetType, TaskType
from agent.capabilities.estimation import (
    EstimationCapability,
    PickPoseRequest,
    PickPoseResult,
    load_test_case,
)
from agent.capabilities.manipulation import ManipulationCapability, PickRequest
from agent.capabilities.pose import BodyPoseCapability
from agent.contracts import ExecutionContext
from agent.layouts.basket import AGV_ROWS
from agent.skus import SkuSpec, sku_spec
from agent.skills.base import (
    SkillError,
    action_id,
    emit_failed,
    emit_started,
    emit_succeeded,
    fail,
    require,
)


@dataclass(frozen=True)
class PickSkuStandardInput:
    sku_id: str
    name: str
    side: str
    hand: Hand
    level: str


@dataclass(frozen=True)
class PickSkuResult:
    sku_id: str
    backend: str
    pose: Pose6D | None = None
    status: str = "SUCCEEDED"


class PickSkuStandardSkill:
    name = "pick_sku_standard"
    version = "4"

    def __init__(
        self,
        camera: CameraCapability,
        estimation: EstimationCapability,
        manipulation: ManipulationCapability,
        pose: BodyPoseCapability,
        calibration_source: Callable[[], Mapping[str, Any]] = load_test_case,
        sku_catalog: Mapping[str, SkuSpec] | None = None,
    ):
        self.camera = camera
        self.estimation = estimation
        self.manipulation = manipulation
        self.pose = pose
        self.calibration_source = calibration_source
        self.sku_catalog = dict(sku_catalog or {})

    def execute(self, context: ExecutionContext, data: PickSkuStandardInput) -> PickSkuResult:
        require(data.sku_id, "sku_id is required")
        require(data.name, "name is required")
        if data.level not in AGV_ROWS:
            raise SkillError("INVALID_INPUT", "level must be L1-L5")
        key = action_id(context, self.name, data.sku_id, data.level)
        emit_started(context, self.name, sku_id=data.sku_id, action_id=key)
        try:
            sku_typ = _sku_typ(data.sku_id, self.sku_catalog)
            # 机器人已在观察姿态。顺序：实时外参 → RGB-D → 定位。
            # Skill 读取本次 RGB-D 文件，规范化深度 NPY 后，将逐帧 K/T
            # 以及 8082 声明的坐标系元数据一起原样发送到 /infer。
            transform = self.pose.camera_transform()
            capture = capture_with_event(
                context,
                self.camera,
                "head",
                (CameraStream.COLOR, CameraStream.DEPTH),
                skill=self.name,
            )
            color, depth = _rgbd_frames(capture)
            if capture.color_intrinsics is None or capture.depth_unit != "mm":
                raise SkillError(
                    "CAPTURE_INVALID",
                    "head RGB-D capture is missing current intrinsics or millimeter depth unit",
                )
            rgb_base64 = base64.b64encode(self.camera.read_frame_bytes(color)).decode("ascii")
            depth_base64 = _float32_depth_base64(
                self.camera.read_frame_bytes(depth),
                expected_shape=(depth.height, depth.width),
            )
            result = self.estimation.estimate_pick_pose(
                PickPoseRequest(
                    TargetType.SKU,
                    sku_typ,
                    rgb_base64,
                    depth_base64,
                    capture.color_intrinsics,
                    transform.T_chassis_camera,
                    T_unit=transform.t_unit,
                    camera_frame=transform.camera_frame,
                    base_frame=transform.base_frame,
                    side=data.side,
                )
            )
            assert isinstance(result, PickPoseResult)  # /infer 形态必返回文档结构
            picked = self.manipulation.pick(
                PickRequest(
                    TaskType.SORTING,
                    TargetType.SKU,
                    sku_typ,
                    data.hand,
                    data.level,
                    result.localization_result,
                ),
                idempotency_key=key,
            )
        except Exception as exc:
            emit_failed(context, self.name, exc, action_id=key)
            raise fail(exc) from exc
        emit_succeeded(
            context,
            self.name,
            sku_id=data.sku_id,
            backend="STANDARD",
            action_id=key,
            completed_moves=picked.completed_moves,
        )
        return PickSkuResult(data.sku_id, "STANDARD")


def _sku_typ(sku_id: str, catalog: Mapping[str, SkuSpec]) -> str:
    try:
        return sku_spec(catalog, sku_id).sku_typ
    except ValueError as exc:
        raise SkillError("SKU_TYPE_UNKNOWN", str(exc)) from exc


def _wrist_camera(hand: Hand) -> str:
    return "left_wrist" if hand is Hand.LEFT else "right_wrist"


def _rgbd_paths(capture: CaptureResult) -> tuple[str, str]:
    if capture.color is None or capture.depth is None:
        raise ValueError("camera capture did not return both color and depth frames")
    return capture.color.path, capture.depth.path


def _rgbd_frames(capture: CaptureResult):
    if capture.color is None or capture.depth is None:
        raise SkillError("CAPTURE_INVALID", "camera capture did not return both RGB and depth")
    if capture.same_shot is not True or capture.depth.aligned is not True:
        raise SkillError("CAPTURE_INVALID", "RGB and aligned depth must come from the same shot")
    return capture.color, capture.depth


def _float32_depth_base64(raw: bytes, *, expected_shape: tuple[int, int]) -> str:
    try:
        depth = np.load(io.BytesIO(raw), allow_pickle=False)
    except (OSError, ValueError) as exc:
        raise SkillError("CAPTURE_INVALID", "depth payload is not a valid NPY array") from exc
    if depth.ndim != 2 or depth.shape != expected_shape:
        raise SkillError(
            "CAPTURE_INVALID",
            f"depth shape {depth.shape} does not match RGB shape {expected_shape}",
        )
    if depth.dtype.kind not in "uif" or not np.isfinite(depth).all():
        raise SkillError("CAPTURE_INVALID", "depth array must contain finite numeric values")
    converted = np.ascontiguousarray(depth, dtype=np.float32)
    output = io.BytesIO()
    np.save(output, converted, allow_pickle=False)
    return base64.b64encode(output.getvalue()).decode("ascii")


def _validate_pose(pose: Pose6D) -> None:
    if (pose.frame, pose.pose_unit, pose.rotation_order) != ("camera", "mm_rad", "zyx"):
        raise SkillError("VERIFICATION_FAILED", "estimation returned unsupported pose metadata")
