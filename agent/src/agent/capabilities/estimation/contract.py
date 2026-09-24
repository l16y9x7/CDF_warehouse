"""Robot pose inference contracts."""

import base64
import copy
import json
import math
from collections.abc import Mapping, Sequence
from dataclasses import dataclass, field as dataclass_field
from pathlib import Path
from typing import Any, Protocol

from agent.capabilities.common import (
    CapabilityError,
    HealthResult,
    TargetType,
)

CAMERA_FRAME = "head_camera_color_optical_frame"
BASE_FRAME = "chassis_link"
SKU_CLASSES = frozenset({"bottle", "box", "tube"})
CASE_PREFIXES = SKU_CLASSES | {"basket"}
SIDES = frozenset({"LEFT", "RIGHT"})
T_UNITS = frozenset({"m", "mm"})
FRONT_RULE_KEYS = frozenset({"front_axis_chassis", "front_origin_chassis", "front_band_mm"})

Matrix3 = tuple[tuple[float, ...], ...]
Matrix4 = tuple[tuple[float, ...], ...]
TOLERANCE = 1e-6

def as_matrix(value: Any, size: int) -> tuple[tuple[float, ...], ...]:
    def invalid() -> ValueError:
        return ValueError(f"expected a {size}x{size} numeric matrix")

    if not isinstance(value, Sequence) or isinstance(value, (str, bytes)) or len(value) != size:
        raise invalid()
    rows: list[tuple[float, ...]] = []
    for row in value:
        if not isinstance(row, Sequence) or isinstance(row, (str, bytes)) or len(row) != size:
            raise invalid()
        try:
            values = tuple(float(item) for item in row)
        except (TypeError, ValueError) as exc:
            raise invalid() from exc
        if not all(math.isfinite(item) for item in values):
            raise invalid()
        rows.append(values)
    return tuple(rows)


def validate_intrinsics(matrix: Matrix3) -> None:
    if matrix[0][0] <= 0.0 or matrix[1][1] <= 0.0:
        raise ValueError("camera intrinsics focal lengths must be positive")


def validate_extrinsics(matrix: Matrix4) -> None:
    rows = [row[:3] for row in matrix[:3]]
    gap = max(
        abs(sum(a * b for a, b in zip(rows[i], rows[j], strict=True)) - (1.0 if i == j else 0.0))
        for i in range(3)
        for j in range(3)
    )
    if gap > TOLERANCE:
        raise ValueError("T_chassis_camera rotation must be orthonormal")
    det = (
        rows[0][0] * (rows[1][1] * rows[2][2] - rows[1][2] * rows[2][1])
        - rows[0][1] * (rows[1][0] * rows[2][2] - rows[1][2] * rows[2][0])
        + rows[0][2] * (rows[1][0] * rows[2][1] - rows[1][1] * rows[2][0])
    )
    if abs(det - 1.0) > TOLERANCE:
        raise ValueError("T_chassis_camera rotation determinant must be ~1")
    if tuple(matrix[3]) != (0.0, 0.0, 0.0, 1.0):
        raise ValueError("T_chassis_camera last row must be [0, 0, 0, 1]")

@dataclass(frozen=True)
class PickPoseRequest:
    """/infer request; frame bytes are already base64 encoded."""

    target_type: TargetType
    sku_typ: str | None
    rgb_base64: str
    depth_npy_base64: str
    K: Matrix3
    T_chassis_camera: Matrix4
    camera_frame: str
    base_frame: str
    side: str | None
    front_rule: Mapping[str, Any] | None = None
    T_unit: str = "m"
    depth_unit: str = "mm"

    def __post_init__(self) -> None:
        object.__setattr__(self, "K", as_matrix(self.K, 3))
        object.__setattr__(self, "T_chassis_camera", as_matrix(self.T_chassis_camera, 4))
        if self.target_type not in (TargetType.SKU, TargetType.BASKET):
            raise ValueError("pick pose inference only supports target_type=sku or basket")
        for name in ("rgb_base64", "depth_npy_base64"):
            value = getattr(self, name)
            if not isinstance(value, str) or not value.strip():
                raise ValueError(f"{name} must be a non-empty base64 string")
        if self.depth_unit != "mm":
            raise ValueError("depth_unit must be mm")
        if self.T_unit not in T_UNITS:
            raise ValueError("T_unit must be m or mm")
        for name in ("camera_frame", "base_frame"):
            value = getattr(self, name)
            if not isinstance(value, str) or not value.strip():
                raise ValueError(f"{name} must be a non-empty string")
        validate_intrinsics(self.K)
        validate_extrinsics(self.T_chassis_camera)
        if self.target_type is TargetType.SKU:
            if not isinstance(self.sku_typ, str) or not self.sku_typ.strip():
                raise ValueError("sku_typ is required")
            if self.side not in SIDES:
                raise ValueError("side must be LEFT or RIGHT")
            if self.front_rule is not None:
                _validate_front_rule(self.front_rule)
            return
        if self.sku_typ is not None:
            raise ValueError("basket infer does not accept sku_typ")
        if self.side is not None:
            raise ValueError("basket infer does not accept side")
        if self.front_rule is not None:
            raise ValueError("basket infer does not accept front_rule")


def _validate_front_rule(rule: Mapping[str, Any]) -> None:
    if set(rule) != FRONT_RULE_KEYS:
        raise ValueError(
            "front_rule requires front_axis_chassis, front_origin_chassis and front_band_mm"
        )
    for key in ("front_axis_chassis", "front_origin_chassis"):
        vector = rule[key]
        if (
            not isinstance(vector, Sequence)
            or isinstance(vector, (str, bytes))
            or len(vector) != 3
        ):
            raise ValueError(f"front_rule.{key} must be a 3-vector")
    if float(rule["front_band_mm"]) <= 0.0:
        raise ValueError("front_rule.front_band_mm must be positive")


def _vector(payload: Mapping[str, Any], key: str, size: int) -> tuple[float, ...] | None:
    value = payload.get(key)
    if value is None:
        return None
    try:
        values = tuple(float(item) for item in value)
    except (TypeError, ValueError):
        return None
    return values if len(values) == size and all(math.isfinite(item) for item in values) else None


def _mapping(payload: Mapping[str, Any], key: str) -> dict[str, Any] | None:
    value = payload.get(key)
    return dict(value) if isinstance(value, Mapping) else None


# ROBOT_API_HANDOFF.md §6「关键字段」表；表外键一律进 raw 透传。
_DOCUMENTED_RESPONSE_KEYS = frozenset(
    {
        "ok",
        "target_type",
        "sku_typ",
        "output_frame",
        "output_unit",
        "sam3_call_count",
        "selected_instance_id",
        "filtered_instance_id",
        "box_selection",
        "front_panel_valid",
        "front_panel_top_edge_midpoint_camera_mm",
        "front_panel_plane_point_camera_mm",
        "front_panel_plane_normal_camera",
        "sam3_score",
        "axis_fit_valid",
        "reference_point_valid",
        "axis_point_camera_mm",
        "axis_direction_camera_up",
        "reference_point_camera_mm",
        "reference_point_chassis_mm",
        "reference_z_mm",
        "rejection_reasons",
        "diagnostics",
        "artifacts",
    }
)


@dataclass(frozen=True)
class PickPoseResult:
    """/infer 响应"""

    ok: bool = False
    target_type: str | None = None
    sku_typ: str | None = None
    output_frame: str | None = None
    output_unit: str | None = None
    sam3_call_count: int | None = None
    selected_instance_id: int | None = None
    filtered_instance_id: int | None = None
    box_selection: dict[str, Any] | None = None
    front_panel_valid: bool | None = None
    front_panel_top_edge_midpoint_camera_mm: tuple[float, ...] | None = None
    front_panel_plane_point_camera_mm: tuple[float, ...] | None = None
    front_panel_plane_normal_camera: tuple[float, ...] | None = None
    sam3_score: float | None = None
    axis_fit_valid: bool | None = None
    reference_point_valid: bool | None = None
    axis_point_camera_mm: tuple[float, ...] | None = None
    axis_direction_camera_up: tuple[float, ...] | None = None
    reference_point_camera_mm: tuple[float, ...] | None = None
    reference_point_chassis_mm: tuple[float, ...] | None = None
    reference_z_mm: float | None = None
    rejection_reasons: tuple[str, ...] = ()
    diagnostics: dict[str, Any] | None = dataclass_field(default=None)
    artifacts: dict[str, Any] | None = dataclass_field(default=None)
    raw: dict[str, Any] = dataclass_field(default_factory=dict)
    localization_result: dict[str, Any] = dataclass_field(default_factory=dict)

    @classmethod
    def from_payload(cls, payload: Any) -> "PickPoseResult":
        if not isinstance(payload, Mapping):
            raise CapabilityError("PICK_POSE_INVALID", "定位服务响应不是 JSON 对象")
        reasons = payload.get("rejection_reasons")
        return cls(
            ok=payload.get("ok") is True,
            target_type=_optional_string(payload.get("target_type")),
            sku_typ=_optional_string(payload.get("sku_typ")),
            output_frame=_optional_string(payload.get("output_frame")),
            output_unit=_optional_string(payload.get("output_unit")),
            sam3_call_count=_optional_int(payload.get("sam3_call_count")),
            selected_instance_id=_optional_int(payload.get("selected_instance_id")),
            filtered_instance_id=_optional_int(payload.get("filtered_instance_id")),
            box_selection=_mapping(payload, "box_selection"),
            front_panel_valid=_optional_bool(payload.get("front_panel_valid")),
            front_panel_top_edge_midpoint_camera_mm=_vector(
                payload, "front_panel_top_edge_midpoint_camera_mm", 3
            ),
            front_panel_plane_point_camera_mm=_vector(
                payload, "front_panel_plane_point_camera_mm", 3
            ),
            front_panel_plane_normal_camera=_vector(payload, "front_panel_plane_normal_camera", 3),
            sam3_score=_optional_float(payload.get("sam3_score")),
            axis_fit_valid=_optional_bool(payload.get("axis_fit_valid")),
            reference_point_valid=_optional_bool(payload.get("reference_point_valid")),
            axis_point_camera_mm=_vector(payload, "axis_point_camera_mm", 3),
            axis_direction_camera_up=_vector(payload, "axis_direction_camera_up", 3),
            reference_point_camera_mm=_vector(payload, "reference_point_camera_mm", 3),
            reference_point_chassis_mm=_vector(payload, "reference_point_chassis_mm", 3),
            reference_z_mm=_optional_float(payload.get("reference_z_mm")),
            rejection_reasons=(
                tuple(str(item) for item in reasons)
                if isinstance(reasons, Sequence) and not isinstance(reasons, (str, bytes))
                else ()
            ),
            diagnostics=_mapping(payload, "diagnostics"),
            artifacts=_mapping(payload, "artifacts"),
            raw={
                key: value
                for key, value in payload.items()
                if key not in _DOCUMENTED_RESPONSE_KEYS
            },
            localization_result=copy.deepcopy(dict(payload)),
        )


@dataclass(frozen=True)
class BasketPoseResult:
    """The basket-shaped response returned by the same ``/infer`` endpoint."""

    ok: bool = False
    target_type: str | None = None
    pose_valid: bool = False
    point_semantics: str | None = None
    model_center_camera_mm: tuple[float, ...] | None = None
    reference_point_camera_mm: tuple[float, ...] | None = None
    reference_point_chassis_mm: tuple[float, ...] | None = None
    pose_4x4: Matrix4 | None = None
    output_frame: str | None = None
    output_unit: str | None = None
    raw: dict[str, Any] = dataclass_field(default_factory=dict)
    localization_result: dict[str, Any] = dataclass_field(default_factory=dict)

    @classmethod
    def from_payload(cls, payload: Any) -> "BasketPoseResult":
        if not isinstance(payload, Mapping):
            raise CapabilityError("BASKET_POSE_INVALID", "篮筐定位服务响应不是 JSON 对象")
        matrix = None
        if payload.get("pose_4x4") is not None:
            try:
                matrix = as_matrix(payload["pose_4x4"], 4)
                validate_extrinsics(matrix)
            except ValueError:
                matrix = None
        return cls(
            ok=payload.get("ok") is True,
            target_type=_optional_string(payload.get("target_type")),
            pose_valid=payload.get("pose_valid") is True,
            point_semantics=_optional_string(payload.get("point_semantics")),
            model_center_camera_mm=_vector(payload, "model_center_camera_mm", 3),
            reference_point_camera_mm=_vector(payload, "reference_point_camera_mm", 3),
            reference_point_chassis_mm=_vector(payload, "reference_point_chassis_mm", 3),
            pose_4x4=matrix,
            output_frame=_optional_string(payload.get("output_frame")),
            output_unit=_optional_string(payload.get("output_unit")),
            raw=copy.deepcopy(dict(payload)),
            localization_result=copy.deepcopy(dict(payload)),
        )


def _optional_bool(value: Any) -> bool | None:
    return value if isinstance(value, bool) else None


def _optional_int(value: Any) -> int | None:
    return int(value) if isinstance(value, int) and not isinstance(value, bool) else None


def _optional_float(value: Any) -> float | None:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        return None
    number = float(value)
    return number if math.isfinite(number) else None


def _optional_string(value: Any) -> str | None:
    return value if isinstance(value, str) and value else None


class EstimationCapability(Protocol):
    def health(self) -> HealthResult: ...
    def estimate_pick_pose(self, request: PickPoseRequest) -> PickPoseResult: ...
    def estimate_basket_pose(self, request: PickPoseRequest) -> BasketPoseResult: ...


TMP_DIR = Path(__file__).resolve().parents[4] / "tmp"
TEST_CASE_ROOT = TMP_DIR / "pick_pose_test_case"
SHARED_REQUEST_NAME = "test_case_request.json"
LEGACY_SHARED_REQUEST_NAME = "pick_pose_request.json"
CASE_REQUEST_NAME = "request.json"
RGB_FILE_NAMES = ("rgb.jpg", "rgb.jpeg", "rgb.png")
DEPTH_FILE_NAME = "depth_mm.npy"
CAMERA_FILE_NAME = "camera.json"
TEST_CASE_JOB_FIELDS = frozenset({"target_type", "sku_typ", "side"})


def scan_test_cases(root: str | Path | None = None) -> list[str]:
    base = Path(root) if root is not None else TEST_CASE_ROOT
    if not base.is_dir():
        return []
    prefixes = {name.lower() for name in CASE_PREFIXES}
    return sorted(
        entry.name
        for entry in base.iterdir()
        if entry.is_dir() and entry.name.split("_", 1)[0].lower() in prefixes
    )


def encode_frame_file(path: str | Path) -> str:
    """把本地帧文件读成 /infer 需要的 base64（JPG/PNG 或 .npy 字节）。"""
    file_path = Path(path)
    if not file_path.is_file():
        raise ValueError(f"帧文件不存在: {file_path}")
    raw = file_path.read_bytes()
    if not raw:
        raise ValueError(f"帧文件为空: {file_path}")
    return base64.b64encode(raw).decode("ascii")


def sku_typ_from_case(case: str) -> str | None:
    prefix = case.split("_", 1)[0].lower()
    for sku in SKU_CLASSES:
        if sku.lower() == prefix:
            return sku
    return None


def load_test_case(case: str | None = None, root: str | Path | None = None) -> dict[str, Any]:
    """加载本地测试用例(tmp/pick_pose_test_case/)"""
    base = Path(root) if root is not None else TEST_CASE_ROOT
    shared = base / SHARED_REQUEST_NAME
    if not shared.exists():
        shared = base / LEGACY_SHARED_REQUEST_NAME
    data = _read_json_object(shared)
    if case is None:
        cases = scan_test_cases(base)
        if len(cases) > 1:
            raise ValueError("存在多个测试用例,请选择: " + ", ".join(cases))
        case = cases[0] if cases else None
    if case is None:
        _merge_frame_files(data, base)
        return data
    case_dir = base / case
    if not case_dir.is_dir():
        raise ValueError(f"测试用例目录不存在: {case_dir}")
    sku = sku_typ_from_case(case)
    if sku is not None:
        data["sku_typ"] = sku
    _merge_frame_files(data, case_dir)
    return {**data, **_read_json_object(case_dir / CASE_REQUEST_NAME)}


def test_case_meta(root: str | Path | None = None) -> list[dict[str, Any]]:
    """调试台用例下拉元数据：只暴露帧/标定，不含目标类型/定位类别/纸箱侧。"""
    base = Path(root) if root is not None else TEST_CASE_ROOT
    cases: list[dict[str, Any]] = []
    for name in scan_test_cases(base):
        merged = load_test_case(name, root=base)
        merged.pop("rgb_base64", None)
        merged.pop("depth_npy_base64", None)
        for key in TEST_CASE_JOB_FIELDS:
            merged.pop(key, None)
        cases.append({"name": name, **merged})
    return cases


def _read_json_object(path: Path) -> dict[str, Any]:
    if not path.exists():
        return {}
    loaded = json.loads(path.read_text("utf-8"))
    if not isinstance(loaded, dict):
        raise ValueError(f"{path} 必须是 JSON 对象")
    return {key: value for key, value in loaded.items() if not key.startswith("_")}


def _merge_frame_files(data: dict[str, Any], directory: Path) -> None:
    camera_path = directory / CAMERA_FILE_NAME
    if camera_path.exists():
        data["K"] = _camera_intrinsics(camera_path)
    for name in RGB_FILE_NAMES:
        rgb_path = directory / name
        if rgb_path.exists():
            data["rgb_base64"] = base64.b64encode(rgb_path.read_bytes()).decode("ascii")
            break
    depth_path = directory / DEPTH_FILE_NAME
    if depth_path.exists():
        data["depth_npy_base64"] = base64.b64encode(depth_path.read_bytes()).decode("ascii")


def _camera_intrinsics(path: Path) -> list[list[float]]:
    values = _read_json_object(path)
    flat = values.get("cam_K", values.get("K"))
    if (
        not isinstance(flat, list)
        or len(flat) != 9
        or not all(isinstance(item, (int, float)) for item in flat)
    ):
        raise ValueError(f"{path} 缺少扁平 9 元素 cam_K 内参")
    return [[float(flat[i]) for i in range(row * 3, row * 3 + 3)] for row in range(3)]
