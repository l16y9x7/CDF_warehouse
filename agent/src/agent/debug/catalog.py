from collections.abc import Callable
from dataclasses import dataclass
from typing import Any, Literal, cast

from agent.capabilities.camera import CameraStream, DepthFormat, capture_with_event
from agent.capabilities.common import (
    DestinationType,
    Hand,
    Pose6D,
    TargetType,
    TaskType,
)
from agent.capabilities.estimation import (
    PickPoseRequest,
    load_test_case,
    scan_test_cases,
)
from agent.capabilities.estimation.contract import BASE_FRAME, CAMERA_FRAME, TEST_CASE_JOB_FIELDS, encode_frame_file
from agent.capabilities.hand import HandPickRequest
from agent.capabilities.manipulation import (
    PickRequest,
    PlaceRequest,
    PushRequest,
    ReviewItemPickRequest,
)
from agent.capabilities.navigation import NAVIGATION_TARGETS
from agent.capabilities.perception import ImageRequest, RecognizeBarcodeRequest
from agent.capabilities.vla import VlaPickRequest
from agent.config import load_sku_catalog
from agent.contracts import ExecutionContext
from agent.models import InspectedItem, ReviewItemCount
from agent.skills.base import action_id
from agent.skills.basket_finish import PushBasketInput
from agent.skills.navigate import NavigateInput
from agent.skills.perception import RecognizeBarcodeInput
from agent.skills.pick_sku_hand import PickSkuHandInput
from agent.skills.pick_sku_standard import PickSkuStandardInput
from agent.skills.place_sku_in_basket import PlaceSkuInBasketInput
from agent.skills.pose import PreparePoseInput
from agent.skills.review import HandOnlyInput, SummarizeReviewInput
from agent.workflows.review import ReviewInput
from agent.workflows.sorting import SortingFinishInput, SortingItemInput

Layer = Literal["capability", "skill", "workflow"]
Executor = Callable[[Any, ExecutionContext, dict[str, Any]], Any]
InputFactory = Callable[[dict[str, Any]], Any]
_SKU_CATALOG = load_sku_catalog()
SKU_IDS = list(_SKU_CATALOG) or ["3282779003131", "887167608641", "7173342765403"]
DEFAULT_SKU_ID = SKU_IDS[0]
SKU_NAMES = [spec.name for spec in _SKU_CATALOG.values() if spec.name] or ["测试商品"]
DEFAULT_SKU_NAME = (
    _SKU_CATALOG[DEFAULT_SKU_ID].name
    if DEFAULT_SKU_ID in _SKU_CATALOG and _SKU_CATALOG[DEFAULT_SKU_ID].name
    else SKU_NAMES[0]
)

FIELD_DESCRIPTIONS = {
    "hand": "执行动作的机械手，LEFT 为左手，RIGHT 为右手。",
    "image_base64": "待识别图片的 Base64 编码字符串。",
    "image_path": "待识别图片在共享目录中的完整路径。",
    "pose": "六维位姿数组，依次为 x、y、z、rx、ry、rz。",
    "frame": "位姿采用的参考坐标系。",
    "pose_unit": "位置与旋转分量使用的单位组合。",
    "rotation_order": "欧拉角旋转轴的应用顺序。",
    "rgb": "彩色图像在共享目录中的完整路径。",
    "depth": "与 RGB 同帧对齐的深度数据路径。",
    "nav_id": "机器人要前往的预定义导航点标识。",
    "target_id": "机器人要前往的预定义导航点标识。",
    "pose_type": "机器人要切换到的预定义姿态名称。",
    "level": "动作对应的 AGV 货架或篮筐层级；部分动作不需要。",
    "camera": "采集画面的相机标识。",
    "streams": "要同时采集的数据流，多个值使用英文逗号分隔。",
    "format": "深度图的输出格式；仅在采集 depth 时生效。",
    "task_type": "当前操作所属的业务任务类型。",
    "target_type": "机械臂或视觉算法处理的目标类别。",
    "destination_type": "放置动作的目标位置类别。",
    "mask": "目标分割掩码或掩码数据的引用。",
    "sku_id": "商品的唯一 SKU 标识；用于识别校验或动作追踪。",
    "sku_typ": "定位与抓取模块使用的商品几何类别。",
    "localization_result": "位姿估计 /infer 返回的完整原始 JSON 对象。",
    "expected_sku_id": "可选的期望 SKU；填写后会校验识别结果。",
    "name": "商品名称，可作为视觉识别的辅助信息，不参与条码或任务校验。",
    "side": "商品所在纸箱侧或机械臂作业侧。",
    "expected_items": "期望商品清单；每项包含 sku_id 和 count。",
    "inspected_items": "实检商品清单；每项包含 sequence 和 actual_sku_id。",
    "basket_row": "篮筐所在行，支持 L1 至 L4。",
    "basket_column": "篮筐所在列，支持 1 至 5。",
    "agv_row": "AGV 纸箱所在行，支持 L1 至 L5。",
    "agv_column": "AGV 纸箱所在列，支持 1 或 2。",
    "test_case": "本地测试用例（tmp/pick_pose_test_case/<bottle|box|tube|basket>_<帧>）；选择后只补全 RGB/D、内外参、单位和坐标系，不改目标类型/定位类别/纸箱侧。",
    "K": "3x3 针孔相机内参，须与当前 RGB 分辨率一致；留空时由测试用例自动补全。",
    "T_chassis_camera": "4x4 逐帧外参，p_chassis = R @ p_camera + t；留空时由测试用例自动补全。",
    "T_unit": "外参平移列 t 的单位；旋转无量纲。",
    "depth_unit": "深度单位，服务内部几何统一使用毫米。",
    "camera_frame": "相机坐标系名称，应与本次外参元数据一致。",
    "base_frame": "基坐标系名称，应与本次外参元数据一致。",
    "front_rule": "chassis 系前排过滤规则：轴、原点与半带宽（mm）；可选，缺省不发送。",
    "sku_typ": "定位几何类别：bottle / box / tube。",
    "rgb": "可选 RGB 文件路径；填写后按文件字节 base64 发送，覆盖测试用例帧。",
    "depth": "可选对齐深度 .npy 路径；填写后按文件字节 base64 发送，覆盖测试用例帧。",
}

FIELD_OPTIONS: dict[str, list[Any]] = {
    "hand": ["LEFT", "RIGHT"],
    "image_base64": ["aW1hZ2U="],
    "image_path": ["/shared/frames/capture-1/rgb.jpg"],
    "pose": [[100, 20, 400, 0, 0, 0], [0, 0, 400, 0, 0, 0]],
    "frame": ["camera", "base", "world"],
    "pose_unit": ["mm_rad", "mm_deg", "m_rad"],
    "rotation_order": ["zyx", "xyz"],
    "rgb": ["/shared/frames/capture-1/rgb.jpg"],
    "depth": ["/shared/frames/capture-1/depth_mm.npy"],
    "nav_id": sorted(NAVIGATION_TARGETS),
    "target_id": sorted(NAVIGATION_TARGETS),
    "pose_type": [
        "AGV_carton_item_inspect",
        "AGV_item_barcode_scan",
        "basket_item_place_prepare",
        "basket_item_pick_inspect",
        "basket_item_pick_prepare",
        "basket_item_barcode_scan",
        "basket_place",
        "review_item_place",
        "basket_pick_prepare",
        "basket_push",
    ],
    "level": ["L1", "L2", "L3", "L4", "L5"],
    "camera": ["head", "left_wrist", "right_wrist"],
    "streams": ["color", "depth", "color,depth"],
    "format": ["raw", "preview"],
    "task_type": [item.value for item in TaskType],
    "target_type": [item.value for item in TargetType],
    "destination_type": [item.value for item in DestinationType],
    "mask": ["mask", "/shared/frames/capture-1/mask.png"],
    "sku_id": SKU_IDS,
    "sku_typ": ["bottle", "box", "tube"],
    "expected_sku_id": SKU_IDS,
    "name": SKU_NAMES,
    "side": ["LEFT", "RIGHT"],
    "expected_items": [[{"sku_id": DEFAULT_SKU_ID, "count": 1}], []],
    "inspected_items": [[{"sequence": 1, "actual_sku_id": DEFAULT_SKU_ID}], []],
    "basket_row": ["L1", "L2", "L3", "L4"],
    "basket_column": ["1", "2", "3", "4", "5"],
    "agv_row": ["L1", "L2", "L3", "L4", "L5"],
    "agv_column": ["1", "2"],
    "T_unit": ["m", "mm"],
    "depth_unit": ["mm"],
    "camera_frame": ["head_camera_color_optical_frame"],
    "base_frame": ["chassis_link"],
    "K": [
        [
            [612.772339587192, 0.0, 641.544745101529],
            [0.0, 611.998287842651, 358.604256964916],
            [0.0, 0.0, 1.0],
        ]
    ],
    "T_chassis_camera": [
        [
            [-0.047653753369, -0.428149456867, 0.902450642625, 0.113767949307],
            [-0.998816369947, 0.011610119563, -0.04723414285, 0.018099569521],
            [0.009745712746, -0.903633359117, -0.428195952077, 1.45137337738],
            [0.0, 0.0, 0.0, 1.0],
        ]
    ],
    "front_rule": [
        {
            "front_axis_chassis": [1.0, 0.0, 0.0],
            "front_origin_chassis": [0.0, 0.0, 0.0],
            "front_band_mm": 100000.0,
        }
    ],
    "localization_result": [
        {
            "ok": True,
            "target_type": "sku",
            "sku_typ": "bottle",
            "output_frame": "head_camera_color_optical_frame",
            "output_unit": "mm",
        }
    ],
}


def field(
    name: str,
    label: str,
    kind: str = "text",
    *,
    required: bool = True,
    options: list[Any] | None = None,
    default: Any = None,
    description: str | None = None,
) -> dict[str, Any]:
    choices = list(options if options is not None else FIELD_OPTIONS.get(name, []))
    if default is not None and default not in choices:
        choices.insert(0, default)
    result = {
        "name": name,
        "label": label,
        "type": kind,
        "required": required,
        "description": description
        or FIELD_DESCRIPTIONS.get(name, f"{label}（{name}）的输入值。"),
        "options": choices,
    }
    if default is not None:
        result["default"] = default
    return result


@dataclass(frozen=True)
class DebugOperation:
    layer: Layer
    name: str
    title: str
    group: str
    description: str
    fields: tuple[dict[str, Any], ...] = ()
    physical: bool = False
    executor: Executor | None = None
    input_factory: InputFactory | None = None

    def payload(self) -> dict[str, Any]:
        return {
            "layer": self.layer,
            "name": self.name,
            "title": self.title,
            "group": self.group,
            "description": self.description,
            "fields": list(self.fields),
            "physical": self.physical,
        }

    def validate(self, payload: dict[str, Any]) -> None:
        allowed = {item["name"] for item in self.fields}
        extra = set(payload) - allowed
        if extra:
            raise ValueError(f"unexpected fields: {', '.join(sorted(extra))}")
        missing = [
            item["name"] for item in self.fields if item["required"] and item["name"] not in payload
        ]
        if missing:
            raise ValueError(f"missing fields: {', '.join(missing)}")

    def execute(self, application: Any, context: ExecutionContext, payload: dict[str, Any]) -> Any:
        if self.executor is not None:
            return self.executor(application, context, payload)
        if self.layer == "skill" and self.input_factory is not None:
            return application.skills[self.name].execute(context, self.input_factory(payload))
        raise ValueError(f"operation {self.name} is not directly executable")

    def workflow_input(self, payload: dict[str, Any]) -> Any:
        if self.layer != "workflow" or self.input_factory is None:
            raise ValueError(f"operation {self.name} is not a workflow")
        return self.input_factory(payload)


def capability(
    name: str,
    title: str,
    group: str,
    description: str,
    executor: Executor,
    fields: tuple[dict[str, Any], ...] = (),
    *,
    physical: bool = False,
) -> DebugOperation:
    return DebugOperation(
        "capability",
        name,
        title,
        group,
        description,
        fields,
        physical,
        executor,
    )


def skill(
    name: str,
    title: str,
    group: str,
    description: str,
    factory: InputFactory,
    fields: tuple[dict[str, Any], ...] = (),
    *,
    physical: bool = False,
) -> DebugOperation:
    return DebugOperation(
        "skill",
        name,
        title,
        group,
        description,
        fields,
        physical,
        input_factory=factory,
    )


def workflow(
    name: str,
    title: str,
    group: str,
    description: str,
    factory: InputFactory,
    fields: tuple[dict[str, Any], ...],
) -> DebugOperation:
    return DebugOperation(
        "workflow",
        name,
        title,
        group,
        description,
        fields,
        input_factory=factory,
    )


def _module(application: Any, operation: str) -> Any:
    return application.capabilities[operation.split(".", 1)[0]]


def _health(operation: str) -> Executor:
    return lambda app, context, payload: _module(app, operation).health()


def _pose(payload: dict[str, Any]) -> Pose6D:
    values = tuple(float(value) for value in payload["pose"])
    if len(values) != 6:
        raise ValueError("pose must contain six values")
    typed_values = cast(tuple[float, float, float, float, float, float], values)
    return Pose6D(
        typed_values,
        ((0.0, 0.0, 0.0),) * 8,
        payload.get("frame", "camera"),
        payload.get("pose_unit", "mm_rad"),
        payload.get("rotation_order", "zyx"),
    )


def _hand(payload: dict[str, Any]) -> Hand:
    return Hand(payload.get("hand", "RIGHT"))


PICK_POSE_FRAME_REQUIRED = (
    "target_type",
    "rgb_base64",
    "depth_npy_base64",
    "K",
    "T_chassis_camera",
)
PICK_POSE_SKU_REQUIRED = ("sku_typ", "side")
PICK_POSE_BASKET_DROP = ("sku_typ", "side", "front_rule")
# 线上必填但取值固定；表单/用例未提供时由 executor 兜底。
PICK_POSE_FIXED = {
    "depth_unit": "mm",
    "T_unit": "m",
    "camera_frame": CAMERA_FRAME,
    "base_frame": BASE_FRAME,
}


TEST_CASE_SENTINEL = "(无本地测试用例)"


def _pick_pose_request(payload: dict[str, Any]) -> PickPoseRequest:
    # test_case 为调试台专用参数（不进请求体）：自动补全本地测试用例的帧
    # 数据（base64/K/T 等），表单显式值优先级最高。
    provided = {
        key: value
        for key, value in payload.items()
        if value not in (None, "", TEST_CASE_SENTINEL)
    }
    case = provided.pop("test_case", None)
    case_data = load_test_case(case)
    for name in TEST_CASE_JOB_FIELDS:
        case_data.pop(name, None)
    data: dict[str, Any] = {**PICK_POSE_FIXED, **case_data, **provided}
    if data.get("rgb"):
        data["rgb_base64"] = encode_frame_file(data["rgb"])
    if data.get("depth"):
        data["depth_npy_base64"] = encode_frame_file(data["depth"])
    target = TargetType(data.get("target_type", "sku"))
    if target is TargetType.BASKET:
        for name in PICK_POSE_BASKET_DROP:
            data.pop(name, None)
        required = PICK_POSE_FRAME_REQUIRED
    else:
        required = PICK_POSE_FRAME_REQUIRED + PICK_POSE_SKU_REQUIRED
    missing = [name for name in required if not data.get(name)]
    if missing:
        raise ValueError(
            "missing fields: " + ", ".join(missing) + "（可在测试用例目录预置）"
        )
    kwargs: dict[str, Any] = {
        "target_type": target,
        "sku_typ": data.get("sku_typ") if target is TargetType.SKU else None,
        "rgb_base64": data["rgb_base64"],
        "depth_npy_base64": data["depth_npy_base64"],
        "K": data["K"],
        "T_chassis_camera": data["T_chassis_camera"],
        "side": data.get("side") if target is TargetType.SKU else None,
        **{name: data[name] for name in PICK_POSE_FIXED},
    }
    if target is TargetType.SKU and data.get("front_rule"):
        kwargs["front_rule"] = data["front_rule"]
    return PickPoseRequest(**kwargs)


HAND = field(
    "hand",
    "工作手",
    "select",
    options=["LEFT", "RIGHT"],
    default="RIGHT",
)
IMAGE = field("image_path", "图片路径", default="/shared/frames/capture-1/rgb.jpg")
BARCODE_IMAGE = field("image_base64", "图片 Base64", default="aW1hZ2U=")
POSE = field("pose", "6D 位姿", "json", default=[100, 20, 400, 0, 0, 0])
POSE_META = (
    field("frame", "坐标系", default="camera"),
    field("pose_unit", "位姿单位", default="mm_rad"),
    field("rotation_order", "旋转顺序", default="zyx"),
)
RGBD = (
    field("rgb", "RGB 路径", default="/shared/frames/capture-1/rgb.jpg"),
    field(
        "depth",
        "深度路径",
        default="/shared/frames/capture-1/depth_mm.npy",
    ),
)

HEALTH_OPERATIONS = tuple(
    capability(
        f"{name}.health",
        "健康检查",
        name.title(),
        "查询模块状态",
        _health(f"{name}.health"),
    )
    for name in (
        "navigation",
        "pose",
        "perception",
        "estimation",
        "camera",
        "manipulation",
        "vla",
        "hand",
    )
)

CAPABILITY_OPERATIONS = (
    *HEALTH_OPERATIONS,
    capability(
        "navigation.navigate",
        "导航到点位",
        "Navigation",
        "执行底盘导航",
        lambda app, ctx, data: _module(app, "navigation.navigate").navigate(
            data["nav_id"], idempotency_key=action_id(ctx, "navigation.navigate", data["nav_id"])
        ),
        (field("nav_id", "导航点", "select", options=sorted(NAVIGATION_TARGETS)),),
        physical=True,
    ),
    capability(
        "pose.prepare",
        "准备位姿",
        "Pose",
        "进入预定义姿态",
        lambda app, ctx, data: _module(app, "pose.prepare").prepare(
            data["pose_type"],
            data.get("level"),
            idempotency_key=action_id(
                ctx, "pose.prepare", data["pose_type"], data.get("level") or "none"
            ),
        ),
        (field("pose_type", "位姿类型"), field("level", "层级", required=False)),
        physical=True,
    ),
    capability(
        "camera.capture",
        "抓取相机帧",
        "Camera",
        "同次抓取彩色和/或对齐深度帧",
        lambda app, ctx, data: capture_with_event(
            ctx,
            _module(app, "camera.capture"),
            data["camera"],
            tuple(CameraStream(item.strip()) for item in data.get("streams", "color").split(",")),
            depth_format=DepthFormat(data.get("format", "raw")),
        ),
        (
            field("camera", "相机", default="head"),
            field("streams", "数据流", default="color"),
            field("format", "深度格式", required=False, default="raw"),
        ),
    ),
    capability(
        "perception.recognize_sku_barcode",
        "识别商品条码",
        "Perception",
        "直接识别条码",
        lambda app, ctx, data: _module(
            app, "perception.recognize_sku_barcode"
        ).recognize_sku_barcode(
            RecognizeBarcodeRequest(
                data["image_base64"],
                data.get("sku_id") or "",
                data.get("name") or "",
            )
        ),
        (
            BARCODE_IMAGE,
            field("sku_id", "SKU 参考", required=False),
            field("name", "名称参考", required=False),
        ),
    ),
    capability(
        "perception.locate_basket_item",
        "定位篮筐商品",
        "Perception",
        "返回 FOUND/NOT_FOUND",
        lambda app, ctx, data: _module(app, "perception.locate_basket_item").locate_basket_item(
            ImageRequest(data["image_path"])
        ),
        (IMAGE,),
    ),
    capability(
        "estimation.pick_pose",
        "估计抓取位姿",
        "Estimation",
        "/infer 商品或篮筐定位，返回文档结构响应",
        lambda app, ctx, data: _module(app, "estimation.pick_pose").estimate_pick_pose(
            _pick_pose_request(data)
        ),
        (
            field("test_case", "测试用例", "select", required=False),
            field(
                "target_type",
                "目标类型",
                "select",
                default="sku",
                options=["sku", "basket"],
            ),
            field(
                "sku_typ",
                "定位类别",
                "select",
                default="bottle",
                options=["bottle", "box", "tube"],
                required=False,
            ),
            field("side", "纸箱侧", "select", default="RIGHT", required=False),
            field("rgb", "RGB 路径", required=False),
            field("depth", "深度路径", required=False),
            field("K", "相机内参 K", "json", required=False),
            field("T_chassis_camera", "外参 T_chassis_camera", "json", required=False),
            field("T_unit", "平移单位", "select", default="m"),
            field("depth_unit", "深度单位", "select", default="mm"),
            field(
                "camera_frame",
                "相机坐标系",
                default="head_camera_color_optical_frame",
            ),
            field("base_frame", "基坐标系", default="chassis_link"),
            field("front_rule", "前排规则", "json", required=False),
        ),
    ),
    capability(
        "estimation.basket_pose",
        "篮筐定位",
        "Estimation",
        "使用 /infer 定位篮筐",
        lambda app, ctx, data: _module(app, "estimation.basket_pose").estimate_basket_pose(
            _pick_pose_request({**data, "target_type": "basket"})
        ),
        (
            field("test_case", "测试用例", "select", required=False),
            field("rgb", "RGB 路径", required=False),
            field("depth", "深度路径", required=False),
            field("K", "相机内参 K", "json", required=False),
            field("T_chassis_camera", "外参 T_chassis_camera", "json", required=False),
            field("T_unit", "平移单位", "select", default="m"),
            field("depth_unit", "深度单位", "select", default="mm"),
            field("camera_frame", "相机坐标系", default="head_camera_color_optical_frame"),
            field("base_frame", "基坐标系", default="chassis_link"),
        ),
    ),
    capability(
        "manipulation.pick",
        "标准抓取",
        "Manipulation",
        "抓取 SKU",
        lambda app, ctx, data: _module(app, "manipulation.pick").pick(
            PickRequest(
                TaskType(data["task_type"]),
                TargetType(data["target_type"]),
                data["sku_typ"],
                _hand(data),
                data["level"],
                data["localization_result"],
            ),
            idempotency_key=action_id(ctx, "manipulation.pick"),
        ),
        (
            field("task_type", "任务类型", default="SORTING"),
            field("target_type", "目标类型", default="sku"),
            field("sku_typ", "定位类别", default="bottle", options=["bottle", "box", "tube"]),
            HAND,
            field("level", "抓取层级", "select"),
            field("localization_result", "完整定位结果", "json"),
        ),
        physical=True,
    ),
    capability(
        "manipulation.pick_review_item",
        "复核商品抓取",
        "Manipulation",
        "抓取任意商品",
        lambda app, ctx, data: _module(app, "manipulation.pick_review_item").pick_review_item(
            ReviewItemPickRequest(_pose(data), _hand(data)),
            idempotency_key=action_id(ctx, "manipulation.pick_review_item"),
        ),
        (HAND, POSE, *POSE_META),
        physical=True,
    ),
    capability(
        "manipulation.place",
        "放置",
        "Manipulation",
        "放置商品或篮筐",
        lambda app, ctx, data: _module(app, "manipulation.place").place(
            PlaceRequest(
                TaskType(data["task_type"]),
                TargetType(data["target_type"]),
                DestinationType(data["destination_type"]),
                _hand(data),
                _pose(data) if data.get("pose") else None,
                None,
                data.get("sku_typ") or None,
                data.get("localization_result") or None,
            ),
            idempotency_key=action_id(ctx, "manipulation.place"),
        ),
        (
            field("task_type", "任务类型", default="REVIEW"),
            field("target_type", "目标类型", default="sku"),
            field("destination_type", "目的地", default="table"),
            HAND,
            field("sku_typ", "定位类别", "select", required=False),
            field("localization_result", "完整定位结果", "json", required=False),
            field("pose", "位姿", "json", required=False),
            *POSE_META,
        ),
        physical=True,
    ),
    capability(
        "manipulation.rotate",
        "旋转商品",
        "Manipulation",
        "旋转重识别",
        lambda app, ctx, data: _module(app, "manipulation.rotate").rotate(
            _hand(data),
            data["sku_typ"],
            idempotency_key=action_id(ctx, "manipulation.rotate"),
        ),
        (
            HAND,
            field(
                "sku_typ",
                "定位类别",
                "select",
                default="bottle",
                options=["bottle", "box", "tube"],
            ),
        ),
        physical=True,
    ),
    capability(
        "manipulation.push",
        "推动篮筐",
        "Manipulation",
        "推动篮筐",
        lambda app, ctx, data: _module(app, "manipulation.push").push(
            PushRequest(_hand(data), data["localization_result"]),
            idempotency_key=action_id(ctx, "manipulation.push"),
        ),
        (
            HAND,
            field(
                "localization_result",
                "篮筐定位结果",
                "json",
                default={
                    "ok": True,
                    "target_type": "basket",
                    "pose_valid": True,
                    "point_semantics": "basket_model_center",
                    "model_center_camera_mm": [200.0, 30.0, 520.0],
                    "pose_4x4": [
                        [1.0, 0.0, 0.0, 50.0],
                        [0.0, 1.0, 0.0, 10.0],
                        [0.0, 0.0, 1.0, 400.0],
                        [0.0, 0.0, 0.0, 1.0],
                    ],
                },
            ),
        ),
        physical=True,
    ),
    capability(
        "vla.pick_review_item",
        "VLA 抓取",
        "VLA",
        "抓取任意商品",
        lambda app, ctx, data: _module(app, "vla.pick_review_item").pick_review_item(
            VlaPickRequest(_hand(data)),
            idempotency_key=action_id(ctx, "vla.pick_review_item"),
        ),
        (HAND,),
        physical=True,
    ),
    capability(
        "hand.pick",
        "灵巧手抓取",
        "Hand",
        "抓取 SKU",
        lambda app, ctx, data: _module(app, "hand.pick").pick(
            HandPickRequest(TaskType.SORTING, TargetType.SKU, data["sku_id"], _hand(data)),
            idempotency_key=action_id(ctx, "hand.pick", data["sku_id"]),
        ),
        (field("sku_id", "SKU", default=DEFAULT_SKU_ID), HAND),
        physical=True,
    ),
)

SKILL_OPERATIONS = (
    skill(
        "navigate",
        "导航",
        "基础动作",
        "导航 Skill",
        lambda data: NavigateInput(data["target_id"]),
        (field("target_id", "导航点", options=sorted(NAVIGATION_TARGETS)),),
        physical=True,
    ),
    skill(
        "prepare_pose",
        "准备位姿",
        "基础动作",
        "Pose Skill",
        lambda data: PreparePoseInput(data["pose_type"], data.get("level") or None),
        (field("pose_type", "姿态"), field("level", "层级", required=False)),
        physical=True,
    ),
    skill(
        "recognize_and_verify_barcode",
        "识别条码",
        "通用视觉",
        "识别并可选校验",
        lambda data: RecognizeBarcodeInput(
            data.get("expected_sku_id") or None,
            data.get("name") or None,
            _hand(data),
            data["sku_typ"],
        ),
        (
            field("expected_sku_id", "期望 SKU", required=False),
            field("name", "商品名", required=False),
            HAND,
            field(
                "sku_typ",
                "定位类别",
                "select",
                default="bottle",
                options=["bottle", "box", "tube"],
            ),
        ),
        physical=True,
    ),
    skill(
        "pick_sku_standard",
        "标准抓取",
        "Sorting",
        "直接识别估姿抓取",
        lambda data: PickSkuStandardInput(
            data["sku_id"], data["name"], data["side"], _hand(data), data["level"]
        ),
        (
            field("sku_id", "SKU", default=DEFAULT_SKU_ID),
            field("name", "名称", default=DEFAULT_SKU_NAME),
            field("side", "纸箱侧", default="LEFT"),
            HAND,
            field("level", "抓取层级", "select"),
        ),
        physical=True,
    ),
    skill(
        "pick_sku_hand",
        "灵巧手抓取",
        "Sorting",
        "端到端抓取",
        lambda data: PickSkuHandInput(data["sku_id"], _hand(data)),
        (field("sku_id", "SKU", default=DEFAULT_SKU_ID), HAND),
        physical=True,
    ),
    skill(
        "place_sku_in_basket",
        "商品入筐",
        "Sorting",
        "动态估姿放置",
        lambda data: PlaceSkuInBasketInput(data["sku_typ"], _hand(data)),
        (field("sku_typ", "定位类别", "select", default="bottle", options=["bottle", "box", "tube"]), HAND),
        physical=True,
    ),
    skill(
        "push_basket",
        "推筐",
        "Sorting",
        "定位估姿推筐",
        lambda data: PushBasketInput(_hand(data)),
        (HAND,),
        physical=True,
    ),
    *(
        skill(
            name,
            title,
            "Review",
            title,
            lambda data: HandOnlyInput(_hand(data)),
            (HAND,),
            physical=True,
        )
        for name, title in (
            ("pick_review_basket", "抓取复核篮筐"),
            ("place_review_basket", "放置复核篮筐"),
            ("pick_review_item_standard", "标准抓取复核商品"),
            ("pick_review_item_vla", "VLA 抓取复核商品"),
            ("place_review_item", "放置复核商品"),
        )
    ),
    skill(
        "confirm_review_basket_empty",
        "确认空筐",
        "Review",
        "新帧确认空筐",
        lambda data: None,
    ),
    skill(
        "summarize_review_result",
        "汇总复核",
        "Review",
        "比较期望与实际",
        lambda data: SummarizeReviewInput(
            tuple(
                ReviewItemCount(item["sku_id"], int(item["count"]))
                for item in data["expected_items"]
            ),
            tuple(
                InspectedItem(int(item["sequence"]), item["actual_sku_id"])
                for item in data["inspected_items"]
            ),
        ),
        (
            field(
                "expected_items",
                "期望",
                "json",
                default=[{"sku_id": DEFAULT_SKU_ID, "count": 1}],
            ),
            field(
                "inspected_items",
                "实检",
                "json",
                default=[{"sequence": 1, "actual_sku_id": DEFAULT_SKU_ID}],
            ),
        ),
    ),
)

BASKET_FIELDS = (
    field("basket_row", "篮筐行", default="L1"),
    field("basket_column", "篮筐列", default="1"),
)
WORKFLOW_OPERATIONS = (
    workflow(
        "sorting_item",
        "Sorting 单件商品",
        "Sorting",
        "完整单件拣选",
        lambda data: SortingItemInput(
            data["sku_id"],
            data["name"],
            data["agv_row"],
            data["agv_column"],
            data["basket_row"],
            data["basket_column"],
        ),
        (
            field("sku_id", "SKU", default=DEFAULT_SKU_ID),
            field("name", "名称", default=DEFAULT_SKU_NAME),
            field("agv_row", "AGV 行", default="L1"),
            field("agv_column", "AGV 列", default="1"),
            *BASKET_FIELDS,
        ),
    ),
    workflow(
        "sorting_finish",
        "Sorting 收尾",
        "Sorting",
        "推筐收尾",
        lambda data: SortingFinishInput(data["basket_row"], data["basket_column"]),
        BASKET_FIELDS,
    ),
    workflow(
        "review",
        "Review 复核",
        "Review",
        "搬筐逐件复核",
        lambda data: ReviewInput(
            str(data.get("order_id") or "debug-order"),
            data["basket_row"],
            data["basket_column"],
            tuple(
                ReviewItemCount(item["sku_id"], int(item["count"]))
                for item in data["expected_items"]
            ),
        ),
        (*BASKET_FIELDS, field("expected_items", "期望商品", "json", default=[])),
    ),
)

OPERATIONS = CAPABILITY_OPERATIONS + SKILL_OPERATIONS + WORKFLOW_OPERATIONS
OPERATIONS_BY_KEY = {(item.layer, item.name): item for item in OPERATIONS}


def catalog_payload() -> dict[str, Any]:
    items = []
    for item in OPERATIONS:
        payload = item.payload()
        if item.layer == "capability" and item.name in {
            "estimation.pick_pose",
            "estimation.basket_pose",
        }:
            # 测试用例选项由本地目录运行时扫描生成；无用例时哨兵兜底。
            options: list[Any] = scan_test_cases() or [TEST_CASE_SENTINEL]
            payload["fields"] = [
                {**entry, "type": "select", "options": options}
                if entry["name"] == "test_case"
                else entry
                for entry in payload["fields"]
            ]
        items.append(payload)
    return {
        "items": items,
        "counts": {
            layer: sum(item.layer == layer for item in OPERATIONS)
            for layer in ("capability", "skill", "workflow")
        },
    }
