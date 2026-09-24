from typing import Any

from agent.capabilities.common import HealthResult, HealthStatus
from agent.capabilities.mocks.delay import pause_mock_processing

from .contract import BasketPoseResult, PickPoseRequest, PickPoseResult

# 实测录制响应（bottle/Avene, 帧 120045958, side=RIGHT）；
RECORDED_INFER_RESPONSE: dict[str, Any] = {
    "ok": True,
    "target_type": "sku",
    "sku_typ": "bottle",
    "class_name": "bottle",
    "recognition_mode": "direct",
    "sam3_call_count": 2,
    "localization_method": "avene_bottle_axis",
    "selected_instance_id": 3,
    "upstream_instance_id": 3,
    "sam3_score": 0.752156138420105,
    "front_panel_valid": True,
    "front_panel_top_edge_midpoint_camera_mm": [100.0, -40.0, 650.0],
    "front_panel_plane_point_camera_mm": [100.0, -20.0, 600.0],
    "front_panel_plane_normal_camera": [1.0, 0.0, 0.0],
    "axis_fit_valid": True,
    "reference_point_valid": True,
    "reference_mode": "visible_axis_midpoint",
    "axis_point_camera_mm": [171.384239628949, -183.304701396738, 441.52773011806],
    "axis_direction_camera_up": [0.037441295923, -0.917692728411, -0.3955226992],
    "reference_point_camera_mm": [162.778093460864, 27.633450272577, 532.441414545861],
    "reference_point_chassis_mm": [574.681812178851, -169.314441068786, 1199.99999999972],
    "reference_z_mm": None,
    "output_frame": "head_camera_color_optical_frame",
    "output_unit": "mm",
    "side_audit": {
        "target_type": "sku",
        "sku_typ": "bottle",
        "side_received": "RIGHT",
        "side_applied": False,
        "side_note": "ignored_for_direct_mode",
    },
    "rejection_reasons": [],
}

# box/tube 无录制数据；结构符合 §6 的合成响应，仅联调用。
MOCK_ESTEE_INFER_RESPONSE: dict[str, Any] = {
    "ok": True,
    "target_type": "sku",
    "sku_typ": "box",
    "sam3_call_count": 2,
    "localization_method": "estee_top_plane",
    "top_plane_valid": True,
    "top_point_valid": True,
    "top_point_camera_mm": [120.0, 35.0, 510.0],
    "output_frame": "head_camera_color_optical_frame",
    "output_unit": "mm",
    "rejection_reasons": [],
}

MOCK_ORIGINS_INFER_RESPONSE: dict[str, Any] = {
    "ok": True,
    "target_type": "sku",
    "sku_typ": "tube",
    "sam3_call_count": 2,
    "localization_method": "origins_visible_top_edge",
    "edge_valid": True,
    "point_valid": True,
    "point_semantics": "visible_top_edge_midpoint",
    "top_edge_center_camera_mm": [95.0, 40.0, 505.0],
    "top_edge_endpoints_camera_mm": [[70.0, 42.0, 505.0], [120.0, 38.0, 505.0]],
    "edge_direction_camera": [1.0, 0.0, 0.0],
    "output_frame": "head_camera_color_optical_frame",
    "output_unit": "mm",
    "rejection_reasons": [],
}

_MOCK_RESPONSES = {
    "box": MOCK_ESTEE_INFER_RESPONSE,
    "tube": MOCK_ORIGINS_INFER_RESPONSE,
}

# §6.1 合成响应，仅联调用；数值无实测意义。
MOCK_BASKET_INFER_RESPONSE: dict[str, Any] = {
    "ok": True,
    "target_type": "basket",
    "pose_valid": True,
    "point_semantics": "basket_model_center",
    "model_center_camera_mm": [200.0, 30.0, 520.0],
    "reference_point_camera_mm": [200.0, 30.0, 520.0],
    "reference_point_chassis_mm": [500.0, -100.0, 800.0],
    "xyz_camera_mm": [200.0, 30.0, 520.0],
    "object_origin_camera_mm": [50.0, 10.0, 400.0],
    "model_center_offset_m": [0.15, 0.02, 0.12],
    "pose_4x4": [
        [1.0, 0.0, 0.0, 50.0],
        [0.0, 1.0, 0.0, 10.0],
        [0.0, 0.0, 1.0, 400.0],
        [0.0, 0.0, 0.0, 1.0],
    ],
    "pose_4x4_input_m": [
        [1.0, 0.0, 0.0, 0.05],
        [0.0, 1.0, 0.0, 0.01],
        [0.0, 0.0, 1.0, 0.4],
        [0.0, 0.0, 0.0, 1.0],
    ],
    "rotation_euler_zyx_rad": [0.0, 0.0, 0.0],
    "xyzrxryrz_camera_mm_rad": [200.0, 30.0, 520.0, 0.0, 0.0, 0.0],
    "output_frame": "head_camera_color_optical_frame",
    "output_unit": "mm",
    "rejection_reasons": [],
}


def mock_infer_response(sku_typ: str | None = None, *, target_type: str = "sku") -> dict[str, Any]:
    if target_type == "basket":
        return dict(MOCK_BASKET_INFER_RESPONSE)
    return dict(_MOCK_RESPONSES.get(sku_typ or "", RECORDED_INFER_RESPONSE))


class MockEstimationCapability:
    """进程内 mock：/infer 形态返回录制/合成响应"""

    def __init__(self) -> None:
        self.requests: list[Any] = []

    def health(self) -> HealthResult:
        pause_mock_processing()
        return HealthResult(HealthStatus.READY)

    def estimate_pick_pose(self, request: PickPoseRequest) -> PickPoseResult:
        pause_mock_processing()
        self.requests.append(request)
        return PickPoseResult.from_payload(
            mock_infer_response(request.sku_typ, target_type=request.target_type.value)
        )

    def estimate_basket_pose(self, request: PickPoseRequest) -> BasketPoseResult:
        pause_mock_processing()
        self.requests.append(request)
        return BasketPoseResult.from_payload(
            mock_infer_response(target_type=request.target_type.value)
        )
