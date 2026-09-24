from typing import Any

from agent.capabilities.common import CapabilityError, HealthResult, HealthStatus, TargetType
from agent.capabilities.http import HttpCapabilityClient

from .contract import BasketPoseResult, PickPoseRequest, PickPoseResult


class HttpEstimationCapability:
    def __init__(self, client: HttpCapabilityClient):
        self.client = client

    def health(self) -> HealthResult:
        payload = self.client.get("/health")
        status = payload.get("status") if isinstance(payload, dict) else None
        try:
            return HealthResult(HealthStatus(status))
        except ValueError:
            return HealthResult(HealthStatus.READY)

    def estimate_pick_pose(self, request: PickPoseRequest) -> PickPoseResult:
        return PickPoseResult.from_payload(
            self.client.post("/infer", _infer_body(request))
        )

    def estimate_basket_pose(self, request: PickPoseRequest) -> BasketPoseResult:
        if request.target_type is not TargetType.BASKET:
            raise ValueError("basket pose inference requires target_type=basket")
        return BasketPoseResult.from_payload(self.client.post("/infer", _infer_body(request)))

def _infer_body(request: PickPoseRequest) -> dict[str, Any]:
    """ /infer 请求体"""
    body: dict[str, Any] = {
        "target_type": request.target_type.value,
        "rgb_base64": _wire_base64("rgb_base64", request.rgb_base64),
        "depth_npy_base64": _wire_base64("depth_npy_base64", request.depth_npy_base64),
        "depth_unit": request.depth_unit,
        "K": [list(row) for row in request.K],
        "T_chassis_camera": [list(row) for row in request.T_chassis_camera],
        "T_unit": request.T_unit,
        "camera_frame": request.camera_frame,
        "base_frame": request.base_frame,
    }
    if request.target_type is TargetType.SKU:
        body["sku_typ"] = request.sku_typ
        body["side"] = request.side
        if request.front_rule:
            body["front_rule"] = dict(request.front_rule)
    return body


def _wire_base64(name: str, value: str) -> str:
    stripped = value.strip()
    if stripped.startswith("<") and stripped.endswith(">"):
        raise CapabilityError(
            "PICK_POSE_PLACEHOLDER_INPUT",
            f"{name} 仍是占位符；请替换测试用例中的调试数据为真实帧字节",
        )
    return value
