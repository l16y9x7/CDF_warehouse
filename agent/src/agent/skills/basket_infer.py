import base64

from agent.capabilities.camera import CameraCapability, CameraStream, capture_with_event
from agent.capabilities.common import TargetType
from agent.capabilities.estimation import BasketPoseResult, EstimationCapability, PickPoseRequest
from agent.capabilities.pose import BodyPoseCapability
from agent.contracts import ExecutionContext
from agent.skills.pick_sku_standard import _float32_depth_base64, _rgbd_frames


def infer_head_basket_pose(
    context: ExecutionContext,
    camera: CameraCapability,
    estimation: EstimationCapability,
    pose: BodyPoseCapability,
    skill: str,
) -> BasketPoseResult:
    transform = pose.camera_transform("head")
    capture = capture_with_event(
        context,
        camera,
        "head",
        (CameraStream.COLOR, CameraStream.DEPTH),
        skill=skill,
    )
    color, depth = _rgbd_frames(capture)
    if capture.color_intrinsics is None or capture.depth_unit != "mm":
        raise ValueError(
            "head RGB-D capture is missing current intrinsics or millimeter depth unit"
        )
    rgb_base64 = base64.b64encode(camera.read_frame_bytes(color)).decode("ascii")
    depth_base64 = _float32_depth_base64(
        camera.read_frame_bytes(depth), expected_shape=(depth.height, depth.width)
    )
    return estimation.estimate_basket_pose(
        PickPoseRequest(
            TargetType.BASKET,
            None,
            rgb_base64,
            depth_base64,
            capture.color_intrinsics,
            transform.T_chassis_camera,
            camera_frame=transform.camera_frame,
            base_frame=transform.base_frame,
            side=None,
            T_unit=transform.t_unit,
        )
    )
