import json
import unittest

import httpx

from agent.capabilities.camera import CameraStream, DepthFormat, HttpCameraCapability
from agent.capabilities.common import (
    CapabilityError,
    DestinationType,
    ErrorSource,
    Hand,
    TargetType,
    TaskType,
)
from agent.capabilities.estimation import (
    HttpEstimationCapability,
    PickPoseRequest as PickPoseRequestContract,
)
from agent.capabilities.hand import HandPickRequest, HttpHandCapability
from agent.capabilities.http import IDEMPOTENCY_HEADER, HttpCapabilityClient
from agent.capabilities.manipulation import (
    BasketPickRequest,
    HttpManipulationCapability,
    PickRequest,
    PlaceRequest,
    PushRequest,
)
from agent.capabilities.navigation import HttpNavigationCapability
from agent.capabilities.perception import (
    HttpPerceptionCapability,
    ImageRequest,
    LocateStatus,
    RecognizeBarcodeRequest,
)
from agent.capabilities.pose import HttpBodyPoseCapability
from agent.capabilities.vla import HttpVlaCapability, VlaPickRequest

K = (
    (612.772339587192, 0.0, 641.544745101529),
    (0.0, 611.998287842651, 358.604256964916),
    (0.0, 0.0, 1.0),
)
T_CHASSIS_CAMERA = (
    (-0.047653753369, -0.428149456867, 0.902450642625, 0.113767949307),
    (-0.998816369947, 0.011610119563, -0.04723414285, 0.018099569521),
    (0.009745712746, -0.903633359117, -0.428195952077, 1.45137337738),
    (0.0, 0.0, 0.0, 1.0),
)

FRONT_RULE = {
    "front_axis_chassis": [1.0, 0.0, 0.0],
    "front_origin_chassis": [0.0, 0.0, 0.0],
    "front_band_mm": 100000.0,
}


def PickPoseRequest(*args, **kwargs):
    """Test builder with explicit standard frame metadata; production has no defaults."""
    kwargs.setdefault("camera_frame", "head_camera_color_optical_frame")
    kwargs.setdefault("base_frame", "chassis_link")
    return PickPoseRequestContract(*args, **kwargs)

class CapabilityContractTest(unittest.TestCase):
    def setUp(self):
        self.requests = []
        self.infer_response: dict = {
            "ok": True,
            "sku_typ": "bottle",
            "selected_instance_id": 3,
            "side_audit": {"side_received": "LEFT", "side_applied": False},
            "axis_fit_valid": True,
            "reference_point_valid": True,
            "reference_point_camera_mm": [
                162.778093460864,
                27.633450272577,
                532.441414545861,
            ],
            "axis_direction_camera_up": [0.037441295923, -0.917692728411, -0.3955226992],
            "rejection_reasons": [],
        }

        def respond(request):
            self.requests.append(request)
            if request.url.path == "/perception/recognize_sku_barcode":
                return httpx.Response(200, json={"barcode_content": "sku-1"})
            if request.url.path.startswith("/perception/"):
                return httpx.Response(
                    200, json={"status": "FOUND", "bbox": [1, 2, 3, 4], "mask": "m"}
                )
            if request.url.path == "/infer":
                return httpx.Response(200, json=self.infer_response)
            return httpx.Response(
                200,
                json={
                    "pose": [1, 2, 3, 4, 5, 6],
                    "corners_mm": [[0, 0, 0]] * 8,
                    "frame": "camera",
                    "pose_unit": "mm_rad",
                    "rotation_order": "zyx",
                },
            )

        raw = httpx.Client(transport=httpx.MockTransport(respond), base_url="http://module")
        self.addCleanup(raw.close)
        client = HttpCapabilityClient("http://module", client=raw)
        self.perception, self.estimation = (
            HttpPerceptionCapability(client),
            HttpEstimationCapability(client),
        )

    def test_barcode_metadata_is_sent_as_empty_strings_when_missing(self):
        self.perception.recognize_sku_barcode(RecognizeBarcodeRequest("aW1hZ2U="))
        self.perception.recognize_sku_barcode(
            RecognizeBarcodeRequest("aW1hZ2U=", "sku-1", "name")
        )
        self.perception.recognize_sku_barcode(
            RecognizeBarcodeRequest("aW1hZ2U=", "sku-1")
        )
        self.assertEqual(
            json.loads(self.requests[0].read()),
            {"image_base64": "aW1hZ2U=", "sku_id": "", "name": ""},
        )
        self.assertEqual(
            json.loads(self.requests[1].read()),
            {"image_base64": "aW1hZ2U=", "sku_id": "sku-1", "name": "name"},
        )
        self.assertEqual(
            json.loads(self.requests[2].read()),
            {"image_base64": "aW1hZ2U=", "sku_id": "sku-1", "name": ""},
        )

    def test_locate_contract_uses_found_not_found(self):
        result = self.perception.locate_basket_item(ImageRequest("/x.jpg"))
        self.assertIs(result.status, LocateStatus.FOUND)
        self.assertEqual(result.detection().mask, "m")

    def test_infer_pick_pose_sends_contract_body_and_parses_response(self):
        front_rule = {
            "front_axis_chassis": [1.0, 0.0, 0.0],
            "front_origin_chassis": [0.0, 0.0, 0.0],
            "front_band_mm": 100000.0,
        }
        request = PickPoseRequest(
            TargetType.SKU,
            "bottle",
            "cmdi",
            "bm9weQ==",
            K,
            T_CHASSIS_CAMERA,
            side="LEFT",
            front_rule=front_rule,
            camera_frame="runtime_camera_optical_frame",
            base_frame="runtime_robot_base",
        )
        result = self.estimation.estimate_pick_pose(request)
        self.assertEqual(self.requests[-1].url.path, "/infer")
        self.assertEqual(
            json.loads(self.requests[-1].read()),
            {
                "target_type": "sku",
                "sku_typ": "bottle",
                "rgb_base64": "cmdi",
                "depth_npy_base64": "bm9weQ==",
                "depth_unit": "mm",
                "K": [list(row) for row in K],
                "T_chassis_camera": [list(row) for row in T_CHASSIS_CAMERA],
                "T_unit": "m",
                "camera_frame": "runtime_camera_optical_frame",
                "base_frame": "runtime_robot_base",
                "side": "LEFT",
                "front_rule": front_rule,
            },
        )
        self.assertTrue(result.ok)
        self.assertEqual(
            result.raw["side_audit"], {"side_received": "LEFT", "side_applied": False}
        )
        self.assertNotIn("ok", result.raw)  # 文档字段不重复进 raw
        self.assertEqual(result.selected_instance_id, 3)
        self.assertEqual(
            result.reference_point_camera_mm,
            (162.778093460864, 27.633450272577, 532.441414545861),
        )
        self.assertEqual(result.sku_typ, "bottle")
        self.assertEqual(result.localization_result["sku_typ"], "bottle")

    def test_infer_invalid_frame_parses_without_raising(self):
        self.infer_response["reference_point_valid"] = False
        self.infer_response["rejection_reasons"] = ["axis_span_too_short"]
        result = self.estimation.estimate_pick_pose(
            PickPoseRequest(
                TargetType.SKU,
                "bottle",
                "cmdi",
                "bm9weQ==",
                K,
                T_CHASSIS_CAMERA,
                side="RIGHT",
                front_rule=FRONT_RULE,
            )
        )
        # 能力层只忠实解析；§6 门由消费方对照（skill 内 TODO 待引入）
        self.assertTrue(result.ok)
        self.assertIs(result.reference_point_valid, False)
        self.assertEqual(result.rejection_reasons, ("axis_span_too_short",))

    def test_infer_non_table_fields_land_in_raw(self):
        self.infer_response = {
            "ok": True,
            "sku_typ": "box",
            "top_plane_valid": True,
            "top_point_valid": True,
            "top_point_camera_mm": [120.0, 35.0, 510.0],
        }
        result = self.estimation.estimate_pick_pose(
            PickPoseRequest(
                TargetType.SKU,
                "box",
                "cmdi",
                "bm9weQ==",
                K,
                T_CHASSIS_CAMERA,
                side="RIGHT",
                front_rule=FRONT_RULE,
            )
        )
        self.assertEqual(result.sku_typ, "box")
        self.assertEqual(result.raw["top_point_camera_mm"], [120.0, 35.0, 510.0])
        self.infer_response = {
            "ok": True,
            "sku_typ": "tube",
            "edge_valid": True,
            "point_valid": True,
            "point_semantics": "visible_top_edge_midpoint",
            "top_edge_center_camera_mm": [95.0, 40.0, 505.0],
            "top_edge_endpoints_camera_mm": [[70.0, 42.0, 505.0], [120.0, 38.0, 505.0]],
        }
        result = self.estimation.estimate_pick_pose(
            PickPoseRequest(
                TargetType.SKU,
                "tube",
                "cmdi",
                "bm9weQ==",
                K,
                T_CHASSIS_CAMERA,
                side="LEFT",
                front_rule=FRONT_RULE,
            )
        )
        self.assertEqual(
            result.raw["top_edge_endpoints_camera_mm"],
            [[70.0, 42.0, 505.0], [120.0, 38.0, 505.0]],
        )
        self.assertEqual(result.raw["point_semantics"], "visible_top_edge_midpoint")

    def test_infer_rejects_placeholder_base64_before_sending(self):
        with self.assertRaises(CapabilityError) as raised:
            self.estimation.estimate_pick_pose(
                PickPoseRequest(
                    TargetType.SKU,
                    "bottle",
                    "<BASE64_OF_RGB_JPG_OR_PNG_FILE_BYTES>",
                    "bm9weQ==",
                    K,
                    T_CHASSIS_CAMERA,
                    side="RIGHT",
                    front_rule=FRONT_RULE,
                )
            )
        self.assertEqual(raised.exception.error_code, "PICK_POSE_PLACEHOLDER_INPUT")
        self.assertEqual(raised.exception.source, ErrorSource.LOCAL)
        self.assertEqual(self.requests, [])

    def test_infer_response_missing_keys_parse_as_none(self):
        self.infer_response = {"ok": False}
        result = self.estimation.estimate_pick_pose(
            PickPoseRequest(
                TargetType.SKU,
                "bottle",
                "cmdi",
                "bm9weQ==",
                K,
                T_CHASSIS_CAMERA,
                side="RIGHT",
                front_rule=FRONT_RULE,
            )
        )
        self.assertFalse(result.ok)
        self.assertIsNone(result.axis_fit_valid)
        self.assertIsNone(result.reference_point_camera_mm)
        self.assertEqual(result.rejection_reasons, ())
        self.assertEqual(result.raw, {})

    def test_infer_omits_front_rule_when_absent(self):
        result = self.estimation.estimate_pick_pose(
            PickPoseRequest(
                TargetType.SKU,
                "bottle",
                "cmdi",
                "bm9weQ==",
                K,
                T_CHASSIS_CAMERA,
                side="RIGHT",
            )
        )
        self.assertTrue(result.ok)
        body = json.loads(self.requests[-1].read())
        self.assertNotIn("front_rule", body)
        self.assertEqual(body["sku_typ"], "bottle")

    def test_infer_basket_sends_frame_only_and_parses_response(self):
        self.infer_response = {
            "ok": True,
            "target_type": "basket",
            "pose_valid": True,
            "point_semantics": "basket_model_center",
            "model_center_camera_mm": [200.0, 30.0, 520.0],
            "reference_point_camera_mm": [200.0, 30.0, 520.0],
            "pose_4x4": [
                [1.0, 0.0, 0.0, 50.0],
                [0.0, 1.0, 0.0, 10.0],
                [0.0, 0.0, 1.0, 400.0],
                [0.0, 0.0, 0.0, 1.0],
            ],
        }
        result = self.estimation.estimate_basket_pose(
            PickPoseRequest(
                TargetType.BASKET,
                None,
                "cmdi",
                "bm9weQ==",
                K,
                T_CHASSIS_CAMERA,
                side=None,
            )
        )
        self.assertEqual(self.requests[-1].url.path, "/infer")
        body = json.loads(self.requests[-1].read())
        self.assertEqual(body["target_type"], "basket")
        self.assertNotIn("sku_typ", body)
        self.assertNotIn("side", body)
        self.assertNotIn("front_rule", body)
        self.assertTrue(result.ok)
        self.assertEqual(result.reference_point_camera_mm, (200.0, 30.0, 520.0))
        self.assertIs(result.pose_valid, True)
        self.assertEqual(result.point_semantics, "basket_model_center")

    def test_pick_pose_request_validates_service_schema(self):
        with self.assertRaises(TypeError):  # frame metadata 必须由调用方显式提供
            PickPoseRequestContract(
                TargetType.SKU,
                "bottle",
                "cmdi",
                "bm9weQ==",
                K,
                T_CHASSIS_CAMERA,
                side="RIGHT",
            )
        with self.assertRaises(ValueError):  # side 只收 LEFT/RIGHT
            PickPoseRequest(
                TargetType.SKU,
                "box",
                "cmdi",
                "bm9weQ==",
                K,
                T_CHASSIS_CAMERA,
                side="UP",
                front_rule=FRONT_RULE,
            )
        request = PickPoseRequest(  # sku_typ 透传，不限定 bottle/box/tube
            TargetType.SKU,
            "sku-1",
            "cmdi",
            "bm9weQ==",
            K,
            T_CHASSIS_CAMERA,
            side="LEFT",
            front_rule=FRONT_RULE,
        )
        self.assertEqual(request.sku_typ, "sku-1")
        with self.assertRaises(ValueError):  # sku_typ 不能为空
            PickPoseRequest(
                TargetType.SKU,
                "  ",
                "cmdi",
                "bm9weQ==",
                K,
                T_CHASSIS_CAMERA,
                side="LEFT",
                front_rule=FRONT_RULE,
            )
        with self.assertRaises(ValueError):  # basket 不接受 sku 专用字段
            PickPoseRequest(
                TargetType.BASKET,
                "bottle",
                "cmdi",
                "bm9weQ==",
                K,
                T_CHASSIS_CAMERA,
                side="RIGHT",
                front_rule=FRONT_RULE,
            )
        PickPoseRequest(  # basket 最小体：sku 字段必须显式为空
            TargetType.BASKET,
            None,
            "cmdi",
            "bm9weQ==",
            K,
            T_CHASSIS_CAMERA,
            side=None,
        )
        with self.assertRaises(ValueError):  # T_unit 只收 m/mm
            PickPoseRequest(
                TargetType.SKU,
                "bottle",
                "cmdi",
                "bm9weQ==",
                K,
                T_CHASSIS_CAMERA,
                T_unit="cm",
                side="RIGHT",
                front_rule=FRONT_RULE,
            )
        for field in ("camera_frame", "base_frame"):
            kwargs = {field: "  "}
            with self.subTest(field=field), self.assertRaises(ValueError):
                PickPoseRequest(
                    TargetType.SKU,
                    "bottle",
                    "cmdi",
                    "bm9weQ==",
                    K,
                    T_CHASSIS_CAMERA,
                    side="RIGHT",
                    front_rule=FRONT_RULE,
                    **kwargs,
                )
        broken = tuple(T_CHASSIS_CAMERA[:3]) + ((0.0, 0.0, 0.0, 2.0),)
        with self.assertRaises(ValueError):  # 外参必须是有效刚体变换
            PickPoseRequest(
                TargetType.SKU,
                "bottle",
                "cmdi",
                "bm9weQ==",
                K,
                broken,
                side="RIGHT",
                front_rule=FRONT_RULE,
            )
        with self.assertRaises(ValueError):  # front_rule 三键齐全
            PickPoseRequest(
                TargetType.SKU,
                "bottle",
                "cmdi",
                "bm9weQ==",
                K,
                T_CHASSIS_CAMERA,
                side="RIGHT",
                front_rule={"front_band_mm": 1.0},
            )

class BodyPoseTransformContractTest(unittest.TestCase):
    def adapter(self, payload):
        self.requests = []

        def respond(request):
            self.requests.append(request)
            return httpx.Response(200, json=payload)

        raw = httpx.Client(transport=httpx.MockTransport(respond), base_url="http://pose")
        self.addCleanup(raw.close)
        return HttpBodyPoseCapability(HttpCapabilityClient("http://pose", client=raw))

    @staticmethod
    def payload(**overrides):
        payload = {
            "T_chassis_camera": [list(row) for row in T_CHASSIS_CAMERA],
            "t_unit": "m",
            "camera": "head",
            "camera_frame": "runtime_camera_optical_frame",
            "base_frame": "runtime_robot_base",
        }
        payload.update(overrides)
        return payload

    def test_preserves_runtime_frame_metadata(self):
        transform = self.adapter(self.payload()).camera_transform("head")
        self.assertEqual(transform.camera_frame, "runtime_camera_optical_frame")
        self.assertEqual(transform.base_frame, "runtime_robot_base")
        self.assertEqual(dict(self.requests[0].url.params), {"camera": "head"})

    def test_rejects_missing_or_empty_frame_metadata(self):
        for field, value in (("camera_frame", None), ("base_frame", "  ")):
            payload = self.payload()
            if value is None:
                payload.pop(field)
            else:
                payload[field] = value
            with self.subTest(field=field), self.assertRaises(ValueError):
                self.adapter(payload).camera_transform("head")

    def test_rejects_response_for_a_different_camera(self):
        with self.assertRaisesRegex(ValueError, "does not match requested camera"):
            self.adapter(self.payload(camera="right_wrist")).camera_transform("head")


class CameraCapabilityContractTest(unittest.TestCase):
    def adapter(self, response):
        self.requests = []

        def respond(request):
            self.requests.append(request)
            return response(request) if callable(response) else httpx.Response(200, json=response)

        raw = httpx.Client(transport=httpx.MockTransport(respond), base_url="http://camera")
        self.addCleanup(raw.close)
        return HttpCameraCapability(HttpCapabilityClient("http://camera", client=raw))

    def payload(self, *, color=True, depth=True, same_shot=True, aligned=True):
        capture_id = "capture-002"
        base = f"/shared/frames/{capture_id}"
        return {
            "ok": True,
            "capture_id": capture_id,
            "camera": "right_wrist",
            "same_shot": same_shot,
            "color": (
                {"path": f"{base}/rgb.jpg", "format": "jpeg", "width": 1280, "height": 720}
                if color
                else None
            ),
            "depth": (
                {
                    "path": f"{base}/depth_mm.npy",
                    "format": "raw",
                    "width": 1280,
                    "height": 720,
                    "aligned": aligned,
                }
                if depth
                else None
            ),
        }

    def test_dual_capture_uses_only_capture_endpoint_and_one_request(self):
        camera = self.adapter(self.payload())
        result = camera.capture(
            "right_wrist", (CameraStream.DEPTH, CameraStream.COLOR), depth_format=DepthFormat.RAW
        )
        self.assertEqual(result.capture_id, "capture-002")
        self.assertEqual(result.color.path, "/shared/frames/capture-002/rgb.jpg")
        self.assertEqual(result.depth.path, "/shared/frames/capture-002/depth_mm.npy")
        self.assertEqual(len(self.requests), 1)
        self.assertEqual(self.requests[0].url.path, "/camera/capture")
        self.assertEqual(self.requests[0].url.params["streams"], "color,depth")

    def test_default_color_omits_optional_parameters(self):
        payload = self.payload(depth=False, same_shot=False)
        camera = self.adapter(payload)
        result = camera.capture("right_wrist")
        self.assertIsNone(result.depth)
        self.assertEqual(dict(self.requests[0].url.params), {"camera": "right_wrist"})

    def test_depth_preview_sends_format(self):
        payload = self.payload(color=False, same_shot=False)
        payload["depth"]["format"] = "preview"
        payload["depth"]["path"] = "/shared/frames/capture-002/depth.jpg"
        camera = self.adapter(payload)
        camera.capture(
            "right_wrist", (CameraStream.DEPTH,), depth_format=DepthFormat.PREVIEW
        )
        self.assertEqual(
            dict(self.requests[0].url.params),
            {"camera": "right_wrist", "streams": "depth", "format": "preview"},
        )

    def test_dual_capture_ignores_preview_format(self):
        camera = self.adapter(self.payload())
        result = camera.capture(
            "right_wrist",
            (CameraStream.COLOR, CameraStream.DEPTH),
            depth_format=DepthFormat.PREVIEW,
        )
        self.assertEqual(result.depth.format, "raw")
        self.assertEqual(
            dict(self.requests[0].url.params),
            {"camera": "right_wrist", "streams": "color,depth"},
        )

    def test_rejects_unaligned_depth_and_non_same_shot_pair(self):
        with self.assertRaises(CapabilityError) as unaligned:
            self.adapter(self.payload(aligned=False)).capture(
                "right_wrist", (CameraStream.COLOR, CameraStream.DEPTH)
            )
        self.assertEqual(unaligned.exception.error_code, "DEPTH_NOT_ALIGNED")
        self.assertEqual(unaligned.exception.source, ErrorSource.LOCAL)

        with self.assertRaises(CapabilityError) as split:
            self.adapter(self.payload(same_shot=False)).capture(
                "right_wrist", (CameraStream.COLOR, CameraStream.DEPTH)
            )
        self.assertEqual(split.exception.error_code, "CAPTURE_FAILED")
        self.assertEqual(split.exception.source, ErrorSource.LOCAL)

    def test_propagates_documented_error_payload(self):
        camera = self.adapter(
            lambda request: httpx.Response(
                503,
                json={
                    "ok": False,
                    "error_code": "CAMERA_NOT_READY",
                    "message": "camera not ready",
                    "camera": "left_wrist",
                },
            )
        )
        with self.assertRaises(CapabilityError) as raised:
            camera.capture("left_wrist")
        self.assertEqual(raised.exception.error_code, "CAMERA_NOT_READY")
        self.assertEqual(raised.exception.code, "CAMERA_NOT_READY")
        self.assertEqual(raised.exception.source, ErrorSource.REMOTE)
        self.assertEqual(raised.exception.status_code, 503)
        self.assertEqual(raised.exception.operation, "GET /camera/capture")

    def test_health_uses_health_endpoint(self):
        payload = self.payload(depth=False, same_shot=False)
        payload["camera"] = "head"
        payload["status"] = "READY"
        camera = self.adapter(payload)
        self.assertEqual(camera.health().status.value, "READY")
        self.assertEqual(self.requests[0].url.path, "/camera/health")

    def test_head_rgbd_uses_formal_endpoint_and_parses_current_intrinsics(self):
        capture_id = "capture-003"
        payload = {
            "ok": True,
            "camera": "head",
            "capture_id": capture_id,
            "rgb": f"/shared/frames/{capture_id}/rgb.jpg",
            "depth": f"/shared/frames/{capture_id}/depth_mm.npy",
            "t_unit": "mm",
            "same_shot": True,
            "color_intrinsics": {
                "width": 1280,
                "height": 720,
                "camera_matrix": [list(row) for row in K],
            },
        }
        camera = self.adapter(payload)
        result = camera.capture("head", (CameraStream.COLOR, CameraStream.DEPTH))
        self.assertEqual(self.requests[0].url.path, "/camera/rgbd")
        self.assertEqual(result.color_intrinsics, K)
        self.assertEqual(result.depth_unit, "mm")
        self.assertTrue(result.depth.aligned)

    def test_rejects_invalid_local_capture_parameters_without_request(self):
        camera = self.adapter(self.payload())
        for camera_id, streams in (
            ("unknown", (CameraStream.COLOR,)),
            ("head", ()),
            ("head", (CameraStream.COLOR, CameraStream.COLOR)),
        ):
            with self.subTest(camera=camera_id, streams=streams), self.assertRaises(ValueError):
                camera.capture(camera_id, streams)
        self.assertEqual(self.requests, [])


class PhysicalActionTimeoutTest(unittest.TestCase):
    def setUp(self):
        def timeout(request):
            raise httpx.ReadTimeout("response timed out", request=request)

        self.raw = httpx.Client(transport=httpx.MockTransport(timeout), base_url="http://module")
        self.addCleanup(self.raw.close)
        self.client = HttpCapabilityClient("http://module", client=self.raw)

    def assert_unknown(self, action):
        with self.assertRaises(CapabilityError) as raised:
            action()
        self.assertEqual(raised.exception.error_code, "ACTION_RESULT_UNKNOWN")
        self.assertEqual(raised.exception.source, ErrorSource.TRANSPORT)
        self.assertTrue((raised.exception.operation or "").startswith("POST "))

    def test_navigation_timeout_has_unknown_action_result(self):
        self.assert_unknown(lambda: HttpNavigationCapability(self.client).navigate("AGV_L"))

    def test_pose_timeout_has_unknown_action_result(self):
        self.assert_unknown(
            lambda: HttpBodyPoseCapability(self.client).prepare("basket_push", "L1")
        )

    def test_manipulation_timeout_has_unknown_action_result(self):
        self.assert_unknown(
            lambda: HttpManipulationCapability(self.client).rotate(Hand.RIGHT, "bottle")
        )

    def test_vla_timeout_has_unknown_action_result(self):
        self.assert_unknown(
            lambda: HttpVlaCapability(self.client).pick_review_item(VlaPickRequest(Hand.RIGHT))
        )

    def test_hand_timeout_has_unknown_action_result(self):
        self.assert_unknown(
            lambda: HttpHandCapability(self.client).pick(
                HandPickRequest(TaskType.SORTING, TargetType.SKU, "sku-1", Hand.RIGHT)
            )
        )

    def test_connect_timeout_remains_module_unavailable(self):
        def connect_timeout(request):
            raise httpx.ConnectTimeout("connect timed out", request=request)

        raw = httpx.Client(transport=httpx.MockTransport(connect_timeout), base_url="http://module")
        self.addCleanup(raw.close)
        client = HttpCapabilityClient("http://module", client=raw)

        with self.assertRaises(CapabilityError) as raised:
            HttpNavigationCapability(client).navigate("AGV_L")

        self.assertEqual(raised.exception.error_code, "MODULE_UNAVAILABLE")
        self.assertEqual(raised.exception.source, ErrorSource.TRANSPORT)
        self.assertEqual(raised.exception.operation, "POST /navigation/navigate")


class CapabilityErrorSourceTest(unittest.TestCase):
    def client(self, handler):
        raw = httpx.Client(transport=httpx.MockTransport(handler), base_url="http://module")
        self.addCleanup(raw.close)
        return HttpCapabilityClient("http://module", client=raw, capability="manipulation")

    def test_http_error_keeps_remote_code_and_operation(self):
        def respond(request):
            return httpx.Response(
                422,
                json={
                    "error_code": "INVALID_INPUT",
                    "message": "扫码支持 bottle/RIGHT 或 box/LEFT",
                },
            )

        with self.assertRaises(CapabilityError) as raised:
            HttpManipulationCapability(self.client(respond)).rotate(Hand.LEFT, "box")
        exc = raised.exception
        self.assertEqual(exc.error_code, "INVALID_INPUT")
        self.assertEqual(exc.code, "INVALID_INPUT")
        self.assertEqual(exc.source, ErrorSource.REMOTE)
        self.assertEqual(exc.status_code, 422)
        self.assertEqual(exc.operation, "POST /manipulation/rotate")

    def test_http_error_without_body_code_uses_fallback(self):
        def respond(request):
            return httpx.Response(500, json={"message": "boom"})

        with self.assertRaises(CapabilityError) as raised:
            self.client(respond).get("/manipulation/health")
        exc = raised.exception
        self.assertEqual(exc.error_code, "CAPABILITY_REQUEST_FAILED")
        self.assertEqual(exc.source, ErrorSource.REMOTE)
        self.assertEqual(exc.status_code, 500)
        self.assertEqual(exc.operation, "GET /manipulation/health")


class PhysicalActionIdempotencyTest(unittest.TestCase):
    def adapter(self, capability):
        self.requests = []

        def respond(request):
            self.requests.append(request)
            if request.url.path == "/manipulation/rotate":
                return httpx.Response(
                    200,
                    json={
                        "status": "SUCCEEDED",
                        "camera": "left_wrist",
                        "image_paths": [
                            f"/shared/frames/barcode-{index}/rgb.jpg"
                            for index in range(1, 6)
                        ],
                    },
                )
            return httpx.Response(200, json={"status": "SUCCEEDED"})

        raw = httpx.Client(transport=httpx.MockTransport(respond), base_url="http://module")
        self.addCleanup(raw.close)
        return capability(HttpCapabilityClient("http://module", client=raw))

    def test_navigation_sends_provided_and_generated_keys(self):
        navigation = self.adapter(HttpNavigationCapability)
        navigation.navigate("AGV_L", idempotency_key="task-1:navigate:AGV_L")
        self.assertEqual(self.requests[0].headers[IDEMPOTENCY_HEADER], "task-1:navigate:AGV_L")
        navigation.navigate("AGV_C")
        generated = self.requests[1].headers[IDEMPOTENCY_HEADER]
        self.assertTrue(generated)
        self.assertNotEqual(generated, "task-1:navigate:AGV_L")

    def test_other_physical_actions_send_header(self):
        pose = self.adapter(HttpBodyPoseCapability)
        pose.prepare("basket_push", "L1", idempotency_key="pose-key")
        self.assertEqual(self.requests[-1].headers[IDEMPOTENCY_HEADER], "pose-key")

        hand = self.adapter(HttpHandCapability)
        hand.pick(
            HandPickRequest(TaskType.SORTING, TargetType.SKU, "sku-1", Hand.RIGHT),
            idempotency_key="hand-key",
        )
        self.assertEqual(self.requests[-1].headers[IDEMPOTENCY_HEADER], "hand-key")

        self.requests.clear()

        def vla_respond(request):
            self.requests.append(request)
            return httpx.Response(200, json={"status": "PICKED"})

        raw = httpx.Client(transport=httpx.MockTransport(vla_respond), base_url="http://module")
        self.addCleanup(raw.close)
        HttpVlaCapability(HttpCapabilityClient("http://module", client=raw)).pick_review_item(
            VlaPickRequest(Hand.RIGHT), idempotency_key="vla-key"
        )
        self.assertEqual(self.requests[-1].headers[IDEMPOTENCY_HEADER], "vla-key")

        manipulation = self.adapter(HttpManipulationCapability)
        result = manipulation.rotate(Hand.RIGHT, "bottle", idempotency_key="rotate-key")
        self.assertEqual(self.requests[-1].headers[IDEMPOTENCY_HEADER], "rotate-key")
        self.assertEqual(
            json.loads(self.requests[-1].read()), {"hand": "RIGHT", "sku_typ": "bottle"}
        )
        self.assertEqual(result.camera, "left_wrist")
        self.assertEqual(len(result.image_paths), 5)

    def test_standard_pick_sends_full_localization_contract(self):
        manipulation = self.adapter(HttpManipulationCapability)
        localization = {"ok": True, "sku_typ": "bottle", "request_id": "infer-1"}
        result = manipulation.pick(
            PickRequest(
                TaskType.SORTING,
                TargetType.SKU,
                "bottle",
                Hand.RIGHT,
                "L4",
                localization,
            ),
            idempotency_key="pick-key",
        )
        self.assertEqual(result.status.value, "SUCCEEDED")
        self.assertEqual(self.requests[-1].headers[IDEMPOTENCY_HEADER], "pick-key")
        self.assertEqual(
            json.loads(self.requests[-1].read()),
            {
                "task_type": "SORTING",
                "target_type": "sku",
                "sku_typ": "bottle",
                "hand": "RIGHT",
                "level": "L4",
                "localization_result": localization,
            },
        )

    def test_place_basket_sends_full_localization_result(self):
        manipulation = self.adapter(HttpManipulationCapability)
        basket = {
            "ok": True,
            "target_type": "basket",
            "sku_typ": None,
            "class_name": "Basket",
            "pose_valid": True,
            "point_semantics": "basket_model_center",
            "model_center_camera_mm": [200.0, 30.0, 520.0],
            "pose_4x4": [[1.0, 0.0, 0.0, 50.0], [0.0, 1.0, 0.0, 10.0], [0.0, 0.0, 1.0, 400.0], [0.0, 0.0, 0.0, 1.0]],
        }
        manipulation.place(
            PlaceRequest(
                TaskType.SORTING,
                TargetType.SKU,
                DestinationType.BASKET,
                Hand.RIGHT,
                sku_typ="bottle",
                localization_result=basket,
            ),
            idempotency_key="place-key",
        )
        self.assertEqual(self.requests[-1].headers[IDEMPOTENCY_HEADER], "place-key")
        body = json.loads(self.requests[-1].read())
        self.assertEqual(body["target_type"], "sku")
        self.assertEqual(body["sku_typ"], "bottle")
        self.assertEqual(body["localization_result"], basket)
        self.assertNotIn("point_semantics", body)
        self.assertNotIn("class_name", body)
        self.assertNotIn("pose", body)

    def test_push_flattens_infer_response(self):
        manipulation = self.adapter(HttpManipulationCapability)
        basket = {
            "ok": True,
            "target_type": "basket",
            "sku_typ": None,
            "class_name": "Basket",
            "pose_valid": True,
            "point_semantics": "basket_model_center",
            "model_center_camera_mm": [200.0, 30.0, 520.0],
            "pose_4x4": [[1.0, 0.0, 0.0, 50.0], [0.0, 1.0, 0.0, 10.0], [0.0, 0.0, 1.0, 400.0], [0.0, 0.0, 0.0, 1.0]],
        }
        manipulation.push(
            PushRequest(Hand.RIGHT, basket),
            idempotency_key="push-key",
        )
        self.assertEqual(self.requests[-1].headers[IDEMPOTENCY_HEADER], "push-key")
        body = json.loads(self.requests[-1].read())
        self.assertEqual(body["hand"], "RIGHT")
        self.assertEqual(body["point_semantics"], "basket_model_center")
        self.assertNotIn("target_type", body)
        self.assertNotIn("sku_typ", body)
        self.assertNotIn("class_name", body)
        self.assertNotIn("localization_result", body)

    def test_pick_basket_flattens_infer_response_and_keeps_action_target(self):
        manipulation = self.adapter(HttpManipulationCapability)
        basket = {
            "ok": True,
            "target_type": "basket",
            "sku_typ": None,
            "class_name": "Basket",
            "pose_valid": True,
            "point_semantics": "basket_model_center",
            "model_center_camera_mm": [200.0, 30.0, 520.0],
            "pose_4x4": [[1.0, 0.0, 0.0, 50.0], [0.0, 1.0, 0.0, 10.0], [0.0, 0.0, 1.0, 400.0], [0.0, 0.0, 0.0, 1.0]],
        }
        manipulation.pick_basket(
            BasketPickRequest(Hand.RIGHT, basket),
            idempotency_key="pick-basket-key",
        )
        self.assertEqual(self.requests[-1].url.path, "/manipulation/pick")
        self.assertEqual(self.requests[-1].headers[IDEMPOTENCY_HEADER], "pick-basket-key")
        body = json.loads(self.requests[-1].read())
        self.assertEqual(body["task_type"], "REVIEW")
        self.assertEqual(body["target_type"], "basket")
        self.assertEqual(body["point_semantics"], "basket_model_center")
        self.assertNotIn("sku_typ", body)
        self.assertNotIn("class_name", body)
        self.assertNotIn("localization_result", body)

    def test_rotate_accepts_any_camera_and_image_paths(self):
        responses = [
            {"status": "SUCCEEDED", "camera": "head", "image_paths": ["/tmp/rgb.jpg"]},
            {"status": "SUCCEEDED"},
        ]

        def respond(request):
            return httpx.Response(200, json=responses.pop(0))

        raw = httpx.Client(transport=httpx.MockTransport(respond), base_url="http://module")
        self.addCleanup(raw.close)
        manipulation = HttpManipulationCapability(HttpCapabilityClient("http://module", client=raw))
        result = manipulation.rotate(Hand.RIGHT, "bottle")
        self.assertEqual(result.camera, "head")
        self.assertEqual(result.image_paths, ("/tmp/rgb.jpg",))
        with self.assertRaises(ValueError):
            manipulation.rotate(Hand.RIGHT, "bottle")
        with self.assertRaises(ValueError):
            manipulation.rotate(Hand.RIGHT, "  ")

    def test_standard_pick_accepts_box_and_left_hand(self):
        request = PickRequest(
            TaskType.SORTING,
            TargetType.SKU,
            "box",
            Hand.LEFT,
            "L2",
            {"ok": True},
        )
        self.assertEqual((request.sku_typ, request.hand), ("box", Hand.LEFT))

    def test_standard_pick_rejects_invalid_level(self):
        for level in ("", "L0", "L6"):
            with self.subTest(level=level), self.assertRaises(ValueError):
                PickRequest(
                    TaskType.SORTING,
                    TargetType.SKU,
                    "bottle",
                    Hand.RIGHT,
                    level,
                    {"ok": True},
                )

    def test_standard_pick_accepts_unknown_sku_typ(self):
        request = PickRequest(
            TaskType.SORTING,
            TargetType.SKU,
            "can",
            Hand.LEFT,
            "L1",
            {"ok": True},
        )
        self.assertEqual(request.sku_typ, "can")


if __name__ == "__main__":
    unittest.main()
