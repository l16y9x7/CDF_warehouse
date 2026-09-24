import base64
import io
import unittest

import numpy as np

from agent.capabilities.camera import MockCameraCapability
from agent.capabilities.common import CapabilityError, Hand
from agent.capabilities.estimation import MockEstimationCapability, PickPoseResult
from agent.capabilities.manipulation import MockManipulationCapability
from agent.capabilities.perception import BarcodeResult, MockPerceptionCapability
from agent.capabilities.pose import CameraTransform, MockBodyPoseCapability
from agent.skills.pose import PreparePoseInput, PreparePoseSkill
from agent.contracts import ExecutionContext
from agent.models import InspectedItem, ReviewItemCount
from agent.skus import SkuSpec
from agent.skills.perception import RecognizeAndVerifyBarcodeSkill, RecognizeBarcodeInput
from agent.skills.pick_sku_standard import PickSkuStandardInput, PickSkuStandardSkill
from agent.skills.place_sku_in_basket import PlaceSkuInBasketInput, PlaceSkuInBasketSkill
from agent.skills.basket_finish import PushBasketInput, PushBasketSkill
from agent.skills.review import HandOnlyInput, PickReviewBasketSkill, SummarizeReviewInput, SummarizeReviewResultSkill

SKU_CATALOG = {
    "3282779003131": SkuSpec("bottle", Hand.RIGHT),
    "887167608641": SkuSpec("box", Hand.LEFT),
    "7173342765403": SkuSpec("tube", Hand.LEFT),
}

TEST_CALIBRATION = {
    "K": ((612.0, 0.0, 641.0), (0.0, 611.0, 358.0), (0.0, 0.0, 1.0)),
    "T_chassis_camera": (
        (1.0, 0.0, 0.0, 0.0),
        (0.0, 1.0, 0.0, 0.0),
        (0.0, 0.0, 1.0, 0.0),
        (0.0, 0.0, 0.0, 1.0),
    ),
    "T_unit": "m",
    "front_rule": {
        "front_axis_chassis": (1.0, 0.0, 0.0),
        "front_origin_chassis": (0.0, 0.0, 0.0),
        "front_band_mm": 100000.0,
    },
}


class BarcodeSequencePerception(MockPerceptionCapability):
    def __init__(self, outcomes):
        super().__init__()
        self.outcomes = list(outcomes)
        self.requests = []

    @property
    def calls(self):
        return len(self.requests)

    def recognize_sku_barcode(self, request):
        self.requests.append(request)
        outcome = self.outcomes.pop(0)
        if isinstance(outcome, CapabilityError):
            raise outcome
        return BarcodeResult(outcome)


class SkillTest(unittest.TestCase):
    def test_place_sku_in_basket_uses_head_basket_infer_then_place(self):
        camera = MockCameraCapability()
        estimation = MockEstimationCapability()
        manipulation = MockManipulationCapability()
        result = PlaceSkuInBasketSkill(
            camera,
            estimation,
            manipulation,
            MockBodyPoseCapability(),
        ).execute(
            ExecutionContext("t"),
            PlaceSkuInBasketInput("bottle", Hand.RIGHT),
        )

        self.assertEqual(result.status, "SUCCEEDED")
        self.assertEqual(camera.captures, ["head"])
        request = estimation.requests[-1]
        self.assertEqual(request.target_type.value, "basket")
        self.assertIsNone(request.sku_typ)
        self.assertIsNone(request.side)
        placed = manipulation.place_requests[-1]
        self.assertEqual(placed.sku_typ, "bottle")
        self.assertEqual(placed.localization_result["point_semantics"], "basket_model_center")
        self.assertEqual(manipulation.calls, ["place"])

    def test_push_basket_uses_head_basket_infer_then_flat_push(self):
        camera = MockCameraCapability()
        estimation = MockEstimationCapability()
        manipulation = MockManipulationCapability()
        result = PushBasketSkill(
            camera,
            estimation,
            manipulation,
            MockBodyPoseCapability(),
        ).execute(ExecutionContext("t"), PushBasketInput(Hand.RIGHT))

        self.assertEqual(result.status, "SUCCEEDED")
        self.assertEqual(camera.captures, ["head"])
        self.assertEqual(estimation.requests[-1].target_type.value, "basket")
        pushed = manipulation.push_requests[-1]
        self.assertEqual(pushed.hand, Hand.RIGHT)
        self.assertEqual(pushed.localization_result["point_semantics"], "basket_model_center")
        self.assertEqual(manipulation.calls, ["push"])

    def test_push_basket_reuses_provided_localization_without_infer(self):
        camera = MockCameraCapability()
        estimation = MockEstimationCapability()
        manipulation = MockManipulationCapability()
        localization = {"ok": True, "point_semantics": "basket_model_center"}
        result = PushBasketSkill(
            camera,
            estimation,
            manipulation,
            MockBodyPoseCapability(),
        ).execute(ExecutionContext("t"), PushBasketInput(Hand.RIGHT, localization))

        self.assertEqual(result.status, "SUCCEEDED")
        self.assertEqual(camera.captures, [])
        self.assertEqual(estimation.requests, [])
        self.assertEqual(manipulation.push_requests[-1].localization_result, localization)
        self.assertEqual(manipulation.calls, ["push"])

    def test_pick_review_basket_uses_head_basket_infer_then_flat_pick(self):
        camera = MockCameraCapability()
        estimation = MockEstimationCapability()
        manipulation = MockManipulationCapability()
        result = PickReviewBasketSkill(
            camera,
            estimation,
            manipulation,
            MockBodyPoseCapability(),
        ).execute(ExecutionContext("t"), HandOnlyInput(Hand.RIGHT))

        self.assertEqual(result.status, "SUCCEEDED")
        self.assertEqual(camera.captures, ["head"])
        self.assertEqual(estimation.requests[-1].target_type.value, "basket")
        picked = manipulation.pick_basket_requests[-1]
        self.assertEqual(picked.hand, Hand.RIGHT)
        self.assertEqual(picked.localization_result["point_semantics"], "basket_model_center")
        self.assertEqual(manipulation.calls, ["pick_basket"])

    def test_barcode_scan_uses_rotate_images_until_expected_sku_is_found(self):
        not_found = CapabilityError("SKU_BARCODE_NOT_FOUND", "barcode not found")
        p = BarcodeSequencePerception([not_found, "sku-1"])
        m, camera = MockManipulationCapability(), MockCameraCapability()
        context = ExecutionContext("t")
        result = RecognizeAndVerifyBarcodeSkill(p, camera, m).execute(
            context, RecognizeBarcodeInput("sku-1", "name", Hand.RIGHT, "bottle")
        )
        self.assertEqual(result.sku_id, "sku-1")
        self.assertEqual(m.calls, ["rotate"])
        self.assertEqual(m.sku_typs, ["bottle"])
        self.assertEqual(m.hands, [Hand.RIGHT])
        self.assertEqual([request.image_base64 for request in p.requests], ["bW9jay1qcGVn"] * 2)
        self.assertEqual(camera.captures, [])
        self.assertEqual(camera.image_reads, list(m.rotate_image_paths[:2]))
        succeeded = [event for event in context.events if event["event"] == "skill.succeeded"][-1]
        self.assertEqual(succeeded["camera"], "left_wrist")
        self.assertEqual(succeeded["image_count"], 5)
        self.assertEqual(succeeded["matched_image_index"], 2)

    def test_barcode_scan_continues_after_wrong_barcode_and_finds_expected(self):
        perception = BarcodeSequencePerception(["wrong", "sku-1"])
        result = RecognizeAndVerifyBarcodeSkill(
            perception, MockCameraCapability(), MockManipulationCapability()
        ).execute(ExecutionContext("t"), RecognizeBarcodeInput("sku-1", "name", Hand.RIGHT, "bottle"))
        self.assertEqual(result.sku_id, "sku-1")
        self.assertEqual(perception.calls, 2)

    def test_barcode_mismatch_has_stable_error(self):
        p = BarcodeSequencePerception(["other"] * 5)
        with self.assertRaisesRegex(Exception, "does not match") as raised:
            RecognizeAndVerifyBarcodeSkill(
                p, MockCameraCapability(), MockManipulationCapability()
            ).execute(ExecutionContext("t"), RecognizeBarcodeInput("sku-1", "name", Hand.RIGHT, "bottle"))
        self.assertEqual(raised.exception.code, "SKU_BARCODE_MISMATCH")
        self.assertEqual(p.calls, 5)

    def test_barcode_not_found_uses_all_five_images_and_rotates_once(self):
        perception = BarcodeSequencePerception(
            [CapabilityError("SKU_BARCODE_NOT_FOUND", "barcode not found") for _ in range(5)]
        )
        manipulation = MockManipulationCapability()
        with self.assertRaises(Exception) as raised:
            RecognizeAndVerifyBarcodeSkill(
                perception, MockCameraCapability(), manipulation
            ).execute(
                ExecutionContext("t"),
                RecognizeBarcodeInput("sku-1", "name", Hand.RIGHT, "bottle"),
            )
        self.assertEqual(raised.exception.code, "SKU_BARCODE_NOT_FOUND")
        self.assertEqual(perception.calls, 5)
        self.assertEqual(manipulation.calls, ["rotate"])

    def test_wire_barcode_mismatch_stops_after_first_image(self):
        perception = BarcodeSequencePerception(
            [CapabilityError("SKU_BARCODE_MISMATCH", "barcode mismatch")]
        )
        manipulation = MockManipulationCapability()
        with self.assertRaises(Exception) as raised:
            RecognizeAndVerifyBarcodeSkill(
                perception, MockCameraCapability(), manipulation
            ).execute(
                ExecutionContext("t"),
                RecognizeBarcodeInput("sku-1", "name", Hand.RIGHT, "bottle"),
            )
        self.assertEqual(raised.exception.code, "SKU_BARCODE_MISMATCH")
        self.assertEqual(perception.calls, 1)
        self.assertEqual(manipulation.calls, ["rotate"])

    def test_other_barcode_capability_errors_are_not_reclassified(self):
        perception = BarcodeSequencePerception(
            [CapabilityError("MODEL_FAILED", "model failed")]
        )
        manipulation = MockManipulationCapability()
        with self.assertRaises(Exception) as raised:
            RecognizeAndVerifyBarcodeSkill(
                perception, MockCameraCapability(), manipulation
            ).execute(
                ExecutionContext("t"),
                RecognizeBarcodeInput("sku-1", "name", Hand.RIGHT, "bottle"),
            )
        self.assertEqual(raised.exception.code, "CAPABILITY_EXECUTION_FAILED")
        self.assertEqual(perception.calls, 1)
        self.assertEqual(manipulation.calls, ["rotate"])

    def test_scan_image_read_failure_has_stable_error(self):
        camera = MockCameraCapability()

        def missing(_path):
            raise FileNotFoundError("missing scan image")

        camera.read_image_bytes = missing
        with self.assertRaises(Exception) as raised:
            RecognizeAndVerifyBarcodeSkill(
                MockPerceptionCapability(), camera, MockManipulationCapability()
            ).execute(
                ExecutionContext("t"),
                RecognizeBarcodeInput("sku-1", "name", Hand.RIGHT, "bottle"),
            )
        self.assertEqual(raised.exception.code, "SCAN_IMAGE_UNAVAILABLE")

    def test_standard_pick_uses_transform_rgbd_infer_then_pick(self):
        camera, estimation, manipulation = (
            MockCameraCapability(),
            MockEstimationCapability(),
            MockManipulationCapability(),
        )
        context = ExecutionContext("t")
        pose = MockBodyPoseCapability()
        result = PickSkuStandardSkill(
            camera,
            estimation,
            manipulation,
            pose,
            calibration_source=lambda: TEST_CALIBRATION,
            sku_catalog=SKU_CATALOG,
        ).execute(
            context, PickSkuStandardInput("3282779003131", "name", "LEFT", Hand.RIGHT, "L3")
        )
        self.assertEqual(result.status, "SUCCEEDED")
        self.assertEqual(pose.transform_calls, ["head"])
        sent = estimation.requests[-1]
        self.assertEqual(base64.b64decode(sent.rgb_base64), b"mock-jpeg")
        depth = np.load(io.BytesIO(base64.b64decode(sent.depth_npy_base64)), allow_pickle=False)
        self.assertEqual(depth.dtype, np.dtype("float32"))
        self.assertEqual(depth.shape, (720, 1280))
        self.assertEqual((sent.sku_typ, sent.side), ("bottle", "LEFT"))
        self.assertIsNone(sent.front_rule)
        self.assertEqual(sent.T_unit, "m")
        self.assertEqual(sent.T_chassis_camera[0][3], 0.113767949307)
        self.assertEqual(sent.camera_frame, "head_camera_color_optical_frame")
        self.assertEqual(sent.base_frame, "chassis_link")
        self.assertEqual(manipulation.calls, ["pick"])
        pick = manipulation.pick_requests[-1]
        self.assertEqual((pick.sku_typ, pick.hand, pick.level), ("bottle", Hand.RIGHT, "L3"))
        self.assertEqual(pick.localization_result["sku_typ"], "bottle")
        self.assertEqual(
            manipulation.idempotency_keys,
            ["t:pick_sku_standard:3282779003131:L3"],
        )

    def test_standard_pick_forwards_estimation_result_even_when_unusable(self):
        estimation = MockEstimationCapability()
        estimation.estimate_pick_pose = lambda request: PickPoseResult.from_payload({"ok": False})
        manipulation = MockManipulationCapability()
        result = PickSkuStandardSkill(
            MockCameraCapability(),
            estimation,
            manipulation,
            MockBodyPoseCapability(),
            sku_catalog=SKU_CATALOG,
        ).execute(
            ExecutionContext("t"),
            PickSkuStandardInput("3282779003131", "name", "RIGHT", Hand.RIGHT, "L1"),
        )
        self.assertEqual(result.status, "SUCCEEDED")
        self.assertEqual(manipulation.calls, ["pick"])
        self.assertEqual(manipulation.pick_requests[-1].localization_result["ok"], False)

    def test_standard_pick_passes_runtime_frames_to_estimation(self):
        pose = MockBodyPoseCapability()

        def camera_transform(camera="head"):
            pose.transform_calls.append(camera)
            return CameraTransform(
                TEST_CALIBRATION["T_chassis_camera"],
                "m",
                camera,
                "runtime_camera_optical_frame",
                "runtime_robot_base",
            )

        pose.camera_transform = camera_transform
        estimation = MockEstimationCapability()
        estimation.estimate_pick_pose = lambda request: (
            estimation.requests.append(request) or PickPoseResult.from_payload({"ok": False})
        )
        manipulation = MockManipulationCapability()

        PickSkuStandardSkill(
            MockCameraCapability(),
            estimation,
            manipulation,
            pose,
            sku_catalog=SKU_CATALOG,
        ).execute(
            ExecutionContext("t"),
            PickSkuStandardInput("3282779003131", "name", "RIGHT", Hand.RIGHT, "L5"),
        )

        sent = estimation.requests[-1]
        self.assertEqual(sent.camera_frame, "runtime_camera_optical_frame")
        self.assertEqual(sent.base_frame, "runtime_robot_base")
        self.assertEqual(manipulation.calls, ["pick"])

    def test_standard_pick_rejects_unmapped_sku_id(self):
        with self.assertRaises(Exception) as raised:
            PickSkuStandardSkill(
                MockCameraCapability(),
                MockEstimationCapability(),
                MockManipulationCapability(),
                MockBodyPoseCapability(),
                calibration_source=lambda: TEST_CALIBRATION,
                sku_catalog=SKU_CATALOG,
            ).execute(
                ExecutionContext("t"),
                PickSkuStandardInput("sku-1", "name", "LEFT", Hand.RIGHT, "L2"),
            )
        self.assertEqual(raised.exception.code, "SKU_TYPE_UNKNOWN")

    def test_standard_pick_accepts_box_with_left_hand(self):
        camera, estimation, manipulation = (
            MockCameraCapability(),
            MockEstimationCapability(),
            MockManipulationCapability(),
        )
        result = PickSkuStandardSkill(
            camera,
            estimation,
            manipulation,
            MockBodyPoseCapability(),
            sku_catalog=SKU_CATALOG,
        ).execute(
            ExecutionContext("t"),
            PickSkuStandardInput("887167608641", "name", "RIGHT", Hand.LEFT, "L2"),
        )
        self.assertEqual(result.status, "SUCCEEDED")
        sent = estimation.requests[-1]
        self.assertEqual((sent.sku_typ, sent.side), ("box", "RIGHT"))
        pick = manipulation.pick_requests[-1]
        self.assertEqual((pick.sku_typ, pick.hand, pick.level), ("box", Hand.LEFT, "L2"))

    def test_standard_pick_rejects_invalid_level_before_capability_calls(self):
        pose = MockBodyPoseCapability()
        estimation = MockEstimationCapability()
        manipulation = MockManipulationCapability()
        with self.assertRaises(Exception) as raised:
            PickSkuStandardSkill(
                MockCameraCapability(),
                estimation,
                manipulation,
                pose,
                sku_catalog=SKU_CATALOG,
            ).execute(
                ExecutionContext("t"),
                PickSkuStandardInput(
                    "3282779003131", "name", "LEFT", Hand.RIGHT, "L6"
                ),
            )
        self.assertEqual(raised.exception.code, "INVALID_INPUT")
        self.assertEqual(pose.transform_calls, [])
        self.assertEqual(estimation.requests, [])
        self.assertEqual(manipulation.calls, [])

    def test_wrong_items_are_not_duplicated_in_extra(self):
        summary = SummarizeReviewResultSkill().execute(
            ExecutionContext("t"),
            SummarizeReviewInput(
                (ReviewItemCount("A", 1),),
                (InspectedItem(1, "A"), InspectedItem(2, "A"), InspectedItem(3, "B")),
            ),
        )
        self.assertEqual(summary.extra, (ReviewItemCount("A", 1),))
        self.assertEqual(summary.wrong, (ReviewItemCount("B", 1),))

    def test_prepare_pose_omits_level_for_barcode_scan(self):
        pose = MockBodyPoseCapability()
        result = PreparePoseSkill(pose).execute(
            ExecutionContext("t"), PreparePoseInput("AGV_item_barcode_scan")
        )
        self.assertEqual(result.status, "SUCCEEDED")
        self.assertEqual(pose.calls, [("AGV_item_barcode_scan", None)])

    def test_prepare_pose_rejects_level_for_barcode_scan(self):
        with self.assertRaises(Exception) as raised:
            PreparePoseSkill(MockBodyPoseCapability()).execute(
                ExecutionContext("t"), PreparePoseInput("AGV_item_barcode_scan", "L2")
            )
        self.assertEqual(raised.exception.code, "INVALID_INPUT")

    def test_prepare_pose_requires_level_for_inspect(self):
        with self.assertRaises(Exception) as raised:
            PreparePoseSkill(MockBodyPoseCapability()).execute(
                ExecutionContext("t"), PreparePoseInput("AGV_carton_item_inspect")
            )
        self.assertEqual(raised.exception.code, "INVALID_INPUT")


if __name__ == "__main__":
    unittest.main()
