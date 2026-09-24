import tempfile
import unittest
from pathlib import Path

from agent.application import build_application_from_capabilities
from agent.capabilities.camera import MockCameraCapability
from agent.capabilities.common import CapabilityError, Hand
from agent.capabilities.estimation import MockEstimationCapability
from agent.capabilities.hand import MockHandCapability
from agent.capabilities.manipulation import MockManipulationCapability
from agent.capabilities.navigation import MockNavigationCapability
from agent.capabilities.perception import MockPerceptionCapability
from agent.capabilities.pose import MockBodyPoseCapability
from agent.capabilities.vla import MockVlaCapability
from agent.skus import SkuSpec
from agent.contracts import ExecutionContext
from agent.callbacks import progress_payload
from agent.workflows.policies import BarcodeMismatchMode, PickPolicy
from agent.workflows.sorting import SortingItemInput

BOTTLE_SKU = "3282779003131"
BOX_SKU = "887167608641"
SKU_CATALOG = {
    BOTTLE_SKU: SkuSpec("bottle", Hand.RIGHT),
    BOX_SKU: SkuSpec("box", Hand.LEFT),
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
}


def capabilities():
    perception = MockPerceptionCapability()
    perception.barcode_content = BOTTLE_SKU
    return {
        "navigation": MockNavigationCapability(),
        "pose": MockBodyPoseCapability(),
        "perception": perception,
        "estimation": MockEstimationCapability(),
        "camera": MockCameraCapability(),
        "manipulation": MockManipulationCapability(),
        "vla": MockVlaCapability(),
        "hand": MockHandCapability(),
    }


class SortingWorkflowTest(unittest.TestCase):
    def test_physical_timeout_waits_for_confirmation(self):
        with tempfile.TemporaryDirectory() as directory:
            caps = capabilities()

            def timeout(*_args, **_kwargs):
                raise CapabilityError("ACTION_RESULT_UNKNOWN", "physical action result is unknown")

            caps["navigation"].navigate = timeout
            app = build_application_from_capabilities(
                caps,
                database_path=Path(directory) / "tasks.db",
                calibration_source=lambda: TEST_CALIBRATION,
                sku_catalog=SKU_CATALOG,
            )
            app.store.create("item", "sorting_item", "http://callback", {})
            with self.assertRaises(Exception) as raised:
                app.workflow("sorting_item").run(
                    ExecutionContext("item"),
                    SortingItemInput(BOTTLE_SKU, "name", "L1", "1", "L1", "1"),
                )
            self.assertEqual(raised.exception.code, "ACTION_RESULT_UNKNOWN")
            snapshot = app.store.load("item")
            self.assertEqual(snapshot["status"], "WAITING_CONFIRMATION")
            self.assertEqual(snapshot["current_node"], "S-I01")
            app.close()

    def test_standard_item_runs_pick_after_observe_pose(self):
        with tempfile.TemporaryDirectory() as directory:
            caps = capabilities()
            app = build_application_from_capabilities(
                caps,
                database_path=Path(directory) / "tasks.db",
                calibration_source=lambda: TEST_CALIBRATION,
                sku_catalog=SKU_CATALOG,
            )
            result = app.workflow("sorting_item").run(
                ExecutionContext("item"), SortingItemInput(BOTTLE_SKU, "name", "L3", "2", "L4", "5")
            )
            self.assertEqual(result, {"sku_id": BOTTLE_SKU})
            self.assertEqual(caps["navigation"].calls, ["AGV_C", "SORTING_BASKET_5"])
            self.assertEqual(
                caps["pose"].history[:2],
                ["prepare:AGV_carton_item_inspect:L3", "camera_transform:head"],
            )
            self.assertEqual(caps["pose"].calls[1], ("AGV_item_barcode_scan", None))
            self.assertIn("pick", caps["manipulation"].calls)
            self.assertEqual(caps["manipulation"].pick_requests[-1].level, "L3")
            app.runtime.shutdown()

    def test_sku_mapped_left_hand_selects_agv_nav(self):
        with tempfile.TemporaryDirectory() as directory:
            caps = capabilities()
            caps["perception"].barcode_content = BOX_SKU
            app = build_application_from_capabilities(
                caps,
                database_path=Path(directory) / "tasks.db",
                calibration_source=lambda: TEST_CALIBRATION,
                sku_catalog=SKU_CATALOG,
            )
            result = app.workflow("sorting_item").run(
                ExecutionContext("item"),
                SortingItemInput(BOX_SKU, "name", "L1", "2", "L1", "1"),
            )
            self.assertEqual(result, {"sku_id": BOX_SKU})
            self.assertEqual(caps["navigation"].calls[0], "AGV_R")
            pick = caps["manipulation"].pick_requests[-1]
            self.assertEqual((pick.sku_typ, pick.hand), ("box", Hand.LEFT))
            self.assertEqual(caps["hand"].calls, [])
            app.runtime.shutdown()

    def test_barcode_mismatch_reports_error_and_continues_to_place(self):
        with tempfile.TemporaryDirectory() as directory:
            caps = capabilities()
            caps["perception"].barcode_content = "wrong-barcode"
            app = build_application_from_capabilities(
                caps,
                database_path=Path(directory) / "tasks.db",
                calibration_source=lambda: TEST_CALIBRATION,
                sku_catalog=SKU_CATALOG,
                policy=PickPolicy(barcode_mismatch=BarcodeMismatchMode.CONTINUE),
            )
            app.store.create("item", "sorting_item", "http://callback", {})
            context = ExecutionContext("item")
            result = app.workflow("sorting_item").run(
                context, SortingItemInput(BOTTLE_SKU, "name", "L1", "1", "L1", "1")
            )
            self.assertEqual(result, {"sku_id": BOTTLE_SKU})
            failed = [event for event in context.events if event["event"] == "skill.failed"]
            self.assertEqual(len(failed), 1)
            self.assertEqual(failed[0]["skill"], "recognize_and_verify_barcode")
            self.assertEqual(failed[0]["error_code"], "SKU_BARCODE_MISMATCH")
            self.assertEqual(
                progress_payload("item", failed[0]),
                {
                    "task_id": "item",
                    "status": "RUNNING",
                    "info": {
                        "skill": "识别并校验商品条码",
                        "progress": "识别并校验商品条码失败",
                        "error": "商品条码与预期不一致",
                        "error_code": "SKU_BARCODE_MISMATCH",
                    },
                },
            )
            self.assertEqual(caps["navigation"].calls, ["AGV_L", "SORTING_BASKET_1"])
            self.assertEqual(caps["manipulation"].calls, ["pick", "rotate", "place"])
            snapshot = app.store.load("item")
            self.assertEqual(snapshot["status"], "SUCCEEDED")
            self.assertIsNone(snapshot["sku_barcode"])
            self.assertEqual(snapshot["place_result"], {"status": "SUCCEEDED"})
            self.assertEqual(snapshot["sorting_policy"]["barcode_mismatch"], "CONTINUE")
            app.close()

    def test_barcode_mismatch_stops_when_configured(self):
        with tempfile.TemporaryDirectory() as directory:
            caps = capabilities()
            caps["perception"].barcode_content = "wrong-barcode"
            app = build_application_from_capabilities(
                caps,
                database_path=Path(directory) / "tasks.db",
                calibration_source=lambda: TEST_CALIBRATION,
                sku_catalog=SKU_CATALOG,
                policy=PickPolicy(barcode_mismatch=BarcodeMismatchMode.STOP),
            )
            app.store.create("item", "sorting_item", "http://callback", {})
            with self.assertRaises(Exception) as raised:
                app.workflow("sorting_item").run(
                    ExecutionContext("item"),
                    SortingItemInput(BOTTLE_SKU, "name", "L1", "1", "L1", "1"),
                )
            self.assertEqual(raised.exception.code, "SKU_BARCODE_MISMATCH")
            snapshot = app.store.load("item")
            self.assertEqual(snapshot["status"], "WAITING_CONFIRMATION")
            self.assertEqual(snapshot["current_node"], "S-I05")
            self.assertEqual(snapshot["sorting_policy"]["barcode_mismatch"], "STOP")
            self.assertNotIn("place", caps["manipulation"].calls)
            app.close()

    def test_barcode_not_found_still_stops(self):
        with tempfile.TemporaryDirectory() as directory:
            caps = capabilities()

            def missing(*_args, **_kwargs):
                raise CapabilityError("SKU_BARCODE_NOT_FOUND", "barcode not found")

            caps["perception"].recognize_sku_barcode = missing
            app = build_application_from_capabilities(
                caps,
                database_path=Path(directory) / "tasks.db",
                calibration_source=lambda: TEST_CALIBRATION,
                sku_catalog=SKU_CATALOG,
            )
            app.store.create("item", "sorting_item", "http://callback", {})
            with self.assertRaises(Exception) as raised:
                app.workflow("sorting_item").run(
                    ExecutionContext("item"),
                    SortingItemInput(BOTTLE_SKU, "name", "L1", "1", "L1", "1"),
                )
            self.assertEqual(raised.exception.code, "SKU_BARCODE_NOT_FOUND")
            snapshot = app.store.load("item")
            self.assertEqual(snapshot["status"], "WAITING_CONFIRMATION")
            self.assertEqual(snapshot["current_node"], "S-I05")
            self.assertNotIn("place", caps["manipulation"].calls)
            app.close()

    def test_barcode_not_found_reports_error_and_continues_to_place(self):
        with tempfile.TemporaryDirectory() as directory:
            caps = capabilities()

            def missing(*_args, **_kwargs):
                raise CapabilityError("SKU_BARCODE_NOT_FOUND", "barcode not found")

            caps["perception"].recognize_sku_barcode = missing
            app = build_application_from_capabilities(
                caps,
                database_path=Path(directory) / "tasks.db",
                calibration_source=lambda: TEST_CALIBRATION,
                sku_catalog=SKU_CATALOG,
                policy=PickPolicy(barcode_mismatch=BarcodeMismatchMode.CONTINUE),
            )
            app.store.create("item", "sorting_item", "http://callback", {})
            context = ExecutionContext("item")
            result = app.workflow("sorting_item").run(
                context, SortingItemInput(BOTTLE_SKU, "name", "L1", "1", "L1", "1")
            )
            self.assertEqual(result, {"sku_id": BOTTLE_SKU})
            failed = [event for event in context.events if event["event"] == "skill.failed"]
            self.assertEqual(len(failed), 1)
            self.assertEqual(failed[0]["skill"], "recognize_and_verify_barcode")
            self.assertEqual(failed[0]["error_code"], "SKU_BARCODE_NOT_FOUND")
            self.assertEqual(caps["manipulation"].calls, ["pick", "rotate", "place"])
            snapshot = app.store.load("item")
            self.assertEqual(snapshot["status"], "SUCCEEDED")
            self.assertIsNone(snapshot["sku_barcode"])
            self.assertEqual(snapshot["place_result"], {"status": "SUCCEEDED"})
            app.close()


if __name__ == "__main__":
    unittest.main()
