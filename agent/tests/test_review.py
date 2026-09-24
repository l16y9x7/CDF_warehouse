import tempfile
import unittest
from pathlib import Path

from agent.application import build_application_from_capabilities
from agent.capabilities.camera import MockCameraCapability
from agent.capabilities.estimation import MockEstimationCapability
from agent.capabilities.hand import MockHandCapability
from agent.capabilities.manipulation import MockManipulationCapability
from agent.capabilities.navigation import MockNavigationCapability
from agent.capabilities.perception import LocateStatus, MockPerceptionCapability
from agent.capabilities.pose import MockBodyPoseCapability
from agent.capabilities.vla import MockVlaCapability
from agent.contracts import ExecutionContext
from agent.models import ReviewItemCount
from agent.workflows.review import ReviewInput


def capabilities(perception=None):
    return {
        "navigation": MockNavigationCapability(),
        "pose": MockBodyPoseCapability(),
        "perception": perception or MockPerceptionCapability(),
        "estimation": MockEstimationCapability(),
        "camera": MockCameraCapability(),
        "manipulation": MockManipulationCapability(),
        "vla": MockVlaCapability(),
        "hand": MockHandCapability(),
    }


class ReviewWorkflowTest(unittest.TestCase):
    def test_review_input_keeps_order_id(self):
        data = ReviewInput("order-1", "L2", "4", (ReviewItemCount("A", 1),))
        self.assertEqual(data.order_id, "order-1")
        self.assertIn("order_id", ReviewInput.__dataclass_fields__)


    @unittest.skip("TODO 旧 pick_pose 迁移待引入：review workflow 依赖旧 pick_pose")
    def test_review_uses_two_new_empty_observations_and_no_order_id(self):
        with tempfile.TemporaryDirectory() as directory:
            p = MockPerceptionCapability()
            p.barcode_content = "A"
            p.basket_item_statuses = [
                LocateStatus.FOUND,
                LocateStatus.NOT_FOUND,
                LocateStatus.NOT_FOUND,
            ]
            caps = capabilities(p)
            app = build_application_from_capabilities(
                caps, database_path=Path(directory) / "tasks.db"
            )
            result = app.workflow("review").run(
                ExecutionContext("review"),
                ReviewInput("order-1", "L2", "4", (ReviewItemCount("A", 1),)),
            )
            self.assertEqual(result["review_status"], "PASS")
            self.assertEqual(caps["navigation"].calls, ["REVIEW_BASKET_4", "REVIEW_TABLE"])
            self.assertEqual(app.store.load("review")["order_id"], "order-1")
            app.runtime.shutdown()

    @unittest.skip("TODO 旧 pick_pose 迁移待引入：review workflow 依赖旧 pick_pose")
    def test_review_records_wrong_barcode_as_sku(self):
        with tempfile.TemporaryDirectory() as directory:
            p = MockPerceptionCapability()
            p.barcode_content = "WRONG"
            p.basket_item_statuses = [
                LocateStatus.FOUND,
                LocateStatus.NOT_FOUND,
                LocateStatus.NOT_FOUND,
            ]
            app = build_application_from_capabilities(
                capabilities(p), database_path=Path(directory) / "tasks.db"
            )
            result = app.workflow("review").run(
                ExecutionContext("review"),
                ReviewInput("order-1", "L1", "1", (ReviewItemCount("A", 1),)),
            )
            self.assertEqual(result["wrong"], ({"sku_id": "WRONG", "count": 1},))
            self.assertEqual(result["extra"], ())
            app.runtime.shutdown()


if __name__ == "__main__":
    unittest.main()
