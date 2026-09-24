import tempfile
import unittest
from pathlib import Path

from agent.application import build_application_from_capabilities
from agent.capabilities.camera import MockCameraCapability
from agent.capabilities.estimation import MockEstimationCapability
from agent.capabilities.hand import MockHandCapability
from agent.capabilities.manipulation import MockManipulationCapability
from agent.capabilities.navigation import MockNavigationCapability
from agent.capabilities.perception import MockPerceptionCapability
from agent.capabilities.pose import MockBodyPoseCapability
from agent.capabilities.vla import MockVlaCapability


def capabilities():
    return {
        "navigation": MockNavigationCapability(),
        "pose": MockBodyPoseCapability(),
        "perception": MockPerceptionCapability(),
        "estimation": MockEstimationCapability(),
        "camera": MockCameraCapability(),
        "manipulation": MockManipulationCapability(),
        "vla": MockVlaCapability(),
        "hand": MockHandCapability(),
    }


class CloseTracker:
    def __init__(self):
        self.closed = False

    def close(self):
        self.closed = True


class UnexpectedHealthCheck:
    def health(self):
        raise AssertionError("optional capability must not be health checked")


class AgentApplicationLifecycleTest(unittest.TestCase):
    def test_closes_only_explicitly_owned_resources(self):
        with tempfile.TemporaryDirectory() as directory:
            owned = CloseTracker()
            injected = CloseTracker()
            app = build_application_from_capabilities(
                capabilities(),
                database_path=Path(directory) / "tasks.db",
                owned_resources=(owned,),
            )
            app.capabilities["external_test_resource"] = injected

            app.close()

            self.assertTrue(owned.closed)
            self.assertFalse(injected.closed)

    def test_builds_without_hand_or_vla(self):
        with tempfile.TemporaryDirectory() as directory:
            core = capabilities()
            core.pop("hand")
            core.pop("vla")
            app = build_application_from_capabilities(
                core, database_path=Path(directory) / "tasks.db"
            )

            self.assertNotIn("pick_sku_hand", app.skills)
            self.assertNotIn("pick_review_item_vla", app.skills)
            app.preflight("sorting_item")
            app.preflight("review")
            app.close()

    def test_preflight_does_not_check_hand_or_vla(self):
        with tempfile.TemporaryDirectory() as directory:
            caps = capabilities()
            caps["hand"] = UnexpectedHealthCheck()
            caps["vla"] = UnexpectedHealthCheck()
            app = build_application_from_capabilities(
                caps, database_path=Path(directory) / "tasks.db"
            )

            app.preflight("sorting_item")
            app.preflight("review")
            app.close()


if __name__ == "__main__":
    unittest.main()
