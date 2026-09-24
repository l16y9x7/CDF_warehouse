import tempfile
import threading
import unittest
from pathlib import Path

import httpx

from agent.api import create_app
from agent.application import build_application_from_capabilities
from agent.callbacks import terminal_payload
from agent.capabilities.camera import MockCameraCapability
from agent.capabilities.estimation import MockEstimationCapability
from agent.capabilities.hand import MockHandCapability
from agent.capabilities.manipulation import MockManipulationCapability
from agent.capabilities.navigation import MockNavigationCapability
from agent.capabilities.perception import MockPerceptionCapability
from agent.capabilities.pose import MockBodyPoseCapability
from agent.capabilities.vla import MockVlaCapability
from agent.runtime import CallbackSender


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


class AgentApiTest(unittest.IsolatedAsyncioTestCase):
    async def asyncSetUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.service = build_application_from_capabilities(
            capabilities(),
            database_path=Path(self.temp.name) / "tasks.db",
            calibration_source=lambda: {
                "K": ((612.0, 0, 641), (0, 611, 358), (0, 0, 1)),
                "T_chassis_camera": ((1, 0, 0, 0), (0, 1, 0, 0), (0, 0, 1, 0), (0, 0, 0, 1)),
                "T_unit": "m",
            },
        )
        callback_client = httpx.Client(
            transport=httpx.MockTransport(lambda request: httpx.Response(204))
        )
        self.service.runtime.callback_sender = CallbackSender(
            self.service.store, backoff=0, client=callback_client
        )
        self.app = create_app(self.service, callback_url="http://callback")
        self.mock_service = self.app.state.mock_agent
        self.mock_service.runtime.callback_sender.close()
        self.mock_service.runtime.callback_sender = CallbackSender(
            self.mock_service.store,
            backoff=0,
            client=httpx.Client(transport=httpx.MockTransport(lambda request: httpx.Response(204))),
        )
        self.client = httpx.AsyncClient(
            transport=httpx.ASGITransport(app=self.app), base_url="http://test"
        )

    async def asyncTearDown(self):
        await self.client.aclose()
        self.service.runtime.shutdown()
        self.mock_service.close()
        self.app.state.logging_manager.shutdown()
        self.temp.cleanup()

    async def test_accepts_new_item_contract_and_deduplicates(self):
        body = {
            "task_id": "item",
            "sku_id": "Avene",
            "name": "name",
            "agv_row": "L1",
            "agv_column": "1",
            "basket_row": "L2",
            "basket_column": "3",
        }
        first = await self.client.post("/agent/sorting/item", json=body)
        second = await self.client.post("/agent/sorting/item", json=body)
        self.assertEqual(first.json(), {"task_id": "item", "status": "ACCEPTED"})
        self.assertEqual(second.json(), first.json())
        self.service.runtime.wait("item")
        self.assertEqual(self.service.store.get("item")["callback_url"], "http://callback")
        self.assertIsNone(self.mock_service.store.get("item"))

    async def test_old_fields_and_removed_endpoint_are_rejected(self):
        self.assertEqual(
            (await self.client.post("/agent/sorting/basket-location", json={})).status_code, 404
        )
        response = await self.client.post(
            "/agent/sorting/item",
            json={"task_id": "x", "callback_url": "x", "basket_location_task_id": "old"},
        )
        self.assertEqual(response.status_code, 422)

    async def test_review_requires_order_id_and_validates_unique_skus(self):
        body = {
            "task_id": "r",
            "basket_row": "L1",
            "basket_column": "1",
            "expected_items": [{"sku_id": "A", "count": 1}],
        }
        missing = await self.client.post("/agent/review", json=body)
        self.assertEqual(missing.status_code, 422)
        self.assertEqual(missing.json()["error_code"], "INVALID_INPUT")
        body["order_id"] = "order-1"
        body["expected_items"] = [{"sku_id": "A", "count": 1}, {"sku_id": "A", "count": 2}]
        self.assertEqual((await self.client.post("/agent/review", json=body)).status_code, 422)
        body["expected_items"] = []
        self.assertEqual((await self.client.post("/agent/review", json=body)).status_code, 200)
        self.service.runtime.wait("r")
        self.assertEqual(self.service.store.get("r")["request"]["order_id"], "order-1")

    async def test_row_and_column_boundaries(self):
        body = {"task_id": "f", "basket_row": "L5", "basket_column": "1"}
        response = await self.client.post("/agent/sorting/finish", json=body)
        self.assertEqual(response.status_code, 422)
        self.assertEqual(response.json()["error_code"], "INVALID_INPUT")

    async def test_terminate_returns_conflict_when_idle(self):
        response = await self.client.post("/agent/terminate")
        self.assertEqual(response.status_code, 409)
        self.assertEqual(response.json()["error_code"], "NO_ACTIVE_TASK")

        invalid = await self.client.post("/agent/terminate", json={"task_id": "not-allowed"})
        self.assertEqual(invalid.status_code, 422)
        self.assertEqual(invalid.json()["error_code"], "INVALID_INPUT")

    async def test_terminate_active_task_and_reject_new_task_while_busy(self):
        started = threading.Event()
        release = threading.Event()
        original = self.service.workflows["sorting_finish"]

        class BlockingWorkflow:
            def run(self, context, data):
                started.set()
                release.wait(2)
                return original.run(context, data)

        self.service.workflows["sorting_finish"] = BlockingWorkflow()
        body = {
            "task_id": "active",
            "basket_row": "L1",
            "basket_column": "1",
        }
        accepted = await self.client.post("/agent/sorting/finish", json=body)
        self.assertEqual(accepted.status_code, 200)
        self.assertTrue(started.wait(1))

        terminated = await self.client.post("/agent/terminate")
        self.assertEqual(
            terminated.json(),
            {"task_id": "active", "status": "ACCEPTED"},
        )
        repeated = await self.client.post("/agent/terminate")
        self.assertEqual(repeated.json(), terminated.json())

        busy_body = {**body, "task_id": "other"}
        busy = await self.client.post("/agent/sorting/finish", json=busy_body)
        self.assertEqual(busy.status_code, 409)
        self.assertEqual(busy.json()["error_code"], "ROBOT_BUSY")

        release.set()
        self.service.runtime.wait("active")
        self.assertEqual(self.service.store.get("active")["status"], "CANCELLED")

    async def test_real_and_mock_runtimes_accept_tasks_together(self):
        started = threading.Event()
        release = threading.Event()
        original = self.service.workflows["sorting_finish"]

        class BlockingWorkflow:
            def run(self, context, data):
                started.set()
                release.wait(2)
                return original.run(context, data)

        self.service.workflows["sorting_finish"] = BlockingWorkflow()
        real_body = {"task_id": "real-task", "basket_row": "L1", "basket_column": "1"}
        accepted = await self.client.post("/agent/sorting/finish", json=real_body)
        self.assertEqual(accepted.status_code, 200)
        self.assertTrue(started.wait(1))

        mock_accepted = await self.client.post(
            "/agent/sorting/finish",
            json={"task_id": "mock-task", "basket_row": "L1", "basket_column": "1", "mock": True},
        )
        self.assertEqual(mock_accepted.json(), {"task_id": "mock-task", "status": "ACCEPTED"})
        self.mock_service.runtime.wait("mock-task")
        self.assertIsNone(self.service.store.get("mock-task"))
        self.assertIsNotNone(self.mock_service.store.get("mock-task"))

        same_side = await self.client.post(
            "/agent/sorting/finish",
            json={"task_id": "real-false", "basket_row": "L1", "basket_column": "1", "mock": False},
        )
        self.assertEqual(same_side.status_code, 409)
        self.assertEqual(same_side.json()["error_code"], "ROBOT_BUSY")

        idle_mock = await self.client.post("/agent/terminate", json={"mock": True})
        self.assertEqual(idle_mock.status_code, 409)
        self.assertEqual(idle_mock.json()["error_code"], "NO_ACTIVE_TASK")

        stopped = await self.client.post("/agent/terminate", json={})
        self.assertEqual(stopped.json(), {"task_id": "real-task", "status": "ACCEPTED"})
        release.set()
        self.service.runtime.wait("real-task")

    def test_failed_callback_uses_error_code(self):
        failed = terminal_payload(
            {"task_id": "t", "status": "FAILED", "error_code": "EXECUTION_FAILED"},
            [],
        )
        self.assertEqual(failed["info"], {"message": "任务失败", "error": "EXECUTION_FAILED"})
        waiting = terminal_payload(
            {
                "task_id": "t",
                "status": "WAITING_CONFIRMATION",
                "error_code": "ACTION_RESULT_UNKNOWN",
            },
            [],
        )
        self.assertEqual(waiting["status"], "FAILED")
        self.assertEqual(waiting["info"]["message"], "任务需要人工确认")


if __name__ == "__main__":
    unittest.main()
