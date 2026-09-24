import asyncio
import sqlite3
import tempfile
import unittest
from pathlib import Path

import httpx

from agent.api import create_app
from agent.application import build_application_from_capabilities
from agent.capabilities.camera import MockCameraCapability
from agent.capabilities.estimation import MockEstimationCapability
from agent.capabilities.hand import MockHandCapability
from agent.capabilities.manipulation import MockManipulationCapability
from agent.capabilities.navigation import MockNavigationCapability
from agent.capabilities.perception import MockPerceptionCapability
from agent.capabilities.pose import MockBodyPoseCapability
from agent.capabilities.vla import MockVlaCapability
from agent.config import load_sku_catalog
from agent.contracts import AgentError
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


class OrderApiTest(unittest.IsolatedAsyncioTestCase):
    async def asyncSetUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.database = Path(self.temp.name) / "tasks.db"
        caps = capabilities()
        caps["perception"].barcode_content = "3282779003131"
        self.application = build_application_from_capabilities(
            caps,
            database_path=self.database,
            sku_catalog=load_sku_catalog(),
            calibration_source=lambda: {
                "K": ((612.0, 0, 641), (0, 611, 358), (0, 0, 1)),
                "T_chassis_camera": (
                    (1, 0, 0, 0),
                    (0, 1, 0, 0),
                    (0, 0, 1, 0),
                    (0, 0, 0, 1),
                ),
                "T_unit": "m",
            },
        )
        self.app = create_app(self.application, callback_url="http://callback")
        for application in (self.application, self.app.state.mock_agent):
            application.runtime.callback_sender.close()
            application.runtime.callback_sender = CallbackSender(
                application.store,
                backoff=0,
                client=httpx.Client(
                    transport=httpx.MockTransport(lambda request: httpx.Response(204))
                ),
            )
        self.client = httpx.AsyncClient(
            transport=httpx.ASGITransport(app=self.app), base_url="http://test"
        )

    async def asyncTearDown(self):
        await self.client.aclose()
        self.app.state.orders.close()
        self.app.state.debug.shutdown()
        self.application.runtime.shutdown()
        self.app.state.mock_agent.close()
        self.app.state.logging_manager.shutdown()
        self.temp.cleanup()

    def _assert_barcode_copy(self, events, name, sku_id, count):
        checking = f"正在核对 {name} 编码为 {sku_id}"
        progresses = [
            event["payload"].get("info", {}).get("progress")
            for event in events
            if event["type"] == "agent.progress"
            and event["payload"].get("info", {}).get("skill") == "核对商品"
        ]
        self.assertEqual(progresses, [checking, "商品正确"] * count)

    async def wait_for_status(self, order_id, statuses, timeout=20):
        deadline = asyncio.get_running_loop().time() + timeout
        while asyncio.get_running_loop().time() < deadline:
            response = await self.client.get(f"/orders/api/orders/{order_id}")
            order = response.json()
            if order["status"] in statuses:
                return order
            await asyncio.sleep(0.02)
        self.fail(f"order did not reach {statuses}")

    async def test_catalog_assets_and_complete_multi_item_order(self):
        products = await self.client.get("/orders/api/products")
        self.assertEqual(products.status_code, 200)
        self.assertEqual(len(products.json()["products"]), 3)
        self.assertNotIn("agv_row", products.json()["products"][0])

        for path, content_type in (
            ("/orders/", "text/html"),
            ("/orders/assets/app.js", "text/javascript"),
            ("/orders/assets/styles.css", "text/css"),
            ("/orders/assets/bottle.svg", "image/svg+xml"),
        ):
            response = await self.client.get(path)
            self.assertEqual(response.status_code, 200)
            self.assertIn(content_type, response.headers["content-type"])
            if path == "/orders/":
                self.assertIn("真机", response.text)
                self.assertIn("Mock", response.text)

        response = await self.client.post(
            "/orders/api/orders",
            json={
                "items": [{
                    "sku_id": "3282779003131", "quantity": 2,
                    "agv_row": "L4", "agv_column": "2",
                }],
                "basket_row": "L2",
                "basket_column": "3",
            },
        )
        self.assertEqual(response.status_code, 201)
        order_id = response.json()["order_id"]
        order = await self.wait_for_status(order_id, {"SUCCEEDED"})
        self.assertEqual(order["completed_items"], 2)
        self.assertEqual(order["progress_percent"], 100)
        self.assertEqual(order["basket_row"], "L2")
        self.assertEqual(order["basket_column"], "3")
        self.assertEqual(order["items"][0]["agv_row"], "L4")
        self.assertEqual(order["items"][0]["agv_column"], "2")
        self.assertIs(order["mock"], False)

        with sqlite3.connect(self.database) as connection:
            task_types = [
                row[0]
                for row in connection.execute(
                    "SELECT task_type FROM tasks WHERE task_id LIKE ? ORDER BY created_at, rowid",
                    (f"{order_id}%",),
                )
            ]
        self.assertEqual(task_types, ["sorting_item", "sorting_item"])
        runs = sorted(
            self.app.state.debug.store.list_runs(target="real"),
            key=lambda run: (run["started_at"], run["task_id"] or ""),
        )
        self.assertEqual(
            [run["operation"] for run in runs],
            ["sorting_item", "sorting_item"],
        )
        self.assertTrue(all(run["status"] == "SUCCEEDED" for run in runs))
        self.assertTrue(all(str(run["task_id"]).startswith(order_id) for run in runs))
        detail = (await self.client.get(f"/debug/api/runs/{runs[0]['run_id']}")).json()
        self.assertEqual(detail["layer"], "workflow")
        self.assertEqual(detail["status"], "SUCCEEDED")
        self.assertTrue(detail["events"])
        self.assertEqual(detail["request"]["sku_id"], "3282779003131")
        trace = (await self.client.get(f"/debug/api/runs/{runs[0]['run_id']}/trace")).json()
        self.assertTrue(any(span["kind"] == "workflow" for span in trace["spans"]))
        self.assertTrue(any(span["kind"] == "skill" for span in trace["spans"]))
        events = self.app.state.orders.events_after(order_id, 0)
        self.assertTrue(any(event["type"] == "agent.progress" for event in events))
        self.assertEqual(events[-1]["type"], "order.succeeded")
        self._assert_barcode_copy(events, "雅漾舒护活泉水", "3282779003131", 2)

    async def test_barcode_mismatch_is_reported_as_success_on_the_order_page(self):
        self.application.capabilities["perception"].barcode_content = "wrong-barcode"
        response = await self.client.post(
            "/orders/api/orders",
            json={
                "items": [{
                    "sku_id": "3282779003131", "quantity": 1,
                    "agv_row": "L1", "agv_column": "1",
                }],
                "basket_row": "L1",
                "basket_column": "1",
            },
        )
        self.assertEqual(response.status_code, 201)
        order_id = response.json()["order_id"]
        order = await self.wait_for_status(order_id, {"SUCCEEDED"})
        self.assertEqual(order["status"], "SUCCEEDED")
        events = self.app.state.orders.events_after(order_id, 0)
        self._assert_barcode_copy(events, "雅漾舒护活泉水", "3282779003131", 1)

    async def test_failure_retries_once_then_pauses_and_manual_retry_continues(self):
        original = self.application.workflows["sorting_item"]

        class FailingWorkflow:
            def run(self, context, data):
                raise AgentError("TEST_FAILURE", "模拟抓取失败")

        self.application.workflows["sorting_item"] = FailingWorkflow()
        response = await self.client.post(
            "/orders/api/orders",
            json={
                "items": [{
                    "sku_id": "3282779003131", "quantity": 1,
                    "agv_row": "L1", "agv_column": "1",
                }],
                "basket_row": "L1",
                "basket_column": "2",
            },
        )
        order_id = response.json()["order_id"]
        paused = await self.wait_for_status(order_id, {"PAUSED"})
        self.assertEqual(paused["units"][0]["attempts"], 2)
        self.assertEqual(paused["units"][0]["status"], "PAUSED")
        self.assertIn("TEST_FAILURE", paused["error"])

        second = await self.client.post(
            "/orders/api/orders",
            json={
                "items": [{
                    "sku_id": "3282779003131", "quantity": 1,
                    "agv_row": "L2", "agv_column": "2",
                }],
                "basket_row": "L1",
                "basket_column": "2",
            },
        )
        self.assertEqual(second.status_code, 409)
        self.assertEqual(second.json()["error_code"], "ORDER_BUSY")

        self.application.workflows["sorting_item"] = original
        retried = await self.client.post(f"/orders/api/orders/{order_id}/retry", json={})
        self.assertEqual(retried.status_code, 200)
        completed = await self.wait_for_status(order_id, {"SUCCEEDED"})
        self.assertEqual(completed["units"][0]["attempts"], 3)

    async def test_mock_order_runs_on_mock_runtime(self):
        response = await self.client.post(
            "/orders/api/orders",
            json={
                "items": [{
                    "sku_id": "3282779003131", "quantity": 1,
                    "agv_row": "L1", "agv_column": "1",
                }],
                "basket_row": "L1",
                "basket_column": "1",
                "mock": True,
            },
        )
        self.assertEqual(response.status_code, 201)
        self.assertIs(response.json()["mock"], True)
        order_id = response.json()["order_id"]
        await self.wait_for_status(order_id, {"SUCCEEDED", "PAUSED"})
        with sqlite3.connect(self.app.state.mock_agent.store.path) as connection:
            mock_ids = [
                row[0]
                for row in connection.execute(
                    "SELECT task_id FROM tasks WHERE task_id LIKE ?",
                    (f"{order_id}%",),
                )
            ]
        with sqlite3.connect(self.database) as connection:
            real_ids = [
                row[0]
                for row in connection.execute(
                    "SELECT task_id FROM tasks WHERE task_id LIKE ?",
                    (f"{order_id}%",),
                )
            ]
        self.assertTrue(mock_ids)
        self.assertEqual(real_ids, [])
        mock_runs = self.app.state.debug.store.list_runs(target="mock")
        self.assertTrue(any(str(run["task_id"]).startswith(order_id) for run in mock_runs))
        self.assertTrue(all(run["layer"] == "workflow" for run in mock_runs if str(run["task_id"]).startswith(order_id)))

    async def test_validation_and_missing_order(self):
        unknown = await self.client.post(
            "/orders/api/orders",
            json={
                "items": [{
                    "sku_id": "missing", "quantity": 1,
                    "agv_row": "L1", "agv_column": "1",
                }],
                "basket_row": "L1",
                "basket_column": "1",
            },
        )
        self.assertEqual(unknown.status_code, 422)
        missing = await self.client.get("/orders/api/orders/not-found")
        self.assertEqual(missing.status_code, 404)


if __name__ == "__main__":
    unittest.main()
