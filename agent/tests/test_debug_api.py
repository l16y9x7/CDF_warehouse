import json
import tempfile
import threading
import unittest
from pathlib import Path
from unittest.mock import patch

import httpx
import numpy as np

from agent.api import create_app
from agent.application import build_application_from_capabilities
from agent.capabilities.camera import MockCameraCapability, capture_with_event
from agent.capabilities.common import TargetType
from agent.capabilities.estimation import MockEstimationCapability, encode_frame_file
from agent.capabilities.estimation.mock import RECORDED_INFER_RESPONSE
from agent.debug.catalog import _pick_pose_request
from agent.capabilities.hand import MockHandCapability
from agent.capabilities.manipulation import MockManipulationCapability
from agent.capabilities.navigation import MockNavigationCapability
from agent.capabilities.perception import MockPerceptionCapability
from agent.capabilities.pose import MockBodyPoseCapability
from agent.capabilities.vla import MockVlaCapability
from agent.debug.store import DebugStore
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


class DebugApiTest(unittest.IsolatedAsyncioTestCase):
    async def asyncSetUp(self):
        self.temp = tempfile.TemporaryDirectory()
        path = Path(self.temp.name) / "tasks.db"
        self.application = build_application_from_capabilities(
            capabilities(),
            database_path=path,
            calibration_source=lambda: {
                "K": ((612.0, 0, 641), (0, 611, 358), (0, 0, 1)),
                "T_chassis_camera": ((1, 0, 0, 0), (0, 1, 0, 0), (0, 0, 1, 0), (0, 0, 0, 1)),
                "T_unit": "m",
            },
        )
        self.app = create_app(self.application, callback_url="http://callback")
        self.debug = self.app.state.debug

        runtime = self.debug.applications["mock"].runtime
        runtime.callback_sender.client.close()

        def callback(request: httpx.Request) -> httpx.Response:
            run_id = request.url.path.rsplit("/", 1)[-1]
            self.debug.callback(run_id, json.loads(request.content))
            return httpx.Response(204)

        runtime.callback_sender = CallbackSender(
            runtime.store,
            backoff=0,
            client=httpx.Client(transport=httpx.MockTransport(callback)),
        )
        self.client = httpx.AsyncClient(
            transport=httpx.ASGITransport(app=self.app), base_url="http://test"
        )

    async def asyncTearDown(self):
        await self.client.aclose()
        self.debug.shutdown()
        self.application.runtime.shutdown()
        self.app.state.mock_agent.close()
        self.app.state.logging_manager.shutdown()
        self.temp.cleanup()

    async def test_catalog_and_embedded_assets(self):
        response = await self.client.get("/debug/api/catalog")
        self.assertEqual(response.status_code, 200)
        catalog = response.json()
        self.assertEqual(catalog["counts"]["workflow"], 3)
        keys = {(item["layer"], item["name"]) for item in catalog["items"]}
        self.assertEqual(len(keys), len(catalog["items"]))
        fields = [field for item in catalog["items"] for field in item["fields"]]
        self.assertTrue(fields)
        self.assertTrue(all(field["description"] for field in fields))
        self.assertTrue(all(field["options"] for field in fields))
        pick = next(item for item in catalog["items"] if item["name"] == "estimation.pick_pose")
        pick_fields = {item["name"]: item for item in pick["fields"]}
        self.assertEqual(pick_fields["sku_typ"]["type"], "select")
        self.assertEqual(pick_fields["sku_typ"]["options"], ["bottle", "box", "tube"])
        self.assertEqual(pick_fields["side"]["type"], "select")
        self.assertEqual(pick_fields["side"]["options"], ["LEFT", "RIGHT"])
        self.assertEqual(pick_fields["target_type"]["type"], "select")
        self.assertEqual(pick_fields["target_type"]["options"], ["sku", "basket"])
        self.assertIn("box", pick_fields["sku_typ"]["options"])
        self.assertIn("LEFT", pick_fields["side"]["options"])
        self.assertEqual(pick_fields["test_case"]["type"], "select")
        self.assertIn("rgb", pick_fields)
        self.assertIn("depth", pick_fields)
        self.assertFalse(pick_fields["rgb"]["required"])
        self.assertFalse(pick_fields["depth"]["required"])
        for name in ("camera_frame", "base_frame"):
            self.assertEqual(pick_fields[name]["type"], "text")
            self.assertTrue(pick_fields[name]["required"])
        for layer, operation in (
            ("capability", "manipulation.pick"),
            ("skill", "pick_sku_standard"),
        ):
            item = next(
                item
                for item in catalog["items"]
                if item["layer"] == layer and item["name"] == operation
            )
            level = next(field for field in item["fields"] if field["name"] == "level")
            self.assertTrue(level["required"])
            self.assertNotIn("default", level)
            self.assertEqual(level["type"], "select")
            self.assertEqual(level["options"], ["L1", "L2", "L3", "L4", "L5"])

        sku_ids = ["3282779003131", "887167608641", "7173342765403"]
        sku_names = [
            "Avene 雅漾 雅漾舒泉调理喷雾 300ml",
            "Estee Lauder 雅诗兰黛 雅诗兰黛特润修护肌活精华眼霜双支装 15ml*2",
            "Origins 悦木之源 ORIGINS一举两得泡沫洁面慕斯 30ml",
        ]
        for item in catalog["items"]:
            for entry in item["fields"]:
                if entry["name"] in {"sku_id", "expected_sku_id"}:
                    self.assertEqual(entry["options"], sku_ids)
                if entry["name"] == "name":
                    self.assertEqual(entry["options"], sku_names)

        redirect = await self.client.get("/debug", follow_redirects=False)
        self.assertEqual(redirect.status_code, 307)
        self.assertEqual(redirect.headers["location"], "/debug/")
        for path, content_type in (
            ("/debug/", "text/html"),
            ("/debug/assets/app.js", "text/javascript"),
            ("/debug/assets/styles.css", "text/css"),
        ):
            asset = await self.client.get(path)
            self.assertEqual(asset.status_code, 200)
            self.assertIn(content_type, asset.headers["content-type"])

        script = (await self.client.get("/debug/assets/app.js")).text
        self.assertIn('const REAL_TARGET_PASSWORD = "zhongmian123"', script)
        self.assertIn('window.prompt("请输入实机操作密码")', script)
        self.assertIn('class="select-menu hidden"', script)
        self.assertIn("请选择或输入自定义值", script)
        self.assertIn('field.type === "select"', script)
        self.assertIn('field.required ? "请选择"', script)
        self.assertIn('skip.has(field.name)', script)
        self.assertIn("selectedMediaKey", script)
        self.assertIn("drawPoseOverlay", script)
        self.assertIn("projectCameraPoint", script)
        self.assertNotIn("已叠加估姿", script)
        self.assertIn("beginPendingRun", script)
        self.assertIn("selectHistoryRun", script)
        self.assertIn("formatDuration", script)
        self.assertIn("viewGeneration", script)
        self.assertIn("执行中…", script)
        self.assertIn(" 秒", script)
        self.assertNotIn('${failed ? "open" : ""}', script)
        index = (await self.client.get("/debug/")).text
        self.assertIn('id="media-thumbnails"', index)
        self.assertIn('id="preview-overlay"', index)

    async def test_test_cases_endpoint_lists_local_cases(self):
        response = await self.client.get("/debug/api/test-cases")
        self.assertEqual(response.status_code, 200)
        cases = response.json()["cases"]
        self.assertIsInstance(cases, list)
        for case in cases:
            self.assertTrue(case["name"])
            for name in ("sku_typ", "side", "target_type", "rgb_base64", "depth_npy_base64"):
                self.assertNotIn(name, case)
            self.assertIn("K", case)
            self.assertIn("T_chassis_camera", case)

    async def test_validation_and_nonphysical_capability_run(self):
        unknown = await self.client.post(
            "/debug/api/runs",
            json={
                "target": "mock",
                "layer": "capability",
                "operation": "camera.missing",
                "payload": {},
            },
        )
        self.assertEqual(unknown.status_code, 422)

        extra = await self.client.post(
            "/debug/api/runs",
            json={
                "target": "mock",
                "layer": "capability",
                "operation": "camera.health",
                "payload": {"extra": True},
            },
        )
        self.assertEqual(extra.status_code, 422)

        run = await self.client.post(
            "/debug/api/runs",
            json={
                "target": "mock",
                "layer": "capability",
                "operation": "camera.health",
                "payload": {},
            },
        )
        self.assertEqual(run.status_code, 200)
        self.assertEqual(run.json()["status"], "SUCCEEDED")
        self.assertEqual(run.json()["result"], {"status": "READY"})

    async def test_physical_confirmation_is_bound_and_one_time(self):
        request = {
            "target": "mock",
            "layer": "capability",
            "operation": "navigation.navigate",
            "payload": {"nav_id": "AGV_L"},
        }
        missing = await self.client.post("/debug/api/runs", json=request)
        self.assertEqual(missing.status_code, 422)

        ticket = (await self.client.post("/debug/api/confirmations", json=request)).json()
        changed = {**request, "payload": {"nav_id": "AGV_R"}, "confirmation_token": ticket["token"]}
        mismatch = await self.client.post("/debug/api/runs", json=changed)
        self.assertEqual(mismatch.status_code, 422)

        valid = {**request, "confirmation_token": ticket["token"]}
        first = await self.client.post("/debug/api/runs", json=valid)
        self.assertEqual(first.status_code, 200)
        self.assertEqual(first.json()["status"], "SUCCEEDED")
        reused = await self.client.post("/debug/api/runs", json=valid)
        self.assertEqual(reused.status_code, 422)

    async def test_skill_events_and_history_persist(self):
        response = await self.client.post(
            "/debug/api/runs",
            json={
                "target": "mock",
                "layer": "skill",
                "operation": "summarize_review_result",
                "payload": {"expected_items": [], "inspected_items": []},
            },
        )
        run = response.json()
        self.assertEqual(run["status"], "SUCCEEDED")
        self.assertTrue(run["events"])

        listed = await self.client.get(
            "/debug/api/runs", params={"target": "mock", "layer": "skill"}
        )
        self.assertEqual(listed.json()["runs"][0]["run_id"], run["run_id"])
        reopened = DebugStore(self.application.store.path).get_run(run["run_id"])
        self.assertEqual(reopened["result"], run["result"])

    async def test_logs_api_filters_chinese_structured_logs(self):
        self.debug.log_store.append(
            {
                "timestamp": "2026-09-18T08:00:00.000Z",
                "level": "ERROR",
                "logger": "agent.test",
                "event": "skill.failed",
                "message": "商品抓取技能执行失败",
                "task_id": "log-task",
                "run_id": "log-run",
                "error_code": "PICK_FAILED",
                "error_message": "需要人工检查机械臂状态",
            },
            1_800_000_000,
        )
        response = await self.client.get(
            "/debug/api/logs",
            params={"task_id": "log-task", "search": "人工检查"},
        )
        self.assertEqual(response.status_code, 200)
        body = response.json()
        self.assertEqual(body["logs"][0]["run_id"], "log-run")
        self.assertIsNone(body["next_cursor"])

    async def test_mock_capture_and_media_security(self):
        response = await self.client.post(
            "/debug/api/runs",
            json={
                "target": "mock",
                "layer": "capability",
                "operation": "camera.capture",
                "payload": {"camera": "head", "streams": "color", "format": "raw"},
            },
        )
        path = response.json()["result"]["color"]["path"]
        self.assertEqual(response.json()["media"][0]["path"], path)
        self.assertEqual(response.json()["media"][0]["stream"], "color")
        image = await self.client.get("/debug/api/media", params={"path": path})
        self.assertEqual(image.status_code, 200)
        self.assertEqual(image.headers["content-type"], "image/png")
        self.assertTrue(image.content.startswith(b"\x89PNG"))

        forbidden = await self.client.get("/debug/api/media", params={"path": "/etc/passwd"})
        self.assertEqual(forbidden.status_code, 403)

    async def test_run_media_includes_pose_overlay(self):
        self.debug.store.create_run("pose-overlay", "mock", "skill", "pick_sku_standard", {})
        self.debug.store.set_status(
            "pose-overlay",
            "SUCCEEDED",
            result=RECORDED_INFER_RESPONSE,
            events=[
                {
                    "event": "camera.captured",
                    "capture_id": "capture-pose",
                    "camera": "head",
                    "skill": "pick_sku_standard",
                    "color_intrinsics": [
                        [600.0, 0.0, 640.0],
                        [0.0, 600.0, 360.0],
                        [0.0, 0.0, 1.0],
                    ],
                    "color": {
                        "path": "/shared/frames/capture-pose/rgb.jpg",
                        "format": "jpeg",
                        "width": 1280,
                        "height": 720,
                    },
                }
            ],
        )
        detail = (await self.client.get("/debug/api/runs/pose-overlay")).json()
        overlay = detail["media"][0]["overlay"]
        self.assertTrue(overlay["hud"])
        self.assertTrue(any(point["label"] == "参考点" for point in overlay["points"]))
        self.assertEqual(overlay["K"][0][2], 640.0)

    async def test_raw_depth_is_converted_to_png(self):
        media_root = Path(self.temp.name) / "frames"
        media_root.mkdir()
        depth_path = media_root / "depth.npy"
        np.save(depth_path, np.array([[0.0, 100.0], [200.0, np.nan]], dtype=np.float32))
        with patch.dict("os.environ", {"AGENT_DEBUG_MEDIA_ROOTS": str(media_root)}):
            response = await self.client.get(
                "/debug/api/media", params={"path": str(depth_path)}
            )
        self.assertEqual(response.status_code, 200)
        self.assertEqual(response.headers["content-type"], "image/png")
        self.assertTrue(response.content.startswith(b"\x89PNG"))

    async def test_invalid_raw_depth_is_rejected(self):
        media_root = Path(self.temp.name) / "invalid-frames"
        media_root.mkdir()
        depth_path = media_root / "depth.npy"
        np.save(depth_path, np.zeros((2, 2, 2), dtype=np.float32))
        with patch.dict("os.environ", {"AGENT_DEBUG_MEDIA_ROOTS": str(media_root)}):
            response = await self.client.get(
                "/debug/api/media", params={"path": str(depth_path)}
            )
        self.assertEqual(response.status_code, 422)
        self.assertEqual(response.json()["error_code"], "DEPTH_PREVIEW_INVALID")

    def test_status_update_without_events_preserves_live_events(self):
        self.debug.store.create_run(
            "live-events", "mock", "workflow", "sorting_finish", {}, task_id="live-task"
        )
        events = [{"event": "camera.captured", "capture_id": "capture-live"}]
        self.debug.store.attach_events("live-task", events)
        self.debug.store.set_status("live-events", "RUNNING", finished=False)
        self.assertEqual(self.debug.store.get_run("live-events")["events"], events)

    async def test_workflow_media_is_visible_before_completion(self):
        runtime = self.debug.applications["mock"].runtime
        camera = self.debug.applications["mock"].capabilities["camera"]
        started = threading.Event()
        release = threading.Event()
        original = self.debug.applications["mock"].workflows["sorting_finish"]

        class CapturingWorkflow:
            def run(self, context, data):
                capture_with_event(context, camera, "head", skill="test_capture")
                started.set()
                release.wait(2)
                return {}

        self.debug.applications["mock"].workflows["sorting_finish"] = CapturingWorkflow()
        try:
            submitted = await self.client.post(
                "/debug/api/workflows/sorting_finish",
                json={"target": "mock", "payload": {"basket_row": "L1", "basket_column": "1"}},
            )
            run = submitted.json()
            self.assertTrue(started.wait(1))
            detail = (await self.client.get(f"/debug/api/runs/{run['run_id']}")).json()
            self.assertIn(detail["status"], {"ACCEPTED", "RUNNING"})
            self.assertEqual(detail["media"][0]["camera"], "head")
        finally:
            release.set()
            runtime.wait(run["task_id"])
            self.debug.applications["mock"].workflows["sorting_finish"] = original

    async def test_workflow_uses_async_callback_and_saves_events(self):
        response = await self.client.post(
            "/debug/api/workflows/sorting_finish",
            json={"target": "mock", "payload": {"basket_row": "L1", "basket_column": "1"}},
        )
        self.assertEqual(response.status_code, 200)
        accepted = response.json()
        self.assertIn(accepted["status"], {"ACCEPTED", "SUCCEEDED"})
        self.debug.applications["mock"].runtime.wait(accepted["task_id"])

        detail = (await self.client.get(f"/debug/api/runs/{accepted['run_id']}")).json()
        self.assertEqual(detail["status"], "SUCCEEDED")
        self.assertEqual(detail["result"], {})
        self.assertTrue(detail["events"])
        self.assertTrue(detail["media"])

        trace_response = await self.client.get(
            f"/debug/api/runs/{accepted['run_id']}/trace"
        )
        self.assertEqual(trace_response.status_code, 200)
        spans = trace_response.json()["spans"]
        workflow = next(span for span in spans if span["kind"] == "workflow")
        skill = next(span for span in spans if span["kind"] == "skill")
        capability = next(
            span for span in spans
            if span["kind"] == "capability" and span["parent_span_id"] == skill["span_id"]
        )
        self.assertIsNone(workflow["parent_span_id"])
        self.assertEqual(skill["parent_span_id"], workflow["span_id"])
        self.assertEqual(capability["node_id"], skill["node_id"])
        self.assertIsNotNone(skill["input"])
        self.assertIsNotNone(skill["output"])
        self.assertIsNotNone(capability["input"])
        self.assertIsNotNone(capability["output"])
        preflight = next(span for span in spans if span["kind"] == "preflight")
        self.assertIsNone(preflight["parent_span_id"])
        self.assertTrue(any(
            span["kind"] == "capability"
            and span["operation"] == "health"
            and span["parent_span_id"] == preflight["span_id"]
            for span in spans
        ))

    async def test_callback_rejects_a_different_task(self):
        self.debug.store.create_run(
            "callback-run", "mock", "workflow", "review", {}, task_id="expected-task"
        )
        response = await self.client.post(
            "/debug/api/callbacks/callback-run",
            json={"task_id": "different-task", "status": "SUCCEEDED", "result": {}},
        )
        self.assertEqual(response.status_code, 422)
        self.assertEqual(self.debug.store.get_run("callback-run")["status"], "RUNNING")

    async def test_terminate_is_scoped_to_target_and_idle_is_conflict(self):
        idle = await self.client.post("/debug/api/terminate", json={"target": "mock"})
        self.assertEqual(idle.status_code, 409)
        self.assertEqual(idle.json()["error_code"], "NO_ACTIVE_TASK")

        runtime = self.debug.applications["mock"].runtime
        started = threading.Event()
        release = threading.Event()
        original = self.debug.applications["mock"].workflows["sorting_finish"]

        class BlockingWorkflow:
            def run(self, context, data):
                started.set()
                release.wait(2)
                return original.run(context, data)

        self.debug.applications["mock"].workflows["sorting_finish"] = BlockingWorkflow()
        submitted = await self.client.post(
            "/debug/api/workflows/sorting_finish",
            json={"target": "mock", "payload": {"basket_row": "L1", "basket_column": "1"}},
        )
        run = submitted.json()
        self.assertTrue(started.wait(1))

        terminated = await self.client.post("/debug/api/terminate", json={"target": "mock"})
        self.assertEqual(terminated.status_code, 200)
        self.assertEqual(terminated.json()["task_id"], run["task_id"])
        real = await self.client.post("/debug/api/terminate", json={"target": "real"})
        self.assertEqual(real.status_code, 409)

        release.set()
        runtime.wait(run["task_id"])
        detail = (await self.client.get(f"/debug/api/runs/{run['run_id']}")).json()
        self.assertEqual(detail["status"], "CANCELLED")


if __name__ == "__main__":
    unittest.main()


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


class PickPoseDebugRequestTest(unittest.TestCase):
    def test_form_overrides_case_and_accepts_other_sku_side(self):
        request = _pick_pose_request(
            {
                "test_case": "bottle_120045958",
                "target_type": "sku",
                "sku_typ": "box",
                "side": "LEFT",
            }
        )
        self.assertIs(request.target_type, TargetType.SKU)
        self.assertEqual(request.sku_typ, "box")
        self.assertEqual(request.side, "LEFT")
        self.assertTrue(request.rgb_base64)
        self.assertTrue(request.depth_npy_base64)

    def test_case_does_not_supply_job_fields(self):
        with self.assertRaisesRegex(ValueError, "sku_typ"):
            _pick_pose_request(
                {"test_case": "bottle_120045958", "target_type": "sku"}
            )

    def test_rgb_depth_paths_encode_and_override_case_frames(self):
        with tempfile.TemporaryDirectory() as tmp:
            rgb = Path(tmp) / "rgb.jpg"
            depth = Path(tmp) / "depth_mm.npy"
            rgb.write_bytes(bytes([0xFF, 0xD8]) + b"custom-rgb")
            depth.write_bytes(bytes([0x93]) + b"NUMPYcustom-depth")
            request = _pick_pose_request(
                {
                    "test_case": "bottle_120045958",
                    "target_type": "sku",
                    "sku_typ": "tube",
                    "side": "RIGHT",
                    "rgb": str(rgb),
                    "depth": str(depth),
                    "K": [list(row) for row in K],
                    "T_chassis_camera": [list(row) for row in T_CHASSIS_CAMERA],
                }
            )
            self.assertEqual(request.sku_typ, "tube")
            self.assertEqual(request.rgb_base64, encode_frame_file(rgb))
            self.assertEqual(request.depth_npy_base64, encode_frame_file(depth))
