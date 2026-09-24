import os
import time
import unittest
from unittest.mock import patch

import httpx

from agent.capabilities.mocks.apps import (
    camera_app,
    estimation_app,
    hand_app,
    manipulation_app,
    navigation_app,
    perception_app,
    pose_app,
    vla_app,
)
from agent.capabilities.mocks.delay import mock_process_delay_s
from agent.capabilities.navigation import MockNavigationCapability

POSE = {
    "pose": [100, 20, 400, 0, 0, 0],
    "hand": "RIGHT",
    "frame": "camera",
    "pose_unit": "mm_rad",
    "rotation_order": "zyx",
}


async def request(app, method, path, **kwargs):
    async with httpx.AsyncClient(
        transport=httpx.ASGITransport(app=app), base_url="http://test"
    ) as client:
        return await client.request(method, path, **kwargs)


class CapabilityMockApiTest(unittest.IsolatedAsyncioTestCase):
    async def test_health_endpoints(self):
        for app, endpoint in (
            (navigation_app, "/navigation/health"),
            (pose_app, "/pose/health"),
            (perception_app, "/perception/health"),
            (estimation_app, "/health"),
            (manipulation_app, "/manipulation/health"),
            (vla_app, "/vla/health"),
            (hand_app, "/hand/health"),
        ):
            self.assertEqual((await request(app, "GET", endpoint)).json(), {"status": "READY"})

    def test_process_delay_is_off_while_unittest_is_loaded(self):
        env = os.environ.copy()
        env.pop("AGENT_MOCK_PROCESS_DELAY_S", None)
        with patch.dict(os.environ, env, clear=True):
            self.assertEqual(mock_process_delay_s(), 0.0)

    async def test_configured_process_delay_applies_before_each_response(self):
        with patch.dict(os.environ, {"AGENT_MOCK_PROCESS_DELAY_S": "0.2"}):
            started = time.monotonic()
            MockNavigationCapability().health()
            response = await request(navigation_app, "GET", "/navigation/health")
            elapsed = time.monotonic() - started
        self.assertEqual(response.json(), {"status": "READY"})
        self.assertGreaterEqual(elapsed, 0.35)

    async def test_perception_contract_and_removed_endpoints(self):
        found = await request(
            perception_app,
            "POST",
            "/perception/locate_basket",
            json={"image_path": "/shared/item.jpg"},
        )
        self.assertEqual(found.status_code, 404)
        empty = await request(
            perception_app,
            "POST",
            "/perception/basket/locate_item",
            json={"image_path": "/shared/empty.jpg"},
        )
        self.assertEqual(empty.json(), {"status": "NOT_FOUND"})
        found = await request(
            perception_app,
            "POST",
            "/perception/basket/locate_item",
            json={"image_path": "/shared/item.jpg"},
        )
        self.assertEqual(
            found.json(),
            {
                "status": "FOUND",
                "bbox": [100, 200, 200, 400],
                "mask": "mock-mask:basket-item",
            },
        )
        barcode = await request(
            perception_app,
            "POST",
            "/perception/recognize_sku_barcode",
            json={"image_base64": "aW1hZ2U=", "sku_id": "sku-x", "name": "x"},
        )
        self.assertEqual(barcode.json(), {"barcode_content": "sku-x"})
        missing_barcode = await request(
            perception_app,
            "POST",
            "/perception/recognize_sku_barcode",
            json={"image_base64": "aW1hZ2U=", "sku_id": "", "name": ""},
        )
        self.assertEqual(missing_barcode.json(), {"status": "NOT_FOUND"})
        legacy_barcode = await request(
            perception_app,
            "POST",
            "/perception/recognize_sku_barcode",
            json={"image_path": "/x", "sku_id": "sku-x", "name": "x"},
        )
        self.assertEqual(legacy_barcode.status_code, 422)
        self.assertEqual(
            (
                await request(
                    perception_app,
                    "POST",
                    "/perception/locate_blue_light",
                    json={"image_path": "/x"},
                )
            ).status_code,
            404,
        )
        self.assertEqual(
            (await request(manipulation_app, "POST", "/manipulation/press", json={})).status_code,
            404,
        )

    async def test_estimation_conditional_contract(self):
        infer = {
            "target_type": "sku",
            "sku_typ": "bottle",
            "side": "LEFT",
            "rgb_base64": "cmdi",
            "depth_npy_base64": "bm9weQ==",
            "depth_unit": "mm",
            "K": [[612.0, 0.0, 641.0], [0.0, 611.0, 358.0], [0.0, 0.0, 1.0]],
            "T_chassis_camera": [
                [1.0, 0.0, 0.0, 0.1],
                [0.0, 1.0, 0.0, 0.2],
                [0.0, 0.0, 1.0, 1.5],
                [0.0, 0.0, 0.0, 1.0],
            ],
            "T_unit": "m",
            "camera_frame": "head_camera_color_optical_frame",
            "base_frame": "chassis_link",
            "front_rule": {
                "front_axis_chassis": [1.0, 0.0, 0.0],
                "front_origin_chassis": [0.0, 0.0, 0.0],
                "front_band_mm": 100000.0,
            },
        }
        accepted = await request(estimation_app, "POST", "/infer", json=infer)
        self.assertEqual(accepted.status_code, 200)
        payload = accepted.json()
        self.assertTrue(payload["ok"])
        self.assertTrue(payload["axis_fit_valid"])
        self.assertTrue(payload["reference_point_valid"])
        self.assertEqual(payload["reference_point_camera_mm"][0], 162.778093460864)
        self.assertEqual(payload.get("sku_typ"), "bottle")
        without_front = {key: value for key, value in infer.items() if key != "front_rule"}
        self.assertEqual(
            (await request(estimation_app, "POST", "/infer", json=without_front)).status_code,
            200,
        )
        self.assertEqual(
            (
                await request(
                    estimation_app,
                    "POST",
                    "/infer",
                    json={k: v for k, v in infer.items() if k != "sku_typ"} | {"sku_id": "bottle"},
                )
            ).status_code,
            422,
        )
        no_side = {key: value for key, value in infer.items() if key != "side"}
        self.assertEqual(
            (
                await request(
                    estimation_app, "POST", "/infer", json={**no_side, "sku_typ": "box"}
                )
            ).status_code,
            422,
        )
        self.assertEqual(
            (
                await request(
                    estimation_app,
                    "POST",
                    "/infer",
                    json={**infer, "sku_typ": "box", "side": "RIGHT"},
                )
            ).status_code,
            200,
        )
        self.assertEqual(
            (
                await request(
                    estimation_app,
                    "POST",
                    "/infer",
                    json={**infer, "rgb_base64": "<BASE64_OF_RGB>"},
                )
            ).status_code,
            422,
        )
        self.assertEqual(
            (
                await request(
                    estimation_app, "POST", "/infer", json={**infer, "target_type": "basket"}
                )
            ).status_code,
            422,
        )
        basket = {key: value for key, value in infer.items() if key not in {"sku_typ", "side", "front_rule"}}
        basket["target_type"] = "basket"
        accepted_basket = await request(estimation_app, "POST", "/infer", json=basket)
        self.assertEqual(accepted_basket.status_code, 200)
        basket_payload = accepted_basket.json()
        self.assertTrue(basket_payload["ok"])
        self.assertTrue(basket_payload["pose_valid"])
        self.assertEqual(basket_payload["point_semantics"], "basket_model_center")
        self.assertEqual(basket_payload["reference_point_camera_mm"], [200.0, 30.0, 520.0])
        # 旧 /estimation/pick_pose 端点已随 mask 链路移除
        self.assertEqual(
            (
                await request(estimation_app, "POST", "/estimation/pick_pose", json={})
            ).status_code,
            404,
        )

    async def test_actions_and_vla_status(self):
        headers = {"Idempotency-Key": "mock-action-1"}
        self.assertEqual(
            (
                await request(
                    navigation_app,
                    "POST",
                    "/navigation/navigate",
                    json={"nav_id": "AGV_C"},
                    headers=headers,
                )
            ).json(),
            {"status": "SUCCEEDED"},
        )
        self.assertEqual(
            (
                await request(
                    manipulation_app,
                    "POST",
                    "/manipulation/pick",
                    json={
                        "task_type": "SORTING",
                        "target_type": "sku",
                        "sku_typ": "bottle",
                        "hand": "RIGHT",
                        "level": "L5",
                        "localization_result": {"ok": True},
                    },
                    headers=headers,
                )
            ).json(),
            {
                "status": "SUCCEEDED",
                "box_clearance": {"frame": "trunk_controller_ref", "unit": "mm"},
                "completed_moves": 6,
            },
        )
        invalid_pick = await request(
            manipulation_app,
            "POST",
            "/manipulation/pick",
            json={
                "task_type": "SORTING",
                "target_type": "sku",
                "sku_typ": "bottle",
                "hand": "RIGHT",
                "level": "L6",
                "localization_result": {"ok": True},
            },
            headers=headers,
        )
        self.assertEqual(invalid_pick.status_code, 422)
        place = await request(
            manipulation_app,
            "POST",
            "/manipulation/place",
            json={
                "task_type": "SORTING",
                "target_type": "sku",
                "destination_type": "basket",
                "hand": "RIGHT",
                "sku_typ": "bottle",
                "localization_result": {
                    "ok": True,
                    "target_type": "basket",
                    "pose_valid": True,
                    "point_semantics": "basket_model_center",
                    "model_center_camera_mm": [200.0, 30.0, 520.0],
                    "pose_4x4": [
                        [1.0, 0.0, 0.0, 50.0],
                        [0.0, 1.0, 0.0, 10.0],
                        [0.0, 0.0, 1.0, 400.0],
                        [0.0, 0.0, 0.0, 1.0],
                    ],
                },
            },
            headers=headers,
        )
        self.assertEqual(place.json(), {"status": "SUCCEEDED"})
        self.assertEqual(
            (
                await request(
                    manipulation_app,
                    "POST",
                    "/manipulation/push",
                    json={
                        "hand": "RIGHT",
                        "ok": True,
                        "pose_valid": True,
                        "point_semantics": "basket_model_center",
                        "model_center_camera_mm": [200.0, 30.0, 520.0],
                        "pose_4x4": [
                            [1.0, 0.0, 0.0, 50.0],
                            [0.0, 1.0, 0.0, 10.0],
                            [0.0, 0.0, 1.0, 400.0],
                            [0.0, 0.0, 0.0, 1.0],
                        ],
                    },
                    headers=headers,
                )
            ).json(),
            {"status": "SUCCEEDED"},
        )
        self.assertEqual(
            (
                await request(
                    manipulation_app,
                    "POST",
                    "/manipulation/pick",
                    json={
                        "task_type": "REVIEW",
                        "target_type": "basket",
                        "hand": "RIGHT",
                        "ok": True,
                        "pose_valid": True,
                        "point_semantics": "basket_model_center",
                        "model_center_camera_mm": [200.0, 30.0, 520.0],
                        "pose_4x4": [
                            [1.0, 0.0, 0.0, 50.0],
                            [0.0, 1.0, 0.0, 10.0],
                            [0.0, 0.0, 1.0, 400.0],
                            [0.0, 0.0, 0.0, 1.0],
                        ],
                    },
                    headers=headers,
                )
            ).json(),
            {"status": "SUCCEEDED"},
        )
        self.assertEqual(
            (
                await request(
                    manipulation_app,
                    "POST",
                    "/manipulation/pick_review_item",
                    json=POSE,
                    headers=headers,
                )
            ).json(),
            {"status": "SUCCEEDED"},
        )
        rotate = await request(
            manipulation_app,
            "POST",
            "/manipulation/rotate",
            json={"hand": "RIGHT", "sku_typ": "bottle"},
            headers=headers,
        )
        self.assertEqual(rotate.status_code, 200)
        self.assertEqual(rotate.json()["status"], "SUCCEEDED")
        self.assertEqual(rotate.json()["camera"], "left_wrist")
        missing_sku_typ = await request(
            manipulation_app,
            "POST",
            "/manipulation/rotate",
            json={"hand": "RIGHT"},
            headers=headers,
        )
        self.assertEqual(missing_sku_typ.status_code, 422)
        self.assertEqual(vla_app.state.result_status, "PICKED")
        vla_app.state.result_status = "NOT_FOUND"
        self.assertEqual(
            (
                await request(
                    vla_app,
                    "POST",
                    "/vla/pick_review_item",
                    json={"hand": "RIGHT"},
                    headers=headers,
                )
            ).json(),
            {"status": "NOT_FOUND"},
        )
        vla_app.state.result_status = "PICKED"

    async def test_physical_actions_require_idempotency_key(self):
        missing = await request(
            navigation_app, "POST", "/navigation/navigate", json={"nav_id": "AGV_C"}
        )
        self.assertEqual(missing.status_code, 400)
        self.assertEqual(missing.json()["error_code"], "INVALID_INPUT")

    async def test_camera_returns_unique_shared_paths(self):
        first = (
            await request(
                camera_app, "GET", "/camera/capture", params={"camera": "head"}
            )
        ).json()
        second = (
            await request(
                camera_app, "GET", "/camera/capture", params={"camera": "head"}
            )
        ).json()
        self.assertNotEqual(first["capture_id"], second["capture_id"])
        self.assertIsNone(first["depth"])
        rgbd = await request(
            camera_app,
            "GET",
            "/camera/capture",
            params={"camera": "right_wrist", "streams": "depth,color"},
        )
        body = rgbd.json()
        self.assertTrue(body["ok"])
        self.assertTrue(body["same_shot"])
        self.assertTrue(body["depth"]["aligned"])
        self.assertIn(f"/{body['capture_id']}/", body["color"]["path"])
        self.assertIn(f"/{body['capture_id']}/", body["depth"]["path"])

        ignored_format = await request(
            camera_app,
            "GET",
            "/camera/capture",
            params={"camera": "right_wrist", "streams": "color,depth", "format": "preview"},
        )
        self.assertEqual(ignored_format.json()["depth"]["format"], "raw")

    async def test_camera_capture_rejects_invalid_parameters(self):
        missing = await request(
            camera_app, "GET", "/camera/capture", params={"camera": "unknown"}
        )
        self.assertEqual(missing.status_code, 404)
        self.assertEqual(missing.json()["error_code"], "CAMERA_NOT_FOUND")

        streams = await request(
            camera_app,
            "GET",
            "/camera/capture",
            params={"camera": "head", "streams": "color,color"},
        )
        self.assertEqual(streams.json()["error_code"], "INVALID_STREAMS")

        depth_format = await request(
            camera_app,
            "GET",
            "/camera/capture",
            params={"camera": "head", "streams": "depth", "format": "jpeg"},
        )
        self.assertEqual(depth_format.json()["error_code"], "INVALID_FORMAT")

    async def test_camera_formal_health_and_rgbd_routes(self):
        health = await request(camera_app, "GET", "/camera/health")
        self.assertTrue(health.json()["ok"])
        rgbd = await request(camera_app, "GET", "/camera/rgbd", params={"camera": "head"})
        self.assertTrue(rgbd.json()["same_shot"])
        self.assertEqual(rgbd.json()["color_intrinsics"]["width"], 1280)

    async def test_camera_exposes_no_other_formal_routes(self):
        for route in ("/camera/list", "/camera/snapshot", "/camera/stream"):
            with self.subTest(route=route):
                response = await request(camera_app, "GET", route)
                self.assertEqual(response.status_code, 404)


if __name__ == "__main__":
    unittest.main()
