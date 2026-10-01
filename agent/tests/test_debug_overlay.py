import unittest

from agent.capabilities.estimation.mock import RECORDED_INFER_RESPONSE
from agent.debug.media import media_from_run
from agent.debug.overlay import overlay_from_pose, overlays_from_run


K = [
    [600.0, 0.0, 640.0],
    [0.0, 600.0, 360.0],
    [0.0, 0.0, 1.0],
]


class PoseOverlayTest(unittest.TestCase):
    def test_overlay_projects_points_and_keeps_pixel_boxes(self):
        """测试简化后的overlay只显示关键信息"""
        payload = {
            **RECORDED_INFER_RESPONSE,
            "input_summary": {"K": {"fx": 600.0, "fy": 600.0, "cx": 640.0, "cy": 360.0}},
            "box_selection": {
                "container_roi_xyxy": [678.5, 148, 1240, 550],
                "box_candidates": [{"bbox": [676, 148, 564, 402], "score": 0.9}],
                "object_membership": [
                    {"mask_centroid_xy": [100.0, 200.0], "upstream_instance_id": 1},
                    {"mask_centroid_xy": [823.2, 350.2], "upstream_instance_id": 3},
                ],
            },
        }
        overlay = overlay_from_pose(payload)
        self.assertIsNotNone(overlay)
        assert overlay is not None
        self.assertEqual(overlay["K"][0][2], 640.0)

        # 只显示参考点，不显示轴线点
        labels = {item["label"] for item in overlay["points"]}
        self.assertIn("参考点", labels)
        self.assertEqual(len(overlay["points"]), 1)

        # 参考点应该是大尺寸且根据ok状态着色（绿色=成功）
        ref_point = overlay["points"][0]
        self.assertEqual(ref_point["size"], "large")
        self.assertEqual(ref_point["color"], "#22c55e")  # 绿色表示ok=True

        # 不显示检测框
        self.assertEqual(len(overlay["boxes"]), 0)

        # 只显示选中的实例(id=3)
        self.assertEqual(len(overlay["markers"]), 1)
        self.assertEqual(overlay["markers"][0]["xy"][0], 823.2)
        self.assertEqual(overlay["markers"][0]["label"], "实例 3")

        # HUD应该包含ok状态和score
        self.assertIn("ok", overlay["hud"][0])
        self.assertIn("score", overlay["hud"][0])

        # 应该有轴线
        self.assertTrue(any(item["label"] == "轴线" for item in overlay["lines"]))
        self.assertEqual(len(overlay["lines"]), 1)

    def test_basket_overlay_draws_cad_axes(self):
        """测试篮筐overlay显示CAD坐标系"""
        overlay = overlay_from_pose(
            {
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
                "K": K,
            }
        )
        self.assertIsNotNone(overlay)
        assert overlay is not None

        # 应该显示CAD坐标系的XYZ轴
        axis_labels = {item["label"] for item in overlay["lines"]}
        self.assertEqual(axis_labels, {"CAD X", "CAD Y", "CAD Z"})

        # 应该显示参考点和篮筐中心
        point_labels = {item["label"] for item in overlay["points"]}
        self.assertIn("参考点", point_labels)
        self.assertIn("篮筐中心", point_labels)

        # 参考点位置正确
        ref_point = next(p for p in overlay["points"] if p["label"] == "参考点")
        self.assertEqual(ref_point["xyz_mm"][2], 520.0)

    def test_media_attaches_overlay_from_estimation_span(self):
        run = {
            "events": [
                {
                    "event": "camera.captured",
                    "capture_id": "capture-1",
                    "camera": "head",
                    "skill": "pick_sku_standard",
                    "color_intrinsics": K,
                    "color": {"path": "/shared/frames/capture-1/rgb.jpg", "format": "jpeg", "width": 1280, "height": 720},
                    "depth": {"path": "/shared/frames/capture-1/depth_mm.npy", "format": "raw", "width": 1280, "height": 720},
                }
            ],
            "result": {"sku_id": "3282779003131", "backend": "STANDARD"},
        }
        spans = [
            {"span_id": "s1", "kind": "skill", "name": "pick_sku_standard", "operation": "execute"},
            {
                "span_id": "s2",
                "parent_span_id": "s1",
                "kind": "capability",
                "name": "estimation",
                "operation": "estimate_pick_pose",
                "input": {"request": {"K": K}},
                "output": RECORDED_INFER_RESPONSE,
            },
        ]
        media = media_from_run(run, spans=spans)
        color = next(item for item in media if item["stream"] == "color")
        self.assertEqual(color["overlay"]["skill"], "pick_sku_standard")
        self.assertEqual(color["overlay"]["K"][0][0], 600.0)
        self.assertTrue(any(point["label"] == "参考点" for point in color["overlay"]["points"]))
        depth = next(item for item in media if item["stream"] == "depth")
        self.assertEqual(depth["overlay"]["skill"], "pick_sku_standard")

    def test_result_overlay_falls_back_to_latest_capture(self):
        run = {
            "events": [
                {
                    "event": "camera.captured",
                    "capture_id": "capture-1",
                    "camera": "head",
                    "color": {"path": "/shared/frames/capture-1/rgb.jpg", "format": "jpeg"},
                }
            ],
            "result": {**RECORDED_INFER_RESPONSE, "K": K},
        }
        overlays = overlays_from_run(run, [])
        self.assertEqual(len(overlays), 1)
        media = media_from_run(run)
        self.assertIn("overlay", media[0])
        self.assertEqual(media[0]["overlay"]["K"][1][2], 360.0)

    def test_reference_point_color_coding(self):
        """测试参考点根据ok状态和score进行颜色编码"""
        # 成功且高置信度 -> 绿色
        overlay = overlay_from_pose({
            "ok": True,
            "target_type": "sku",
            "reference_point_camera_mm": [100.0, 50.0, 600.0],
            "sam3_score": 0.85,
        })
        self.assertEqual(overlay["points"][0]["color"], "#22c55e")  # 绿色

        # 成功但低置信度 -> 橙色
        overlay = overlay_from_pose({
            "ok": True,
            "target_type": "sku",
            "reference_point_camera_mm": [100.0, 50.0, 600.0],
            "sam3_score": 0.65,
        })
        self.assertEqual(overlay["points"][0]["color"], "#fb923c")  # 橙色

        # 失败 -> 红色
        overlay = overlay_from_pose({
            "ok": False,
            "target_type": "sku",
            "reference_point_camera_mm": [100.0, 50.0, 600.0],
            "sam3_score": 0.3,
            "rejection_reasons": ["depth_invalid"],
        })
        self.assertEqual(overlay["points"][0]["color"], "#ef4444")  # 红色
        self.assertIn("拒绝:", overlay["hud"][-1])

    def test_only_selected_instance_marker(self):
        """测试只显示选中的实例标记"""
        payload = {
            "ok": True,
            "target_type": "sku",
            "reference_point_camera_mm": [100.0, 50.0, 600.0],
            "selected_instance_id": 2,
            "box_selection": {
                "object_membership": [
                    {"mask_centroid_xy": [100.0, 200.0], "upstream_instance_id": 1},
                    {"mask_centroid_xy": [300.0, 400.0], "upstream_instance_id": 2},
                    {"mask_centroid_xy": [500.0, 600.0], "upstream_instance_id": 3},
                ]
            },
        }
        overlay = overlay_from_pose(payload)
        # 只应该显示实例2
        self.assertEqual(len(overlay["markers"]), 1)
        self.assertEqual(overlay["markers"][0]["label"], "实例 2")
        self.assertEqual(overlay["markers"][0]["xy"], [300.0, 400.0])


if __name__ == "__main__":
    unittest.main()
