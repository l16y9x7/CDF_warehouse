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
        payload = {
            **RECORDED_INFER_RESPONSE,
            "input_summary": {"K": {"fx": 600.0, "fy": 600.0, "cx": 640.0, "cy": 360.0}},
            "box_selection": {
                "container_roi_xyxy": [678.5, 148, 1240, 550],
                "box_candidates": [{"bbox": [676, 148, 564, 402], "score": 0.9}],
                "object_membership": [{"mask_centroid_xy": [823.2, 350.2], "upstream_instance_id": 1}],
            },
        }
        overlay = overlay_from_pose(payload)
        self.assertIsNotNone(overlay)
        assert overlay is not None
        self.assertEqual(overlay["K"][0][2], 640.0)
        labels = {item["label"] for item in overlay["points"]}
        self.assertIn("参考点", labels)
        self.assertIn("轴线点", labels)
        self.assertTrue(any(item["kind"] == "xyxy" for item in overlay["boxes"]))
        self.assertTrue(any(item["kind"] == "xywh" for item in overlay["boxes"]))
        self.assertEqual(overlay["markers"][0]["xy"][0], 823.2)
        self.assertIn("ok", overlay["hud"][0])
        self.assertTrue(any(item["label"] == "轴线" for item in overlay["lines"]))

    def test_basket_overlay_draws_cad_axes(self):
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
        axis_labels = {item["label"] for item in overlay["lines"]}
        self.assertEqual(axis_labels, {"CAD X", "CAD Y", "CAD Z"})
        self.assertEqual(overlay["points"][0]["xyz_mm"][2], 520.0)

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


if __name__ == "__main__":
    unittest.main()
