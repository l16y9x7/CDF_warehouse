#!/usr/bin/env python3
"""Unit tests for response sanitization and ROBOT_API_HANDOFF contract enforcement."""
import json
from pathlib import Path
import sys
import unittest
import numpy as np

DEPLOY_DIR = Path(__file__).resolve().parents[1] / 'deploy'
if str(DEPLOY_DIR) not in sys.path:
    sys.path.insert(0, str(DEPLOY_DIR))
for _sub in ('core', 'perception', 'pipeline', 'algorithms'):
    _p = str(DEPLOY_DIR / _sub)
    if _p not in sys.path:
        sys.path.insert(0, _p)

from response_schema import (
    build_robot_response,
    sanitize_artifacts,
    sanitize_diagnostics,
    sanitize_http_response,
    sanitize_targets,
)
import server


class TestResponseSanitization(unittest.TestCase):
    def setUp(self):
        self.maxDiff = None

    def test_bottle_response_strips_internal_arrays(self):
        """Verify that large internal arrays (fm, raw, fit, ins, center_fit) are removed from bottle response."""
        heavy_mask = np.ones((720, 1280), dtype=bool)
        raw_cloud = np.random.randn(5000, 3)
        fit_cloud = np.random.randn(2000, 3)
        ins_mask = np.ones(2000, dtype=bool)

        internal_result = {
            'ok': True,
            'request_id': '20260924_120000_1234abcd',
            'target_type': 'sku',
            'sku_typ': 'bottle',
            'class_name': 'bottle',
            'localization_method': 'bottle_cylinder_axis',
            'selected_instance_id': 3,
            'upstream_instance_id': 3,
            'filtered_instance_id': 1,
            'sam3_score': 0.85,
            'axis_fit_valid': True,
            'reference_point_valid': True,
            'axis_point_camera_mm': [100.0, 50.0, 500.0],
            'axis_direction_camera_up': [0.0, -1.0, 0.0],
            'reference_point_camera_mm': [100.0, 50.0, 500.0],
            'reference_point_chassis_mm': [500.0, 20.0, 1200.0],
            'reference_mode': 'visible_axis_midpoint',
            'front_panel_valid': True,
            'front_panel_top_edge_midpoint_camera_mm': [105.0, 110.0, 510.0],
            'front_panel_plane_point_camera_mm': [110.0, 120.0, 515.0],
            'front_panel_plane_normal_camera': [0.0, 0.5, -0.866],
            'box_selection': {
                'enabled': True,
                'target_box': 1,
                'stage_status': 'target_instances_filtered',
                'valid': True,
                'reason': 'ok',
            },
            'center_fit': {
                'raw': raw_cloud,
                'fit': fit_cloud,
                'ins': ins_mask,
                'fm': heavy_mask,
                'p': [100.0, 50.0, 500.0],
                'a': [0.0, -1.0, 0.0],
            },
            'left_fit': {'valid': False, 'reasons': ['no left target']},
            'right_fit': {'valid': False, 'reasons': ['no right target']},
            'diagnostics': {
                'mask_valid_depth_ratio': 0.95,
                'fit_region_point_count': 5000,
                'fit_inlier_count': 2000,
                'radial_median_mm': 0.5,
                'radial_p90_mm': 1.8,
                'raw_points_leak': raw_cloud,  # leak simulation
            },
            'artifacts': {
                'all_instances_overlay': '/path/to/all_instances_overlay.jpg',
                'selected_instance_overlay': '/path/to/selected_overlay.jpg',
                'point_cloud': '/path/to/selected_points.ply',
                'json_log': '/path/to/response.json',
                'all_instances_overlay_jpeg_base64': 'LARGE_BASE64_STRING' * 1000,
                'selected_overlay_jpeg_base64': 'LARGE_BASE64_STRING' * 1000,
            },
        }

        # Wire serialization without return_visualizations
        robot_resp = build_robot_response(internal_result, req={'return_visualizations': False})

        # Assert internal large fields are completely removed
        self.assertNotIn('center_fit', robot_resp)
        self.assertNotIn('left_fit', robot_resp)
        self.assertNotIn('right_fit', robot_resp)
        self.assertNotIn('raw', robot_resp)
        self.assertNotIn('fit', robot_resp)
        self.assertNotIn('ins', robot_resp)
        self.assertNotIn('fm', robot_resp)
        self.assertNotIn('raw_points_leak', robot_resp['diagnostics'])

        # Assert no base64 in artifacts
        self.assertNotIn('all_instances_overlay_jpeg_base64', robot_resp['artifacts'])
        self.assertNotIn('selected_overlay_jpeg_base64', robot_resp['artifacts'])
        self.assertIn('all_instances_overlay', robot_resp['artifacts'])
        self.assertIn('json_log', robot_resp['artifacts'])

        # Assert contract fields present and correct
        self.assertTrue(robot_resp['ok'])
        self.assertEqual(robot_resp['sku_typ'], 'bottle')
        self.assertTrue(robot_resp['axis_fit_valid'])
        self.assertTrue(robot_resp['reference_point_valid'])
        self.assertEqual(robot_resp['reference_point_camera_mm'], [100.0, 50.0, 500.0])
        self.assertEqual(robot_resp['reference_point_chassis_mm'], [500.0, 20.0, 1200.0])
        self.assertIsNone(robot_resp['reference_z_mm'])

        # Wire payload size must be < 30 KB
        payload = json.dumps(robot_resp, ensure_ascii=False).encode('utf-8')
        self.assertLess(len(payload), 30 * 1024)

    def test_box_response_strips_analyses(self):
        """Verify that multi-erosion analyses and geometric point clouds are removed from box response."""
        analyses_mock = {
            '0': {'camera_points': np.random.randn(8000, 3), 'fit': {'valid': True}},
            '1': {'camera_points': np.random.randn(7000, 3), 'fit': {'valid': True}},
            '2': {'camera_points': np.random.randn(6000, 3), 'fit': {'valid': True}},
        }

        internal_result = {
            'ok': True,
            'request_id': '20260924_120001_5678efgh',
            'target_type': 'sku',
            'sku_typ': 'box',
            'class_name': 'box',
            'localization_method': 'box_top_surface',
            'selected_instance_id': 2,
            'top_plane_valid': True,
            'top_point_valid': True,
            'point_semantics': 'top_surface_geometric_center',
            'point_source': 'geometric',
            'fallback_used': False,
            'fallback_reason': None,
            'top_point_camera_mm': [-50.0, 30.0, 450.0],
            'top_point_chassis_mm': [350.0, -100.0, 1150.0],
            'top_point_uv': [320.0, 240.0],
            'top_plane_point_camera_mm': [-50.0, 30.0, 450.0],
            'top_plane_point_chassis_mm': [350.0, -100.0, 1150.0],
            'plane_normal_camera': [0.0, 0.2, -0.98],
            'plane_normal_chassis': [0.0, 0.0, 1.0],
            'center_fit': {
                'analyses': analyses_mock,
                'point_camera_mm': [-50.0, 30.0, 450.0],
            },
            'diagnostics': {
                'valid_depth_ratio': 0.98,
                'top_inlier_count': 4200,
                'top_residual_median_mm': 1.1,
                'top_residual_p90_mm': 2.3,
                'analyses': analyses_mock,  # leak simulation
            },
            'artifacts': {
                'all_instances_overlay': '/path/to/all_instances_overlay.jpg',
                'selected_instance_overlay': '/path/to/selected_overlay.jpg',
                'all_instances_overlay_jpeg_base64': 'BASE64_DATA' * 500,
            },
        }

        robot_resp = build_robot_response(internal_result, req={'return_visualizations': False})

        self.assertNotIn('center_fit', robot_resp)
        self.assertNotIn('analyses', robot_resp)
        self.assertNotIn('analyses', robot_resp['diagnostics'])
        self.assertNotIn('all_instances_overlay_jpeg_base64', robot_resp['artifacts'])

        self.assertTrue(robot_resp['ok'])
        self.assertEqual(robot_resp['sku_typ'], 'box')
        self.assertTrue(robot_resp['top_plane_valid'])
        self.assertTrue(robot_resp['top_point_valid'])
        self.assertEqual(robot_resp['point_semantics'], 'top_surface_geometric_center')
        self.assertEqual(robot_resp['top_point_camera_mm'], [-50.0, 30.0, 450.0])

        payload = json.dumps(robot_resp, ensure_ascii=False).encode('utf-8')
        self.assertLess(len(payload), 30 * 1024)

    def test_tube_response_sanitization(self):
        """Verify tube response conforms to ROBOT_API_HANDOFF contract."""
        internal_result = {
            'ok': True,
            'request_id': '20260924_120002_tube1234',
            'target_type': 'sku',
            'sku_typ': 'tube',
            'class_name': 'tube',
            'localization_method': 'tube_height_band_edge',
            'tube_fit_mode': 'height_band',
            'selected_instance_id': 1,
            'edge_valid': True,
            'point_valid': True,
            'point_semantics': 'visible_top_edge_midpoint',
            'top_edge_center_camera_mm': [10.0, 20.0, 400.0],
            'top_edge_center_chassis_mm': [400.0, 50.0, 1100.0],
            'top_edge_endpoints_camera_mm': [[5.0, 20.0, 400.0], [15.0, 20.0, 400.0]],
            'edge_direction_camera': [1.0, 0.0, 0.0],
            'rejection_reasons': [],
            'diagnostics': {
                'edge_point_count': 150,
                'edge_residual_median_mm': 0.8,
            },
            'artifacts': {
                'all_instances_overlay': '/path/to/all.jpg',
                'selected_instance_overlay': '/path/to/sel.jpg',
            },
        }

        robot_resp = build_robot_response(internal_result, req={'return_visualizations': False})
        self.assertTrue(robot_resp['ok'])
        self.assertEqual(robot_resp['sku_typ'], 'tube')
        self.assertTrue(robot_resp['edge_valid'])
        self.assertTrue(robot_resp['point_valid'])
        self.assertEqual(robot_resp['point_semantics'], 'visible_top_edge_midpoint')
        self.assertEqual(robot_resp['top_edge_center_camera_mm'], [10.0, 20.0, 400.0])
        self.assertEqual(len(robot_resp['top_edge_endpoints_camera_mm']), 2)

        payload = json.dumps(robot_resp, ensure_ascii=False).encode('utf-8')
        self.assertLess(len(payload), 30 * 1024)

    def test_basket_response_sanitization(self):
        """Verify basket response conforms to ROBOT_API_HANDOFF Section 6.1."""
        pose_mm = np.eye(4)
        pose_mm[:3, 3] = [50.0, 10.0, 800.0]
        pose_m = np.eye(4)
        pose_m[:3, 3] = [0.05, 0.01, 0.8]

        internal_result = {
            'ok': True,
            'request_id': '20260924_120003_basket12',
            'target_type': 'basket',
            'pose_valid': True,
            'model_center_camera_mm': [50.0, 10.0, 800.0],
            'reference_point_camera_mm': [50.0, 10.0, 800.0],
            'reference_point_chassis_mm': [600.0, 0.0, 1250.0],
            'xyz_camera_mm': [50.0, 10.0, 800.0],
            'point_semantics': 'basket_model_center',
            'object_origin_camera_mm': [40.0, 5.0, 780.0],
            'model_center_offset_m': [0.01, 0.005, 0.02],
            'pose_4x4': pose_mm,
            'pose_4x4_input_m': pose_m,
            'rotation_euler_zyx_rad': [0.1, 0.2, 0.3],
            'xyzrxryrz_camera_mm_rad': [50.0, 10.0, 800.0, 0.1, 0.2, 0.3],
            'foundationpose_http_status': 200,
            'foundationpose_wall_ms': 120.5,
            'rejection_reasons': [],
            'diagnostics': {'foundationpose_response_ok': True},
            'artifacts': {
                'all_instances_overlay': '/path/to/all.jpg',
                'vis_pose_path_base64': 'BASE64_DATA' * 200,
            },
        }

        # Case 1: return_visualizations = False
        robot_resp_no_viz = build_robot_response(internal_result, req={'return_visualizations': False})
        self.assertTrue(robot_resp_no_viz['ok'])
        self.assertEqual(robot_resp_no_viz['target_type'], 'basket')
        self.assertEqual(robot_resp_no_viz['point_semantics'], 'basket_model_center')
        self.assertNotIn('vis_pose_path_base64', robot_resp_no_viz['artifacts'])
        self.assertLess(len(json.dumps(robot_resp_no_viz)), 30 * 1024)

        # Case 2: return_visualizations = True
        robot_resp_with_viz = build_robot_response(internal_result, req={'return_visualizations': True})
        self.assertIn('vis_pose_path_base64', robot_resp_with_viz['artifacts'])

    def test_return_visualizations_control(self):
        """Ensure base64 payloads are only embedded when return_visualizations: true."""
        arts = {
            'all_instances_overlay': '/path/overlay.jpg',
            'all_instances_overlay_jpeg_base64': 'BASE64_A',
            'point_cloud': '/path/points.ply',
            'point_cloud_ply_base64': 'BASE64_B',
        }
        stripped = sanitize_artifacts(arts, return_visualizations=False)
        self.assertIn('all_instances_overlay', stripped)
        self.assertIn('point_cloud', stripped)
        self.assertNotIn('all_instances_overlay_jpeg_base64', stripped)
        self.assertNotIn('point_cloud_ply_base64', stripped)

        kept = sanitize_artifacts(arts, return_visualizations=True)
        self.assertIn('all_instances_overlay_jpeg_base64', kept)
        self.assertIn('point_cloud_ply_base64', kept)

    def test_multi_target_sanitization(self):
        """Ensure multi-target slots (center, left, right) only contain clean scalars and points."""
        targets = {
            'center': {
                'instance_id': 10,
                'valid': True,
                'point_camera_mm': [1.0, 2.0, 3.0],
                'point_chassis_mm': [4.0, 5.0, 6.0],
                'axis_direction_camera_up': [0.0, -1.0, 0.0],
                'axis_direction_robot': [0.0, 0.0, 1.0],
                'score': 0.92,
                'reasons': [],
                'reason': 'front-row instance nearest box center',
                'internal_heavy_mask': np.ones((100, 100)),  # must be stripped
            }
        }
        cleaned = sanitize_targets(targets)
        self.assertIn('center', cleaned)
        self.assertEqual(cleaned['center']['instance_id'], 10)
        self.assertTrue(cleaned['center']['valid'])
        self.assertNotIn('internal_heavy_mask', cleaned['center'])

    def test_server_charset_header(self):
        """Ensure server Handler._send sets application/json; charset=utf-8."""
        class MockHandler(server.Handler):
            def __init__(self):
                self.headers_sent = {}
            def send_response(self, code):
                self.code = code
            def send_header(self, key, val):
                self.headers_sent[key] = val
            def end_headers(self):
                pass
            def wfile_write(self, data):
                pass
            wfile = property(lambda self: type('WFile', (), {'write': lambda s, d: None})())

        h = MockHandler()
        h._send(200, {'ok': True})
        self.assertEqual(h.headers_sent.get('Content-Type'), 'application/json; charset=utf-8')


if __name__ == '__main__':
    unittest.main()
