#!/usr/bin/env python3
"""Unit tests for sku_geometry.py."""
import unittest
from pathlib import Path
import sys
import numpy as np

DEPLOY_DIR = Path(__file__).resolve().parents[1] / 'deploy'
if str(DEPLOY_DIR) not in sys.path:
    sys.path.insert(0, str(DEPLOY_DIR))
for _sub in ('core', 'perception', 'pipeline', 'algorithms'):
    _p = str(DEPLOY_DIR / _sub)
    if _p not in sys.path:
        sys.path.insert(0, _p)

import sku_geometry
from geometry_cache import GeometryCache


class TestSkuGeometry(unittest.TestCase):
    def test_cad_geometry_lookup(self):
        # Explicit override
        req = {'sku_geometry': {'depth_axis_size_mm': 65.0, 'lateral_axis_size_mm': 35.0}}
        geo = sku_geometry.get_sku_geometry(req)
        self.assertEqual(geo['depth_axis_size_mm'], 65.0)
        self.assertEqual(geo['lateral_axis_size_mm'], 35.0)

        # Config lookup for standard SKU types
        req_bottle = {'sku_typ': 'bottle'}
        geo_bot = sku_geometry.get_sku_geometry(req_bottle)
        self.assertEqual(geo_bot['depth_axis_size_mm'], 80.0)

    def test_box_frame_and_feature_extraction(self):
        roi = [100.0, 100.0, 300.0, 300.0]
        K = {'fx': 500.0, 'fy': 500.0, 'cx': 320.0, 'cy': 240.0}
        T_m = np.eye(4)
        req = {'camera_frame': 'head_camera_color_optical_frame', 'base_frame': 'chassis_link'}
        bf = sku_geometry.build_box_frame(roi, None, req, K, T_m)
        self.assertIsNotNone(bf.origin_robot_mm)
        self.assertIsNotNone(bf.inward_axis_robot)

        H, W = 480, 640
        depth = np.full((H, W), 500.0, dtype=np.float32)
        mask = np.zeros((H, W), dtype=bool)
        mask[150:200, 150:200] = True
        det = {'upstream_instance_id': 1, 'bbox': [150, 150, 50, 50], '_mask': mask}

        cache = GeometryCache()
        feat = sku_geometry.extract_instance_features(det, depth, K, T_m, bf, cache)
        self.assertIn('depth_inward_mm', feat)
        self.assertIn('lateral_mm', feat)
        self.assertGreater(feat['valid_point_count'], 0)


if __name__ == '__main__':
    unittest.main()
