#!/usr/bin/env python3
"""Unit tests for sku_target_fitting.py (Items 13, 14, 15, 17)."""
import unittest
from pathlib import Path
import sys
from unittest.mock import MagicMock
import numpy as np

DEPLOY_DIR = Path(__file__).resolve().parents[1] / 'deploy'
if str(DEPLOY_DIR) not in sys.path:
    sys.path.insert(0, str(DEPLOY_DIR))
for _sub in ('core', 'perception', 'pipeline', 'algorithms'):
    _p = str(DEPLOY_DIR / _sub)
    if _p not in sys.path:
        sys.path.insert(0, _p)

from geometry_cache import GeometryCache
from sku_target_fitting import (
    CATEGORY_FITTERS,
    build_targets_dict,
    fit_category_targets,
    fit_single_bottle_target,
    fit_single_box_target,
    fit_single_tube_target,
)


class TestTargetFittingContract(unittest.TestCase):
    def setUp(self):
        self.depth = np.full((480, 640), 500.0, dtype=float)
        self.K = {'fx': 600.0, 'fy': 600.0, 'cx': 320.0, 'cy': 240.0}
        self.T_m = np.eye(4)
        self.cache = GeometryCache()

    def test_item13_bottle_empty_candidate_contract(self):
        # 13. Bottle empty candidate returns valid=False and expected reasons
        res = fit_single_bottle_target(
            None, {}, self.depth, self.K, self.T_m, self.cache
        )
        self.assertFalse(res['valid'])
        self.assertIn('no target candidate', res['reasons'])

    def test_item14_tube_empty_candidate_contract(self):
        # 14. Tube empty candidate returns valid=False and expected reasons
        res = fit_single_tube_target(
            None, {}, self.depth, self.K, self.T_m, self.cache
        )
        self.assertFalse(res['valid'])
        self.assertIn('no target candidate', res['reasons'])

    def test_item15_box_empty_candidate_contract(self):
        # 15. Box empty candidate returns valid=False and expected reasons
        res = fit_single_box_target(
            None, {}, self.depth, self.K, self.T_m, self.cache
        )
        self.assertFalse(res['valid'])
        self.assertIn('no target candidate', res['reasons'])

    def test_item17_top_level_compatibility_points_to_center(self):
        # 17. Verify that top-level fields in fit_category_targets strictly match the center candidate
        center_item = {'instance_id': 12, 'score': 0.95}
        center_fit = {
            'valid': True,
            'axis_ok': True,
            'ref_ok': True,
            'point_robot_mm': [100.0, 20.0, 500.0],
            'point_camera_mm': [20.0, 10.0, 480.0],
            'axis_point_camera_mm': [0.0, 0.0, 500.0],
            'axis_direction_camera_up': [0.0, -1.0, 0.0],
            'reasons': [],
        }

        left_item = {'instance_id': 11, 'score': 0.90}
        left_fit = {'valid': True, 'point_robot_mm': [90.0, -50.0, 500.0], 'point_camera_mm': [-50.0, 10.0, 480.0]}

        right_item = {'instance_id': 13, 'score': 0.91}
        right_fit = {'valid': True, 'point_robot_mm': [90.0, 90.0, 500.0], 'point_camera_mm': [90.0, 10.0, 480.0]}

        targets = build_targets_dict('bottle', center_item, center_fit, left_item, left_fit, right_item, right_fit)
        self.assertEqual(targets['center']['instance_id'], 12)
        self.assertEqual(targets['left']['instance_id'], 11)
        self.assertEqual(targets['right']['instance_id'], 13)

        # Check top-level contract matches center
        self.assertEqual(targets['center']['point_camera_mm'], center_fit['point_camera_mm'])
        self.assertEqual(targets['center']['point_chassis_mm'], center_fit['point_robot_mm'])


if __name__ == '__main__':
    unittest.main()
