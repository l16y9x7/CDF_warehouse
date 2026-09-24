#!/usr/bin/env python3
"""Unit tests for sku_selection.py (Items 10, 11, 12)."""
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

import sku_selection
from sku_geometry import build_box_frame


class TestSkuSelection(unittest.TestCase):
    def test_item10_merged_proposal_detection(self):
        # 10. Merge suspect flagged when sample >= 3 and ratio exceeds threshold
        inst1 = {'instance_id': 1, 'valid_depth': True, 'mask_area_px': 5000, 'bbox_width_px': 100}
        inst2 = {'instance_id': 2, 'valid_depth': True, 'mask_area_px': 2500, 'bbox_width_px': 50}
        inst3 = {'instance_id': 3, 'valid_depth': True, 'mask_area_px': 2600, 'bbox_width_px': 50}
        suspects = sku_selection.detect_merged_instances([inst1, inst2, inst3])
        self.assertTrue(inst1['merged_suspect'])
        self.assertFalse(inst2['merged_suspect'])

    def test_item11_front_row_depth_grouping(self):
        # 11. Row partitioning: gap > 0.60 * D_inward splits into separate rows
        items = [
            {'instance_id': 1, 'depth_inward_mm': 100.0, 'valid_depth_points': 100},
            {'instance_id': 2, 'depth_inward_mm': 120.0, 'valid_depth_points': 100},  # gap 20 <= 48
            {'instance_id': 3, 'depth_inward_mm': 180.0, 'valid_depth_points': 100},  # gap 60 > 48
        ]
        rows = sku_selection.group_rows_by_depth(items, 80.0)
        self.assertEqual(len(rows), 2)
        self.assertEqual({x['instance_id'] for x in rows[0]}, {1, 2})
        self.assertEqual({x['instance_id'] for x in rows[1]}, {3})

    def test_item12_center_left_right_selection_and_deduplication(self):
        # 12. Strict deduplication among center, left, right targets
        roi = [100.0, 100.0, 500.0, 400.0]
        bf = build_box_frame(roi, None, {'camera_frame': 'head_camera_color_optical_frame', 'base_frame': 'chassis_link'}, {'fx': 500, 'fy': 500, 'cx': 300, 'cy': 250}, np.eye(4))

        mask = np.zeros((480, 640), dtype=bool)
        mask[150:250, 280:320] = True
        item = {
            'instance_id': 1,
            'upstream_instance_id': 1,
            'score': 0.95,
            'depth_inward_mm': 100.0,
            'lateral_coord_mm': 0.0,
            'valid_depth': True,
            'usable_for_selection': True,
            'valid_depth_points': 100,
            'bbox': [280, 150, 40, 100],
            '_mask': mask,
        }
        sku_geo = {'depth_axis_size_mm': 80.0, 'lateral_axis_size_mm': 40.0}
        res = sku_selection.select_front_row_targets([item], bf, None, sku_geo, roi, (480, 640))
        self.assertIsNotNone(res['center_item'])
        self.assertEqual(res['center_item']['instance_id'], 1)
        self.assertIsNone(res['left_item'])
        self.assertIsNone(res['right_item'])


if __name__ == '__main__':
    unittest.main()
