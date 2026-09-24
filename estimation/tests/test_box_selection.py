#!/usr/bin/env python3
"""Unit tests for box_selection.py (Items 7, 8, 9)."""
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

import box_selection
from sam3_client import decode_detection_mask, encode_mask


class TestBoxSelection(unittest.TestCase):
    def test_item7_box_selection_adaptive(self):
        # 1. No valid candidates raises no_cardboard_box_detected
        tiny_candidate = [{'upstream_instance_id': 1, 'bbox': [10, 10, 100, 100], 'score': 0.9}]
        with self.assertRaises(ValueError) as ctx:
            box_selection.choose_pair(tiny_candidate, (480, 640))
        self.assertIn('no_cardboard_box_detected', str(ctx.exception))

        # 2. Adaptive single box: exactly one candidate meeting size requirements
        c1 = {'upstream_instance_id': 1, 'bbox': [50, 100, 200, 200], 'score': 0.95}
        indices, rois = box_selection.choose_pair([c1], (480, 640))
        self.assertEqual(len(indices), 1)
        self.assertEqual(len(rois), 1)
        self.assertEqual(indices[0], 0)
        self.assertEqual(rois[0], [50, 100, 250, 300])

        # 3. Two candidates meeting size and adjacency requirements: dual box pair
        c2 = {'upstream_instance_id': 2, 'bbox': [260, 100, 200, 200], 'score': 0.96}
        indices, rois = box_selection.choose_pair([c1, c2], (480, 640))
        self.assertEqual(len(indices), 2)
        self.assertEqual(len(rois), 2)
        self.assertLess(rois[0][0], rois[1][0])

    def test_item8_full_image_filter_roi_inside_ratio(self):
        # 8. filter_instances_detailed audits inside_ratio
        H, W = 480, 640
        roi = [100.0, 100.0, 300.0, 300.0]

        # Mask completely inside ROI
        mask_in = np.zeros((H, W), dtype=bool)
        mask_in[150:200, 150:200] = True
        rle_in = encode_mask(mask_in)
        det_in = {'upstream_instance_id': 1, 'segmentation': rle_in, 'score': 0.9}

        # Mask completely outside ROI
        mask_out = np.zeros((H, W), dtype=bool)
        mask_out[10:50, 10:50] = True
        rle_out = encode_mask(mask_out)
        det_out = {'upstream_instance_id': 2, 'segmentation': rle_out, 'score': 0.9}

        kept, rejected, membership = box_selection.filter_instances_detailed(
            [det_in, det_out], roi, (H, W), decode_detection_mask, min_inside=0.8
        )
        self.assertEqual(len(kept), 1)
        self.assertEqual(kept[0]['upstream_instance_id'], 1)
        self.assertEqual(len(rejected), 1)
        self.assertEqual(rejected[0]['upstream_instance_id'], 2)


if __name__ == '__main__':
    unittest.main()
