#!/usr/bin/env python3
"""End-to-end pipeline execution and regression tests using real 20260915 dataset."""
import copy
import json
from pathlib import Path
import sys
import tempfile
import unittest
from unittest.mock import patch

import cv2
import numpy as np

DEPLOY_DIR = Path(__file__).resolve().parents[1] / 'deploy'
ROOT_DIR = Path(__file__).resolve().parents[1]
if str(DEPLOY_DIR) not in sys.path:
    sys.path.insert(0, str(DEPLOY_DIR))
for _sub in ('core', 'perception', 'pipeline', 'algorithms'):
    _p = str(DEPLOY_DIR / _sub)
    if _p not in sys.path:
        sys.path.insert(0, _p)

import server
import target_pipeline


class TestPipelineExecution(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.data_dir = ROOT_DIR.parent / 'data' / '20260915'
        cls.tf_path = (
            ROOT_DIR.parent
            / 'sam3'
            / 'test'
            / 'axis_fit_support'
            / 'head_camera_transform_20260915'
            / 'transforms_20260915.json'
        )
        cls.transforms = {}
        if cls.tf_path.is_file():
            with open(cls.tf_path, 'r', encoding='utf-8') as f:
                tf_data = json.load(f)
                cls.transforms = {r['sample']: np.array(r['matrix_4x4'], dtype=float) for r in tf_data.get('rows', [])}

    def _load_real_sample(self, sample_id):
        sample_path = self.data_dir / sample_id
        if not sample_path.is_dir():
            self.skipTest(f"Sample data {sample_path} not available locally")
        rgb = cv2.imread(str(sample_path / 'head_rgb.jpg'))
        depth = np.load(str(sample_path / 'head_depth_aligned.npy'))
        cam_file = sample_path / 'camera.json'
        if not cam_file.is_file():
            cam_file = self.data_dir / '120045958' / 'camera.json'
        with open(cam_file, 'r', encoding='utf-8') as f:
            cam = json.load(f)
        K = {
            'fx': cam['cam_K'][0],
            'fy': cam['cam_K'][4],
            'cx': cam['cam_K'][2],
            'cy': cam['cam_K'][5],
        }
        T_m = self.transforms.get(sample_id, np.eye(4))
        return rgb, depth, K, T_m

    def test_full_pipeline_bottle_real_scene(self):
        rgb, depth, K, T_m = self._load_real_sample('120045958')
        H, W = rgb.shape[:2]

        box_mask_left = np.zeros((H, W), dtype=bool)
        box_mask_left[180:600, 30:600] = True
        box_mask_right = np.zeros((H, W), dtype=bool)
        box_mask_right[180:600, 650:1250] = True

        sku_mask = np.zeros((H, W), dtype=bool)
        sku_mask[300:500, 850:1050] = True

        box_sam_resp = {
            'detections': [
                {'upstream_instance_id': 1, 'score': 0.95, 'bbox': [30, 180, 570, 420], '_mask': box_mask_left},
                {'upstream_instance_id': 2, 'score': 0.96, 'bbox': [650, 180, 600, 420], '_mask': box_mask_right},
            ],
            'num_detections': 2,
        }
        target_sam_resp = {
            'detections': [
                {'upstream_instance_id': 1, 'score': 0.93, 'bbox': [850, 300, 200, 200], '_mask': sku_mask},
            ],
            'num_detections': 1,
        }

        def mock_sam(image, prompt, threshold=0.5, endpoint=None):
            if 'box' in prompt.lower() or 'cardboard' in prompt.lower():
                return dict(box_sam_resp)
            return dict(target_sam_resp)

        req = {
            'sku_typ': 'bottle',
            'side': 'RIGHT',
            'camera_frame': 'head_camera_color_optical_frame',
            'base_frame': 'chassis_link',
            'camera_K': [K['fx'], 0.0, K['cx'], 0.0, K['fy'], K['cy'], 0.0, 0.0, 1.0],
            'transform_camera_to_base': T_m.tolist(),
            'transform_unit': 'm',
            'rgb_base64': '',
            'depth_raw_b64': '',
        }

        with tempfile.TemporaryDirectory() as td:
            root = Path(td) / 'test_bot'
            root.mkdir()

            with patch('target_pipeline.call_sam3', side_effect=mock_sam), \
                 patch('target_pipeline.common_input', return_value=(rgb, depth, K, T_m)):
                res = target_pipeline.execute_pipeline(copy.deepcopy(req), 'bot_run', root)

            self.assertIn('pipeline_steps', res)
            self.assertEqual(res['target_type'], 'sku')
            self.assertEqual(res['sku_typ'], 'bottle')
            self.assertTrue(res['box_selection']['valid'])

    def test_full_pipeline_box_pair_failure_branch(self):
        H, W = 480, 640
        rgb = np.full((H, W, 3), 120, dtype=np.uint8)
        depth = np.full((H, W), 500.0, dtype=np.float32)
        K = {'fx': 600.0, 'fy': 600.0, 'cx': 320.0, 'cy': 240.0}
        T_m = np.eye(4)

        box_sam_resp = {
            'detections': [
                {'upstream_instance_id': 1, 'score': 0.95, 'bbox': [30, 180, 570, 420]},
            ],
            'num_detections': 1,
        }

        def mock_sam(image, prompt, threshold=0.5, endpoint=None):
            return dict(box_sam_resp)

        req = {
            'sku_typ': 'box',
            'side': 'RIGHT',
            'camera_frame': 'head_camera_color_optical_frame',
            'base_frame': 'chassis_link',
        }

        with tempfile.TemporaryDirectory() as td:
            root = Path(td) / 'box_fail'
            root.mkdir()

            with patch('target_pipeline.call_sam3', side_effect=mock_sam), \
                 patch('target_pipeline.common_input', return_value=(rgb, depth, K, T_m)):
                res = target_pipeline.execute_pipeline(copy.deepcopy(req), 'box_fail_run', root)

            self.assertFalse(res['ok'])
            self.assertIn('box selection failed', res['rejection_reasons'][0])

    def test_full_pipeline_empty_target_in_box(self):
        rgb, depth, K, T_m = self._load_real_sample('120045958')
        H, W = rgb.shape[:2]

        box_mask_left = np.zeros((H, W), dtype=bool)
        box_mask_left[180:600, 30:600] = True
        box_mask_right = np.zeros((H, W), dtype=bool)
        box_mask_right[180:600, 650:1250] = True

        outside_mask = np.zeros((H, W), dtype=bool)
        outside_mask[300:500, 100:300] = True  # outside right box

        box_sam_resp = {
            'detections': [
                {'upstream_instance_id': 1, 'score': 0.95, 'bbox': [30, 180, 570, 420], '_mask': box_mask_left},
                {'upstream_instance_id': 2, 'score': 0.96, 'bbox': [650, 180, 600, 420], '_mask': box_mask_right},
            ],
            'num_detections': 2,
        }
        target_sam_resp = {
            'detections': [
                {'upstream_instance_id': 1, 'score': 0.93, 'bbox': [100, 300, 200, 200], '_mask': outside_mask},
            ],
            'num_detections': 1,
        }

        def mock_sam(image, prompt, threshold=0.5, endpoint=None):
            if 'box' in prompt.lower() or 'cardboard' in prompt.lower():
                return dict(box_sam_resp)
            return dict(target_sam_resp)

        req = {
            'sku_typ': 'bottle',
            'side': 'RIGHT',
            'camera_frame': 'head_camera_color_optical_frame',
            'base_frame': 'chassis_link',
            'camera_K': [K['fx'], 0.0, K['cx'], 0.0, K['fy'], K['cy'], 0.0, 0.0, 1.0],
            'transform_camera_to_base': T_m.tolist(),
            'transform_unit': 'm',
            'rgb_base64': '',
            'depth_raw_b64': '',
        }

        with tempfile.TemporaryDirectory() as td:
            root = Path(td) / 'empty_target'
            root.mkdir()

            with patch('target_pipeline.call_sam3', side_effect=mock_sam), \
                 patch('target_pipeline.common_input', return_value=(rgb, depth, K, T_m)):
                res = target_pipeline.execute_pipeline(copy.deepcopy(req), 'empty_target_run', root)

            self.assertFalse(res['ok'])
            self.assertIsNone(res['selected_instance_id'])
            self.assertIn('no target instance assigned to selected box', res['rejection_reasons'])

    def test_full_pipeline_single_box_adaptive_success(self):
        H, W = 480, 640
        rgb = np.full((H, W, 3), 120, dtype=np.uint8)
        depth = np.full((H, W), 500.0, dtype=np.float32)
        K = {'fx': 600.0, 'fy': 600.0, 'cx': 320.0, 'cy': 240.0}
        T_m = np.eye(4)

        box_mask = np.zeros((H, W), dtype=bool)
        box_mask[100:300, 50:250] = True

        box_sam_resp = {
            'detections': [
                {'upstream_instance_id': 1, 'score': 0.95, 'bbox': [50, 100, 200, 200], '_mask': box_mask},
            ],
            'num_detections': 1,
        }

        def mock_sam(image, prompt, threshold=0.5, endpoint=None):
            return dict(box_sam_resp)

        req = {
            'sku_typ': 'box',
            'side': 'RIGHT',
            'camera_frame': 'head_camera_color_optical_frame',
            'base_frame': 'chassis_link',
        }

        with tempfile.TemporaryDirectory() as td:
            root = Path(td) / 'single_box'
            root.mkdir()

            with patch('target_pipeline.call_sam3', side_effect=mock_sam), \
                 patch('target_pipeline.common_input', return_value=(rgb, depth, K, T_m)):
                res = target_pipeline.execute_pipeline(copy.deepcopy(req), 'single_box_run', root)

            self.assertTrue(res['box_selection']['valid'])
            self.assertIn('single_box_adaptive', res['box_selection']['target_box_semantics'])
            self.assertEqual(len(res['box_selection']['left_right_rois_xyxy']), 1)


if __name__ == '__main__':
    unittest.main()
