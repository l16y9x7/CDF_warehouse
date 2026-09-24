#!/usr/bin/env python3
"""Unit tests for request_codec.py (Items 1 to 6)."""
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

from request_codec import parse_K, parse_T_m, decode_npy_b64, common_input


class TestRequestCodec(unittest.TestCase):
    def test_item1_parse_K_dict_and_matrix(self):
        # 1. camera_K supports both 3x3 list/matrix and fx/fy/cx/cy dict
        d = {'fx': 600.0, 'fy': 610.0, 'cx': 320.0, 'cy': 240.0}
        k1 = parse_K(d)
        self.assertEqual(k1, d)

        m = [
            [600.0, 0.0, 320.0],
            [0.0, 610.0, 240.0],
            [0.0, 0.0, 1.0],
        ]
        k2 = parse_K(m)
        self.assertEqual(k2, d)

        # Invalid formats raise ValueError
        with self.assertRaises(ValueError):
            parse_K([1, 2, 3])

    def test_item2_T_unit_m_and_mm(self):
        # 2. T_chassis_camera supports translation_unit m and mm
        T = np.eye(4)
        T[0, 3] = 0.5  # 0.5 m = 500 mm
        T_m = parse_T_m({
            'T_chassis_camera': T.tolist(),
            'T_unit': 'm',
            'camera_frame': 'head_camera_color_optical_frame',
            'base_frame': 'chassis_link',
        })
        self.assertAlmostEqual(T_m[0, 3], 0.5)

        # In mm unit
        T_mm = np.eye(4)
        T_mm[0, 3] = 500.0  # 500 mm -> 0.5 m
        T_m2 = parse_T_m({
            'T_chassis_camera': T_mm.tolist(),
            'T_unit': 'mm',
            'camera_frame': 'head_camera_color_optical_frame',
            'base_frame': 'chassis_link',
        })
        self.assertAlmostEqual(T_m2[0, 3], 0.5)

    def test_item3_invalid_rotation_matrix(self):
        # 3. parse_T_m validates rotation matrix orthogonality
        bad_T = np.eye(4)
        bad_T[0, 0] = 2.0  # not orthogonal
        with self.assertRaises(ValueError):
            parse_T_m({
                'T_chassis_camera': bad_T.tolist(),
                'T_unit': 'm',
                'camera_frame': 'head_camera_color_optical_frame',
                'base_frame': 'chassis_link',
            })

    def test_item4_invalid_frame_names(self):
        # 4. Wrong frame names raise ValueError
        with self.assertRaises(ValueError):
            parse_T_m({
                'T_chassis_camera': np.eye(4).tolist(),
                'T_unit': 'm',
                'camera_frame': 'wrong_camera',
                'base_frame': 'wrong_base',
            })

    def test_item5_depth_shape_mismatch(self):
        # 5. common_input validates depth shape matching RGB shape
        import cv2
        import base64
        import io

        rgb = np.zeros((480, 640, 3), dtype=np.uint8)
        _, enc_rgb = cv2.imencode('.jpg', rgb)
        rgb_b64 = base64.b64encode(enc_rgb).decode('ascii')

        depth = np.zeros((300, 300), dtype=np.float32)
        buf = io.BytesIO()
        np.save(buf, depth)
        depth_b64 = base64.b64encode(buf.getvalue()).decode('ascii')

        req = {
            'rgb_base64': rgb_b64,
            'depth_npy_base64': depth_b64,
            'depth_unit': 'mm',
            'K': {'fx': 500, 'fy': 500, 'cx': 320, 'cy': 240},
            'T_chassis_camera': np.eye(4).tolist(),
            'T_unit': 'm',
            'camera_frame': 'head_camera_color_optical_frame',
            'base_frame': 'chassis_link',
        }
        with self.assertRaises(ValueError):
            common_input(req)

    def test_item6_depth_invalid_values(self):
        # 6. decode_npy_b64 rejects 1D array
        import base64
        import io

        bad_1d = np.array([1.0, 2.0, 3.0], dtype=np.float32)
        buf = io.BytesIO()
        np.save(buf, bad_1d)
        depth_b64 = base64.b64encode(buf.getvalue()).decode('ascii')

        with self.assertRaises(ValueError):
            decode_npy_b64(depth_b64)


if __name__ == '__main__':
    unittest.main()
