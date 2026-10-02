"""Debug proxy contract: exact parameters, mask geometry, and local path bounds."""
import base64
from pathlib import Path
import sys
import tempfile
import unittest
from unittest.mock import patch, MagicMock

import cv2
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'tools'))
import sam3_debug_server as debug


class TestSam3DebugServer(unittest.TestCase):
    def payload(self):
        self.image = np.full((12, 20, 3), (20, 70, 150), np.uint8)
        ok, png = cv2.imencode('.png', self.image)
        self.assertTrue(ok)
        return {'image_base64': base64.b64encode(png).decode(), 'prompt': ' soap packages ',
                'threshold': 0.31, 'mask_threshold': 0.62}

    def test_forwards_exact_thresholds_and_returns_unfiltered_instances(self):
        payload = self.payload()
        mask = np.zeros((12, 20), np.uint8)
        mask[2:8, 3:13] = 255
        mask_b64 = base64.b64encode(cv2.imencode('.png', mask)[1]).decode()
        response = MagicMock(status_code=200)
        response.json.return_value = {'instances': [
            {'instance_id': 17, 'bbox_xyxy': [3, 2, 13, 8], 'score': 0.75, 'mask_png_base64': mask_b64}]}
        with patch.object(debug.requests, 'Session') as factory:
            session = factory.return_value.__enter__.return_value
            session.post.return_value = response
            result = debug.segment(payload, debug.DEFAULT_UPSTREAM)
            self.assertFalse(session.trust_env)
            args, kwargs = session.post.call_args
            self.assertEqual(args[0], debug.DEFAULT_UPSTREAM)
            self.assertEqual(kwargs['data'], {'prompt': 'soap packages', 'threshold': '0.31', 'mask_threshold': '0.62'})
            encoded_image = kwargs['files']['image'][1]
            np.testing.assert_array_equal(cv2.imdecode(np.frombuffer(encoded_image, np.uint8), cv2.IMREAD_COLOR), self.image)
        self.assertEqual(result['instance_count'], 1)
        self.assertEqual(result['instances'][0]['id'], 1)
        self.assertEqual(result['instances'][0]['upstream_instance_id'], 17)
        self.assertEqual(result['instances'][0]['area_pixels'], 60)
        self.assertEqual(result['instances'][0]['area_ratio'], 0.25)

    def test_invalid_inputs_never_call_upstream(self):
        good = self.payload()
        with patch.object(debug.requests, 'Session') as factory:
            for changes in ({'prompt': ''}, {'threshold': True}, {'threshold': float('nan')},
                            {'mask_threshold': 1.1}, {'image_base64': 'not an image'}):
                with self.subTest(changes=changes), self.assertRaises(debug.ApiError):
                    debug.segment({**good, **changes}, debug.DEFAULT_UPSTREAM)
            factory.assert_not_called()

    def test_rejects_mask_size_mismatch(self):
        payload = self.payload()
        mask_b64 = base64.b64encode(cv2.imencode('.png', np.ones((2, 3), np.uint8))[1]).decode()
        response = MagicMock(status_code=200)
        response.json.return_value = {'instances': [{'score': 0.8, 'bbox_xyxy': [0, 0, 3, 2], 'mask_png_base64': mask_b64}]}
        with patch.object(debug.requests, 'Session') as factory:
            factory.return_value.__enter__.return_value.post.return_value = response
            with self.assertRaises(debug.ApiError) as caught:
                debug.segment(payload, debug.DEFAULT_UPSTREAM)
            self.assertEqual(caught.exception.status, 502)

    def test_log_keys_cannot_escape_sample_root(self):
        with tempfile.TemporaryDirectory() as tmp, patch.object(debug, 'SAMPLES', Path(tmp)):
            sample = Path(tmp) / 'sku/20261002_test'
            sample.mkdir(parents=True)
            self.assertEqual(debug.sample_dir('sku/20261002_test'), sample.resolve())
            for key in ('../secret', 'sku/../../secret', 'sku/C:/secret', 'basket\\secret', '/sku/20261002_test'):
                with self.subTest(key=key), self.assertRaises(debug.ApiError):
                    debug.sample_dir(key)


if __name__ == '__main__':
    unittest.main()
