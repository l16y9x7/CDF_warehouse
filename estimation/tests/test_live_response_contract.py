#!/usr/bin/env python3
"""Repeatable live HTTP regression test for localization response contract.

By default, skips execution in local unit test suite to avoid network/A800 dependency.
To enable:
    set RUN_LIVE_A800_TESTS=1
    set A800_INFER_URL=http://211.137.21.33:25540/infer
"""
import json
import os
from pathlib import Path
import sys
import unittest
import urllib.request

DEPLOY_DIR = Path(__file__).resolve().parents[1] / 'deploy'
if str(DEPLOY_DIR) not in sys.path:
    sys.path.insert(0, str(DEPLOY_DIR))
for _sub in ('core', 'perception', 'pipeline', 'algorithms'):
    _p = str(DEPLOY_DIR / _sub)
    if _p not in sys.path:
        sys.path.insert(0, _p)


class TestLiveResponseContract(unittest.TestCase):
    def setUp(self):
        if os.environ.get('RUN_LIVE_A800_TESTS') != '1':
            self.skipTest("Live A800 HTTP tests disabled by default. Set RUN_LIVE_A800_TESTS=1 to enable.")

        self.url = os.environ.get('A800_INFER_URL', 'http://127.0.0.1:25540/infer')

    def _post(self, req_path: Path, force_return_viz: bool = False):
        with open(req_path, 'r', encoding='utf-8') as f:
            req_data = json.load(f)

        req_data['return_visualizations'] = force_return_viz
        body = json.dumps(req_data).encode('utf-8')
        http_req = urllib.request.Request(
            self.url,
            data=body,
            headers={'Content-Type': 'application/json'},
            method='POST'
        )

        with urllib.request.urlopen(http_req, timeout=120) as resp:
            resp_body = resp.read()
            status = resp.status
            content_type = resp.headers.get('Content-Type')

        self.assertEqual(status, 200)
        self.assertIn('charset=utf-8', content_type.lower())
        if not force_return_viz:
            self.assertLess(len(resp_body), 50 * 1024, "Response size should be tens of KB")
        else:
            self.assertLess(len(resp_body), 5 * 1024 * 1024, "Response size with viz must be < 5MB")

        resp_json = json.loads(resp_body.decode('utf-8'))

        # Forbidden internal arrays/keys must never be present
        for key in ['center_fit', 'left_fit', 'right_fit', 'analyses', 'fm', 'raw', 'fit', 'ins', 'camera_points', '_mask']:
            self.assertNotIn(key, resp_json)

        # Artifacts base64 check
        if not force_return_viz:
            for k in resp_json.get('artifacts', {}):
                self.assertFalse(k.endswith('_base64'), f"Forbidden base64 payload {k} when return_visualizations is False")

        return resp_json

    def test_live_tube_contract_and_geometry_success(self):
        """Validates that a high-quality tube request passes BOTH HTTP contract AND geometric localization."""
        req_path = os.environ.get('A800_TUBE_SUCCESS_REQUEST')
        if not req_path:
            candidate = Path('/home/quinn/cosmetics_pose/requests/sku/20260923_182816_5d23627e/request.json')
            if candidate.is_file():
                req_path = str(candidate)

        if not req_path or not Path(req_path).is_file():
            self.skipTest("No valid tube success request found or A800_TUBE_SUCCESS_REQUEST not set.")

        resp = self._post(Path(req_path), force_return_viz=False)

        # HTTP Contract & Formal fields
        self.assertEqual(resp.get('sku_typ'), 'tube')
        self.assertIn('edge_valid', resp)
        self.assertIn('point_valid', resp)

        # Geometric Localization Quality Gates (Success Path)
        self.assertTrue(resp.get('ok'), "Tube localization must succeed for this high-quality sample")
        self.assertTrue(resp.get('edge_valid'), "Edge must be valid")
        self.assertTrue(resp.get('point_valid'), "Point must be valid")
        self.assertEqual(resp.get('point_semantics'), 'visible_top_edge_midpoint')

        pt = resp.get('top_edge_center_camera_mm')
        self.assertIsInstance(pt, list)
        self.assertEqual(len(pt), 3)
        for coord in pt:
            self.assertIsInstance(coord, (int, float))

    def test_live_tube_contract_on_rejection(self):
        """Validates that a low-quality tube request STRICTLY ENFORCES response contract even when geometry fails."""
        req_path = os.environ.get('A800_TUBE_REJECT_REQUEST')
        if not req_path:
            candidate = Path('/home/quinn/cosmetics_pose/requests/sku/20260924_120056_9d9ff346/request.json')
            if candidate.is_file():
                req_path = str(candidate)

        if not req_path or not Path(req_path).is_file():
            self.skipTest("No valid tube rejection request found or A800_TUBE_REJECT_REQUEST not set.")

        resp = self._post(Path(req_path), force_return_viz=False)

        # HTTP Contract strictly enforced
        self.assertEqual(resp.get('sku_typ'), 'tube')
        self.assertIn('edge_valid', resp)
        self.assertIn('point_valid', resp)

        # Geometric Quality Gate (Rejection Path)
        self.assertFalse(resp.get('edge_valid'))
        self.assertFalse(resp.get('point_valid'))
        self.assertIsNone(resp.get('top_edge_center_camera_mm'))
        self.assertIsNone(resp.get('top_edge_endpoints_camera_mm'))


if __name__ == '__main__':
    unittest.main()
