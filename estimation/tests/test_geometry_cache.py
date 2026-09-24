#!/usr/bin/env python3
"""Unit tests for geometry_cache.py."""
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

from geometry_cache import GeometryCache


class TestGeometryCache(unittest.TestCase):
    def test_geometry_cache_lifecycle_and_stats(self):
        cache = GeometryCache()
        self.assertEqual(cache.stats['mask_hits'], 0)
        self.assertEqual(cache.stats['mask_misses'], 0)

        H, W = 100, 100
        depth = np.full((H, W), 500.0, dtype=np.float32)
        K = {'fx': 500.0, 'fy': 500.0, 'cx': 50.0, 'cy': 50.0}
        T_m = np.eye(4)

        mask = np.zeros((H, W), dtype=bool)
        mask[10:20, 10:20] = True

        pts, pixels, valid, ch_pts = cache.get_mask(mask, depth, K, T_m)
        self.assertEqual(len(pts), 100)
        self.assertEqual(len(pixels), 100)
        self.assertEqual(cache.stats['mask_misses'], 1)

        # Second query should be a hit
        pts2, pixels2, _, _ = cache.get_mask(mask, depth, K, T_m)
        self.assertEqual(cache.stats['mask_hits'], 1)
        np.testing.assert_array_equal(pts, pts2)


if __name__ == '__main__':
    unittest.main()
