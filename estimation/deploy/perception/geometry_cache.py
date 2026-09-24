#!/usr/bin/env python3
"""Per-request 3D camera and chassis coordinate point map cache.

Ensures that backprojection and rigid transformation between camera and chassis frames
are computed lazily once per request frame, and cached across mask proposals.
All cached point coordinates are in millimetres (mm).
"""
from typing import Any, Dict, Optional, Tuple
import numpy as np


class GeometryCache:
    """Per-request camera-point/chassis-point cache; all cached points remain mm."""

    def __init__(self):
        self._mask: Dict[Tuple[Tuple[int, ...], bytes], Tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]] = {}
        self._maps: Optional[Tuple[np.ndarray, np.ndarray, np.ndarray]] = None
        self.stats: Dict[str, int] = {
            'mask_hits': 0,
            'mask_misses': 0,
            'pixel_hits': 0,
            'pixel_misses': 0,
        }

    def _ensure_maps(
        self,
        depth: np.ndarray,
        K: Dict[str, float],
        T_m: np.ndarray
    ) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
        if self._maps is not None:
            return self._maps
        h, w = depth.shape
        yy, xx = np.indices((h, w), dtype=float)
        z = np.asarray(depth, dtype=float)
        valid = np.isfinite(z) & (z > 0)
        cam = np.empty((h, w, 3), dtype=float)
        cam[..., 0] = (xx - K['cx']) * z / K['fx']
        cam[..., 1] = (yy - K['cy']) * z / K['fy']
        cam[..., 2] = z
        cam[~valid] = np.nan
        ch = (cam.reshape(-1, 3) @ T_m[:3, :3].T).reshape(h, w, 3) + T_m[:3, 3] * 1000.0
        self._maps = (cam, ch, valid)
        return self._maps

    @staticmethod
    def _mask_key(mask: np.ndarray) -> Tuple[Tuple[int, ...], bytes]:
        a = np.asarray(mask, dtype=bool, order='C')
        return (a.shape, a.tobytes())

    def get_mask(
        self,
        mask: np.ndarray,
        depth: np.ndarray,
        K: Dict[str, float],
        T_m: np.ndarray
    ) -> Tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
        """Returns (camera_points_mm, pixels_xy, valid_mask, chassis_points_mm)."""
        key = self._mask_key(mask)
        if key in self._mask:
            self.stats['mask_hits'] += 1
            return self._mask[key]

        cam_map, ch_map, depth_valid = self._ensure_maps(depth, K, T_m)
        valid = mask & depth_valid
        y, x = np.where(valid)
        pixels = np.column_stack((x, y))
        pc = cam_map[valid].copy()
        ch = ch_map[valid].copy()
        out = (pc, pixels, valid, ch)
        self._mask[key] = out
        self.stats['mask_misses'] += 1
        return out
