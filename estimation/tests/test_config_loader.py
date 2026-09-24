#!/usr/bin/env python3
"""Automated tests for config.json and config_loader.py.

Verifies:
1. config.json static defaults loading and structure (Requirement 13);
2. Environment variable overrides (Requirement 14);
3. Default URL selection for legacy_18003 vs multipart_segment backends (Requirement 15);
4. CAD path resolution ensuring Basket CAD is within deploy/cad (Requirement 16);
5. Parameter validation errors on invalid thresholds or backends.
"""
import json
from pathlib import Path
import sys
import unittest

DEPLOY_DIR = Path(__file__).resolve().parents[1] / 'deploy'
if str(DEPLOY_DIR) not in sys.path:
    sys.path.insert(0, str(DEPLOY_DIR))
for _sub in ('core', 'perception', 'pipeline', 'algorithms'):
    _p = str(DEPLOY_DIR / _sub)
    if _p not in sys.path:
        sys.path.insert(0, _p)

import config_loader


class TestConfigLoader(unittest.TestCase):
    def setUp(self):
        self.json_path = DEPLOY_DIR / 'config.json'

    # Requirement 13: config.json 加载测试
    def test_item13_config_json_exists_and_loads(self):
        self.assertTrue(self.json_path.is_file(), f"Missing config.json at {self.json_path}")
        raw = config_loader.load_raw_json(self.json_path)
        self.assertIsInstance(raw, dict)
        self.assertIn('service', raw)
        self.assertIn('sam3', raw)
        self.assertIn('basket', raw)
        self.assertIn('prompts', raw)
        self.assertIn('classes', raw)
        self.assertIn('front_rule_default', raw)

        # Check default classes
        self.assertIn('bottle', raw['classes'])
        self.assertIn('box', raw['classes'])
        self.assertIn('tube', raw['classes'])

    # Requirement 14: 环境变量覆盖测试
    def test_item14_environment_variable_overrides(self):
        raw = config_loader.load_raw_json(self.json_path)
        test_env = {
            'HOST': '127.0.0.1',
            'PORT': '29999',
            'SAM3_BACKEND': 'legacy_18003',
            'SAM3_URL': 'http://custom.upstream:18003/infer',
            'SAM3_MASK_THRESHOLD': '0.65',
            'SAM3_TIMEOUT_S': '90.0',
            'SAVE_REQUEST_INPUTS': '0',
            'DEFAULT_BOX_PROMPT': 'custom box prompt',
            'ENABLE_3D_RENDER': '0',
            'ASYNC_3D_RENDER': '0',
        }
        cfg = config_loader.build_config(raw_cfg=raw, env=test_env)
        self.assertEqual(cfg['DEFAULT_HOST'], '127.0.0.1')
        self.assertEqual(cfg['DEFAULT_PORT'], 29999)
        self.assertEqual(cfg['SAM3_URL'], 'http://custom.upstream:18003/infer')
        self.assertEqual(cfg['SAM3_MASK_THRESHOLD'], 0.65)
        self.assertEqual(cfg['SAM3_TIMEOUT_S'], 90.0)
        self.assertFalse(cfg['SAVE_REQUEST_INPUTS'])
        self.assertEqual(cfg['CONTAINER_BOX_PROMPT'], 'custom box prompt')
        self.assertEqual(cfg['CLASS_CONFIG']['bottle']['box_prompt'], 'custom box prompt')
        self.assertFalse(cfg['ENABLE_3D_RENDER'])
        self.assertFalse(cfg['ASYNC_3D_RENDER'])

    def test_invalid_env_raises_validation_error(self):
        raw = config_loader.load_raw_json(self.json_path)
        with self.assertRaises(ValueError):
            config_loader.build_config(raw_cfg=raw, env={'SAM3_BACKEND': 'invalid_backend'})

        with self.assertRaises(ValueError):
            config_loader.build_config(raw_cfg=raw, env={'SAM3_MASK_THRESHOLD': '1.5'})

        with self.assertRaises(ValueError):
            config_loader.build_config(raw_cfg=raw, env={'SAM3_MASK_THRESHOLD': '-0.1'})

    # Requirement 15: 测试 legacy_18003 和 multipart_segment 两种 backend 的默认 URL
    def test_item15_sam3_backend_default_urls(self):
        raw = config_loader.load_raw_json(self.json_path)

        # 1. legacy_18003 default URL
        cfg_legacy = config_loader.build_config(raw_cfg=raw, env={'SAM3_BACKEND': 'legacy_18003'})
        self.assertEqual(cfg_legacy['SAM3_BACKEND'], 'legacy_18003')
        self.assertEqual(cfg_legacy['SAM3_URL'], 'http://127.0.0.1:25551/infer')

        # 2. multipart_segment default URL
        cfg_multi = config_loader.build_config(raw_cfg=raw, env={'SAM3_BACKEND': 'multipart_segment'})
        self.assertEqual(cfg_multi['SAM3_BACKEND'], 'multipart_segment')
        self.assertEqual(cfg_multi['SAM3_URL'], 'http://127.0.0.1:25541/api/v1/segment')

        # 3. Explicit SAM3_URL overrides default for both
        cfg_override = config_loader.build_config(
            raw_cfg=raw,
            env={'SAM3_BACKEND': 'multipart_segment', 'SAM3_URL': 'http://10.0.0.1:8000/segment'}
        )
        self.assertEqual(cfg_override['SAM3_URL'], 'http://10.0.0.1:8000/segment')

    # Requirement 16: 测试 CAD 路径解析结果位于正式 deploy/cad 目录内
    def test_item16_cad_path_within_deploy_cad(self):
        raw = config_loader.load_raw_json(self.json_path)
        cfg = config_loader.build_config(raw_cfg=raw, env={})
        basket_cad = Path(cfg['BASKET_MESH_PATH']).resolve()
        cad_dir = (DEPLOY_DIR / 'cad').resolve()

        # Check path containment
        self.assertTrue(
            cad_dir in basket_cad.parents or basket_cad.parent == cad_dir,
            f"CAD path {basket_cad} is not inside {cad_dir}"
        )
        # Check actual CAD file exists on disk
        self.assertTrue(basket_cad.is_file(), f"Basket CAD file does not exist: {basket_cad}")
        self.assertTrue(str(basket_cad).endswith('.obj'))


if __name__ == '__main__':
    unittest.main()
