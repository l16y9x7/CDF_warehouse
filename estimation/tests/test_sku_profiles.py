"""Regression coverage for optional business IDs and container area rejection."""
import copy
import json
from pathlib import Path
import sys
import tempfile
import unittest

import numpy as np

DEPLOY_DIR = Path(__file__).resolve().parents[1] / 'deploy'
for sub in ('core', 'perception', 'pipeline', 'algorithms'):
    sys.path.insert(0, str(DEPLOY_DIR / sub))

import box_selection
import config_loader
from request_codec import normalize_request_target
from sku_profiles import apply_sku_profile, load_sku_profiles


class TestSkuProfiles(unittest.TestCase):
    def test_old_agent_without_id_retains_category_and_request_prompt(self):
        for present in (False, True):
            req = {'sku_typ': 'tube', 'side': 'RIGHT', 'sam3_prompt': 'diagnostic prompt'}
            if present:
                req['sku_id'] = None
            self.assertEqual(normalize_request_target(req), 'tube')
            self.assertEqual(req['sam3_prompt'], 'diagnostic prompt')
            self.assertEqual(req['_sku_profile']['source'], 'category_default')

    def test_id_resolves_type_and_server_parameters_preserving_side(self):
        req = {'sku_id': 'demo_cream_box', 'side': 'LEFT', 'sam3_prompt': 'stale client prompt',
               'sam3_threshold': 0.01, 'box_selection': {'target_threshold': 0.01, 'max_mask_area_ratio': 1.0}}
        self.assertEqual(normalize_request_target(req), 'box')
        profile = load_sku_profiles()['profiles']['demo_cream_box']
        self.assertEqual(req['sam3_prompt'], profile['sam3_prompt'])
        parsed = box_selection.parse_box_selection(req, config_loader.CLASS_CONFIG['box'])
        self.assertEqual(parsed['target_threshold'], profile['target_threshold'])
        self.assertEqual(parsed['max_mask_area_ratio'], profile['max_mask_area_ratio'])
        self.assertEqual(parsed['target_box'], 1)
        self.assertEqual(req['_sku_profile']['sku_id'], 'demo_cream_box')
        self.assertEqual(req['_sku_profile']['library_revision'], load_sku_profiles()['revision'])

    def test_unknown_or_conflicting_id_is_not_silently_treated_as_generic(self):
        for req, reason in (
            ({'sku_id': 'not_configured', 'sku_typ': 'box', 'side': 'LEFT'}, 'unknown sku_id'),
            ({'sku_id': 'demo_white_tube', 'sku_typ': 'box', 'side': 'LEFT'}, 'conflicts'),
            ({'sku_id': '', 'sku_typ': 'box', 'side': 'LEFT'}, 'non-empty string'),
            ({'sku_id': 123, 'sku_typ': 'box', 'side': 'LEFT'}, 'non-empty string'),
            ({'sku_typ': 'box', 'class_name': 'box', 'side': 'LEFT'}, 'class_name was removed'),
            ({'target_type': 'basket', 'sku_id': 'demo_white_tube'}, 'must not include'),
        ):
            with self.subTest(req=req), self.assertRaisesRegex(ValueError, reason):
                normalize_request_target(req)

    def test_library_updates_apply_to_next_request_and_are_isolated(self):
        library = load_sku_profiles()
        with tempfile.TemporaryDirectory() as td:
            path = Path(td) / 'profiles.json'
            path.write_text(json.dumps(library), encoding='utf-8')
            first = {'sku_id': 'demo_cream_box'}
            apply_sku_profile(first, path)
            library['revision'] = 'test-update'
            library['profiles']['demo_cream_box']['sam3_prompt'] = 'updated top face prompt'
            library['profiles']['demo_cream_box']['max_mask_area_ratio'] = 0.25
            path.write_text(json.dumps(library), encoding='utf-8')
            second = {'sku_id': 'demo_cream_box'}
            apply_sku_profile(second, path)
            self.assertEqual(second['sam3_prompt'], 'updated top face prompt')
            self.assertEqual(second['box_selection']['max_mask_area_ratio'], 0.25)
            self.assertNotEqual(first['sam3_prompt'], second['sam3_prompt'])
            self.assertEqual(first['box_selection']['max_mask_area_ratio'], 0.5)

    def test_library_rejects_bad_thresholds_and_duplicate_ids(self):
        library = load_sku_profiles()
        with tempfile.TemporaryDirectory() as td:
            path = Path(td) / 'profiles.json'
            for value in (0, 1.1, True, float('nan')):
                bad = copy.deepcopy(library)
                bad['profiles']['demo_cream_box']['max_mask_area_ratio'] = value
                path.write_text(json.dumps(bad), encoding='utf-8')
                with self.assertRaises(ValueError):
                    load_sku_profiles(path)
            path.write_text('{"schema_version":1,"revision":"a","profiles":{"x":{},"x":{}}}')
            with self.assertRaisesRegex(ValueError, 'duplicate'):
                load_sku_profiles(path)

    def test_legacy_defaults_are_color_independent_and_apply_area_limit(self):
        for sku_typ in ('box', 'tube', 'bottle'):
            cfg = config_loader.CLASS_CONFIG[sku_typ]
            self.assertNotIn('green', cfg['sam3_prompt'])
            self.assertNotIn('white', cfg['sam3_prompt'])
            self.assertEqual(box_selection.parse_box_selection({}, cfg)['max_mask_area_ratio'], 0.5)
        for value in (0, -1, 1.1, float('nan')):
            with self.assertRaises(ValueError):
                box_selection.parse_box_selection({'box_selection': {'max_mask_area_ratio': value}})


class TestContainerAreaFilter(unittest.TestCase):
    def test_large_high_score_container_removed_small_product_keeps_original_id(self):
        large = np.zeros((120, 120), dtype=bool)
        large[10:100, 10:100] = True
        product = np.zeros_like(large)
        product[30:45, 30:45] = True
        detections = [{'upstream_instance_id': 4, 'score': 0.895, 'segmentation': large},
                      {'upstream_instance_id': 2, 'score': 0.55, 'segmentation': product}]
        kept, rejected, audit = box_selection.filter_instances_detailed(
            detections, [10, 10, 110, 110], large.shape, lambda m: m, max_mask_area_ratio=0.5)
        self.assertEqual([d['upstream_instance_id'] for d in kept], [2])
        self.assertEqual(kept[0]['filtered_instance_id'], 1)
        self.assertEqual(rejected[0]['reason'], 'mask_area_ratio_above_threshold')
        self.assertEqual(audit[0]['mask_area_pixels'], 8100)
        self.assertEqual(audit[0]['box_roi_area_pixels'], 10000)
        self.assertEqual(audit[0]['mask_area_ratio'], 0.81)

    def test_limit_boundary_and_larger_sku_override(self):
        mask = np.zeros((100, 100), dtype=bool)
        mask[:50, :] = True
        detections = [{'score': 0.9, 'segmentation': mask}]
        def kept_at(limit):
            return box_selection.filter_instances_detailed(
                detections, [0, 0, 100, 100], mask.shape, lambda m: m, max_mask_area_ratio=limit)[0]
        self.assertEqual(len(kept_at(0.5)), 1)
        self.assertEqual(len(kept_at(0.49)), 0)
        self.assertEqual(len(kept_at(0.8)), 1)


if __name__ == '__main__':
    unittest.main()
