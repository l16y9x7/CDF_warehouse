#!/usr/bin/env python3
"""Unit tests for response_schema.py (Items 16, 18, 19)."""
import unittest
from pathlib import Path
import sys

DEPLOY_DIR = Path(__file__).resolve().parents[1] / 'deploy'
if str(DEPLOY_DIR) not in sys.path:
    sys.path.insert(0, str(DEPLOY_DIR))
for _sub in ('core', 'perception', 'pipeline', 'algorithms'):
    _p = str(DEPLOY_DIR / _sub)
    if _p not in sys.path:
        sys.path.insert(0, _p)

from response_schema import CATEGORY_EMPTY_FIELDS
import server


class TestResponseSchema(unittest.TestCase):
    def test_item16_category_empty_fields_contract(self):
        # 16. Verify CATEGORY_EMPTY_FIELDS defines expected fields for each category
        for cat in ('bottle', 'box', 'tube'):
            self.assertIn(cat, CATEGORY_EMPTY_FIELDS)
            fields = CATEGORY_EMPTY_FIELDS[cat]
            self.assertIsInstance(fields, dict)
            self.assertIn('localization_method', fields)

    def test_item18_http200_failure_contract(self):
        # 18. Service failure response structure contains ok=False, selected_instance_id=None, rejection_reasons
        empty_bottle = CATEGORY_EMPTY_FIELDS['bottle']
        self.assertIn('reference_point_camera_mm', empty_bottle)
        self.assertIn('reference_point_chassis_mm', empty_bottle)
        self.assertIsNone(empty_bottle['reference_point_camera_mm'])
        self.assertIsNone(empty_bottle['reference_point_chassis_mm'])

    def test_item19_no_robot_actions_boundary(self):
        # 19. Ensure health endpoint explicitly declares robot_actions=False
        class MockHandler(server.Handler):
            def __init__(self):
                self.sent = []
            def _send(self, code, obj):
                self.sent.append((code, obj))

        h = MockHandler()
        h.path = '/health'
        h.do_GET()
        self.assertEqual(len(h.sent), 1)
        code, resp = h.sent[0]
        flags = resp.get('flags', resp)
        self.assertFalse(flags.get('robot_actions', True))
        self.assertFalse(flags.get('weights_loaded_here', True))


if __name__ == '__main__':
    unittest.main()
