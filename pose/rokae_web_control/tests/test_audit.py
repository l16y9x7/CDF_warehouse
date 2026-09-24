from __future__ import annotations

import json
import unittest
from pathlib import Path

from rokae_web.audit import JsonlAuditLogger


class JsonlAuditLoggerTests(unittest.TestCase):
    def test_record_is_immediately_persisted_as_jsonl(self) -> None:
        directory = Path(__file__).parent / "_audit_test_output"
        logger = JsonlAuditLogger(directory)
        logger.record("joint_motion_requested", module="left_arm", target=[1, 2, 3])

        files = list(directory.glob("motion-*.jsonl"))
        self.assertEqual(len(files), 1)
        entry = json.loads(files[0].read_text(encoding="utf-8").strip())
        self.assertEqual(entry["event"], "joint_motion_requested")
        self.assertEqual(entry["module"], "left_arm")
        self.assertEqual(entry["target"], [1, 2, 3])
        self.assertEqual(entry["sequence"], 1)
        self.assertTrue(entry["session_id"])
        logger.close()
        files[0].unlink()


if __name__ == "__main__":
    unittest.main()
