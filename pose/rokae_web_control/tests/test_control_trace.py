import copy
import csv
import json
from pathlib import Path
import tempfile
import threading
import unittest
import zipfile

from rokae_web.audit import JsonlAuditLogger, MemoryAuditLogger
from rokae_web.backends import MockRobotBackend, MockChassisBackend
from rokae_web.config import DEFAULT_CONFIG
from rokae_web.control_trace import context, fields, invoke, worker_thread
from rokae_web.log_analysis import export
from rokae_web.service import ControlService


class TraceTests(unittest.TestCase):
    def test_concurrent_workers_keep_separate_contexts(self):
        audit = MemoryAuditLogger()
        gate = threading.Barrier(3)
        def worker():
            gate.wait()
            audit.record("worker")
        workers = []
        for value in ("one", "two"):
            token = context.set({"operation_id": value})
            workers.append(worker_thread(target=worker))
            context.reset(token)
        for worker in workers:
            worker.start()
        gate.wait()
        for worker in workers:
            worker.join(2)
        self.assertEqual({r["operation_id"] for r in audit.records}, {"one", "two"})
        self.assertNotIn("operation_id", fields())

    def test_sdk_mutable_output_and_error_are_preserved_without_changing_call(self):
        audit = MemoryAuditLogger()
        class PyTypeVectorInt:
            def __init__(self):
                self.data = [0]
            def content(self):
                return self.data
        output = PyTypeVectorInt()
        def call(value, ec):
            value.data[0] = 123
            ec.update(ec=-32, message="no IK")
            return [1.2]
        ec = {}
        self.assertEqual(invoke(audit.record, "IK", call, (output,), ec), [1.2])
        record = audit.records[-1]
        self.assertEqual(record["args"][0]["content"], [0])
        self.assertEqual(record["args_after"][0]["content"], [123])
        self.assertEqual(record["ec"]["ec"], -32)
        self.assertGreaterEqual(record["duration_ms"], 0)

    def test_jsonl_assets_and_offline_export(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            config = copy.deepcopy(DEFAULT_CONFIG)
            calibration = root / "calibration.json"
            calibration.write_text('{"transform": [1, 2, 3]}')
            model = root / "model.zip"
            with zipfile.ZipFile(model, "w") as archive:
                archive.writestr("robot/robot.urdf", "<robot name='test'/>")
                archive.writestr("robot/meshes/large.stl", "not needed")
            config["pose_estimation"].update(calibration_file=str(calibration), urdf_file=str(model))
            logger = JsonlAuditLogger(root / "logs")
            logger.capture_assets(config)
            token = context.set({"request_id": "click", "operation_id": "move"})
            try:
                logger.record("robot_state_observed", state=MockRobotBackend().read_state())
                logger.record("phase_finished", phase_name="pregrasp_plan", duration_ms=12.5, ok=True)
                logger.record("robot_state_sample", request_id="overlapping-click", operation_id="overlapping-move",
                              related_request_ids=["click"], related_operation_ids=["move"],
                              state=MockRobotBackend().read_state())
            finally:
                context.reset(token)
            log = next((root / "logs").glob("motion-*.jsonl"))
            entries = [json.loads(line) for line in log.read_text().splitlines()]
            self.assertEqual(entries[1]["schema_version"], 2)
            self.assertGreater(entries[2]["monotonic_ns"], entries[1]["monotonic_ns"])
            self.assertTrue(any(asset.get("file", "").endswith(".urdf") for asset in entries[0]["assets"]))
            report = export(log, root / "analysis", "click")
            self.assertEqual(report["state_rows"], 2)
            self.assertEqual(report["phases"][0]["duration_ms"], 12.5)
            with (root / "analysis" / "states.csv").open() as handle:
                rows = list(csv.DictReader(handle))
            self.assertEqual(rows[0]["operation_id"], "move")
            self.assertIn("right_arm_j7_deg", rows[0])
            logger.close()

    def test_disk_error_is_visible_and_does_not_prevent_stop(self):
        class BrokenAudit:
            def record(self, *args, **kwargs):
                raise OSError("disk full")
            def close(self):
                pass
        service = ControlService(copy.deepcopy(DEFAULT_CONFIG), MockRobotBackend(), MockChassisBackend(), False,
                                 audit=BrokenAudit())
        try:
            service.chassis_stop()
            service.disarm()
            status = service.status()
            self.assertEqual(status["logging"]["error"], "disk full")
            self.assertGreater(status["logging"]["failed_records"], 0)
            self.assertFalse(status["armed"])
        finally:
            service.close()


if __name__ == "__main__":
    unittest.main()
