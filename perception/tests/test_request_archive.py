"""HTTP-level archival checks with local images and stubbed model calls."""

from __future__ import annotations

import base64
import hashlib
import json
import logging
from concurrent.futures import ThreadPoolExecutor
from datetime import date, timedelta
from pathlib import Path
import sys
import tempfile
import unittest
from unittest.mock import patch
from uuid import uuid4

import cv2
import numpy as np
import requests
from fastapi.testclient import TestClient

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from app import create_app
from clients.sam3_client import SamLocateResult
from core import request_archive, service_logging
from core.recognize_trace import RecognizePipelineTrace, Sam3CandidateTrace
from services import barcode_compare


class RequestArchiveTests(unittest.TestCase):
    def setUp(self):
        self.temporary = tempfile.TemporaryDirectory(prefix="perception_archive_test_")
        self.root = Path(self.temporary.name)
        self.addCleanup(self.temporary.cleanup)
        for name in ("perception.requests", "perception.recognize_sku_barcode", "test.rotation",
                     "perception.barcode_opencv_sr", "perception.barcode_zxing_cpp"):
            self.reset_logger(name)
            self.addCleanup(self.reset_logger, name)
        for key, value in {
            "PERCEPTION_REQUEST_DIR": str(self.root / "requests"),
            "PERCEPTION_REQUEST_LOG_PATH": str(self.root / "requests.log"),
            "RECOGNIZE_SKU_BARCODE_LOG_PATH": str(self.root / "recognize.log"),
            "PERCEPTION_OPENCV_SR_LOG_PATH": str(self.root / "opencv_sr.log"),
            "PERCEPTION_ZXING_CPP_LOG_PATH": str(self.root / "zxing_cpp.log"),
            "PERCEPTION_REQUEST_RETENTION_DAYS": 0,
        }.items():
            override = patch.object(request_archive, key, value)
            override.start()
            self.addCleanup(override.stop)
        self.client = TestClient(create_app())
        self.addCleanup(self.client.close)
        self.pixels = np.full((30, 40, 3), 120, dtype=np.uint8)
        self.png = cv2.imencode(".png", self.pixels)[1].tobytes()
        self.payload = {
            "sku_id": "same-sku", "name": "测试商品", "image_base64": base64.b64encode(self.png).decode(),
        }
        self.candidate = SamLocateResult(bbox=[12, 8, 28, 22], mask="test-mask", score=0.9, confidence=0.9)
        model = patch("services.sku_locate.locate_sam3_instances", return_value=[self.candidate])
        self.model = model.start()
        self.addCleanup(model.stop)
        decoder = patch("services.sku_recognize.decode_barcode_from_bbox", side_effect=self.decode)
        self.decoder = decoder.start()
        self.addCleanup(decoder.stop)
        enabled = patch.object(barcode_compare, "PERCEPTION_BARCODE_COMPARISON_ENABLED", True)
        enabled.start()
        self.addCleanup(enabled.stop)
        for method, function in (("sr", "_run_opencv_sr"), ("zxing", "_run_zxing_cpp")):
            comparison = patch.object(barcode_compare, function, side_effect=self.compare)
            setattr(self, method, comparison.start())
            self.addCleanup(comparison.stop)

    @staticmethod
    def reset_logger(name):
        logger = logging.getLogger(name)
        for handler in logger.handlers[:]:
            handler.close()
            logger.removeHandler(handler)

    @staticmethod
    def decode(image, bbox, *, trace, **kwargs):
        trace.record_decode_attempt(tier="basic", rotation=0, variant="bgr", result="ok", barcode_content="123456")
        return "123456"

    @staticmethod
    def compare(crop, result):
        result.codes = [{"content": "other-code", "format": "EAN13"}]

    def post(self, payload=None, endpoint="recognize_sku_barcode"):
        return self.client.post(f"/perception/{endpoint}", json=self.payload if payload is None else payload)

    def archive(self, response):
        request_id = response.headers["X-Request-ID"]
        self.assertRegex(request_id, r"^[0-9a-f]{32}$")
        matches = list((self.root / "requests").glob(f"*/{request_id}"))
        self.assertEqual(len(matches), 1)
        directory = matches[0]
        records = {name: json.loads((directory / f"{name}.json").read_text(encoding="utf-8"))
                   for name in ("request", "response", "trace")}
        for record in records.values():
            self.assertEqual(record["request_id"], request_id)
        self.assertEqual(records["response"]["http_status"], response.status_code)
        self.assertEqual(records["trace"]["state"], "completed")
        return directory, records

    def test_success_base64_preserves_original_bytes_and_response(self):
        self.payload["image_base64"] = "data:image/png;base64," + self.payload["image_base64"]

        def verify_input_before_inference(*args, **kwargs):
            saved = list((self.root / "requests").glob("*/*/input.png"))
            self.assertEqual(len(saved), 1)
            self.assertEqual(saved[0].read_bytes(), self.png)
            self.assertTrue((saved[0].parent / "request.json").exists())
            return [self.candidate]

        self.model.side_effect = verify_input_before_inference
        response = self.post()
        self.assertEqual(response.json(), {"status": "FOUND", "barcode_content": "123456"})
        directory, records = self.archive(response)
        self.assertEqual((directory / "input.png").read_bytes(), self.png)
        self.assertEqual(records["request"]["image"]["sha256"], hashlib.sha256(self.png).hexdigest())
        self.assertTrue(records["request"]["body"]["image_base64"]["omitted"])
        self.assertNotIn(self.payload["image_base64"], (directory / "request.json").read_text())
        self.assertEqual(records["response"]["body"], response.json())
        pipeline = records["trace"]["pipeline"]
        self.assertEqual(pipeline["barcode_content"], "123456")
        self.assertEqual(pipeline["selected_bbox"], self.candidate.bbox)
        self.assertEqual(len(pipeline["decode_attempts"]), 1)
        self.assertIn(response.headers["X-Request-ID"], (self.root / "recognize.log").read_text())

    def test_path_jpeg_is_read_once_and_original_is_preserved(self):
        jpeg = cv2.imencode(".jpg", self.pixels)[1].tobytes()
        original = self.root / "原图.jpg"
        original.write_bytes(jpeg)
        payload = {"sku_id": "same", "name": "path", "image_path": str(original)}
        read_bytes = Path.read_bytes
        counts = []

        def overwrite_after_read(path):
            data = read_bytes(path)
            if path == original:
                counts.append(1)
                original.write_bytes(b"replaced after read")
            return data

        with patch.object(Path, "read_bytes", overwrite_after_read):
            response = self.post(payload)
        self.assertEqual(response.status_code, 200)
        directory, records = self.archive(response)
        self.assertEqual(counts, [1])
        self.assertEqual((directory / "input.jpg").read_bytes(), jpeg)
        self.assertEqual(records["request"]["image"]["source"], "path")

    def test_not_found_is_archived_with_image_reference(self):
        for candidates, reason in (([], "sam3_empty"),
                                   ([SamLocateResult(bbox=[0, 0, 2, 2], mask="mask", score=0.9, confidence=0.9)], "center_filter_empty")):
            with self.subTest(reason=reason):
                self.model.return_value = candidates
                response = self.post()
                directory, records = self.archive(response)
                self.assertEqual(response.json()["status"], "NOT_FOUND")
                self.assertEqual(records["trace"]["pipeline"]["failure_reason"], reason)
                self.assertEqual((directory / "input.png").read_bytes(), self.png)
                self.assertIn(repr(str(directory / "input.png")), (self.root / "recognize.log").read_text(encoding="utf-8"))

    def test_upstream_error_is_not_reported_as_not_found(self):
        self.model.side_effect = requests.Timeout("test timeout")
        response = self.post()
        self.assertEqual(response.status_code, 502)
        directory, records = self.archive(response)
        self.assertEqual(records["trace"]["status"], "ERROR")
        self.assertEqual(records["trace"]["pipeline"]["failure_reason"], "sam3_request_failed")
        self.assertIn("test timeout", records["trace"]["error"]["traceback"])
        self.assertEqual((directory / "input.png").read_bytes(), self.png)
        summary = (self.root / "recognize.log").read_text(encoding="utf-8")
        self.assertIn("ERROR: sam3_request_failed", summary)
        self.assertNotIn("NOT_FOUND", summary)

    def test_validation_errors_archive_params_and_available_image_without_base64_duplication(self):
        bad_payload = {"image_base64": self.payload["image_base64"]}
        response = self.post(bad_payload)
        self.assertEqual(response.status_code, 422)
        directory, records = self.archive(response)
        self.assertEqual((directory / "input.png").read_bytes(), self.png)
        self.assertEqual(records["trace"]["error"]["type"], "validation_error")
        self.assertNotIn(self.payload["image_base64"], (directory / "response.json").read_text())
        self.model.assert_not_called()

    def test_bad_json_and_conflicting_sources_are_archived(self):
        malformed = self.client.post("/perception/recognize_sku_barcode", content=b'{"bad":',
                                     headers={"Content-Type": "application/json"})
        self.assertEqual(malformed.status_code, 422)
        directory, records = self.archive(malformed)
        self.assertEqual((directory / "request_body.bin").read_bytes(), b'{"bad":')
        self.assertIn("body_parse_error", records["request"])
        conflicting = self.post({**self.payload, "image_path": "unused.jpg"})
        self.assertEqual(conflicting.status_code, 422)
        self.archive(conflicting)
        self.model.assert_not_called()

    def test_unreadable_images_and_invalid_base64_are_archived_as_400(self):
        payloads = [
            {"sku_id": "same", "name": "missing", "image_path": str(self.root / "missing.jpg")},
            {**self.payload, "image_base64": "%%%"},
            {**self.payload, "image_base64": base64.b64encode(b"not an image").decode()},
        ]
        for payload in payloads:
            with self.subTest(payload=payload):
                response = self.post(payload)
                self.assertEqual(response.status_code, 400)
                directory, records = self.archive(response)
                self.assertEqual(records["trace"]["status"], "ERROR")
                if payload.get("image_base64") == base64.b64encode(b"not an image").decode():
                    self.assertEqual((directory / "input.bin").read_bytes(), b"not an image")
        self.model.assert_not_called()

    def test_unexpected_exception_retains_image_and_traceback(self):
        self.model.side_effect = RuntimeError("unexpected test error")
        response = self.post()
        self.assertEqual(response.status_code, 500)
        directory, records = self.archive(response)
        self.assertEqual(records["response"]["body"], "Internal Server Error")
        self.assertIn("unexpected test error", records["trace"]["error"]["traceback"])
        self.assertEqual((directory / "input.png").read_bytes(), self.png)

    def test_locate_saves_bbox_mask_and_candidates(self):
        response = self.post({"image_path": " ", "image_base64": self.payload["image_base64"]}, "locate_sku_qr_code")
        self.assertEqual(response.status_code, 200)
        directory, records = self.archive(response)
        self.assertEqual(records["response"]["body"]["mask"], "test-mask")
        self.assertEqual(records["response"]["body"]["bbox"], self.candidate.bbox)
        self.assertEqual(records["trace"]["pipeline"]["operation"], "locate_sku_qr_code")
        self.assertEqual(records["trace"]["pipeline"]["image_source"], "base64")
        self.assertEqual(len(records["trace"]["pipeline"]["sam3_candidates"]), 1)
        self.assertEqual((directory / "input.png").read_bytes(), self.png)
        self.assertNotIn("NOT_FOUND", (self.root / "recognize.log").read_text(encoding="utf-8"))

    def test_concurrent_requests_use_distinct_archives(self):
        with ThreadPoolExecutor(max_workers=4) as executor:
            responses = list(executor.map(lambda _: self.post(), range(8)))
        ids = set()
        for response in responses:
            self.assertEqual(response.status_code, 200)
            self.archive(response)
            ids.add(response.headers["X-Request-ID"])
        self.assertEqual(len(ids), 8)
        self.assertEqual(len(logging.getLogger("perception.requests").handlers), 1)

    def test_archive_write_failure_does_not_change_recognition_result(self):
        with patch.object(Path, "write_bytes", side_effect=PermissionError("simulated read-only archive")):
            response = self.post()
        self.assertEqual(response.json(), {"status": "FOUND", "barcode_content": "123456"})
        self.assertIn("X-Request-ID", response.headers)
        self.assertIn("archive_write_failed", (self.root / "requests.log").read_text())

    def test_health_does_not_create_archives(self):
        self.assertEqual(self.client.get("/perception/health").status_code, 200)
        self.assertFalse((self.root / "requests").exists())

    def test_rotating_log_has_bounded_backups(self):
        with patch.object(service_logging, "PERCEPTION_LOG_MAX_BYTES", 150), \
                patch.object(service_logging, "PERCEPTION_LOG_BACKUP_COUNT", 2):
            logger = service_logging.file_logger("test.rotation", str(self.root / "rotating.log"))
            for _ in range(12):
                logger.info("rotation test %s", "x" * 80)
        self.assertTrue((self.root / "rotating.log.1").exists())
        self.assertTrue((self.root / "rotating.log.2").exists())
        self.assertFalse((self.root / "rotating.log.3").exists())
        self.assertEqual(len(logger.handlers), 1)

    def test_retention_preserves_active_unknown_and_recent_data(self):
        root = self.root / "retention"
        old_day = date.today() - timedelta(days=10)

        def create(day, state, *, managed=True):
            request_id = uuid4().hex
            directory = root / day.isoformat() / request_id
            directory.mkdir(parents=True)
            record = {"archive_version": 1 if managed else 999, "request_id": request_id, "state": state}
            (directory / "trace.json").write_text(json.dumps(record), encoding="utf-8")
            return directory

        expired = create(old_day, "completed")
        active = create(old_day, "received")
        unknown = create(old_day, "completed", managed=False)
        recent = create(date.today(), "completed")
        request_archive.cleanup_archives(root, 0)
        self.assertTrue(expired.exists())
        request_archive.cleanup_archives(root, 7)
        self.assertFalse(expired.exists())
        for directory in (active, unknown, recent):
            self.assertTrue(directory.exists())

    def test_summary_early_returns_include_image_and_correct_status(self):
        trace = RecognizePipelineTrace(request_id="test", status="ERROR", failure_reason="sam3_request_failed",
                                       saved_image_path="input.png")
        self.assertNotIn("NOT_FOUND", trace.format_summary())
        self.assertIn("input.png", trace.format_summary())
        trace.status = "NOT_FOUND"
        self.assertIn("input.png", trace.format_summary())
        trace.sam3_candidates = [Sam3CandidateTrace(1, 0.9, [0, 0, 2, 2], 0.9, False)]
        self.assertIn("input.png", trace.format_summary())

    def test_comparisons_are_saved_separately_without_replacing_business_result(self):
        response = self.post()
        self.assertEqual(response.json(), {"status": "FOUND", "barcode_content": "123456"})
        directory, records = self.archive(response)
        for method in ("opencv_sr", "zxing_cpp"):
            record = json.loads((directory / f"decode_{method}.json").read_text(encoding="utf-8"))
            self.assertTrue(record["comparison_only"])
            self.assertEqual(record["request_id"], response.headers["X-Request-ID"])
            self.assertEqual(record["status"], "FOUND")
            self.assertEqual(record["barcode_content"], "other-code")
            self.assertEqual(record["business_barcode_content"], "123456")
            self.assertFalse(record["matches_business_result"])
            self.assertEqual(record["image"]["sha256"], hashlib.sha256(self.png).hexdigest())
            self.assertEqual(record["saved_image_path"], str(directory / "input.png"))
            self.assertEqual(records["trace"]["pipeline"]["decode_comparisons"][method]["status"], "FOUND")
            self.assertIn(response.headers["X-Request-ID"], (self.root / f"{method}.log").read_text())
        self.assertEqual(len(records["trace"]["pipeline"]["decode_attempts"]), 1)

    def test_comparison_success_never_changes_not_found_response(self):
        self.decoder.side_effect = None
        self.decoder.return_value = None
        response = self.post()
        self.assertEqual(response.json(), {"status": "NOT_FOUND", "barcode_content": None})
        _, records = self.archive(response)
        self.assertEqual(records["trace"]["pipeline"]["failure_reason"], "decode_failed")
        for record in records["trace"]["pipeline"]["decode_comparisons"].values():
            self.assertEqual(record["status"], "FOUND")

    def test_each_comparison_failure_is_isolated(self):
        self.sr.side_effect = FileNotFoundError("sr.caffemodel missing")
        response = self.post()
        self.assertEqual(response.json(), {"status": "FOUND", "barcode_content": "123456"})
        _, records = self.archive(response)
        comparisons = records["trace"]["pipeline"]["decode_comparisons"]
        self.assertEqual(comparisons["opencv_sr"]["status"], "ERROR")
        self.assertIn("sr.caffemodel missing", comparisons["opencv_sr"]["error"])
        self.assertEqual(comparisons["zxing_cpp"]["status"], "FOUND")
        self.zxing.side_effect = ImportError("zxingcpp missing")
        response = self.post()
        self.assertEqual(response.json()["barcode_content"], "123456")
        _, records = self.archive(response)
        self.assertEqual(records["trace"]["pipeline"]["decode_comparisons"]["zxing_cpp"]["status"], "ERROR")

    def test_comparisons_skip_rejected_candidates_and_failed_requests(self):
        self.model.return_value = []
        response = self.post()
        directory, _ = self.archive(response)
        self.sr.assert_not_called()
        self.zxing.assert_not_called()
        for method in ("opencv_sr", "zxing_cpp"):
            record = json.loads((directory / f"decode_{method}.json").read_text())
            self.assertEqual((record["status"], record["reason"]), ("SKIPPED", "sam3_empty"))
        response = self.post({})
        directory, _ = self.archive(response)
        for method in ("opencv_sr", "zxing_cpp"):
            record = json.loads((directory / f"decode_{method}.json").read_text())
            self.assertEqual((record["status"], record["reason"]), ("SKIPPED", "request_failed"))

    def test_comparison_archive_failure_preserves_business_response(self):
        write_bytes = Path.write_bytes

        def fail_comparisons(path, data):
            if path.name.startswith("decode_"):
                raise PermissionError("comparison evidence unwritable")
            return write_bytes(path, data)

        with patch.object(Path, "write_bytes", fail_comparisons):
            response = self.post()
        self.assertEqual(response.json()["barcode_content"], "123456")
        _, records = self.archive(response)
        self.assertEqual(len(records["trace"]["archive_errors"]), 2)
        self.assertIn(response.headers["X-Request-ID"], (self.root / "zxing_cpp.log").read_text())

    def test_locate_does_not_run_or_archive_comparisons(self):
        response = self.post({"image_base64": self.payload["image_base64"]}, "locate_sku_qr_code")
        directory, _ = self.archive(response)
        self.sr.assert_not_called()
        self.zxing.assert_not_called()
        self.assertEqual(list(directory.glob("decode_*.json")), [])


if __name__ == "__main__":
    unittest.main()
