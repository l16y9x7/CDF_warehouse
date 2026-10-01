"""Decoder isolation checks; optional native smoke checks need no SAM3/GPU."""

from __future__ import annotations

import asyncio
from concurrent.futures import ThreadPoolExecutor
from dataclasses import asdict
from pathlib import Path
import sys
import tempfile
from threading import Barrier, local
from types import SimpleNamespace
import unittest
from unittest.mock import patch

import cv2
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from core.recognize_trace import BarcodeComparisonTrace, RecognizePipelineTrace
from clients.sam3_client import SamLocateResult
from services import barcode_compare, qr_decode, sku_recognize


def ean13_sample():
    # Valid EAN-13 5901234123457, L/G parity LGGLLG for leading digit 5.
    even = ('0100111', '0110011', '0011011', '0100001', '0011101',
            '0111001', '0000101', '0010001', '0001001', '0010111')
    # Use explicit standard L patterns (separate from the decoder under test).
    left = ('0001101', '0011001', '0010011', '0111101', '0100011',
            '0110001', '0101111', '0111011', '0110111', '0001011')
    bits = '101' + ''.join((left if kind == 'L' else even)[int(digit)]
                         for kind, digit in zip('LGGLLG', '901234'))
    bits += '01010' + ''.join(''.join('1' if b == '0' else '0' for b in left[int(d)]) for d in '123457') + '101'
    row = np.array([0 if b == '1' else 255 for b in ('0' * 12 + bits + '0' * 12)], dtype=np.uint8)
    gray = np.tile(np.repeat(row, 2), (80, 1))
    return cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR)


class BarcodeComparisonTests(unittest.IsolatedAsyncioTestCase):
    def setUp(self):
        enabled = patch.object(barcode_compare, "PERCEPTION_BARCODE_COMPARISON_ENABLED", True)
        enabled.start()
        self.addCleanup(enabled.stop)
        self.image = np.arange(100 * 120 * 3, dtype=np.uint8).reshape(100, 120, 3)
        self.trace = RecognizePipelineTrace(
            selected_bbox=[20, 25, 80, 75], bbox_padding_ratio=0.15,
            barcode_content="original", decode_error="old-error", timings_ms={"decode": 12.3},
        )

    async def test_same_crop_as_baseline_and_no_cross_method_or_business_mutation(self):
        before = asdict(self.trace)
        image_before = self.image.copy()
        baseline_crop = []
        with patch.object(qr_decode, "_decode_barcode_pipeline", side_effect=lambda crop, **_: baseline_crop.append(crop.copy())):
            qr_decode.decode_barcode_from_bbox(self.image, self.trace.selected_bbox, padding_ratio=0.15)
        crops = []

        def decode(crop, result):
            crops.append(crop.copy())
            crop[:] = 0
            result.codes = [{"content": "original", "format": "EAN13"}]

        with patch.object(barcode_compare, "_run_opencv_sr", side_effect=decode), \
                patch.object(barcode_compare, "_run_zxing_cpp", side_effect=decode):
            await barcode_compare.run_barcode_comparisons(self.image, self.trace)
        for crop in crops:
            np.testing.assert_array_equal(crop, baseline_crop[0])
        np.testing.assert_array_equal(self.image, image_before)
        after = asdict(self.trace)
        for key in before.keys() - {"decode_comparisons", "timings_ms"}:
            self.assertEqual(before[key], after[key])
        self.assertEqual(self.trace.timings_ms["decode"], 12.3)
        self.assertTrue(all(r.barcode_content == "original" for r in self.trace.decode_comparisons.values()))
        self.assertTrue(all(r.matches_business_result is None for r in self.trace.decode_comparisons.values()))

    async def test_disabled_and_invalid_input_do_not_call_decoders(self):
        with patch.object(barcode_compare, "_run_opencv_sr") as sr, patch.object(barcode_compare, "_run_zxing_cpp") as zx:
            with patch.object(barcode_compare, "PERCEPTION_BARCODE_COMPARISON_ENABLED", False):
                await barcode_compare.run_barcode_comparisons(self.image, self.trace)
            self.assertTrue(all(r.reason == "disabled" for r in self.trace.decode_comparisons.values()))
            self.trace.selected_bbox = [20, 20, 10, 10]
            await barcode_compare.run_barcode_comparisons(self.image, self.trace)
            self.assertTrue(all(r.status == "ERROR" for r in self.trace.decode_comparisons.values()))
            sr.assert_not_called()
            zx.assert_not_called()

    def test_sr_native_crop_first_and_attempt_details(self):
        detector = SimpleNamespace(detectAndDecodeWithType=lambda image: (True, ('one', 'two'), ('EAN_13', 'EAN_8'), None))
        result = BarcodeComparisonTrace("opencv_sr")
        with patch.object(barcode_compare, "_get_sr_detector", return_value=detector):
            barcode_compare._run_opencv_sr(self.image, result)
        self.assertEqual(len(result.attempts), 1)
        self.assertEqual(result.attempts[0]["variant"], "bgr")
        self.assertEqual(result.attempts[0]["rotation"], 0)
        self.assertEqual(result.attempts[0]["candidate_shape"], [120, 100])
        self.assertEqual([code["content"] for code in result.codes], ["one", "two"])
        self.assertTrue(result.runtime["sr_configured"])
        self.assertIn("not_observable", result.runtime["sr_execution"])

    async def test_missing_dependency_is_error_and_other_method_still_runs(self):
        with patch.object(barcode_compare, "_run_opencv_sr") as sr, \
                patch.object(barcode_compare, "import_module", side_effect=ModuleNotFoundError("zxingcpp")):
            await barcode_compare.run_barcode_comparisons(self.image, self.trace)
        sr.assert_called_once()
        self.assertEqual(self.trace.decode_comparisons["opencv_sr"].status, "NOT_FOUND")
        self.assertEqual(self.trace.decode_comparisons["zxing_cpp"].status, "ERROR")
        self.assertIn("ModuleNotFoundError", self.trace.decode_comparisons["zxing_cpp"].error)
        self.assertEqual(self.trace.barcode_content, "original")

    def test_sr_cache_reuses_per_thread_but_never_shares_across_threads(self):
        with tempfile.TemporaryDirectory() as directory:
            for name in ("sr.prototxt", "sr.caffemodel", "sr.onnx"):
                (Path(directory) / name).write_bytes(b"test")
            gate = Barrier(2)

            def get_twice():
                first = barcode_compare._get_sr_detector()
                gate.wait(timeout=5)
                self.assertIs(first, barcode_compare._get_sr_detector())
                return first

            with patch.object(barcode_compare, "PERCEPTION_BARCODE_SR_MODEL_DIR", Path(directory)), \
                    patch.object(barcode_compare, "_thread_models", local()), \
                    patch.object(cv2.barcode, "BarcodeDetector", side_effect=lambda *args: object()) as construct:
                with ThreadPoolExecutor(max_workers=2) as pool:
                    results = list(pool.map(lambda _: get_twice(), range(2)))
                self.assertIsNot(results[0], results[1])
                self.assertEqual(construct.call_count, 2)

    def test_sr_selects_version_specific_constructor_and_cache(self):
        with tempfile.TemporaryDirectory() as directory:
            for name in ("sr.prototxt", "sr.caffemodel", "sr.onnx"):
                (Path(directory) / name).write_bytes(b"test")
            with patch.object(barcode_compare, "PERCEPTION_BARCODE_SR_MODEL_DIR", Path(directory)), \
                    patch.object(barcode_compare, "_thread_models", local()), \
                    patch.object(cv2.barcode, "BarcodeDetector", side_effect=lambda *args: object()) as construct:
                for version, names in (("4.13.0", ["sr.prototxt", "sr.caffemodel"]), ("5.0.0", ["sr.onnx"])):
                    with self.subTest(version=version), patch.object(cv2, "__version__", version):
                        detector = barcode_compare._get_sr_detector()
                        self.assertEqual([Path(path).name for path in construct.call_args.args], names)
                        self.assertIs(detector, barcode_compare._get_sr_detector())
                self.assertEqual(construct.call_count, 2)

    def test_opencv5_missing_onnx_reports_required_file(self):
        with tempfile.TemporaryDirectory() as directory:
            for name in ("sr.prototxt", "sr.caffemodel"):
                (Path(directory) / name).write_bytes(b"test")
            with patch.object(barcode_compare, "PERCEPTION_BARCODE_SR_MODEL_DIR", Path(directory)), \
                    patch.object(cv2, "__version__", "5.0.0"), \
                    patch.object(cv2.barcode, "BarcodeDetector") as construct:
                with self.assertRaisesRegex(FileNotFoundError, "requires SR model file .*sr.onnx"):
                    barcode_compare._get_sr_detector()
                construct.assert_not_called()

    def test_zxing_filters_to_linear_and_logs_all_valid_codes(self):
        module = SimpleNamespace(
            BarcodeFormat=SimpleNamespace(LinearCodes="linear-only"),
            read_barcodes=lambda *args, **kwargs: [],
        )
        codes = [SimpleNamespace(text="ABC123", format="Code128", valid=True),
                 SimpleNamespace(text="BAD", format="EAN13", valid=False)]
        result = BarcodeComparisonTrace("zxing_cpp")
        with patch.object(barcode_compare, "import_module", return_value=module), \
                patch.object(module, "read_barcodes", return_value=codes) as read:
            barcode_compare._run_zxing_cpp(self.image, result)
        self.assertEqual(read.call_args.kwargs["formats"], "linear-only")
        self.assertEqual(result.codes, [{"content": "ABC123", "format": "Code128"}])
        self.assertEqual(len(result.attempts), 1)
        self.assertEqual(result.attempts[0]["rotation"], 0)

    def test_zxing_rotation_retries_stop_at_first_valid_result(self):
        module = SimpleNamespace(BarcodeFormat=SimpleNamespace(LinearCodes="linear-only"), read_barcodes=None)
        found = SimpleNamespace(text="3282779003131", format="EAN13", valid=True)
        result = BarcodeComparisonTrace("zxing_cpp")
        with patch.object(barcode_compare, "import_module", return_value=module), \
                patch.object(module, "read_barcodes", side_effect=[[], [], [found]]) as read:
            barcode_compare._run_zxing_cpp(self.image, result)
        self.assertEqual(read.call_count, 3)
        self.assertEqual([item["rotation"] for item in result.attempts], [0, -5, 5])
        self.assertEqual([item["status"] for item in result.attempts], ["NOT_FOUND", "NOT_FOUND", "FOUND"])
        self.assertEqual(result.codes[0]["content"], "3282779003131")


class NativeBarcodeSmokeTests(unittest.TestCase):
    def test_real_three_method_pipeline(self):
        try:
            import zxingcpp
        except ImportError:
            self.skipTest("zxing-cpp not installed in this interpreter")
        if not hasattr(zxingcpp, "read_barcodes"):
            self.skipTest("zxing-cpp native module is unavailable")
        if not all(path.is_file() for path in barcode_compare._sr_model_paths()):
            self.skipTest("SR model files not installed")
        image = ean13_sample()
        height, width = image.shape[:2]
        candidate = SamLocateResult(bbox=[0, 0, width, height], mask="", score=0.9, confidence=0.9)
        trace = RecognizePipelineTrace()
        with patch("services.sku_locate.locate_sam3_instances", return_value=[candidate]) as sam3, \
                patch.object(barcode_compare, "PERCEPTION_BARCODE_COMPARISON_ENABLED", True):
            content = asyncio.run(sku_recognize.recognize_sku_barcode(
                image, sam3_prompt="barcode", trace=trace, timings_ms=trace.timings_ms,
            ))
        sam3.assert_called_once()
        self.assertEqual(content, "5901234123457")
        for comparison in trace.decode_comparisons.values():
            self.assertEqual(comparison.status, "FOUND")
            self.assertEqual(comparison.barcode_content, content)
            self.assertTrue(comparison.matches_business_result)
        self.assertTrue({"decode", "decode_comparisons", "decode_parallel"} <= trace.timings_ms.keys())

    def test_real_sr_model_and_original_decoder(self):
        if not all(path.is_file() for path in barcode_compare._sr_model_paths()):
            self.skipTest("SR model files not installed")
        image = ean13_sample()
        expected = "5901234123457"
        self.assertEqual(qr_decode._decode_barcode_pipeline(image), expected)
        result = BarcodeComparisonTrace("opencv_sr")
        barcode_compare._run_opencv_sr(image, result)
        self.assertEqual(result.codes[0]["content"], expected)

    def test_real_zxing_regular_rotated_and_blank(self):
        try:
            import zxingcpp
        except ImportError:
            self.skipTest("zxing-cpp not installed in this interpreter")
        if not hasattr(zxingcpp, "read_barcodes"):
            self.skipTest("zxing-cpp native module is unavailable")
        image = ean13_sample()
        for candidate in (image, cv2.rotate(image, cv2.ROTATE_90_CLOCKWISE)):
            result = BarcodeComparisonTrace("zxing_cpp")
            barcode_compare._run_zxing_cpp(candidate, result)
            self.assertEqual(result.codes[0]["content"], "5901234123457")
        result = BarcodeComparisonTrace("zxing_cpp")
        barcode_compare._run_zxing_cpp(np.full_like(image, 255), result)
        self.assertEqual(result.codes, [])
        self.assertEqual(len(result.attempts), len(barcode_compare.ZXING_RETRY_ANGLES))

    def test_zxing_tilted_archived_barcode_recovers_after_native_miss(self):
        try:
            import zxingcpp
        except ImportError:
            self.skipTest("zxing-cpp not installed in this interpreter")
        if not hasattr(zxingcpp, "read_barcodes"):
            self.skipTest("zxing-cpp native module is unavailable")
        fixture = Path(__file__).parent / "fixtures/barcode_tilted_21b789.png"
        image = cv2.imdecode(np.frombuffer(fixture.read_bytes(), dtype=np.uint8), cv2.IMREAD_COLOR)
        result = BarcodeComparisonTrace("zxing_cpp")
        barcode_compare._run_zxing_cpp(image, result)
        self.assertEqual(result.codes[0]["content"], "3282779003131")
        # Version 3.1.1 misses the native crop and succeeds on a +25 degree
        # retry; future native improvements may also pass on the first call.
        self.assertEqual(result.attempts[-1]["status"], "FOUND")


if __name__ == "__main__":
    unittest.main()
