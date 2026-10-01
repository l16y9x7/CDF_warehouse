"""Decoder isolation checks; optional native smoke checks need no SAM3/GPU."""

from __future__ import annotations

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
from services import barcode_compare, qr_decode


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


class BarcodeComparisonTests(unittest.TestCase):
    def setUp(self):
        enabled = patch.object(barcode_compare, "PERCEPTION_BARCODE_COMPARISON_ENABLED", True)
        enabled.start()
        self.addCleanup(enabled.stop)
        self.image = np.arange(100 * 120 * 3, dtype=np.uint8).reshape(100, 120, 3)
        self.trace = RecognizePipelineTrace(
            selected_bbox=[20, 25, 80, 75], bbox_padding_ratio=0.15,
            barcode_content="original", decode_error="old-error", timings_ms={"decode": 12.3},
        )

    def test_same_crop_as_baseline_and_no_cross_method_or_business_mutation(self):
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
            barcode_compare.run_barcode_comparisons(self.image, self.trace)
        for crop in crops:
            np.testing.assert_array_equal(crop, baseline_crop[0])
        np.testing.assert_array_equal(self.image, image_before)
        after = asdict(self.trace)
        for key in before.keys() - {"decode_comparisons", "timings_ms"}:
            self.assertEqual(before[key], after[key])
        self.assertEqual(self.trace.timings_ms["decode"], 12.3)
        self.assertTrue(all(r.matches_business_result for r in self.trace.decode_comparisons.values()))

    def test_disabled_and_invalid_input_do_not_call_decoders(self):
        with patch.object(barcode_compare, "_run_opencv_sr") as sr, patch.object(barcode_compare, "_run_zxing_cpp") as zx:
            with patch.object(barcode_compare, "PERCEPTION_BARCODE_COMPARISON_ENABLED", False):
                barcode_compare.run_barcode_comparisons(self.image, self.trace)
            self.assertTrue(all(r.reason == "disabled" for r in self.trace.decode_comparisons.values()))
            self.trace.selected_bbox = [20, 20, 10, 10]
            barcode_compare.run_barcode_comparisons(self.image, self.trace)
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

    def test_missing_dependency_is_error_and_other_method_still_runs(self):
        with patch.object(barcode_compare, "_run_opencv_sr") as sr, \
                patch.object(barcode_compare, "import_module", side_effect=ModuleNotFoundError("zxingcpp")):
            barcode_compare.run_barcode_comparisons(self.image, self.trace)
        sr.assert_called_once()
        self.assertEqual(self.trace.decode_comparisons["opencv_sr"].status, "NOT_FOUND")
        self.assertEqual(self.trace.decode_comparisons["zxing_cpp"].status, "ERROR")
        self.assertIn("ModuleNotFoundError", self.trace.decode_comparisons["zxing_cpp"].error)
        self.assertEqual(self.trace.barcode_content, "original")

    def test_sr_cache_reuses_per_thread_but_never_shares_across_threads(self):
        with tempfile.TemporaryDirectory() as directory:
            for name in ("sr.prototxt", "sr.caffemodel"):
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


class NativeBarcodeSmokeTests(unittest.TestCase):
    def test_real_sr_model_and_original_decoder(self):
        directory = Path(barcode_compare.PERCEPTION_BARCODE_SR_MODEL_DIR)
        if not all((directory / name).is_file() for name in ("sr.prototxt", "sr.caffemodel")):
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


if __name__ == "__main__":
    unittest.main()
