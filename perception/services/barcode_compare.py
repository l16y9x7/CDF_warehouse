"""Independent diagnostic decoders; their outputs never replace the business result."""

from __future__ import annotations

from importlib import import_module, metadata
from pathlib import Path
from threading import local
import time

import cv2
import numpy as np

from config import PERCEPTION_BARCODE_COMPARISON_ENABLED, PERCEPTION_BARCODE_SR_MODEL_DIR
from core.recognize_trace import BarcodeComparisonTrace, RecognizePipelineTrace
from services.qr_decode import (
    BARCODE_DECODE_ROTATION_ANGLES, _preprocess_variants_named, _rotate_image, expand_bbox,
)

COMPARISON_METHODS = ("opencv_sr", "zxing_cpp")
_thread_models = local()


def skipped_comparisons(reason: str) -> dict[str, BarcodeComparisonTrace]:
    return {method: BarcodeComparisonTrace(method=method, reason=reason) for method in COMPARISON_METHODS}


def _get_sr_detector():
    # OpenCV DNN Net is mutable during inference. Never share a detector between
    # FastAPI worker threads. Reload only if the files or configured path change.
    directory = Path(PERCEPTION_BARCODE_SR_MODEL_DIR)
    paths = (directory / "sr.prototxt", directory / "sr.caffemodel")
    key = tuple((str(path), path.stat().st_mtime_ns, path.stat().st_size) for path in paths)
    cached = getattr(_thread_models, "sr", None)
    if cached is None or cached[0] != key:
        if not hasattr(cv2, "dnn"):
            raise RuntimeError("OpenCV was built without DNN support")
        # Relative paths also avoid OpenCV's narrow-character Windows file API
        # rejecting a Chinese workspace prefix when launched inside the repo.
        model_paths = []
        for path in paths:
            try:
                model_paths.append(str(path.relative_to(Path.cwd())))
            except ValueError:
                model_paths.append(str(path))
        detector = cv2.barcode.BarcodeDetector(*model_paths)
        _thread_models.sr = (key, detector)
    return _thread_models.sr[1]


def _opencv_sr_codes(detector, candidate: np.ndarray) -> list[dict[str, str]]:
    ok, contents, formats, _ = detector.detectAndDecodeWithType(candidate)
    if not ok:
        return []
    return [
        {"content": content.strip(), "format": str(formats[index]) if index < len(formats) else ""}
        for index, content in enumerate(contents) if content and content.strip()
    ]


def _run_opencv_sr(crop: np.ndarray, result: BarcodeComparisonTrace) -> None:
    result.runtime.update({
        "opencv_version": cv2.__version__,
        "model_dir": str(PERCEPTION_BARCODE_SR_MODEL_DIR),
        # BarcodeDetector does not expose whether its internal size gate ran SR.
        # Do not claim that every successful attempt actually used the network.
        "sr_configured": False,
        "sr_execution": "opencv_internal_size_gate_not_observable",
    })
    started = time.perf_counter()
    detector = _get_sr_detector()
    result.runtime["initialization_ms"] = round((time.perf_counter() - started) * 1000, 1)
    result.runtime["sr_configured"] = True
    # Start at native resolution so interpolation does not bypass the SR size
    # gate; then use the same rotations and variants as the baseline decoder.
    for enhanced in (False, True):
        for rotation in BARCODE_DECODE_ROTATION_ANGLES:
            rotated = crop if rotation == 0 else _rotate_image(crop, float(rotation))
            for variant, candidate in _preprocess_variants_named(rotated, enhanced=enhanced):
                attempt = {
                    "tier": "enhanced" if enhanced else "basic", "rotation": rotation,
                    "variant": variant, "candidate_shape": [candidate.shape[1], candidate.shape[0]],
                    "status": "ERROR", "codes": [],
                }
                result.attempts.append(attempt)
                started = time.perf_counter()
                try:
                    codes = _opencv_sr_codes(detector, candidate)
                except Exception as error:
                    attempt["error"] = f"{type(error).__name__}: {error}"
                    raise
                finally:
                    attempt["duration_ms"] = round((time.perf_counter() - started) * 1000, 1)
                attempt.update(status="FOUND" if codes else "NOT_FOUND", codes=codes)
                if codes:
                    result.codes = codes
                    return


def _run_zxing_cpp(crop: np.ndarray, result: BarcodeComparisonTrace) -> None:
    # Lazy import keeps the existing service usable when an optional deployment
    # dependency is missing; the method's record then explicitly reports ERROR.
    zxingcpp = import_module("zxingcpp")
    try:
        result.runtime["zxing_cpp_version"] = metadata.version("zxing-cpp")
    except metadata.PackageNotFoundError:
        result.runtime["zxing_cpp_version"] = "unknown"
    result.runtime.update({"formats": "LinearCodes", "try_rotate": True, "try_downscale": True})
    attempt = {
        "tier": "native", "rotation": "automatic", "variant": "bgr",
        "candidate_shape": [crop.shape[1], crop.shape[0]], "status": "ERROR", "codes": [],
    }
    result.attempts.append(attempt)
    started = time.perf_counter()
    try:
        decoded = zxingcpp.read_barcodes(
            crop, formats=zxingcpp.BarcodeFormat.LinearCodes,
            try_rotate=True, try_downscale=True,
        )
        result.codes = [
            {"content": item.text.strip(), "format": str(item.format)}
            for item in decoded if item.valid and item.text and item.text.strip()
        ]
        attempt.update(status="FOUND" if result.codes else "NOT_FOUND", codes=result.codes)
    except Exception as error:
        attempt["error"] = f"{type(error).__name__}: {error}"
        raise
    finally:
        attempt["duration_ms"] = round((time.perf_counter() - started) * 1000, 1)


def run_barcode_comparisons(image_bgr: np.ndarray, trace: RecognizePipelineTrace) -> None:
    """Use the baseline's selected bbox, but leave all its fields/results intact.

    Each method is isolated and recorded even when the other fails. The methods
    run synchronously, so total request latency includes their diagnostic work.
    """
    trace.decode_comparisons = skipped_comparisons("not_run")
    if not PERCEPTION_BARCODE_COMPARISON_ENABLED:
        trace.decode_comparisons = skipped_comparisons("disabled")
        return
    if trace.selected_bbox is None:
        trace.decode_comparisons = skipped_comparisons(trace.failure_reason or "no_selected_candidate")
        return
    started = time.perf_counter()
    try:
        bbox = expand_bbox(trace.selected_bbox, image_bgr.shape, padding_ratio=trace.bbox_padding_ratio)
        x1, y1, x2, y2 = bbox
        if x2 <= x1 or y2 <= y1:
            raise ValueError("bbox 无效")
        crop = image_bgr[y1:y2, x1:x2].copy()
        for method, decode in (("opencv_sr", _run_opencv_sr), ("zxing_cpp", _run_zxing_cpp)):
            result = trace.decode_comparisons[method]
            result.reason = None
            result.runtime.update({"expanded_bbox": bbox, "crop_shape": [crop.shape[1], crop.shape[0]]})
            method_started = time.perf_counter()
            try:
                decode(crop.copy(), result)
                result.status = "FOUND" if result.codes else "NOT_FOUND"
                result.barcode_content = result.codes[0]["content"] if result.codes else None
                if trace.barcode_content:
                    result.matches_business_result = any(
                        code["content"] == trace.barcode_content for code in result.codes
                    )
            except Exception as error:
                result.status = "ERROR"
                result.error = f"{type(error).__name__}: {error}"
            finally:
                result.duration_ms = round((time.perf_counter() - method_started) * 1000, 1)
    except Exception as error:
        for result in trace.decode_comparisons.values():
            if result.reason == "not_run":
                result.status, result.reason = "ERROR", "comparison_input_failed"
                result.error = f"{type(error).__name__}: {error}"
    finally:
        trace.timings_ms["decode_comparisons"] = round((time.perf_counter() - started) * 1000, 1)
