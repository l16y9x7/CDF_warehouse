"""Independent diagnostic decoders; their outputs never replace the business result."""

from __future__ import annotations

import asyncio
from importlib import import_module, metadata
from pathlib import Path
from threading import local
import time

import cv2
import numpy as np
from starlette.concurrency import run_in_threadpool

from config import PERCEPTION_BARCODE_COMPARISON_ENABLED, PERCEPTION_BARCODE_SR_MODEL_DIR
from core.recognize_trace import BarcodeComparisonTrace, RecognizePipelineTrace
from services.qr_decode import (
    BARCODE_DECODE_ROTATION_ANGLES, _preprocess_variants_named, _rotate_image, expand_bbox,
)

COMPARISON_METHODS = ("opencv_sr", "zxing_cpp")
# ZXing's built-in rotation only adds orthogonal scan directions. Cover the
# remaining angles after a native miss, at most 19 calls including native.
ZXING_RETRY_ANGLES = (0,) + tuple(angle for step in range(5, 46, 5) for angle in (-step, step))
_thread_models = local()


def skipped_comparisons(reason: str) -> dict[str, BarcodeComparisonTrace]:
    return {method: BarcodeComparisonTrace(method=method, reason=reason) for method in COMPARISON_METHODS}


def _sr_model_paths() -> tuple[Path, ...]:
    directory = Path(PERCEPTION_BARCODE_SR_MODEL_DIR)
    if int(cv2.__version__.split(".")[0]) >= 5:
        return (directory / "sr.onnx",)
    return (directory / "sr.prototxt", directory / "sr.caffemodel")


def _get_sr_detector():
    # OpenCV DNN Net is mutable during inference. Never share a detector between
    # FastAPI worker threads. Reload only if the files or configured path change.
    paths = _sr_model_paths()
    for path in paths:
        if not path.is_file():
            raise FileNotFoundError(
                f"OpenCV {cv2.__version__} requires SR model file {path}; "
                "OpenCV 5 uses sr.onnx, OpenCV 4 uses sr.prototxt + sr.caffemodel"
            )
    key = (cv2.__version__, tuple((str(path), path.stat().st_mtime_ns, path.stat().st_size) for path in paths))
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
        "model_format": "onnx" if len(_sr_model_paths()) == 1 else "caffe",
        "model_files": [path.name for path in _sr_model_paths()],
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
    result.runtime.update({
        "formats": "LinearCodes", "try_rotate": True, "try_downscale": True,
        "retry_angles": list(ZXING_RETRY_ANGLES),
    })
    for angle in ZXING_RETRY_ANGLES:
        started = time.perf_counter()
        candidate = crop if angle == 0 else _rotate_image(crop, float(angle))
        attempt = {
            "tier": "native" if angle == 0 else "rotation_retry", "rotation": angle, "variant": "bgr",
            "candidate_shape": [candidate.shape[1], candidate.shape[0]], "status": "ERROR", "codes": [],
        }
        result.attempts.append(attempt)
        try:
            decoded = zxingcpp.read_barcodes(
                candidate, formats=zxingcpp.BarcodeFormat.LinearCodes,
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
        if result.codes:
            return


def _run_comparison(decode, crop: np.ndarray, result: BarcodeComparisonTrace) -> None:
    """One worker owns one result and one image copy, including on failure."""
    result.reason = None
    started = time.perf_counter()
    try:
        decode(crop.copy(), result)
        result.status = "FOUND" if result.codes else "NOT_FOUND"
        result.barcode_content = result.codes[0]["content"] if result.codes else None
    except Exception as error:
        result.status = "ERROR"
        result.error = f"{type(error).__name__}: {error}"
    finally:
        result.duration_ms = round((time.perf_counter() - started) * 1000, 1)


async def run_barcode_comparisons(image_bgr: np.ndarray, trace: RecognizePipelineTrace) -> None:
    """Run both diagnostic decoders concurrently using the selected bbox.

    Workers only write their own comparison record. Matching against the
    business result is deferred to the caller after all three methods finish.
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
        jobs = []
        for method, decode in (("opencv_sr", _run_opencv_sr), ("zxing_cpp", _run_zxing_cpp)):
            result = trace.decode_comparisons[method]
            result.runtime.update({"expanded_bbox": bbox, "crop_shape": [crop.shape[1], crop.shape[0]]})
            jobs.append(run_in_threadpool(_run_comparison, decode, crop, result))
        outcomes = await asyncio.gather(*jobs, return_exceptions=True)
        for method, outcome in zip(COMPARISON_METHODS, outcomes):
            if isinstance(outcome, Exception):
                result = trace.decode_comparisons[method]
                result.status = "ERROR"
                result.reason = "comparison_execution_failed"
                result.error = f"{type(outcome).__name__}: {outcome}"
    except Exception as error:
        for result in trace.decode_comparisons.values():
            if result.reason == "not_run":
                result.status, result.reason = "ERROR", "comparison_input_failed"
                result.error = f"{type(error).__name__}: {error}"
    finally:
        trace.timings_ms["decode_comparisons"] = round((time.perf_counter() - started) * 1000, 1)
