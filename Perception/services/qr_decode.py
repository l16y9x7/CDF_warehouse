"""QR and barcode decoding from image regions."""

from __future__ import annotations

from typing import TYPE_CHECKING

import cv2
import numpy as np

from core.visualize import decode_mask_b64
from models.qr_parse import TargetType

if TYPE_CHECKING:
    from core.recognize_trace import RecognizePipelineTrace

BARCODE_DECODE_ROTATION_ANGLES = (0, 90, 180, 270)


def _extract_roi_and_mask(
    image_bgr: np.ndarray,
    bbox: list[int],
    mask_b64: str,
) -> tuple[np.ndarray, np.ndarray]:
    if len(bbox) != 4:
        raise ValueError("bbox 必须是 [x1, y1, x2, y2]")

    height, width = image_bgr.shape[:2]
    x1, y1, x2, y2 = map(int, bbox)
    x1 = max(0, min(x1, width))
    x2 = max(0, min(x2, width))
    y1 = max(0, min(y1, height))
    y2 = max(0, min(y2, height))
    if x2 <= x1 or y2 <= y1:
        raise ValueError("bbox 无效")

    mask = decode_mask_b64(mask_b64)
    if mask is None:
        raise ValueError("mask 无法解码")

    if mask.shape[:2] != (height, width):
        mask = cv2.resize(mask, (width, height), interpolation=cv2.INTER_NEAREST)

    mask_roi = (mask[y1:y2, x1:x2] > 0).astype(np.uint8)
    roi = image_bgr[y1:y2, x1:x2].copy()
    roi[mask_roi == 0] = 255
    return roi, mask_roi


def extract_code_roi(
    image_bgr: np.ndarray,
    bbox: list[int],
    mask_b64: str,
) -> np.ndarray:
    roi, _ = _extract_roi_and_mask(image_bgr, bbox, mask_b64)
    return roi


def _largest_contour(mask_roi: np.ndarray) -> np.ndarray | None:
    contours, _ = cv2.findContours(
        (mask_roi > 0).astype(np.uint8),
        cv2.RETR_EXTERNAL,
        cv2.CHAIN_APPROX_SIMPLE,
    )
    if not contours:
        return None

    contour = max(contours, key=cv2.contourArea)
    if cv2.contourArea(contour) < 4:
        return None
    return contour


def _deskew_angle_from_mask(mask_roi: np.ndarray) -> float | None:
    cleaned = cv2.morphologyEx(
        (mask_roi > 0).astype(np.uint8),
        cv2.MORPH_CLOSE,
        np.ones((3, 3), dtype=np.uint8),
    )
    contour = _largest_contour(cleaned)
    if contour is None:
        return None

    _, (width, height), angle = cv2.minAreaRect(contour)
    if width <= 1 or height <= 1:
        return None

    if width < height:
        angle += 90.0

    aspect = max(width, height) / max(1.0, min(width, height))
    if aspect < 1.2:
        return None

    if abs(angle) < 0.5:
        return 0.0
    return angle


def _rotate_image(image: np.ndarray, angle: float) -> np.ndarray:
    height, width = image.shape[:2]
    center = (width / 2.0, height / 2.0)
    matrix = cv2.getRotationMatrix2D(center, angle, 1.0)
    cos = abs(matrix[0, 0])
    sin = abs(matrix[0, 1])
    new_width = int(height * sin + width * cos)
    new_height = int(height * cos + width * sin)
    matrix[0, 2] += new_width / 2.0 - center[0]
    matrix[1, 2] += new_height / 2.0 - center[1]
    border_value = (255, 255, 255) if image.ndim == 3 else 255
    return cv2.warpAffine(
        image,
        matrix,
        (new_width, new_height),
        flags=cv2.INTER_LINEAR,
        borderMode=cv2.BORDER_CONSTANT,
        borderValue=border_value,
    )


def _to_gray(image: np.ndarray) -> np.ndarray:
    if image.ndim == 2:
        return image
    return cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)


def _resize_long_edge(gray: np.ndarray, target_long_edge: int) -> np.ndarray:
    height, width = gray.shape[:2]
    long_edge = max(height, width)
    if long_edge >= target_long_edge:
        return gray
    scale = target_long_edge / long_edge
    return cv2.resize(
        gray,
        (max(1, int(width * scale)), max(1, int(height * scale))),
        interpolation=cv2.INTER_LANCZOS4,
    )


def _apply_clahe(gray: np.ndarray) -> np.ndarray:
    clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8, 8))
    return clahe.apply(gray)


def _apply_unsharp(gray: np.ndarray, *, sigma: float = 1.0, amount: float = 1.5) -> np.ndarray:
    blurred = cv2.GaussianBlur(gray, (0, 0), sigma)
    sharpened = cv2.addWeighted(gray, 1.0 + amount, blurred, -amount, 0)
    return np.clip(sharpened, 0, 255).astype(np.uint8)


def _apply_otsu(gray: np.ndarray) -> np.ndarray:
    _, binary = cv2.threshold(gray, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)
    return binary


def _apply_adaptive_threshold(gray: np.ndarray) -> np.ndarray:
    block_size = max(11, (min(gray.shape[:2]) // 8) | 1)
    return cv2.adaptiveThreshold(
        gray,
        255,
        cv2.ADAPTIVE_THRESH_GAUSSIAN_C,
        cv2.THRESH_BINARY,
        block_size,
        2,
    )


def _dedupe_variants_named(variants: list[tuple[str, np.ndarray]]) -> list[tuple[str, np.ndarray]]:
    seen: set[tuple[int, ...]] = set()
    unique: list[tuple[str, np.ndarray]] = []
    for name, image in variants:
        key = (image.shape[0], image.shape[1], image.ndim, image.tobytes())
        if key in seen:
            continue
        seen.add(key)
        unique.append((name, image))
    return unique


def _preprocess_variants_basic_named(image_bgr: np.ndarray) -> list[tuple[str, np.ndarray]]:
    gray = _to_gray(image_bgr)
    variants: list[tuple[str, np.ndarray]] = [("bgr", image_bgr), ("gray", gray)]
    enlarged = _resize_long_edge(gray, 256)
    if enlarged is not gray:
        variants.append(("enlarged_256", enlarged))
    return variants


def _preprocess_variants_enhanced_named(image_bgr: np.ndarray) -> list[tuple[str, np.ndarray]]:
    gray = _to_gray(image_bgr)
    clahe = _apply_clahe(gray)
    clahe_large = _apply_clahe(_resize_long_edge(gray, 1024))
    variants = [
        ("resize_512", _resize_long_edge(gray, 512)),
        ("resize_1024", _resize_long_edge(gray, 1024)),
        ("clahe", clahe),
        ("clahe_1024", clahe_large),
        ("unsharp", _apply_unsharp(gray)),
        ("unsharp_clahe_1024", _apply_unsharp(clahe_large)),
        ("otsu_clahe", _apply_otsu(clahe)),
        ("otsu_clahe_1024", _apply_otsu(clahe_large)),
        ("adaptive_clahe_1024", _apply_adaptive_threshold(clahe_large)),
    ]
    return _dedupe_variants_named(variants)


def _preprocess_variants(image_bgr: np.ndarray, *, enhanced: bool = False) -> list[np.ndarray]:
    if enhanced:
        named = _preprocess_variants_enhanced_named(image_bgr)
    else:
        named = _preprocess_variants_basic_named(image_bgr)
    return [image for _, image in named]


def _preprocess_variants_named(
    image_bgr: np.ndarray,
    *,
    enhanced: bool = False,
) -> list[tuple[str, np.ndarray]]:
    if enhanced:
        return _preprocess_variants_enhanced_named(image_bgr)
    return _preprocess_variants_basic_named(image_bgr)


def _decode_barcode(
    image: np.ndarray,
    *,
    enhanced: bool = False,
    rotation: int = 0,
    trace: RecognizePipelineTrace | None = None,
) -> str | None:
    tier = "enhanced" if enhanced else "basic"
    detector = cv2.barcode.BarcodeDetector()
    for variant_name, candidate in _preprocess_variants_named(image, enhanced=enhanced):
        info, _, _ = detector.detectAndDecode(candidate)
        if trace is not None:
            candidate_height, candidate_width = candidate.shape[:2]
            trace.record_decode_attempt(
                tier=tier,
                rotation=rotation,
                variant=variant_name,
                result="ok" if info else "miss",
                barcode_content=info.strip() if info else None,
                candidate_shape=(candidate_width, candidate_height),
            )
        if info:
            return info.strip()
    return None


def _decode_barcode_pipeline(
    roi: np.ndarray,
    *,
    trace: RecognizePipelineTrace | None = None,
) -> str | None:
    for enhanced in (False, True):
        for rotation in BARCODE_DECODE_ROTATION_ANGLES:
            rotated = roi if rotation == 0 else _rotate_image(roi, float(rotation))
            content = _decode_barcode(
                rotated,
                enhanced=enhanced,
                rotation=rotation,
                trace=trace,
            )
            if content:
                return content
    return None


def _decode_barcode_with_deskew(roi: np.ndarray, mask_roi: np.ndarray) -> str | None:
    base = roi
    deskew_angle = _deskew_angle_from_mask(mask_roi)
    if deskew_angle is not None and abs(deskew_angle) >= 0.5:
        base = _rotate_image(roi, deskew_angle)
    return _decode_barcode_pipeline(base)


def _decode_qr_code(image: np.ndarray) -> str | None:
    detector = cv2.QRCodeDetector()
    for candidate in _preprocess_variants(image):
        data, _, _ = detector.detectAndDecode(candidate)
        if data:
            return data.strip()

        ok, decoded_info, _, _ = detector.detectAndDecodeMulti(candidate)
        if ok and decoded_info:
            for item in decoded_info:
                if item:
                    return item.strip()
    return None


def expand_bbox(
    bbox: list[int],
    image_shape: tuple[int, ...],
    *,
    padding_ratio: float,
) -> list[int]:
    if padding_ratio < 0:
        raise ValueError("padding_ratio 不能为负数")

    height, width = image_shape[:2]
    x1, y1, x2, y2 = map(int, bbox)
    box_width = max(1, x2 - x1)
    box_height = max(1, y2 - y1)
    pad_x = int(box_width * padding_ratio)
    pad_y = int(box_height * padding_ratio)
    return [
        max(0, x1 - pad_x),
        max(0, y1 - pad_y),
        min(width, x2 + pad_x),
        min(height, y2 + pad_y),
    ]


def decode_barcode_from_bbox(
    image_bgr: np.ndarray,
    bbox: list[int],
    *,
    padding_ratio: float = 0.0,
    trace: RecognizePipelineTrace | None = None,
) -> str | None:
    if len(bbox) != 4:
        raise ValueError("bbox 必须是 [x1, y1, x2, y2]")

    expanded_bbox = (
        expand_bbox(bbox, image_bgr.shape, padding_ratio=padding_ratio)
        if padding_ratio > 0
        else bbox
    )
    height, width = image_bgr.shape[:2]
    x1, y1, x2, y2 = map(int, expanded_bbox)
    x1 = max(0, min(x1, width))
    x2 = max(0, min(x2, width))
    y1 = max(0, min(y1, height))
    y2 = max(0, min(y2, height))
    if x2 <= x1 or y2 <= y1:
        raise ValueError("bbox 无效")

    crop = image_bgr[y1:y2, x1:x2].copy()
    if trace is not None:
        crop_height, crop_width = crop.shape[:2]
        trace.expanded_bbox = list(expanded_bbox)
        trace.crop_shape = (crop_width, crop_height)

    return _decode_barcode_pipeline(crop, trace=trace)


def decode_barcode_from_region(
    image_bgr: np.ndarray,
    bbox: list[int],
    mask_b64: str,
) -> str | None:
    roi, mask_roi = _extract_roi_and_mask(image_bgr, bbox, mask_b64)
    return _decode_barcode_with_deskew(roi, mask_roi)


def parse_qr_code(
    image_bgr: np.ndarray,
    *,
    target_type: TargetType,
    bbox: list[int],
    mask_b64: str,
) -> str | None:
    roi, mask_roi = _extract_roi_and_mask(image_bgr, bbox, mask_b64)
    if target_type == "sku":
        content = _decode_qr_code(roi)
        if content is None:
            content = _decode_barcode_with_deskew(roi, mask_roi)
        return content

    content = _decode_barcode_with_deskew(roi, mask_roi)
    if content is None:
        content = _decode_qr_code(roi)
    return content
