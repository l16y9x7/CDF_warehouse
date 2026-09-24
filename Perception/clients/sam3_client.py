"""SAM3 HTTP client and text-prompt instance localization."""

from __future__ import annotations

import base64
import binascii
import io
import os
import time
from dataclasses import dataclass, field
from typing import Any, Literal, Sequence

import cv2
import numpy as np
import requests
from PIL import Image

from config import SAM3_URL


SAM3_TIMEOUT_SECONDS = 120
Sam3Backend = Literal["segment", "infer"]
_EMPTY_INFER_PAYLOAD: dict[str, Any] = {"ok": True, "detections": []}
_EMPTY_SEGMENT_PAYLOAD: dict[str, Any] = {"instances": []}


def _safe_response_json(response: requests.Response) -> dict[str, Any] | None:
    try:
        payload = response.json()
    except ValueError:
        return None
    return payload if isinstance(payload, dict) else None


def _sam_error_indicates_no_detections(message: object) -> bool:
    if not isinstance(message, str):
        return False
    normalized = message.strip().lower()
    return "no valid detections" in normalized or "no detections" in normalized


def _sam_response_indicates_no_detections(response: requests.Response) -> bool:
    payload = _safe_response_json(response)
    if payload is not None:
        for key in ("error", "detail", "message"):
            if _sam_error_indicates_no_detections(payload.get(key)):
                return True
    return _sam_error_indicates_no_detections(response.text)


def _raise_for_sam_status(response: requests.Response) -> None:
    if response.status_code in {422, 404} and _sam_response_indicates_no_detections(response):
        return
    response.raise_for_status()


def sam_instance_bbox(instance: dict[str, Any]) -> list[Any] | None:
    for key in ("bbox_xyxy", "bbox"):
        value = instance.get(key)
        if isinstance(value, list) and len(value) == 4:
            return value
    return None


def sam_instance_bbox_xyxy(instance: dict[str, Any]) -> list[int] | None:
    bbox = sam_instance_bbox(instance)
    if bbox is None:
        return None
    normalized: list[int] = []
    for value in bbox:
        if not isinstance(value, (int, float)) or isinstance(value, bool):
            return None
        normalized.append(int(round(float(value))))
    if normalized[2] <= normalized[0] or normalized[3] <= normalized[1]:
        return None
    return normalized


def sam_instance_mask(instance: dict[str, Any]) -> str | None:
    for key in ("mask_png_base64", "mask"):
        value = instance.get(key)
        if isinstance(value, str) and value.strip():
            return value
    return None


def sam3_backend(url: str | None = None) -> Sam3Backend:
    override = os.getenv("SAM3_BACKEND", "").strip().lower()
    if override in {"infer", "segment"}:
        return override  # type: ignore[return-value]

    resolved_url = SAM3_URL if url is None else url
    normalized = resolved_url.rstrip("/").lower()
    if normalized.endswith("/infer"):
        return "infer"
    return "segment"


def _decode_mask_png_base64(
    encoded_mask: str,
    target_shape: tuple[int, int],
) -> np.ndarray | None:
    try:
        raw = base64.b64decode(encoded_mask.split(",", 1)[-1], validate=True)
        mask_u8 = cv2.imdecode(np.frombuffer(raw, dtype=np.uint8), cv2.IMREAD_GRAYSCALE)
    except (ValueError, binascii.Error, cv2.error):
        return None
    if mask_u8 is None:
        return None

    height, width = target_shape
    if mask_u8.shape != (height, width):
        mask_u8 = cv2.resize(mask_u8, (width, height), interpolation=cv2.INTER_NEAREST)
    return mask_u8 >= 128


def _bbox_xywh_to_xyxy(bbox: Sequence[int | float]) -> list[int]:
    x, y, width, height = (int(bbox[0]), int(bbox[1]), int(bbox[2]), int(bbox[3]))
    return [x, y, x + max(width, 1), y + max(height, 1)]


def _decode_coco_rle_mask(rle: dict[str, Any]) -> np.ndarray:
    try:
        from pycocotools import mask as mask_util
    except ImportError as error:
        raise ValueError(
            "SAM3 infer 响应的 RLE mask 需要安装 pycocotools"
        ) from error

    counts = rle.get("counts")
    size = rle.get("size")
    if not isinstance(size, list) or len(size) != 2:
        raise ValueError("SAM3 RLE mask 缺少有效 size")

    payload = {"size": [int(size[0]), int(size[1])], "counts": counts}
    if isinstance(counts, str):
        payload["counts"] = counts.encode("ascii")
    decoded = mask_util.decode(payload)
    return np.asarray(decoded, dtype=bool)


@dataclass(frozen=True)
class SamInstance:
    bbox_xyxy: list[int]
    mask: np.ndarray
    score: float
    prompt: str


@dataclass(frozen=True)
class SamLocateConfig:
    sam3_prompt: str
    sam3_threshold: float = 0.5
    sam3_mask_threshold: float = 0.5
    minimum_instance_area: int = 16
    merge_iou_threshold: float = 0.35


@dataclass(frozen=True)
class SamLocateResult:
    bbox: list[int]
    mask: str
    confidence: float
    score: float
    timings_ms: dict[str, float] = field(default_factory=dict)


def _sam_segment_multipart(
    image_bgr: np.ndarray,
    prompt: str,
    *,
    threshold: float,
    mask_threshold: float,
    timeout: float,
) -> dict[str, Any]:
    ok, encoded = cv2.imencode(".jpg", image_bgr, [cv2.IMWRITE_JPEG_QUALITY, 95])
    if not ok:
        raise ValueError("SAM3 输入图编码失败")

    response = requests.post(
        SAM3_URL,
        files={"image": ("snapshot.jpg", encoded.tobytes(), "image/jpeg")},
        data={
            "prompt": prompt,
            "threshold": threshold,
            "mask_threshold": mask_threshold,
        },
        timeout=timeout,
    )
    if response.status_code in {422, 404} and _sam_response_indicates_no_detections(response):
        return dict(_EMPTY_SEGMENT_PAYLOAD)
    _raise_for_sam_status(response)
    payload = response.json()
    if not isinstance(payload, dict):
        raise ValueError("SAM3 响应不是有效 JSON")
    if payload.get("ok") is False:
        if _sam_error_indicates_no_detections(payload.get("error")):
            return dict(_EMPTY_SEGMENT_PAYLOAD)
        raise ValueError(str(payload.get("error") or "SAM3 segment 失败"))
    instances = payload.get("instances")
    if instances is None:
        num_instances = payload.get("num_instances")
        if isinstance(num_instances, int) and num_instances == 0:
            payload = dict(payload)
            payload["instances"] = []
            return payload
        raise ValueError("SAM3 响应缺少 instances 数组")
    if not isinstance(instances, list):
        raise ValueError("SAM3 响应缺少 instances 数组")
    return payload


def _sam_segment_infer(
    image_bgr: np.ndarray,
    prompt: str,
    *,
    threshold: float,
    mask_threshold: float,
    timeout: float,
) -> dict[str, Any]:
    ok, encoded = cv2.imencode(".jpg", image_bgr, [cv2.IMWRITE_JPEG_QUALITY, 95])
    if not ok:
        raise ValueError("SAM3 输入图编码失败")

    response = requests.post(
        SAM3_URL,
        json={
            "image_base64": base64.b64encode(encoded.tobytes()).decode("ascii"),
            "prompt": prompt,
            "threshold": threshold,
            "mask_threshold": mask_threshold,
        },
        timeout=timeout,
    )
    if response.status_code in {422, 404} and _sam_response_indicates_no_detections(response):
        return dict(_EMPTY_INFER_PAYLOAD)
    _raise_for_sam_status(response)
    payload = response.json()
    if not isinstance(payload, dict):
        raise ValueError("SAM3 响应不是有效 JSON")
    if payload.get("ok") is False:
        if _sam_error_indicates_no_detections(payload.get("error")):
            return dict(_EMPTY_INFER_PAYLOAD)
        raise ValueError(str(payload.get("error") or "SAM3 infer 失败"))
    if not isinstance(payload.get("detections"), list):
        raise ValueError("SAM3 infer 响应缺少 detections 数组")
    return payload


def sam_segment(
    image_bgr: np.ndarray,
    prompt: str,
    *,
    threshold: float = 0.5,
    mask_threshold: float = 0.5,
    timeout: float = SAM3_TIMEOUT_SECONDS,
) -> dict[str, Any]:
    if sam3_backend() == "infer":
        return _sam_segment_infer(
            image_bgr,
            prompt,
            threshold=threshold,
            mask_threshold=mask_threshold,
            timeout=timeout,
        )
    return _sam_segment_multipart(
        image_bgr,
        prompt,
        threshold=threshold,
        mask_threshold=mask_threshold,
        timeout=timeout,
    )


def _decode_infer_detections(
    payload: dict[str, Any],
    shape: tuple[int, int],
    *,
    prompt: str,
    minimum_area: int,
) -> list[SamInstance]:
    detections = payload.get("detections")
    if not isinstance(detections, list):
        raise ValueError("SAM3 infer 响应缺少 detections 数组")

    height, width = shape
    decoded: list[SamInstance] = []
    for detection in detections:
        if not isinstance(detection, dict):
            continue

        segmentation = detection.get("segmentation")
        if not isinstance(segmentation, dict):
            continue
        try:
            mask = _decode_coco_rle_mask(segmentation)
        except ValueError:
            continue
        if mask.shape != (height, width):
            mask = (
                cv2.resize(
                    mask.astype(np.uint8),
                    (width, height),
                    interpolation=cv2.INTER_NEAREST,
                )
                > 0
            )

        area = int(np.count_nonzero(mask))
        if area < minimum_area:
            continue

        bbox_raw = detection.get("bbox")
        if isinstance(bbox_raw, list) and len(bbox_raw) == 4:
            bbox = _bbox_xywh_to_xyxy(bbox_raw)
        else:
            ys, xs = np.nonzero(mask)
            bbox = [int(xs.min()), int(ys.min()), int(xs.max()) + 1, int(ys.max()) + 1]

        score_value = detection.get("score")
        score = (
            float(score_value)
            if isinstance(score_value, (int, float))
            and not isinstance(score_value, bool)
            else 0.0
        )
        decoded.append(
            SamInstance(
                bbox_xyxy=bbox,
                mask=mask,
                score=score,
                prompt=prompt,
            )
        )
    return decoded


def decode_sam_instances(
    payload: dict[str, Any],
    shape: tuple[int, int],
    *,
    prompt: str,
    minimum_area: int = 16,
) -> list[SamInstance]:
    if isinstance(payload.get("detections"), list):
        return _decode_infer_detections(
            payload,
            shape,
            prompt=prompt,
            minimum_area=minimum_area,
        )

    instances = payload.get("instances")
    if not isinstance(instances, list):
        raise ValueError("SAM3 响应缺少 instances 或 detections 数组")

    height, width = shape
    decoded: list[SamInstance] = []
    for instance in instances:
        if not isinstance(instance, dict):
            continue
        encoded_mask = sam_instance_mask(instance)
        if not encoded_mask:
            continue
        mask = _decode_mask_png_base64(encoded_mask, (height, width))
        if mask is None:
            continue

        area = int(np.count_nonzero(mask))
        if area < minimum_area:
            continue

        bbox = sam_instance_bbox_xyxy(instance)
        if bbox is None:
            ys, xs = np.nonzero(mask)
            bbox = [int(xs.min()), int(ys.min()), int(xs.max()) + 1, int(ys.max()) + 1]

        score_value = instance.get("score")
        score = (
            float(score_value)
            if isinstance(score_value, (int, float))
            and not isinstance(score_value, bool)
            else 0.0
        )
        decoded.append(
            SamInstance(
                bbox_xyxy=bbox,
                mask=mask,
                score=score,
                prompt=prompt,
            )
        )
    return decoded


def bbox_iou(first: Sequence[int], second: Sequence[int]) -> float:
    x1 = max(first[0], second[0])
    y1 = max(first[1], second[1])
    x2 = min(first[2], second[2])
    y2 = min(first[3], second[3])
    if x2 <= x1 or y2 <= y1:
        return 0.0
    intersection = (x2 - x1) * (y2 - y1)
    area_first = max(0, first[2] - first[0]) * max(0, first[3] - first[1])
    area_second = max(0, second[2] - second[0]) * max(0, second[3] - second[1])
    union = area_first + area_second - intersection
    if union <= 0:
        return 0.0
    return intersection / union


def merge_overlapping_instances(
    instances: list[SamInstance],
    *,
    iou_threshold: float,
) -> list[SamInstance]:
    ordered = sorted(instances, key=lambda item: item.score, reverse=True)
    kept: list[SamInstance] = []
    for candidate in ordered:
        if any(
            bbox_iou(candidate.bbox_xyxy, existing.bbox_xyxy) >= iou_threshold
            for existing in kept
        ):
            continue
        kept.append(candidate)
    return kept


def encode_mask_png_base64(mask: np.ndarray) -> str:
    mask_u8 = (np.asarray(mask) > 0).astype(np.uint8) * 255
    buffer = io.BytesIO()
    Image.fromarray(mask_u8, mode="L").save(buffer, format="PNG")
    return base64.b64encode(buffer.getvalue()).decode("ascii")


def _sam_instance_to_result(instance: SamInstance) -> SamLocateResult:
    score = round(instance.score, 4)
    return SamLocateResult(
        bbox=[int(v) for v in instance.bbox_xyxy],
        mask=encode_mask_png_base64(instance.mask),
        confidence=score,
        score=score,
    )


def locate_sam3_instances(
    image_bgr: np.ndarray,
    *,
    config: SamLocateConfig,
    top_k: int = 3,
    instance_ranker: Any | None = None,
    timings_ms: dict[str, float] | None = None,
    timing_key: str = "sam3",
) -> list[SamLocateResult]:
    if image_bgr.ndim != 3 or image_bgr.dtype != np.uint8:
        raise ValueError("输入必须是 uint8 BGR 彩色图")
    if top_k < 1:
        raise ValueError("top_k 必须 >= 1")

    timings = timings_ms if timings_ms is not None else {}
    sam_started = time.perf_counter()
    payload = sam_segment(
        image_bgr,
        config.sam3_prompt,
        threshold=config.sam3_threshold,
        mask_threshold=config.sam3_mask_threshold,
    )
    height, width = image_bgr.shape[:2]
    instances = decode_sam_instances(
        payload,
        (height, width),
        prompt=config.sam3_prompt,
        minimum_area=config.minimum_instance_area,
    )
    instances = merge_overlapping_instances(
        instances,
        iou_threshold=config.merge_iou_threshold,
    )
    timings[timing_key] = round((time.perf_counter() - sam_started) * 1000, 1)

    if not instances:
        return []

    ranker = instance_ranker or (lambda item: item.score)
    ordered = sorted(instances, key=ranker, reverse=True)
    return [_sam_instance_to_result(instance) for instance in ordered[:top_k]]


def locate_sam3_instance(
    image_bgr: np.ndarray,
    *,
    config: SamLocateConfig,
    timings_ms: dict[str, float] | None = None,
    timing_key: str = "sam3",
) -> SamLocateResult | None:
    results = locate_sam3_instances(
        image_bgr,
        config=config,
        top_k=1,
        timings_ms=timings_ms,
        timing_key=timing_key,
    )
    return results[0] if results else None
