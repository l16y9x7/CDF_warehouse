"""Render bbox + mask overlays for supervision-shelf detection results."""

from __future__ import annotations

import base64
from pathlib import Path

import cv2
import numpy as np


def decode_mask_b64(mask_b64: str) -> np.ndarray | None:
    payload = mask_b64.strip()
    if payload.startswith("data:"):
        payload = payload.split(",", 1)[-1]
    mask_bytes = base64.b64decode(payload)
    return cv2.imdecode(np.frombuffer(mask_bytes, dtype=np.uint8), cv2.IMREAD_GRAYSCALE)


def render_blue_light_overlay(
    image_bgr: np.ndarray,
    *,
    bbox: list[int] | None,
    mask_b64: str | None,
    label: str = "blue light",
) -> np.ndarray:
    """Draw semi-transparent mask and bbox on a copy of the source image."""
    output = image_bgr.copy()

    if mask_b64:
        mask = decode_mask_b64(mask_b64)
        if mask is not None:
            if mask.shape[:2] != output.shape[:2]:
                mask = cv2.resize(
                    mask,
                    (output.shape[1], output.shape[0]),
                    interpolation=cv2.INTER_NEAREST,
                )
            tint = output.copy()
            tint[mask > 0] = (255, 128, 0)
            output = cv2.addWeighted(output, 0.55, tint, 0.45, 0)
            contours, _ = cv2.findContours(
                (mask > 0).astype(np.uint8),
                cv2.RETR_EXTERNAL,
                cv2.CHAIN_APPROX_SIMPLE,
            )
            cv2.drawContours(output, contours, -1, (0, 255, 255), 2)

    if isinstance(bbox, list) and len(bbox) == 4:
        x1, y1, x2, y2 = map(int, bbox)
        cv2.rectangle(output, (x1, y1), (x2, y2), (0, 0, 255), 2)
        cv2.putText(
            output,
            label,
            (x1, max(24, y1 - 10)),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.7,
            (0, 0, 255),
            2,
            cv2.LINE_AA,
        )

    legend = "mask: cyan fill + yellow contour | bbox: red"
    cv2.putText(
        output,
        legend,
        (12, output.shape[0] - 12),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.55,
        (255, 255, 255),
        2,
        cv2.LINE_AA,
    )
    cv2.putText(
        output,
        legend,
        (12, output.shape[0] - 12),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.55,
        (0, 0, 0),
        1,
        cv2.LINE_AA,
    )
    return output


def save_sam_locate_overlay(
    source_path: Path,
    result: dict,
    output_path: Path | None = None,
    *,
    label: str = "target",
    suffix: str = "result",
) -> Path:
    image = cv2.imread(str(source_path), cv2.IMREAD_COLOR)
    if image is None:
        raise ValueError(f"无法读取图片: {source_path}")

    overlay = render_blue_light_overlay(
        image,
        bbox=result.get("bbox"),
        mask_b64=result.get("mask"),
        label=label,
    )
    out_path = output_path or source_path.with_name(f"{source_path.stem}_{suffix}.jpg")
    out_path.parent.mkdir(parents=True, exist_ok=True)
    if not cv2.imwrite(str(out_path), overlay):
        raise RuntimeError(f"写入失败: {out_path}")
    return out_path


def save_blue_light_overlay(
    source_path: Path,
    result: dict,
    output_path: Path | None = None,
) -> Path:
    return save_sam_locate_overlay(
        source_path,
        result,
        output_path,
        label="blue light",
        suffix="blue_light_result",
    )


def save_basket_overlay(
    source_path: Path,
    result: dict,
    output_path: Path | None = None,
) -> Path:
    return save_sam_locate_overlay(
        source_path,
        result,
        output_path,
        label="basket",
        suffix="basket_result",
    )


def save_carton_qr_code_overlay(
    source_path: Path,
    result: dict,
    output_path: Path | None = None,
) -> Path:
    return save_sam_locate_overlay(
        source_path,
        result,
        output_path,
        label="carton QR code",
        suffix="carton_qr_code_result",
    )


def save_sku_qr_code_overlay(
    source_path: Path,
    result: dict,
    output_path: Path | None = None,
) -> Path:
    return save_sam_locate_overlay(
        source_path,
        result,
        output_path,
        label="SKU QR code",
        suffix="sku_qr_code_result",
    )


def save_carton_item_overlay(
    source_path: Path,
    result: dict,
    output_path: Path | None = None,
) -> Path:
    return save_sam_locate_overlay(
        source_path,
        result,
        output_path,
        label="carton item",
        suffix="carton_item_result",
    )


def save_basket_item_overlay(
    source_path: Path,
    result: dict,
    output_path: Path | None = None,
) -> Path:
    return save_sam_locate_overlay(
        source_path,
        result,
        output_path,
        label="basket item",
        suffix="basket_item_result",
    )


def save_basket_button_overlay(
    source_path: Path,
    result: dict,
    output_path: Path | None = None,
) -> Path:
    return save_sam_locate_overlay(
        source_path,
        result,
        output_path,
        label="basket button",
        suffix="basket_button_result",
    )
