"""Collect recognize_sku_barcode pipeline steps and render a single summary log."""

from __future__ import annotations

from dataclasses import dataclass, field


@dataclass
class Sam3CandidateTrace:
    index: int
    score: float
    bbox: list[int]
    center_dist: float
    pass_filter: bool


@dataclass
class DecodeAttemptTrace:
    tier: str
    rotation: int
    variant: str
    result: str
    barcode_content: str | None = None
    candidate_shape: tuple[int, int] | None = None


@dataclass
class RecognizePipelineTrace:
    sku_id: str = ""
    name: str = ""
    image_path: str = ""
    image_source: str = "path"
    sam3_prompt: str = ""
    sam3_threshold: float = 0.5
    image_shape: tuple[int, int] | None = None
    center_distance_max: float = 0.0
    bbox_padding_ratio: float = 0.0
    sam3_candidates: list[Sam3CandidateTrace] = field(default_factory=list)
    filtered_count: int = 0
    selected_index: int | None = None
    selected_score: float | None = None
    selected_bbox: list[int] | None = None
    selected_center_dist: float | None = None
    expanded_bbox: list[int] | None = None
    crop_shape: tuple[int, int] | None = None
    decode_attempts: list[DecodeAttemptTrace] = field(default_factory=list)
    decode_error: str | None = None
    saved_image_path: str | None = None
    barcode_content: str | None = None
    status: str = ""
    failure_reason: str | None = None
    timings_ms: dict[str, float] = field(default_factory=dict)
    total_ms: float = 0.0

    def record_sam3_candidates(
        self,
        instances: list[object],
        image_shape: tuple[int, ...],
        *,
        center_distance_max: float,
        center_distance_fn,
    ) -> None:
        self.sam3_candidates = []
        for index, instance in enumerate(instances, start=1):
            center_dist = center_distance_fn(instance.bbox, image_shape)
            self.sam3_candidates.append(
                Sam3CandidateTrace(
                    index=index,
                    score=float(instance.score),
                    bbox=list(instance.bbox),
                    center_dist=center_dist,
                    pass_filter=center_dist <= center_distance_max,
                )
            )

    def record_decode_attempt(
        self,
        *,
        tier: str,
        rotation: int,
        variant: str,
        result: str,
        barcode_content: str | None = None,
        candidate_shape: tuple[int, int] | None = None,
    ) -> None:
        self.decode_attempts.append(
            DecodeAttemptTrace(
                tier=tier,
                rotation=rotation,
                variant=variant,
                result=result,
                barcode_content=barcode_content,
                candidate_shape=candidate_shape,
            )
        )

    def format_summary(self) -> str:
        lines: list[str] = [
            f"step=pipeline_summary status={self.status}",
            (
                f"request sku_id={self.sku_id!r} name={self.name!r} "
                f"image_source={self.image_source!r} "
                f"image_path={self.image_path!r} sam3_prompt={self.sam3_prompt!r} "
                f"sam3_threshold={self.sam3_threshold:.3f}"
            ),
        ]

        read_ms = self.timings_ms.get("read_image")
        if self.image_shape is not None:
            width, height = self.image_shape
            read_part = f"读图 ({read_ms}ms)" if read_ms is not None else "读图"
            lines.extend([read_part, f"  shape={width}x{height}", "  ↓"])

        sam3_ms = self.timings_ms.get("sam3")
        candidate_count = len(self.sam3_candidates)
        sam3_part = f"SAM3 定位 ({sam3_ms}ms)" if sam3_ms is not None else "SAM3 定位"
        lines.append(f"{sam3_part} → {candidate_count} 候选")
        for candidate in self.sam3_candidates:
            verdict = "pass" if candidate.pass_filter else "reject"
            lines.append(
                f"  #{candidate.index} score={candidate.score:.4f} bbox={candidate.bbox} "
                f"center_dist={candidate.center_dist:.3f} {verdict}"
            )

        if candidate_count == 0:
            lines.extend(["  ↓", f"NOT_FOUND: SAM3 无候选 (total_ms={self.total_ms})"])
            return "\n".join(lines)

        lines.append("  ↓")
        if self.filtered_count == 0:
            lines.append(
                f"中心过滤 ({self.center_distance_max:.2f}) → 0 个通过 (strict_mode)"
            )
            lines.extend(["  ↓", f"NOT_FOUND: 无候选通过中心过滤 (total_ms={self.total_ms})"])
            return "\n".join(lines)

        selected_label = f"#{self.selected_index}" if self.selected_index is not None else "?"
        selected_dist = (
            f"{self.selected_center_dist:.3f}"
            if self.selected_center_dist is not None
            else "?"
        )
        lines.append(
            f"中心过滤 ({self.center_distance_max:.2f}) → 剩 {self.filtered_count} 个 "
            f"({selected_label}, dist={selected_dist})"
        )

        if self.selected_bbox is None:
            lines.extend(["  ↓", f"NOT_FOUND: 未选中候选 (total_ms={self.total_ms})"])
            return "\n".join(lines)

        lines.append("  ↓")
        crop_label = (
            f"{self.crop_shape[0]}x{self.crop_shape[1]}"
            if self.crop_shape is not None
            else "?"
        )
        lines.append(
            f"扩边 {self.bbox_padding_ratio:.0%} 裁 ROI ({crop_label}) "
            f"bbox={self.selected_bbox} expanded_bbox={self.expanded_bbox}"
        )

        decode_ms = self.timings_ms.get("decode")
        lines.append("  ↓")
        decode_header = (
            f"OpenCV 解码 ({decode_ms}ms)" if decode_ms is not None else "OpenCV 解码"
        )
        lines.append(decode_header)

        if self.decode_error is not None:
            lines.append(f"  error={self.decode_error}")
        elif not self.decode_attempts:
            lines.append("  (无 decode 尝试记录)")
        else:
            winning = next(
                (item for item in self.decode_attempts if item.result == "ok"),
                None,
            )
            if winning is not None:
                lines.append(
                    f"  {winning.tier} / rotation={winning.rotation} / "
                    f"variant={winning.variant} → 成功 {winning.barcode_content!r}"
                )
                miss_count = sum(1 for item in self.decode_attempts if item.result != "ok")
                if miss_count:
                    lines.append(f"  (此前尝试 {miss_count} 次未命中)")
            else:
                for attempt in self.decode_attempts:
                    shape_label = (
                        f" shape={attempt.candidate_shape[0]}x{attempt.candidate_shape[1]}"
                        if attempt.candidate_shape is not None
                        else ""
                    )
                    lines.append(
                        f"  {attempt.tier} / rotation={attempt.rotation} / "
                        f"variant={attempt.variant} → miss{shape_label}"
                    )

        lines.append("  ↓")
        if self.status == "FOUND" and self.barcode_content is not None:
            lines.append(
                f"FOUND: {self.barcode_content} (total_ms={self.total_ms}, "
                f"timings_ms={self.timings_ms})"
            )
        else:
            reason = self.failure_reason or "decode_failed"
            not_found_line = (
                f"NOT_FOUND: {reason} (total_ms={self.total_ms}, "
                f"timings_ms={self.timings_ms})"
            )
            if self.saved_image_path is not None:
                not_found_line += f", saved_image={self.saved_image_path!r}"
            lines.append(not_found_line)

        return "\n".join(lines)
