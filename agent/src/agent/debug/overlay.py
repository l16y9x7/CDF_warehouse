from __future__ import annotations

from collections.abc import Mapping, Sequence
from typing import Any

# 各类正式输出。box/tube 的抓取点不叫 reference_point，前挡板点在商品点无效时仍要画出来。
_POINT_SPECS = (
    ("reference_point_camera_mm", "参考点", "#22d3ee"),
    ("top_point_camera_mm", "顶面点", "#22d3ee"),
    ("top_edge_center_camera_mm", "上沿中点", "#22d3ee"),
    ("front_panel_top_edge_midpoint_camera_mm", "前挡板上沿", "#a78bfa"),
    ("model_center_camera_mm", "篮筐中心", "#22d3ee"),
)
_PRIMARY_POINT_KEYS = frozenset(
    {
        "reference_point_camera_mm",
        "top_point_camera_mm",
        "top_edge_center_camera_mm",
    }
)
_ESTIMATION_OPERATIONS = frozenset({"estimate_pick_pose", "estimate_basket_pose"})


def overlay_from_pose(
    payload: Any,
    *,
    k: Sequence[Sequence[float]] | None = None,
    skill: str | None = None,
) -> dict[str, Any] | None:
    """Compact 2D overlay spec from an /infer payload or serialized result."""
    sources = _sources(payload)
    if not sources:
        return None
    matrix = _intrinsics(sources, k)
    points = _points(sources)
    lines = _lines(sources)
    boxes = _boxes(sources)
    markers = _markers(sources)
    hud = _hud(sources)
    if not (points or lines or boxes or markers or hud):
        return None
    overlay: dict[str, Any] = {"hud": hud, "points": points, "lines": lines, "boxes": boxes, "markers": markers}
    if matrix is not None:
        overlay["K"] = matrix
    if skill:
        overlay["skill"] = skill
    request_id = _first(sources, "request_id")
    if isinstance(request_id, str) and request_id:
        overlay["request_id"] = request_id
    return overlay


def overlays_from_run(
    run: Mapping[str, Any] | None,
    spans: Sequence[Mapping[str, Any]] | None = None,
) -> list[dict[str, Any]]:
    """Collect overlay specs from a debug run result and estimation spans."""
    found: list[dict[str, Any]] = []
    seen: set[tuple[Any, ...]] = set()
    for span in spans or ():
        if not _is_estimation_span(span):
            continue
        overlay = overlay_from_pose(
            span.get("output"),
            k=_span_intrinsics(span),
            skill=_span_skill(span, spans or ()),
        )
        _append_unique(found, seen, overlay)
    overlay = overlay_from_pose(
        (run or {}).get("result"),
        k=_request_intrinsics((run or {}).get("request")),
    )
    _append_unique(found, seen, overlay)
    return found


def attach_overlays(
    media: list[dict[str, Any]], overlays: Sequence[Mapping[str, Any]]
) -> list[dict[str, Any]]:
    """Attach each overlay to matching color/depth frames without mutating inputs."""
    if not media:
        return media
    attached = [dict(item) for item in media]
    remaining = list(overlays)
    for overlay in overlays:
        skill = overlay.get("skill")
        matches = [item for item in attached if skill and item.get("skill") == skill]
        if not matches:
            continue
        capture_id = next((item.get("capture_id") for item in matches if item.get("capture_id")), None)
        for item in attached:
            if capture_id is not None and item.get("capture_id") != capture_id:
                continue
            if capture_id is None and item not in matches:
                continue
            item["overlay"] = _with_fallback_k(overlay, item)
        remaining = [item for item in remaining if item is not overlay]
    if remaining:
        latest = next(
            (item.get("capture_id") for item in reversed(attached) if item.get("capture_id")),
            None,
        )
        for item in attached:
            if latest is not None and item.get("capture_id") != latest:
                continue
            if "overlay" not in item:
                item["overlay"] = _with_fallback_k(remaining[-1], item)
    return attached


def _append_unique(
    found: list[dict[str, Any]], seen: set[tuple[Any, ...]], overlay: dict[str, Any] | None
) -> None:
    if overlay is None:
        return
    key = (
        overlay.get("request_id"),
        overlay.get("skill"),
        tuple(overlay.get("hud") or ()),
        len(overlay.get("points") or ()),
    )
    if key in seen:
        return
    seen.add(key)
    found.append(overlay)


def _with_fallback_k(overlay: Mapping[str, Any], item: Mapping[str, Any]) -> dict[str, Any]:
    result = dict(overlay)
    if result.get("K") is None and item.get("K") is not None:
        result["K"] = item["K"]
    return result


def _sources(payload: Any) -> list[dict[str, Any]]:
    if not isinstance(payload, Mapping):
        return []
    sources = [dict(payload)]
    for key in ("localization_result", "raw"):
        nested = payload.get(key)
        if isinstance(nested, Mapping) and nested not in sources:
            sources.append(dict(nested))
    return sources


def _first(sources: Sequence[Mapping[str, Any]], key: str) -> Any:
    for source in sources:
        value = source.get(key)
        if value is not None:
            return value
    return None


def _intrinsics(
    sources: Sequence[Mapping[str, Any]], fallback: Sequence[Sequence[float]] | None
) -> list[list[float]] | None:
    for source in sources:
        matrix = _as_intrinsics(source.get("K"))
        if matrix is not None:
            return matrix
        summary = source.get("input_summary")
        if isinstance(summary, Mapping):
            matrix = _as_intrinsics(summary.get("K"))
            if matrix is not None:
                return matrix
    return _as_intrinsics(fallback)


def _as_intrinsics(value: Any) -> list[list[float]] | None:
    if isinstance(value, Mapping):
        try:
            fx, fy, cx, cy = (float(value[key]) for key in ("fx", "fy", "cx", "cy"))
        except (KeyError, TypeError, ValueError):
            return None
        return [[fx, 0.0, cx], [0.0, fy, cy], [0.0, 0.0, 1.0]]
    if not isinstance(value, Sequence) or isinstance(value, (str, bytes)):
        return None
    if len(value) == 9:
        try:
            numbers = [float(item) for item in value]
        except (TypeError, ValueError):
            return None
        return [numbers[0:3], numbers[3:6], numbers[6:9]]
    if len(value) != 3:
        return None
    rows: list[list[float]] = []
    for row in value:
        if not isinstance(row, Sequence) or isinstance(row, (str, bytes)) or len(row) != 3:
            return None
        try:
            rows.append([float(item) for item in row])
        except (TypeError, ValueError):
            return None
    return rows


def _vec3(value: Any) -> list[float] | None:
    if not isinstance(value, Sequence) or isinstance(value, (str, bytes)) or len(value) != 3:
        return None
    try:
        numbers = [float(item) for item in value]
    except (TypeError, ValueError):
        return None
    return numbers if all(number == number for number in numbers) else None


def _points(sources: Sequence[Mapping[str, Any]]) -> list[dict[str, Any]]:
    points: list[dict[str, Any]] = []
    seen: set[str] = set()

    # 获取状态信息用于颜色编码
    ok = _first(sources, "ok")
    score = _first(sources, "sam3_score")

    for key, label, base_color in _POINT_SPECS:
        xyz = _vec3(_first(sources, key))
        if xyz is None or key in seen:
            continue
        seen.add(key)

        if key in _PRIMARY_POINT_KEYS:
            if ok is False:
                color = "#ef4444"
            elif ok is True and isinstance(score, (int, float)) and score <= 0.7:
                color = "#fb923c"
            elif ok is True:
                color = "#22c55e"
            else:
                color = base_color
            points.append({"name": key, "label": label, "xyz_mm": xyz, "color": color, "size": "large"})
        else:
            points.append({"name": key, "label": label, "xyz_mm": xyz, "color": base_color})

    return points


def _lines(sources: Sequence[Mapping[str, Any]]) -> list[dict[str, Any]]:
    lines: list[dict[str, Any]] = []

    # 抓取轴线：从axis_point或reference_point沿axis_direction绘制
    origin = _vec3(_first(sources, "axis_point_camera_mm")) or _vec3(
        _first(sources, "reference_point_camera_mm")
    )
    direction = _vec3(_first(sources, "axis_direction_camera_up"))
    if origin is not None and direction is not None:
        span = 120.0  # 增加长度到120mm
        lines.append(
            {
                "label": "轴线",
                "color": "#fbbf24",
                "width": 3,  # 加粗线条
                "xyz_mm": [
                    [origin[i] - direction[i] * span for i in range(3)],
                    [origin[i] + direction[i] * span for i in range(3)],
                ],
            }
        )

    endpoints = _first(sources, "top_edge_endpoints_camera_mm")
    if isinstance(endpoints, Sequence) and not isinstance(endpoints, (str, bytes)) and len(endpoints) >= 2:
        start, end = _vec3(endpoints[0]), _vec3(endpoints[1])
        if start is not None and end is not None:
            lines.append({"label": "上沿", "color": "#fbbf24", "width": 3, "xyz_mm": [start, end]})

    # 篮筐位姿：CAD坐标系XYZ轴
    pose = _first(sources, "pose_4x4")
    origin = _pose_translation(pose)
    rotation = _pose_rotation(pose)
    if origin is not None and rotation is not None:
        axis_colors = (("#ef4444", "X"), ("#22c55e", "Y"), ("#3b82f6", "Z"))
        for index, (color, label) in enumerate(axis_colors):
            axis = rotation[index]
            lines.append(
                {
                    "label": f"CAD {label}",
                    "color": color,
                    "width": 2,
                    "xyz_mm": [origin, [origin[i] + axis[i] * 120.0 for i in range(3)]],
                }
            )
    return lines


def _boxes(sources: Sequence[Mapping[str, Any]]) -> list[dict[str, Any]]:
    """不显示任何检测框，这些是中间步骤，对验证位姿准确性无用"""
    return []


def _markers(sources: Sequence[Mapping[str, Any]]) -> list[dict[str, Any]]:
    """只显示选中的实例，不显示所有候选"""
    markers: list[dict[str, Any]] = []
    selected_instance_id = _first(sources, "selected_instance_id")

    if selected_instance_id is None:
        return markers

    selection = _first(sources, "box_selection")
    if not isinstance(selection, Mapping):
        return markers

    members = selection.get("object_membership")
    if not isinstance(members, Sequence):
        return markers

    # 只显示selected_instance_id对应的实例
    for member in members:
        if not isinstance(member, Mapping):
            continue
        if member.get("upstream_instance_id") != selected_instance_id:
            continue
        xy = member.get("mask_centroid_xy")
        if not isinstance(xy, Sequence) or len(xy) != 2:
            continue
        try:
            markers.append(
                {
                    "xy": [float(xy[0]), float(xy[1])],
                    "label": f"实例 {selected_instance_id}",
                    "color": "#facc15",
                }
            )
            break  # 只显示一个
        except (TypeError, ValueError):
            continue

    return markers


def _hud(sources: Sequence[Mapping[str, Any]]) -> list[str]:
    """精简HUD显示，只保留关键信息"""
    lines: list[str] = []

    # 第一行：状态 · 目标类型 · SKU类型
    ok = _first(sources, "ok")
    target = _first(sources, "target_type")
    sku = _first(sources, "sku_typ")

    status_text = "ok" if ok is True else "失败" if ok is False else None
    parts = [part for part in (status_text, str(target) if target else None, str(sku) if sku else None) if part]

    # 添加score信息到第一行
    score = _first(sources, "sam3_score")
    if isinstance(score, (int, float)):
        parts.append(f"score {score:.3f}")

    if parts:
        lines.append(" · ".join(parts))

    for key, label, _color in _POINT_SPECS:
        if key not in _PRIMARY_POINT_KEYS and key != "front_panel_top_edge_midpoint_camera_mm":
            continue
        point = _vec3(_first(sources, key))
        if point is None:
            continue
        lines.append(label + " " + ", ".join(f"{value:.1f}" for value in point) + " mm")

    # 第三行：拒绝原因（如果有）
    reasons = _first(sources, "rejection_reasons")
    if isinstance(reasons, Sequence) and reasons and not isinstance(reasons, (str, bytes)):
        lines.append("拒绝: " + ", ".join(str(item) for item in reasons[:3]))

    return lines


def _pose_translation(matrix: Any) -> list[float] | None:
    if not isinstance(matrix, Sequence) or isinstance(matrix, (str, bytes)) or len(matrix) < 3:
        return None
    try:
        return [float(matrix[row][3]) for row in range(3)]
    except (IndexError, TypeError, ValueError):
        return None


def _pose_rotation(matrix: Any) -> list[list[float]] | None:
    if not isinstance(matrix, Sequence) or isinstance(matrix, (str, bytes)) or len(matrix) < 3:
        return None
    axes: list[list[float]] = []
    for column in range(3):
        try:
            axes.append([float(matrix[row][column]) for row in range(3)])
        except (IndexError, TypeError, ValueError):
            return None
    return axes


def _is_estimation_span(span: Mapping[str, Any]) -> bool:
    return span.get("kind") == "capability" and span.get("operation") in _ESTIMATION_OPERATIONS


def _span_intrinsics(span: Mapping[str, Any]) -> list[list[float]] | None:
    payload = span.get("input")
    if isinstance(payload, Mapping):
        request = payload.get("request")
        if isinstance(request, Mapping):
            return _as_intrinsics(request.get("K"))
        return _as_intrinsics(payload.get("K"))
    return None


def _span_skill(span: Mapping[str, Any], spans: Sequence[Mapping[str, Any]]) -> str | None:
    parent_id = span.get("parent_span_id")
    by_id = {item.get("span_id"): item for item in spans if item.get("span_id")}
    while parent_id and parent_id in by_id:
        parent = by_id[parent_id]
        if parent.get("kind") == "skill" and isinstance(parent.get("name"), str):
            return parent["name"]
        parent_id = parent.get("parent_span_id")
    return None


def _request_intrinsics(request: Any) -> list[list[float]] | None:
    if isinstance(request, Mapping):
        payload = request.get("payload")
        if isinstance(payload, Mapping):
            return _as_intrinsics(payload.get("K"))
        return _as_intrinsics(request.get("K"))
    return None
