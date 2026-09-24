"""把平台 method 转成场景 HTTP，payload 原样转发。"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any, Dict, Mapping


@dataclass(frozen=True)
class RouteSpec:
    path: str
    scenario: str = ""
    scenario_field: str = ""
    default: str = ""
    all_scenarios: bool = False


# 架构文档 §3.10。Gateway 加路由不算改能力合同。
BUILTIN_ROUTES: Dict[str, RouteSpec] = {
    "task_start": RouteSpec(
        path="/tasks", scenario_field="scenario"
    ),
    "StartTask": RouteSpec(
        path="/tasks", scenario_field="scenario"
    ),
    "start_material_task": RouteSpec(path="/tasks", scenario="wrc"),
    "play": RouteSpec(path="/tasks", scenario="piano"),
    "PlayPiece": RouteSpec(path="/tasks", scenario="piano"),
    "retail_start": RouteSpec(path="/tasks", scenario="retail"),
    "rokae_start": RouteSpec(path="/tasks", scenario="rokae"),
    "task_stop": RouteSpec(path="/tasks/stop", all_scenarios=True),
    "task_recovery": RouteSpec(path="/tasks/recover", scenario="smt"),
}


def load_routes(config: Mapping[str, Any]) -> Dict[str, RouteSpec]:
    routes = dict(BUILTIN_ROUTES)
    extra = config.get("routes") or {}
    if not isinstance(extra, Mapping):
        return routes
    for method, raw in extra.items():
        if not isinstance(raw, Mapping):
            continue
        name = str(method or "").strip()
        if not name:
            continue
        routes[name] = RouteSpec(
            path=str(raw.get("path") or "/tasks"),
            scenario=str(raw.get("scenario") or ""),
            scenario_field=str(raw.get("scenario_field") or ""),
            default=str(raw.get("default") or ""),
            all_scenarios=bool(raw.get("all_scenarios", False)),
        )
    return routes


def resolve_scenario(
    route: RouteSpec,
    data: Mapping[str, Any],
    *,
    default_scenario: str,
) -> str:
    if route.all_scenarios:
        return ""
    if route.scenario:
        return route.scenario
    if route.scenario_field:
        value = str(data.get(route.scenario_field) or "").strip()
        if value:
            return value
        return default_scenario or route.default
    return default_scenario


def enabled_scenarios(config: Mapping[str, Any]) -> Dict[str, str]:
    items = config.get("scenarios") or {}
    result: Dict[str, str] = {}
    if not isinstance(items, Mapping):
        return result
    for name, raw in items.items():
        if not isinstance(raw, Mapping):
            continue
        if raw.get("enabled") is False:
            continue
        url = str(raw.get("url") or "").strip()
        if not url:
            continue
        result[str(name)] = url.rstrip("/")
    return result
