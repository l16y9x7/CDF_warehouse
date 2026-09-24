"""OSD 状态源：配置里写成 URL 字符串即可。"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any, Dict, List, Mapping, Optional, Tuple


_REQUIRED_SOURCES = {"navigation"}


@dataclass(frozen=True)
class StateSource:
    name: str
    url: str
    state_path: str = "/state"
    health_path: str = "/health"
    required: bool = True
    urls: Tuple[str, ...] = ()

    def base_urls(self) -> List[str]:
        items: list[str] = []
        for raw in (self.url, *self.urls):
            url = str(raw or "").strip().rstrip("/")
            if url and url not in items:
                items.append(url)
        return items

    @property
    def state_url(self) -> str:
        return self.state_url_for(self.url)

    def state_url_for(self, base: str) -> str:
        return f"{str(base).rstrip('/')}{self.state_path}"

    @property
    def health_url(self) -> str:
        return self.health_url_for(self.url)

    def health_url_for(self, base: str) -> str:
        if not self.health_path:
            return ""
        return f"{str(base).rstrip('/')}{self.health_path}"


def iter_state_sources(config: Mapping[str, Any]) -> List[StateSource]:
    sources: list[StateSource] = []
    state_cfg = config.get("state") or {}
    if isinstance(state_cfg, Mapping):
        for name, raw in state_cfg.items():
            if str(name) in {"cache_ttl_sec", "skip_backoff_sec"}:
                continue
            source = _parse_source(str(name), raw)
            if source is not None:
                sources.append(source)
    scenarios = config.get("scenarios") or {}
    if isinstance(scenarios, Mapping):
        for name, raw in scenarios.items():
            if not isinstance(raw, Mapping) or raw.get("enabled") is False:
                continue
            # 默认只做命令路由，不轮询场景 /state（避免未起进程刷失败）
            if not bool(raw.get("poll_state", False)):
                continue
            url = str(raw.get("url") or "").strip().rstrip("/")
            if not url:
                continue
            sources.append(
                StateSource(
                    name=f"scenario.{name}",
                    url=url,
                    required=bool(raw.get("required", False)),
                )
            )
    return sources


def adapt_state_fragment(
    _source: StateSource,
    body: Optional[Mapping[str, Any]],
) -> Dict[str, Any]:
    if not isinstance(body, Mapping) or not body:
        return {}
    return dict(body)


def _parse_source(name: str, raw: Any) -> Optional[StateSource]:
    extra_urls: List[str] = []
    state_path = "/state"
    health_path = "/health"
    required: Optional[bool] = None
    url = ""
    if isinstance(raw, str):
        url = raw.strip().rstrip("/")
    elif isinstance(raw, Mapping):
        extra_urls = _string_list(raw.get("urls"))
        url = str(raw.get("url") or "").strip().rstrip("/")
        if not url and extra_urls:
            url = extra_urls[0]
        if "state_path" in raw:
            state_path = str(raw.get("state_path") or "").strip() or "/state"
        if "health_path" in raw:
            health_path = str(raw.get("health_path") or "").strip()
        if "required" in raw:
            required = bool(raw.get("required"))
    else:
        return None
    if not url:
        return None
    if required is None:
        required = name in _REQUIRED_SOURCES
    if not state_path.startswith("/"):
        state_path = "/" + state_path
    if health_path and not health_path.startswith("/"):
        health_path = "/" + health_path
    return StateSource(
        name=name,
        url=url,
        state_path=state_path,
        health_path=health_path,
        required=bool(required),
        urls=tuple(item for item in extra_urls if item != url),
    )


def _string_list(raw: Any) -> List[str]:
    if isinstance(raw, str):
        values = [raw]
    elif isinstance(raw, list):
        values = raw
    else:
        return []
    items: list[str] = []
    for item in values:
        url = str(item or "").strip().rstrip("/")
        if url and url not in items:
            items.append(url)
    return items
