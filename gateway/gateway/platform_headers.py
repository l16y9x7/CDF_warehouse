"""平台 HTTP 请求头的统一构造，对齐 SMT `platform_headers.py`。"""

from __future__ import annotations

from typing import Any, Dict, Mapping


def json_request_headers(config: Mapping[str, Any]) -> Dict[str, str]:
    """Build JSON POST headers, including platform resource auth when configured."""

    headers = {"Content-Type": "application/json"}
    config_key, platform = _platform_auth(config)
    if platform is None:
        return headers
    if not isinstance(platform, Mapping):
        raise ValueError(f"{config_key} must be an object")
    if not platform:
        return headers
    resource = str(platform.get("resource") or "").strip()
    token = str(
        platform.get("resource_token") or platform.get("resource-token") or ""
    ).strip()
    if not resource and not token:
        return headers
    if not resource or not token:
        raise ValueError(f"{config_key} requires resource and resource_token")
    if len(resource) > 128 or len(token) > 512:
        raise ValueError(f"{config_key} value is too long")
    if _contains_unsafe_header_character(resource):
        raise ValueError(f"{config_key} resource is invalid")
    if _contains_unsafe_header_character(token):
        raise ValueError(f"{config_key} resource_token is invalid")
    headers["resource"] = resource
    headers["resource-token"] = token
    return headers


def _platform_auth(config: Mapping[str, Any]) -> tuple[str, Any]:
    if "platform_api" in config:
        return "platform_api", config.get("platform_api")
    if "map_api_auth" in config:
        return "map_api_auth", config.get("map_api_auth")
    nested = config.get("map_sync")
    if isinstance(nested, Mapping):
        if "platform_api" in nested:
            return "map_sync.platform_api", nested.get("platform_api")
        if "map_api_auth" in nested:
            return "map_sync.map_api_auth", nested.get("map_api_auth")
    return "", None


def _contains_unsafe_header_character(value: str) -> bool:
    return any(ord(character) < 33 or ord(character) == 127 for character in value)
