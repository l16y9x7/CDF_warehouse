"""日志脱敏。"""

from __future__ import annotations

from typing import Any
from urllib.parse import parse_qsl, urlencode, urlsplit, urlunsplit

SECRET_KEYS = {
    "access_token",
    "key",
    "password",
    "passwd",
    "secret",
    "sign",
    "token",
}


def redact_url(value: Any) -> str:
    text = str(value or "").strip()
    if not text:
        return ""
    parsed = urlsplit(text)
    if not parsed.query:
        return text
    params = []
    changed = False
    for key, item in parse_qsl(parsed.query, keep_blank_values=True):
        if key.lower() in SECRET_KEYS:
            params.append((key, "***"))
            changed = True
        else:
            params.append((key, item))
    if not changed:
        return text
    return urlunsplit(
        (parsed.scheme, parsed.netloc, parsed.path, urlencode(params), parsed.fragment)
    )


def log_safe(value: Any) -> Any:
    if isinstance(value, dict):
        safe: dict[str, Any] = {}
        for key, item in value.items():
            if str(key).lower() in SECRET_KEYS:
                safe[key] = "***"
            elif isinstance(item, str) and "://" in item:
                safe[key] = redact_url(item)
            else:
                safe[key] = log_safe(item)
        return safe
    if isinstance(value, list):
        return [log_safe(item) for item in value]
    if isinstance(value, str) and "://" in value:
        return redact_url(value)
    return value
