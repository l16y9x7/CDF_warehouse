"""构造与 SMT 相同的云端 RTMP 推流地址。"""

from __future__ import annotations

from typing import Any, Dict
from urllib.parse import parse_qs, urlencode, urlparse, urlunparse


DEFAULT_ROBOT_RTMP_PATH_TEMPLATE = (
    "dksl/stream/{sn}_{stream_slot}_{camera_index}-"
    "{video_channel_index}-0_normal-0"
)

ROBOT_STREAM_SLOTS = {
    "head": 1,
    "hand_left": 2,
    "hand_right": 3,
}


def build_rtmp_target(
    stream_server_url: str,
    stream_key: str = "",
    *,
    device_sn: str,
    stream_slot: int,
    camera_index: int = 99,
    video_channel_index: int = 0,
    camera_id: str = "",
    stream_path: str = "",
    stream_path_template: str = DEFAULT_ROBOT_RTMP_PATH_TEMPLATE,
) -> str:
    parsed = urlparse(str(stream_server_url or "").strip())
    if parsed.netloc:
        scheme = parsed.scheme or "rtmp"
        server_host = parsed.netloc
    else:
        scheme = "rtmp"
        server_host = (
            str(stream_server_url or "")
            .replace("rtmp://", "")
            .replace("http://", "")
            .replace("https://", "")
            .strip("/")
        )
    if not server_host:
        return ""

    resolved_path = str(stream_path or "").strip()
    if not resolved_path:
        resolved_path = stream_path_template.format(
            sn=device_sn,
            camera_id=camera_id,
            stream_slot=stream_slot,
            camera_index=camera_index,
            video_channel_index=video_channel_index,
        )
    if not resolved_path.startswith("/"):
        resolved_path = f"/{resolved_path}"

    query_params: Dict[str, Any] = {
        key: values[0] if len(values) == 1 else values
        for key, values in parse_qs(parsed.query).items()
    }
    key = str(stream_key or "").strip()
    if key:
        query_params["sign"] = key
    return urlunparse(
        (
            scheme,
            server_host,
            resolved_path,
            "",
            urlencode(query_params) if query_params else "",
            "",
        )
    )


def redact_url(url: str) -> str:
    text = str(url or "")
    if "sign=" not in text.lower():
        return text
    parsed = urlparse(text)
    query = parse_qs(parsed.query, keep_blank_values=True)
    lowered = {key.lower(): key for key in query}
    if "sign" in lowered:
        query[lowered["sign"]] = ["***"]
    return urlunparse(
        (
            parsed.scheme,
            parsed.netloc,
            parsed.path,
            parsed.params,
            urlencode(query, doseq=True),
            parsed.fragment,
        )
    )
