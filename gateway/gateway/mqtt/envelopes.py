"""MQTT 云端信封：CommandReply 与 TaskEvent，对齐 dog_device-SMT 协议。"""

from __future__ import annotations

import re
from typing import Any, Dict, Optional

_NONCE_RE = re.compile(r"[A-Za-z0-9_-]{16,128}")


def normalize_service_result(result: Optional[Dict[str, Any]]) -> Dict[str, Any]:
    if not isinstance(result, dict):
        return {
            "code": -1,
            "message": "Invalid handler result",
            "data": {"result": -1, "error": "Invalid handler result"},
        }

    if isinstance(result.get("data"), dict) and (
        "code" in result or "message" in result
    ):
        explicit_code = result.get("code")
        code = explicit_code if isinstance(explicit_code, int) else 0
        message = str(result.get("message") or ("success" if code == 0 else "failed"))
        data = dict(result["data"])
        data.setdefault("result", code)
        if code != 0:
            data.setdefault("error", message)
        return {"code": code, "message": message, "data": data}

    data = dict(result)
    result_code = data.get("result")
    explicit_code = data.get("code")
    if isinstance(explicit_code, int):
        code = explicit_code
    elif isinstance(result_code, int):
        code = result_code
    else:
        code = 0

    message = data.get("message") or data.get("error")
    if not message:
        message = "success" if code == 0 else "failed"
    data.setdefault("result", code)
    if code != 0:
        data.setdefault("error", message)
    return {"code": code, "message": message, "data": data}


def result_succeeded(result: Any) -> bool:
    normalized = normalize_service_result(result if isinstance(result, dict) else None)
    data = normalized.get("data")
    result_code = data.get("result") if isinstance(data, dict) else None
    return normalized["code"] == 0 and result_code in (None, 0)


def result_summary(method: str, result: Any) -> str:
    del method
    if not isinstance(result, dict):
        return f"result={result}"
    normalized = normalize_service_result(result)
    data = normalized.get("data")
    if not isinstance(data, dict):
        data = {}
    parts = [
        f"code={normalized['code']}",
        f"message={normalized['message']}",
    ]
    result_code = data.get("result")
    if result_code is not None:
        parts.append(f"result={result_code}")
    error = data.get("error")
    if error:
        parts.append(f"error={error}")
    elif data:
        parts.append("data_keys=" + ",".join(sorted(str(key) for key in data.keys())))
    for key in ("task_id", "status", "scenario"):
        if key in data:
            parts.append(f"{key}={data[key]}")
    return " ".join(parts)


def normalize_task_event(data: Dict[str, Any]) -> Dict[str, Any]:
    """校验 SMT event Topic 信封；Gateway 不解释 stage 业务含义。"""

    source = dict(data or {})
    if "type" in source:
        raise ValueError("event type is not supported; use stage")

    task_id = str(source.pop("task_id", "") or "").strip()
    stage = str(source.pop("stage", "") or "").strip()
    title = str(source.pop("title", "") or "").strip()
    step_index = source.pop("step_index", None)
    thingking = source.pop("thingking", "")

    if not task_id:
        raise ValueError("event task_id is required")
    if not stage:
        raise ValueError("event stage is required")
    if not title:
        raise ValueError("event title is required")
    if (
        isinstance(step_index, bool)
        or not isinstance(step_index, int)
        or step_index < 1
    ):
        raise ValueError("event step_index must be a positive integer")
    if thingking is None:
        thingking = ""
    if not isinstance(thingking, str):
        raise ValueError("event thingking must be a string")

    phase_fields = _optional_handoff_fields(source)
    unsupported = sorted(set(source) - {"description", "image_url"})
    if unsupported:
        raise ValueError("unsupported event data fields: " + ", ".join(unsupported))

    normalized: Dict[str, Any] = {
        "task_id": task_id,
        "step_index": step_index,
        "stage": stage,
        "title": title,
        "thingking": "",
    }
    normalized.update(source)
    normalized.update(phase_fields)
    return normalized


def normalize_task_result(data: Dict[str, Any]) -> Dict[str, Any]:
    """TaskResult 映射到云端 event 信封，不发明业务终态。"""

    source = dict(data or {})
    task_id = str(source.get("task_id") or "").strip()
    if not task_id:
        raise ValueError("result task_id is required")
    terminal = str(
        source.get("terminal_state") or source.get("stage") or ""
    ).strip()
    if not terminal:
        raise ValueError("result terminal_state is required")
    title = str(source.get("title") or terminal).strip()
    step_index = source.get("step_index", 1)
    if (
        isinstance(step_index, bool)
        or not isinstance(step_index, int)
        or step_index < 1
    ):
        raise ValueError("result step_index must be a positive integer")
    stage = _result_stage(terminal)
    event: Dict[str, Any] = {
        "task_id": task_id,
        "step_index": step_index,
        "stage": stage,
        "title": title,
        "thingking": "",
    }
    description = source.get("description") or source.get("message")
    if description:
        event["description"] = str(description)
    return event


def task_status_code(terminal_or_status: str) -> int:
    value = str(terminal_or_status or "").strip().lower()
    if value in {"", "idle", "none"}:
        return 0
    if value in {"running", "accepted", "cancelling", "1"}:
        return 1
    if value in {"completed", "succeeded", "success", "2"}:
        return 2
    return 3


def _result_stage(terminal: str) -> str:
    value = str(terminal or "").strip()
    lowered = value.lower()
    mapping = {
        "succeeded": "task_completed",
        "completed": "task_completed",
        "failed": "task_failed",
        "cancelled": "task_cancelled",
        "canceled": "task_cancelled",
        "timed_out": "task_failed",
        "timeout": "task_failed",
        "rejected": "task_failed",
        "task_completed": "task_completed",
        "task_failed": "task_failed",
        "task_cancelled": "task_cancelled",
    }
    return mapping.get(lowered, value)


def _optional_handoff_fields(source: Dict[str, Any]) -> Dict[str, str]:
    phase_present = "phase" in source
    nonce_present = "confirmation_nonce" in source
    if not phase_present and not nonce_present:
        return {}
    phase = source.pop("phase", None)
    nonce = source.pop("confirmation_nonce", None)
    if not phase_present or not nonce_present:
        raise ValueError("event phase and confirmation_nonce must be provided together")
    if not isinstance(phase, str) or not str(phase).strip():
        raise ValueError("event phase must be a string")
    if not isinstance(nonce, str):
        raise ValueError("event confirmation_nonce must be a string")
    normalized_nonce = nonce.strip()
    if not _NONCE_RE.fullmatch(normalized_nonce):
        raise ValueError("event confirmation_nonce is invalid")
    return {"phase": phase.strip(), "confirmation_nonce": normalized_nonce}
