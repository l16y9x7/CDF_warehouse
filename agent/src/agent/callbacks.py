"""External workflow callback payloads and Chinese display vocabulary."""

from collections.abc import Mapping
from typing import Any

SKILL_LABELS = {
    "navigate": "导航",
    "prepare_pose": "准备姿态",
    "pick_sku_hand": "手动抓取商品",
    "pick_sku_standard": "标准抓取商品",
    "pick_review_basket": "抓取复核篮筐",
    "place_review_basket": "放置复核篮筐",
    "pick_review_item_standard": "标准抓取复核商品",
    "pick_review_item_vla": "智能抓取复核商品",
    "recognize_and_verify_barcode": "识别并校验商品条码",
    "place_sku_in_basket": "放置商品入篮筐",
    "place_review_item": "放置复核商品",
    "confirm_review_basket_empty": "确认复核篮筐为空",
    "summarize_review_result": "汇总复核结果",
    "push_basket": "推送篮筐",
}

ERROR_MESSAGES = {
    "CAPABILITY_UNAVAILABLE": "执行模块不可用",
    "CAPABILITY_EXECUTION_FAILED": "执行模块运行失败",
    "ACTION_RESULT_UNKNOWN": "动作结果不确定，请人工确认",  # noqa: RUF001
    "SCAN_IMAGE_UNAVAILABLE": "扫码图片不可读取，请人工确认",  # noqa: RUF001
    "SKU_BARCODE_NOT_FOUND": "未识别到商品条码，请人工确认",  # noqa: RUF001
    "SKU_BARCODE_MISMATCH": "商品条码与预期不一致",
    "VERIFICATION_FAILED": "抓取位姿校验失败",
    "TIMEOUT": "任务执行超时",
    "CANCELLED": "任务已取消",
    "PROCESS_INTERRUPTED": "服务执行中断，请人工确认任务状态",  # noqa: RUF001
    "INVALID_INPUT": "输入参数无效",
}


def skill_label(name: str | None) -> str:
    return SKILL_LABELS.get(name or "", name or "任务")


def error_message(code: str | None) -> str:
    return ERROR_MESSAGES.get(code or "", "任务执行失败")


def payload(task_id: str, status: str, info: Mapping[str, Any] | None = None) -> dict[str, Any]:
    return {"task_id": task_id, "status": status, "info": dict(info or {})}


def progress_payload(task_id: str, event: Mapping[str, Any]) -> dict[str, Any] | None:
    event_name = event.get("event")
    if event_name not in {"skill.started", "skill.succeeded", "skill.failed"}:
        return None
    name = str(event.get("skill", ""))
    label = skill_label(name)
    if event_name == "skill.started":
        info = {"skill": label, "progress": f"正在执行{label}"}
    elif event_name == "skill.succeeded":
        info = {"skill": label, "progress": f"{label}完成"}
    else:
        code = str(event.get("error_code") or "")
        info = {
            "skill": label,
            "progress": f"{label}失败",
            "error": error_message(code),
            "error_code": code,
        }
    return payload(task_id, "RUNNING", info)


def terminal_payload(task: Mapping[str, Any], events: list[Mapping[str, Any]]) -> dict[str, Any]:
    status = str(task.get("status", "FAILED"))
    if status == "SUCCEEDED":
        info: dict[str, Any] = {"message": "任务完成"}
        result = task.get("result")
        if isinstance(result, dict):
            info["result"] = result
        elif result is not None:
            info["result"] = result
        return payload(str(task["task_id"]), "SUCCEEDED", info)

    if status == "CANCELLED":
        return payload(
            str(task["task_id"]),
            "CANCELLED",
            {"message": "任务已取消", "error": error_message("CANCELLED")},
        )

    failure = next((event for event in reversed(events) if event.get("event") == "skill.failed"), None)
    code = str(task.get("error_code") or (failure or {}).get("error_code") or "EXECUTION_FAILED")
    if status == "WAITING_CONFIRMATION":
        info = {"message": "任务需要人工确认", "error": error_message(code)}
    else:
        info = {"message": "任务失败", "error": code}
    if failure is not None:
        info["skill"] = skill_label(str(failure.get("skill", "")))
        info["progress"] = f"{info['skill']}失败"
    return payload(str(task["task_id"]), "FAILED", info)
