import asyncio
import logging
import threading
import time
import uuid
from concurrent.futures import ThreadPoolExecutor
from typing import Any

import httpx

from agent.application import AgentApplication
from agent.callbacks import progress_payload
from agent.contracts import reportable_error_code

from .catalog import OrderProduct
from .store import OrderStore

logger = logging.getLogger(__name__)

_BARCODE_SKILL = "recognize_and_verify_barcode"


class OrderNotFound(KeyError):
    pass


class OrderConflict(RuntimeError):
    pass


class OrderService:
    def __init__(
        self,
        application: AgentApplication,
        *,
        database_path: str,
        products: dict[str, OrderProduct],
        mock_application: AgentApplication | None = None,
    ) -> None:
        self.application = application
        self.mock_application = mock_application or application
        self.products = products
        self.store = OrderStore(database_path)
        self.store.recover_interrupted()
        self._executor = ThreadPoolExecutor(max_workers=1, thread_name_prefix="user-order")
        self._schedule_lock = threading.RLock()
        self._scheduled: set[str] = set()
        self._watched: set[int] = set()
        self._app: Any = None

    def bind_app(self, app: Any) -> None:
        self._app = app

    def watch_runtime(self, application: AgentApplication) -> None:
        runtime_id = id(application.runtime)
        if runtime_id in self._watched:
            return
        self._watched.add(runtime_id)
        previous = application.runtime.event_sink

        def sink(task_id: str, events: list[Any], previous: Any = previous) -> None:
            if previous is not None:
                previous(task_id, events)
            if events:
                self._on_agent_event(task_id, events[-1])

        application.runtime.event_sink = sink

    def close(self) -> None:
        self._executor.shutdown(wait=True, cancel_futures=True)

    def product_list(self) -> list[dict[str, str]]:
        return [product.public_dict() for product in self.products.values()]

    def current(self) -> dict[str, Any] | None:
        return self.store.active_order() or self.store.latest_order()

    def get(self, order_id: str) -> dict[str, Any]:
        order = self.store.get_order(order_id)
        if order is None:
            raise OrderNotFound(order_id)
        return order

    def submit(
        self,
        items: list[tuple[str, int, str, str]],
        basket_row: str,
        basket_column: str,
        *,
        mock: bool = False,
    ) -> dict[str, Any]:
        if not items:
            raise ValueError("订单至少需要一个商品")
        if basket_row not in {f"L{number}" for number in range(1, 5)}:
            raise ValueError("basket_row must be L1-L4")
        if basket_column not in {str(number) for number in range(1, 6)}:
            raise ValueError("basket_column must be 1-5")
        if self._target(mock).runtime.is_busy:
            raise OrderConflict("所选运行时当前有任务正在运行")
        normalized: list[tuple[OrderProduct, int, str, str]] = []
        seen: set[str] = set()
        total_quantity = 0
        for sku_id, quantity, agv_row, agv_column in items:
            if sku_id not in self.products:
                raise ValueError(f"未知商品：{sku_id}")
            if sku_id in seen:
                raise ValueError(f"订单中商品重复：{sku_id}")
            if quantity < 1 or quantity > 20:
                raise ValueError("单个商品数量必须在 1 到 20 之间")
            if agv_row not in {f"L{number}" for number in range(1, 6)}:
                raise ValueError("agv_row must be L1-L5")
            if agv_column not in {"1", "2"}:
                raise ValueError("agv_column must be 1 or 2")
            seen.add(sku_id)
            total_quantity += quantity
            normalized.append((self.products[sku_id], quantity, agv_row, agv_column))
        if total_quantity > 50:
            raise ValueError("单个订单最多包含 50 件商品")
        order_id = f"ORD-{time.strftime('%Y%m%d-%H%M%S')}-{uuid.uuid4().hex[:6].upper()}"
        try:
            order = self.store.create_order(
                order_id,
                normalized,
                basket_row,
                basket_column,
                mock=mock,
            )
        except RuntimeError as exc:
            if str(exc) == "ACTIVE_ORDER_EXISTS":
                raise OrderConflict("当前已有未完成订单") from exc
            raise
        self._schedule(order_id)
        return order

    def retry(self, order_id: str) -> dict[str, Any]:
        order = self.get(order_id)
        if self._target(bool(order["mock"])).runtime.is_busy:
            raise OrderConflict("所选运行时当前仍有任务正在运行")
        try:
            self.store.retry_paused(order_id)
        except RuntimeError as exc:
            raise OrderConflict("只有暂停的订单可以重试") from exc
        self._schedule(order_id)
        return self.get(order_id)

    def cancel(self, order_id: str) -> dict[str, Any]:
        order = self.get(order_id)
        if order["status"] in {"SUCCEEDED", "CANCELLED"}:
            raise OrderConflict("订单已经结束")
        task_id = order.get("current_task_id")
        target = self._target(bool(order["mock"]))
        if task_id and target.runtime.active_task_id == task_id:
            self.store.update_order(
                order_id,
                status="CANCELLING",
                progress_text="正在停止机器人当前任务",
            )
            self.store.event(
                order_id, "order.cancelling", {"message": "正在取消订单"}
            )
            status_code, payload = self._post("/agent/terminate", {"mock": bool(order["mock"])})
            if status_code != 200:
                self.store.update_order(
                    order_id,
                    status=order["status"],
                    progress_text=order.get("progress_text"),
                )
                message = payload.get("message") or payload.get("error_code") or "终止任务失败"
                raise OrderConflict(message)
        else:
            self._finish_cancelled(order_id)
        return self.get(order_id)

    def events_after(self, order_id: str, event_id: int) -> list[dict[str, Any]]:
        self.get(order_id)
        return self.store.events_after(order_id, event_id)

    def _schedule(self, order_id: str) -> None:
        with self._schedule_lock:
            if order_id in self._scheduled:
                return
            self._scheduled.add(order_id)
        future = self._executor.submit(self._run_order, order_id)
        future.add_done_callback(lambda _: self._unschedule(order_id))

    def _unschedule(self, order_id: str) -> None:
        with self._schedule_lock:
            self._scheduled.discard(order_id)

    def _run_order(self, order_id: str) -> None:
        try:
            order = self.get(order_id)
            if order["status"] != "RUNNING":
                return
            for unit in self.store.units(order_id):
                if unit["status"] == "SUCCEEDED":
                    continue
                if not self._run_unit(order_id, unit):
                    return
            order = self.get(order_id)
            if order["status"] != "RUNNING":
                return
            self._complete_order(order_id)
        except Exception as exc:
            logger.exception("用户订单编排异常", extra={"order_id": order_id})
            order = self.store.get_order(order_id)
            if order is not None and order["status"] not in {"SUCCEEDED", "CANCELLED"}:
                self._pause(order_id, str(exc), stage=order["stage"])

    def _run_unit(self, order_id: str, unit: dict[str, Any]) -> bool:
        self.store.update_order(
            order_id,
            stage="ITEMS",
            current_unit_id=unit["unit_id"],
            current_skill=None,
            progress_text=f"准备抓取第 {unit['sequence']} 件商品：{unit['name']}",
            error=None,
        )
        for cycle_attempt in range(1, 3):
            if self.get(order_id)["status"] != "RUNNING":
                return False
            task_id = f"{order_id}-I{unit['sequence']:03d}-A{unit['attempts'] + cycle_attempt}"
            self.store.increment_unit_attempt(unit["unit_id"], task_id)
            self.store.update_order(order_id, current_task_id=task_id)
            self.store.event(
                order_id,
                "item.started" if cycle_attempt == 1 else "item.retrying",
                {
                    "sequence": unit["sequence"],
                    "sku_id": unit["sku_id"],
                    "name": unit["name"],
                    "attempt": cycle_attempt,
                },
            )
            order = self.get(order_id)
            error = self._execute_task(
                order_id,
                task_id,
                "/agent/sorting/item",
                {
                    "task_id": task_id,
                    "sku_id": unit["sku_id"],
                    "name": unit["name"],
                    "agv_row": unit["agv_row"],
                    "agv_column": unit["agv_column"],
                    "basket_row": order["basket_row"],
                    "basket_column": order["basket_column"],
                    "mock": bool(order["mock"]),
                },
            )
            if error is None:
                self.store.update_unit(
                    unit["unit_id"], status="SUCCEEDED", progress_text="商品已放入篮筐", error=None
                )
                self.store.event(
                    order_id,
                    "item.succeeded",
                    {"sequence": unit["sequence"], "sku_id": unit["sku_id"]},
                )
                return True
            if self.get(order_id)["status"] in {"CANCELLING", "CANCELLED"}:
                self._finish_cancelled(order_id)
                return False
            if cycle_attempt == 1:
                self.store.update_unit(
                    unit["unit_id"],
                    status="RETRYING",
                    progress_text="首次执行失败，正在自动重试",
                    error=error,
                )
                self.store.update_order(
                    order_id, progress_text="当前商品失败，正在自动重试一次", error=error
                )
                self.store.event(
                    order_id,
                    "item.retrying",
                    {"sequence": unit["sequence"], "error": error},
                )
                continue
            self.store.update_unit(
                unit["unit_id"], status="PAUSED", progress_text="重试后仍失败", error=error
            )
            self._pause(order_id, error, stage="ITEMS")
            return False
        return False

    def _complete_order(self, order_id: str) -> None:
        now = time.time()
        self.store.update_order(
            order_id,
            status="SUCCEEDED",
            stage="ITEMS",
            current_unit_id=None,
            current_skill=None,
            progress_text="订单已完成",
            error=None,
            finished_at=now,
        )
        self.store.event(order_id, "order.succeeded", {"message": "订单已完成"})

    def _target(self, use_mock: bool) -> AgentApplication:
        return self.mock_application if use_mock else self.application

    def _post(self, path: str, body: dict[str, Any]) -> tuple[int, dict[str, Any]]:
        if self._app is None:
            raise RuntimeError("订单服务尚未绑定 HTTP 应用")

        async def request() -> tuple[int, dict[str, Any]]:
            transport = httpx.ASGITransport(app=self._app)
            async with httpx.AsyncClient(transport=transport, base_url="http://agent") as client:
                response = await client.post(path, json=body)
                payload = response.json() if response.content else {}
                return response.status_code, payload if isinstance(payload, dict) else {}

        return asyncio.run(request())

    def _execute_task(
        self,
        order_id: str,
        task_id: str,
        path: str,
        body: dict[str, Any],
    ) -> str | None:
        target = self._target(bool(body.get("mock")))
        try:
            status_code, payload = self._post(path, body)
            if status_code != 200:
                code = payload.get("error_code") or "EXECUTION_FAILED"
                message = payload.get("message") or code
                return f"{code}：{message}"
            target.runtime.wait(task_id, timeout=24 * 60 * 60)
        except Exception as exc:
            return f"{reportable_error_code(exc)}：{exc}"
        task = target.store.get(task_id)
        if task is not None and task["status"] == "SUCCEEDED":
            return None
        if task is None:
            return "EXECUTION_FAILED：未找到机器人任务记录"
        return f"{task.get('error_code') or task['status']}：机器人任务执行失败"

    def _pause(self, order_id: str, error: str, *, stage: str) -> None:
        self.store.update_order(
            order_id,
            status="PAUSED",
            stage=stage,
            progress_text="自动重试后仍失败，等待人工处理",
            error=error,
        )
        self.store.event(
            order_id,
            "order.paused",
            {"message": "订单已暂停", "error": error, "stage": stage},
        )

    def _finish_cancelled(self, order_id: str) -> None:
        order = self.store.get_order(order_id)
        if order is None or order["status"] == "CANCELLED":
            return
        if order.get("current_unit_id"):
            unit = next(
                (
                    item
                    for item in self.store.units(order_id)
                    if item["unit_id"] == order["current_unit_id"]
                ),
                None,
            )
            if unit is not None and unit["status"] != "SUCCEEDED":
                self.store.update_unit(
                    unit["unit_id"], status="CANCELLED", progress_text="订单已取消"
                )
        self.store.update_order(
            order_id,
            status="CANCELLED",
            progress_text="订单已取消",
            current_skill=None,
            error=None,
            finished_at=time.time(),
        )
        self.store.event(order_id, "order.cancelled", {"message": "订单已取消"})

    def _barcode_progress(self, order: dict[str, Any], event: dict[str, Any]) -> dict[str, str] | None:
        if event.get("skill") != _BARCODE_SKILL:
            return None
        kind = event.get("event")
        if kind == "skill.started":
            unit = next(
                (
                    item
                    for item in order.get("units") or []
                    if item.get("unit_id") == order.get("current_unit_id")
                ),
                None,
            )
            name = str((unit or {}).get("name") or "商品")
            sku_id = str((unit or {}).get("sku_id") or "")
            return {"skill": "核对商品", "progress": f"正在核对 {name} 编码为 {sku_id}"}
        if kind in {"skill.succeeded", "skill.failed"}:
            return {"skill": "核对商品", "progress": "商品正确"}
        return None

    def _on_agent_event(self, task_id: str, event: dict[str, Any]) -> None:
        payload = progress_payload(task_id, event)
        if payload is None:
            return
        order_id = self.store.order_id_for_task(task_id)
        if order_id is None:
            return
        order = self.store.get_order(order_id)
        if order is None or payload.get("task_id") != order.get("current_task_id"):
            return
        info = dict(payload.get("info") if isinstance(payload.get("info"), dict) else {})
        barcode = self._barcode_progress(order, event)
        if barcode is not None:
            info = barcode
        skill = info.get("skill")
        progress = info.get("progress") or info.get("message")
        values: dict[str, Any] = {}
        if skill is not None:
            values["current_skill"] = str(skill)
        if progress is not None:
            values["progress_text"] = str(progress)
        if values:
            self.store.update_order(order_id, **values)
            if order.get("current_unit_id"):
                self.store.update_unit(order["current_unit_id"], **values)
        self.store.event(
            order_id,
            "agent.progress",
            {"status": payload.get("status"), "task_id": payload.get("task_id"), "info": info},
        )
