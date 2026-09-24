import logging
import time
from collections.abc import Callable, Mapping
from concurrent.futures import Future, ThreadPoolExecutor
from threading import Lock
from typing import Any

import httpx

from agent.callbacks import payload, progress_payload, terminal_payload
from agent.contracts import AgentError, ExecutionContext
from agent.observability import log_context, log_event
from agent.task_store import TERMINAL_STATUSES, SqliteTaskStore
from agent.workflows.base import WorkflowCancelled

logger = logging.getLogger(__name__)


class RobotBusyError(AgentError):
    def __init__(self):
        super().__init__("ROBOT_BUSY", "机器人当前有任务正在运行")


class NoActiveTaskError(AgentError):
    def __init__(self):
        super().__init__("NO_ACTIVE_TASK", "当前没有运行中的任务")


class CallbackSender:
    """Persist and deliver callbacks without blocking workflow execution."""

    def __init__(
        self,
        store: SqliteTaskStore,
        *,
        attempts: int = 3,
        backoff: float = 0.1,
        client: httpx.Client | None = None,
        max_workers: int = 4,
    ):
        self.store = store
        self.attempts = attempts
        self.backoff = backoff
        self._owns_client = client is None
        self.client = client or httpx.Client(timeout=10)
        self._executor = ThreadPoolExecutor(max_workers=max_workers, thread_name_prefix="agent-callback")
        self._locks: dict[str, Lock] = {}
        self._locks_guard = Lock()
        self._local_handlers: dict[str, Callable[[str, dict[str, Any]], None]] = {}

    def register_local_handler(
        self, prefix: str, handler: Callable[[str, dict[str, Any]], None]
    ) -> None:
        if not prefix or "://" not in prefix:
            raise ValueError("local callback prefix must include ://")
        with self._locks_guard:
            self._local_handlers[prefix] = handler

    def unregister_local_handler(self, prefix: str) -> None:
        with self._locks_guard:
            self._local_handlers.pop(prefix, None)

    def enqueue(self, task_id: str, callback_url: str, body: dict[str, Any]) -> int:
        event_id = self.store.enqueue_callback(task_id, callback_url, body)
        with self._locks_guard:
            self._locks.setdefault(task_id, Lock())
        self._executor.submit(self._drain, task_id)
        return event_id

    def _drain(self, task_id: str) -> None:
        with self._locks_guard:
            lock = self._locks.setdefault(task_id, Lock())
        with lock:
            while True:
                event = self.store.next_callback(task_id)
                if event is None:
                    return
                self._deliver_event(event)

    def _deliver_event(self, event: dict[str, Any]) -> None:
        detail = ""
        for attempt in range(1, self.attempts + 1):
            started = time.monotonic()
            try:
                callback_url = str(event["callback_url"])
                with self._locks_guard:
                    handler = next(
                        (
                            local_handler
                            for prefix, local_handler in self._local_handlers.items()
                            if callback_url.startswith(prefix)
                        ),
                        None,
                    )
                if handler is not None:
                    handler(callback_url, event["payload"])
                    status_code = 204
                else:
                    response = self.client.post(callback_url, json=event["payload"])
                    response.raise_for_status()
                    status_code = response.status_code
            except Exception as exc:
                detail = str(exc)
                self.store.record_callback_event_attempt(
                    event["id"], event["task_id"], attempt, False, detail
                )
                log_event(
                    logger, logging.WARNING, "callback.attempt_failed",
                    f"任务回调第 {attempt} 次发送失败", task_id=event["task_id"],
                    callback_event_id=event["id"], attempt=attempt, status="FAILED",
                    duration_ms=round((time.monotonic() - started) * 1000, 3),
                    error_code=type(exc).__name__, error_message=str(exc),
                    suggestion="系统将按策略自动重试，请检查回调服务是否可用",  # noqa: RUF001
                )
                if attempt < self.attempts and self.backoff:
                    time.sleep(self.backoff * (2 ** (attempt - 1)))
            else:
                self.store.record_callback_event_attempt(
                    event["id"], event["task_id"], attempt, True, str(status_code)
                )
                self.store.finish_callback_event(event["id"], sent=True)
                log_event(
                    logger, logging.INFO, "callback.succeeded", "任务回调发送成功",
                    task_id=event["task_id"], callback_event_id=event["id"],
                    attempt=attempt, status="SUCCEEDED", http_status=status_code,
                    duration_ms=round((time.monotonic() - started) * 1000, 3),
                )
                return
        self.store.finish_callback_event(event["id"], sent=False, detail=detail)
        log_event(
            logger, logging.ERROR, "callback.exhausted", "任务回调重试次数已耗尽",
            task_id=event["task_id"], callback_event_id=event["id"], status="FAILED",
            error_code="CALLBACK_EXHAUSTED", error_message=detail,
            suggestion="请检查回调服务并通过任务记录确认最终状态",
        )

    def send(self, task: dict[str, Any]) -> None:
        """Synchronously send one payload for callers that use the legacy helper."""
        if "info" in task:
            body = payload(task["task_id"], str(task["status"]), task.get("info"))
        else:
            # Keep the low-level helper compatible for existing internal callers.
            body = {"task_id": task["task_id"], "status": task["status"]}
            if task["status"] == "SUCCEEDED":
                body["result"] = task.get("result") or {}
            else:
                body["error_code"] = task.get("error_code") or "EXECUTION_FAILED"
        for attempt in range(1, self.attempts + 1):
            try:
                response = self.client.post(task["callback_url"], json=body)
                response.raise_for_status()
            except Exception as exc:
                self.store.record_callback(task["task_id"], attempt, False, str(exc))
                if attempt < self.attempts and self.backoff:
                    time.sleep(self.backoff * (2 ** (attempt - 1)))
            else:
                self.store.record_callback(task["task_id"], attempt, True, str(response.status_code))
                return

    def close(self) -> None:
        self._executor.shutdown(wait=True)
        if self._owns_client:
            self.client.close()

    def wait(self, task_id: str, timeout: float = 5) -> None:
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            if self.store.next_callback(task_id) is None:
                return
            time.sleep(0.005)


class AgentRuntime:
    def __init__(
        self,
        store: SqliteTaskStore,
        workflow_factory: Callable[[str], Any],
        *,
        callback_sender: CallbackSender | None = None,
        event_sink: Callable[[str, list[Any]], None] | None = None,
        trace_sink: Callable[[str, Mapping[str, Any]], None] | None = None,
        max_workers: int = 4,
    ):
        self.store = store
        self.workflow_factory = workflow_factory
        self.callback_sender = callback_sender or CallbackSender(store)
        self.event_sink = event_sink
        self.trace_sink = trace_sink
        self.executor = ThreadPoolExecutor(max_workers=max_workers, thread_name_prefix="agent-task")
        self._futures: dict[str, Future[Any]] = {}
        self._lock = Lock()
        self._active_task_id: str | None = None
        self._active_context: ExecutionContext | None = None
        for pending_task in self.store.pending_callback_tasks():
            self.callback_sender._executor.submit(self.callback_sender._drain, pending_task)

    @property
    def is_busy(self) -> bool:
        with self._lock:
            return self._active_task_id is not None

    @property
    def active_task_id(self) -> str | None:
        with self._lock:
            return self._active_task_id

    def accept(
        self,
        task_id: str,
        task_type: str,
        callback_url: str,
        request: dict[str, Any],
        workflow_input: Any,
        metadata: Mapping[str, Any] | None = None,
    ) -> bool:
        context = ExecutionContext(
            task_id,
            metadata={"workflow_id": task_type, **dict(metadata or {})},
            event_handler=lambda event: self._on_event(task_id, event, context.events),
            trace_handler=lambda span: self._on_span(task_id, span),
        )
        with self._lock:
            if self.store.get(task_id) is not None:
                return False
            if self._active_task_id is not None:
                raise RobotBusyError()
            created = self.store.create(task_id, task_type, callback_url, request)
            if not created:
                return False
            self._active_task_id = task_id
            self._active_context = context
        log_event(
            logger, logging.INFO, "runtime.task.accepted", "机器人任务已受理",
            task_id=task_id, workflow=task_type, request_id=context.metadata.get("request_id"),
            run_id=context.metadata.get("run_id"), status="ACCEPTED",
        )
        self.callback_sender.enqueue(
            task_id, callback_url, payload(task_id, "ACCEPTED", {"message": "任务已收到"})
        )
        try:
            future = self.executor.submit(self._execute, task_id, task_type, workflow_input, context)
        except Exception:
            with self._lock:
                if self._active_task_id == task_id:
                    self._active_task_id = None
                    self._active_context = None
            self.store.set_status(task_id, "FAILED", error_code="EXECUTION_FAILED")
            raise
        with self._lock:
            self._futures[task_id] = future
        future.add_done_callback(lambda _: self._forget(task_id))
        return True

    def terminate(self) -> str:
        with self._lock:
            if self._active_task_id is None or self._active_context is None:
                raise NoActiveTaskError()
            task_id = self._active_task_id
            if not self._active_context.is_cancelled:
                self._active_context.cancel()
                self._active_context.emit("workflow.cancellation_requested", status="CANCELLED")
            return task_id

    def _on_event(
        self, task_id: str, event: Mapping[str, Any], events: list[Mapping[str, Any]]
    ) -> None:
        if self.event_sink is not None:
            self.event_sink(task_id, list(events))
        body = progress_payload(task_id, event)
        if body is None:
            return
        task = self.store.get(task_id)
        if task is not None:
            self.callback_sender.enqueue(task_id, task["callback_url"], body)

    def _on_span(self, task_id: str, span: Mapping[str, Any]) -> None:
        if self.trace_sink is not None:
            self.trace_sink(task_id, span)

    def _execute(
        self,
        task_id: str,
        task_type: str,
        workflow_input: Any,
        context: ExecutionContext,
    ) -> None:
        with log_context(
            task_id=task_id, workflow=task_type, run_id=context.metadata.get("run_id"),
            request_id=context.metadata.get("request_id"),
        ):
            started = time.monotonic()
            self.store.set_status(task_id, "RUNNING")
            result: Any = None
            failure: Exception | None = None
            workflow_span = context.trace_start(
                "workflow", task_type, "run", input=workflow_input
            )
            with context.activate(), log_context(span_id=workflow_span):
                try:
                    result = self.workflow_factory(task_type).run(context, workflow_input)
                except Exception as exc:
                    failure = exc
                    trace_status = "CANCELLED" if getattr(exc, "code", None) == "CANCELLED" else "FAILED"
                    context.trace_end(workflow_span, trace_status, error=exc)
                else:
                    context.trace_end(workflow_span, "SUCCEEDED", output=result)
            self._finalize(task_id, context, result, failure)
            task = self.store.get(task_id)
            final_status = task["status"] if task else "FAILED"
            log_event(
                logger, logging.ERROR if failure else logging.INFO, "runtime.task.completed",
                "机器人任务执行失败" if failure else "机器人任务执行完成",
                status=final_status, duration_ms=round((time.monotonic() - started) * 1000, 3),
                error_code=getattr(failure, "code", type(failure).__name__) if failure else None,
                error_message=str(failure) if failure else None,
                suggestion=(
                    "请结合任务节点和能力调用日志定位失败原因" if failure else None
                ),
            )
            if self.event_sink is not None:
                self.event_sink(task_id, list(context.events))
            if task is not None:
                self.callback_sender.enqueue(
                    task_id, task["callback_url"], terminal_payload(task, context.events)
                )

    def _finalize(
        self,
        task_id: str,
        context: ExecutionContext,
        result: Any,
        failure: Exception | None,
    ) -> None:
        with self._lock:
            if context.is_cancelled or isinstance(failure, WorkflowCancelled):
                snapshot = self.store.load(task_id)
                if isinstance(snapshot, dict) and "status" in snapshot:
                    snapshot["status"] = "CANCELLED"
                    self.store.save(task_id, snapshot)
                self.store.set_status(task_id, "CANCELLED", error_code="CANCELLED")
                context.emit("workflow.cancelled", status="CANCELLED", error_code="CANCELLED")
            elif failure is not None:
                snapshot = self.store.load(task_id) or {}
                status = snapshot.get("status", "FAILED")
                if status not in {"FAILED", "WAITING_CONFIRMATION"}:
                    status = "FAILED"
                self.store.set_status(
                    task_id,
                    status,
                    error_code=getattr(failure, "code", "EXECUTION_FAILED"),
                )
            else:
                self.store.set_status(task_id, "SUCCEEDED", result=result)
            if self._active_task_id == task_id:
                self._active_task_id = None
                self._active_context = None

    def seal_interrupted_tasks(self) -> None:
        for interrupted in self.store.mark_interrupted():
            log_event(
                logger, logging.WARNING, "runtime.task.interrupted",
                "发现进程异常中断前未完成的任务，已转为等待人工确认",  # noqa: RUF001
                task_id=interrupted["task_id"], status="WAITING_CONFIRMATION",
                error_code="PROCESS_INTERRUPTED",
                suggestion="请人工确认机器人和物品状态后再决定是否重新执行",
            )
            task = self.store.get(interrupted["task_id"])
            if task is not None:
                self.callback_sender.enqueue(
                    task["task_id"], task["callback_url"], terminal_payload(task, [])
                )

    def wait(self, task_id: str, timeout: float = 5) -> None:
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            with self._lock:
                future = self._futures.get(task_id)
            if future is not None:
                future.result(timeout=max(0.01, deadline - time.monotonic()))
                self.callback_sender.wait(task_id, timeout=max(0.01, deadline - time.monotonic()))
                return
            task = self.store.get(task_id)
            if task is not None and task["status"] in TERMINAL_STATUSES:
                return
            time.sleep(0.01)
        raise TimeoutError(f"task did not finish: {task_id}")

    def shutdown(self) -> None:
        self.executor.shutdown(wait=True)
        self.callback_sender.close()

    def _forget(self, task_id: str) -> None:
        with self._lock:
            self._futures.pop(task_id, None)
