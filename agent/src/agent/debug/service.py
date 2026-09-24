import concurrent.futures
import logging
import time
import uuid
from dataclasses import asdict, is_dataclass
from enum import Enum
from pathlib import Path
from typing import Any

from agent.application import (
    WORKFLOW_CAPABILITY_NAMES,
    AgentApplication,
    build_application_from_capabilities,
)
from agent.callbacks import error_message, progress_payload
from agent.capabilities.camera import MockCameraCapability
from agent.capabilities.estimation import MockEstimationCapability, test_case_meta
from agent.capabilities.hand import MockHandCapability
from agent.capabilities.manipulation import MockManipulationCapability
from agent.capabilities.navigation import MockNavigationCapability
from agent.capabilities.perception import MockPerceptionCapability
from agent.capabilities.pose import MockBodyPoseCapability
from agent.capabilities.vla import MockVlaCapability
from agent.contracts import ExecutionContext, reportable_error_code
from agent.observability import LogStore, log_context, log_event

from .catalog import OPERATIONS_BY_KEY, DebugOperation, catalog_payload
from .media import media_from_run
from .store import DebugStore

TERMINAL_STATUSES = {"SUCCEEDED", "FAILED", "CANCELLED"}
logger = logging.getLogger(__name__)


def _progress_message(task_id: str, events: list[Any]) -> str | None:
    for event in reversed(events):
        if not isinstance(event, dict):
            continue
        body = progress_payload(task_id, event)
        if body is not None:
            progress = body.get("info", {}).get("progress")
            if progress:
                return str(progress)
    return None


def _mock_capabilities() -> dict[str, Any]:
    return {
        "navigation": MockNavigationCapability(),
        "pose": MockBodyPoseCapability(),
        "perception": MockPerceptionCapability(),
        "estimation": MockEstimationCapability(),
        "camera": MockCameraCapability(),
        "manipulation": MockManipulationCapability(),
        "vla": MockVlaCapability(),
        "hand": MockHandCapability(),
    }


def serialize(value: Any) -> Any:
    if is_dataclass(value):
        return {key: serialize(item) for key, item in asdict(value).items()}
    if isinstance(value, Enum):
        return value.value
    if isinstance(value, dict):
        return {str(key): serialize(item) for key, item in value.items()}
    if isinstance(value, (list, tuple)):
        return [serialize(item) for item in value]
    return value


class DebugService:
    def __init__(
        self, real: AgentApplication, *, database_path: str | Path, log_store: LogStore,
    ) -> None:
        self.store = DebugStore(database_path)
        self.log_store = log_store
        self.executor = concurrent.futures.ThreadPoolExecutor(
            max_workers=4, thread_name_prefix="agent-debug"
        )
        self.applications = {
            "real": real,
            "mock": build_application_from_capabilities(
                _mock_capabilities(),
                database_path=f"{database_path}.mock",
                policy=real.policy,
                sku_catalog=real.skills["pick_sku_standard"].sku_catalog,
            ),
        }
        self._real_event_sink = real.runtime.event_sink
        self._real_trace_sink = real.runtime.trace_sink
        for application in self.applications.values():
            self.watch_application(application)

    def shutdown(self) -> None:
        self.executor.shutdown(wait=True, cancel_futures=True)
        self.applications["real"].runtime.event_sink = self._real_event_sink
        self.applications["real"].runtime.trace_sink = self._real_trace_sink
        self.applications["mock"].close()

    def watch_application(self, application: AgentApplication) -> None:
        application.runtime.event_sink = (
            lambda task_id, events, application=application: self._on_runtime_events(
                application, task_id, events
            )
        )
        application.runtime.trace_sink = self.store.attach_task_span

    def record_agent_task(
        self, target: str, operation: str, task_id: str, request: dict[str, Any]
    ) -> str:
        run_id = uuid.uuid4().hex
        self.store.create_run(run_id, target, "workflow", operation, request, task_id=task_id)
        self.store.set_status(
            run_id,
            "ACCEPTED",
            result={"task_id": task_id, "status": "ACCEPTED"},
            finished=False,
        )
        return run_id

    def fail_agent_task(self, run_id: str, exc: Exception) -> None:
        try:
            self.store.set_status(
                run_id,
                "FAILED",
                error_code=reportable_error_code(exc),
                message=str(exc),
            )
        except Exception:
            logger.exception("记录未受理的机器人任务失败", extra={"run_id": run_id})

    def _on_runtime_events(
        self, application: AgentApplication, task_id: str, events: list[Any]
    ) -> None:
        try:
            self.store.attach_events(task_id, events)
            self._sync_agent_task(application, task_id, list(events))
        except Exception:
            logger.exception("同步调试执行记录失败", extra={"task_id": task_id})

    def _sync_agent_task(
        self, application: AgentApplication, task_id: str, events: list[Any]
    ) -> None:
        run_id = self.store.run_id_for_task(task_id)
        if run_id is None:
            return
        task = application.store.get(task_id)
        current = self.store.get_run(run_id)
        if task is None or current is None:
            return
        status = str(task["status"])
        terminal = TERMINAL_STATUSES | {"WAITING_CONFIRMATION"}
        if current["status"] in terminal and status not in terminal:
            return
        finished = status in terminal
        result = task.get("result") if status == "SUCCEEDED" else None
        error_code = None
        message = _progress_message(task_id, events)
        if status == "SUCCEEDED":
            message = "任务完成"
        elif status == "CANCELLED":
            error_code = task.get("error_code") or "CANCELLED"
            message = "任务已取消"
        elif finished:
            error_code = task.get("error_code") or "EXECUTION_FAILED"
            message = error_message(error_code)
        self.store.set_status(
            run_id,
            status,
            result=result,
            events=events,
            error_code=error_code,
            message=message,
            finished=finished,
        )

    def catalog(self) -> dict[str, Any]:
        return catalog_payload()

    def targets(self) -> dict[str, Any]:
        result = []
        for name in ("mock", "real"):
            statuses = {}
            for module in WORKFLOW_CAPABILITY_NAMES:
                capability = self.applications[name].capabilities[module]
                try:
                    statuses[module] = capability.health().status.value
                except Exception:
                    statuses[module] = "ERROR"
            result.append(
                {"name": name, "label": "Mock" if name == "mock" else "真机", "modules": statuses}
            )
        return {"targets": result}

    def test_cases(self) -> dict[str, Any]:
        """调试台测试用例下拉:扫描 tmp/pick_pose_test_case/ 的 SKU 前缀用例。"""
        return {"cases": test_case_meta()}

    def confirmation(
        self, target: str, layer: str, operation: str, payload: dict[str, Any]
    ) -> dict[str, Any]:
        item = self._item(target, layer, operation, payload)
        if layer == "workflow" or not item.physical:
            raise ValueError("this operation does not require confirmation")
        token, expires_at = self.store.create_confirmation(target, layer, operation, payload)
        return {
            "token": token,
            "expires_at": expires_at,
            "summary": {
                "target": target,
                "layer": layer,
                "operation": operation,
                "payload": payload,
            },
        }

    def run(
        self,
        target: str,
        layer: str,
        operation: str,
        payload: dict[str, Any],
        *,
        confirmation_token: str | None = None,
    ) -> dict[str, Any]:
        item = self._item(target, layer, operation, payload)
        if layer == "workflow":
            raise ValueError("use the workflow endpoint for workflow runs")
        if item.physical:
            if not confirmation_token:
                raise ValueError("physical operation requires confirmation")
            self.store.consume_confirmation(confirmation_token, target, layer, operation, payload)
        run_id = uuid.uuid4().hex
        self.store.create_run(run_id, target, layer, operation, payload)
        context = ExecutionContext(
            run_id,
            metadata={"debug_target": target, "debug_layer": layer, "run_id": run_id},
            event_handler=lambda _: self.store.attach_run_events(run_id, list(context.events)),
            trace_handler=lambda span: self.store.attach_span(run_id, dict(span)),
        )
        started = time.monotonic()
        with log_context(run_id=run_id):
            try:
                with context.activate():
                    result = item.execute(self.applications[target], context, payload)
            except Exception as exc:
                self.store.set_status(
                    run_id,
                    "FAILED",
                    events=list(context.events),
                    error_code=reportable_error_code(exc),
                    message=str(exc),
                )
                log_event(
                    logger, logging.ERROR, "debug.run.completed",
                    "调试操作执行失败", run_id=run_id, debug_target=target,
                    debug_layer=layer, operation=operation, status="FAILED",
                    duration_ms=round((time.monotonic() - started) * 1000, 3),
                    error_code=reportable_error_code(exc),
                    error_message=str(exc), suggestion="请检查操作参数和关联能力日志",
                )
            else:
                self.store.set_status(
                    run_id, "SUCCEEDED", result=serialize(result), events=list(context.events)
                )
                log_event(
                    logger, logging.INFO, "debug.run.completed",
                    "调试操作执行成功", run_id=run_id, debug_target=target,
                    debug_layer=layer, operation=operation, status="SUCCEEDED",
                    duration_ms=round((time.monotonic() - started) * 1000, 3),
                )
        run = self.get_run(run_id)
        assert run is not None
        run["client_elapsed_ms"] = round((time.monotonic() - started) * 1000, 3)
        return run

    def submit_workflow(
        self, target: str, operation: str, payload: dict[str, Any], *, callback_base_url: str
    ) -> dict[str, Any]:
        item = self._item(target, "workflow", operation, payload)
        application = self.applications[target]
        task_id, run_id = f"debug-{uuid.uuid4().hex}", uuid.uuid4().hex
        callback_url = f"{callback_base_url.rstrip('/')}/debug/api/callbacks/{run_id}"
        workflow_input = item.workflow_input(payload)
        request = {"task_id": task_id, "callback_url": callback_url, **payload}
        self.store.create_run(run_id, target, "workflow", operation, request, task_id=task_id)
        self.store.set_status(
            run_id, "ACCEPTED", result={"task_id": task_id, "status": "ACCEPTED"}, finished=False
        )
        preflight_context = ExecutionContext(
            task_id,
            metadata={"run_id": run_id, "workflow_id": operation},
            trace_handler=lambda span: self.store.attach_span(run_id, dict(span)),
        )
        try:
            preflight_span = preflight_context.trace_start(
                "preflight", "preflight", "health_checks", input={"workflow": operation}
            )
            with preflight_context.activate():
                try:
                    application.preflight(operation)
                except Exception as exc:
                    preflight_context.trace_end(preflight_span, "FAILED", error=exc)
                    raise
                else:
                    preflight_context.trace_end(
                        preflight_span, "SUCCEEDED", output={"ready": True}
                    )
            application.runtime.accept(
                task_id, operation, callback_url, request, workflow_input,
                metadata={"run_id": run_id},
            )
        except Exception as exc:
            self.store.set_status(
                run_id,
                "FAILED",
                error_code=reportable_error_code(exc),
                message=str(exc),
            )
        run = self.get_run(run_id)
        assert run is not None
        return run

    def get_run(self, run_id: str) -> dict[str, Any] | None:
        run = self.store.get_run(run_id)
        if run is not None:
            run["media"] = media_from_run(run, spans=self.store.trace(run_id).get("spans"))
        return run

    def get_trace(self, run_id: str) -> dict[str, Any] | None:
        if self.store.get_run(run_id) is None:
            return None
        return self.store.trace(run_id)

    def terminate(self, target: str) -> dict[str, str]:
        if target not in self.applications:
            raise ValueError("target must be mock or real")
        task_id = self.applications[target].runtime.terminate()
        return {"task_id": task_id, "status": "TERMINATION_REQUESTED"}

    def callback(self, run_id: str, payload: dict[str, Any]) -> None:
        run = self.store.get_run(run_id)
        if run is None or run["layer"] != "workflow":
            raise KeyError(run_id)
        if payload.get("task_id") != run.get("task_id"):
            raise ValueError("callback task_id does not match debug run")
        if run["status"] in TERMINAL_STATUSES:
            return
        status = str(payload.get("status", "FAILED"))
        info = payload.get("info")
        self.store.set_status(
            run_id,
            status,
            result=(info.get("result") if isinstance(info, dict) else payload.get("result")),
            events=run.get("events"),
            error_code=payload.get("error_code"),
            message=(info or {}).get("message") if isinstance(info, dict) else payload.get("message"),
            info=info if isinstance(info, dict) else None,
            finished=status in TERMINAL_STATUSES,
        )

    def _item(
        self, target: str, layer: str, operation: str, payload: dict[str, Any]
    ) -> DebugOperation:
        if target not in self.applications:
            raise ValueError("target must be mock or real")
        item = OPERATIONS_BY_KEY.get((layer, operation))
        if item is None:
            raise ValueError("unknown debug operation")
        item.validate(payload)
        return item
