import time
from collections.abc import Callable
from dataclasses import asdict, dataclass, field, is_dataclass
from typing import Any, Protocol

from agent.contracts import AgentError, ExecutionContext


class WorkflowStateStore(Protocol):
    def save(self, task_id: str, snapshot: Any) -> None: ...
    def load(self, task_id: str) -> Any | None: ...


@dataclass
class InMemoryWorkflowStateStore:
    snapshots: dict[str, Any] = field(default_factory=dict)
    nodes: dict[str, list[dict[str, Any]]] = field(default_factory=dict)

    def save(self, task_id: str, snapshot: Any) -> None:
        self.snapshots[task_id] = snapshot

    def load(self, task_id: str) -> Any | None:
        return self.snapshots.get(task_id)

    def record_node(self, task_id: str, record: dict[str, Any]) -> None:
        self.nodes.setdefault(task_id, []).append(record)


class WorkflowCancelled(AgentError):
    def __init__(self, message: str = "workflow cancelled"):
        super().__init__("CANCELLED", message)


class WorkflowTimeout(AgentError):
    def __init__(self, message: str = "workflow deadline exceeded"):
        super().__init__("TIMEOUT", message)


def check_context(context: ExecutionContext) -> None:
    if context.is_cancelled:
        raise WorkflowCancelled()
    if context.deadline is not None and time.monotonic() >= context.deadline:
        raise WorkflowTimeout()


class NodeRunner:
    """Persist node boundaries and keep business state transitions outside Skills."""

    def __init__(
        self,
        context: ExecutionContext,
        store: WorkflowStateStore | None = None,
        *,
        state: Any = None,
        node_attribute: str | None = None,
    ):
        self.context = context
        self.store = store
        self.state = state
        self.node_attribute = node_attribute

    def run(
        self,
        node_id: str,
        action: Callable[[], Any],
        *,
        on_success: Callable[[Any], None] | None = None,
    ) -> Any:
        check_context(self.context)
        started = time.monotonic()
        self.context.metadata["node_id"] = node_id
        self._update_node(node_id)
        self._save_state()
        self.context.emit("workflow.node.started", workflow_node=node_id, status="RUNNING")
        if self.store is not None and hasattr(self.store, "record_node"):
            self.store.record_node(self.context.task_id, {"node_id": node_id, "status": "RUNNING"})
        try:
            result = action()
            if on_success is not None:
                on_success(result)
        except Exception as exc:
            self.context.emit(
                "workflow.node.failed",
                workflow_node=node_id,
                status="FAILED",
                error_code=getattr(exc, "code", type(exc).__name__),
                error_message=str(exc),
                duration_ms=round((time.monotonic() - started) * 1000, 3),
            )
            if self.store is not None and hasattr(self.store, "record_node"):
                self.store.record_node(
                    self.context.task_id,
                    {"node_id": node_id, "status": "FAILED", "error": str(exc)},
                )
            raise
        self.context.emit(
            "workflow.node.succeeded", workflow_node=node_id, status="SUCCEEDED",
            duration_ms=round((time.monotonic() - started) * 1000, 3),
        )
        if self.store is not None and hasattr(self.store, "record_node"):
            self.store.record_node(
                self.context.task_id, {"node_id": node_id, "status": "SUCCEEDED"}
            )
        self._save_state()
        # A synchronous capability cannot be interrupted safely. Honour a
        # cancellation as soon as that call returns and before another node starts.
        check_context(self.context)
        return result

    def _update_node(self, node_id: str) -> None:
        if self.state is not None and self.node_attribute is not None:
            setattr(self.state, self.node_attribute, node_id)

    def _save_state(self) -> None:
        if self.store is None or self.state is None:
            return
        snapshot = asdict(self.state) if is_dataclass(self.state) else self.state
        self.store.save(self.context.task_id, snapshot)
