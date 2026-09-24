import time
import unittest
from dataclasses import dataclass

from agent.contracts import ExecutionContext
from agent.workflows.base import (
    InMemoryWorkflowStateStore,
    NodeRunner,
    WorkflowCancelled,
    WorkflowTimeout,
)


@dataclass
class ExampleState:
    current_node: str | None = None
    value: int = 0


class WorkflowRuntimeTest(unittest.TestCase):
    def test_cancelled_context_does_not_execute_node(self):
        context = ExecutionContext("task-1", cancelled=True)
        with self.assertRaises(WorkflowCancelled):
            NodeRunner(context).run("node", lambda: self.fail("must not run"))

    def test_expired_deadline_does_not_execute_node(self):
        context = ExecutionContext("task-1", deadline=time.monotonic() - 1)
        with self.assertRaises(WorkflowTimeout):
            NodeRunner(context).run("node", lambda: self.fail("must not run"))

    def test_node_runner_persists_node_and_successful_state_transition(self):
        context = ExecutionContext("task-1")
        store = InMemoryWorkflowStateStore()
        state = ExampleState()
        runner = NodeRunner(context, store, state=state, node_attribute="current_node")

        runner.run(
            "node-1",
            lambda: 42,
            on_success=lambda value: setattr(state, "value", value),
        )

        self.assertEqual(store.load("task-1"), {"current_node": "node-1", "value": 42})
        self.assertEqual(
            [record["status"] for record in store.nodes["task-1"]],
            ["RUNNING", "SUCCEEDED"],
        )


if __name__ == "__main__":
    unittest.main()
