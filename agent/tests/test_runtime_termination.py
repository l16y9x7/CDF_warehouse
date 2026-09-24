import json
import tempfile
import threading
import unittest
from pathlib import Path

import httpx

from agent.runtime import AgentRuntime, CallbackSender, RobotBusyError
from agent.task_store import SqliteTaskStore
from agent.workflows.base import NodeRunner


class BlockingWorkflow:
    def __init__(self) -> None:
        self.started = threading.Event()
        self.release = threading.Event()
        self.actions: list[str] = []

    def run(self, context, workflow_input):
        runner = NodeRunner(context)

        def first():
            self.actions.append("first")
            self.started.set()
            self.release.wait(2)

        runner.run("first", first)
        runner.run("second", lambda: self.actions.append("second"))
        return {}


class AgentRuntimeTerminationTest(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.store = SqliteTaskStore(Path(self.temp.name) / "tasks.db")
        self.workflow = BlockingWorkflow()
        self.callbacks: list[dict] = []

        def callback(request):
            self.callbacks.append(json.loads(request.content))
            return httpx.Response(204)

        sender = CallbackSender(
            self.store,
            backoff=0,
            client=httpx.Client(transport=httpx.MockTransport(callback)),
        )
        self.runtime = AgentRuntime(
            self.store, lambda _: self.workflow, callback_sender=sender, max_workers=1
        )

    def tearDown(self):
        self.workflow.release.set()
        self.runtime.shutdown()
        self.temp.cleanup()

    def test_terminates_after_current_node_and_rejects_another_task_while_busy(self):
        self.assertTrue(
            self.runtime.accept("one", "test", "http://callback", {"task_id": "one"}, None)
        )
        self.assertTrue(self.workflow.started.wait(1))

        self.assertEqual(self.runtime.terminate(), "one")
        self.assertEqual(self.runtime.terminate(), "one")
        with self.assertRaises(RobotBusyError):
            self.runtime.accept("two", "test", "http://callback", {"task_id": "two"}, None)

        self.workflow.release.set()
        self.runtime.wait("one")

        task = self.store.get("one")
        self.assertEqual(task["status"], "CANCELLED")
        self.assertEqual(task["error_code"], "CANCELLED")
        self.assertEqual(self.workflow.actions, ["first"])
        self.assertEqual(self.callbacks[-1]["status"], "CANCELLED")
        self.assertEqual(
            self.callbacks[-1]["info"],
            {"message": "任务已取消", "error": "任务已取消"},
        )

    def test_accepts_next_task_after_cancelled_task_finishes(self):
        self.runtime.accept("one", "test", "http://callback", {"task_id": "one"}, None)
        self.assertTrue(self.workflow.started.wait(1))
        self.runtime.terminate()
        self.workflow.release.set()
        self.runtime.wait("one")

        next_workflow = BlockingWorkflow()
        next_workflow.release.set()
        self.workflow = next_workflow
        self.assertTrue(
            self.runtime.accept("two", "test", "http://callback", {"task_id": "two"}, None)
        )
        self.runtime.wait("two")
        self.assertEqual(self.store.get("two")["status"], "SUCCEEDED")


if __name__ == "__main__":
    unittest.main()
