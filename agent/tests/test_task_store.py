import tempfile
import unittest
from pathlib import Path

import httpx

from agent.runtime import CallbackSender
from agent.task_store import SqliteTaskStore


class SqliteTaskStoreTest(unittest.TestCase):
    def test_persists_and_seals_interrupted_tasks(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "tasks.db"
            store = SqliteTaskStore(path)
            self.assertTrue(store.create("task", "review", "http://callback", {"task_id": "task"}))
            self.assertFalse(store.create("task", "review", "http://callback", {}))
            store.save("task", {"last_node": "T2-N03c"})
            sealed = SqliteTaskStore(path).mark_interrupted()
            self.assertEqual(len(sealed), 1)
            task = SqliteTaskStore(path).get("task")
            self.assertEqual(task["status"], "WAITING_CONFIRMATION")
            self.assertEqual(task["error_code"], "PROCESS_INTERRUPTED")
            self.assertEqual(task["snapshot"], {"last_node": "T2-N03c"})

    def test_callback_retries_and_uses_terminal_payload(self):
        calls = []

        def respond(request):
            calls.append(request)
            return httpx.Response(500 if len(calls) < 3 else 204)

        with tempfile.TemporaryDirectory() as directory:
            store = SqliteTaskStore(Path(directory) / "tasks.db")
            client = httpx.Client(transport=httpx.MockTransport(respond))
            CallbackSender(store, attempts=3, backoff=0, client=client).send(
                {
                    "task_id": "task",
                    "callback_url": "http://callback/result",
                    "status": "WAITING_CONFIRMATION",
                    "error_code": "ACTION_RESULT_UNKNOWN",
                }
            )
            self.assertEqual(len(calls), 3)
            self.assertEqual(
                calls[-1].read(),
                b'{"task_id":"task","status":"WAITING_CONFIRMATION","error_code":"ACTION_RESULT_UNKNOWN"}',
            )
            client.close()


if __name__ == "__main__":
    unittest.main()
