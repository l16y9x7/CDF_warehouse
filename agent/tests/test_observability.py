import json
import logging
import tempfile
import time
import unittest
from pathlib import Path

from agent.capabilities.common import CapabilityError, ErrorSource
from agent.contracts import AgentError, ExecutionContext, reportable_error_code, trace_value
from agent.debug.store import DebugStore
from agent.observability import JsonFormatter, LogStore, log_context, log_event


class ObservabilityTest(unittest.TestCase):
    def test_trace_value_redacts_sensitive_fields_and_binary_content(self):
        value = trace_value(
            {"target_id": "AGV_L", "token": "private", "image": b"pixels"}
        )
        self.assertEqual(value["target_id"], "AGV_L")
        self.assertEqual(value["token"], "[已脱敏]")
        self.assertEqual(value["image"], {"type": "bytes", "size": 6})

    def test_json_formatter_uses_chinese_message_and_redacts_sensitive_values(self):
        formatter = JsonFormatter()
        logger = logging.getLogger("test.observability")
        record = logger.makeRecord(
            logger.name, logging.ERROR, __file__, 1, "外部模块调用失败", (), None,
            extra={
                "event": "capability.failed",
                "status": "FAILED",
                "authorization": "Bearer private",
                "callback_url": "http://example/callback?token=private",
                "error_message": "访问 http://user:pass@example/path?token=private 失败",
            },
        )
        payload = json.loads(formatter.format(record))
        self.assertEqual(payload["message"], "外部模块调用失败")
        self.assertEqual(payload["authorization"], "[已脱敏]")
        self.assertEqual(payload["callback_url"], "[已脱敏]")
        self.assertNotIn("private", json.dumps(payload, ensure_ascii=False))
        self.assertNotIn("user:pass", json.dumps(payload, ensure_ascii=False))

    def test_log_store_filters_searches_and_paginates(self):
        with tempfile.TemporaryDirectory() as directory:
            store = LogStore(Path(directory) / "logs.db")
            formatter = JsonFormatter()
            for index in range(3):
                record = logging.LogRecord(
                    "test", logging.INFO, __file__, 1, f"任务执行说明 {index}", (), None
                )
                record.event = "skill.succeeded"
                record.task_id = "task-1"
                record.run_id = f"run-{index}"
                record.skill = "pick_sku"
                record.error_message = "需要人工检查" if index == 2 else None
                store.append(formatter.format_record(record), time.time() + index)

            first, cursor = store.query(task_id="task-1", limit=2)
            self.assertEqual(len(first), 2)
            self.assertIsNotNone(cursor)
            second, next_cursor = store.query(task_id="task-1", limit=2, cursor=int(cursor))
            self.assertEqual(len(second), 1)
            self.assertIsNone(next_cursor)
            searched, _ = store.query(search="人工检查")
            self.assertEqual([item["run_id"] for item in searched], ["run-2"])
            component, _ = store.query(component="pick_sku")
            self.assertEqual(len(component), 3)

    def test_execution_event_preserves_business_level_field(self):
        context = ExecutionContext("task-1")
        context.emit("skill.started", skill="prepare_pose", level="L1")
        self.assertEqual(context.events[0]["level"], "L1")

    def test_reportable_error_code_prefers_capability_and_agent_codes(self):
        self.assertEqual(
            reportable_error_code(CapabilityError("INVALID_INPUT", "bad hand")),
            "INVALID_INPUT",
        )
        self.assertEqual(
            reportable_error_code(AgentError("SKU_BARCODE_NOT_FOUND", "missing")),
            "SKU_BARCODE_NOT_FOUND",
        )
        self.assertEqual(reportable_error_code(ValueError("bad")), "ValueError")

    def test_trace_end_keeps_remote_capability_error_fields(self):
        context = ExecutionContext("task-1")
        span_id = context.trace_start(
            "capability", "manipulation", "rotate", input={"hand": "LEFT"}
        )
        context.trace_end(
            span_id,
            "FAILED",
            error=CapabilityError(
                "INVALID_INPUT",
                "扫码支持 bottle/RIGHT 或 box/LEFT",
                status_code=422,
                source=ErrorSource.REMOTE,
                operation="POST /manipulation/rotate",
            ),
        )
        span = context.trace_spans[0]
        self.assertEqual(span["error_code"], "INVALID_INPUT")
        self.assertEqual(span["error_type"], "CapabilityError")
        self.assertEqual(span["error_source"], "remote")
        self.assertEqual(span["http_status"], 422)
        self.assertEqual(span["error_operation"], "POST /manipulation/rotate")
        self.assertIn("扫码支持 bottle/RIGHT 或 box/LEFT", span["error_message"])

    def test_trace_end_records_local_capability_error_without_http_status(self):
        context = ExecutionContext("task-1")
        span_id = context.trace_start("capability", "camera", "capture")
        context.trace_end(
            span_id,
            "FAILED",
            error=CapabilityError("DEPTH_NOT_ALIGNED", "camera depth capture is not aligned"),
        )
        span = context.trace_spans[0]
        self.assertEqual(span["error_code"], "DEPTH_NOT_ALIGNED")
        self.assertEqual(span["error_source"], "local")
        self.assertNotIn("http_status", span)
        self.assertNotIn("error_operation", span)

    def test_debug_store_roundtrips_capability_error_source(self):
        with tempfile.TemporaryDirectory() as directory:
            store = DebugStore(Path(directory) / "debug.db")
            store.create_run("run-1", "mock", "capability", "rotate", {})
            context = ExecutionContext(
                "task-1",
                trace_handler=lambda span: store.attach_span("run-1", dict(span)),
            )
            span_id = context.trace_start("capability", "manipulation", "rotate")
            context.trace_end(
                span_id,
                "FAILED",
                error=CapabilityError(
                    "INVALID_INPUT",
                    "扫码支持 bottle/RIGHT 或 box/LEFT",
                    status_code=422,
                    source=ErrorSource.REMOTE,
                    operation="POST /manipulation/rotate",
                ),
            )
            span = store.trace("run-1")["spans"][0]
            self.assertEqual(span["error_code"], "INVALID_INPUT")
            self.assertEqual(span["error_source"], "remote")
            self.assertEqual(span["http_status"], 422)
            self.assertEqual(span["error_operation"], "POST /manipulation/rotate")

    def test_log_context_is_added_to_event(self):
        records = []

        class Handler(logging.Handler):
            def emit(self, record):
                records.append(record)

        logger = logging.getLogger("test.context")
        logger.handlers = [Handler()]
        logger.propagate = False
        logger.setLevel(logging.INFO)
        with log_context(task_id="task-1", run_id="run-1"):
            log_event(logger, logging.INFO, "test.event", "关联信息测试")
        self.assertEqual(records[0].task_id, "task-1")
        self.assertEqual(records[0].run_id, "run-1")


if __name__ == "__main__":
    unittest.main()
