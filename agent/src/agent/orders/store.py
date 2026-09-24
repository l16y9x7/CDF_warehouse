import json
import sqlite3
import threading
import time
from contextlib import contextmanager
from pathlib import Path
from typing import Any

from .catalog import OrderProduct

ACTIVE_ORDER_STATUSES = ("RUNNING", "PAUSED", "CANCELLING")
TERMINAL_ORDER_STATUSES = frozenset({"SUCCEEDED", "CANCELLED"})


class OrderStore:
    def __init__(self, path: str | Path):
        self.path = str(path)
        self._lock = threading.RLock()
        self._initialize()

    def _connect(self) -> sqlite3.Connection:
        connection = sqlite3.connect(self.path, timeout=30)
        connection.row_factory = sqlite3.Row
        return connection

    @contextmanager
    def _connection(self):
        connection = self._connect()
        try:
            with connection:
                yield connection
        finally:
            connection.close()

    def _initialize(self) -> None:
        with self._connection() as connection:
            connection.executescript(
                """
                CREATE TABLE IF NOT EXISTS user_orders (
                    order_id TEXT PRIMARY KEY,
                    status TEXT NOT NULL,
                    stage TEXT NOT NULL DEFAULT 'ITEMS',
                    basket_row TEXT NOT NULL,
                    basket_column TEXT NOT NULL,
                    mock INTEGER NOT NULL DEFAULT 0,
                    current_unit_id INTEGER,
                    current_task_id TEXT,
                    current_skill TEXT,
                    progress_text TEXT,
                    error TEXT,
                    finish_attempts INTEGER NOT NULL DEFAULT 0,
                    created_at REAL NOT NULL,
                    updated_at REAL NOT NULL,
                    finished_at REAL
                );
                CREATE INDEX IF NOT EXISTS user_orders_created
                    ON user_orders(created_at DESC);
                CREATE TABLE IF NOT EXISTS user_order_units (
                    unit_id INTEGER PRIMARY KEY AUTOINCREMENT,
                    order_id TEXT NOT NULL,
                    sequence INTEGER NOT NULL,
                    sku_id TEXT NOT NULL,
                    name TEXT NOT NULL,
                    description TEXT NOT NULL,
                    image_url TEXT NOT NULL,
                    category TEXT NOT NULL,
                    agv_row TEXT NOT NULL,
                    agv_column TEXT NOT NULL,
                    status TEXT NOT NULL DEFAULT 'PENDING',
                    attempts INTEGER NOT NULL DEFAULT 0,
                    current_task_id TEXT,
                    current_skill TEXT,
                    progress_text TEXT,
                    error TEXT,
                    UNIQUE(order_id, sequence)
                );
                CREATE INDEX IF NOT EXISTS user_order_units_order
                    ON user_order_units(order_id, sequence);
                CREATE INDEX IF NOT EXISTS user_order_units_task
                    ON user_order_units(current_task_id);
                CREATE TABLE IF NOT EXISTS user_order_events (
                    event_id INTEGER PRIMARY KEY AUTOINCREMENT,
                    order_id TEXT NOT NULL,
                    event_type TEXT NOT NULL,
                    payload_json TEXT NOT NULL,
                    created_at REAL NOT NULL
                );
                CREATE INDEX IF NOT EXISTS user_order_events_order
                    ON user_order_events(order_id, event_id);
                """
            )
            columns = {
                row[1] for row in connection.execute("PRAGMA table_info(user_orders)")
            }
            if "mock" not in columns:
                connection.execute(
                    "ALTER TABLE user_orders ADD COLUMN mock INTEGER NOT NULL DEFAULT 0"
                )

    def recover_interrupted(self) -> None:
        now = time.time()
        with self._lock, self._connection() as connection:
            rows = connection.execute(
                "SELECT order_id FROM user_orders WHERE status IN ('RUNNING', 'CANCELLING')"
            ).fetchall()
            for row in rows:
                connection.execute(
                    """UPDATE user_orders SET status='PAUSED',
                       progress_text='服务重启，需人工确认机器人状态后重试',
                       error='服务执行中断', updated_at=? WHERE order_id=?""",
                    (now, row["order_id"]),
                )
                connection.execute(
                    """UPDATE user_order_units SET status='PAUSED', error='服务执行中断'
                       WHERE order_id=? AND status IN ('RUNNING', 'RETRYING')""",
                    (row["order_id"],),
                )
                self._insert_event(
                    connection,
                    row["order_id"],
                    "order.paused",
                    {"message": "服务重启，订单已暂停，请确认现场状态"},
                    now,
                )

    def create_order(
        self,
        order_id: str,
        items: list[tuple[OrderProduct, int, str, str]],
        basket_row: str,
        basket_column: str,
        *,
        mock: bool = False,
    ) -> dict[str, Any]:
        now = time.time()
        with self._lock, self._connection() as connection:
            active = connection.execute(
                "SELECT order_id FROM user_orders "
                "WHERE status IN ('RUNNING','PAUSED','CANCELLING') LIMIT 1"
            ).fetchone()
            if active is not None:
                raise RuntimeError("ACTIVE_ORDER_EXISTS")
            connection.execute(
                """INSERT INTO user_orders
                   (order_id,status,stage,basket_row,basket_column,mock,progress_text,
                    created_at,updated_at)
                   VALUES (?, 'RUNNING', 'ITEMS', ?, ?, ?, '订单已提交，等待机器人开始', ?, ?)""",
                (order_id, basket_row, basket_column, int(mock), now, now),
            )
            sequence = 0
            for product, quantity, agv_row, agv_column in items:
                for _ in range(quantity):
                    sequence += 1
                    connection.execute(
                        """INSERT INTO user_order_units
                           (order_id,sequence,sku_id,name,description,image_url,category,
                            agv_row,agv_column)
                           VALUES (?,?,?,?,?,?,?,?,?)""",
                        (
                            order_id, sequence, product.sku_id, product.name,
                            product.description, product.image_url, product.category,
                            agv_row, agv_column,
                        ),
                    )
            self._insert_event(
                connection,
                order_id,
                "order.created",
                {"message": "订单已创建", "total_items": sequence},
                now,
            )
        result = self.get_order(order_id)
        assert result is not None
        return result

    def active_order(self) -> dict[str, Any] | None:
        with self._connection() as connection:
            row = connection.execute(
                """SELECT order_id FROM user_orders
                   WHERE status IN ('RUNNING','PAUSED','CANCELLING')
                   ORDER BY created_at DESC LIMIT 1"""
            ).fetchone()
        return None if row is None else self.get_order(row["order_id"])

    def latest_order(self) -> dict[str, Any] | None:
        with self._connection() as connection:
            row = connection.execute(
                "SELECT order_id FROM user_orders ORDER BY created_at DESC LIMIT 1"
            ).fetchone()
        return None if row is None else self.get_order(row["order_id"])

    def order_id_for_task(self, task_id: str) -> str | None:
        with self._connection() as connection:
            row = connection.execute(
                "SELECT order_id FROM user_orders WHERE current_task_id=?",
                (task_id,),
            ).fetchone()
        return None if row is None else str(row["order_id"])

    def get_order(self, order_id: str) -> dict[str, Any] | None:
        with self._connection() as connection:
            order = connection.execute(
                "SELECT * FROM user_orders WHERE order_id=?", (order_id,)
            ).fetchone()
            if order is None:
                return None
            units = connection.execute(
                "SELECT * FROM user_order_units WHERE order_id=? ORDER BY sequence", (order_id,)
            ).fetchall()
        return self._serialize_order(dict(order), [dict(row) for row in units])

    def units(self, order_id: str) -> list[dict[str, Any]]:
        with self._connection() as connection:
            rows = connection.execute(
                "SELECT * FROM user_order_units WHERE order_id=? ORDER BY sequence", (order_id,)
            ).fetchall()
        return [dict(row) for row in rows]

    def update_order(self, order_id: str, **values: Any) -> None:
        if not values:
            return
        allowed = {
            "status", "stage", "current_unit_id", "current_task_id", "current_skill",
            "progress_text", "error", "finish_attempts", "finished_at",
        }
        if set(values) - allowed:
            raise ValueError("unsupported order update")
        values["updated_at"] = time.time()
        assignments = ", ".join(f"{key}=?" for key in values)
        with self._lock, self._connection() as connection:
            connection.execute(
                f"UPDATE user_orders SET {assignments} WHERE order_id=?",  # noqa: S608
                (*values.values(), order_id),
            )

    def update_unit(self, unit_id: int, **values: Any) -> None:
        allowed = {
            "status", "attempts", "current_task_id", "current_skill", "progress_text", "error",
        }
        if not values or set(values) - allowed:
            if values:
                raise ValueError("unsupported order unit update")
            return
        assignments = ", ".join(f"{key}=?" for key in values)
        with self._lock, self._connection() as connection:
            connection.execute(
                f"UPDATE user_order_units SET {assignments} WHERE unit_id=?",  # noqa: S608
                (*values.values(), unit_id),
            )

    def increment_unit_attempt(self, unit_id: int, task_id: str) -> int:
        with self._lock, self._connection() as connection:
            connection.execute(
                """UPDATE user_order_units SET attempts=attempts+1, current_task_id=?,
                   status='RUNNING', error=NULL WHERE unit_id=?""",
                (task_id, unit_id),
            )
            row = connection.execute(
                "SELECT attempts FROM user_order_units WHERE unit_id=?", (unit_id,)
            ).fetchone()
        return int(row["attempts"])

    def event(self, order_id: str, event_type: str, payload: dict[str, Any]) -> int:
        with self._lock, self._connection() as connection:
            return self._insert_event(connection, order_id, event_type, payload, time.time())

    def events_after(self, order_id: str, event_id: int, limit: int = 100) -> list[dict[str, Any]]:
        with self._connection() as connection:
            rows = connection.execute(
                """SELECT * FROM user_order_events WHERE order_id=? AND event_id>?
                   ORDER BY event_id LIMIT ?""",
                (order_id, event_id, limit),
            ).fetchall()
        return [
            {
                "event_id": row["event_id"],
                "type": row["event_type"],
                "payload": json.loads(row["payload_json"]),
                "created_at": row["created_at"],
            }
            for row in rows
        ]

    def retry_paused(self, order_id: str) -> None:
        now = time.time()
        with self._lock, self._connection() as connection:
            order = connection.execute(
                "SELECT status,stage FROM user_orders WHERE order_id=?", (order_id,)
            ).fetchone()
            if order is None:
                raise KeyError(order_id)
            if order["status"] != "PAUSED":
                raise RuntimeError("ORDER_NOT_PAUSED")
            if order["stage"] == "ITEMS":
                connection.execute(
                    """UPDATE user_order_units SET status='PENDING', error=NULL
                       WHERE unit_id=(SELECT current_unit_id FROM user_orders WHERE order_id=?)""",
                    (order_id,),
                )
            connection.execute(
                """UPDATE user_orders SET status='RUNNING', progress_text='正在重新执行',
                   error=NULL, updated_at=? WHERE order_id=?""",
                (now, order_id),
            )
            self._insert_event(
                connection, order_id, "order.retry_requested", {"message": "已请求重试"}, now
            )

    def _insert_event(
        self,
        connection: sqlite3.Connection,
        order_id: str,
        event_type: str,
        payload: dict[str, Any],
        created_at: float,
    ) -> int:
        cursor = connection.execute(
            """INSERT INTO user_order_events(order_id,event_type,payload_json,created_at)
               VALUES (?,?,?,?)""",
            (order_id, event_type, json.dumps(payload, ensure_ascii=False), created_at),
        )
        return int(cursor.lastrowid)

    def _serialize_order(
        self, order: dict[str, Any], units: list[dict[str, Any]]
    ) -> dict[str, Any]:
        groups: dict[str, dict[str, Any]] = {}
        status_priority = {
            "PAUSED": 6, "RUNNING": 5, "RETRYING": 4, "PENDING": 3,
            "CANCELLED": 2, "SUCCEEDED": 1,
        }
        for unit in units:
            sku_id = unit["sku_id"]
            if sku_id not in groups:
                groups[sku_id] = {
                    "sku_id": sku_id,
                    "name": unit["name"],
                    "description": unit["description"],
                    "image_url": unit["image_url"],
                    "category": unit["category"],
                    "agv_row": unit["agv_row"],
                    "agv_column": unit["agv_column"],
                    "quantity": 0,
                    "completed_quantity": 0,
                    "status": unit["status"],
                }
            group = groups[sku_id]
            group["quantity"] += 1
            group["completed_quantity"] += int(unit["status"] == "SUCCEEDED")
            if status_priority.get(unit["status"], 0) > status_priority.get(group["status"], 0):
                group["status"] = unit["status"]
        completed = sum(unit["status"] == "SUCCEEDED" for unit in units)
        total_steps = len(units) or 1
        finished_steps = completed
        current_sequence = next(
            (unit["sequence"] for unit in units if unit["unit_id"] == order["current_unit_id"]),
            None,
        )
        order["mock"] = bool(order.get("mock"))
        return {
            **order,
            "items": list(groups.values()),
            "units": [
                {
                    key: unit[key]
                    for key in (
                        "unit_id", "sequence", "sku_id", "name", "status", "attempts",
                        "current_skill", "progress_text", "error",
                    )
                }
                for unit in units
            ],
            "total_items": len(units),
            "completed_items": completed,
            "current_sequence": current_sequence,
            "progress_percent": round(finished_steps / total_steps * 100),
        }
