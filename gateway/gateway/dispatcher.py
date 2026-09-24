"""命令分发：校验、去重、转发场景 StartTask/StopTask/RecoverTask。"""

from __future__ import annotations

import logging
import threading
import time
from typing import Any, Dict, List, Mapping, Optional

from gateway.logging_utils import log_safe
from gateway.records import CommandLog, CommandRecord, TaskStateStore, now_ms
from gateway.routing import (
    RouteSpec,
    enabled_scenarios,
    load_routes,
    resolve_scenario,
)
from gateway.scenario_client import ScenarioClient, ScenarioResponse

LOGGER = logging.getLogger(__name__)


class CommandDispatcher:
    def __init__(
        self,
        config: Mapping[str, Any],
        *,
        scenario_client: Optional[ScenarioClient] = None,
        command_log: Optional[CommandLog] = None,
        task_state: Optional[TaskStateStore] = None,
    ) -> None:
        self._config = config
        self._routes = load_routes(config)
        self._scenarios = enabled_scenarios(config)
        self._default_scenario = str(config.get("default_scenario") or "smt")
        self._client = scenario_client or ScenarioClient(
            timeout_sec=float(config.get("http_timeout_sec") or 8.0)
        )
        self.command_log = command_log or CommandLog()
        self.task_state = task_state or TaskStateStore()
        mqtt_cfg = config.get("mqtt") or {}
        self._dedupe_ttl_sec = float(mqtt_cfg.get("command_dedupe_ttl_sec") or 300)
        self._dedupe_max = int(mqtt_cfg.get("command_dedupe_max_entries") or 2048)
        self._dedupe_lock = threading.Lock()
        self._dedupe: Dict[str, Dict[str, Any]] = {}
        LOGGER.info(
            "dispatcher ready: default_scenario=%s scenarios=%s methods=%s",
            self._default_scenario,
            ",".join(self.scenario_names) or "(none)",
            ",".join(self.methods),
        )

    @property
    def methods(self) -> tuple[str, ...]:
        return tuple(self._routes)

    @property
    def scenario_names(self) -> tuple[str, ...]:
        return tuple(sorted(self._scenarios))

    def dispatch(
        self,
        method: str,
        data: Any,
        *,
        bid: str = "",
        tid: str = "",
    ) -> Dict[str, Any]:
        method = str(method or "").strip()
        payload = data if isinstance(data, dict) else {}
        dedupe_key = _dedupe_key(method, tid, bid)
        if dedupe_key:
            cached = self._check_duplicate_or_mark(dedupe_key)
            if cached is not None:
                status = str(cached.get("status") or "")
                LOGGER.info(
                    "duplicate command: method=%s tid=%s bid=%s status=%s",
                    method,
                    tid,
                    bid,
                    status,
                )
                return dict(cached.get("result") or _processing_reply())

        LOGGER.info(
            "dispatch: method=%s tid=%s bid=%s task_id=%s payload=%s",
            method,
            tid,
            bid,
            str(payload.get("task_id") or ""),
            log_safe(payload),
        )

        try:
            result = self._dispatch_uncached(method, payload, bid=bid, tid=tid)
        except Exception as exc:
            LOGGER.exception("gateway dispatch failed: method=%s", method)
            result = _reject(str(exc), error_code="GATEWAY_INTERNAL_ERROR")
        if dedupe_key:
            self._mark_done(dedupe_key, result)
        return result

    def _dispatch_uncached(
        self,
        method: str,
        payload: Dict[str, Any],
        *,
        bid: str,
        tid: str,
    ) -> Dict[str, Any]:
        route = self._routes.get(method)
        if route is None:
            LOGGER.warning("unknown method: %s", method)
            return _reject(f"Unknown method: {method}", error_code="GATEWAY_UNKNOWN_METHOD")

        task_id = str(payload.get("task_id") or "").strip()
        if not task_id:
            LOGGER.warning("reject command without task_id: method=%s tid=%s", method, tid)
            result = _reject("task_id is required", error_code="GATEWAY_REQUEST_INVALID")
            self._record(method, bid, tid, "", "", result)
            return result

        if route.all_scenarios:
            return self._dispatch_all(method, route, payload, bid=bid, tid=tid)

        scenario = resolve_scenario(
            route, payload, default_scenario=self._default_scenario
        )
        base_url = self._scenarios.get(scenario)
        if not base_url:
            LOGGER.warning(
                "scenario not configured: method=%s scenario=%s task_id=%s",
                method,
                scenario,
                task_id,
            )
            result = _reject(
                f"scenario {scenario} is not configured",
                error_code="GATEWAY_SCENARIO_NOT_CONFIGURED",
                extra={"task_id": task_id, "scenario": scenario},
            )
            self._record(method, bid, tid, task_id, scenario, result)
            return result

        body = _scenario_body(
            task_id=task_id,
            scenario=scenario,
            idempotency_key=str(payload.get("idempotency_key") or tid or bid),
            payload=payload,
        )
        LOGGER.info(
            "forward to scenario: method=%s scenario=%s path=%s task_id=%s url=%s",
            method,
            scenario,
            route.path,
            task_id,
            base_url,
        )
        response = self._client.post(base_url, route.path, body)
        result = _reply_from_response(
            response,
            task_id=task_id,
            scenario=scenario,
            accepted_status="running",
        )
        if result.get("code") == 0:
            LOGGER.info(
                "command accepted: method=%s scenario=%s task_id=%s status=%s",
                method,
                scenario,
                task_id,
                result.get("data", {}).get("status"),
            )
        else:
            LOGGER.warning(
                "command rejected: method=%s scenario=%s task_id=%s error=%s message=%s",
                method,
                scenario,
                task_id,
                result.get("data", {}).get("error"),
                result.get("message"),
            )
        self._record(method, bid, tid, task_id, scenario, result)
        if result.get("code") == 0:
            if route.path == "/tasks" and not route.all_scenarios:
                self.task_state.on_accepted(task_id)
            elif method == "task_stop":
                self.task_state.on_stop_accepted(task_id)
        return result

    def _dispatch_all(
        self,
        method: str,
        route: RouteSpec,
        payload: Dict[str, Any],
        *,
        bid: str,
        tid: str,
    ) -> Dict[str, Any]:
        task_id = str(payload.get("task_id") or "").strip()
        if not self._scenarios:
            LOGGER.warning("task_stop with no enabled scenario: task_id=%s", task_id)
            result = _reject(
                "no scenario is configured",
                error_code="GATEWAY_SCENARIO_NOT_CONFIGURED",
                extra={"task_id": task_id},
            )
            self._record(method, bid, tid, task_id, "", result)
            return result

        LOGGER.info(
            "fanout stop: task_id=%s scenarios=%s",
            task_id,
            ",".join(self._scenarios),
        )
        fanout: List[Dict[str, Any]] = []
        any_accepted = False
        last_error = ""
        last_error_code = ""
        for name, base_url in self._scenarios.items():
            body = _scenario_body(
                task_id=task_id,
                scenario=name,
                idempotency_key=str(payload.get("idempotency_key") or tid or bid),
                payload=payload,
            )
            response = self._client.post(base_url, route.path, body)
            fanout.append(
                {
                    "scenario": name,
                    "accepted": response.accepted,
                    "error": response.error,
                    "error_code": response.error_code,
                }
            )
            if response.accepted:
                any_accepted = True
            else:
                last_error = response.error
                last_error_code = response.error_code

        if any_accepted:
            LOGGER.info("task stop accepted: task_id=%s fanout=%s", task_id, fanout)
            result = {
                "code": 0,
                "message": "task stop accepted",
                "data": {
                    "result": 0,
                    "task_id": task_id,
                    "status": "accepted",
                    "scenarios": fanout,
                },
            }
            self.task_state.on_stop_accepted(task_id)
        else:
            LOGGER.warning(
                "task stop rejected by all scenarios: task_id=%s fanout=%s",
                task_id,
                fanout,
            )
            result = _reject(
                last_error or "all scenarios rejected stop",
                error_code=last_error_code or "GATEWAY_SCENARIO_REJECTED",
                extra={"task_id": task_id, "scenarios": fanout},
            )
        self._record(method, bid, tid, task_id, "*", result)
        return result

    def _record(
        self,
        method: str,
        bid: str,
        tid: str,
        task_id: str,
        scenario: str,
        result: Dict[str, Any],
    ) -> None:
        accepted = int(result.get("code") if "code" in result else -1) == 0
        data = result.get("data") if isinstance(result.get("data"), dict) else {}
        self.command_log.add(
            CommandRecord(
                timestamp_ms=now_ms(),
                method=method,
                bid=bid,
                tid=tid,
                task_id=task_id,
                scenario=scenario,
                accepted=accepted,
                code=int(result.get("code") if "code" in result else -1),
                message=str(result.get("message") or ""),
                error_code=str(data.get("error") or data.get("error_code") or ""),
            )
        )

    def _check_duplicate_or_mark(self, key: str) -> Optional[Dict[str, Any]]:
        now_ts = time.time()
        with self._dedupe_lock:
            self._prune_locked(now_ts)
            existing = self._dedupe.get(key)
            if existing:
                existing["updated_at"] = now_ts
                return dict(existing)
            self._dedupe[key] = {
                "status": "processing",
                "updated_at": now_ts,
                "result": _processing_reply(),
            }
            return None

    def _mark_done(self, key: str, result: Dict[str, Any]) -> None:
        now_ts = time.time()
        with self._dedupe_lock:
            self._dedupe[key] = {
                "status": "done",
                "updated_at": now_ts,
                "result": dict(result),
            }
            self._prune_locked(now_ts)

    def _prune_locked(self, now_ts: float) -> None:
        ttl = max(1.0, self._dedupe_ttl_sec)
        expired = [
            key
            for key, value in self._dedupe.items()
            if (now_ts - float(value.get("updated_at", now_ts))) > ttl
        ]
        for key in expired:
            self._dedupe.pop(key, None)
        if len(self._dedupe) <= self._dedupe_max:
            return
        sorted_items = sorted(
            self._dedupe.items(),
            key=lambda item: float(item[1].get("updated_at", 0.0)),
        )
        overflow = len(self._dedupe) - self._dedupe_max
        for index in range(max(0, overflow)):
            self._dedupe.pop(sorted_items[index][0], None)


def _scenario_body(
    *,
    task_id: str,
    scenario: str,
    idempotency_key: str,
    payload: Mapping[str, Any],
) -> Dict[str, Any]:
    return {
        "task_id": task_id,
        "scenario": scenario,
        "idempotency_key": idempotency_key,
        "payload": dict(payload),
    }


def _reply_from_response(
    response: ScenarioResponse,
    *,
    task_id: str,
    scenario: str,
    accepted_status: str,
) -> Dict[str, Any]:
    if response.accepted:
        status = str(response.body.get("status") or accepted_status)
        return {
            "code": 0,
            "message": str(response.body.get("message") or "accepted"),
            "data": {
                "result": 0,
                "task_id": task_id,
                "scenario": scenario,
                "status": status,
            },
        }
    error = response.error or "scenario rejected"
    error_code = response.error_code or "GATEWAY_SCENARIO_REJECTED"
    return _reject(
        error,
        error_code=error_code,
        extra={"task_id": task_id, "scenario": scenario},
        code=-1,
    )


def _reject(
    message: str,
    *,
    error_code: str,
    extra: Optional[Dict[str, Any]] = None,
    code: int = -1,
) -> Dict[str, Any]:
    data: Dict[str, Any] = {"result": -1, "error": error_code}
    if extra:
        data.update(extra)
    return {"code": code, "message": message, "data": data}


def _processing_reply() -> Dict[str, Any]:
    return {
        "code": 0,
        "message": "command is processing",
        "data": {"result": 0, "processing": True},
    }


def _dedupe_key(method: str, tid: str, bid: str) -> str:
    tid_val = str(tid or "").strip()
    bid_val = str(bid or "").strip()
    if tid_val:
        return f"{method}|tid|{tid_val}"
    if bid_val:
        return f"{method}|bid|{bid_val}"
    return ""
