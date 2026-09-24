"""MQTT 入站 Adapter：把公开 method 交给 Gateway Dispatcher。"""

from __future__ import annotations

from typing import Any, Callable, Dict

from gateway.dispatcher import CommandDispatcher


class MqttCommandAdapter:
    """只负责把 MQTT data 交给分发器，不解释场景 payload。"""

    def __init__(self, dispatcher: CommandDispatcher) -> None:
        self._dispatcher = dispatcher

    def handlers(self) -> Dict[str, Callable[[Dict[str, Any]], Dict[str, Any]]]:
        return {
            method: self._wrap(method) for method in self._dispatcher.methods
        }

    def _wrap(
        self, method: str
    ) -> Callable[[Dict[str, Any]], Dict[str, Any]]:
        def handler(data: Dict[str, Any], *, bid: str = "", tid: str = "") -> Dict[str, Any]:
            payload = data if isinstance(data, dict) else {}
            return self._dispatcher.dispatch(method, payload, bid=bid, tid=tid)

        handler.__name__ = f"handle_{method}"
        return handler
