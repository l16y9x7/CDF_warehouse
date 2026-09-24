"""用 uvicorn 跑 FastAPI。"""

from __future__ import annotations

import logging
import threading
import time
from typing import Any, Optional

import uvicorn

LOGGER = logging.getLogger(__name__)


class UvicornHttpServer:
    def __init__(self, app: Any, *, host: str, port: int, log_level: str = "info") -> None:
        self.host = host
        self.port = int(port)
        self._server = uvicorn.Server(
            uvicorn.Config(
                app,
                host=host,
                port=self.port,
                log_level=log_level,
                access_log=False,
            )
        )
        self._thread: Optional[threading.Thread] = None

    def serve_forever(self) -> None:
        LOGGER.info("HTTP listening on %s:%s", self.host, self.port)
        self._server.run()

    def start_background(self, name: str = "uvicorn-http") -> None:
        self._server.install_signal_handlers = lambda: None  # type: ignore[method-assign]
        self._thread = threading.Thread(target=self._server.run, name=name, daemon=True)
        self._thread.start()
        deadline = time.monotonic() + 5.0
        while time.monotonic() < deadline:
            if self._server.started:
                break
            time.sleep(0.02)
        LOGGER.info("HTTP listening on %s:%s", self.host, self.port)

    def shutdown(self) -> None:
        self._server.should_exit = True
        if self._thread is not None:
            self._thread.join(timeout=2.0)
            self._thread = None
        LOGGER.info("HTTP stopped")
