"""用 uvicorn 跑 FastAPI。能力层 :8001 走 serve_forever；中免 :8081 走 start_background。"""

from __future__ import annotations

import logging
import socket
import threading
import time
from typing import Any, Optional

import uvicorn

LOGGER = logging.getLogger(__name__)


class UvicornHttpServer:
    def __init__(self, app: Any, *, host: str, port: int, log_level: str = "info") -> None:
        self.host = host
        self.port = int(port)
        self._sock: Optional[socket.socket] = None
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
        sock = self._bind()
        LOGGER.info("HTTP listening on %s:%s", self.host, self.port)
        self._server.run(sockets=[sock])

    def start_background(self, name: str = "uvicorn-http") -> None:
        # 关掉 uvicorn 自己抢 SIGINT，否则中免线程会把主进程信号处理搞乱。
        sock = self._bind()
        self._server.install_signal_handlers = lambda: None  # type: ignore[method-assign]
        self._thread = threading.Thread(
            target=self._server.run,
            kwargs={"sockets": [sock]},
            name=name,
            daemon=True,
        )
        self._thread.start()
        deadline = time.monotonic() + 5.0
        while time.monotonic() < deadline:
            if self._server.started:
                LOGGER.info("HTTP listening on %s:%s", self.host, self.port)
                return
            time.sleep(0.02)
        raise RuntimeError(f"HTTP failed to start on {self.host}:{self.port}")

    def shutdown(self) -> None:
        self._server.should_exit = True
        if self._thread is not None:
            self._thread.join(timeout=2.0)
            self._thread = None
        if self._sock is not None:
            try:
                self._sock.close()
            except OSError:
                pass
            self._sock = None
        LOGGER.info("HTTP stopped")

    def _bind(self) -> socket.socket:
        # 自己绑 IPv4。uvicorn 对 0.0.0.0 会 IPv4/IPv6 各绑一次，第二次就是 Errno 98。
        host = "0.0.0.0" if self.host in ("", "::", "*") else self.host
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        try:
            sock.bind((host, self.port))
        except OSError:
            sock.close()
            raise
        sock.listen(2048)
        sock.set_inheritable(True)
        self._sock = sock
        return sock
