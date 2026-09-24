"""导航能力进程入口：一条命令开 :8001，并按配置后台开中免 :8081。"""

from __future__ import annotations

import argparse
import logging
import sys
from pathlib import Path
from typing import Any, Dict, Optional

from navigation.config import load_config
from navigation.http_app import create_app
from navigation.http_server import UvicornHttpServer
from navigation.ros_face import RosStatusBridge
from navigation.service import NavigationService

LOGGER = logging.getLogger(__name__)


class NavigationApp:
    def __init__(self, config: Dict[str, Any]) -> None:
        self.config = config
        self.service = NavigationService(config)
        http_cfg = config.get("http") or {}
        self.host = str(http_cfg.get("host") or "0.0.0.0")
        self.port = int(http_cfg["port"]) if "port" in http_cfg else 8001
        adapter_name = str(config.get("adapter") or "tianji").strip().lower()
        # 天机 ROS face 订 retail_nav；珞石 Adapter 自己订 sr_amr_control。
        self._ros = (
            RosStatusBridge(self.service.adapter, config)
            if adapter_name == "tianji"
            else None
        )
        self.api = create_app(self.service)
        self._http: Optional[UvicornHttpServer] = None
        self._zhongmian_http: Optional[UvicornHttpServer] = None

    def serve_forever(self) -> None:
        if self._ros is not None:
            self._ros.start()
        self._start_zhongmian()
        LOGGER.info(
            "navigation started: adapter=%s http://%s:%s docs=/docs",
            self.config.get("adapter") or "tianji",
            self.host,
            self.port,
        )
        try:
            # 能力层占主线程；中免已在后台线程。Ctrl+C 会走进 finally 两边一起停。
            self._http = UvicornHttpServer(self.api, host=self.host, port=self.port)
            self._http.serve_forever()
        finally:
            if self._zhongmian_http is not None:
                self._zhongmian_http.shutdown()
                self._zhongmian_http = None
            if self._ros is not None:
                self._ros.stop()
            close = getattr(self.service.adapter, "close", None)
            if callable(close):
                close()
            LOGGER.info("navigation stopped")

    def _start_zhongmian(self) -> None:
        # Dc：业务层和能力层一起起。天机厂家自己占用 :8081，不能再拉中免。
        raw = self.config.get("zhongmian")
        if raw is False:
            return
        zm = raw if isinstance(raw, dict) else {}
        if zm.get("enabled", True) is False:
            return
        adapter_name = str(self.config.get("adapter") or "").strip().lower()
        if adapter_name == "tianji":
            LOGGER.warning("skip zhongmian: tianji adapter already uses :8081")
            return
        try:
            _ensure_apps_path()
            from zhongmian.app import build_server
            from zhongmian.config import load_config as load_zhongmian

            cfg_path = str(zm.get("config") or "apps/zhongmian/zhongmian.json")
            self._zhongmian_http = build_server(load_zhongmian(cfg_path))
            self._zhongmian_http.start_background(name="zhongmian-http")
        except Exception:
            LOGGER.exception("zhongmian failed to start")
            self._zhongmian_http = None


def _ensure_apps_path() -> None:
    # apps/zhongmian 是独立包，启动时挂上 sys.path，现场不必再记一层 PYTHONPATH。
    apps = Path(__file__).resolve().parent.parent / "apps"
    path = str(apps)
    if apps.is_dir() and path not in sys.path:
        sys.path.insert(0, path)


def main(argv: Optional[list[str]] = None) -> int:
    parser = argparse.ArgumentParser(description="Navigation capability")
    parser.add_argument("--config", default="", help="navigation JSON config path")
    parser.add_argument("--log-level", default="INFO")
    args = parser.parse_args(argv)
    level = getattr(logging, str(args.log_level or "INFO").upper(), logging.INFO)
    logging.basicConfig(level=level, format="%(asctime)s %(levelname)s %(name)s: %(message)s")
    LOGGER.info("navigation process starting")
    NavigationApp(load_config(args.config or None)).serve_forever()
    return 0
