"""中免 :8081 怎么起来：可 python -m zhongmian，也可被 navigation 后台拉起。"""

from __future__ import annotations

import argparse
import logging
from typing import Any, Dict, Optional

from navigation.http_server import UvicornHttpServer
from zhongmian.config import load_config
from zhongmian.http_app import create_app
from zhongmian.service import NavigationFacade

LOGGER = logging.getLogger(__name__)


def build_server(config: Dict[str, Any]) -> UvicornHttpServer:
    http_cfg = config.get("http") or {}
    host = str(http_cfg.get("host") or "0.0.0.0")
    port = int(http_cfg.get("port") or 8081)
    facade = NavigationFacade(config)
    LOGGER.info("zhongmian started: http://%s:%s docs=/docs", host, port)
    return UvicornHttpServer(create_app(facade), host=host, port=port)


def serve(config: Dict[str, Any]) -> None:
    build_server(config).serve_forever()


def main(argv: Optional[list[str]] = None) -> int:
    parser = argparse.ArgumentParser(description="Zhongmian Agent navigation API")
    parser.add_argument("--config", default="", help="zhongmian JSON config path")
    parser.add_argument("--log-level", default="INFO")
    args = parser.parse_args(argv)
    level = getattr(logging, str(args.log_level or "INFO").upper(), logging.INFO)
    logging.basicConfig(level=level, format="%(asctime)s %(levelname)s %(name)s: %(message)s")
    serve(load_config(args.config or None))
    return 0
