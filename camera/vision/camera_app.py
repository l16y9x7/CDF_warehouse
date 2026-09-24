"""相机进程入口（vision 组件，默认 :8003）。"""

from __future__ import annotations

import argparse
import logging
from typing import Any, Dict, Optional

from vision.config import load_config
from vision.http_app import create_camera_app
from vision.http_server import UvicornHttpServer
from vision.service import CameraService

LOGGER = logging.getLogger(__name__)


class CameraApp:
    def __init__(self, config: Dict[str, Any]) -> None:
        self.config = config
        self.service = CameraService(config)
        camera_cfg = config.get("camera") or {}
        self.host = str(camera_cfg.get("host") or "0.0.0.0")
        self.port = int(camera_cfg["port"]) if "port" in camera_cfg else 8003
        self.api = create_camera_app(self.service)
        self._http: Optional[UvicornHttpServer] = None

    def serve_forever(self) -> None:
        LOGGER.info(
            "camera started: adapter=%s http://%s:%s docs=/docs",
            self.config.get("adapter") or "tianji",
            self.host,
            self.port,
        )
        self._http = UvicornHttpServer(self.api, host=self.host, port=self.port)
        self._http.serve_forever()
        LOGGER.info("camera stopped")


def main(argv: Optional[list[str]] = None) -> int:
    parser = argparse.ArgumentParser(description="Vision camera capability")
    parser.add_argument("--config", default="")
    parser.add_argument("--log-level", default="INFO")
    args = parser.parse_args(argv)
    level = getattr(logging, str(args.log_level or "INFO").upper(), logging.INFO)
    logging.basicConfig(level=level, format="%(asctime)s %(levelname)s %(name)s: %(message)s")
    LOGGER.info("camera process starting")
    CameraApp(load_config(args.config or None)).serve_forever()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
