"""推流进程入口（vision 组件，默认 :8005）。

Tianji/SMT 对齐：Media 对应 RosTopicVideoAdapter — 只订 ROS Topic / 拉 Owner HTTP，
绝不打开 /dev/video* 或 Orbbec/RealSense SDK。设备唯一 owner 是 head-rgbd / right-rgbd。
"""

from __future__ import annotations

import argparse
import logging
from typing import Any, Dict, Optional

from vision.config import load_config
from vision.http_app import create_media_app
from vision.http_server import UvicornHttpServer
from vision.service import CameraService, MediaService

LOGGER = logging.getLogger(__name__)


class MediaApp:
    def __init__(self, config: Dict[str, Any]) -> None:
        self.config = config
        push = ((config.get("media") or {}).get("push") or {})
        if str(push.get("source", "http")).strip().lower() == "ros":
            from vision.ownership import SOURCE_RUNTIME_UNITS, ensure_ros_media_subscriber_only
            ensure_ros_media_subscriber_only()
            LOGGER.info(
                "media ros subscriber-only: source_units=%s media_unit=%s",
                {k: SOURCE_RUNTIME_UNITS[k] for k in ("head", "hand_right")},
                SOURCE_RUNTIME_UNITS["media"],
            )
        self.service = MediaService(CameraService(config), config)
        media_cfg = config.get("media") or {}
        self.host = str(media_cfg.get("host") or "0.0.0.0")
        self.port = int(media_cfg["port"]) if "port" in media_cfg else 8005
        self.api = create_media_app(self.service)
        self._http: Optional[UvicornHttpServer] = None

    def serve_forever(self) -> None:
        LOGGER.info("media started: http://%s:%s docs=/docs", self.host, self.port)
        self._http = UvicornHttpServer(self.api, host=self.host, port=self.port)
        self._http.serve_forever()
        LOGGER.info("media stopped")


def main(argv: Optional[list[str]] = None) -> int:
    parser = argparse.ArgumentParser(description="Vision media capability")
    parser.add_argument("--config", default="")
    parser.add_argument("--log-level", default="INFO")
    args = parser.parse_args(argv)
    level = getattr(logging, str(args.log_level or "INFO").upper(), logging.INFO)
    logging.basicConfig(level=level, format="%(asctime)s %(levelname)s %(name)s: %(message)s")
    LOGGER.info("media process starting")
    MediaApp(load_config(args.config or None)).serve_forever()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
