"""python -m vision.rokae_runtime --config config/vision.json"""

from __future__ import annotations

import argparse
import logging
import signal
import sys
import threading

from vision.config import load_config
from vision.rokae_runtime.http_app import serve_owner
from vision.rokae_runtime.owner import RokaeCameraOwner


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description="珞石（ROKAE）相机 Owner HTTP")
    parser.add_argument("--config", default=None)
    parser.add_argument("--log-level", default="INFO")
    args = parser.parse_args(argv)
    logging.basicConfig(
        level=getattr(logging, str(args.log_level).upper(), logging.INFO),
        format="%(asctime)s %(levelname)s %(name)s: %(message)s",
    )
    config = load_config(args.config)
    source = (config.get("rokae") or {}).get("owner", {}).get("source", "direct")
    if source == "ros":
        from vision.rokae_runtime.ros_bridge import RosCameraOwner

        owner = RosCameraOwner(config)
    elif source == "direct":
        owner = RokaeCameraOwner(config)
    else:
        raise ValueError(f"unknown rokae.owner.source: {source}")
    server = None
    try:
        # Bind first: a duplicate process must not open devices before failing.
        server = serve_owner(owner)
        owner.start()
    except Exception:
        owner.stop()
        if server is not None:
            server.server_close()
        raise

    def _stop(*_args) -> None:
        logging.info("stopping rokae owner")
        # shutdown must run outside the serve_forever thread.
        threading.Thread(target=server.shutdown, daemon=True).start()

    signal.signal(signal.SIGINT, _stop)
    signal.signal(signal.SIGTERM, _stop)
    try:
        server.serve_forever()
    finally:
        owner.stop()
        server.server_close()
    return 0


if __name__ == "__main__":
    sys.exit(main())
