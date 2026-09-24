#!/usr/bin/env python3
"""本地 Media 冒烟：Owner fake MJPEG → ffmpeg(mpjpeg) → 本地 .flv（不需云端 stream_key）。

用法:
  PYTHONPATH=. python scripts/media_local_smoke.py
环境变量（可选）: VISION_SMOKE_OUT、VISION_SMOKE_OWNER_PORT、FFMPEG_BIN
"""

from __future__ import annotations

import os
import sys
import threading
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))


def _ffmpeg_bin() -> str:
    env = str(os.environ.get("FFMPEG_BIN") or "").strip()
    if env:
        return env
    try:
        from imageio_ffmpeg import get_ffmpeg_exe

        return get_ffmpeg_exe()
    except Exception:
        return "ffmpeg"


def main() -> int:
    from vision.rokae_runtime.http_app import serve_owner
    from vision.rokae_runtime.owner import RokaeCameraOwner
    from vision.service import CameraService, MediaService

    out_dir = Path(os.environ["VISION_SMOKE_OUT"]) if os.environ.get("VISION_SMOKE_OUT") else (
        ROOT / "runtime" / "smoke-out"
    )
    out_dir.mkdir(parents=True, exist_ok=True)

    port = int(os.environ.get("VISION_SMOKE_OWNER_PORT") or "18089")
    ffmpeg_bin = _ffmpeg_bin()
    config = {
        "adapter": "rokae",
        "rokae": {
            "base_url": f"http://127.0.0.1:{port}",
            "owner": {
                "host": "127.0.0.1",
                "port": port,
                "cameras": {
                    "head": {
                        "enabled": True,
                        "backend": "fake",
                        "match": {"type": "fake"},
                        "width": 160,
                        "height": 120,
                    },
                    "hand_left": {"enabled": False, "backend": "fake", "match": {"type": "fake"}},
                    "hand_right": {"enabled": False, "backend": "fake", "match": {"type": "fake"}},
                },
            },
        },
        "media": {
            "push": {
                "enabled": True,
                "device_sn": "SMOKE_LOCAL",
                "stream_server_url": f"file://{out_dir}",
                "ffmpeg_bin": ffmpeg_bin,
                "streams": [
                    {
                        "camera_id": "head",
                        "stream_slot": 1,
                        "enabled": True,
                        "width": 160,
                        "height": 120,
                        "fps": 5,
                        "bitrate": "400k",
                        "preset": "ultrafast",
                    }
                ],
            }
        },
    }

    owner = RokaeCameraOwner(config)
    owner.start()
    server = serve_owner(owner)
    thread = threading.Thread(target=server.serve_forever, daemon=True)
    thread.start()
    media = MediaService(CameraService(config), config)
    flv = out_dir / "head.flv"
    if flv.exists():
        flv.unlink()
    try:
        stream = media.stream("head")
        print("stream_uri=", stream.get("uri"))
        started = media.start_push(camera_id="head", device_sn="SMOKE_LOCAL")
        print("push_start=", started.get("ok"), "streams=", started.get("streams"))
        deadline = time.time() + 6.0
        saw_online = False
        while time.time() < deadline:
            status = media.push_status(camera_id="head")
            row = (status.get("streams") or [{}])[0]
            if row.get("online"):
                saw_online = True
            if flv.exists() and flv.stat().st_size > 1024:
                break
            time.sleep(0.4)
        media.stop_push()
        time.sleep(0.5)
        size = flv.stat().st_size if flv.exists() else 0
        print("flv_bytes=", size, "saw_online=", saw_online)
        if size > 512 and saw_online:
            print("OK smoke wrote", flv)
            return 0
        print("FAIL status after stop size=", size, file=sys.stderr)
        return 1
    finally:
        try:
            media.stop_push()
        except Exception:
            pass
        server.shutdown()
        server.server_close()
        owner.stop()


if __name__ == "__main__":
    raise SystemExit(main())
