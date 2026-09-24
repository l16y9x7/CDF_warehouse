"""珞石（ROKAE）相机 Owner HTTP：/camera/health|list|snapshot|stream|capture。"""

from __future__ import annotations

import json
import logging
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from typing import Any, Dict, Tuple
from urllib.parse import parse_qs, urlparse

from vision.rokae_runtime.capture_api import parse_capture_query
from vision.rokae_runtime.agent_contract import snapshot, query, stream_frame
from vision.rokae_runtime.owner import RokaeCameraOwner
from vision.rokae_runtime.preview import PREVIEW_HTML

LOGGER = logging.getLogger(__name__)


def _json_bytes(payload: Dict[str, Any]) -> bytes:
    return json.dumps(payload, ensure_ascii=False).encode("utf-8")


class RokaeCameraHandler(BaseHTTPRequestHandler):
    owner: RokaeCameraOwner

    def log_message(self, fmt: str, *args) -> None:
        LOGGER.info("HTTP " + fmt, *args)

    def _send(self, status: int, body: bytes, content_type: str) -> None:
        self.send_response(status)
        self.send_header("Content-Type", content_type)
        self.send_header("Content-Length", str(len(body)))
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        self.wfile.write(body)

    def _send_json(self, status: int, payload: Dict[str, Any]) -> None:
        self._send(status, _json_bytes(payload), "application/json; charset=utf-8")

    def _parse(self) -> Tuple[str, Dict[str, str]]:
        parsed = urlparse(self.path)
        qs = {k: (v[0] if v else "") for k, v in parse_qs(parsed.query).items()}
        return parsed.path, qs

    def do_GET(self) -> None:  # noqa: N802
        path, qs = self._parse()
        if path in {"/", "/preview"}:
            self._send(200, PREVIEW_HTML.encode("utf-8"), "text/html; charset=utf-8")
            return
        if path in {"/camera/health", "/health"}:
            if path == '/camera/health':
                rows = [r for r in self.owner.listing()['cameras'] if r['enabled']]
                ready = bool(rows) and all(r['color']['online'] and
                    (r['id'] != 'head' or (r['depth']['online'] and r['depth']['aligned'])) for r in rows)
                self._send_json(200, dict(status='READY' if ready else 'ERROR', ok=ready, all_ready=ready))
            else:
                self._send_json(200, self.owner.health())
            return
        if path in {"/camera/list", "/list"}:
            self._send_json(200, self.owner.listing()["cameras"] if path == "/camera/list" else self.owner.listing())
            return
        if path == '/camera/frame':
            # Compatibility for the existing vision adapter's binary image URI.
            jpeg = self.owner.get_jpeg(qs.get('camera', ''))
            if jpeg is None:
                self._send_json(503, dict(ok=False, error_code='CAMERA_NOT_READY'))
            else:
                self._send(200, jpeg, 'image/jpeg')
            return
        if path in {"/camera/capture", "/capture"}:
            parsed, err = parse_capture_query(qs)
            if err is not None:
                code = 404 if err.get("error_code") == "CAMERA_NOT_FOUND" else 400
                self._send_json(code, err)
                return
            result = self.owner.capture(
                contract=parsed["contract"],
                internal=parsed["internal"],
                streams=parsed["streams"],
                format=parsed["format"],
            )
            if not result.get("ok"):
                code = 404 if result.get("error_code") in {
                    "CAMERA_NOT_FOUND", "CAMERA_NOT_READY",
                } else 409 if result.get("error_code") == "DEPTH_NOT_ALIGNED" else (
                    500 if result.get("error_code") == "CAPTURE_FAILED" else 400
                )
                self._send_json(code, result)
                return
            self._send_json(200, result)
            return
        if path in {"/camera/snapshot", "/snapshot", "/camera/rgbd"}:
            try:
                result = snapshot(self.owner, qs, rgbd=path == "/camera/rgbd")
                self._send_json(200 if result.get('ok') else 503, result)
            except ValueError as exc:
                self._send_json(400, {'ok': False, 'error_code': str(exc)})
            return
        if path in {"/camera/stream", "/stream"}:
            try:
                camera, _, kind, _ = query(qs, stream=True)
            except ValueError as exc:
                self._send_json(400, {'ok': False, 'error_code': str(exc)})
                return
            if not self.owner.camera_ready(camera):
                self._send_json(404, {"ok": False, "error_code": "CAMERA_NOT_READY", "camera": camera})
                return
            self.send_response(200)
            self.send_header("Age", "0")
            self.send_header("Cache-Control", "no-cache, private")
            self.send_header("Pragma", "no-cache")
            self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=frame")
            self.end_headers()
            try:
                interval = self.owner.stream_interval_sec(camera)
                while True:
                    jpeg = stream_frame(self.owner, camera, kind)
                    if jpeg is None:
                        # End this consumer session; never hold an empty stream forever.
                        return
                    self.wfile.write(b"--frame\r\n")
                    self.wfile.write(b"Content-Type: image/jpeg\r\n")
                    self.wfile.write(f"Content-Length: {len(jpeg)}\r\n\r\n".encode("ascii"))
                    self.wfile.write(jpeg)
                    self.wfile.write(b"\r\n")
                    self.wfile.flush()
                    time.sleep(interval)
            except (BrokenPipeError, ConnectionResetError):
                return
        self._send_json(404, {"ok": False, "error_code": "NOT_FOUND", "path": path})


def serve_owner(owner: RokaeCameraOwner) -> ThreadingHTTPServer:
    handler = type("BoundHandler", (RokaeCameraHandler,), {"owner": owner})
    server = ThreadingHTTPServer((owner.host, owner.port), handler)
    LOGGER.info("rokae owner listening http://%s:%s", owner.host, owner.port)
    return server
