from __future__ import annotations

import json
import mimetypes
import re
import time
import uuid
from contextlib import nullcontext
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from typing import Any
from urllib.parse import urlparse

from .backends import BackendError
from .service import ControlService, ValidationError
from .control_trace import context, fields


JOINT_ROUTE = re.compile(r"^/api/joints/(left_arm|right_arm|trunk|head)$")
POSE_ROUTE = re.compile(r"^/api/pose/(left_arm|right_arm|trunk)$")
MOVEL_ROUTE = re.compile(r"^/api/movel/(left_arm|right_arm)/(plain|protected)$")
DRAG_ROUTE = re.compile(r"^/api/drag/(left_arm|right_arm)$")
CAMERA_FRAME_ROUTE = re.compile(r"^/api/cameras/(head|left_wrist|right_wrist)/frame/(rgb)$")
CAMERA_RECORD_ROUTE = re.compile(r"^/api/cameras/(head|left_wrist|right_wrist)/record$")
POSE_RESULT_IMAGE_ROUTE = re.compile(
    r"^/api/pose-estimation/results/(\d{8})/(pose_[0-9]{9}_[a-f0-9]{6})/image$"
)


class ControlHTTPServer(ThreadingHTTPServer):
    daemon_threads = True

    def __init__(self, address: tuple[str, int], service: ControlService, static_dir: Path):
        self.service = service
        self.static_dir = static_dir.resolve()
        super().__init__(address, ControlRequestHandler)


class ControlRequestHandler(BaseHTTPRequestHandler):
    server: ControlHTTPServer

    def log_message(self, format: str, *args: Any) -> None:
        print(f"[{self.log_date_time_string()}] {format % args}")

    def _safe_audit(self, event: str, **data: Any) -> None:
        try:
            self.server.service.audit_event(event, **data)
        except Exception:
            pass

    def _send_json(self, status: int, payload: dict[str, Any]) -> None:
        data = json.dumps(payload, ensure_ascii=False).encode("utf-8")
        self.send_response(status)
        self.send_header("Content-Type", "application/json; charset=utf-8")
        self.send_header("Content-Length", str(len(data)))
        self.send_header("Cache-Control", "no-store")
        if fields().get("request_id"):
            self.send_header("X-Control-Request-Id", fields()["request_id"])
        self.send_header("X-Content-Type-Options", "nosniff")
        self.send_header("X-Frame-Options", "DENY")
        self.end_headers()
        self.wfile.write(data)

    def _read_json(self) -> dict[str, Any]:
        try:
            length = int(self.headers.get("Content-Length", "0"))
        except ValueError as exc:
            raise ValidationError("Content-Length 非法") from exc
        if length < 0 or length > 65536:
            raise ValidationError("请求体过大")
        if length == 0:
            return {}
        try:
            payload = json.loads(self.rfile.read(length).decode("utf-8"))
        except (UnicodeDecodeError, json.JSONDecodeError) as exc:
            raise ValidationError("请求体不是有效 JSON") from exc
        if not isinstance(payload, dict):
            raise ValidationError("请求体必须是 JSON 对象")
        return payload

    def _serve_static(self, request_path: str) -> None:
        relative = "index.html" if request_path == "/" else request_path.lstrip("/")
        candidate = (self.server.static_dir / relative).resolve()
        try:
            candidate.relative_to(self.server.static_dir)
        except ValueError:
            self.send_error(404)
            return
        if not candidate.is_file():
            self.send_error(404)
            return
        data = candidate.read_bytes()
        content_type = mimetypes.guess_type(candidate.name)[0] or "application/octet-stream"
        self.send_response(200)
        self.send_header("Content-Type", f"{content_type}; charset=utf-8")
        self.send_header("Content-Length", str(len(data)))
        self.send_header("Cache-Control", "no-store")
        self.send_header("X-Content-Type-Options", "nosniff")
        self.send_header("X-Frame-Options", "DENY")
        self.end_headers()
        self.wfile.write(data)

    def _serve_camera_frame(self, camera_id: str, kind: str) -> None:
        status = self.server.service.camera.status()[camera_id]
        if not status.get("enabled") and not status.get("externally_managed"):
            self._send_json(409, {"ok": False, "error": "摄像头尚未取得有效 ROS2 数据"})
            return
        frame, sequence, enabled = self.server.service.camera_frame(camera_id, kind, 0, 3.0)
        if not enabled or frame is None:
            self._send_json(409, {"ok": False, "error": "摄像头尚未取得有效画面"})
            return
        self.send_response(200)
        self.send_header("Content-Type", "image/jpeg")
        self.send_header("Content-Length", str(len(frame)))
        self.send_header("X-Camera-Sequence", str(sequence))
        self.send_header("Cache-Control", "no-store, no-cache, must-revalidate")
        self.send_header("Pragma", "no-cache")
        self.send_header("X-Content-Type-Options", "nosniff")
        self.send_header("X-Frame-Options", "DENY")
        self.end_headers()
        self.wfile.write(frame)

    def _serve_pose_estimation_image(self, day: str, leaf: str) -> None:
        try:
            image = self.server.service.pose_estimation_image(day, leaf)
        except BackendError as exc:
            self._send_json(404, {"ok": False, "error": str(exc)})
            return
        self.send_response(200)
        self.send_header("Content-Type", "image/jpeg")
        self.send_header("Content-Length", str(len(image)))
        self.send_header("Cache-Control", "no-store, no-cache, must-revalidate")
        self.send_header("Pragma", "no-cache")
        self.send_header("X-Content-Type-Options", "nosniff")
        self.send_header("X-Frame-Options", "DENY")
        self.end_headers()
        self.wfile.write(image)

    def do_GET(self) -> None:
        path = urlparse(self.path).path
        try:
            camera_match = CAMERA_FRAME_ROUTE.match(path)
            pose_image_match = POSE_RESULT_IMAGE_ROUTE.match(path)
            if path == "/api/status":
                self._send_json(200, {"ok": True, "data": self.server.service.status()})
            elif path == "/api/memory-points":
                self._send_json(200, {"ok": True, "data": self.server.service.memory.listing()})
            elif path == "/api/gripper/status":
                self._send_json(200, {"ok": True, "data": self.server.service.gripper_status()})
            elif path == "/api/pose-estimation/health":
                self._send_json(
                    200,
                    {"ok": True, "data": self.server.service.pose_estimation_health()},
                )
            elif pose_image_match:
                self._serve_pose_estimation_image(
                    pose_image_match.group(1), pose_image_match.group(2)
                )
            elif camera_match:
                self._serve_camera_frame(camera_match.group(1), camera_match.group(2))
            elif path.startswith("/api/"):
                self._send_json(404, {"ok": False, "error": "接口不存在"})
            else:
                self._serve_static(path)
        except (BrokenPipeError, ConnectionResetError):
            return
        except (ValidationError, BackendError) as exc:
            self._send_json(400, {"ok": False, "error": str(exc)})
        except Exception as exc:
            self._send_json(500, {"ok": False, "error": f"服务器错误: {exc}"})

    def do_POST(self) -> None:
        # Reserve/check Agent ownership atomically with legacy web dispatch.
        # Read payload once and reuse it in the existing request handler.
        try:
            payload = self._read_json()
            actions = getattr(self.server.service, 'agent_actions', None)
            gate = actions.web_request(urlparse(self.path).path, payload) if actions else nullcontext()
            with gate:
                self._post_payload = payload
                self._dispatch_post()
        except (ValidationError, BackendError, ValueError) as exc:
            self._send_json(getattr(exc, 'status', 400), {'ok': False, 'error': str(exc)})

    def _dispatch_post(self) -> None:
        started = time.monotonic_ns()
        supplied_id = self.headers.get("X-Control-Request-Id", "")
        request_id = supplied_id if re.fullmatch(r"[a-zA-Z0-9_-]{1,80}", supplied_id) else uuid.uuid4().hex
        token = context.set({"request_id": request_id, "operation_id": request_id,
                             "request_received_monotonic_ns": started})
        path = urlparse(self.path).path
        payload: dict[str, Any] = {}
        try:
            payload = self._post_payload
            service = self.server.service
            service.audit_event(
                "http_request",
                client_ip=self.client_address[0],
                method="POST",
                path=path,
                payload=payload,
                client_sent_at=self.headers.get("X-Control-Sent-At", "")[:80],
            )
            joint_match = JOINT_ROUTE.match(path)
            pose_match = POSE_ROUTE.match(path)
            movel_match = MOVEL_ROUTE.match(path)
            drag_match = DRAG_ROUTE.match(path)
            camera_record_match = CAMERA_RECORD_ROUTE.match(path)
            if path == "/api/control/arm":
                result = service.arm(payload)
            elif path == "/api/memory-points/create":
                result = service.memory.save(payload)
            elif path == "/api/memory-points/overwrite":
                result = service.memory.save(payload, overwrite=True)
            elif path == "/api/memory-points/delete":
                result = service.memory.delete(payload)
            elif path == "/api/memory-points/execute":
                result = service.memory.execute(payload)
            elif path == "/api/memory-points/stop":
                result = service.memory.stop()
            elif path == "/api/movel/stop":
                result = service.arm_movel.stop()
            elif path == "/api/control/disarm":
                result = service.disarm()
            elif path == "/api/gripper/lock":
                result = service.set_gripper_unlocked(payload)
            elif path == "/api/suction/set":
                result = service.set_suction(payload)
            elif path == "/api/gripper/activate":
                result = service.activate_gripper()
            elif path == "/api/gripper/position":
                result = service.move_gripper(payload)
            elif path == "/api/readback":
                result = service.readback()
            elif path == "/api/speed":
                result = service.set_speed(payload)
            elif joint_match:
                result = service.move_joints(joint_match.group(1), payload)
            elif pose_match:
                result = service.move_pose(pose_match.group(1), payload)
            elif movel_match:
                result = service.arm_movel.execute(movel_match.group(1), payload, movel_match.group(2) == "protected")
            elif drag_match:
                result = service.set_drag(drag_match.group(1), payload)
            elif path == "/api/chassis/enable":
                result = service.set_chassis_enabled(payload)
            elif path == "/api/chassis/obstacle-avoidance":
                result = service.set_chassis_obstacle_avoidance(payload)
            elif path == "/api/chassis/command":
                result = service.chassis_command(payload)
            elif path == "/api/chassis/stop":
                result = service.chassis_stop()
            elif path == "/api/chassis/release-emergency-stop":
                result = service.release_chassis_emergency_stop()
            elif path == "/api/camera/enable":
                result = service.set_camera_enabled(payload)
            elif path == "/api/cameras/enable":
                result = service.set_camera_enabled(payload)
            elif path == "/api/cameras/restart":
                result = service.restart_camera(payload)
            elif path == "/api/pose-estimation/grasp-object":
                result = service.estimate_grasp_object_pose(payload) if payload else service.estimate_grasp_object_pose()
            elif path == "/api/pose-estimation/reproject-left-shoulder":
                result = service.reproject_latest_grasp_to_left_shoulder()
            elif path == "/api/pose-estimation/reproject-right-shoulder":
                kind = payload.get('sku_typ', 'bottle')
                result = (service.reproject_latest_grasp_to_right_shoulder() if kind == 'bottle'
                          else service.reproject_latest_grasp_to_right_shoulder(kind))
            elif path == "/api/pose-estimation/grasp-test":
                result = service.grasp_test.execute(payload)
            elif path == "/api/pose-estimation/grasp-test/stop":
                result = service.grasp_test.stop()
            elif path == "/api/placement/start":
                result = service.placement.execute(payload)
            elif path == "/api/placement/stop":
                result = service.placement.stop()
            elif path == "/api/scan-sequence/start":
                result = service.scan_sequence.execute(payload)
            elif path == "/api/scan-sequence/stop":
                result = service.scan_sequence.stop()
            elif path in ("/api/cameras/left_wrist/record-rgb", "/api/cameras/right_wrist/record-rgb"):
                result = service.record_camera_rgb(path.split("/")[3])
            elif camera_record_match:
                result = service.record_camera_snapshot(camera_record_match.group(1))
            else:
                service.audit_event("http_response", path=path, ok=False, status=404,
                                    duration_ms=(time.monotonic_ns() - started) / 1e6)
                self._send_json(404, {"ok": False, "error": "接口不存在"})
                return
            service.audit_event("http_response", path=path, ok=True, result=result,
                                duration_ms=(time.monotonic_ns() - started) / 1e6)
            self._send_json(200, {"ok": True, "data": result})
        except (ValidationError, BackendError) as exc:
            self._safe_audit(
                "http_response",
                path=path,
                ok=False,
                error_type=type(exc).__name__,
                error=str(exc),
                duration_ms=(time.monotonic_ns() - started) / 1e6,
            )
            self._send_json(400, {"ok": False, "error": str(exc)})
        except Exception as exc:
            self._safe_audit(
                "http_response",
                path=path,
                ok=False,
                error_type=type(exc).__name__,
                error=str(exc),
                duration_ms=(time.monotonic_ns() - started) / 1e6,
            )
            self._send_json(500, {"ok": False, "error": f"服务器错误: {exc}"})
        finally:
            context.reset(token)


def make_server(host: str, port: int, service: ControlService, static_dir: Path) -> ControlHTTPServer:
    return ControlHTTPServer((host, port), service, static_dir)
