"""管理 head / hand_left / hand_right 三路相机。"""

from __future__ import annotations

import logging
import json
import math
import os
import threading
from typing import Any, Dict, Optional

from vision.rokae_runtime.capture import (
    FakeCaptureWorker,
    OrbbecSdkCaptureWorker,
    V4L2CaptureWorker,
)
from vision.rokae_runtime.devices import resolve_device

LOGGER = logging.getLogger(__name__)
CAMERA_IDS = ("head", "hand_left", "hand_right")


class RokaeCameraOwner:
    def __init__(self, config: Dict[str, Any]) -> None:
        self.config = config
        rokae = config.get("rokae") or {}
        owner = rokae.get("owner") or {}
        self.host = str(owner.get("host") or "0.0.0.0")
        self.port = int(owner.get("port", 8085))
        self._cameras_cfg: Dict[str, Any] = owner.get("cameras") or {}
        self._workers: Dict[str, Any] = {}
        self._binding_lock = threading.Lock()
        self._meta: Dict[str, Dict[str, Any]] = {}

    def start(self) -> None:
        # Calling start twice must not open a second set of devices.
        if self._workers:
            return
        self._meta.clear()
        claimed = set()
        for camera_id in CAMERA_IDS:
            try:
                with self._binding_lock:
                    self._start_camera(camera_id, claimed)
            except Exception as exc:
                LOGGER.exception("camera configuration/start failed camera=%s", camera_id)
                self._meta[camera_id] = {
                    "enabled": True, "ready": False, "error": str(exc),
                }

    def _start_camera(self, camera_id: str, claimed: set) -> None:
        cfg = dict(self._cameras_cfg.get(camera_id) or {})
        enabled = cfg.get("enabled", True) is not False
        meta = {"enabled": enabled, "ready": False, "backend": "", "device": ""}
        self._meta[camera_id] = meta
        if not enabled:
            return
        width = int(cfg.get("width", 640))
        height = int(cfg.get("height", 480))
        fps = int(cfg.get("fps", 10))
        stale = float(cfg.get("frame_stale_sec", 2.0))
        if min(width, height, fps) <= 0 or not math.isfinite(stale) or stale <= 0:
            raise ValueError("width/height/fps/frame_stale_sec must be positive")
        match = cfg.get("match") or {}
        if not isinstance(match, dict):
            raise ValueError("camera.match must be an object")
        match_type = str(match.get("type") or "").strip().lower()
        backend = str(cfg.get("backend") or
                      (match_type if match_type in {"fake", "orbbec", "orbbec_sdk"} else "v4l2")).strip().lower()
        meta.update(backend=backend, fps=fps)
        identity = None
        binding = (backend, json.dumps(match, sort_keys=True))
        if backend == "v4l2" and binding in claimed:
            meta["error"] = "DEVICE_ALREADY_ASSIGNED"
            return
        if backend == "fake":
            colors = {"head": (40, 160, 40), "hand_left": (40, 40, 160), "hand_right": (160, 40, 40)}
            worker = FakeCaptureWorker(camera_id=camera_id, width=width, height=height,
                                       color=colors[camera_id])
        elif backend in {"orbbec", "orbbec_sdk"}:
            serial = str(match.get("serial") or cfg.get("serial") or
                         (match.get("value") if match_type in {"orbbec", "orbbec_sdk"} else "") or "").strip()
            # Multiple SDK roles require explicit distinct SNs, before opening hardware.
            sdk_serials = []
            for candidate in self._cameras_cfg.values():
                if not isinstance(candidate, dict) or candidate.get("enabled", True) is False:
                    continue
                candidate_match = candidate.get("match") or {}
                if not isinstance(candidate_match, dict):
                    continue
                candidate_type = str(candidate_match.get("type") or "").strip().lower()
                candidate_backend = str(candidate.get("backend") or candidate_type).strip().lower()
                if candidate_backend in {"orbbec", "orbbec_sdk"}:
                    sdk_serials.append(str(candidate_match.get("serial") or candidate.get("serial") or
                        (candidate_match.get("value") if candidate_type in {"orbbec", "orbbec_sdk"} else "") or "").strip())
            if len(sdk_serials) > 1 and not all(sdk_serials):
                raise ValueError("multiple Orbbec roles require explicit distinct serials")
            identity = ("orbbec", serial)
            worker = OrbbecSdkCaptureWorker(camera_id=camera_id, width=width, height=height,
                                            fps=fps, serial=serial)
            meta.update(backend="orbbec", device=serial or "orbbec")
        elif backend == "v4l2":
            fourcc = str(cfg.get("fourcc") or "").strip().upper()
            if fourcc not in {"", "MJPG", "YUYV", "UYVY"}:
                raise ValueError("fourcc must be MJPG, YUYV or UYVY")
            device = resolve_device(match, width=width, height=height, fourcc=fourcc)
            if not device:
                meta["error"] = "DEVICE_NOT_FOUND"
            identity = ("v4l2", os.path.realpath(device)) if device else None
            def rediscover() -> Optional[str]:
                nonlocal identity
                with self._binding_lock:
                    excluded = {path for kind, path in claimed
                                if kind == "v4l2" and path.startswith("/") and (kind, path) != identity}
                    found = resolve_device(match, width=width, height=height,
                                           fourcc=fourcc, exclude=excluded)
                    if found:
                        next_identity = ("v4l2", os.path.realpath(found))
                        if next_identity in claimed and next_identity != identity:
                            return None
                        claimed.discard(identity)
                        identity = next_identity
                        claimed.add(identity)
                        meta["device"] = found
                        meta.pop("error", None)
                    else:
                        meta["device"] = ""
                        meta["error"] = "DEVICE_NOT_FOUND"
                    return found

            worker = V4L2CaptureWorker(camera_id=camera_id, device=device or "", width=width,
                                       height=height, fps=fps,
                                       warmup_frames=int(cfg.get("warmup_frames") or 3), fourcc=fourcc,
                                       device_resolver=rediscover)
            meta["device"] = device or ""
        else:
            raise ValueError(f"unknown camera backend: {backend}")
        if identity is not None and identity in claimed:
            meta["error"] = "DEVICE_ALREADY_ASSIGNED"
            return
        try:
            if not worker.start():
                worker.stop()
                meta["error"] = "CAPTURE_START_FAILED"
                return
        except Exception:
            worker.stop()
            raise
        if identity is not None:
            claimed.add(identity)
        if backend == "v4l2":
            claimed.add(binding)
        self._workers[camera_id] = {"worker": worker, "stale": stale}
        meta["ready"] = self.camera_ready(camera_id)
        LOGGER.info("owner camera=%s metadata=%s", camera_id, meta)

    def stop(self) -> None:
        for camera_id, entry in list(self._workers.items()):
            try:
                entry["worker"].stop()
            except Exception as exc:
                LOGGER.warning("stop camera=%s failed: %s", camera_id, exc)
            else:
                self._workers.pop(camera_id, None)

    def health(self) -> Dict[str, Any]:
        any_ready = any(self.camera_ready(cid) for cid in CAMERA_IDS)
        enabled = [cid for cid in CAMERA_IDS if (self._meta.get(cid) or {}).get("enabled")]
        return {"status": "READY" if any_ready else "ERROR", "ok": any_ready,
                "all_ready": bool(enabled) and all(self.camera_ready(cid) for cid in enabled)}

    def camera_ready(self, camera_id: str) -> bool:
        from vision.rokae_runtime.capture_api import resolve_camera

        resolved = resolve_camera(camera_id)
        internal = resolved[1] if resolved else str(camera_id or "").strip()
        meta = self._meta.get(internal) or {}
        if not meta.get("enabled", True):
            return False
        entry = self._workers.get(internal)
        if entry is None:
            return False
        try:
            ready = bool(entry["worker"].ready(max_stale_sec=float(entry["stale"])))
        except Exception:
            LOGGER.exception("readiness failed camera=%s", internal)
            ready = False
        meta["ready"] = ready
        return ready

    def stream_interval_sec(self, camera_id: str) -> float:
        from vision.rokae_runtime.capture_api import resolve_camera

        resolved = resolve_camera(camera_id)
        internal = resolved[1] if resolved else str(camera_id or "").strip()
        meta = self._meta.get(internal) or {}
        fps = int(meta.get("fps") or 10)
        return 1.0 / float(max(fps, 1))

    def listing(self) -> Dict[str, Any]:
        from vision.rokae_runtime.capture_api import contract_camera_id

        cameras = []
        for camera_id in CAMERA_IDS:
            meta = dict(self._meta.get(camera_id) or {"camera_id": camera_id, "enabled": False})
            enabled = bool(meta.get("enabled", True))
            ready = self.camera_ready(camera_id) if enabled else False
            entry = self._workers.get(camera_id)
            if entry is not None and meta.get("backend") == "orbbec":
                meta["error"] = "" if ready else entry["worker"].last_error or "CAMERA_NOT_READY"
                meta["device"] = entry["worker"].serial or meta.get("device", "")
            contract = contract_camera_id(camera_id)
            color = {"online": False, "width": None, "height": None, "age_sec": None, "stamp": None}
            depth = {
                "online": False, "aligned": False,
                "width": None, "height": None, "age_sec": None, "stamp": None,
            }
            if entry is not None:
                stale = float(entry["stale"])
                peek = None
                worker = entry["worker"]
                if hasattr(worker, "peek"):
                    peek = worker.peek(max_stale_sec=stale)
                else:
                    frame = worker.get_frame(max_stale_sec=stale)
                    if frame is not None:
                        peek = {
                            "online": True,
                            "width": int(frame.shape[1]),
                            "height": int(frame.shape[0]),
                            "age_sec": None,
                            "stamp": None,
                        }
                if peek is not None:
                    color.update(peek)
            # Direct Owner has no depth topics.
            cfg = self._cameras_cfg.get(camera_id) or {}
            if color["width"] is None and cfg.get("width"):
                color["width"] = int(cfg["width"])
                color["height"] = int(cfg.get("height") or 0) or None
            cameras.append(
                {
                    "id": contract,
                    "name": contract,
                    "camera_id": contract,
                    "legacy_id": camera_id,
                    "enabled": enabled,
                    "ready": ready,
                    "online": ready,
                    "color": color,
                    "depth": depth,
                    "backend": meta.get("backend") or "",
                    "device": meta.get("device") or "",
                    "error": meta.get("error") or "",
                }
            )
        ok = any(row["ready"] for row in cameras)
        return {"ok": ok, "cameras": cameras}

    def get_jpeg(self, camera_id: str, quality: int = 80) -> Optional[bytes]:
        from vision.rokae_runtime.capture_api import resolve_camera

        resolved = resolve_camera(camera_id)
        internal = resolved[1] if resolved else str(camera_id or "").strip()
        entry = self._workers.get(internal)
        if entry is None:
            return None
        frame = entry["worker"].get_frame(max_stale_sec=float(entry["stale"]))
        if frame is None:
            return None
        import cv2

        ok, buf = cv2.imencode(".jpg", frame, [int(cv2.IMWRITE_JPEG_QUALITY), int(quality)])
        if not ok:
            return None
        return buf.tobytes()

    def get_depth_mm(self, camera_id: str):
        """Return (depth_mm, aligned) or None. Direct Owner has no depth."""
        return None

    def capture(self, *, contract: str, internal: str, streams: set, format: Optional[str] = None) -> Dict[str, Any]:
        """Execute one capture session per GET /camera/capture contract."""
        import time

        from vision.rokae_runtime.capture_api import new_capture_id, write_capture_dir

        need_color = "color" in streams
        need_depth = "depth" in streams
        color_jpeg = None
        depth_mm = None
        depth_aligned = False
        last_message = "camera not ready"
        for attempt in range(3):
            if need_color:
                color_jpeg = self.get_jpeg(internal)
            if need_depth:
                depth_result = self.get_depth_mm(internal)
                if depth_result is not None:
                    depth_mm, depth_aligned = depth_result
                else:
                    depth_mm, depth_aligned = None, False
            color_ok = (not need_color) or color_jpeg is not None
            depth_ok = (not need_depth) or depth_mm is not None
            if color_ok and depth_ok:
                break
            if need_color and color_jpeg is None:
                last_message = "color frame not ready"
            elif need_depth and depth_mm is None:
                last_message = "depth frame not ready"
            if attempt < 2:
                time.sleep(0.08)
        else:
            return {
                "ok": False,
                "error_code": "CAMERA_NOT_READY",
                "message": last_message,
                "camera": contract,
            }
        if need_depth and not depth_aligned:
            return {
                "ok": False,
                "error_code": "DEPTH_NOT_ALIGNED",
                "message": "aligned depth unavailable; refusing unaligned native depth",
                "camera": contract,
            }
        capture_id = new_capture_id()
        try:
            written = write_capture_dir(
                capture_id=capture_id,
                color_jpeg=color_jpeg if need_color else None,
                depth_mm=depth_mm if need_depth else None,
                depth_format=format if need_depth else None,
                depth_aligned=depth_aligned if need_depth else None,
            )
        except Exception as exc:
            LOGGER.exception("capture write failed camera=%s", contract)
            return {
                "ok": False,
                "error_code": "CAPTURE_FAILED",
                "message": str(exc),
                "camera": contract,
            }
        return {
            "ok": True,
            "capture_id": capture_id,
            "camera": contract,
            "same_shot": True,
            "color": written["color"],
            "depth": written["depth"],
        }
