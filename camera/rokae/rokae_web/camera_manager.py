from __future__ import annotations

import json
import shutil
import threading
import time
import uuid
from datetime import datetime
from pathlib import Path
from typing import Any

from .backends import BackendError
from .camera import CameraSnapshot, DisabledCameraBackend


CAMERA_LABELS = {
    "head": "头部摄像头",
    "left_wrist": "左腕 RealSense",
    "right_wrist": "右腕 RealSense",
}
WRIST_CAMERA_IDS = ("left_wrist", "right_wrist")


class MultiCameraRecordingStore:
    def __init__(self, data_root: str | Path) -> None:
        self.root = Path(data_root).expanduser().resolve()
        self.root.mkdir(parents=True, exist_ok=True)
        self._lock = threading.Lock()

    def save(
        self,
        snapshots: dict[str, CameraSnapshot],
        robot_state: dict[str, Any],
    ) -> dict[str, Any]:
        import cv2
        import numpy as np

        if not snapshots:
            raise BackendError("没有已开启且可记录的摄像头")
        with self._lock:
            while True:
                recorded_at = datetime.now().astimezone()
                day_dir = self.root / recorded_at.strftime("%Y%m%d")
                leaf = recorded_at.strftime("%H%M%S") + f"{recorded_at.microsecond // 1000:03d}"
                target = day_dir / leaf
                if not target.exists():
                    break
                time.sleep(0.001)
            day_dir.mkdir(parents=True, exist_ok=True)
            temporary = day_dir / f".{leaf}-{uuid.uuid4().hex}.tmp"
            temporary.mkdir()
            files: list[str] = []
            cameras: dict[str, Any] = {}
            try:
                for camera_id in sorted(snapshots):
                    snapshot = snapshots[camera_id]
                    rgb_name = f"{camera_id}_rgb.jpg"
                    depth_name = f"{camera_id}_depth_aligned.npy"
                    metadata_name = f"{camera_id}_camera_metadata.json"
                    if not cv2.imwrite(
                        str(temporary / rgb_name),
                        snapshot.rgb_bgr,
                        [cv2.IMWRITE_JPEG_QUALITY, 95],
                    ):
                        raise BackendError(f"保存 {CAMERA_LABELS[camera_id]} RGB 图像失败")
                    np.save(
                        temporary / depth_name,
                        snapshot.depth_aligned_mm.astype(np.float32, copy=False),
                        allow_pickle=False,
                    )
                    camera_metadata = {
                        "camera_id": camera_id,
                        "camera_label": CAMERA_LABELS[camera_id],
                        "camera_frame_captured_at": snapshot.captured_at,
                        "color_timestamp_ms": snapshot.color_timestamp_ms,
                        "depth_timestamp_ms": snapshot.depth_timestamp_ms,
                        "camera_sequence": snapshot.sequence,
                        "rgb_file": rgb_name,
                        "depth_file": depth_name,
                        "depth_dtype": "float32",
                        "depth_unit": "millimeter",
                        "invalid_depth_value": 0,
                        "camera": snapshot.camera_info,
                    }
                    (temporary / metadata_name).write_text(
                        json.dumps(camera_metadata, ensure_ascii=False, indent=2),
                        encoding="utf-8",
                    )
                    files.extend([rgb_name, depth_name, metadata_name])
                    cameras[camera_id] = camera_metadata

                (temporary / "robot_state.json").write_text(
                    json.dumps(robot_state, ensure_ascii=False, indent=2),
                    encoding="utf-8",
                )
                record_metadata = {
                    "recorded_at": recorded_at.isoformat(timespec="milliseconds"),
                    "active_cameras": sorted(snapshots),
                    "camera_labels": {
                        camera_id: CAMERA_LABELS[camera_id] for camera_id in sorted(snapshots)
                    },
                    "cameras": cameras,
                    "robot_state_file": "robot_state.json",
                }
                (temporary / "record_metadata.json").write_text(
                    json.dumps(record_metadata, ensure_ascii=False, indent=2),
                    encoding="utf-8",
                )
                files.extend(["robot_state.json", "record_metadata.json"])
                temporary.replace(target)
            except Exception:
                shutil.rmtree(temporary, ignore_errors=True)
                raise

        relative = target.relative_to(self.root)
        return {
            "directory": str(target),
            "relative_directory": f"{self.root.name}/{relative.as_posix()}",
            "active_cameras": sorted(snapshots),
            "camera_labels": [CAMERA_LABELS[name] for name in sorted(snapshots)],
            "files": files,
        }


class CameraManager:
    def __init__(self, backends: dict[str, Any], data_root: str | Path) -> None:
        missing = set(CAMERA_LABELS) - set(backends)
        if missing:
            raise ValueError(f"缺少摄像头后端: {sorted(missing)}")
        self._backends = dict(backends)
        self._store = MultiCameraRecordingStore(data_root)
        self._switch_lock = threading.Lock()

    def _backend(self, camera_id: str) -> Any:
        if camera_id not in CAMERA_LABELS:
            raise BackendError("未知摄像头")
        return self._backends[camera_id]

    def status(self) -> dict[str, dict[str, Any]]:
        result: dict[str, dict[str, Any]] = {}
        for camera_id, label in CAMERA_LABELS.items():
            status = dict(self._backends[camera_id].status())
            status.update(camera_id=camera_id, label=label)
            result[camera_id] = status
        return result

    def set_enabled(self, camera_id: str, enabled: bool) -> dict[str, Any]:
        with self._switch_lock:
            backend = self._backend(camera_id)
            if enabled and camera_id in WRIST_CAMERA_IDS:
                for other_id in WRIST_CAMERA_IDS:
                    if other_id == camera_id:
                        continue
                    other = self._backends[other_id]
                    other_status = other.status()
                    if other_status.get("enabled") or other_status.get("starting"):
                        other.stop()
            result = backend.start() if enabled else backend.stop()
        return {**result, "camera_id": camera_id, "label": CAMERA_LABELS[camera_id]}

    def frame_jpeg(
        self,
        camera_id: str,
        kind: str,
        after_sequence: int,
        timeout: float,
    ) -> tuple[bytes | None, int, bool]:
        return self._backend(camera_id).frame_jpeg(kind, after_sequence, timeout)

    def record(self, camera_id: str, robot_state: dict[str, Any]) -> dict[str, Any]:
        backend = self._backend(camera_id)
        if not backend.status().get("enabled"):
            raise BackendError(f"{CAMERA_LABELS[camera_id]}未开启")
        return self._store.save({camera_id: backend.snapshot()}, robot_state)

    def snapshot(self, camera_id: str) -> CameraSnapshot:
        backend = self._backend(camera_id)
        if not backend.status().get("enabled"):
            raise BackendError(f"{CAMERA_LABELS[camera_id]}未开启")
        return backend.snapshot()

    def close(self) -> None:
        for backend in self._backends.values():
            try:
                backend.close()
            except Exception:
                pass


class DisabledCameraManager:
    def __init__(self) -> None:
        self._backends = {name: DisabledCameraBackend() for name in CAMERA_LABELS}

    def status(self) -> dict[str, dict[str, Any]]:
        return {
            name: {**backend.status(), "camera_id": name, "label": CAMERA_LABELS[name]}
            for name, backend in self._backends.items()
        }

    def set_enabled(self, camera_id: str, enabled: bool) -> dict[str, Any]:
        if camera_id not in CAMERA_LABELS:
            raise BackendError("未知摄像头")
        if enabled:
            raise BackendError("当前服务模式未启用摄像头")
        return self.status()[camera_id]

    def frame_jpeg(
        self,
        camera_id: str,
        kind: str,
        after_sequence: int,
        timeout: float,
    ) -> tuple[bytes | None, int, bool]:
        if camera_id not in CAMERA_LABELS:
            raise BackendError("未知摄像头")
        return self._backends[camera_id].frame_jpeg(kind, after_sequence, timeout)

    def record(self, camera_id: str, robot_state: dict[str, Any]) -> dict[str, Any]:
        del camera_id, robot_state
        raise BackendError("当前服务模式未启用摄像头")

    def snapshot(self, camera_id: str) -> CameraSnapshot:
        if camera_id not in CAMERA_LABELS:
            raise BackendError("未知摄像头")
        raise BackendError("当前服务模式未启用摄像头")

    def close(self) -> None:
        return
