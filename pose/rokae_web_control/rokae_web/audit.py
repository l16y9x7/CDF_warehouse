from __future__ import annotations

import json
import threading
import uuid
import time
import hashlib
import zipfile
import platform
import sys
from datetime import datetime
from pathlib import Path
from typing import Any, Protocol

from .control_trace import fields, plain


class AuditSink(Protocol):
    def record(self, event: str, **data: Any) -> None: ...

    def close(self) -> None: ...


class NullAuditLogger:
    def record(self, event: str, **data: Any) -> None:
        del event, data

    def close(self) -> None:
        return


class MemoryAuditLogger:
    """Test sink that keeps structured records in memory."""

    def __init__(self) -> None:
        self.records: list[dict[str, Any]] = []

    def record(self, event: str, **data: Any) -> None:
        self.records.append({"event": event, **fields(), **plain(data)})

    def close(self) -> None:
        return


class JsonlAuditLogger:
    """Append-only JSONL logger with one local-date file per day."""

    def __init__(self, directory: str | Path) -> None:
        self.directory = Path(directory).expanduser().resolve()
        self.directory.mkdir(parents=True, exist_ok=True)
        self.session_id = str(uuid.uuid4())
        self._sequence = 0
        self._lock = threading.RLock()
        self._handle = None
        self._path = None

    def record(self, event: str, **data: Any) -> None:
        now = datetime.now().astimezone()
        with self._lock:
            self._sequence += 1
            entry = {
                "schema_version": 2,
                "timestamp": now.isoformat(timespec="milliseconds"),
                "monotonic_ns": time.monotonic_ns(),
                "thread": threading.current_thread().name,
                "session_id": self.session_id,
                "sequence": self._sequence,
                "event": event,
                **fields(),
                **data,
            }
            path = self.directory / f"motion-{now:%Y-%m-%d}.jsonl"
            encoded = json.dumps(entry, ensure_ascii=False, default=plain) + "\n"
            if self._handle is None or self._path != path:
                if self._handle is not None:
                    self._handle.close()
                self._handle = path.open("a", encoding="utf-8", newline="\n")
                self._path = path
            self._handle.write(encoded)
            self._handle.flush()

    def capture_assets(self, config):
        """Persist replay inputs once at startup, outside the command path."""
        destination = self.directory / "sessions" / self.session_id
        destination.mkdir(parents=True, exist_ok=True)
        sources = Path(__file__).resolve().parent
        assets = []
        def save(name, raw, source):
            path = destination / name
            path.parent.mkdir(parents=True, exist_ok=True)
            path.write_bytes(raw)
            assets.append({"file": str(path), "source": str(source), "sha256": hashlib.sha256(raw).hexdigest()})
        save("effective-config.json", json.dumps(config, ensure_ascii=False, indent=2).encode(), "effective config")
        save("runtime.json", json.dumps({"python": sys.version, "platform": platform.platform()}).encode(), "runtime")
        for path in sorted(sources.glob("*.py")):
            save("source/" + path.name, path.read_bytes(), path)
        for name in ("server.py", "static/app.js", "static/index.html"):
            path = sources.parent / name
            if path.is_file():
                save("web/" + name, path.read_bytes(), path)
        for path in sorted((sources.parents[1] / "arm_motion_control").glob("*.py")):
            save("arm_motion_control/" + path.name, path.read_bytes(), path)
        for key in ("calibration_file", "urdf_file"):
            path = Path(config["pose_estimation"][key])
            try:
                if key == "urdf_file" and zipfile.is_zipfile(path):
                    with zipfile.ZipFile(path) as archive:
                        for index, name in enumerate(n for n in archive.namelist() if n.endswith(".urdf")):
                            save(f"models/{index}.urdf", archive.read(name), str(path) + ":" + name)
                else:
                    save("models/" + path.name, path.read_bytes(), path)
            except (OSError, ValueError, zipfile.BadZipFile) as exc:
                assets.append({"source": str(path), "error": str(exc)})
        (destination / "manifest.json").write_text(json.dumps(assets, ensure_ascii=False, indent=2), encoding="utf-8")
        self.record("replay_assets", assets=assets, units={"state_joints": "deg", "state_pose": "mm,deg", "sdk": "m,rad"})

    def close(self) -> None:
        with self._lock:
            if self._handle is not None:
                self._handle.close()
                self._handle = None
