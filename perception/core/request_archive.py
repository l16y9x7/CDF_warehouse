"""Request evidence for JSON vision APIs, including validation and server errors."""

from __future__ import annotations

import hashlib
import json
import re
import shutil
import time
import traceback
from dataclasses import asdict
from datetime import date, datetime, timedelta
from pathlib import Path
from threading import Lock
from uuid import uuid4

from fastapi import HTTPException, Request
from fastapi.exception_handlers import http_exception_handler, request_validation_exception_handler
from fastapi.exceptions import RequestValidationError
from fastapi.routing import APIRoute
from starlette.concurrency import run_in_threadpool
from starlette.responses import PlainTextResponse, Response
from starlette.exceptions import HTTPException as StarletteHTTPException

from config import (
    PERCEPTION_REQUEST_DIR, PERCEPTION_REQUEST_LOG_PATH,
    PERCEPTION_REQUEST_RETENTION_DAYS, RECOGNIZE_SKU_BARCODE_LOG_PATH,
)
from core.image_io import decode_image_bytes, read_image_from_base64, read_image_from_path
from core.recognize_trace import RecognizePipelineTrace
from core.service_logging import file_logger

_cleanup_lock = Lock()
_last_cleanup = 0.0


def _without_image_base64(value):
    """Avoid duplicating image data, including in Pydantic validation responses."""
    if isinstance(value, dict):
        return {
            key: {"omitted": True, "encoded_length": len(item) if isinstance(item, str) else None}
            if key == "image_base64" else _without_image_base64(item)
            for key, item in value.items()
        }
    if isinstance(value, list):
        return [_without_image_base64(item) for item in value]
    return value


def _image_suffix(data: bytes) -> str:
    if data.startswith(b"\xff\xd8\xff"):
        return ".jpg"
    if data.startswith(b"\x89PNG\r\n\x1a\n"):
        return ".png"
    if data.startswith(b"RIFF") and data[8:12] == b"WEBP":
        return ".webp"
    if data.startswith(b"BM"):
        return ".bmp"
    if data.startswith((b"II*\x00", b"MM\x00*")):
        return ".tiff"
    return ".bin"


def cleanup_archives(root: Path, days: int, today: date | None = None) -> None:
    """Expire only completed archives we created; retain unknown and active data."""
    if days <= 0 or not root.is_dir():
        return
    root = root.resolve()
    cutoff = (today or date.today()) - timedelta(days=days)
    for day_dir in root.iterdir():
        if day_dir.is_symlink() or not day_dir.is_dir():
            continue
        try:
            archive_date = date.fromisoformat(day_dir.name)
        except ValueError:
            continue
        if archive_date.isoformat() != day_dir.name or archive_date >= cutoff:
            continue
        for request_dir in day_dir.iterdir():
            if (request_dir.is_symlink() or not request_dir.is_dir()
                    or not re.fullmatch(r"[0-9a-f]{32}", request_dir.name)):
                continue
            if not request_dir.resolve().is_relative_to(root):
                continue
            marker = request_dir / "trace.json"
            if marker.is_symlink() or not marker.is_file():
                continue
            try:
                record = json.loads(marker.read_text(encoding="utf-8"))
            except (OSError, ValueError):
                continue
            if (isinstance(record, dict) and record.get("archive_version") == 1 and record.get("state") == "completed"
                    and record.get("request_id") == request_dir.name):
                shutil.rmtree(request_dir)
        # Keep the date directory; another worker may still be using it.


class RequestArchive:
    def __init__(self, request: Request):
        self.request_id = uuid4().hex
        self.started = time.perf_counter()
        now = datetime.now().astimezone()
        self.directory = Path(PERCEPTION_REQUEST_DIR) / now.date().isoformat() / self.request_id
        self.metadata = {
            "archive_version": 1, "request_id": self.request_id,
            "received_at": now.isoformat(), "method": request.method, "path": request.url.path,
        }
        self.logger = file_logger("perception.requests", PERCEPTION_REQUEST_LOG_PATH)
        self.trace: RecognizePipelineTrace | None = None
        self.image_bytes: bytes | None = None
        self.image_error: HTTPException | None = None
        self.image_info: dict = {}
        self.archive_errors: list[str] = []
        self.error: dict | None = None

    def _write(self, filename: str, data: bytes) -> bool:
        try:
            self.directory.mkdir(parents=True, exist_ok=True)
            temporary = self.directory / (filename + ".tmp")
            temporary.write_bytes(data)
            temporary.replace(self.directory / filename)
            return True
        except OSError as error:
            self.archive_errors.append(f"{filename}: {error}")
            self.logger.exception("request_id=%s archive_write_failed file=%s", self.request_id, filename)
            return False

    def _json(self, filename: str, payload: dict) -> None:
        self._write(filename, json.dumps(payload, ensure_ascii=False, indent=2, default=str).encode("utf-8"))

    def begin(self, payload, body: bytes, parse_error: str | None = None) -> None:
        self.logger.info("request_id=%s received method=%s path=%s archive=%s",
                         self.request_id, self.metadata["method"], self.metadata["path"], self.directory)
        self._json("trace.json", {**self.metadata, "state": "received"})
        request_record = {
            **self.metadata, "body": _without_image_base64(payload),
            "body_bytes": len(body), "body_sha256": hashlib.sha256(body).hexdigest(),
        }
        if parse_error is not None:
            request_record["body_parse_error"] = parse_error
            request_record["raw_body_file"] = "request_body.bin"
            self._write("request_body.bin", body)
        # Persist request parameters before any image I/O or inference.
        self._json("request.json", request_record)
        if isinstance(payload, dict):
            path, encoded = payload.get("image_path"), payload.get("image_base64")
            has_path = isinstance(path, str) and bool(path.strip())
            has_encoded = isinstance(encoded, str) and bool(encoded.strip())
            if has_path != has_encoded:
                self.image_info["source"] = "path" if has_path else "base64"
                try:
                    self.image_bytes = read_image_from_path(path.strip()) if has_path else read_image_from_base64(encoded)
                except HTTPException as error:
                    self.image_error = error
                    self.image_info["error"] = str(error.detail)
                else:
                    filename = "input" + _image_suffix(self.image_bytes)
                    self.image_info.update({
                        "file": filename, "bytes": len(self.image_bytes),
                        "sha256": hashlib.sha256(self.image_bytes).hexdigest(),
                        "saved": self._write(filename, self.image_bytes),
                    })
        request_record["image"] = self.image_info
        self._json("request.json", request_record)
        self._cleanup()

    def _cleanup(self) -> None:
        global _last_cleanup
        if PERCEPTION_REQUEST_RETENTION_DAYS <= 0 or not _cleanup_lock.acquire(blocking=False):
            return
        try:
            if time.monotonic() - _last_cleanup >= 3600:
                _last_cleanup = time.monotonic()
                cleanup_archives(Path(PERCEPTION_REQUEST_DIR), PERCEPTION_REQUEST_RETENTION_DAYS)
        except OSError:
            self.logger.exception("request_id=%s archive_cleanup_failed", self.request_id)
        finally:
            _cleanup_lock.release()

    def load_image(self):
        if self.image_error is not None:
            raise self.image_error
        if self.image_bytes is None:
            raise HTTPException(status_code=400, detail="没有可读取的输入图片")
        # Decode exactly the bytes archived above, even if the caller overwrites its file.
        return decode_image_bytes(self.image_bytes)

    def finish(self, response: Response) -> None:
        raw = response.body
        try:
            response_body = json.loads(raw)
        except (ValueError, UnicodeError):
            response_body = raw.decode("utf-8", errors="replace")
        elapsed = round((time.perf_counter() - self.started) * 1000, 1)
        self._json("response.json", {
            "request_id": self.request_id, "http_status": response.status_code,
            "body": _without_image_base64(response_body),
        })
        status = "ERROR" if response.status_code >= 400 else (
            response_body.get("status", "OK") if isinstance(response_body, dict) else "OK"
        )
        pipeline = None
        if self.trace is not None:
            self.trace.request_id = self.request_id
            self.trace.status = status
            self.trace.total_ms = elapsed
            if self.image_info.get("saved"):
                self.trace.saved_image_path = str(self.directory / self.image_info["file"])
            if status == "ERROR" and not self.trace.failure_reason:
                self.trace.failure_reason = (self.error or {}).get("type", "request_failed")
            pipeline = asdict(self.trace)
            file_logger("perception.recognize_sku_barcode", RECOGNIZE_SKU_BARCODE_LOG_PATH).info(
                "\n%s", self.trace.format_summary())
        self._json("trace.json", {
            **self.metadata, "state": "completed", "status": status,
            "http_status": response.status_code, "total_ms": elapsed,
            "completed_at": datetime.now().astimezone().isoformat(),
            "image": self.image_info, "pipeline": pipeline, "error": self.error,
            "archive_errors": self.archive_errors,
        })
        self.logger.info("request_id=%s completed http_status=%s status=%s total_ms=%s archive_errors=%s",
                         self.request_id, response.status_code, status, elapsed, len(self.archive_errors))


class ArchivedRoute(APIRoute):
    def get_route_handler(self):
        original_handler = super().get_route_handler()

        async def handle(request: Request) -> Response:
            archive = RequestArchive(request)
            request.state.archive = archive
            body = await request.body()
            parse_error = None
            try:
                payload = json.loads(body)
            except (ValueError, UnicodeError) as error:
                payload = None
                parse_error = str(error)
            await run_in_threadpool(archive.begin, payload, body, parse_error)
            try:
                response = await original_handler(request)
            except RequestValidationError as error:
                archive.error = {"type": "validation_error"}
                response = await request_validation_exception_handler(request, error)
            except StarletteHTTPException as error:
                archive.error = {"type": "http_error", "detail": error.detail,
                                 "traceback": traceback.format_exc()}
                response = await http_exception_handler(request, error)
            except Exception as error:
                archive.error = {"type": type(error).__name__, "detail": str(error), "traceback": traceback.format_exc()}
                archive.logger.exception("request_id=%s unhandled_exception", archive.request_id)
                response = PlainTextResponse("Internal Server Error", status_code=500)
            response.headers["X-Request-ID"] = archive.request_id
            await run_in_threadpool(archive.finish, response)
            return response

        return handle
