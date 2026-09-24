"""Own and restart the head RealSense launch process without touching wrists."""

from __future__ import annotations

import argparse
import fcntl
import hashlib
import json
import os
import signal
import subprocess
import sys
import tempfile
import time
from contextlib import contextmanager
from dataclasses import asdict, dataclass, replace
from pathlib import Path
from typing import Iterator, Optional

import yaml


_SUPPORTED_RESOLUTIONS = (720, 1080)
_DEFAULT_RUNTIME_DIR = "/tmp/retail_nav_bridge/head_camera"
_PROCESS_IDENTITY_TIMEOUT_SEC = 1.0


def _boot_id() -> str:
    try:
        return Path("/proc/sys/kernel/random/boot_id").read_text(
            encoding="utf-8"
        ).strip()
    except OSError:
        return "unknown"


def _process_start_ticks(pid: int) -> Optional[int]:
    try:
        raw = Path(f"/proc/{pid}/stat").read_text(encoding="utf-8")
        fields = raw.rsplit(")", 1)[1].strip().split()
    except OSError:
        return None
    except IndexError:
        return None
    try:
        return int(fields[19])
    except (IndexError, ValueError):
        return None


def _process_state(pid: int) -> Optional[str]:
    try:
        raw = Path(f"/proc/{pid}/stat").read_text(encoding="utf-8")
        fields = raw.rsplit(")", 1)[1].strip().split()
    except (OSError, IndexError):
        return None
    return fields[0] if fields else None


def _process_cmdline(pid: int) -> tuple[str, ...]:
    try:
        raw = Path(f"/proc/{pid}/cmdline").read_bytes()
    except OSError:
        return ()
    return tuple(
        part.decode("utf-8", errors="replace")
        for part in raw.split(b"\0")
        if part
    )


@dataclass(frozen=True)
class HeadCameraProcessRecord:
    boot_id: str
    pid: int
    pgid: int
    sid: int
    start_ticks: int
    resolution: int
    serial_no: str
    command_sha256: str
    started_at_unix_s: float
    live_cmdline_sha256: str = ""
    state: str = "RUNNING"


class HeadCameraProcessSupervisor:
    """Manage one independently-owned head-camera launch process group."""

    def __init__(
        self,
        *,
        profile_configs: dict[int, str | Path],
        serial_no: str,
        runtime_dir: str | Path = _DEFAULT_RUNTIME_DIR,
        ros2_executable: str = "/opt/ros/humble/bin/ros2",
        launch_package: str = "retail_nav_bridge",
        launch_file: str = "tianji_cameras.launch.py",
        stop_timeout_sec: float = 8.0,
        startup_probe_sec: float = 1.0,
    ) -> None:
        self.profile_configs = {
            int(resolution): Path(path).resolve()
            for resolution, path in profile_configs.items()
        }
        if set(self.profile_configs) != set(_SUPPORTED_RESOLUTIONS):
            raise ValueError("head profile configs must contain 720 and 1080")
        for resolution, path in self.profile_configs.items():
            if not path.is_file():
                raise ValueError(
                    f"head {resolution} profile config does not exist: {path}"
                )
        self.serial_no = str(serial_no).strip().lstrip("_")
        if not self.serial_no:
            raise ValueError("head camera serial_no is required")
        self.runtime_dir = Path(runtime_dir).expanduser().resolve()
        self.ros2_executable = str(ros2_executable)
        self.launch_package = str(launch_package)
        self.launch_file = str(launch_file)
        self.stop_timeout_sec = float(stop_timeout_sec)
        self.startup_probe_sec = float(startup_probe_sec)
        if self.stop_timeout_sec <= 0 or self.startup_probe_sec <= 0:
            raise ValueError("head process timeouts must be positive")
        self.runtime_dir.mkdir(mode=0o700, parents=True, exist_ok=True)
        try:
            self.runtime_dir.chmod(0o700)
        except OSError:
            pass
        self._lock_path = self.runtime_dir / "head-camera.lock"
        self._record_path = self.runtime_dir / "head-camera.json"
        self._log_path = self.runtime_dir / "head-camera.log"

    @contextmanager
    def _locked(self) -> Iterator[None]:
        with self._lock_path.open("a+", encoding="utf-8") as lock_file:
            fcntl.flock(lock_file.fileno(), fcntl.LOCK_EX)
            try:
                yield
            finally:
                fcntl.flock(lock_file.fileno(), fcntl.LOCK_UN)

    def _command(self, resolution: int) -> list[str]:
        try:
            config_path = self.profile_configs[int(resolution)]
        except (KeyError, TypeError, ValueError) as exc:
            raise ValueError("resolution must be 720 or 1080") from exc
        return [
            self.ros2_executable,
            "launch",
            self.launch_package,
            self.launch_file,
            f"camera_device_config:={config_path}",
        ]

    @staticmethod
    def _command_sha256(command: list[str] | tuple[str, ...]) -> str:
        payload = b"\0".join(part.encode("utf-8") for part in command)
        return hashlib.sha256(payload).hexdigest()

    @staticmethod
    def _live_command_matches_requested(
        live_command: list[str] | tuple[str, ...],
        requested_command: list[str] | tuple[str, ...],
    ) -> bool:
        """Accept direct exec or the kernel's interpreter-prefixed shebang argv."""
        live = tuple(str(part) for part in live_command)
        requested = tuple(str(part) for part in requested_command)
        return live == requested or (
            len(live) == len(requested) + 1 and live[1:] == requested
        )

    def _read_record(self) -> Optional[HeadCameraProcessRecord]:
        try:
            payload = json.loads(self._record_path.read_text(encoding="utf-8"))
            return HeadCameraProcessRecord(**payload)
        except (OSError, ValueError, TypeError, json.JSONDecodeError):
            return None

    def _write_record(self, record: HeadCameraProcessRecord) -> None:
        fd, temp_name = tempfile.mkstemp(
            prefix=".head-camera-",
            suffix=".json",
            dir=self.runtime_dir,
            text=True,
        )
        try:
            with os.fdopen(fd, "w", encoding="utf-8") as handle:
                json.dump(asdict(record), handle, ensure_ascii=False)
                handle.write("\n")
                handle.flush()
                os.fsync(handle.fileno())
            os.replace(temp_name, self._record_path)
        finally:
            try:
                os.unlink(temp_name)
            except FileNotFoundError:
                pass

    def _record_matches_live_process(
        self,
        record: HeadCameraProcessRecord,
    ) -> bool:
        if record.boot_id != _boot_id():
            return False
        if _process_start_ticks(record.pid) != record.start_ticks:
            return False
        if _process_state(record.pid) == "Z":
            return False
        try:
            if os.getpgid(record.pid) != record.pgid:
                return False
            if os.getsid(record.pid) != record.sid:
                return False
        except ProcessLookupError:
            return False
        if record.pid != record.pgid or record.pid != record.sid:
            return False
        expected = self._command(record.resolution)
        if record.command_sha256 != self._command_sha256(expected):
            return False
        if not record.live_cmdline_sha256:
            return False
        live_command = _process_cmdline(record.pid)
        return (
            bool(live_command)
            and record.live_cmdline_sha256
            == self._command_sha256(live_command)
        )

    @staticmethod
    def _owned_group_member_pids(
        record: HeadCameraProcessRecord,
    ) -> list[int]:
        members: list[int] = []
        for entry in Path("/proc").iterdir():
            if not entry.name.isdigit():
                continue
            pid = int(entry.name)
            try:
                if (
                    os.getpgid(pid) == record.pgid
                    and os.getsid(pid) == record.sid
                    and _process_state(pid) != "Z"
                ):
                    members.append(pid)
            except ProcessLookupError:
                continue
        return members

    def _head_node_matches_serial(self, pid: int) -> bool:
        command = _process_cmdline(pid)
        if not command:
            return False
        if Path(command[0]).name != "realsense2_camera_node":
            return False
        if "__node:=head" not in command or "__ns:=/camera" not in command:
            return False
        direct_serials = {
            part.split(":=", 1)[1].lstrip("_")
            for part in command
            if part.startswith("serial_no:=") and ":=" in part
        }
        if self.serial_no in direct_serials:
            return True
        parameter_files: list[Path] = []
        for index, part in enumerate(command[:-1]):
            if part == "--params-file":
                parameter_files.append(Path(command[index + 1]))
        for path in parameter_files:
            try:
                payload = yaml.safe_load(path.read_text(encoding="utf-8")) or {}
            except (OSError, ValueError, TypeError, yaml.YAMLError):
                continue
            for node in payload.values() if isinstance(payload, dict) else ():
                if not isinstance(node, dict):
                    continue
                parameters = node.get("ros__parameters")
                if not isinstance(parameters, dict):
                    continue
                serial = str(parameters.get("serial_no") or "").lstrip("_")
                if (
                    serial == self.serial_no
                    and parameters.get("camera_name") == "head"
                ):
                    return True
        return False

    def _record_matches_owned_orphan_group(
        self,
        record: HeadCameraProcessRecord,
    ) -> bool:
        if record.boot_id != _boot_id():
            return False
        if _process_state(record.pid) not in (None, "Z"):
            return False
        if record.pid != record.pgid or record.pid != record.sid:
            return False
        if record.command_sha256 != self._command_sha256(
            self._command(record.resolution)
        ):
            return False
        members = self._owned_group_member_pids(record)
        return bool(members) and all(
            self._head_node_matches_serial(pid) for pid in members
        )

    @staticmethod
    def _owned_group_exists(record: HeadCameraProcessRecord) -> bool:
        if record.boot_id != _boot_id():
            return False
        proc_root = Path("/proc")
        for entry in proc_root.iterdir():
            if not entry.name.isdigit():
                continue
            pid = int(entry.name)
            try:
                if (
                    os.getpgid(pid) == record.pgid
                    and os.getsid(pid) == record.sid
                    and _process_state(pid) != "Z"
                ):
                    return True
            except ProcessLookupError:
                continue
        return False

    @staticmethod
    def _unowned_head_node_pids() -> list[int]:
        matches: list[int] = []
        for entry in Path("/proc").iterdir():
            if not entry.name.isdigit():
                continue
            pid = int(entry.name)
            command = _process_cmdline(pid)
            if not command:
                continue
            executable = Path(command[0]).name
            if executable != "realsense2_camera_node":
                continue
            if "__node:=head" not in command or "__ns:=/camera" not in command:
                continue
            if _process_state(pid) != "Z":
                matches.append(pid)
        return matches

    def status(self) -> dict[str, object]:
        with self._locked():
            return self._status_unlocked()

    def _status_unlocked(self) -> dict[str, object]:
        record_exists = self._record_path.exists()
        record = self._read_record()
        group_exists = bool(record and self._owned_group_exists(record))
        leader_ownership_valid = bool(
            record and self._record_matches_live_process(record)
        )
        orphan_ownership_valid = bool(
            record and self._record_matches_owned_orphan_group(record)
        )
        ownership_valid = leader_ownership_valid or orphan_ownership_valid
        running = group_exists and ownership_valid
        return {
            "owned": record_exists,
            "running": running,
            "ownership_valid": ownership_valid,
            "ownership_mode": (
                "leader"
                if leader_ownership_valid
                else "verified_orphan"
                if orphan_ownership_valid
                else None
            ),
            "ownership_conflict": record_exists
            and (record is None or (group_exists and not ownership_valid)),
            "resolution": record.resolution if running else None,
            "pid": record.pid if running else None,
            "pgid": record.pgid if running else None,
            "record": None if record is None else asdict(record),
            "runtime_dir": str(self.runtime_dir),
            "log_file": str(self._log_path),
        }

    def apply(self, resolution: int, *, force_restart: bool = False) -> dict:
        resolution = int(resolution)
        if resolution not in _SUPPORTED_RESOLUTIONS:
            raise ValueError("resolution must be 720 or 1080")
        with self._locked():
            record_exists = self._record_path.exists()
            record = self._read_record()
            if record_exists and record is None:
                raise RuntimeError("head camera ownership record is unreadable")
            if (
                not force_restart
                and record is not None
                and record.resolution == resolution
                and record.state == "RUNNING"
                and self._record_matches_live_process(record)
                and self._owned_group_exists(record)
            ):
                result = self._status_unlocked()
                result["changed"] = False
                return result
            self._stop_unlocked(record)
            self._start_unlocked(resolution)
            result = self._status_unlocked()
            result["changed"] = True
            return result

    def stop(self) -> dict:
        with self._locked():
            record_exists = self._record_path.exists()
            record = self._read_record()
            if record_exists and record is None:
                raise RuntimeError("head camera ownership record is unreadable")
            self._stop_unlocked(record)
            return self._status_unlocked()

    def _stop_unlocked(
        self,
        record: Optional[HeadCameraProcessRecord],
    ) -> None:
        if record is None:
            return
        ownership_valid = self._record_matches_live_process(
            record
        ) or self._record_matches_owned_orphan_group(record)
        group_exists = self._owned_group_exists(record)
        if not ownership_valid and group_exists:
            raise RuntimeError(
                "refusing to signal head camera process group because "
                "ownership cannot be verified; inspect the runtime record "
                f"and process group {record.pgid}"
            )
        if not group_exists:
            self._reap_child_best_effort(record.pid)
            try:
                self._record_path.unlink()
            except FileNotFoundError:
                pass
            return

        deadline = time.monotonic() + self.stop_timeout_sec
        try:
            os.killpg(record.pgid, signal.SIGINT)
        except ProcessLookupError:
            pass
        while time.monotonic() < deadline:
            if not self._owned_group_exists(record):
                break
            time.sleep(0.05)
        if self._owned_group_exists(record):
            try:
                os.killpg(record.pgid, signal.SIGTERM)
            except ProcessLookupError:
                pass
            term_deadline = time.monotonic() + min(2.0, self.stop_timeout_sec)
            while time.monotonic() < term_deadline:
                if not self._owned_group_exists(record):
                    break
                time.sleep(0.05)
        if self._owned_group_exists(record):
            try:
                os.killpg(record.pgid, signal.SIGKILL)
            except ProcessLookupError:
                pass
            kill_deadline = time.monotonic() + 2.0
            while time.monotonic() < kill_deadline:
                if not self._owned_group_exists(record):
                    break
                time.sleep(0.05)
        if self._owned_group_exists(record):
            raise RuntimeError(
                f"owned head camera process group {record.pgid} did not stop"
            )
        self._reap_child_best_effort(record.pid)
        try:
            self._record_path.unlink()
        except FileNotFoundError:
            pass

    @staticmethod
    def _reap_child_best_effort(pid: int, timeout_sec: float = 0.5) -> None:
        """Reap a stopped leader only when this supervisor is its parent."""
        deadline = time.monotonic() + timeout_sec
        while True:
            try:
                waited_pid, _ = os.waitpid(pid, os.WNOHANG)
            except ChildProcessError:
                return
            if waited_pid == pid:
                return
            if time.monotonic() >= deadline:
                return
            time.sleep(0.01)

    def _start_unlocked(self, resolution: int) -> None:
        unowned = self._unowned_head_node_pids()
        if unowned:
            raise RuntimeError(
                "refusing to start a duplicate unowned head camera node: "
                f"pids={unowned}"
            )
        command = self._command(resolution)
        self._log_path.parent.mkdir(mode=0o700, parents=True, exist_ok=True)
        with self._log_path.open("ab", buffering=0) as log_file:
            process = subprocess.Popen(
                command,
                stdin=subprocess.DEVNULL,
                stdout=log_file,
                stderr=subprocess.STDOUT,
                start_new_session=True,
                close_fds=True,
            )
        pgid: Optional[int] = None
        sid: Optional[int] = None
        try:
            identity_deadline = time.monotonic() + _PROCESS_IDENTITY_TIMEOUT_SEC
            while True:
                return_code = process.poll()
                if return_code is not None:
                    raise RuntimeError(
                        "head camera launch exited early before identity verification "
                        f"with code {return_code}; see {self._log_path}"
                    )
                try:
                    pgid = os.getpgid(process.pid)
                    sid = os.getsid(process.pid)
                except ProcessLookupError:
                    pgid = sid = None
                process_state = _process_state(process.pid)
                start_ticks = _process_start_ticks(process.pid)
                live_command = _process_cmdline(process.pid)
                if (
                    pgid == process.pid
                    and sid == process.pid
                    and process_state not in (None, "Z")
                    and start_ticks is not None
                    and live_command
                    and self._live_command_matches_requested(
                        live_command,
                        command,
                    )
                ):
                    break
                if time.monotonic() >= identity_deadline:
                    raise RuntimeError(
                        "head camera launch did not exec the requested command "
                        "in a verified session"
                    )
                time.sleep(0.01)
            provisional_record = HeadCameraProcessRecord(
                boot_id=_boot_id(),
                pid=process.pid,
                pgid=pgid,
                sid=sid,
                start_ticks=start_ticks,
                resolution=resolution,
                serial_no=self.serial_no,
                command_sha256=self._command_sha256(command),
                started_at_unix_s=time.time(),
                live_cmdline_sha256=self._command_sha256(live_command),
                state="STARTING",
            )
            # Persist ownership immediately after the new session identity is
            # known. A supervisor/gateway crash during the startup probe can
            # then recover or stop this exact process group.
            self._write_record(provisional_record)
            deadline = time.monotonic() + self.startup_probe_sec
            while True:
                return_code = process.poll()
                if return_code is not None:
                    raise RuntimeError(
                        "head camera launch exited early "
                        f"with code {return_code}; see {self._log_path}"
                    )
                remaining = deadline - time.monotonic()
                if remaining <= 0:
                    break
                time.sleep(min(0.05, remaining))
            return_code = process.poll()
            if return_code is not None:
                raise RuntimeError(
                    "head camera launch exited early "
                    f"with code {return_code}; see {self._log_path}"
                )
            self._write_record(replace(provisional_record, state="RUNNING"))
        except Exception:
            # Popen succeeded in this call. Tear down its exact new session on
            # every later failure, including record-write failures, so an
            # unowned head process cannot survive a partial start.
            try:
                # start_new_session=True defines the new PGID as the spawned
                # PID. This remains the trusted target even if the launch
                # leader exits between Popen and getpgid/getsid.
                os.killpg(process.pid, signal.SIGKILL)
            except ProcessLookupError:
                try:
                    process.kill()
                except ProcessLookupError:
                    pass
            try:
                self._record_path.unlink()
            except FileNotFoundError:
                pass
            raise


def _default_profile_configs(config_dir: Path) -> dict[int, Path]:
    return {
        720: config_dir / "tianji_head_camera_720.yaml",
        1080: config_dir / "tianji_head_camera_1080.yaml",
    }


def main(argv: Optional[list[str]] = None) -> int:
    parser = argparse.ArgumentParser(
        description="Manage the independently-owned TianJi head camera",
    )
    parser.add_argument("action", choices=("apply", "ensure-default", "status", "stop"))
    parser.add_argument("resolution", nargs="?", choices=("720", "1080"))
    parser.add_argument(
        "--config-dir",
        default=str(Path(__file__).resolve().parents[1] / "config"),
    )
    parser.add_argument(
        "--runtime-dir",
        default=os.environ.get(
            "TIANJI_HEAD_CAMERA_RUNTIME_DIR",
            _DEFAULT_RUNTIME_DIR,
        ),
    )
    parser.add_argument(
        "--serial-no",
        default=os.environ.get("TIANJI_HEAD_CAMERA_SERIAL", "349622071479"),
    )
    parser.add_argument("--force", action="store_true")
    args = parser.parse_args(argv)

    config_dir = Path(args.config_dir).resolve()
    supervisor = HeadCameraProcessSupervisor(
        profile_configs=_default_profile_configs(config_dir),
        serial_no=args.serial_no,
        runtime_dir=args.runtime_dir,
    )
    try:
        if args.action == "status":
            result = supervisor.status()
        elif args.action == "stop":
            result = supervisor.stop()
        else:
            resolution = 720 if args.action == "ensure-default" else args.resolution
            if resolution is None:
                parser.error("apply requires resolution 720 or 1080")
            result = supervisor.apply(
                int(resolution),
                force_restart=bool(args.force),
            )
    except Exception as exc:
        print(
            json.dumps(
                {"ok": False, "error": str(exc)},
                ensure_ascii=False,
            )
        )
        return 1
    print(json.dumps({"ok": True, **result}, ensure_ascii=False))
    return 0


if __name__ == "__main__":
    sys.exit(main())
