"""Fail-closed ownership for one independently spawned process group."""

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
from typing import Iterator, Optional, Sequence


_PROCESS_IDENTITY_TIMEOUT_SEC = 1.0


def _boot_id() -> str:
    try:
        return Path("/proc/sys/kernel/random/boot_id").read_text().strip()
    except OSError:
        return "unknown"


def _proc_stat(pid: int) -> tuple[Optional[str], Optional[int]]:
    try:
        raw = Path(f"/proc/{pid}/stat").read_text()
        fields = raw.rsplit(")", 1)[1].strip().split()
        return fields[0], int(fields[19])
    except (OSError, IndexError, ValueError):
        return None, None


def _cmdline(pid: int) -> tuple[str, ...]:
    try:
        raw = Path(f"/proc/{pid}/cmdline").read_bytes()
    except OSError:
        return ()
    return tuple(
        part.decode("utf-8", errors="replace")
        for part in raw.split(b"\0")
        if part
    )


def _command_hash(command: Sequence[str]) -> str:
    return hashlib.sha256(
        b"\0".join(str(part).encode("utf-8") for part in command)
    ).hexdigest()


def _live_command_matches_requested(
    live_command: Sequence[str],
    requested_command: Sequence[str],
) -> bool:
    """Accept direct exec or the kernel's interpreter-prefixed shebang argv."""
    live = tuple(str(part) for part in live_command)
    requested = tuple(str(part) for part in requested_command)
    return live == requested or (
        len(live) == len(requested) + 1 and live[1:] == requested
    )


@dataclass(frozen=True)
class OwnedProcessRecord:
    boot_id: str
    pid: int
    pgid: int
    sid: int
    start_ticks: int
    requested_command_sha256: str
    live_cmdline_sha256: str
    state: str
    started_at_unix_s: float
    member_cmdline_sha256: tuple[str, ...] = ()


class OwnedProcessGroup:
    """Start/stop one process group and never signal an unverified owner."""

    def __init__(
        self,
        *,
        record_file: str | Path,
        log_file: str | Path,
        stop_timeout_sec: float = 10.0,
        startup_probe_sec: float = 1.0,
    ) -> None:
        self.record_file = Path(record_file).expanduser().resolve()
        self.log_file = Path(log_file).expanduser().resolve()
        self.lock_file = self.record_file.with_suffix(
            self.record_file.suffix + ".lock"
        )
        self.stop_timeout_sec = float(stop_timeout_sec)
        self.startup_probe_sec = float(startup_probe_sec)
        if self.stop_timeout_sec <= 0 or self.startup_probe_sec <= 0:
            raise ValueError("process timeouts must be positive")
        self.record_file.parent.mkdir(mode=0o700, parents=True, exist_ok=True)
        self.log_file.parent.mkdir(mode=0o700, parents=True, exist_ok=True)
        self.record_file.parent.chmod(0o700)

    @contextmanager
    def _locked(self) -> Iterator[None]:
        with self.lock_file.open("a+", encoding="utf-8") as handle:
            fcntl.flock(handle.fileno(), fcntl.LOCK_EX)
            try:
                yield
            finally:
                fcntl.flock(handle.fileno(), fcntl.LOCK_UN)

    def _read_record(self) -> Optional[OwnedProcessRecord]:
        try:
            return OwnedProcessRecord(
                **json.loads(self.record_file.read_text(encoding="utf-8"))
            )
        except (OSError, ValueError, TypeError, json.JSONDecodeError):
            return None

    def _write_record(self, record: OwnedProcessRecord) -> None:
        fd, temp_name = tempfile.mkstemp(
            prefix=f".{self.record_file.stem}-",
            suffix=".json",
            dir=self.record_file.parent,
        )
        try:
            with os.fdopen(fd, "w", encoding="utf-8") as handle:
                json.dump(asdict(record), handle)
                handle.write("\n")
                handle.flush()
                os.fsync(handle.fileno())
            os.chmod(temp_name, 0o600)
            os.replace(temp_name, self.record_file)
        finally:
            try:
                os.unlink(temp_name)
            except FileNotFoundError:
                pass

    @staticmethod
    def _group_exists(record: OwnedProcessRecord) -> bool:
        if record.boot_id != _boot_id():
            return False
        for entry in Path("/proc").iterdir():
            if not entry.name.isdigit():
                continue
            pid = int(entry.name)
            state, _ = _proc_stat(pid)
            if state in (None, "Z"):
                continue
            try:
                if os.getpgid(pid) == record.pgid and os.getsid(pid) == record.sid:
                    return True
            except ProcessLookupError:
                pass
        return False

    @staticmethod
    def _identity_valid(record: OwnedProcessRecord) -> bool:
        state, start_ticks = _proc_stat(record.pid)
        if record.boot_id != _boot_id() or state in (None, "Z"):
            return False
        if start_ticks != record.start_ticks:
            return False
        if record.pid != record.pgid or record.pid != record.sid:
            return False
        try:
            if os.getpgid(record.pid) != record.pgid:
                return False
            if os.getsid(record.pid) != record.sid:
                return False
        except ProcessLookupError:
            return False
        command = _cmdline(record.pid)
        return bool(command) and _command_hash(command) == record.live_cmdline_sha256

    @staticmethod
    def _group_member_hashes(record: OwnedProcessRecord) -> tuple[str, ...]:
        hashes: list[str] = []
        for entry in Path("/proc").iterdir():
            if not entry.name.isdigit():
                continue
            pid = int(entry.name)
            if pid == record.pid:
                continue
            state, _ = _proc_stat(pid)
            if state in (None, "Z"):
                continue
            try:
                in_group = (
                    os.getpgid(pid) == record.pgid
                    and os.getsid(pid) == record.sid
                )
            except ProcessLookupError:
                continue
            if not in_group:
                continue
            command = _cmdline(pid)
            if not command:
                return ()
            hashes.append(_command_hash(command))
        return tuple(sorted(hashes))

    @classmethod
    def _orphan_identity_valid(cls, record: OwnedProcessRecord) -> bool:
        state, _ = _proc_stat(record.pid)
        if record.boot_id != _boot_id() or state not in (None, "Z"):
            return False
        if record.pid != record.pgid or record.pid != record.sid:
            return False
        expected = tuple(sorted(record.member_cmdline_sha256))
        current = cls._group_member_hashes(record)
        return bool(expected) and current == expected

    def status(self) -> dict[str, object]:
        with self._locked():
            return self._status_unlocked()

    def _status_unlocked(self) -> dict[str, object]:
        record_exists = self.record_file.exists()
        record = self._read_record()
        group_exists = bool(record and self._group_exists(record))
        leader_valid = bool(record and self._identity_valid(record))
        orphan_valid = bool(record and self._orphan_identity_valid(record))
        ownership_valid = leader_valid or orphan_valid
        return {
            "record_exists": record_exists,
            "running": group_exists and ownership_valid,
            "ownership_valid": ownership_valid,
            "ownership_mode": (
                "leader" if leader_valid else "verified_orphan" if orphan_valid else None
            ),
            "ownership_conflict": record_exists
            and (record is None or (group_exists and not ownership_valid)),
            "pid": record.pid if group_exists and ownership_valid else None,
            "state": record.state if group_exists and ownership_valid else None,
            "record": None if record is None else asdict(record),
        }

    def start(self, command: Sequence[str]) -> dict[str, object]:
        command = tuple(str(part) for part in command)
        if not command:
            raise ValueError("owned process command is required")
        requested_hash = _command_hash(command)
        with self._locked():
            record_exists = self.record_file.exists()
            record = self._read_record()
            if record_exists and record is None:
                raise RuntimeError("owned process record is unreadable")
            if record is not None:
                group_exists = self._group_exists(record)
                identity_valid = self._identity_valid(
                    record
                ) or self._orphan_identity_valid(record)
                if group_exists and not identity_valid:
                    raise RuntimeError(
                        "owned process group exists but identity cannot be verified"
                    )
                if (
                    group_exists
                    and self._identity_valid(record)
                    and record.state == "RUNNING"
                    and record.requested_command_sha256 == requested_hash
                ):
                    result = self._status_unlocked()
                    result["changed"] = False
                    return result
                self._stop_unlocked(record)
            self._start_unlocked(command, requested_hash)
            result = self._status_unlocked()
            result["changed"] = True
            return result

    def stop(self) -> dict[str, object]:
        with self._locked():
            record_exists = self.record_file.exists()
            record = self._read_record()
            if record_exists and record is None:
                raise RuntimeError("owned process record is unreadable")
            self._stop_unlocked(record)
            return self._status_unlocked()

    def _stop_unlocked(self, record: Optional[OwnedProcessRecord]) -> None:
        if record is None:
            return
        group_exists = self._group_exists(record)
        if not group_exists:
            self._reap_child_best_effort(record.pid)
            self.record_file.unlink(missing_ok=True)
            return
        if not (
            self._identity_valid(record)
            or self._orphan_identity_valid(record)
        ):
            raise RuntimeError(
                "refusing to signal process group because ownership cannot be verified"
            )
        self._signal_and_wait(record, signal.SIGINT, self.stop_timeout_sec)
        if self._group_exists(record):
            self._signal_and_wait(record, signal.SIGTERM, 2.0)
        if self._group_exists(record):
            self._signal_and_wait(record, signal.SIGKILL, 2.0)
        if self._group_exists(record):
            raise RuntimeError(f"owned process group {record.pgid} did not stop")
        self._reap_child_best_effort(record.pid)
        self.record_file.unlink(missing_ok=True)

    def _signal_and_wait(
        self,
        record: OwnedProcessRecord,
        signal_number: signal.Signals,
        timeout_sec: float,
    ) -> None:
        try:
            os.killpg(record.pgid, signal_number)
        except ProcessLookupError:
            return
        deadline = time.monotonic() + timeout_sec
        while time.monotonic() < deadline and self._group_exists(record):
            time.sleep(0.05)

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

    def _start_unlocked(
        self,
        command: tuple[str, ...],
        requested_hash: str,
    ) -> None:
        with self.log_file.open("ab", buffering=0) as log_handle:
            process = subprocess.Popen(
                command,
                stdin=subprocess.DEVNULL,
                stdout=log_handle,
                stderr=subprocess.STDOUT,
                start_new_session=True,
                close_fds=True,
            )
        try:
            identity_deadline = time.monotonic() + _PROCESS_IDENTITY_TIMEOUT_SEC
            while True:
                code = process.poll()
                if code is not None:
                    raise RuntimeError(
                        f"owned process exited early before identity verification "
                        f"with code {code}"
                    )
                try:
                    pgid = os.getpgid(process.pid)
                    sid = os.getsid(process.pid)
                except ProcessLookupError:
                    pgid = sid = None
                state, start_ticks = _proc_stat(process.pid)
                command_live = _cmdline(process.pid)
                if (
                    pgid == process.pid
                    and sid == process.pid
                    and state not in (None, "Z")
                    and start_ticks is not None
                    and command_live
                    and _live_command_matches_requested(command_live, command)
                ):
                    break
                if time.monotonic() >= identity_deadline:
                    raise RuntimeError(
                        "spawned process did not exec the requested command "
                        "in a verified session"
                    )
                time.sleep(0.01)
            provisional = OwnedProcessRecord(
                boot_id=_boot_id(),
                pid=process.pid,
                pgid=pgid,
                sid=sid,
                start_ticks=start_ticks,
                requested_command_sha256=requested_hash,
                live_cmdline_sha256=_command_hash(command_live),
                state="STARTING",
                started_at_unix_s=time.time(),
            )
            self._write_record(provisional)
            deadline = time.monotonic() + self.startup_probe_sec
            while True:
                code = process.poll()
                if code is not None:
                    raise RuntimeError(f"owned process exited early with code {code}")
                remaining = deadline - time.monotonic()
                if remaining <= 0:
                    break
                time.sleep(min(0.05, remaining))
            code = process.poll()
            if code is not None:
                raise RuntimeError(f"owned process exited early with code {code}")
            self._write_record(
                replace(
                    provisional,
                    state="RUNNING",
                    member_cmdline_sha256=self._group_member_hashes(
                        provisional
                    ),
                )
            )
        except Exception:
            try:
                os.killpg(process.pid, signal.SIGKILL)
            except ProcessLookupError:
                try:
                    process.kill()
                except ProcessLookupError:
                    pass
            self.record_file.unlink(missing_ok=True)
            raise


def main(argv: Optional[list[str]] = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("action", choices=("start", "status", "stop"))
    parser.add_argument("--record-file", required=True)
    parser.add_argument("--log-file", required=True)
    parser.add_argument("--stop-timeout-sec", type=float, default=10.0)
    parser.add_argument("--startup-probe-sec", type=float, default=1.0)
    parser.add_argument("command", nargs=argparse.REMAINDER)
    args = parser.parse_args(argv)
    command = args.command[1:] if args.command[:1] == ["--"] else args.command
    supervisor = OwnedProcessGroup(
        record_file=args.record_file,
        log_file=args.log_file,
        stop_timeout_sec=args.stop_timeout_sec,
        startup_probe_sec=args.startup_probe_sec,
    )
    try:
        if args.action == "start":
            result = supervisor.start(command)
        elif args.action == "stop":
            result = supervisor.stop()
        else:
            result = supervisor.status()
    except Exception as exc:
        print(json.dumps({"ok": False, "error": str(exc)}), file=sys.stderr)
        return 1
    print(json.dumps({"ok": True, **result}))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
