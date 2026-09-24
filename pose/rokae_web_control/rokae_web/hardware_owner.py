"""Process lifetime lock: one mui backend owns all three SDK connections."""
from __future__ import annotations

import os
from pathlib import Path


class HardwareOwner:
    def __init__(self, path):
        self.path = Path(path)
        self.handle = None

    def acquire(self):
        import fcntl
        self.path.parent.mkdir(parents=True, exist_ok=True, mode=0o700)
        handle = self.path.open("a+")
        try:
            fcntl.flock(handle, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError as exc:
            handle.close()
            raise RuntimeError("mui 运控后端已运行；请复用网页和遥测接口，不要重复启动 SDK 进程") from exc
        handle.seek(0)
        handle.truncate()
        handle.write(str(os.getpid()) + "\n")
        handle.flush()
        self.handle = handle

    def close(self):
        if self.handle is not None:
            self.handle.close()
            self.handle = None
