"""Thread-safe initialization of rotating business logs."""

import logging
from logging.handlers import RotatingFileHandler
from pathlib import Path
from threading import Lock

from config import PERCEPTION_LOG_BACKUP_COUNT, PERCEPTION_LOG_MAX_BYTES

_lock = Lock()


def file_logger(name: str, path: str) -> logging.Logger:
    logger = logging.getLogger(name)
    with _lock:
        if logger.handlers:
            return logger
        logger.setLevel(logging.INFO)
        logger.propagate = False
        formatter = logging.Formatter(
            "%(asctime)s %(levelname)s [%(name)s] %(message)s",
            datefmt="%Y-%m-%d %H:%M:%S",
        )
        try:
            Path(path).parent.mkdir(parents=True, exist_ok=True)
            handler = RotatingFileHandler(
                path, maxBytes=max(1, PERCEPTION_LOG_MAX_BYTES),
                backupCount=max(1, PERCEPTION_LOG_BACKUP_COUNT), encoding="utf-8",
            )
        except OSError:
            # A logging failure must not turn a valid recognition into HTTP 500.
            handler = logging.StreamHandler()
            handler.setFormatter(formatter)
            logger.addHandler(handler)
            logger.exception("cannot open business log path=%s; using stderr", path)
            return logger
        handler.setFormatter(formatter)
        logger.addHandler(handler)
    return logger
