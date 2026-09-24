"""One-second processing pause shared by the in-process mocks and mock HTTP modules."""

import asyncio
import os
import sys
import time

DELAY_ENV = "AGENT_MOCK_PROCESS_DELAY_S"
DEFAULT_DELAY_S = 1.0


def mock_process_delay_s() -> float:
    raw = os.getenv(DELAY_ENV)
    if raw is None:
        # unittest is already imported when tests call these mocks.
        return 0.0 if "unittest" in sys.modules else DEFAULT_DELAY_S
    try:
        return max(0.0, float(raw))
    except ValueError:
        return DEFAULT_DELAY_S


def pause_mock_processing() -> None:
    time.sleep(mock_process_delay_s())


async def pause_mock_processing_async() -> None:
    await asyncio.sleep(mock_process_delay_s())
