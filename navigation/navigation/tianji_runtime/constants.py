"""Shared constants for retail_nav_bridge."""

from __future__ import annotations

from enum import Enum


class NavState(str, Enum):
    UNREADY = "UNREADY"
    IDLE = "IDLE"
    MAPPING = "MAPPING"
    SAVING = "SAVING"
    LOCALIZING = "LOCALIZING"
    READY = "READY"
    NAVIGATING = "NAVIGATING"
    ARRIVED = "ARRIVED"
    PAUSED = "PAUSED"
    HOLDING = "HOLDING"
    ERROR = "ERROR"


class Mode(str, Enum):
    NONE = "none"
    MAPPING = "mapping"
    NAVIGATION = "navigation"


class ResultCode(str, Enum):
    ARRIVED = "arrived"
    TIMEOUT = "timeout"
    BLOCKED = "blocked"
    CANCELLED = "cancelled"
    ERROR = "error"
    DONE = "done"


class EventType(str, Enum):
    ARRIVED = "ARRIVED"
    BLOCKED = "BLOCKED"
    MAPPING_DONE = "MAPPING_DONE"
    RELOC_OK = "RELOC_OK"
    COLLISION_RISK = "COLLISION_RISK"
    OUT_OF_BOUNDARY_RISK = "OUT_OF_BOUNDARY_RISK"
    ERROR = "ERROR"
    CANCELLED = "CANCELLED"


class ErrorCode:
    OK = 0
    NOT_READY = 1
    INVALID_STATE = 2
    UNKNOWN_STATION = 3
    LOW_CONFIDENCE = 4
    UNREACHABLE = 5
    TIMEOUT = 6
    CANCELLED = 7
    SAFETY = 8
    VENDOR = 9


# mapping_status aligned with server/State
MAPPING_NONE = 1
MAPPING_SCANNING = 2
MAPPING_BUILDING = 3
MAPPING_DONE = 4
