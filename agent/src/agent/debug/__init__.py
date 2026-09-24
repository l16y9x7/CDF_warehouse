"""Development console backend and embedded frontend."""

from .router import create_debug_router
from .service import DebugService

__all__ = ["DebugService", "create_debug_router"]
