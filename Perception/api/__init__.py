"""HTTP API routes."""

from api.qr_parse import router as qr_parse_router
from api.sku_locate import router as sku_locate_router
from api.sku_recognize import router as sku_recognize_router

__all__ = [
    "qr_parse_router",
    "sku_locate_router",
    "sku_recognize_router",
]
