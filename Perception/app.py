"""FastAPI application factory for the Perception vision barcode service.

Run from the Perception directory:

    python -m uvicorn app:app --host 0.0.0.0 --port 25546

Or with nohup:

    SAM3_URL=http://host:port/api/v1/segment \\
    nohup python -m uvicorn app:app --host 0.0.0.0 --port 25546 \\
      > perception_barcode.log 2>&1 &
"""

from __future__ import annotations

import os

import uvicorn
from fastapi import FastAPI

from api import sku_locate_router, sku_recognize_router
from config import SERVICE_BIND_HOST


def create_app() -> FastAPI:
    application = FastAPI(title="Perception Vision Barcode API", version="1.0.0")
    application.include_router(sku_locate_router)
    application.include_router(sku_recognize_router)

    @application.get("/perception/health")
    def perception_health() -> dict[str, str]:
        """Report that the barcode-only gateway is ready."""
        return {"status": "READY"}

    return application


app = create_app()


if __name__ == "__main__":
    port = int(os.getenv("PORT", "8083"))
    uvicorn.run(app, host=SERVICE_BIND_HOST, port=port)
