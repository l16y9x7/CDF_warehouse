"""Backward-compatible entry point. Prefer `python -m uvicorn app:app`."""

from app import app

__all__ = ["app"]
