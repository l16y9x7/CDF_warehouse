import asyncio
import json
from pathlib import Path
from typing import Annotated

from fastapi import APIRouter, Header, Request
from fastapi.responses import JSONResponse, RedirectResponse, Response, StreamingResponse
from pydantic import BaseModel, ConfigDict, Field

from .service import OrderConflict, OrderNotFound, OrderService

STATIC_DIR = Path(__file__).with_name("static")


class StrictModel(BaseModel):
    model_config = ConfigDict(extra="forbid")


class OrderItemRequest(StrictModel):
    sku_id: str = Field(min_length=1)
    quantity: int = Field(ge=1, le=20)
    agv_row: str = Field(pattern=r"^L[1-5]$")
    agv_column: str = Field(pattern=r"^[12]$")


class CreateOrderRequest(StrictModel):
    items: list[OrderItemRequest] = Field(min_length=1)
    basket_row: str = Field(pattern=r"^L[1-4]$")
    basket_column: str = Field(pattern=r"^[1-5]$")
    mock: bool = False


def create_order_router(service: OrderService) -> APIRouter:
    router = APIRouter()

    @router.get("/orders", include_in_schema=False)
    async def orders_redirect():
        return RedirectResponse("/orders/", status_code=307)

    @router.get("/orders/", include_in_schema=False)
    async def index():
        return Response(
            (STATIC_DIR / "index.html").read_bytes(),
            media_type="text/html",
            headers={"Cache-Control": "no-store"},
        )

    @router.get("/orders/assets/{filename}", include_in_schema=False)
    async def asset(filename: str):
        allowed = {
            "app.js": "text/javascript",
            "styles.css": "text/css",
            "bottle.svg": "image/svg+xml",
            "box.svg": "image/svg+xml",
            "tube.svg": "image/svg+xml",
        }
        if filename not in allowed:
            return JSONResponse({"error_code": "NOT_FOUND"}, status_code=404)
        return Response(
            (STATIC_DIR / filename).read_bytes(),
            media_type=allowed[filename],
            headers={"Cache-Control": "no-store"},
        )

    @router.get("/orders/api/products")
    async def products():
        return {"products": service.product_list()}

    @router.get("/orders/api/current")
    async def current():
        order = service.current()
        if order is None:
            return Response(status_code=204)
        return order

    @router.post("/orders/api/orders", status_code=201)
    async def create_order(body: CreateOrderRequest):
        try:
            return service.submit(
                [
                    (item.sku_id, item.quantity, item.agv_row, item.agv_column)
                    for item in body.items
                ],
                body.basket_row,
                body.basket_column,
                mock=body.mock,
            )
        except OrderConflict as exc:
            return JSONResponse(
                {"error_code": "ORDER_BUSY", "message": str(exc)}, status_code=409
            )

    @router.get("/orders/api/orders/{order_id}")
    async def order_detail(order_id: str):
        try:
            return service.get(order_id)
        except OrderNotFound:
            return JSONResponse({"error_code": "ORDER_NOT_FOUND"}, status_code=404)

    @router.post("/orders/api/orders/{order_id}/retry")
    async def retry(order_id: str):
        try:
            return service.retry(order_id)
        except OrderNotFound:
            return JSONResponse({"error_code": "ORDER_NOT_FOUND"}, status_code=404)
        except OrderConflict as exc:
            return JSONResponse(
                {"error_code": "ORDER_CONFLICT", "message": str(exc)}, status_code=409
            )

    @router.post("/orders/api/orders/{order_id}/cancel")
    async def cancel(order_id: str):
        try:
            return service.cancel(order_id)
        except OrderNotFound:
            return JSONResponse({"error_code": "ORDER_NOT_FOUND"}, status_code=404)
        except OrderConflict as exc:
            return JSONResponse(
                {"error_code": "ORDER_CONFLICT", "message": str(exc)}, status_code=409
            )

    @router.get("/orders/api/orders/{order_id}/events")
    async def events(
        order_id: str,
        request: Request,
        last_event_id: Annotated[str | None, Header(alias="Last-Event-ID")] = None,
    ):
        try:
            service.get(order_id)
        except OrderNotFound:
            return JSONResponse({"error_code": "ORDER_NOT_FOUND"}, status_code=404)
        try:
            cursor = max(0, int(last_event_id or 0))
        except ValueError:
            cursor = 0

        async def stream():
            nonlocal cursor
            heartbeat = 0
            while not await request.is_disconnected():
                records = service.events_after(order_id, cursor)
                for record in records:
                    cursor = record["event_id"]
                    data = json.dumps(record, ensure_ascii=False, separators=(",", ":"))
                    yield f"id: {cursor}\nevent: order\ndata: {data}\n\n"
                heartbeat += 1
                if heartbeat >= 20:
                    yield ": keep-alive\n\n"
                    heartbeat = 0
                await asyncio.sleep(0.75)

        return StreamingResponse(
            stream(),
            media_type="text/event-stream",
            headers={"Cache-Control": "no-cache", "X-Accel-Buffering": "no"},
        )

    return router
