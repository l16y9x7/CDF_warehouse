from agent.capabilities.common import ActionResult, ActionStatus, Hand, HealthResult, HealthStatus
from agent.capabilities.http import HttpCapabilityClient

from .contract import (
    BasketPickRequest,
    PickActionResult,
    PickRequest,
    PlaceRequest,
    RotateActionResult,
    PushRequest,
    ReviewItemPickRequest,
)


class HttpManipulationCapability:
    def __init__(self, client: HttpCapabilityClient):
        self.client = client

    def health(self) -> HealthResult:
        return HealthResult(HealthStatus(self.client.get("/manipulation/health")["status"]))

    def pick(
        self, request: PickRequest, *, idempotency_key: str | None = None
    ) -> PickActionResult:
        body = {
            "task_type": request.task_type.value,
            "target_type": request.target_type.value,
            "sku_typ": request.sku_typ,
            "hand": request.hand.value,
            "level": request.level,
            "localization_result": dict(request.localization_result),
        }
        payload = self.client.post_action(
            "/manipulation/pick", body, idempotency_key=idempotency_key
        )
        if not isinstance(payload, dict):
            raise ValueError("manipulation pick response must be an object")
        completed_moves = payload.get("completed_moves")
        if completed_moves is not None and (
            not isinstance(completed_moves, int) or isinstance(completed_moves, bool)
        ):
            raise ValueError("manipulation pick completed_moves must be an integer")
        box_clearance = payload.get("box_clearance")
        if box_clearance is not None and not isinstance(box_clearance, dict):
            raise ValueError("manipulation pick box_clearance must be an object")
        return PickActionResult(
            ActionStatus(payload["status"]),
            box_clearance,
            completed_moves,
            dict(payload),
        )

    def pick_review_item(
        self, request: ReviewItemPickRequest, *, idempotency_key: str | None = None
    ) -> ActionResult:
        return self._action(
            "/manipulation/pick_review_item",
            {"hand": request.hand.value, **_pose_payload(request.pose)},
            idempotency_key=idempotency_key,
        )

    def place(self, request: PlaceRequest, *, idempotency_key: str | None = None) -> ActionResult:
        body = {
            "task_type": request.task_type.value,
            "target_type": request.target_type.value,
            "destination_type": request.destination_type.value,
            "hand": request.hand.value,
        }
        if request.destination_type.value == "basket":
            body["sku_typ"] = request.sku_typ
            body["localization_result"] = dict(request.localization_result or {})
        elif request.pose is not None:
            body.update(_pose_payload(request.pose))
        elif request.sku_id is not None:
            body["sku_id"] = request.sku_id
        return self._action("/manipulation/place", body, idempotency_key=idempotency_key)

    def rotate(
        self, hand: Hand, sku_typ: str, *, idempotency_key: str | None = None
    ) -> RotateActionResult:
        typ = str(sku_typ).strip()
        if not typ:
            raise ValueError("manipulation rotate requires sku_typ")
        payload = self.client.post_action(
            "/manipulation/rotate",
            {"hand": hand.value, "sku_typ": typ},
            idempotency_key=idempotency_key,
        )
        if not isinstance(payload, dict):
            raise ValueError("manipulation rotate response must be an object")
        camera = payload.get("camera")
        if not isinstance(camera, str) or not camera:
            raise ValueError("manipulation rotate response camera must be a non-empty string")
        return RotateActionResult(
            ActionStatus(payload["status"]), camera, tuple(payload.get("image_paths") or ())
        )

    def push(self, request: PushRequest, *, idempotency_key: str | None = None) -> ActionResult:
        body = {"hand": request.hand.value, **_flatten_basket_localization(request.localization_result)}
        return self._action("/manipulation/push", body, idempotency_key=idempotency_key)

    def pick_basket(
        self, request: BasketPickRequest, *, idempotency_key: str | None = None
    ) -> ActionResult:
        body = {
            "task_type": request.task_type.value,
            "target_type": request.target_type.value,
            "hand": request.hand.value,
            **_flatten_basket_localization(request.localization_result),
        }
        return self._action("/manipulation/pick", body, idempotency_key=idempotency_key)

    def _action(
        self, path: str, body: dict, *, idempotency_key: str | None = None
    ) -> ActionResult:
        return ActionResult(
            ActionStatus(
                self.client.post_action(path, body, idempotency_key=idempotency_key)["status"]
            )
        )


def _flatten_basket_localization(localization_result) -> dict:
    return {
        key: value
        for key, value in localization_result.items()
        if key not in {"target_type", "sku_typ", "class_name"}
    }


def _pose_payload(pose) -> dict:
    body = pose.as_payload()
    body.pop("corners_mm")
    return body
