from collections.abc import Callable
from typing import Any

from fastapi import FastAPI, Header, Query
from fastapi.responses import JSONResponse

from agent.capabilities.estimation.contract import (
    BASE_FRAME,
    CAMERA_FRAME,
    FRONT_RULE_KEYS,
    SIDES,
    T_UNITS,
    as_matrix,
    validate_extrinsics,
    validate_intrinsics,
)
from agent.capabilities.estimation.mock import mock_infer_response
from agent.capabilities.mocks.delay import pause_mock_processing_async
from agent.capabilities.navigation import NAVIGATION_TARGETS
from agent.capabilities.perception.mock import default_locate_status, locate_found_payload


def _install_process_delay(app: FastAPI) -> None:
    @app.middleware("http")
    async def _process_delay(request, call_next):
        await pause_mock_processing_async()
        return await call_next(request)


def _health_app(title: str, path: str) -> FastAPI:
    app = FastAPI(title=title)
    _install_process_delay(app)

    async def health() -> dict[str, str]:
        return {"status": "READY"}

    app.get(path)(health)
    return app


def _invalid_fields(
    body: dict[str, Any], required: set[str], allowed: set[str] | None = None
) -> JSONResponse | None:
    if not required <= body.keys() or not body.keys() <= (allowed or required):
        return JSONResponse(
            {"error_code": "INVALID_INPUT", "message": "request fields do not match contract"},
            status_code=422,
        )
    return None


def _missing_idempotency_key(idempotency_key: str | None) -> JSONResponse | None:
    if not (idempotency_key or "").strip():
        return JSONResponse(
            {"error_code": "INVALID_INPUT", "message": "Idempotency-Key is required"},
            status_code=400,
        )
    return None


def create_navigation_app() -> FastAPI:
    app = _health_app("Mock Navigation Module", "/navigation/health")

    @app.post("/navigation/navigate")
    async def navigate(
        body: dict[str, Any],
        idempotency_key: str | None = Header(default=None, alias="Idempotency-Key"),
    ):
        missing = _missing_idempotency_key(idempotency_key)
        if missing:
            return missing
        invalid = _invalid_fields(body, {"nav_id"})
        if invalid:
            return invalid
        if body["nav_id"] not in NAVIGATION_TARGETS:
            return JSONResponse({"error_code": "INVALID_NAV_ID"}, status_code=422)
        return {"status": "SUCCEEDED"}

    return app


def create_pose_app() -> FastAPI:
    app = _health_app("Mock Body Pose Module", "/pose/health")

    @app.post("/pose/prepare")
    async def prepare(
        body: dict[str, Any],
        idempotency_key: str | None = Header(default=None, alias="Idempotency-Key"),
    ):
        return _missing_idempotency_key(idempotency_key) or _invalid_fields(
            body, {"pose_type"}, {"pose_type", "level"}
        ) or {"status": "SUCCEEDED"}

    @app.get("/pose/camera_transform")
    async def camera_transform(camera: str = Query("head")):
        if camera != "head":
            return JSONResponse({"error_code": "CAMERA_NOT_SUPPORTED"}, status_code=400)
        return {
            "T_chassis_camera": [
                [1.0, 0.0, 0.0, 0.113],
                [0.0, 1.0, 0.0, 0.018],
                [0.0, 0.0, 1.0, 1.451],
                [0.0, 0.0, 0.0, 1.0],
            ],
            "t_unit": "m",
            "camera": "head",
            "camera_frame": "head_camera_color_optical_frame",
            "base_frame": "chassis_link",
            "sampled_at_unix_s": 0.0,
            "upper_body_joints_deg": [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
        }

    return app


def create_perception_app() -> FastAPI:
    app = _health_app("Mock Perception Module", "/perception/health")
    app.state.locate_statuses = []
    app.state.barcode_content = None

    async def locate_item(body: dict[str, Any]):
        invalid = _invalid_fields(body, {"image_path"})
        if invalid:
            return invalid
        if app.state.locate_statuses:
            status = app.state.locate_statuses.pop(0)
        else:
            status = default_locate_status(str(body["image_path"])).value
        if status == "NOT_FOUND":
            return {"status": "NOT_FOUND"}
        return locate_found_payload()

    app.post("/perception/basket/locate_item")(locate_item)

    @app.post("/perception/recognize_sku_barcode")
    async def barcode(body: dict[str, Any]):
        invalid = _invalid_fields(
            body, {"image_base64"}, {"image_base64", "sku_id", "name"}
        )
        if invalid:
            return invalid
        if ("sku_id" in body) != ("name" in body):
            return JSONResponse({"error_code": "INVALID_INPUT"}, status_code=422)
        content = (
            app.state.barcode_content
            if app.state.barcode_content is not None
            else str(body.get("sku_id") or "").strip()
        )
        if not str(content).strip():
            return {"status": "NOT_FOUND"}
        return {"barcode_content": str(content)}

    return app


def create_estimation_app() -> FastAPI:
    app = FastAPI(title="Mock Estimation Module")
    _install_process_delay(app)

    @app.get("/health")
    async def health() -> dict[str, str]:
        return {"status": "READY"}

    @app.post("/infer")
    async def infer(body: dict[str, Any]):
        invalid = _invalid_infer(body)
        return invalid or mock_infer_response(
            body.get("sku_typ"),
            target_type=str(body.get("target_type") or "sku"),
        )

    return app


INFER_FRAME_REQUIRED = frozenset(
    {
        "target_type",
        "rgb_base64",
        "depth_npy_base64",
        "depth_unit",
        "K",
        "T_chassis_camera",
        "T_unit",
        "camera_frame",
        "base_frame",
    }
)
INFER_REQUIRED = INFER_FRAME_REQUIRED | {"sku_typ", "side"}
INFER_ALLOWED = INFER_REQUIRED | {"front_rule"}
INFER_BASKET_REQUIRED = INFER_FRAME_REQUIRED
INFER_BASKET_ALLOWED = INFER_BASKET_REQUIRED


def _invalid_infer(body: dict[str, Any]) -> JSONResponse | None:
    def reject(message: str) -> JSONResponse:
        return JSONResponse(
            {"error_code": "INVALID_INPUT", "message": message}, status_code=422
        )

    target = body.get("target_type")
    if target == "basket":
        required, allowed = INFER_BASKET_REQUIRED, INFER_BASKET_ALLOWED
    elif target == "sku":
        required, allowed = INFER_REQUIRED, INFER_ALLOWED
    else:
        return reject("target_type must be sku or basket")
    invalid = _invalid_fields(body, required, allowed)
    if invalid:
        return invalid
    for name in ("rgb_base64", "depth_npy_base64"):
        value = str(body[name]).strip()
        if value.startswith("<") and value.endswith(">"):
            return reject(f"{name} must not be a literal placeholder")
    if body["depth_unit"] != "mm":
        return reject("depth_unit must be mm")
    if body["T_unit"] not in T_UNITS:
        return reject("T_unit must be m or mm")
    if body["camera_frame"] != CAMERA_FRAME or body["base_frame"] != BASE_FRAME:
        return reject("camera_frame/base_frame do not match the contract")
    if target == "basket":
        return None
    if not str(body.get("sku_typ") or "").strip():
        return reject("sku_typ is required")
    if body["side"] not in SIDES:
        return reject("side must be LEFT or RIGHT")
    if "front_rule" in body:
        rule = body["front_rule"]
        if not isinstance(rule, dict) or set(rule) != FRONT_RULE_KEYS:
            return reject("front_rule requires front_axis_chassis, front_origin_chassis and front_band_mm")
    return None


def create_manipulation_app() -> FastAPI:
    app = _health_app("Mock Manipulation Module", "/manipulation/health")
    pose_fields = {"pose", "hand", "frame", "pose_unit", "rotation_order"}

    def action(required: set[str], allowed: set[str] | None = None) -> Callable:
        async def execute(
            body: dict[str, Any],
            idempotency_key: str | None = Header(default=None, alias="Idempotency-Key"),
        ):
            missing = _missing_idempotency_key(idempotency_key)
            if missing:
                return missing
            invalid = _invalid_fields(body, required, allowed)
            if invalid:
                return invalid
            if "hand" in body and body["hand"] not in {"LEFT", "RIGHT"}:
                return JSONResponse({"error_code": "INVALID_INPUT"}, status_code=422)
            return {"status": "SUCCEEDED"}

        return execute

    @app.post("/manipulation/pick")
    async def pick(
        body: dict[str, Any],
        idempotency_key: str | None = Header(default=None, alias="Idempotency-Key"),
    ):
        missing = _missing_idempotency_key(idempotency_key)
        if missing:
            return missing
        if body.get("target_type") == "basket":
            required = {
                "task_type",
                "target_type",
                "hand",
                "ok",
                "pose_valid",
                "point_semantics",
                "model_center_camera_mm",
                "pose_4x4",
            }
            if (
                not required <= body.keys()
                or body["task_type"] != "REVIEW"
                or body["hand"] not in {"LEFT", "RIGHT"}
                or any(key in body for key in ("sku_typ", "level", "sku_id", "pose", "localization_result"))
            ):
                return JSONResponse({"error_code": "INVALID_INPUT"}, status_code=422)
            return {"status": "SUCCEEDED"}
        required = {
            "task_type",
            "target_type",
            "sku_typ",
            "hand",
            "level",
            "localization_result",
        }
        invalid = _invalid_fields(body, required)
        if invalid:
            return invalid
        if (
            body["task_type"] != "SORTING"
            or body["target_type"] != "sku"
            or not str(body.get("sku_typ") or "").strip()
            or body["hand"] not in {"LEFT", "RIGHT"}
            or body["level"] not in {"L1", "L2", "L3", "L4", "L5"}
            or not isinstance(body["localization_result"], dict)
        ):
            return JSONResponse({"error_code": "INVALID_INPUT"}, status_code=422)
        return {
            "status": "SUCCEEDED",
            "box_clearance": {"frame": "trunk_controller_ref", "unit": "mm"},
            "completed_moves": 6,
        }

    app.post("/manipulation/pick_review_item")(action(pose_fields))

    @app.post("/manipulation/rotate")
    async def rotate(
        body: dict[str, Any],
        idempotency_key: str | None = Header(default=None, alias="Idempotency-Key"),
    ):
        missing = _missing_idempotency_key(idempotency_key)
        if missing:
            return missing
        invalid = _invalid_fields(body, {"hand", "sku_typ"})
        if invalid:
            return invalid
        if body["hand"] not in {"LEFT", "RIGHT"} or not str(body.get("sku_typ") or "").strip():
            return JSONResponse({"error_code": "INVALID_INPUT"}, status_code=422)
        return {
            "status": "SUCCEEDED",
            "camera": "left_wrist",
            "image_paths": [
                f"/shared/frames/mock-barcode-{index}/rgb.jpg" for index in range(1, 6)
            ],
        }

    @app.post("/manipulation/place")
    async def place(
        body: dict[str, Any],
        idempotency_key: str | None = Header(default=None, alias="Idempotency-Key"),
    ):
        missing = _missing_idempotency_key(idempotency_key)
        if missing:
            return missing
        base = {"task_type", "target_type", "destination_type", "hand"}
        if body.get("destination_type") == "table":
            return _invalid_fields(body, base) or {"status": "SUCCEEDED"}
        required = base | {"sku_typ", "localization_result"}
        if (
            not required <= body.keys()
            or any(key in body for key in ("sku_id", "pose"))
            or not str(body.get("sku_typ") or "").strip()
            or not isinstance(body.get("localization_result"), dict)
        ):
            return JSONResponse({"error_code": "INVALID_INPUT"}, status_code=422)
        return {"status": "SUCCEEDED"}

    @app.post("/manipulation/push")
    async def push(
        body: dict[str, Any],
        idempotency_key: str | None = Header(default=None, alias="Idempotency-Key"),
    ):
        missing = _missing_idempotency_key(idempotency_key)
        if missing:
            return missing
        required = {
            "hand",
            "ok",
            "pose_valid",
            "point_semantics",
            "model_center_camera_mm",
            "pose_4x4",
        }
        if (
            not required <= body.keys()
            or body["hand"] not in {"LEFT", "RIGHT"}
            or any(key in body for key in ("sku_id", "pose", "localization_result"))
        ):
            return JSONResponse({"error_code": "INVALID_INPUT"}, status_code=422)
        return {"status": "SUCCEEDED"}

    return app


def create_vla_app() -> FastAPI:
    app = _health_app("Mock VLA Module", "/vla/health")
    app.state.result_status = "PICKED"

    @app.post("/vla/pick_review_item")
    async def pick(
        body: dict[str, Any],
        idempotency_key: str | None = Header(default=None, alias="Idempotency-Key"),
    ):
        missing = _missing_idempotency_key(idempotency_key)
        if missing:
            return missing
        invalid = _invalid_fields(body, {"hand"})
        return invalid or {"status": app.state.result_status}

    return app


def create_hand_app() -> FastAPI:
    app = _health_app("Mock Hand Module", "/hand/health")

    @app.post("/hand/pick")
    async def pick(
        body: dict[str, Any],
        idempotency_key: str | None = Header(default=None, alias="Idempotency-Key"),
    ):
        missing = _missing_idempotency_key(idempotency_key)
        if missing:
            return missing
        return _invalid_fields(body, {"task_type", "target_type", "sku_id", "hand"}) or {
            "status": "SUCCEEDED"
        }

    return app


def create_camera_app() -> FastAPI:
    app = FastAPI(title="Mock Camera Service")
    _install_process_delay(app)
    capture = 0

    def valid(camera: str) -> bool:
        return camera in {"head", "left_wrist", "right_wrist"}

    def failed(error_code: str, message: str, camera: str, status_code: int = 422):
        return JSONResponse(
            {"ok": False, "error_code": error_code, "message": message, "camera": camera},
            status_code=status_code,
        )

    @app.get("/camera/health")
    async def camera_health():
        return {"status": "READY", "ok": True, "all_ready": True}

    @app.get("/camera/rgbd")
    async def rgbd(camera: str = Query(...)):
        nonlocal capture
        if camera != "head":
            return failed("CAMERA_NOT_SUPPORTED", "RGB-D is head-only", camera)
        capture += 1
        capture_id = f"capture-{capture}"
        base = f"/shared/frames/{capture_id}"
        return {
            "ok": True,
            "camera": "head",
            "capture_id": capture_id,
            "captured_at": "2026-09-21T12:00:00+08:00",
            "rgb": f"{base}/rgb.jpg",
            "depth": f"{base}/depth_mm.npy",
            "t_unit": "mm",
            "same_shot": True,
            "timestamps": {
                "color_s": 1.0,
                "depth_s": 1.0,
                "pairing": "exact_ros_stamp",
            },
            "color_intrinsics": {
                "width": 1280,
                "height": 720,
                "camera_matrix": [
                    [612.772339587192, 0.0, 641.544745101529],
                    [0.0, 611.998287842651, 358.604256964916],
                    [0.0, 0.0, 1.0],
                ],
                "distortion_model": "plumb_bob",
                "distortion_coefficients": [],
            },
        }

    @app.get("/camera/capture")
    async def capture_frame(
        camera: str = Query(...),
        streams: str = Query(default="color"),
        format: str = Query(default="raw"),
    ):
        nonlocal capture
        if not valid(camera):
            return failed("CAMERA_NOT_FOUND", "camera not found", camera, 404)
        requested = streams.split(",")
        if (
            not requested
            or any(item not in {"color", "depth"} for item in requested)
            or len(requested) != len(set(requested))
        ):
            return failed("INVALID_STREAMS", "invalid camera streams", camera)
        requested_set = set(requested)
        if requested_set != {"depth"}:
            format = "raw"
        elif format not in {"raw", "preview"}:
            return failed("INVALID_FORMAT", "invalid depth format", camera)
        capture += 1
        capture_id = f"capture-{capture}"
        base = f"/shared/frames/{capture_id}"
        color = (
            {"path": f"{base}/rgb.jpg", "format": "jpeg", "width": 1280, "height": 720}
            if "color" in requested_set
            else None
        )
        depth = (
            {
                "path": f"{base}/{'depth_mm.npy' if format == 'raw' else 'depth.jpg'}",
                "format": format,
                "width": 1280,
                "height": 720,
                "aligned": True,
            }
            if "depth" in requested_set
            else None
        )
        return {
            "ok": True,
            "capture_id": capture_id,
            "camera": camera,
            "same_shot": len(requested_set) == 2,
            "color": color,
            "depth": depth,
        }

    return app


navigation_app, pose_app, perception_app = (
    create_navigation_app(),
    create_pose_app(),
    create_perception_app(),
)
estimation_app, manipulation_app = create_estimation_app(), create_manipulation_app()
vla_app, hand_app, camera_app = create_vla_app(), create_hand_app(), create_camera_app()
