#!/usr/bin/env python3
"""HTTP localization service entry point for SKU spatial localization.

Coordinates Box Selection, SAM3 segmentation, front-row target selection,
and category pose estimation for bottles, boxes, tubes, and baskets.
Dispatches requests to target_pipeline.py and exposes /health and /infer.
"""
from datetime import datetime
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import json
import os
from pathlib import Path
import sys
import threading
import time
from typing import Any
import uuid

os.environ.setdefault('MPLBACKEND', 'Agg')

_HERE = Path(__file__).resolve().parent
for _sub in ('core', 'perception', 'pipeline', 'algorithms'):
    _p = str(_HERE / _sub)
    if _p not in sys.path:
        sys.path.insert(0, _p)

from config_loader import (
    ARTIFACT_ROOT,
    ASYNC_3D_RENDER,
    BASKET_DEFAULT_PROMPT,
    BASKET_DEFAULT_THRESHOLD,
    BASKET_FP_REGISTERED_CAD,
    BASKET_FP_URL,
    BASKET_MESH_PATH,
    BASKET_MESH_SCALE,
    CLASS_CONFIG,
    DEFAULT_HOST,
    DEFAULT_PORT,
    ENABLE_3D_RENDER,
    FRONT_RULE_DEFAULT,
    MAX_BODY_BYTES,
    SAM3_BACKEND,
    SAM3_MASK_THRESHOLD,
    SAM3_URL,
    SAVE_REQUEST_INPUTS,
)
from geometry_cache import GeometryCache
from module_loader import get_deploy_modules
from request_codec import jsonable
from response_schema import build_robot_response, persist_request_inputs, save_json
from target_pipeline import (
    apply_front_rule,
    box_overlay,
    box_surface_cfg,
    dispatch_3d_render,
    execute_pipeline,
    fit_selected_box_front_panel,
    parse_box_selection,
    process_basket,
    process_bottle,
    process_box,
    process_sku,
    process_tube,
    select_front,
)

# Initialize loaded algorithm modules and legacy handles
MODULES = get_deploy_modules()
FIT = MODULES.fit_bottle_axis
BOX = MODULES.fit_estee_box_top_surface
BOXSEL = MODULES.box_selection
TUBE = MODULES.fit_tube_top_edge
FRONT = MODULES.fit_front_panel_plane
BGC = MODULES.box_geometric_center
BOX_PIPE = MODULES.box_pipeline

_STAGES = threading.local()


def mark(name: str, t0: float) -> float:
    """Test instrumentation: record stage duration in ms into the per-request stage map.

    Timing only; never changes request logic, values, or quality gates.
    """
    d = getattr(_STAGES, 'd', None)
    if d is None:
        d = _STAGES.d = {}
    d[name] = round((time.perf_counter() - t0) * 1000, 3)
    return time.perf_counter()


class Handler(BaseHTTPRequestHandler):
    """HTTP request handler for /health and /infer endpoints."""

    def _send(self, code: int, obj: Any) -> None:
        body = json.dumps(obj, ensure_ascii=False, default=jsonable).encode('utf-8')
        self.send_response(code)
        self.send_header('Content-Type', 'application/json; charset=utf-8')
        self.send_header('Content-Length', str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def do_GET(self) -> None:
        if self.path == '/health':
            self._send(200, {
                'ok': True,
                'service': 'spatial-localization-diagnostic',
                'basket': {
                    'pipeline': 'SAM3 central white plastic basket -> local FoundationPose 25550',
                    'prompt': BASKET_DEFAULT_PROMPT,
                    'threshold': BASKET_DEFAULT_THRESHOLD,
                    'cad_path': str(BASKET_MESH_PATH),
                    'mesh_scale': BASKET_MESH_SCALE,
                    'foundationpose_url': BASKET_FP_URL,
                    'cad_transport': 'registered_default' if BASKET_FP_REGISTERED_CAD else 'per_request_upload',
                },
                'sku': {
                    'supported_types': sorted(CLASS_CONFIG),
                    'pipeline': 'box_selection_then_product_localization',
                    'sam3_backend': SAM3_BACKEND,
                    'sam3_url': SAM3_URL,
                    'mask_threshold': SAM3_MASK_THRESHOLD,
                    'default_front_rule': FRONT_RULE_DEFAULT,
                },
                'render': {
                    'enable_3d': ENABLE_3D_RENDER,
                    'async_3d': ASYNC_3D_RENDER,
                },
                'flags': {
                    'weights_loaded_here': False,
                    'robot_actions': False,
                },
            })
        else:
            self._send(404, {'ok': False, 'error': 'unknown route'})

    def do_POST(self) -> None:
        if self.path != '/infer':
            self._send(404, {'ok': False, 'error': 'unknown route'})
            return

        root = None
        rid = datetime.now().strftime('%Y%m%d_%H%M%S') + '_' + uuid.uuid4().hex[:8]
        req = None
        try:
            service_started = time.perf_counter()
            _STAGES.d = {}
            length = int(self.headers.get('Content-Length', '0'))
            if length <= 0 or length > MAX_BODY_BYTES:
                raise ValueError('invalid Content-Length')

            t_body = time.perf_counter()
            raw_body = self.rfile.read(length)
            try:
                req = json.loads(raw_body)
            except Exception:
                root = ARTIFACT_ROOT / 'rejected' / rid
                root.mkdir(parents=True, exist_ok=False)
                (root / 'request_body.bin').write_bytes(raw_body)
                raise

            mark('body_read_parse', t_body)
            bucket = 'basket' if req.get('target_type') == 'basket' else 'sku'
            root = ARTIFACT_ROOT / bucket / rid
            root.mkdir(parents=True, exist_ok=False)
            persist_request_inputs(root, raw_body, req)

            t_compute = time.perf_counter()
            result = execute_pipeline(req, rid, root, mark=mark)
            compute_ms = round((time.perf_counter() - t_compute) * 1000, 3)

            _diag = result.setdefault('diagnostics', {})
            _diag['service_compute_wall_ms'] = compute_ms
            _diag['service_total_wall_ms'] = round((time.perf_counter() - service_started) * 1000, 3)
            _diag['request_artifact_bucket'] = bucket
            _diag['request_inputs_saved'] = bool(SAVE_REQUEST_INPUTS)
            _diag['stage_timing_ms'] = dict(getattr(_STAGES, 'd', {}))

            save_json(root / 'response.json', result)

            robot_resp = build_robot_response(result, req)
            self._send(200, robot_resp)

        except Exception as exc:
            error = {'ok': False, 'request_id': rid, 'error': str(exc), 'error_type': type(exc).__name__}
            if root is not None:
                save_json(root / 'error.json', error)
            robot_error = build_robot_response(error, req if isinstance(req, dict) else None)
            self._send(400, robot_error)

    def log_message(self, fmt, *args):
        return


def main():
    """Service CLI entry point."""
    import argparse

    parser = argparse.ArgumentParser(description="SKU spatial localization service")
    parser.add_argument('--host', default=DEFAULT_HOST, help="Listen host")
    parser.add_argument('--port', type=int, default=DEFAULT_PORT, help="Listen port")
    args = parser.parse_args()

    ARTIFACT_ROOT.mkdir(parents=True, exist_ok=True)
    print(
        f"LISTEN {args.host}:{args.port}; "
        f"SAM3_BACKEND={SAM3_BACKEND}; SAM3_URL={SAM3_URL}; "
        f"classes={','.join(CLASS_CONFIG)}",
        flush=True,
    )
    server = ThreadingHTTPServer((args.host, args.port), Handler)
    server.serve_forever()


if __name__ == '__main__':
    main()
