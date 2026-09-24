#!/usr/bin/env python3
"""Standalone CLI runner for verifying response contracts on live HTTP endpoints.

Usage:
    python tests/live_response_contract.py --url http://127.0.0.1:25540/infer --request-json /path/to/request.json
"""
import argparse
import json
from pathlib import Path
import sys
import time
import urllib.request


def audit_response(url: str, request_json_path: Path, force_return_viz: bool = False) -> dict:
    if not request_json_path.is_file():
        raise FileNotFoundError(f"Request JSON file not found: {request_json_path}")

    with open(request_json_path, 'r', encoding='utf-8') as f:
        req_data = json.load(f)

    if not force_return_viz:
        req_data['return_visualizations'] = False
    else:
        req_data['return_visualizations'] = True

    body = json.dumps(req_data).encode('utf-8')
    http_req = urllib.request.Request(
        url,
        data=body,
        headers={'Content-Type': 'application/json'},
        method='POST'
    )

    t0 = time.perf_counter()
    with urllib.request.urlopen(http_req, timeout=120) as resp:
        wire_body = resp.read()
        elapsed = time.perf_counter() - t0
        status = resp.status
        content_type = resp.headers.get('Content-Type', '')

    size_bytes = len(wire_body)
    resp_json = json.loads(wire_body.decode('utf-8'))

    forbidden_keys = [
        'center_fit', 'left_fit', 'right_fit', 'analyses',
        'fm', 'raw', 'fit', 'ins', 'camera_points', '_mask'
    ]
    forbidden_found = [k for k in forbidden_keys if k in resp_json]

    artifacts = resp_json.get('artifacts', {})
    base64_keys = [k for k in artifacts if str(k).endswith('_base64')]

    sku_typ = resp_json.get('sku_typ')
    target_type = resp_json.get('target_type')

    audit = {
        'url': url,
        'request_json': str(request_json_path),
        'http_status': status,
        'content_type': content_type,
        'elapsed_seconds': round(elapsed, 3),
        'wire_bytes': size_bytes,
        'wire_kb': round(size_bytes / 1024, 2),
        'target_type': target_type,
        'sku_typ': sku_typ,
        'ok': resp_json.get('ok'),
        'request_id': resp_json.get('request_id'),
        'no_forbidden_keys': len(forbidden_found) == 0,
        'forbidden_keys_found': forbidden_found,
        'return_visualizations': force_return_viz,
    }

    if not force_return_viz:
        audit['no_unauthorized_base64'] = len(base64_keys) == 0
        audit['base64_keys_found'] = base64_keys
    else:
        audit['authorized_base64_present'] = len(base64_keys) > 0
        audit['base64_keys_found'] = base64_keys

    # Check category fields
    if sku_typ == 'tube':
        audit['edge_valid'] = resp_json.get('edge_valid')
        audit['point_valid'] = resp_json.get('point_valid')
        audit['point_semantics'] = resp_json.get('point_semantics')
        audit['top_edge_center_camera_mm'] = resp_json.get('top_edge_center_camera_mm')
        audit['top_edge_endpoints_camera_mm'] = resp_json.get('top_edge_endpoints_camera_mm')
    elif sku_typ == 'bottle':
        audit['axis_fit_valid'] = resp_json.get('axis_fit_valid')
        audit['reference_point_valid'] = resp_json.get('reference_point_valid')
        audit['reference_point_camera_mm'] = resp_json.get('reference_point_camera_mm')
    elif sku_typ == 'box':
        audit['top_plane_valid'] = resp_json.get('top_plane_valid')
        audit['top_point_valid'] = resp_json.get('top_point_valid')
        audit['top_point_camera_mm'] = resp_json.get('top_point_camera_mm')
    elif target_type == 'basket':
        audit['pose_valid'] = resp_json.get('pose_valid')
        audit['model_center_camera_mm'] = resp_json.get('model_center_camera_mm')

    max_allowed_bytes = (5 * 1024 * 1024) if force_return_viz else (50 * 1024)
    # Overall HTTP response contract pass/fail (protocol, size, security, headers)
    contract_passed = (
        status == 200
        and 'charset=utf-8' in content_type.lower()
        and size_bytes <= max_allowed_bytes
        and len(forbidden_found) == 0
        and (len(base64_keys) == 0 if not force_return_viz else True)
    )
    audit['contract_passed'] = contract_passed

    # Algorithm quality gate / geometric localization pass/fail
    if sku_typ == 'tube':
        audit['geometry_passed'] = bool(audit.get('edge_valid') and audit.get('point_valid'))
    elif sku_typ == 'bottle':
        audit['geometry_passed'] = bool(audit.get('axis_fit_valid') and audit.get('reference_point_valid'))
    elif sku_typ == 'box':
        audit['geometry_passed'] = bool(audit.get('top_plane_valid') and audit.get('top_point_valid'))
    elif target_type == 'basket':
        audit['geometry_passed'] = bool(audit.get('pose_valid'))
    else:
        audit['geometry_passed'] = bool(resp_json.get('ok', False))

    return audit


def main():
    parser = argparse.ArgumentParser(description="Audit live HTTP response against ROBOT_API_HANDOFF contract.")
    parser.add_argument('--url', default='http://127.0.0.1:25540/infer', help="Target infer URL")
    parser.add_argument('--request-json', required=True, type=Path, help="Path to request JSON file")
    parser.add_argument('--return-visualizations', action='store_true', help="Embed base64 visualizations")
    args = parser.parse_args()

    try:
        result = audit_response(args.url, args.request_json, force_return_viz=args.return_visualizations)
        print(json.dumps(result, indent=2, ensure_ascii=False))
        if not result.get('contract_passed'):
            sys.exit(1)
    except Exception as exc:
        print(json.dumps({'error': str(exc), 'contract_passed': False}, indent=2), file=sys.stderr)
        sys.exit(2)


if __name__ == '__main__':
    main()
