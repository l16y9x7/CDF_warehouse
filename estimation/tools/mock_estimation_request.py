#!/usr/bin/env python3
"""Replay local RGB-D files against /infer; requires Python 3 and numpy.

Default camera parameters are a timing-only fixture from request
20261001_084306_f205f149, NOT the calibration/pose of the supplied image.
Use --metadata with that frame's real request parameters for pose validation.
"""
import argparse
import base64
from datetime import datetime
import http.client
import io
import json
from pathlib import Path
import sys
import time
from urllib.parse import urlsplit
import uuid

import numpy as np


DEFAULT_FRAME = Path('/shared/frames/capture-5e39297885c7')
SAMPLE_METADATA = {
    'target_type': 'sku', 'sku_typ': 'bottle', 'side': 'RIGHT',
    'depth_unit': 'mm', 'T_unit': 'm',
    'camera_frame': 'head_camera_color_optical_frame',
    'base_frame': 'chassis_link',
    'K': [[610.3707275390625, 0, 642.1707153320312],
          [0, 610.3666381835938, 360.5491943359375], [0, 0, 1]],
    'T_chassis_camera': [
        [0.00598634865644126, -0.546860580467513, 0.8372022868816401, 0.16804379857500734],
        [-0.9999413698094504, -0.010828250872561729, 0.00007698750857115586, 0.024036391255695046],
        [0.00902333495934487, -0.8371536624267816, -0.5468933396121017, 0.9885605610615616],
        [0, 0, 0, 1],
    ],
}


def log(message):
    print('{} {}'.format(datetime.now().isoformat(timespec='milliseconds'), message), flush=True)


def write_json(path, value):
    path.write_text(json.dumps(value, ensure_ascii=False, indent=2, allow_nan=False), encoding='utf-8')


def timed(report, name, operation):
    started = time.perf_counter()
    try:
        return operation()
    finally:
        elapsed = (time.perf_counter() - started) * 1000
        report['client_timing_ms'][name] = round(elapsed, 3)
        log('{}: {:.3f} ms'.format(name, elapsed))


def load_metadata(args):
    if args.metadata:
        req = json.loads(args.metadata.read_text(encoding='utf-8-sig'))
        if not isinstance(req, dict):
            raise ValueError('--metadata must contain a JSON object')
    else:
        req = json.loads(json.dumps(SAMPLE_METADATA))
        log('TIMING FIXTURE: using old K/T, bottle RIGHT; pose is not validated for this frame.')
    # Persisted metadata may include server-added fields rejected by /infer.
    for key in list(req):
        if key in ('rgb_base64', 'depth_npy_base64', 'class_name', 'sku_id') or key.startswith('_'):
            del req[key]
    if args.target_type:
        req['target_type'] = args.target_type
    if args.sku_typ:
        req['sku_typ'] = args.sku_typ
    if args.side:
        req['side'] = args.side
        if isinstance(req.get('box_selection'), dict):
            req['box_selection']['target_box'] = 1 if args.side == 'LEFT' else 2
    if req.get('target_type') == 'basket':
        req.pop('sku_typ', None)
    elif req.get('target_type') == 'sku':
        if req.get('sku_typ') not in ('bottle', 'box', 'tube'):
            raise ValueError('SKU metadata requires sku_typ=bottle/box/tube')
        if req.get('side') not in ('LEFT', 'RIGHT') and req.get('box_selection', {}).get('target_box') not in (1, 2):
            raise ValueError('SKU metadata requires side=LEFT/RIGHT or box_selection.target_box')
    else:
        raise ValueError('target_type must be sku or basket')
    for key in ('K', 'T_chassis_camera', 'T_unit', 'camera_frame', 'base_frame', 'depth_unit'):
        if key not in req:
            raise ValueError('Missing metadata field: ' + key)
    if req['depth_unit'] != 'mm':
        raise ValueError('This script expects depth values in mm; it does not rescale units.')
    return req


def prepare_body(args, report, output):
    req = load_metadata(args)
    rgb_bytes = timed(report, 'rgb_file_read', args.rgb.read_bytes)
    depth_bytes = timed(report, 'depth_file_read', args.depth.read_bytes)
    depth = timed(report, 'depth_npy_load', lambda: np.load(io.BytesIO(depth_bytes), allow_pickle=False))
    if not isinstance(depth, np.ndarray) or depth.ndim != 2 or depth.dtype.kind not in 'uif':
        raise ValueError('Depth must be a 2-D real numeric NPY array')
    report['inputs'] = {
        'rgb': str(args.rgb), 'rgb_bytes': len(rgb_bytes),
        'depth': str(args.depth), 'depth_file_bytes': len(depth_bytes),
        'depth_shape': list(depth.shape), 'depth_source_dtype': str(depth.dtype),
        'depth_wire_dtype': 'float32' if args.depth_dtype == 'float32' else str(depth.dtype),
    }
    if args.depth_dtype == 'float32':
        def encode_depth():
            buffer = io.BytesIO()
            np.save(buffer, depth.astype(np.float32, copy=False), allow_pickle=False)
            return buffer.getvalue()
        depth_bytes = timed(report, 'depth_float32_npy_encode', encode_depth)
    elif depth.dtype.kind != 'f':
        raise ValueError('/infer requires floating depth; use the default --depth-dtype float32')
    report['inputs']['depth_wire_bytes'] = len(depth_bytes)
    log('Depth: {} {} -> {} ({} -> {} bytes); values remain mm'.format(
        depth.shape, depth.dtype, report['inputs']['depth_wire_dtype'],
        report['inputs']['depth_file_bytes'], len(depth_bytes)))
    req['rgb_base64'] = timed(report, 'rgb_base64_encode', lambda: base64.b64encode(rgb_bytes).decode('ascii'))
    req['depth_npy_base64'] = timed(report, 'depth_base64_encode', lambda: base64.b64encode(depth_bytes).decode('ascii'))
    body = timed(report, 'json_serialize', lambda: json.dumps(req, ensure_ascii=False, allow_nan=False).encode('utf-8'))
    if len(body) > 33554432:
        raise ValueError('Request exceeds the service default limit of 32 MiB')
    report['request_bytes'] = len(body)
    report['target'] = {k: req.get(k) for k in ('target_type', 'sku_typ', 'side')}
    write_json(output / 'request_metadata.json', {k: v for k, v in req.items() if not k.endswith('_base64')})
    if args.save_request or args.prepare_only:
        (output / 'request.json').write_bytes(body)
    log('Request: {:,} bytes ({:.3f} MiB)'.format(len(body), len(body) / 1048576))
    return body


def parse_http_url(value):
    parsed = urlsplit(value)
    if parsed.scheme != 'http' or not parsed.hostname or parsed.username or parsed.password or parsed.fragment:
        raise ValueError('Use an http://host:port URL without credentials or fragment')
    return parsed


def post_body(args, body, report, output):
    target = parse_http_url(args.url)
    endpoint = parse_http_url(args.proxy) if args.proxy else target
    path = (target.path or '/') + ('?' + target.query if target.query else '')
    conn = http.client.HTTPConnection(endpoint.hostname, endpoint.port or 80, timeout=args.timeout)
    http_started = time.perf_counter()
    http_finished = None
    try:
        log('Connecting to {}:{} ({})'.format(endpoint.hostname, endpoint.port or 80,
                                             'explicit HTTP proxy' if args.proxy else 'DIRECT; proxy environment ignored'))
        timed(report, 'connect', conn.connect)
        report['local_address'] = list(conn.sock.getsockname())
        report['peer_address'] = list(conn.sock.getpeername())
        log('TCP local={} peer={}'.format(report['local_address'], report['peer_address']))

        def send_headers():
            conn.putrequest('POST', args.url if args.proxy else path, skip_host=True, skip_accept_encoding=True)
            conn.putheader('Host', target.netloc)
            conn.putheader('Content-Type', 'application/json')
            conn.putheader('Content-Length', str(len(body)))
            conn.putheader('Connection', 'close')
            conn.endheaders()
        timed(report, 'headers_send', send_headers)

        chunks = []
        report['socket_write_chunks'] = chunks
        report['body_bytes_written'] = 0
        body_view = memoryview(body)
        send_started = time.perf_counter()
        last_log = send_started
        log('Writing body to socket; completion does NOT mean all bytes have reached 107.')

        def send_body():
            nonlocal last_log
            chunk_bytes = args.chunk_kib * 1024
            for offset in range(0, len(body), chunk_bytes):
                part = body_view[offset:offset + chunk_bytes]
                started = time.perf_counter()
                conn.send(part)
                now = time.perf_counter()
                sent = offset + len(part)
                report['body_bytes_written'] = sent
                chunks.append({'end_byte': sent, 'write_ms': round((now - started) * 1000, 3),
                               'elapsed_ms': round((now - send_started) * 1000, 3)})
                if now - last_log >= 1 or sent == len(body):
                    log('Socket accepted {}/{} bytes ({:.1f}%), elapsed {:.3f} s'.format(
                        sent, len(body), sent * 100 / len(body), now - send_started))
                    last_log = now
        timed(report, 'body_socket_write', send_body)
        log('Waiting for response headers (remaining transfer + server work may both be included)...')
        response = timed(report, 'wait_response_headers', conn.getresponse)
        report['http_status'] = response.status
        raw = timed(report, 'response_body_read', response.read)
        http_finished = time.perf_counter()
        (output / 'response_body.bin').write_bytes(raw)
        try:
            result = json.loads(raw)
        except (ValueError, UnicodeDecodeError):
            raise ValueError('HTTP {} returned non-JSON; see response_body.bin'.format(response.status))
        write_json(output / 'response.json', result)
        if not isinstance(result, dict):
            raise ValueError('Expected a JSON object response')
        report['request_id'] = result.get('request_id')
        report['service_ok'] = result.get('ok')
        diagnostics = result.get('diagnostics') or {}
        report['server_timing'] = {key: diagnostics.get(key) for key in (
            'service_total_wall_ms', 'service_compute_wall_ms', 'stage_timing_ms')}
        log('HTTP {}; ok={}; request_id={}'.format(response.status, result.get('ok'), result.get('request_id')))
        for key, value in report['server_timing'].items():
            log('{}: {}'.format(key, json.dumps(value, ensure_ascii=False)))
        body_read_ms = (diagnostics.get('stage_timing_ms') or {}).get('body_read')
        if isinstance(body_read_ms, (int, float)) and body_read_ms > 0:
            # Server read may start after some bytes were already buffered: this is only an approximation.
            report['approx_body_read_mbit_s'] = round(len(body) * 8 / (body_read_ms * 1000), 3)
            log('Approx. request bytes / server body_read: {} Mbit/s (not a wire-speed measurement)'.format(
                report['approx_body_read_mbit_s']))
        return 0 if response.status == 200 and result.get('ok') is True else 1
    finally:
        report['client_timing_ms']['http_total'] = round(((http_finished or time.perf_counter()) - http_started) * 1000, 3)
        conn.close()


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--rgb', type=Path, default=DEFAULT_FRAME / 'rgb.jpg')
    parser.add_argument('--depth', type=Path, default=DEFAULT_FRAME / 'depth_mm.npy')
    parser.add_argument('--url', default='http://192.168.3.107:25540/infer')
    parser.add_argument('--metadata', type=Path, help='Current-frame K/T and request options; overrides timing fixture')
    parser.add_argument('--target-type', choices=('sku', 'basket'))
    parser.add_argument('--sku-typ', choices=('bottle', 'box', 'tube'))
    parser.add_argument('--side', choices=('LEFT', 'RIGHT'))
    parser.add_argument('--depth-dtype', choices=('float32', 'preserve'), default='float32')
    parser.add_argument('--proxy', help='Explicit HTTP proxy, e.g. http://192.168.3.107:17891; default DIRECT')
    parser.add_argument('--timeout', type=float, default=300, help='Per blocking socket operation, seconds (default 300)')
    parser.add_argument('--chunk-kib', type=int, default=64, help='Socket write size; wire uses Content-Length, NOT HTTP chunked')
    parser.add_argument('--output-dir', type=Path, default=Path('logs/estimation_mock'))
    parser.add_argument('--save-request', action='store_true', help='Also save the full Base64 request.json')
    parser.add_argument('--prepare-only', action='store_true', help='Save request.json without sending')
    args = parser.parse_args()
    if args.timeout <= 0 or args.chunk_kib <= 0:
        parser.error('--timeout and --chunk-kib must be positive')
    output = args.output_dir / (datetime.now().strftime('%Y%m%d_%H%M%S') + '_' + uuid.uuid4().hex[:8])
    output.mkdir(parents=True, exist_ok=False)
    report = {'started_at': datetime.now().astimezone().isoformat(), 'url': args.url,
              'proxy': args.proxy, 'metadata_source': str(args.metadata) if args.metadata else 'timing_fixture_20261001_084306_f205f149',
              'client_timing_ms': {}}
    log('Output: ' + str(output.resolve()))
    started = time.perf_counter()
    code = 1
    try:
        parse_http_url(args.url)
        if args.proxy:
            parse_http_url(args.proxy)
        body = prepare_body(args, report, output)
        code = 0 if args.prepare_only else post_body(args, body, report, output)
    except KeyboardInterrupt:
        report['error'] = 'Interrupted by user'
        code = 130
        log(report['error'])
    except Exception as exc:
        report['error'] = '{}: {}'.format(type(exc).__name__, exc)
        log(report['error'])
    finally:
        report['client_timing_ms']['run_total'] = round((time.perf_counter() - started) * 1000, 3)
        report['exit_code'] = code
        write_json(output / 'timing.json', report)
        log('Client timings (ms): ' + json.dumps(report['client_timing_ms']))
        log('Saved: ' + str(output.resolve()))
    return code


if __name__ == '__main__':
    sys.exit(main())
