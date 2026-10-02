#!/usr/bin/env python3
"""Loopback-only SAM3 debug UI. Run with Python + requests + numpy + OpenCV."""
import argparse
import base64
import binascii
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import json
import math
from pathlib import Path
import re
import threading
import time
from urllib.parse import parse_qs, urlsplit

import cv2
import numpy as np
import requests

REPO = Path(__file__).resolve().parents[2]
ASSETS = Path(__file__).with_name('sam3_debug')
PROFILES = REPO / 'estimation/deploy/sku_profiles.json'
SAMPLES = REPO / 'logs/estimation/requests'
DEFAULT_UPSTREAM = 'http://192.168.3.107:25541/api/v1/segment'
MAX_BODY = 32 * 1024 * 1024
APP_ID = 'cdf-sam3-debug'


class ApiError(Exception):
    def __init__(self, message, status=400):
        super().__init__(message)
        self.status = status


def read_json(path):
    return json.loads(path.read_text(encoding='utf-8-sig'))


def sample_dir(key):
    if not isinstance(key, str) or not re.fullmatch(r'(sku|basket)/[A-Za-z0-9_-]{1,100}', key):
        raise ApiError('无效的日志编号。')
    path = (SAMPLES / key).resolve()
    if not path.is_relative_to(SAMPLES.resolve()) or not path.is_dir():
        raise ApiError('找不到这条日志。', 404)
    return path


def sample_metadata(path):
    meta = read_json(path / 'request_metadata.json') if (path / 'request_metadata.json').is_file() else {}
    box = meta.get('box_selection') or {}
    return {k: v for k, v in {
        'sku_id': meta.get('sku_id'), 'sku_typ': meta.get('sku_typ'),
        'prompt': meta.get('sam3_prompt'),
        'threshold': box.get('target_threshold', meta.get('sam3_threshold')),
    }.items() if v is not None}


def get_config(upstream):
    library = read_json(PROFILES)
    profiles = [{'id': key, **{field: value for field, value in profile.items()
                             if field in ('name', 'sku_typ', 'sam3_prompt', 'target_threshold')}}
                for key, profile in library['profiles'].items()]
    paths = sorted((p for bucket in ('sku', 'basket') for p in (SAMPLES / bucket).glob('*')
                    if p.is_dir() and (p / 'input_rgb.jpg').is_file()),
                   key=lambda p: p.name, reverse=True)[:40]
    samples = []
    for path in paths:
        try:
            samples.append({'key': f'{path.parent.name}/{path.name}', 'name': path.name,
                            **sample_metadata(path)})
        except (OSError, ValueError, TypeError):
            continue
    return {'upstream': upstream, 'profiles': profiles, 'samples': samples,
            'revision': library.get('revision'), 'max_image_bytes': 20 * 1024 * 1024}


def threshold_value(payload, name, default):
    value = payload.get(name, default)
    if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value) or not 0 <= value <= 1:
        raise ApiError(f'{name} 必须是 0～1 之间的数字。')
    return float(value)


def segment(payload, upstream):
    started = time.perf_counter()
    if not isinstance(payload, dict):
        raise ApiError('请求必须是 JSON 对象。')
    prompt = payload.get('prompt', '')
    if not isinstance(prompt, str) or not prompt.strip() or len(prompt) > 2000:
        raise ApiError('请输入 1～2000 个字符的 prompt。')
    threshold = threshold_value(payload, 'threshold', 0.5)
    mask_threshold = threshold_value(payload, 'mask_threshold', 0.5)
    encoded = payload.get('image_base64')
    if not isinstance(encoded, str):
        raise ApiError('请先上传或选择一张图片。')
    try:
        raw = base64.b64decode(encoded, validate=True)
    except (ValueError, binascii.Error) as exc:
        raise ApiError('图片 Base64 编码无效。') from exc
    if not raw or len(raw) > 20 * 1024 * 1024:
        raise ApiError('图片文件必须小于 20 MiB。')
    image = cv2.imdecode(np.frombuffer(raw, np.uint8), cv2.IMREAD_COLOR)
    if image is None:
        raise ApiError('无法读取图片，请使用 JPG、PNG 或 WebP。')
    height, width = image.shape[:2]
    if height * width > 24_000_000:
        raise ApiError('图片超过 2400 万像素，请缩小后上传。')
    decoded_at = time.perf_counter()
    ok, png = cv2.imencode('.png', image)
    if not ok:
        raise ApiError('无法将图片编码为 PNG。')
    encoded_at = time.perf_counter()
    # Match estimation's BGR decode -> lossless PNG transport. Ignore proxy env.
    try:
        with requests.Session() as session:
            session.trust_env = False
            response = session.post(upstream,
                                    files={'image': ('frame.png', png.tobytes(), 'image/png')},
                                    data={'prompt': prompt.strip(), 'threshold': str(threshold),
                                          'mask_threshold': str(mask_threshold)},
                                    timeout=(5, 180), allow_redirects=False)
    except requests.Timeout as exc:
        raise ApiError('SAM3 请求超时，请检查 107 的网络和模型服务。', 504) from exc
    except requests.RequestException as exc:
        raise ApiError(f'无法连接 SAM3：{exc}', 502) from exc
    returned_at = time.perf_counter()
    if response.status_code != 200:
        raise ApiError(f'SAM3 返回 HTTP {response.status_code}：{response.text[:500]}', 502)
    try:
        upstream_data = response.json()
    except ValueError as exc:
        raise ApiError('SAM3 返回的内容不是有效 JSON。', 502) from exc
    if not isinstance(upstream_data, dict) or upstream_data.get('ok') is False:
        raise ApiError('SAM3 返回失败：' + str(upstream_data)[:500], 502)
    items = upstream_data.get('instances', [])
    if not isinstance(items, list):
        raise ApiError('SAM3 的 instances 格式不正确。', 502)
    instances = []
    for index, item in enumerate(items, 1):
        try:
            mask_b64 = item['mask_png_base64']
            mask = cv2.imdecode(np.frombuffer(base64.b64decode(mask_b64, validate=True), np.uint8), cv2.IMREAD_GRAYSCALE)
            bbox = np.asarray(item['bbox_xyxy'], dtype=float)
            score = float(item['score'])
            if mask is None or mask.shape != (height, width) or bbox.shape != (4,) or not np.isfinite(bbox).all() or not math.isfinite(score):
                raise ValueError('mask dimensions / bbox / score')
            area = int(np.count_nonzero(mask))
            instances.append({'id': index, 'upstream_instance_id': item.get('instance_id'),
                              'score': score, 'bbox_xyxy': bbox.tolist(), 'area_pixels': area,
                              'area_ratio': area / (width * height), 'mask_png_base64': mask_b64})
        except (KeyError, TypeError, ValueError, cv2.error, binascii.Error) as exc:
            raise ApiError(f'SAM3 第 {index} 个实例的 mask 或坐标无效。', 502) from exc
    return {'ok': True, 'prompt': prompt.strip(), 'threshold': threshold, 'mask_threshold': mask_threshold,
            'upstream': upstream, 'image_name': str(payload.get('image_name', 'image'))[:200],
            'width': width, 'height': height, 'instances': instances, 'instance_count': len(instances),
            'timing_ms': {'image_decode': round((decoded_at - started) * 1000, 2),
                          'image_encode': round((encoded_at - decoded_at) * 1000, 2),
                          'upstream': round((returned_at - encoded_at) * 1000, 2),
                          'total': round((time.perf_counter() - started) * 1000, 2)}}


class DebugServer(ThreadingHTTPServer):
    daemon_threads = True

    def __init__(self, address, upstream):
        super().__init__(address, Handler)
        self.upstream = upstream
        self.inference_lock = threading.Lock()


class Handler(BaseHTTPRequestHandler):
    def send_content(self, status, data, content_type):
        self.send_response(status)
        self.send_header('Content-Type', content_type)
        self.send_header('Content-Length', str(len(data)))
        self.send_header('Cache-Control', 'no-store')
        self.send_header('X-Content-Type-Options', 'nosniff')
        self.send_header('Referrer-Policy', 'no-referrer')
        self.end_headers()
        self.wfile.write(data)

    def json(self, status, data):
        self.send_content(status, json.dumps(data, ensure_ascii=False, allow_nan=False).encode('utf-8'),
                          'application/json; charset=utf-8')

    def check_origin(self):
        expected = {f'127.0.0.1:{self.server.server_port}', f'localhost:{self.server.server_port}'}
        if self.headers.get('Host') not in expected:
            raise ApiError('仅允许本机访问。', 403)
        origin = self.headers.get('Origin')
        if origin and origin not in {f'http://{host}' for host in expected}:
            raise ApiError('不接受其他网页发起的请求。', 403)

    def do_GET(self):
        try:
            self.check_origin()
            url = urlsplit(self.path)
            if url.path == '/health':
                self.json(200, {'ok': True, 'app': APP_ID, 'upstream': self.server.upstream})
            elif url.path == '/api/config':
                self.json(200, get_config(self.server.upstream))
            elif url.path == '/api/sample':
                path = sample_dir(parse_qs(url.query).get('key', [''])[0])
                image_path = (path / 'input_rgb.jpg').resolve()
                if not image_path.is_relative_to(SAMPLES.resolve()) or not image_path.is_file():
                    raise ApiError('日志图片不存在。', 404)
                raw = image_path.read_bytes()
                self.json(200, {'image_base64': base64.b64encode(raw).decode('ascii'), 'mime': 'image/jpeg',
                                'name': path.name + '/input_rgb.jpg', 'bytes': len(raw), **sample_metadata(path)})
            else:
                files = {'/': ('index.html', 'text/html; charset=utf-8'),
                         '/index.html': ('index.html', 'text/html; charset=utf-8'),
                         '/style.css': ('style.css', 'text/css; charset=utf-8'),
                         '/app.js': ('app.js', 'text/javascript; charset=utf-8'),
                         '/favicon.svg': ('favicon.svg', 'image/svg+xml')}
                if url.path not in files:
                    raise ApiError('页面不存在。', 404)
                name, mime = files[url.path]
                self.send_content(200, (ASSETS / name).read_bytes(), mime)
        except ApiError as exc:
            self.json(exc.status, {'ok': False, 'error': str(exc)})
        except (OSError, ValueError, KeyError) as exc:
            self.json(500, {'ok': False, 'error': f'读取本地配置失败：{exc}'})

    def do_POST(self):
        try:
            self.check_origin()
            if self.path != '/api/segment':
                raise ApiError('接口不存在。', 404)
            if self.headers.get_content_type() != 'application/json':
                raise ApiError('请使用 application/json。', 415)
            try:
                length = int(self.headers.get('Content-Length', '0'))
            except ValueError as exc:
                raise ApiError('无效的 Content-Length。') from exc
            if not 0 < length <= MAX_BODY:
                raise ApiError('请求为空或超过 32 MiB。', 413)
            self.connection.settimeout(30)
            try:
                payload = json.loads(self.rfile.read(length))
            except (ValueError, TimeoutError) as exc:
                raise ApiError('请求 JSON 无效或未完整上传。') from exc
            if not self.server.inference_lock.acquire(blocking=False):
                raise ApiError('已有分割请求正在运行，请等待完成。', 409)
            try:
                result = segment(payload, self.server.upstream)
            finally:
                self.server.inference_lock.release()
            self.json(200, result)
        except ApiError as exc:
            self.json(exc.status, {'ok': False, 'error': str(exc)})
        except (cv2.error, OSError, ValueError) as exc:
            self.json(500, {'ok': False, 'error': f'处理失败：{exc}'})


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--port', type=int, default=8765)
    parser.add_argument('--sam3-url', default=DEFAULT_UPSTREAM)
    args = parser.parse_args()
    url = urlsplit(args.sam3_url)
    if url.scheme not in ('http', 'https') or not url.hostname or url.username or url.password:
        parser.error('--sam3-url 必须是无账号密码的 HTTP(S) 地址')
    server = DebugServer(('127.0.0.1', args.port), args.sam3_url)
    print(f'SAM3 Debug: http://127.0.0.1:{server.server_port}', flush=True)
    print(f'Upstream: {args.sam3_url} (proxy environment ignored)', flush=True)
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        server.server_close()


if __name__ == '__main__':
    main()
