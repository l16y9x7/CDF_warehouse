"""Web camera facade: all images come from the boot-managed 8085 Owner."""
import json
import time
from pathlib import Path
from urllib.parse import urlencode
from urllib.request import urlopen
import cv2
import numpy as np
from .backends import BackendError
from .camera import CameraSnapshot
from .camera_manager import CAMERA_LABELS, MultiCameraRecordingStore
from .capture_preview import capture_rgb
from .rgb_recording import RgbRecordingStore
from .ros_camera import ExistingCameraRecovery


class HttpCameraManager:
    externally_managed = True
    def __init__(self, config, directory, quality=90):
        self.config, self.jpeg_quality = config, quality
        self.base = 'http://127.0.0.1:8085'
        self.capture_timeout = 5.0
        self.store = MultiCameraRecordingStore(directory)
        self.rgb_store = RgbRecordingStore(config.get('left_wrist_rgb_directory', '/home/admin/mui/rokae_web_control/data_left_wrist_rgb'))
        self.right_rgb_store = RgbRecordingStore(config.get('right_wrist_rgb_directory',
            str(self.rgb_store.root.with_name('data_right_wrist_rgb'))))
        self.recovery = ExistingCameraRecovery()

    def _get(self, route, **query):
        with urlopen(self.base+route+('?' + urlencode(query) if query else ''), timeout=5) as response:
            data = response.read(65537)
        if len(data) > 65536:
            raise BackendError('相机 JSON 响应过大')
        return json.loads(data)

    def status(self):
        try:
            rows = self._get('/camera/list')
            rows = rows if isinstance(rows, list) else rows['cameras']
            rows = {r['id']: r for r in rows}
            error = ''
        except Exception as exc:
            rows, error = {}, str(exc)
        result = {}
        for name, label in CAMERA_LABELS.items():
            row = rows.get(name, {})
            color_ready = bool(row.get('color',{}).get('online'))
            depth_ready = bool(row.get('depth',{}).get('online') and row.get('depth',{}).get('aligned'))
            result[name] = dict(camera_id=name, label=label, enabled=color_ready and (name != 'head' or depth_ready),
                available=bool(row.get('enabled')), frame_available=color_ready,
                rgb_available=bool(row.get('color',{}).get('online')),
                depth_available=name == 'head' and bool(row.get('depth',{}).get('online')),
                source='http8085', externally_managed=True, error=error or row.get('error',''),
                rgb_profile=dict(width=row.get('color',{}).get('width'),height=row.get('color',{}).get('height'),fps='—'),
                recovery_available=True)
            result[name].update(self.recovery.status())
        return result

    def set_enabled(self, *_):
        raise BackendError('相机由现有开机服务管理')

    def restart(self, camera_id):
        if camera_id != 'all':
            raise BackendError('请刷新网页，使用三路摄像头服务重启按钮')
        return self.recovery.trigger()

    def frame_jpeg(self, camera_id, kind, after_sequence, timeout):
        if kind != 'rgb':
            raise BackendError('网页预览仅支持 RGB')
        return capture_rgb(camera_id, timeout), time.time_ns(), True

    def record_rgb(self, camera_id):
        if camera_id not in CAMERA_LABELS:
            raise BackendError('未知相机')
        if camera_id == 'left_wrist':
            return self.rgb_store.save(capture_rgb(camera_id, 5))
        result = self._get('/camera/snapshot', camera=camera_id, type='color')
        if result.get('ok') is not True:
            raise BackendError(str(result))
        path = Path(result['image_path'])
        return dict(camera_id=camera_id, path=str(path), directory=str(path.parent),
                    relative_directory=str(path.parent), files=[path.name])

    def _scan_store(self, camera_id):
        if camera_id not in ('left_wrist', 'right_wrist'):
            raise BackendError('扫码只支持左右腕 RGB')
        return self.right_rgb_store if camera_id == 'right_wrist' else self.rgb_store

    def begin_scan_recording(self, camera_id='left_wrist'):
        return self._scan_store(camera_id).begin_scan()

    def record_scan_rgb(self, group, index, cancelled, camera_id='left_wrist'):
        if cancelled.is_set():
            raise BackendError('扫码采集已停止')
        store = self._scan_store(camera_id)
        jpeg, metadata = capture_rgb(camera_id, 5, return_metadata=True)
        if cancelled.is_set():
            raise BackendError('扫码采集已停止')
        saved = store.save_scan(group, index, jpeg)
        saved.update(camera_id=camera_id, image_path=metadata['color']['path'], capture_id=metadata['capture_id'])
        return saved

    def fresh_snapshot(self, camera_id, timeout=None):
        if camera_id != 'head':
            raise BackendError('腕部仅支持 RGB，不能采集 RGB-D')
        result = self._get('/camera/rgbd', camera=camera_id)
        if result.get('ok') is not True or result.get('same_shot') is not True:
            raise BackendError(str(result))
        root = Path('/shared/frames').resolve()
        folder = root / result['capture_id']
        paths = [Path(result[k]).resolve() for k in ('rgb','depth')]
        if folder.parent != root or any(p.parent != folder for p in paths):
            raise BackendError('RGB-D 文件路径不属于同次采集')
        rgb = cv2.imread(str(paths[0]))
        depth = np.load(paths[1], allow_pickle=False)
        if rgb is None or depth.shape != rgb.shape[:2] or depth.dtype != np.uint16:
            raise BackendError('RGB-D 尺寸/数据类型不一致')
        timestamps = result['timestamps']
        return CameraSnapshot(rgb, depth.astype(np.float32), result['captured_at'],
            timestamps['color_s']*1000, timestamps['depth_s']*1000, time.time_ns(),
            dict(source='http8085', aligned=True, depth_unit='mm', color_intrinsics=result['color_intrinsics'], width=rgb.shape[1], height=rgb.shape[0]))

    snapshot = fresh_snapshot

    def save_snapshot(self, camera_id, snapshot, robot_state):
        return self.store.save({camera_id: snapshot}, robot_state)

    def record(self, camera_id, robot_state):
        if camera_id != 'head':
            return self.record_rgb(camera_id)
        return self.save_snapshot(camera_id, self.fresh_snapshot(camera_id), robot_state)

    def close(self):
        self.recovery.close()
