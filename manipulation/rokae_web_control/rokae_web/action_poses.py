"""Independent action calibrations; never fall back to mutable teaching points."""
import copy
import json
import math
from pathlib import Path

from .backends import BackendError
from .memory_points import MemoryPointStore, vector
from .trunk_frame import TRUNK_FRAME

POSES = Path(__file__).resolve().parent.parent / 'poses'


def action_point(config, name, mode):
    path = Path(config.get('action_poses_file', POSES / 'action_poses.json'))
    matches = [p for p in MemoryPointStore(path).list() if p['name'] == name]
    if len(matches) != 1 or matches[0]['mode'] != mode:
        raise BackendError(f'独立动作标定位姿缺失或模式不符：{name}（{path}）')
    return copy.deepcopy(matches[0])


def box_heights(pose_config):
    path = Path(pose_config.get('box_grasp_file', POSES / 'box_grasp.json'))
    try:
        data = json.loads(path.read_text(encoding='utf8'))
        if data['version'] != 1 or data['frame'] != TRUNK_FRAME or data['unit'] != 'mm':
            raise ValueError('标定版本、坐标系或单位不符')
        heights = data['heights_mm']
        for name in ('grasp', 'descend', 'lift'):
            value = heights[name]
            if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value):
                raise ValueError('标定高度无效：' + name)
        if not heights['descend'] < heights['grasp'] < heights['lift']:
            raise ValueError('下压、抓取、提起高度顺序不正确')
        return copy.deepcopy(data)
    except (OSError, ValueError, KeyError, TypeError) as exc:
        raise BackendError(f'盒子高度标定读取失败（{path}）：{exc}') from exc


def tube_height(pose_config):
    path = Path(pose_config.get('tube_grasp_file', POSES / 'tube_grasp.json'))
    try:
        data = json.loads(path.read_text(encoding='utf8'))
        if data['version'] != 1 or data['frame'] != TRUNK_FRAME or data['unit'] != 'mm':
            raise ValueError('标定版本、坐标系或单位不符')
        value = data['pregrasp_flange_height_mm']
        if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value):
            raise ValueError('软管法兰高度无效')
        return copy.deepcopy(data)
    except (OSError, ValueError, KeyError, TypeError) as exc:
        raise BackendError(f'软管高度标定读取失败（{path}）：{exc}') from exc


def tube_scan_point(config, name, mode):
    local = dict(config, action_poses_file=config.get('tube_scan_poses_file', POSES / 'tube_scan.json'))
    return action_point(local, name, mode)


def tube_scan_turn_point(config, mode):
    """Frozen successful right-arm arrival; never substitute a teaching point."""
    path = Path(config.get('tube_scan_turn_file', POSES / 'tube_scan_turn.json'))
    try:
        point = json.loads(path.read_text(encoding='utf8'))
        if point['version'] != 1 or point['name'] != '软管扫码翻转A' or point['mode'] != mode:
            raise ValueError('翻转点版本、名称或模式不符')
        state = point['state']
        vector(state['joints_deg']['right_arm'], 7, 'A点右臂关节')
        vector(state['poses']['right_arm'], 6, 'A点右臂位姿')
        vector([state['arm_elbow_deg']['right_arm']], 1, 'A点右臂臂角')
        if state['pose_frames']['right_arm'] != 'right_arm_sdk_world':
            raise ValueError('A点右肩坐标系不符')
        for part in ('end', 'ref'):
            vector(state['toolsets']['right_arm'][part], 6, 'A点右臂工具/工件')
        conf = state['arm_conf_data']['right_arm']
        if not isinstance(conf, list) or not conf or any(type(v) is not int for v in conf):
            raise ValueError('A点右臂构型数据无效')
        return copy.deepcopy(point)
    except (OSError, ValueError, KeyError, TypeError, BackendError) as exc:
        raise BackendError(f'软管扫码翻转A读取失败（{path}）：{exc}') from exc
