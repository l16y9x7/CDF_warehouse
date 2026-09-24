"""Fixed trunk-only barcode pose calibrated from completed retreat logs."""
import json
from pathlib import Path
from .backends import BackendError
from .memory_points import vector
from .trunk_frame import TRUNK_FRAME


def load_barcode_pose(config):
    path = Path(config.get('barcode_pose_file') or
                Path(__file__).resolve().parents[1] / 'poses' / 'barcode_trunk.json')
    try:
        record = json.loads(path.read_text(encoding='utf8'))
        if (record.get('schema_version') != 1 or record.get('frame') != TRUNK_FRAME
                or record.get('pose_type') != 'AGV_item_barcode_scan'):
            raise ValueError('扫码位姿版本、类型或坐标系不符')
        state = record['state']
        if state.get('pose_frames', {}).get('trunk') != TRUNK_FRAME:
            raise ValueError('扫码躯干目标不是躯干 SDK 参考系')
        vector(state['poses']['trunk'], 6, '扫码躯干 Pose')
        for key in ('end', 'ref'):
            vector(state['toolsets']['trunk'][key], 6, '扫码躯干工具/参考系')
    except (OSError, ValueError, KeyError, TypeError) as exc:
        raise BackendError('固定扫码位姿不可用：' + str(exc)) from exc
    return record
