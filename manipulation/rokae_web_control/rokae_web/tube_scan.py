"""Shared two-photo tube scan; saved body poses are never dispatched."""
import copy

from .arm_movel import load_guard_plane
from .backends import BackendError
from .memory_points import require_idle
from .placement_sequence import PlacementSequence
from .scan_sequence import arm_target


class TubeScanMotion(PlacementSequence):
    """Use the shared guarded moves/arrival checks inside the scan job's lifecycle."""
    def __init__(self, scan):
        super().__init__(scan.service)
        self.scan = scan
        self.cancelled = scan.cancelled
        self.poll_seconds, self.timeout_seconds = scan.poll_seconds, scan.timeout_seconds
        self.attempted = False

    def _check_cancel(self):
        self.scan._check_cancel()
        if not self.service.armed:
            raise BackendError('扫码控制已锁定；后续动作已取消')

    def _set(self, **values):
        self.scan._set(**values)


def run(scan, points, speeds):
    scan._motion_attempted = False
    motion = TubeScanMotion(scan)
    try:
        body = scan.service.robot.read_memory_state()
        require_idle(body)
        plane = load_guard_plane(scan.service.config)
        # Freeze and validate all recorded arm tools/frames before the first move.
        for point in points:
            state = point['state']
            if state.get('toolsets') != body.get('toolsets'):
                raise BackendError('软管扫码标定工具/TCP 或参考系与当前配置不一致')
            if any(state.get('pose_frames', {}).get(arm) != f'{arm}_sdk_world'
                   for arm in ('left_arm', 'right_arm')):
                raise BackendError('软管扫码标定肩部坐标系不符')
        motion._check_cancel()
        group = scan.service.camera.begin_scan_recording(camera_id='left_wrist')
        scan._set(directory=group['directory'], relative_directory=group['relative_directory'])
        scan.service.audit_event('scan_sequence_started', points=points, speed=speeds,
                                 body=body, group=group, sku_typ='tube', camera='left_wrist')
        scan._set(stage='tube_scan1')
        first = scan._arms(arm_target(points[0]['state'], body), speeds, '软管扫码1', body)
        first = motion._fresh(dict(first, toolsets=body['toolsets']))
        photos = []
        for index in (1, 2):
            if index == 2:
                # _memory_right copies only the saved right-arm fields; the left
                # wrist camera remains exactly at the first scan pose.
                scan._set(stage='tube_turn_a', phase='arms', message='右臂前往历史翻转A点，回放完整7轴构型，左臂保持不动')
                turned = motion._memory_right(points[1]['state'], shifted, speeds)
                pose = list(turned['poses']['right_arm']); pose[1] += 100.0
                scan._set(stage='tube_scan2_approach', phase='arms',
                          message='A点已到位，右臂保持姿态沿右肩Y+左移100 mm到第二拍照点', arm_target=pose)
                arrived = motion._linear(pose, turned, plane, speeds)
            else:
                arrived = first
            motion._fresh(arrived)
            scan._set(stage=f'tube_capture{index}', phase='capture',
                      message=f'软管扫码{index}已到位，采集左腕 RGB')
            photo = scan.service.camera.record_scan_rgb(group, index, scan.cancelled, camera_id='left_wrist')
            photos.append(photo)
            scan._set(photos=copy.deepcopy(photos), completed_points=index)
            motion._fresh(arrived)
            pose = list(arrived['poses']['right_arm']); pose[1] -= 100.0
            scan._set(stage=f'tube_shift{index}', phase='arms',
                      message=f'右臂从软管扫码{index}沿右肩 Y−100 mm 平移', arm_target=pose)
            shifted = motion._linear(pose, arrived, plane, speeds)
        motion._fresh(shifted)
        scan._set(stage='tube_return_arms')
        returned = scan._arms(arm_target(points[2]['state'], body), speeds, '返回 L2抓取双臂位置', body)
        returned = motion._fresh(dict(returned, toolsets=body['toolsets']))
        target = list(returned['poses']['trunk']); target[0] += 100.0
        scan._set(stage='tube_advance', phase='trunk',
                  message='双臂已返回 L2抓取，躯干沿 SDK X+100 mm 前进；头部与双臂保持不动', trunk_target=target)
        final = motion._trunk_linear(target, returned, speeds, arm_module='right_arm')
        motion._check_cancel()
        scan._set(active=False, phase='completed', stage='completed', final_state=final,
                  message='软管扫码完成：已保存2张左腕照片，双臂已回 L2抓取，躯干前进100 mm到位')
    except Exception as exc:
        errors = scan._stop_and_verify() if scan._motion_attempted or motion.attempted else []
        if errors:
            with scan.service._lock:
                scan.service.armed = False
        scan._set(active=False, stop_unconfirmed=bool(errors),
                  phase='cancelled' if scan.cancelled.is_set() and not errors else 'failed',
                  message=str(exc) + ('；停止未确认：' + '; '.join(errors) if errors else ''))
