"""One box or five bottle arm poses, wrist photographs, then arms/trunk return."""
import copy
import math
import threading

from .backends import BackendError
from .control_trace import fields, operation, worker_thread
from .memory_points import ARMS, MemoryPoints, joint_error, require_idle
from .memory_motion import tool_signature
from .pose_frames import world_pose_to_ref
from .action_poses import action_point, tube_scan_point, tube_scan_turn_point
from .pose_protocol import sku_type


POINT_NAMES = tuple(f'扫码{i}' for i in range(1, 6)) + ('L2抓取',)


def arm_target(saved, body):
    target = copy.deepcopy(saved)
    for name in ('head', 'trunk'):
        target['joints_deg'][name] = list(body['joints_deg'][name])
        if name in body['poses']:
            target['poses'][name] = list(body['poses'][name])
    return target


def offset_target(backend, saved, body, cancelled):
    """Replace SDK-world Y absolutely; preserve all other TCP axes and elbow."""
    target = arm_target(saved, body)
    with backend._lock:
        sdk = backend._load_sdk()
        for name, y in (('left_arm', 50.0), ('right_arm', -50.0)):
            if cancelled.is_set():
                raise BackendError('扫码已停止')
            target['poses'][name][1] = y
            robot = backend._robot(name)
            tool = backend._call(f'读取 {name} 工具', robot.toolset)
            if tool_signature(tool) != saved['toolsets'][name]:
                raise BackendError('工具/TCP 或工件坐标系已改变，请重新保存扫码点')
            cart = sdk.CartesianPosition(world_pose_to_ref(backend._pose_to_sdk(target['poses'][name]),
                                                          backend._world_from_ref(tool)))
            cart.elbow = math.radians(target['arm_elbow_deg'][name])
            cart.hasElbow = True
            cart.confData = list(target['arm_conf_data'][name])
            from .memory_points import vector
            joints = vector(list(backend._call(f'{name} 扫码5偏移逆解', robot.model().calcIk, cart, tool)), 7, name)
            target['joints_deg'][name] = [math.degrees(v) for v in joints]
    return target


class ScanSequence(MemoryPoints):
    # Reuse the existing joint+TCP arrival checks and verified stop, with an
    # independent cancellation event/job; never start a saved head/trunk stage.
    def __init__(self, service):
        self.service = service
        self.store = service.memory.store
        self.mode = service.memory.mode
        self.cancelled = threading.Event()
        self.thread = None
        self.poll_seconds, self.timeout_seconds = 0.1, 600.0
        self.job = dict(active=False, phase='idle', message='扫码采集待命', photos=[], completed_points=0)

    def status(self):
        return dict(version='scan-sequence-v5', **copy.deepcopy(self.job))

    def ensure_idle(self):
        if self.job.get('stop_unconfirmed'):
            raise BackendError('扫码停止尚未确认，请先停止扫码并确认控制器静止')
        if self.job['active']:
            raise BackendError('扫码采集正在执行，请等待完成或先停止扫码')

    def _check_cancel(self):
        if self.cancelled.is_set():
            raise BackendError('扫码采集已停止；后续动作已取消')

    def _set(self, **values):
        with self.service._lock:
            self.job.update(values)
        self.service.audit_event('scan_sequence_progress', **values)

    @operation('scan_sequence')
    def execute(self, payload):
        if not isinstance(payload, dict) or set(payload) - {'sku_typ'}:
            raise BackendError('扫码流程只接受 sku_typ')
        try:
            kind = sku_type(payload.get('sku_typ', 'bottle'))
        except ValueError as exc:
            raise BackendError(str(exc)) from exc
        names = (('软管扫码1', '软管扫码翻转A', 'L2抓取') if kind == 'tube' else
                 ('盒子扫码', 'L2抓取') if kind == 'box' else POINT_NAMES)
        camera_id = 'right_wrist' if kind == 'box' else 'left_wrist'
        with self.service._lock:
            self.service._require_armed()
            selected = [(tube_scan_turn_point(self.service.config, self.mode) if name == '软管扫码翻转A'
                         else (tube_scan_point if name == '软管扫码1' else action_point)(
                             self.service.config, name, self.mode)) for name in names]
            if not hasattr(self.service.camera, 'record_scan_rgb'):
                raise BackendError('当前相机不支持扫码 RGB 采集')
            camera = self.service.camera.status().get(camera_id, {})
            if not camera.get('enabled') or camera.get('available') is False:
                raise BackendError(f'{camera_id} RGB 数据不可用，请先恢复摄像头')
            require_idle(self.service.robot.read_state())
            speeds = dict(linear_mm_s=float(self.service.speed_mm_s),
                          rotation_deg_s=float(self.service.rotation_deg_s))
            self.service.chassis.stop()
            self.cancelled.clear()
            self.job = dict(active=True, phase='planning', message='正在准备扫码流程',
                            photos=[], completed_points=0, total_photos=len(names)-1, sku_typ=kind,
                            camera=camera_id, speed=speeds, **fields())
            self.thread = worker_thread(target=self._run_scan, args=(selected, speeds, kind),
                                        name='scan-sequence', daemon=True)
            self.thread.start()
            return self.status()

    def _plan(self, target, speeds, strict_linear=False):
        self._check_cancel()
        current = self.service.robot.read_state()
        for name, maximum in self.service.config['motion']['max_joint_step_deg'].items():
            if maximum is not None and joint_error(current, target, (name,)) > float(maximum):
                raise BackendError(f'{name} 超过当前配置的单次关节变化上限')
        plan = self.service.robot.prepare_memory_motion(target, speeds, self.cancelled)
        if strict_linear and any(v['motion'] not in ('MoveL', '已到位', 'MoveL (MOCK)')
                                 for v in plan['modes'].values()):
            raise BackendError('扫码5的 Y 偏移直线轨迹预检未通过，后续动作已取消')
        plan['synchronized'] = True
        return plan

    def _arms(self, target, speeds, label, fixed, strict_linear=False):
        self._set(phase='planning', message=f'正在预检{label}')
        plan = self._plan(target, speeds, strict_linear)
        if joint_error(plan['start'], fixed, ('head', 'trunk')) > 0.3:
            raise BackendError('头部或躯干发生变化，扫码已中止')
        self._check_cancel()
        self._set(phase='arms', message=f'{label}：双臂运动中，等待两臂到位', arm_modes=plan['modes'])
        self._motion_attempted = True
        self.service.robot.start_memory_arms(plan, self.cancelled)
        self.service._request_motion_sampling('scan_arms', 'upper_body')
        return self._wait_reached(target, ARMS, fixed)

    def _run_scan(self, points, speeds, kind='bottle'):
        if kind == 'tube':
            from .tube_scan import run
            return run(self, points, speeds)
        self._motion_attempted = False
        try:
            body = self.service.robot.read_memory_state()
            require_idle(body)
            # Resolve offset IK before any movement, keeping recorded elbow/config.
            if kind == 'box':
                offset = None
            elif self.service.hardware_enabled:
                offset = offset_target(self.service.robot, points[4]['state'], body, self.cancelled)
            else:
                offset = arm_target(points[4]['state'], body)
                offset['poses']['left_arm'][1], offset['poses']['right_arm'][1] = 50.0, -50.0
            self._check_cancel()
            camera_id = 'right_wrist' if kind == 'box' else 'left_wrist'
            group = (self.service.camera.begin_scan_recording(camera_id=camera_id) if kind == 'box' else
                     self.service.camera.begin_scan_recording())
            self._set(directory=group['directory'], relative_directory=group['relative_directory'])
            self.service.audit_event('scan_sequence_started', points=points, speed=speeds, body=body, group=group, sku_typ=kind, camera=camera_id)
            photos = []
            for index, point in enumerate(points[:-1], 1):
                target = arm_target(point['state'], body)
                self._arms(target, speeds, point['name'], body)
                self._check_cancel()
                self._set(phase='capture', message=f'{point["name"]}已到位，采集{camera_id} RGB')
                photo = (self.service.camera.record_scan_rgb(group, index, self.cancelled, camera_id=camera_id)
                         if kind == 'box' else self.service.camera.record_scan_rgb(group, index, self.cancelled))
                photos.append(photo)
                self._set(photos=copy.deepcopy(photos), completed_points=index)
            if offset is not None:
                self._arms(offset, speeds, '扫码5偏移（左 Y=50，右 Y=-50）', body, strict_linear=True)
            returned = self._arms(arm_target(points[-1]['state'], body), speeds, '返回 L2抓取双臂位置', body)
            self._check_cancel()
            trunk_target = arm_target(points[-1]['state'], body)
            trunk_target['joints_deg']['trunk'] = list(points[-1]['state']['joints_deg']['trunk'])
            trunk_target['poses']['trunk'] = list(points[-1]['state']['poses']['trunk'])
            plan = self._plan(trunk_target, speeds)
            if joint_error(plan['start'], returned, ('head', 'trunk')) > 0.3:
                raise BackendError('回位前头部或躯干发生变化，已停止流程')
            self._check_cancel()
            self._set(phase='trunk', message='双臂已返回 L2抓取，正在恢复躯干；头部保持不动')
            self.service.robot.start_memory_head_trunk(plan, self.cancelled)
            self.service._request_motion_sampling('scan_trunk_return', 'trunk')
            self._wait_reached(trunk_target, ('trunk',), returned)
            self._check_cancel()
            self._set(active=False, phase='completed', message=f'扫码采集完成：已保存{len(photos)}张图片，双臂与躯干已返回 L2抓取')
        except Exception as exc:
            errors = self._stop_and_verify() if self._motion_attempted else []
            if errors:
                with self.service._lock:
                    self.service.armed = False
            self._set(active=False, stop_unconfirmed=bool(errors),
                      phase='cancelled' if self.cancelled.is_set() and not errors else 'failed',
                      message=str(exc) + ('；停止未确认：' + '; '.join(errors) if errors else ''))

    def stop(self):
        self.cancelled.set()
        with self.service._lock:
            if self.job['active']:
                self.job.update(phase='stopping', message='正在停止扫码；后续动作已取消')
            elif self.job.get('stop_unconfirmed'):
                errors = self._stop_and_verify()
                self.job.update(stop_unconfirmed=bool(errors), message='；'.join(errors) if errors else '已确认停止')
            return self.status()
