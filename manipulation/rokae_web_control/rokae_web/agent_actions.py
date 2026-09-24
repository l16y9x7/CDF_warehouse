"""Serialized Agent actions over the same ControlService as the web UI."""
import copy
import hashlib
import json
import re
import sqlite3
import threading
import time
from contextlib import contextmanager
from pathlib import Path
from .backends import BackendError
from .memory_points import require_idle
from .agent_geometry import AgentGeometry
from .agent_planning import preflight_pick, prepare_trunk, execute_trunk
from .barcode_pose import load_barcode_pose
from .action_poses import action_point
from .grasp_gripper import ensure_gripper_open, ensure_gripper_position


class AgentError(BackendError):
    def __init__(self, code, message, status=400):
        super().__init__(message)
        self.code, self.status = code, status


class ActionLedger:
    """Reserve before dispatch; never replay an indeterminate physical action."""
    def __init__(self, path):
        Path(path).parent.mkdir(parents=True, exist_ok=True)
        self.db = sqlite3.connect(path, check_same_thread=False)
        self.lock = threading.Lock()
        self.db.execute('PRAGMA synchronous=FULL')
        self.db.execute('CREATE TABLE IF NOT EXISTS actions (key TEXT PRIMARY KEY, digest TEXT, response TEXT)')
        self.db.commit()

    def begin(self, key, route, payload):
        if not isinstance(key, str) or not re.fullmatch(r'[A-Za-z0-9_.:-]{1,128}', key):
            raise AgentError('IDEMPOTENCY_KEY_REQUIRED', '运动接口必须提供有效 Idempotency-Key')
        digest = hashlib.sha256(json.dumps([route, payload], sort_keys=True, allow_nan=False).encode()).hexdigest()
        with self.lock:
            row = self.db.execute('SELECT digest,response FROM actions WHERE key=?', (key,)).fetchone()
            if row:
                if row[0] != digest:
                    raise AgentError('IDEMPOTENCY_CONFLICT', '同一幂等键对应不同请求', 409)
                if row[1] is None:
                    raise AgentError('ACTION_IN_PROGRESS_OR_UNKNOWN', '动作执行中或结果未知，禁止重复下发；请检查日志与机器人状态', 409)
                return json.loads(row[1])
            self.db.execute('INSERT INTO actions VALUES (?,?,NULL)', (key, digest))
            self.db.commit()
        return None

    def finish(self, key, response):
        with self.lock:
            self.db.execute('UPDATE actions SET response=? WHERE key=?', (json.dumps(response, ensure_ascii=False), key))
            self.db.commit()


class AgentActions:
    def __init__(self, service, directory):
        self.service = service
        self.directory = Path(directory)
        self.ledger = ActionLedger(self.directory / 'actions.sqlite3')
        self.geometry = AgentGeometry(service)
        self.active = None
        self.cancelled = threading.Event()
        self.stop_unconfirmed = False
        service.agent_actions = self

    def ensure_idle(self):
        if self.active or self.stop_unconfirmed:
            raise AgentError('BUSY', 'Agent 动作正在执行或停止未确认', 409)
        s = self.service
        for job in (s.memory, s.arm_movel, s.grasp_test, s.scan_sequence, s.placement):
            job.ensure_idle()
        if s._pose_estimation_lock.locked():
            raise AgentError('BUSY', '网页位姿估计正在进行', 409)
        require_idle(s.robot.read_memory_state())

    @contextmanager
    def web_request(self, path, payload):
        # Same lock covers reservation and web dispatch. Do not hold this lock
        # across worker joins or stop calls: those workers also use service._lock.
        stop = path.endswith('/stop') or path == '/api/control/disarm' or (
            path == '/api/gripper/lock' and payload.get('unlocked') is False)
        if stop:
            with self.service._lock:
                if self.active:
                    self.cancelled.set()
            yield
            return
        with self.service._lock:
            if (self.active or self.stop_unconfirmed) and path != '/api/readback':
                raise AgentError('BUSY', 'Agent 动作占用运控，网页仍可查看状态或停止', 409)
            yield

    def _check(self):
        if self.cancelled.is_set():
            raise AgentError('CANCELLED', '动作已停止；后续步骤取消', 409)

    def health(self, *, pose=False):
        s = self.service
        with s._lock:
            state = s.robot.read_memory_state()
            if self.stop_unconfirmed:
                raise AgentError('STOP_UNCONFIRMED', '上一次停止尚未确认', 503)
            g = s.robot.gripper_status()
            if pose and g.get('activation_state') != 3:
                self.ensure_idle()
                s.audit_event('agent_gripper_activation_requested')
                g = s.robot.gripper_activate()
            if g.get('fault_code') or g.get('activation_state') != 3:
                raise AgentError('GRIPPER_NOT_READY', '右夹爪未就绪', 503)
            operations = state.get('operation_state', {})
            if any(str(operations.get(m)).lower() not in ('idle', 'moving', 'mock-idle')
                   for m in ('left_arm', 'right_arm', 'trunk')):
                raise AgentError('ROBOT_NOT_READY', '控制器运行状态异常', 503)
            return dict(status='READY', busy=bool(self.active), right_gripper=g,
                        operation_state=operations, left_suction_checked=False)

    def point(self, name):
        return action_point(self.service.config, name, self.service.memory.mode)

    def wait_job(self, job):
        while job.status().get('active'):
            if self.cancelled.wait(.05):
                job.stop()
                self._check()
        result = job.status()
        if result.get('stop_unconfirmed') or (not self.service.armed and not self.cancelled.is_set()):
            self.stop_unconfirmed = True
        if result.get('phase') != 'completed':
            raise AgentError('ACTION_FAILED', result.get('message', '动作失败'), 500)
        self._check()
        return result

    def _gripper_ready(self):
        g = self.service.robot.gripper_status()
        if g.get('activation_state') != 3 or g.get('fault_code'):
            raise AgentError('GRIPPER_NOT_READY', '请先调用 /pose/health 激活右夹爪')
        if g.get('going_to_position') and g.get('object_state') == 0:
            raise AgentError('GRIPPER_BUSY', '右夹爪仍在运动', 409)

    def prepare(self, payload):
        kind, level = payload.get('pose_type'), payload.get('level')
        if kind == 'AGV_item_barcode_scan':
            if level is not None:
                raise AgentError('INVALID_LEVEL', '扫码姿态不接受 level')
            record = load_barcode_pose(self.service.config)
            saved = record['state']
            start = self.service.robot.read_memory_state()
            move, _ = prepare_trunk(self.service, saved, self.cancelled, start)
            self.service.audit_event('barcode_trunk_prepare', calibration=record, start=start,
                                     target=saved['poses']['trunk'], fixed_modules=['left_arm', 'right_arm', 'head'])
            reached = execute_trunk(self.service, move, start)
            self.service.audit_event('barcode_trunk_completed', state=reached)
            return {'frame': record['frame'], 'trunk_pose_mm_deg': saved['poses']['trunk'],
                    'moved_modules': ['trunk']}
        levels = {'AGV_carton_item_inspect': ('L1','L2','L3','L4','L5'),
                  'basket_item_place_prepare': ('L1','L2','L3','L4'),
                  'basket_push': ('L1','L2','L3','L4')}
        if kind not in levels:
            raise AgentError('NOT_IMPLEMENTED', '该姿态暂不开发，不执行运动', 501)
        if level not in levels[kind]:
            raise AgentError('INVALID_LEVEL', '无效层号')
        point = self.point('L2观察' if kind == 'AGV_carton_item_inspect' else 'L2抓取')
        self.service.memory.execute({}, prepared_point=point)
        self.wait_job(self.service.memory)
        return {'memory_point': point['name']}

    def _sorting(self, payload, *, pick):
        if 'sku_id' in payload or 'class_name' in payload:
            raise AgentError('INVALID_CATEGORY', '旧商品字段已停用，请使用 sku_typ=bottle/box/tube')
        kind = payload.get('sku_typ')
        hand = 'LEFT' if kind == 'box' else 'RIGHT'
        if (payload.get('task_type') != 'SORTING' or payload.get('target_type') != 'sku'
                or kind not in ('bottle', 'box', 'tube') or payload.get('hand') != hand):
            raise AgentError('NOT_IMPLEMENTED', '仅支持 SORTING / sku，bottle/RIGHT、tube/RIGHT 或 box/LEFT', 501)
        if pick and payload.get('level') != 'L2':
            raise AgentError('LEVEL_NOT_CONFIGURED', '当前仅配置 L2 抓取；其他层直接报错')
        if not pick and payload.get('destination_type') != 'basket':
            raise AgentError('NOT_IMPLEMENTED', '仅支持放入篮筐', 501)
        result = payload.get('localization_result')
        if not isinstance(result, dict):
            raise AgentError('LOCALIZATION_REQUIRED', 'localization_result 必须包含完整定位结果，六维 pose 不足以规划')
        return result

    def _close_pick_suction(self):
        """Close once before box planning, under this Agent action's reservation."""
        from .suction import validate_config
        config = copy.deepcopy(validate_config(self.service.config.get('suction')))
        with self.service._lock:
            self._check()
            self.service._suction_result = {'commanded_open': None, 'confirmed': False}
            try:
                result = self.service.robot.suction_set(False, config)
                if result.get('confirmed') is not True or result.get('commanded_open') is not False:
                    raise BackendError('吸盘关闭未确认，抓取规划及运动已取消')
            except Exception as exc:
                self.service._suction_result['error'] = str(exc)
                self.service.audit_event('suction_command_failed', requested_open=False,
                                         stage='before_pick_planning', error=str(exc))
                raise
            self.service._suction_result = result
            self.service.audit_event('suction_command', stage='before_pick_planning', **result)
        self._check()
        return config

    def pick(self, payload):
        response = self._sorting(payload, pick=True)
        kind = payload['sku_typ']
        prepared_suction_config = None
        if kind in ('bottle', 'tube'):
            self._gripper_ready()
            if kind == 'tube':
                ensure_gripper_position(self.service, self.cancelled, self._check, 130)
            else:
                ensure_gripper_open(self.service, self.cancelled, self._check)
        else:
            prepared_suction_config = self._close_pick_suction()
        saved = self.point('L2抓取')['state']
        start = self.service.robot.read_memory_state()
        frozen = self.geometry.freeze(response, start, kind)
        move, predicted, result = preflight_pick(self.service, frozen, saved, self.geometry, self.cancelled)
        preplanned_advance = result['preplanned_advance_mm']
        self._check()
        actual = execute_trunk(self.service, move, move.start)
        result = self.geometry.project(frozen, actual)
        result['preplanned_advance_mm'] = preplanned_advance
        self._check()
        self.service.grasp_test.execute({'source_result_id': result['source_result_id'], 'sku_typ': kind},
                                        prepared_result=result, require_gripper=kind in ('bottle', 'tube'),
                                        prepared_suction_config=prepared_suction_config)
        job = self.wait_job(self.service.grasp_test)
        state = self.service.robot.read_memory_state()
        # Record the most recent pick, without changing the fixed scan calibration.
        temporary = self.directory / 'last_pick_trunk.tmp'
        temporary.write_text(json.dumps(state, ensure_ascii=False), encoding='utf8')
        temporary.replace(self.directory / 'last_pick_trunk.json')
        return dict(box_clearance=result['box_clearance'], completed_moves=job['completed_moves'])

    def place(self, payload):
        response = self._sorting(payload, pick=False)
        kind = payload['sku_typ']
        if kind in ('bottle', 'tube'):
            self._gripper_ready()
        state = self.service.robot.read_memory_state()
        point = self.geometry.basket(response, state, 'left_arm' if kind == 'box' else 'right_arm')
        self.service.placement.execute({'source_result_id': str(response.get('request_id') or 'agent-basket'), 'sku_typ': kind},
                                       prepared_reference=point, preflight_all=True)
        self.wait_job(self.service.placement)
        return {}

    def rotate(self, payload):
        kind = payload.get('sku_typ', 'bottle')
        hand = 'LEFT' if kind == 'box' else 'RIGHT'
        if kind not in ('bottle', 'box', 'tube') or payload.get('hand') != hand or set(payload) - {'hand', 'sku_typ'}:
            raise AgentError('NOT_IMPLEMENTED', '扫码支持 bottle/RIGHT、tube/RIGHT 或 box/LEFT', 501)
        if kind in ('bottle', 'tube'):
            self._gripper_ready()
        self.service.scan_sequence.execute({'sku_typ': kind})
        result = self.wait_job(self.service.scan_sequence)
        photos = result.get('photos', [])
        paths = [p.get('image_path', p['path']) for p in photos]
        count = 1 if kind == 'box' else 2 if kind == 'tube' else 5
        if len(photos) != count or any(not Path(p).is_file() for p in paths):
            raise AgentError('IMAGES_INCOMPLETE', f'扫码流程未生成{count}张有效照片', 500)
        return dict(image_paths=paths, camera='right_wrist' if kind == 'box' else 'left_wrist')

    def run(self, route, payload, key):
        s = self.service
        reserved = False
        prior = (False, False)
        with s._lock:
            cached = self.ledger.begin(key, route, payload)
            if cached is not None:
                return cached
            try:
                self.ensure_idle()
                self.active = dict(route=route, key=key)
                self.cancelled.clear()
                prior = s.armed, s.gripper_unlocked
                s.armed, s.gripper_unlocked = True, True
                reserved = True
            except Exception as exc:
                response = self.error(exc)
                self.ledger.finish(key, response)
                return response
        s.audit_event('agent_action_start', route=route, key=key, input=payload,
                      speed_mm_s=s.speed_mm_s, rotation_deg_s=s.rotation_deg_s)
        try:
            methods = {'/pose/prepare': self.prepare, '/manipulation/pick': self.pick,
                       '/manipulation/rotate': self.rotate, '/manipulation/place': self.place}
            if route not in methods:
                raise AgentError('NOT_IMPLEMENTED', '该接口暂不开发，不执行运动', 501)
            response = [200, dict(status='SUCCEEDED', **methods[route](payload))]
        except Exception as exc:
            response = self.error(exc)
        finally:
            with s._lock:
                if reserved:
                    # A web stop/lock must never be undone by restoring old flags.
                    s.armed, s.gripper_unlocked = ((prior[0] and s.armed, prior[1] and s.gripper_unlocked)
                        if not self.cancelled.is_set() else (False, False))
                    self.active = None
        s.audit_event('agent_action_finished', route=route, key=key, response=response)
        self.ledger.finish(key, response)
        return response

    @staticmethod
    def error(exc):
        return [getattr(exc, 'status', 400 if isinstance(exc, (BackendError, ValueError)) else 500),
                dict(status='ERROR', error_code=getattr(exc, 'code', 'ACTION_FAILED'), message=str(exc))]
