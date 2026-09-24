"""Bottle/box/tube basket placement; explicit stages, no automatic failure retreat."""
import copy
import json
import math
import re
import threading
import time

from .arm_movel import HardwareMoveL, load_guard_plane
from .backends import BackendError, JOINT_COUNTS
from .control_trace import fields, operation, worker_thread
from .memory_motion import poses_match
from .memory_points import ARMS, MemoryPoints, joint_error, require_idle, vector
from .action_poses import action_point

ARM = 'right_arm'
LEFT_ARM = 'left_arm'
BOX_Y_OFFSET_MM = 110.0
TUBE_Y_OFFSET_MM = 80.0
BOX_LOWER_MM = 100.0
BOX_ADVANCE_MM = 100.0


def read_basket_reference(estimator, source, arm=ARM):
    side = "left" if arm == LEFT_ARM else "right"
    label = "左肩" if arm == LEFT_ARM else "右肩"
    if not isinstance(source, str) or not re.fullmatch(r'\d{8}/pose_\d{9}_[a-f0-9]+', source):
        raise BackendError(f'请先在网页识别篮筐，取得有效{label}参考点')
    root = estimator.data_root.resolve()
    folder = (root / source).resolve()
    if root not in folder.parents:
        raise BackendError('篮筐结果路径无效')
    try:
        saved = json.loads((folder / 'pose_estimation_summary.json').read_text(encoding='utf-8'))
        request = json.loads((folder / 'pose_estimation_request.json').read_text(encoding='utf-8'))
    except (OSError, ValueError) as exc:
        raise BackendError('无法读取指定的篮筐结果') from exc
    loc = saved.get('localization') or {}
    if (saved.get('selected_target') != 'basket' or saved.get('usable') is not True
            or loc.get('target') != 'basket' or loc.get('valid') is not True
            or loc.get(f'{side}_shoulder_frame') != f'{arm}_sdk_world'):
        raise BackendError(f'指定结果不是有效篮筐{label}参考点，请重新识别篮筐')
    point = vector(loc.get(f'point_{side}_shoulder_mm'), 3, f'篮筐{label}参考点')
    captured = vector(request.get('upper_body_joints_deg'), 6, '篮筐拍摄姿态')
    return list(point), captured


def basket_reference(estimator, source, start, arm=ARM):
    point, captured = read_basket_reference(estimator, source, arm)
    if max(abs(a-b) for a,b in zip(captured[:4], start['joints_deg']['trunk'])) > .3:
        raise BackendError('躯干已偏离篮筐识别时的姿态，请重新获取篮筐位姿后放置')
    return list(point)


class PlacementSequence(MemoryPoints):
    def __init__(self, service):
        self.service = service
        self.store, self.mode = service.memory.store, service.memory.mode
        self.cancelled = threading.Event()
        self.thread = None
        self.poll_seconds, self.timeout_seconds = .1, 180.
        self.job = dict(active=False, phase='idle', message='放置待命')

    def status(self):
        with self.service._lock:
            return dict(version='placement-v7', **copy.deepcopy(self.job))

    def ensure_idle(self):
        if self.job.get('stop_unconfirmed') or self.job['active']:
            raise BackendError('放置正在执行或停止尚未确认，请先停止放置')

    def _check_cancel(self):
        if (self.cancelled.is_set() or not self.service.armed
                or (self.job['active'] and self.job.get('sku_typ', 'bottle') in ('bottle', 'tube')
                    and not self.service.gripper_unlocked)):
            raise BackendError('放置已停止；后续动作已取消')

    def _set(self, **values):
        with self.service._lock:
            self.job.update(values)
        self.service.audit_event('placement_progress', **values)

    def _fresh(self, expected, arm=ARM):
        self._check_cancel()
        current = self.service.robot.read_memory_state()
        require_idle(current)
        if (joint_error(current, expected, JOINT_COUNTS) > .3
                or any(not poses_match(current['poses'][m], expected['poses'][m]) for m in (arm, 'trunk'))
                or current.get('toolsets') != expected.get('toolsets')):
            raise BackendError('放置步骤之间位置或工具配置改变，已取消后续动作')
        return current

    @operation('placement')
    def execute(self, payload, *, prepared_reference=None, preflight_all=False):
        if (not isinstance(payload, dict) or 'source_result_id' not in payload
                or set(payload) - {'source_result_id', 'sku_typ'}):
            raise BackendError('放置需要网页上一次有效的篮筐识别结果')
        kind = payload.get('sku_typ', 'bottle')
        if kind not in ('bottle', 'box', 'tube'):
            raise BackendError('放置仅支持 bottle、box 或 tube')
        if kind in ('box', 'tube'):
            return self._execute_lowering(payload, prepared_reference=prepared_reference,
                                          preflight_all=preflight_all or kind == 'tube')
        plane = load_guard_plane(self.service.config)
        with self.service._lock:
            self.service._require_armed()
            if self.service._pose_estimation_lock.locked():
                raise BackendError('位姿估计尚未完成，请等待篮筐结果后放置')
            if not self.service.gripper_unlocked:
                raise BackendError('放置需要松开夹爪，请先解锁并初始化夹爪')
            self._gripper_ready()
            points = [action_point(self.service.config, 'L2放置1', self.mode)]
            start = self.service.robot.read_memory_state()
            require_idle(start)
            point = (vector(prepared_reference, 3, '篮筐右肩参考点') if prepared_reference is not None
                     else basket_reference(self.service.pose_estimator, payload['source_result_id'], start))
            self.service.chassis.stop()
            self.cancelled.clear()
            speed = dict(linear_mm_s=float(self.service.speed_mm_s), rotation_deg_s=float(self.service.rotation_deg_s))
            self.job = dict(active=True, phase='planning', stage='memory', message='准备右臂前往 L2放置1',
                            source_result_id=payload['source_result_id'], sku_typ='bottle', basket_right_shoulder_mm=point,
                            start_state=copy.deepcopy(start), speed=speed, completed_moves=0, total_moves=8, **fields())
            self.thread = worker_thread(target=self._run_place,
                args=(copy.deepcopy(points[0]['state']), start, point, plane, speed, preflight_all), name='placement', daemon=True)
            self.thread.start()
            return self.status()

    def _execute_lowering(self, payload, *, prepared_reference=None, preflight_all=False):
        from .suction import validate_config
        plane = load_guard_plane(self.service.config)
        kind = payload['sku_typ']
        arm = ARM if kind == 'tube' else LEFT_ARM
        label, side = ('软管', '右') if kind == 'tube' else ('盒子', '左')
        suction = copy.deepcopy(validate_config(self.service.config.get('suction'))) if kind == 'box' else None
        with self.service._lock:
            self.service._require_armed()
            if self.service._pose_estimation_lock.locked():
                raise BackendError('位姿估计尚未完成，请等待篮筐结果后放置')
            if kind == 'tube':
                if not self.service.gripper_unlocked:
                    raise BackendError('软管放置需要完全松开夹爪，请先解锁并初始化夹爪')
                self._gripper_ready()
            points = [action_point(self.service.config, name, self.mode) for name in
                      ('L2盒子预放置', 'L2盒子放置点', 'L2抓取')]
            start = self.service.robot.read_memory_state()
            require_idle(start)
            reference = (vector(prepared_reference, 3, f'篮筐{side}肩参考点') if prepared_reference is not None
                         else basket_reference(self.service.pose_estimator, payload['source_result_id'], start, arm))
            pre = points[0]['state']
            # The taught second pose documents the final two offsets; execution
            # now reaches them separately, trunk first and then the left arm.
            # Reject inconsistent calibration before any movement or release.
            expected_place = copy.deepcopy(pre)
            expected_place['poses'][LEFT_ARM][2] -= BOX_LOWER_MM
            expected_place['poses']['trunk'][0] += BOX_ADVANCE_MM
            for point in points:
                state = point['state']
                if state.get('pose_frames', {}).get(LEFT_ARM) != 'left_arm_sdk_world' or state.get('pose_frames', {}).get('trunk') != 'trunk_controller_ref':
                    raise BackendError(f'{label}放置标定坐标系不符')
                if kind == 'tube' and state.get('pose_frames', {}).get(ARM) != 'right_arm_sdk_world':
                    raise BackendError('软管放置标定右肩坐标系不符')
                if state.get('toolsets') != start.get('toolsets'):
                    raise BackendError(f'{label}放置标定工具/TCP 或参考系与当前配置不一致')
            if any(not poses_match(points[1]['state']['poses'][m], expected_place['poses'][m]) for m in (LEFT_ARM, 'trunk')):
                raise BackendError('L2盒子放置点与预放置的左臂下降100 / 躯干前进100标定不一致')
            pose = vector(pre['poses'][LEFT_ARM], 6, '盒子预放置位姿')
            seed = float(pre['arm_elbow_deg'][LEFT_ARM])
            if kind == 'tube':
                from .tube_placement import mirrored_preplacement
                pose = mirrored_preplacement(pose, pre['toolsets'], reference[1] - TUBE_Y_OFFSET_MM)
                seed = -seed  # Mirrored elbow is a search seed, not a joint command.
            else:
                pose[1] = reference[1] + BOX_Y_OFFSET_MM
            self.service.chassis.stop()
            self.cancelled.clear()
            speeds = dict(linear_mm_s=float(self.service.speed_mm_s), rotation_deg_s=float(self.service.rotation_deg_s))
            self.job = dict(active=True, phase='planning', stage=f'{kind}_preplacement', sku_typ=kind,
                            message=f'准备{side}臂前往{label}预放置点', source_result_id=payload['source_result_id'],
                            preplacement_pose=pose, preplacement_seed_deg=seed, speed=speeds,
                            **{f'basket_{"right" if kind == "tube" else "left"}_shoulder_mm':reference},
                            completed_moves=0, total_moves=6 if kind == 'tube' else 5, **fields())
            self.thread = worker_thread(target=self._run_lowering_place,
                args=(points, start, pose, reference, plane, speeds, suction, preflight_all, kind), name=f'{kind}-placement', daemon=True)
            self.thread.start()
            return self.status()

    def _release_box(self, expected, config):
        self._fresh(expected, LEFT_ARM)
        with self.service._lock:
            self._check_cancel()
            self.service._suction_result = {'commanded_open': None, 'confirmed': False}
            try:
                result = self.service.robot.suction_set(False, config)
                if result.get('confirmed') is not True or result.get('commanded_open') is not False:
                    raise BackendError('吸盘关闭未确认，取消返回 L2抓取')
            except Exception as exc:
                self.service._suction_result['error'] = str(exc)
                self.service.audit_event('suction_command_failed', requested_open=False, error=str(exc))
                raise
            self.service._suction_result = result
            self.service.audit_event('suction_command', **result)
        self._set(suction_result=result)
        self._check_cancel()

    def _return_lowering_l2(self, saved, expected, speeds, kind):
        arm = ARM if kind == 'tube' else LEFT_ARM
        released = '夹爪已完全张开' if kind == 'tube' else '吸盘已关闭'
        start = self._fresh(expected, arm)
        self._limits(saved, start, JOINT_COUNTS)
        plan = self.service.robot.prepare_memory_motion(copy.deepcopy(saved), speeds, self.cancelled)
        plan['synchronized'] = True
        self._fresh(start, arm)
        self._check_cancel()
        self._set(stage=f'{kind}_return_arms', phase='moving', message=f'{released}，双臂返回 L2抓取')
        self.attempted = True
        self.service.robot.start_memory_arms(plan, self.cancelled)
        self.service._request_motion_sampling(f'{kind}_placement_return_arms', 'upper_body')
        after_arms = self._wait(start, ARMS, {m:saved['joints_deg'][m] for m in ARMS},
                                {m:saved['poses'][m] for m in ARMS}, arm=arm)
        self._check_cancel()
        self._set(stage=f'{kind}_return_body', completed_moves=5 if kind == 'tube' else 4, message='双臂已到位，头部与躯干返回 L2抓取')
        self.service.robot.start_memory_head_trunk(plan, self.cancelled)
        self.service._request_motion_sampling(f'{kind}_placement_return_body', 'upper_body')
        return self._wait(after_arms, ('head','trunk'), {m:saved['joints_deg'][m] for m in ('head','trunk')},
                          {'trunk':saved['poses']['trunk']}, arm=arm)

    def _run_lowering_place(self, points, start, pose, reference, plane, speeds, suction, preflight_all=False, kind='box'):
        arm = ARM if kind == 'tube' else LEFT_ARM
        label, side = ('软管', '右') if kind == 'tube' else ('盒子', '左')
        seed = float(points[0]['state']['arm_elbow_deg'][LEFT_ARM]) * (-1 if kind == 'tube' else 1)
        self.attempted, self.gripper_attempted = False, False
        try:
            self.service.audit_event(f'{kind}_placement_start', start=start, points=points,
                                     reference=reference, preplacement_pose=pose, speed=speeds)
            if preflight_all:
                from .placement_planning import preflight_box, preflight_tube
                self._set(stage='preflight', phase='planning', message=f'提前演算{label}放置及返回 L2抓取的完整流程')
                (preflight_tube if kind == 'tube' else preflight_box)(self, points, start, pose, plane, speeds)
                self._fresh(start, arm)
            self._set(stage=f'{kind}_preplacement')
            aligned = self._linear(pose, start, plane, speeds, arm_module=arm,
                                   seed=seed)
            trunk_pose = list(aligned['poses']['trunk']); trunk_pose[0] += BOX_ADVANCE_MM
            self._set(stage=f'{kind}_advance', phase='moving', completed_moves=1,
                      message='躯干沿 SDK X+100 mm 前进，双臂关节保持不动', trunk_target=trunk_pose)
            advanced = self._trunk_linear(trunk_pose, aligned, speeds, arm_module=arm)
            if kind == 'tube':
                self._set(stage='tube_joint6', phase='moving', completed_moves=2,
                          message='保持水平朝前的预放置姿态，右臂 J6 从当前值减30°')
                placed = self._joint6(advanced['joints_deg'][ARM][5] - 30, advanced, speeds)
                self._set(stage='tube_release', phase='opening', completed_moves=3,
                          message='J6 已到位，完全张开夹爪释放软管')
                self._open(placed)
                self._set(stage='tube_joint6_return', phase='moving', message='恢复右臂 J6 原值')
                placed = self._joint6(advanced['joints_deg'][ARM][5], placed, speeds)
                if joint_error(placed, advanced, (ARM,)) > .5 or not poses_match(placed['poses'][ARM], advanced['poses'][ARM]):
                    raise BackendError('J6 恢复后右臂未回到下摆前位姿')
                self._set(completed_moves=4)
            else:
                lowered = list(pose); lowered[2] -= BOX_LOWER_MM
                self._set(stage='box_lower', phase='planning', completed_moves=2,
                          message='躯干已到位，左臂保护 MoveL 沿左肩 Z−100 mm 下降', arm_target=lowered)
                placed = self._linear(lowered, advanced, plane, speeds, arm_module=arm)
                self._set(stage='box_release', phase='opening', completed_moves=3, message='左臂已到放置点，关闭吸盘释放盒子')
                self._release_box(placed, suction)
            final = self._return_lowering_l2(points[2]['state'], placed, speeds, kind)
            self._check_cancel()
            self._set(active=False, phase='completed', stage='completed', completed_moves=6 if kind == 'tube' else 5,
                      message=f'{label}放置完成：' + ('夹爪保持完全张开' if kind == 'tube' else '吸盘保持关闭') + '，已返回 L2抓取', final_state=final)
        except Exception as exc:
            errors = self._stop_and_verify() if self.attempted else []
            if self.gripper_attempted:
                try:self.service.robot.gripper_stop()
                except Exception as err:errors.append(str(err))
            if errors:
                with self.service._lock:self.service.armed=False
            # Never release on failure/cancel or retry a suction command.
            self._set(active=False, phase='cancelled' if self.cancelled.is_set() and not errors else 'failed',
                      stop_unconfirmed=bool(errors), message=str(exc)+('；停止未确认：'+'; '.join(errors) if errors else ''))

    def _gripper_ready(self):
        g = self.service.robot.gripper_status()
        if g['fault_code'] or g['activation_state'] != 3:
            raise BackendError('夹爪未就绪或存在故障')
        if g['going_to_position'] and g['object_state'] == 0:
            raise BackendError('夹爪仍在运动')
        return g

    def _limits(self, state, start, moving):
        limits = self.service.robot.soft_limit_status()['joint_limits_deg'] if self.service.hardware_enabled else None
        for name in moving:
            q = vector(state['joints_deg'][name], JOINT_COUNTS[name], name)
            if limits and any(not lo <= value <= hi for value,(lo,hi) in zip(q, limits[name])):
                raise BackendError(f'{name} 超出软限位')
            maximum = self.service.config['motion']['max_joint_step_deg'][name]
            if maximum is not None and joint_error(state,start,(name,)) > float(maximum):
                raise BackendError(f'{name} 超过单次关节变化上限')

    def _wait(self, start, moving, joints, poses, elbow=None, arm=ARM):
        deadline, stable, idle_since = time.monotonic()+self.timeout_seconds, 0, None
        while time.monotonic() < deadline:
            self._check_cancel()
            state = self.service.robot.read_memory_state()
            if any(state.get('dragging',{}).values()) or state.get('toolsets') != start.get('toolsets'):
                raise BackendError('放置期间拖拽或工具配置改变')
            if joint_error(state,start,[m for m in JOINT_COUNTS if m not in moving]) > .3:
                raise BackendError('非本阶段关节发生变化，放置已停止')
            for name in ('left_arm', ARM, 'trunk'):
                allowed = ('idle','mock-idle','moving') if name in moving else ('idle','mock-idle')
                if str(state['operation_state'][name]).lower() not in allowed:
                    raise BackendError(f'{name} 运行状态异常')
            self._limits(state,start,moving)
            idle = all(str(state['operation_state']['trunk' if m == 'head' else m]).lower() in ('idle','mock-idle') for m in moving)
            reached = all(max(abs(a-b) for a,b in zip(state['joints_deg'][m],q)) <= .5 for m,q in joints.items())
            reached = reached and all(poses_match(state['poses'][m],p) for m,p in poses.items())
            if elbow is not None:
                reached = reached and abs(state['arm_elbow_deg'][arm]-elbow) <= .5
            stable = stable+1 if idle and reached else 0
            if stable >= 2:
                return state
            idle_since = (idle_since or time.monotonic()) if idle and not reached else None
            if idle_since and time.monotonic()-idle_since > 3:
                raise BackendError('控制器已静止但未到放置目标')
            self.cancelled.wait(self.poll_seconds)
        raise BackendError('放置到位等待超时')

    def _memory_right(self, saved, expected, speeds):
        start = self._fresh(expected)
        target = copy.deepcopy(start)
        for key in ('joints_deg','poses','arm_elbow_deg','arm_conf_data','toolsets'):
            if ARM in saved.get(key,{}):
                target.setdefault(key,{})[ARM] = copy.deepcopy(saved[key][ARM])
        self._limits(target,start,(ARM,))
        plan = self.service.robot.prepare_memory_motion(target,speeds,self.cancelled)
        if self.service.hardware_enabled and set(plan['commands']) - {ARM}:
            raise BackendError('放置首尾阶段只能发送右臂指令')
        self._check_cancel()
        self.attempted = True
        self.service.robot.start_memory_arms(plan,self.cancelled)
        return self._wait(start,(ARM,),{ARM:target['joints_deg'][ARM]}, {ARM:target['poses'][ARM]},target['arm_elbow_deg'][ARM])

    def _linear(self, pose, expected, plane, speeds, arm_module=ARM, seed=None):
        start = self._fresh(expected, arm_module)
        if self.service.hardware_enabled:
            move = HardwareMoveL(self.service.robot,arm_module,self.service.config,self.cancelled)
            move.plan(pose,start['arm_elbow_deg'][arm_module] if seed is None else seed,plane)
            self._check_cancel()
            self.attempted = True
            move.start_step(0,speeds['linear_mm_s'],speeds['rotation_deg_s'])
            move.wait_step(0)
        else:
            self.attempted = True
            self.service.robot.move_pose(arm_module,pose,speeds['linear_mm_s'])
        result = self.service.robot.read_memory_state()
        self._fresh(result, arm_module)
        return result

    def _trunk_linear(self, pose, expected, speeds, arm_module=LEFT_ARM):
        """Finish the trunk MoveL before any subsequent arm command is queued."""
        start = self._fresh(expected, arm_module)
        backend = self.service.robot
        if self.service.hardware_enabled:
            with backend._lock:
                sdk, trunk = backend._load_sdk(), backend._robot('trunk')
                current = backend._call('读取放置躯干起点', trunk.cartPosture, sdk.CoordinateType.endInRef)
                if (len(current.external) < 2 or max(abs(math.degrees(a)-b) for a,b in
                        zip(current.external[:2], start['joints_deg']['head'])) > .1):
                    raise BackendError('躯干外部轴与头部回读不一致')
                target = sdk.CartesianPosition(backend._pose_to_sdk(pose))
                target.confData, target.external = list(current.confData), list(current.external)
                self._fresh(start, arm_module)
                self._check_cancel()
                self.attempted = True
                backend._prepare_motion(trunk, 'trunk', speeds['linear_mm_s'])
                command = sdk.MoveLCommand(target, speeds['linear_mm_s'], 0)
                command.rotSpeed = math.radians(speeds['rotation_deg_s'])
                self._check_cancel()
                backend._call('下发放置躯干 MoveL', trunk.moveAppend, [command], sdk.PyString())
                self._check_cancel()
                backend._call('启动放置躯干 MoveL', trunk.moveStart)
        else:
            self._check_cancel()
            self.attempted = True
            backend.move_pose('trunk', pose, speeds['linear_mm_s'])
        self.service.audit_event('placement_trunk_dispatch', target=pose, speed=speeds, start_state=start)
        return self._wait(start, ('trunk',), {}, {'trunk':pose}, arm=arm_module)

    def _joint7(self, value, expected, speeds):
        return self._joint(6, value, expected, speeds)

    def _joint6(self, value, expected, speeds):
        return self._joint(5, value, expected, speeds)

    def _joint(self, index, value, expected, speeds):
        start = self._fresh(expected)
        target = copy.deepcopy(start)
        target['joints_deg'][ARM][index] = value
        self._limits(target,start,(ARM,))
        self._check_cancel()
        self.attempted = True
        self.service.robot.move_joints(ARM,target['joints_deg'][ARM],speeds['linear_mm_s'])
        return self._wait(start,(ARM,),{ARM:target['joints_deg'][ARM]}, {})

    def _pair(self, arm_pose, trunk_pose, expected, plane, speeds, returning=None, arm_module=ARM):
        start = self._fresh(expected, arm_module)
        backend = self.service.robot
        if not self.service.hardware_enabled:
            self.attempted = True
            with backend._lock:
                backend.move_pose(arm_module,arm_pose,speeds['linear_mm_s'])
                backend.move_pose('trunk',trunk_pose,speeds['linear_mm_s'])
            return self._wait(start,(arm_module,'trunk'),{}, {arm_module:arm_pose,'trunk':trunk_pose}, arm=arm_module)
        arm = HardwareMoveL(backend,arm_module,self.service.config,self.cancelled)
        seed = returning['arm_elbow_deg'][arm_module] if returning else start['arm_elbow_deg'][arm_module]
        arm.plan(arm_pose,seed,plane)
        q = arm.steps[0]['q']
        if arm_module == ARM and not returning:
            joint7_target = copy.deepcopy(start)
            joint7_target['joints_deg'][arm_module] = list(q)
            joint7_target['joints_deg'][arm_module][6] -= 30
            self._limits(joint7_target,start,(arm_module,))
        if returning and max(abs(a-b) for a,b in zip(q,returning['joints_deg'][arm_module])) > .5:
            raise BackendError('同步回退的右臂解与前进起点不一致，禁止换构型回退')
        with backend._lock:
            sdk, trunk = backend._load_sdk(), backend._robot('trunk')
            current = backend._call('读取放置躯干起点',trunk.cartPosture,sdk.CoordinateType.endInRef)
            if (len(current.external) < 2 or max(abs(math.degrees(a)-b) for a,b in
                    zip(current.external[:2],start['joints_deg']['head'])) > .1):
                raise BackendError('躯干外部轴与头部回读不一致')
            target = sdk.CartesianPosition(backend._pose_to_sdk(trunk_pose))
            target.confData, target.external = list(current.confData), list(current.external)
            arm.check_fresh()
            self._fresh(start, arm_module)
            commands = {arm_module:sdk.MoveLCommand(arm.steps[0]['cart'],speeds['linear_mm_s'],0),
                        'trunk':sdk.MoveLCommand(target,speeds['linear_mm_s'],0)}
            self._check_cancel()
            self.attempted = True
            # Queue both controllers before dispatching either; one shared barrier starts them.
            for module,command in commands.items():
                self._check_cancel()
                robot = backend._robot(module)
                backend._prepare_motion(robot,module,speeds['linear_mm_s'])
                command.rotSpeed = math.radians(speeds['rotation_deg_s'])
                backend._call(f'{module} 下发放置同步运动',robot.moveAppend,[command],sdk.PyString())
            self._check_cancel()
            if hasattr(backend,'start_arms_synchronized'):
                dispatch = backend.start_arms_synchronized([arm_module,'trunk'])
            else:
                from .synchronized_start import start as synchronized_start
                dispatch = synchronized_start(backend,[arm_module,'trunk'])
        self.service.audit_event('placement_synchronized_dispatch',**dispatch)
        joints = {arm_module:q}
        if returning:
            joints['trunk'] = returning['joints_deg']['trunk']
        return self._wait(start,(arm_module,'trunk'),joints,{arm_module:arm_pose,'trunk':trunk_pose},arm.steps[0]['angle'], arm=arm_module)

    def _open(self, expected):
        self._fresh(expected)
        if not self.service.gripper_unlocked:
            raise BackendError('夹爪已锁定，放置中止')
        self._gripper_ready()
        self._check_cancel()
        self.gripper_attempted = True
        self.service.robot.gripper_start_move(0)
        deadline = time.monotonic()+30
        while time.monotonic()<deadline:
            self._fresh(expected)
            g = self.service.robot.gripper_status()
            if g['fault_code'] or g['activation_state'] != 3:
                raise BackendError('夹爪张开受阻或故障，已取消回退')
            if g['requested_position']==0 and g['object_state'] in (1,2):
                raise BackendError('夹爪张开受阻，已取消回退')
            if g['requested_position']==0 and g['object_state']==3 and g['measured_position']<=5:
                self._set(gripper_result=g)
                return
            self.cancelled.wait(self.poll_seconds)
        raise BackendError('夹爪张开等待超时，已取消回退')

    def _run_place(self, saved, start, reference, plane, speeds, preflight_all=False):
        self.attempted, self.gripper_attempted = False, False
        try:
            self.service.audit_event('placement_start',start=start,memory=saved,reference=reference,speed=speeds)
            if preflight_all:
                from .placement_planning import preflight_bottle
                self._set(stage='preflight', phase='planning', message='先演算放置全流程，尚未启动任何运动')
                preflight_bottle(self, saved, start, reference, plane, speeds)
                self._fresh(start)
                self._set(stage='memory', phase='moving', message='全流程预规划通过，右臂前往 L2放置1')
            state = self._memory_right(saved,start,speeds)
            self._set(stage='align_y',phase='moving',completed_moves=1,message='右臂保护运动：Y=篮筐参考点 Y−30 mm')
            y_pose = list(state['poses'][ARM]); y_pose[1] = reference[1]-30
            aligned = self._linear(y_pose,state,plane,speeds)
            arm_pose = list(aligned['poses'][ARM]); arm_pose[0] = reference[0]-350
            trunk_pose = list(aligned['poses']['trunk']); trunk_pose[0] += 200
            self._set(stage='advance',completed_moves=2,message='躯干 X+200 mm 与右臂 X=参考点 X−350 mm 同步前进',
                      paired_start_state=copy.deepcopy(aligned),arm_target=arm_pose,trunk_target=trunk_pose)
            advanced = self._pair(arm_pose,trunk_pose,aligned,plane,speeds)
            self._set(stage='joint7',completed_moves=3,message='右臂 J7 在当前值基础上减 30°')
            lowered = self._joint7(advanced['joints_deg'][ARM][6]-30,advanced,speeds)
            self._set(stage='open',phase='opening',completed_moves=4,message='松开夹爪，等待张开完成')
            self._open(lowered)
            self._set(stage='joint7_return',phase='moving',message='恢复右臂 J7')
            restored = self._joint7(advanced['joints_deg'][ARM][6],lowered,speeds)
            if joint_error(restored,advanced,(ARM,))>.5 or not poses_match(restored['poses'][ARM],advanced['poses'][ARM]):
                raise BackendError('J7 恢复后右臂未回到前进终点')
            self._set(stage='retreat',completed_moves=5,message='躯干与右臂同步回退至前进起点')
            retreated = self._pair(aligned['poses'][ARM],aligned['poses']['trunk'],restored,plane,speeds,returning=aligned)
            self._set(stage='unalign_y',completed_moves=6,message='右臂保护运动：反向平移回 L2放置1')
            unaligned = self._linear(state['poses'][ARM],retreated,plane,speeds)
            self._set(stage='return_start',completed_moves=7,message='右臂返回点击放置前的位置')
            final = self._memory_right(start,unaligned,speeds)
            self._fresh(start)
            self._check_cancel()
            self._set(active=False,phase='completed',stage='completed',completed_moves=8,
                      message='放置完成：夹爪保持张开，右臂与躯干已返回开始位置',final_state=final)
        except Exception as exc:
            errors = self._stop_and_verify() if self.attempted else []
            if self.gripper_attempted:
                try:self.service.robot.gripper_stop()
                except Exception as err:errors.append(str(err))
            if errors:
                with self.service._lock:self.service.armed=False
            self._set(active=False,phase='cancelled' if self.cancelled.is_set() and not errors else 'failed',
                      stop_unconfirmed=bool(errors),message=str(exc)+('；停止未确认：'+'; '.join(errors) if errors else ''))

    def stop(self):
        self.cancelled.set()
        with self.service._lock:
            if self.job['active']:
                self.job.update(phase='stopping',message='正在停止放置；不会自动回退')
            elif self.job.get('stop_unconfirmed'):
                errors=self._stop_and_verify()
                if getattr(self,'gripper_attempted',False):
                    try:self.service.robot.gripper_stop()
                    except Exception as exc:errors.append(str(exc))
                self.job.update(stop_unconfirmed=bool(errors),message='；'.join(errors) if errors else '已确认停止')
            return self.status()
