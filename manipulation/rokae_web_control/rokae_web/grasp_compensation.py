"""Read-only reach compensation and paired arm/PCB4 MoveL execution."""
import copy
import math
import time

import numpy as np

from .arm_movel import MotionPlanUnavailable
from .backends import BackendError, JOINT_COUNTS
from .control_trace import invoke
from .memory_motion import IK_UNREACHABLE, poses_match, tool_signature
from .memory_points import joint_error, require_idle, vector

ADVANCE_OPTIONS_MM = (50.0, 100.0, 150.0, 200.0)


def shifted(pose, rotation, x_mm):
    """Translate along PCB4 reference X, expressed in the moving shoulder frame."""
    rotation = np.asarray(rotation, dtype=float)
    if (rotation.shape != (3, 3) or not np.isfinite(rotation).all()
            or not np.allclose(rotation.T @ rotation, np.eye(3), atol=1e-6)
            or not np.isclose(np.linalg.det(rotation), 1, atol=1e-6)):
        raise BackendError('缺少有效的躯干 SDK 到肩部方向变换')
    return (np.asarray(vector(pose, 6, '抓取目标')[:3]) + rotation[:, 0] * x_mm).tolist() + list(pose[3:])


def shifted_targets(targets, rotation, advance):
    # After the paired retreat the shoulder target equals the original retreat
    # target: (grasp-A) - (d+extra-A) = grasp-(d+extra), even if negative.
    # The caller sets extra to 20 mm for the right arm, 30 mm for the left.
    return [(key, label, shifted(pose, rotation, -advance) if key in ('grasp', 'descend', 'lift') else list(pose))
            for key, label, pose in targets]


def retreat_trunk_pose(origin):
    """Undo only the approach advance; the final 100 mm is a separate move."""
    return list(origin)


def plan_trunk(backend, config, cancel, start, pose):
    """One PCB4 endpoint IK. No queuing, power commands, or controller start."""
    if cancel.is_set():
        raise BackendError('抓取已取消')
    pose = vector(pose, 6, '躯干补偿目标')
    with backend._lock:
        sdk, robot = backend._load_sdk(), backend._robot('trunk')
        tool = backend._call('读取抓取补偿躯干工具', robot.toolset)
        signature = tool_signature(tool)
        if signature != start['toolsets']['trunk']:
            raise BackendError('抓取补偿规划期间躯干工具或参考系改变')
        current = backend._call('读取抓取补偿躯干构型', robot.cartPosture, sdk.CoordinateType.endInRef)
        if (len(current.external) < 2 or max(abs(math.degrees(a)-b) for a,b in
                zip(current.external[:2], start['joints_deg']['head'])) > .1):
            raise BackendError('抓取补偿躯干外部轴与头部不一致')
        target = sdk.CartesianPosition(backend._pose_to_sdk(pose))
        target.confData, target.external = list(current.confData), list(current.external)
        ec = {}
        raw = invoke(getattr(backend, 'audit_callback', None), '抓取补偿躯干终点逆解',
                     robot.model().calcIk, (target, tool), ec)
        if ec.get('ec', 0) in IK_UNREACHABLE | {-32}:
            raise MotionPlanUnavailable(f"躯干补偿终点逆解无解 (ec={ec['ec']})")
        backend._check_ec('抓取补偿躯干终点逆解', ec)
        q = vector(np.degrees(list(raw)[:4]).tolist(), 4, '躯干补偿关节')
        limits = backend.soft_limit_status()['joint_limits_deg']['trunk']
    if cancel.is_set():
        raise BackendError('抓取已取消')
    if any(not low <= value <= high for value, (low, high) in zip(q, limits)):
        raise MotionPlanUnavailable('躯干补偿目标超出软限位')
    maximum = config['motion']['max_joint_step_deg']['trunk']
    if maximum is not None and max(abs(a-b) for a,b in zip(q, start['joints_deg']['trunk'])) > maximum:
        raise MotionPlanUnavailable('躯干补偿目标超过单次关节变化上限')
    return dict(cart=target, q=q, pose=pose, signature=signature, limits=copy.deepcopy(limits))


def plan_approach(arm, target, rotation, plane, *, preselected=None):
    """Try the ordinary approach, then the first feasible advance in fixed order."""
    audit = getattr(arm.backend, 'audit_callback', None)
    seed = arm.start['arm_elbow_deg'][arm.module]
    if preselected is not None and preselected not in (0.0, *ADVANCE_OPTIONS_MM):
        raise BackendError('内部抓取预规划的前移量无效')
    if not preselected:
        try:
            return arm.plan(target, seed, plane), 0.0, None
        except MotionPlanUnavailable as exc:
            if audit:
                audit('grasp_approach_unreachable', module=arm.module, error=str(exc))
            # An interface action must not invent a branch after full preflight.
            if preselected == 0.0:
                raise
    options = (preselected,) if preselected is not None else ADVANCE_OPTIONS_MM
    failures = []
    for advance in options:
        arm.check(planning=True)
        trunk_pose = list(arm.start['poses']['trunk'])
        trunk_pose[0] += advance
        arm_target = shifted(target, rotation, -advance)
        try:
            trunk = plan_trunk(arm.backend, arm.config, arm.cancel, arm.start, trunk_pose)
            result = arm.plan(arm_target, seed, plane)
        except MotionPlanUnavailable as exc:
            failures.append(dict(advance_mm=advance, error=str(exc)))
            if audit:
                audit('grasp_advance_candidate', module=arm.module, advance_mm=advance,
                      feasible=False, error=str(exc), arm_target=arm_target, trunk_target=trunk_pose)
            continue
        if audit:
            audit('grasp_advance_candidate', module=arm.module, advance_mm=advance, feasible=True,
                  arm_target=arm_target, trunk_target=trunk_pose, trunk_joints_deg=trunk['q'], arm_plan=result)
        return result, advance, trunk
    raise MotionPlanUnavailable('抓取前移补偿无解，未启动补偿运动：' + '; '.join(
        f"{item['advance_mm']:g}mm: {item['error']}" for item in failures))


class PairedGraspMove:
    """Queue both controllers, start together, and wait until BOTH arrive."""
    def __init__(self, arm, trunk):
        self.arm, self.trunk = arm, trunk
        self.backend = arm.backend
        self.start = copy.deepcopy(arm.start)
        self.modules = (arm.module, 'trunk')

    def start_move(self, speed, rotation):
        arm, backend = self.arm, self.backend
        with backend._lock:
            arm.check_fresh()
            sdk = backend._load_sdk()
            robot = backend._robot('trunk')
            if tool_signature(backend._call('回读同步抓取躯干工具', robot.toolset)) != self.trunk['signature']:
                raise BackendError('同步抓取躯干工具或参考系改变')
            commands = {arm.module: sdk.MoveLCommand(arm.steps[0]['cart'], float(speed), 0),
                        'trunk': sdk.MoveLCommand(self.trunk['cart'], float(speed), 0)}
            for module, command in commands.items():
                arm.check()
                target_robot = backend._robot(module)
                backend._prepare_motion(target_robot, module, speed)
                command.rotSpeed = math.radians(rotation)
                backend._call(f'{module} 下发抓取同步 MoveL', target_robot.moveAppend, [command], sdk.PyString())
            arm.check()
            if hasattr(backend, 'start_arms_synchronized'):
                result = backend.start_arms_synchronized(list(self.modules))
            else:
                from .synchronized_start import start
                result = start(backend, list(self.modules))
        if getattr(backend, 'audit_callback', None):
            backend.audit_callback('grasp_synchronized_dispatch', **result)

    def wait_move(self):
        deadline, stable, idle_since = time.monotonic()+180, 0, None
        observed = set()
        while time.monotonic() < deadline:
            self.arm.check()
            with self.backend._lock:
                state = self.arm._snapshot()
            if state.get('toolsets') != self.start.get('toolsets') or any(state.get('dragging', {}).values()):
                raise BackendError('同步抓取期间工具配置或拖拽状态改变')
            if joint_error(state, self.start, [m for m in JOINT_COUNTS if m not in self.modules]) > .1:
                raise BackendError('同步抓取期间另一手臂或头部移动')
            for module, limits in ((self.arm.module, self.arm.limits), ('trunk', self.trunk['limits'])):
                q = vector(state['joints_deg'][module], JOINT_COUNTS[module], module)
                if any(not low <= value <= high for value, (low, high) in zip(q, limits)):
                    raise BackendError(f'{module} 同步抓取超过软限位')
                maximum = self.arm.config['motion']['max_joint_step_deg'][module]
                if maximum is not None and joint_error(state, self.start, (module,)) > maximum:
                    raise BackendError(f'{module} 同步抓取超过单次关节变化上限')
            for name in ('left_arm', 'right_arm', 'trunk'):
                allowed = ('idle', 'moving') if name in self.modules else ('idle',)
                if str(state['operation_state'][name]).lower() not in allowed:
                    raise BackendError(f'{name} 同步抓取运行状态异常')
                if name in self.modules and name not in observed and (str(state['operation_state'][name]).lower() == 'moving'
                        or joint_error(state, self.start, (name,)) > .02):
                    observed.add(name)
                    if getattr(self.backend, 'audit_callback', None):
                        self.backend.audit_callback('motion_first_observed', module=name, state=state)
            idle = all(str(state['operation_state'][m]).lower() == 'idle' for m in self.modules)
            reached = (self.arm.reached(state, self.arm.steps[0])
                       and poses_match(state['poses']['trunk'], self.trunk['pose'])
                       and max(abs(a-b) for a,b in zip(state['joints_deg']['trunk'], self.trunk['q'])) <= .5)
            stable = stable+1 if idle and reached else 0
            if stable >= 2:
                return state
            idle_since = (idle_since or time.monotonic()) if idle and not reached else None
            if idle_since and time.monotonic()-idle_since > 3:
                raise BackendError('同步抓取已静止但手臂或躯干未到目标')
            self.arm.cancel.wait(.1)
        raise BackendError('同步抓取等待到位超时')

    def stop_and_verify(self):
        errors = []
        # Attempt BOTH stops even if one fails. Never release the held object.
        with self.backend._lock:
            robots = {name: self.backend._robot(name) for name in self.modules}
            for name, robot in robots.items():
                for method in ('stop', 'moveReset'):
                    try:
                        self.backend._call(f'{name} 抓取同步停止 {method}', getattr(robot, method))
                    except Exception as exc:
                        errors.append(str(exc))
        deadline = time.monotonic()+3
        while time.monotonic() < deadline:
            try:
                with self.backend._lock:
                    idle = all(self.backend._operation_name(robot, name).lower() == 'idle'
                               for name, robot in robots.items())
                if idle:
                    return errors
            except Exception as exc:
                return errors + [str(exc)]
            time.sleep(.1)
        return errors + ['无法确认手臂和躯干均已停止']
