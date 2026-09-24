"""Read-only preflight of bottle/box/tube placement motions, before dispatch."""
import copy
import math
import time

from .arm_movel import HardwareMoveL
from .backends import BackendError, JOINT_COUNTS
from .memory_motion import prepare, tool_signature
from .memory_points import vector
from .pose_frames import world_pose_to_ref

ARM = 'right_arm'


class PlanningRobot:
    """Expose only geometry queries; Cartesian starts come from predicted state."""
    def __init__(self, view, name):
        self.view, self.name = view, name

    def toolset(self, ec):
        tool = self.view.actual._robot(self.name).toolset(ec)
        if not ec.get('ec') and tool_signature(tool) != self.view.state['toolsets'][self.name]:
            raise BackendError('放置预规划期间工具或参考系改变')
        return tool

    def cartPosture(self, coordinate, ec):
        view, name = self.view, self.name
        tool = view._call('读取预规划工具', self.toolset)
        pose = view._pose_to_sdk(view.state['poses'][name])
        if name != 'trunk':
            pose = world_pose_to_ref(pose, view._world_from_ref(tool))
        cart = view._load_sdk().CartesianPosition(pose)
        if name == 'trunk':
            cart.confData = list(view.trunk_conf)
            cart.external = list(view.trunk_external)
        else:
            cart.confData = list(view.state['arm_conf_data'][name])
            cart.elbow = math.radians(view.state['arm_elbow_deg'][name])
            cart.hasElbow = True
        return cart

    def model(self):
        return self.view.actual._robot(self.name).model()

    def checkPath(self, *args):
        return self.view.actual._robot(self.name).checkPath(*args)


class PlanningBackend:
    """Private state, real read-only SDK math, no motion methods."""
    def __init__(self, actual, start):
        self.actual, self.state = actual, copy.deepcopy(start)
        self._lock = actual._lock
        self.audit_callback = getattr(actual, 'audit_callback', None)
        with self._lock:
            current = actual._call('读取放置预规划躯干构型', actual._robot('trunk').cartPosture,
                                   actual._load_sdk().CoordinateType.endInRef)
        self.trunk_conf, self.trunk_external = list(current.confData), list(current.external)
        if (len(self.trunk_external) < 2 or max(abs(math.degrees(a)-b) for a, b in
                zip(self.trunk_external[:2], start['joints_deg']['head'])) > .1):
            raise BackendError('放置预规划躯干外部轴与头部不一致')

    def __getattr__(self, name):
        if name in ('_load_sdk', '_pose_to_sdk', '_world_from_ref', '_call', '_check_ec', 'soft_limit_status'):
            return getattr(self.actual, name)
        raise AttributeError('只读预规划不提供操作：' + name)

    def _robot(self, name):
        return PlanningRobot(self, name)

    def read_state(self):
        return copy.deepcopy(self.state)


class BottlePreflight:
    arm = ARM

    def __init__(self, job, start, plane, speeds):
        self.job, self.service = job, job.service
        self.view = PlanningBackend(self.service.robot, start)
        self.plane, self.speeds = plane, speeds
        self.steps = []
        self.stage = 'memory'

    def record(self, **values):
        self.job._check_cancel()
        self.steps.append(dict(stage=self.stage, **values))
        self.service.audit_event('placement_preflight_stage', **self.steps[-1])

    def memory(self, saved):
        start = self.view.read_state()
        target = copy.deepcopy(start)
        for key in ('joints_deg', 'poses', 'arm_elbow_deg', 'arm_conf_data', 'toolsets'):
            if ARM in saved.get(key, {}):
                target.setdefault(key, {})[ARM] = copy.deepcopy(saved[key][ARM])
        self.job._limits(target, start, (ARM,))
        plan = prepare(self.view, target, self.speeds, self.job.cancelled)
        if set(plan['commands']) - {ARM}:
            raise BackendError('放置预规划首尾阶段只能包含右臂')
        self.record(modes=plan['modes'], target=target['poses'][ARM], joints_deg=target['joints_deg'][ARM])
        self.view.state = target
        return self.view.read_state()

    def linear(self, pose, seed=None, returning=None):
        arm = self.arm
        move = HardwareMoveL(self.view, arm, self.service.config, self.job.cancelled)
        result = move.plan(pose, move.start['arm_elbow_deg'][arm] if seed is None else seed, self.plane)
        step = move.steps[0]
        if returning and max(abs(a-b) for a,b in zip(step['q'], returning['joints_deg'][arm])) > .5:
            raise BackendError('同步回退的手臂解与前进起点不一致，禁止换构型回退')
        state = self.view.state
        state['joints_deg'][arm] = list(step['q'])
        state['poses'][arm] = list(step['pose'])
        state['arm_elbow_deg'][arm] = step['angle']
        state['arm_conf_data'][arm] = list(step['cart'].confData)
        self.record(module=arm, target=pose, joints_deg=step['q'], arm_plan=result)
        return self.view.read_state()

    def trunk(self, pose, returning=None):
        self.job._check_cancel()
        start = self.view.read_state()
        robot = self.view._robot('trunk')
        sdk = self.view._load_sdk()
        with self.view._lock:
            current = self.view._call('读取放置预规划躯干起点', robot.cartPosture, sdk.CoordinateType.endInRef)
            tool = self.view._call('读取放置预规划躯干工具', robot.toolset)
            target = sdk.CartesianPosition(self.view._pose_to_sdk(pose))
            target.confData, target.external = list(current.confData), list(current.external)
            q = self.view._call('放置预规划躯干终点逆解', robot.model().calcIk, target, tool)
        q = vector([math.degrees(v) for v in list(q)[:4]], 4, '躯干逆解')
        predicted = copy.deepcopy(start)
        predicted['joints_deg']['trunk'], predicted['poses']['trunk'] = q, list(pose)
        self.job._limits(predicted, start, ('trunk',))
        if returning and max(abs(a-b) for a,b in zip(q, returning['joints_deg']['trunk'])) > .5:
            raise BackendError('躯干回退逆解与前进起点不一致')
        self.record(module='trunk', target=pose, joints_deg=q, ik_calls=1)
        self.view.state = predicted
        return self.view.read_state()

    def joint7(self, value):
        return self.joint(6, value)

    def joint(self, index, value):
        start = self.view.read_state()
        target = copy.deepcopy(start)
        target['joints_deg'][ARM][index] = value
        self.job._limits(target, start, (ARM,))
        # No Cartesian motion starts between the joint adjustment and its inverse.
        # These two joint segments only require their joint targets and limits.
        self.record(module=ARM, motion='MoveAbsJ', joints_deg=target['joints_deg'][ARM])
        self.view.state = target

    def run(self, saved, origin, reference):
        state = self.memory(saved)
        self.stage = 'align_y'
        y = list(state['poses'][ARM]); y[1] = reference[1] - 30
        aligned = self.linear(y)
        self.stage = 'advance'
        arm = list(aligned['poses'][ARM]); arm[0] = reference[0] - 350
        trunk = list(aligned['poses']['trunk']); trunk[0] += 200
        self.linear(arm)
        # Match the paired-dispatch gate, which checks the following J7 target
        # against the state before both controllers advance.
        joint7_target = self.view.read_state()
        joint7_target['joints_deg'][ARM][6] -= 30
        self.job._limits(joint7_target, aligned, (ARM,))
        advanced = self.trunk(trunk)
        self.stage = 'joint7'
        self.joint7(advanced['joints_deg'][ARM][6] - 30)
        self.stage = 'joint7_return'
        self.joint7(advanced['joints_deg'][ARM][6])
        self.stage = 'retreat'
        self.linear(aligned['poses'][ARM], seed=aligned['arm_elbow_deg'][ARM], returning=aligned)
        self.trunk(aligned['poses']['trunk'], returning=aligned)
        self.stage = 'unalign_y'
        self.linear(state['poses'][ARM])
        self.stage = 'return_start'
        self.memory(origin)
        return self.steps


class BoxPreflight(BottlePreflight):
    arm = 'left_arm'
    kind = 'box'

    def seed(self, points):
        return points[0]['state']['arm_elbow_deg']['left_arm'] * (-1 if self.kind == 'tube' else 1)

    def run(self, points, pose):
        from .placement_sequence import BOX_LOWER_MM, BOX_ADVANCE_MM
        self.stage = f'{self.kind}_preplacement'
        aligned = self.linear(pose, seed=self.seed(points))
        lowered = list(pose); lowered[2] -= BOX_LOWER_MM
        trunk = list(aligned['poses']['trunk']); trunk[0] += BOX_ADVANCE_MM
        self.stage = f'{self.kind}_advance'
        self.trunk(trunk)
        # The arm starts only after the trunk arrives. Use that predicted body
        # state for its checkPath and chest-frame elbow guard.
        self.stage = f'{self.kind}_lower'
        placed = self.linear(lowered)
        # Same memory preparation as runtime: both arms first, then head/trunk.
        # Body return is joint replay; validate every target joint, no extra IK.
        self.stage = f'{self.kind}_return_arms'
        saved = copy.deepcopy(points[2]['state'])
        self.job._limits(saved, placed, JOINT_COUNTS)
        plan = prepare(self.view, saved, self.speeds, self.job.cancelled)
        self.record(modes=plan['modes'], joints_deg={m:saved['joints_deg'][m] for m in ('left_arm','right_arm')})
        self.stage = f'{self.kind}_return_body'
        self.record(motion='MoveAbsJ', joints_deg={m:saved['joints_deg'][m] for m in ('head','trunk')})
        return self.steps


class TubePreflight(BoxPreflight):
    arm = ARM
    kind = 'tube'

    def run(self, points, pose):
        from .placement_sequence import BOX_ADVANCE_MM
        self.stage = 'tube_preplacement'
        aligned = self.linear(pose, seed=self.seed(points))
        trunk = list(aligned['poses']['trunk']); trunk[0] += BOX_ADVANCE_MM
        self.stage = 'tube_advance'
        advanced = self.trunk(trunk)
        self.stage = 'tube_joint6'
        self.joint(5, advanced['joints_deg'][ARM][5] - 30)
        self.stage = 'tube_joint6_return'
        self.joint(5, advanced['joints_deg'][ARM][5])
        # All seven joints return exactly to the predicted pre-swing solution;
        # its Cartesian pose is therefore also the start of the memory return.
        self.view.state = copy.deepcopy(advanced)
        self.stage = 'tube_return_arms'
        saved = copy.deepcopy(points[2]['state'])
        self.job._limits(saved, advanced, JOINT_COUNTS)
        plan = prepare(self.view, saved, self.speeds, self.job.cancelled)
        self.record(modes=plan['modes'], joints_deg={m:saved['joints_deg'][m] for m in ('left_arm','right_arm')})
        self.stage = 'tube_return_body'
        self.record(motion='MoveAbsJ', joints_deg={m:saved['joints_deg'][m] for m in ('head','trunk')})
        return self.steps



def preflight_tube(job, points, start, pose, plane, speeds):
    return preflight_box(job, points, start, pose, plane, speeds, kind='tube')


def preflight_box(job, points, start, pose, plane, speeds, *, kind='box'):
    started = time.perf_counter()
    if not job.service.hardware_enabled:
        job.service.audit_event('placement_preflight_mock', sku_typ=kind, native_planning=False)
        return
    planner = None
    try:
        planner = (TubePreflight if kind == 'tube' else BoxPreflight)(job, start, plane, speeds)
        steps = planner.run(points, pose)
        job._check_cancel()
        job.service.audit_event('placement_preflight_passed', sku_typ=kind, steps=steps, speed=speeds,
                                duration_ms=(time.perf_counter()-started)*1000)
    except Exception as exc:
        stage = planner.stage if planner else 'initial_state'
        job.service.audit_event('placement_preflight_failed', sku_typ=kind, stage=stage, error=str(exc),
                                motion_started=False, duration_ms=(time.perf_counter()-started)*1000)
        label = '软管' if kind == 'tube' else '盒子'
        raise BackendError(f'{label}放置全流程预规划在 {stage} 阶段失败，整套动作未启动：{exc}') from exc


def preflight_bottle(job, saved, start, reference, plane, speeds):
    started = time.perf_counter()
    if not job.service.hardware_enabled:
        job.service.audit_event('placement_preflight_mock', native_planning=False)
        return
    planner = None
    try:
        planner = BottlePreflight(job, start, plane, speeds)
        steps = planner.run(saved, start, reference)
        job._check_cancel()
        job.service.audit_event('placement_preflight_passed', steps=steps, speed=speeds,
                                duration_ms=(time.perf_counter()-started)*1000)
    except Exception as exc:
        stage = planner.stage if planner else 'initial_state'
        job.service.audit_event('placement_preflight_failed', stage=stage, error=str(exc),
                                motion_started=False, duration_ms=(time.perf_counter()-started)*1000)
        raise BackendError(f'放置全流程预规划在 {stage} 阶段失败，整套动作未启动：{exc}') from exc
