"""Read-only full pick preflight. Dispatch is a separate explicit step."""
import copy
import math
import numpy as np
from .backends import BackendError, JOINT_COUNTS
from .arm_movel import HardwareMoveL, load_guard_plane
from .grasp_test import flange_targets, left_flange_targets, flange_to_tcp, RETREAT_MM
from .trunk_retreat import HardwareTrunkRetreat, cart_pose
from .memory_points import joint_error, require_idle, vector
from .memory_motion import poses_match
from .grasp_compensation import plan_approach, plan_trunk, retreat_trunk_pose, shifted_targets


def trunk_ik(move, cart):
    backend = move.backend
    with backend._lock:
        q = backend._call('Agent 躯干终点逆解', move.robot.model().calcIk, cart, move.toolset)
    q = np.degrees(list(q)[:4]).tolist()
    vector(q, 4, '躯干逆解')
    for value, (low, high) in zip(q, move.limits['trunk']):
        if not low <= value <= high:
            raise BackendError('躯干逆解超出软限位')
    maximum = move.config['motion']['max_joint_step_deg']['trunk']
    if maximum is not None and max(abs(a-b) for a,b in zip(q, move.start['joints_deg']['trunk'])) > maximum:
        raise BackendError('躯干目标超过单次关节变化限制')
    return q


def prepare_trunk(service, saved, cancel, start):
    require_idle(start)
    if saved['toolsets']['trunk'] != start['toolsets']['trunk']:
        raise BackendError('躯干记忆点的工具/参考系与当前不同')
    move = HardwareTrunkRetreat(service.robot, service.config, cancel, start, RETREAT_MM)
    goal = move.sdk.CartesianPosition(service.robot._pose_to_sdk(saved['poses']['trunk']))
    goal.confData, goal.external = list(move.current.confData), list(move.current.external)
    q = trunk_ik(move, goal)
    move.moves['trunk_retreat'] = dict(cart=goal, q=q, from_q=list(start['joints_deg']['trunk']))
    predicted = copy.deepcopy(start)
    predicted['joints_deg']['trunk'], predicted['poses']['trunk'] = q, cart_pose(goal)
    return move, predicted


def preflight_pick(service, frozen, saved, geometry, cancel):
    kind = frozen.get('sku_typ', 'bottle')
    module = 'left_arm' if kind == 'box' else 'right_arm'
    try:
        return _preflight_pick(service, frozen, saved, geometry, cancel, module)
    except Exception as exc:
        service.audit_event('agent_pick_preflight_failed', sku_typ=kind, module=module,
                            error=str(exc), motion_started=False)
        raise BackendError(f'抓取全流程预规划失败，整套动作未启动：{exc}') from exc


def _preflight_pick(service, frozen, saved, geometry, cancel, module):
    start = service.robot.read_memory_state()
    body, predicted = prepare_trunk(service, saved, cancel, start)
    projected = geometry.project(frozen, predicted)
    arm = HardwareMoveL(service.robot, module, service.config, cancel)
    if joint_error(start, arm.start, JOINT_COUNTS) > .1:
        raise BackendError('演算期间起点变化')
    arm.start = copy.deepcopy(predicted)
    plane = load_guard_plane(service.config)
    if frozen.get('sku_typ') == 'tube':
        from .tube_grasp import tube_targets, plan_targets
        targets = [(k,label,flange_to_tcp(p,arm.signature)) for k,label,p in tube_targets(projected['shoulder_grasp'])]
        rotation = projected['shoulder_grasp']['R_right_shoulder_from_trunk_ref']
        def check():
            if cancel.is_set():raise BackendError('软管接口预规划已停止')
        advance, _, plans = plan_targets(arm,predicted,targets,rotation,plane,None,check)
        projected['preplanned_advance_mm'] = advance
        service.audit_event('agent_pick_preflight',start=start,predicted=predicted,module=module,sku_typ='tube',
                            frozen=frozen,projected=projected,arm_plans=plans,advance_mm=advance,
                            retreat_pose=list(arm.start['poses']['trunk']))
        return body,predicted,projected
    plans = []
    make_targets = left_flange_targets if module == 'left_arm' else flange_targets
    targets = make_targets(projected['shoulder_grasp'])
    kind = 'box' if module == 'left_arm' else 'bottle'
    rotation = projected['shoulder_grasp'].get('R_left_shoulder_from_trunk_ref' if kind == 'box'
                                             else 'R_right_shoulder_from_trunk_ref')
    advance = 0.0
    for index, (stage, _, pose) in enumerate(targets):
        target = flange_to_tcp(pose, arm.signature)
        trunk_plan = None
        if stage == 'grasp':
            planned, advance, trunk_plan = plan_approach(arm, target, rotation, plane)
            if advance:
                targets[:] = shifted_targets(targets, rotation, advance)
                target = flange_to_tcp(targets[index][2], arm.signature)
        else:
            planned = arm.plan(target, arm.start['arm_elbow_deg'][module], plane)
            if stage == 'arm_retreat' and advance:
                trunk_plan = plan_trunk(service.robot, service.config, cancel, arm.start,
                                        retreat_trunk_pose(predicted['poses']['trunk']))
        if trunk_plan is not None:
            arm.start['joints_deg']['trunk'] = list(trunk_plan['q'])
            arm.start['poses']['trunk'] = list(trunk_plan['pose'])
        step = arm.steps[0]
        plans.append(dict(stage=stage, target=target, joints_deg=step['q'], trunk_plan=trunk_plan, **planned))
        arm.start['joints_deg'][module] = list(step['q'])
        arm.start['poses'][module] = list(step['pose'])
        arm.start['arm_elbow_deg'][module] = step['angle']
        arm.start['arm_conf_data'][module] = list(step['cart'].confData)
        arm.current, arm.conf = step['cart'], list(step['cart'].confData)
    # Both sides undo only A in the pair. Plan the separate final 100 mm after
    # all arm stages, including the bottle's clearance lift at the original X.
    retreat_pose = list(arm.start['poses']['trunk'])
    retreat_pose[0] -= RETREAT_MM
    retreat = body.sdk.CartesianPosition(service.robot._pose_to_sdk(retreat_pose))
    retreat.confData, retreat.external = list(body.current.confData), list(body.current.external)
    final_q = trunk_ik(body, retreat)
    projected['preplanned_advance_mm'] = advance
    service.audit_event('agent_pick_preflight', start=start, predicted=predicted, module=module,
                        frozen=frozen, projected=projected, arm_plans=plans,
                        retreat_pose=retreat_pose, retreat_joints_deg=final_q, advance_mm=advance)
    return body, predicted, projected


def execute_trunk(service, move, expected):
    attempted = False
    try:
        move.check()
        attempted = True
        move.start_move('trunk_retreat', expected, service.speed_mm_s, service.rotation_deg_s)
        return move.wait_move()
    except Exception:
        if attempted:
            try:
                errors = move.stop_and_verify()
            except Exception:
                errors = ['stop acknowledgement unavailable']
            if errors:
                service.agent_actions.stop_unconfirmed = True
        raise
