"""Shared tube pick: tilt, close, recover/retreat, then paired 40/100."""
import copy
import time

import numpy as np

from .arm_movel import HardwareMoveL
from .backends import BackendError, JOINT_COUNTS
from .grasp_compensation import PairedGraspMove, plan_approach, plan_trunk, shifted
from .grasp_gripper import ensure_gripper_position
from .grasp_test import flange_targets, flange_to_tcp
from .head_kinematics import rpy_rotation, rotation_to_rpy_deg
from .memory_motion import poses_match
from .memory_points import joint_error, require_idle


STAGES = ['pregrasp', 'grasp', 'tilt_down', 'recover_retreat', 'final_retreat']


def tube_targets(shoulder):
    # Reuse the validated right-shoulder pregrasp/grasp and trunk rotation.
    base = {k:p for k,_,p in flange_targets(shoulder)}
    pre, grasp = base['pregrasp'], base['grasp']
    d = float(shoulder['box_clearance']['d_mm'])
    rotation = np.asarray(shoulder['R_right_shoulder_from_trunk_ref'])
    down = list(grasp[:3]) + rotation_to_rpy_deg(rotation @ rpy_rotation(np.radians([180., -60., 0.])))
    recover = shifted(grasp, rotation, -(d - 20.0))
    final = shifted(recover, rotation, -40.0)
    return [('pregrasp', '软管预抓取点', pre), ('grasp', '软管水平抓取点', grasp),
            ('tilt_down', '法兰原地下俯30°', down),
            ('recover_retreat', f'法兰回正并沿躯干X后退 {d-20:g} mm', recover),
            ('final_retreat', '右臂退40 mm与躯干退100 mm', final)]


def compensated_targets(targets, rotation, advance):
    # Recovery ends at its original shoulder target while the trunk returns A.
    return [(k,label,shifted(p,rotation,-advance) if k in ('grasp','tilt_down') else list(p))
            for k,label,p in targets]


def body_goal(origin, stage):
    pose = list(origin)
    if stage == 'final_retreat':
        pose[0] -= 100.0
    return pose


def preflight(job, start, targets, rotation, plane, seed, signature, preselected=None):
    """Simulate every arm/body stage before any arm or body dispatch."""
    arm = HardwareMoveL(job.service.robot, 'right_arm', job.service.config, job.cancel)
    if (joint_error(arm.start, start, JOINT_COUNTS) > .1
            or not poses_match(arm.start['poses']['right_arm'], start['poses']['right_arm'])
            or arm.start['toolsets'] != start['toolsets'] or arm.signature != signature):
        raise BackendError('软管预规划期间起点或工具改变')
    arm.start = copy.deepcopy(start)
    advance,goals,plans = plan_targets(arm,start,targets,rotation,plane,seed,job._check,preselected)
    job.service.audit_event('tube_full_preflight_passed',start=start,advance_mm=advance,plans=plans)
    return advance,goals


def plan_targets(arm, start, targets, rotation, plane, seed, check, preselected=None):
    """Plan from the caller's measured or predicted arm/body state; never dispatch."""
    goals = copy.deepcopy(targets)
    advance = 0.0
    plans = []
    for index,(stage,label,pose) in enumerate(goals):
        check()
        trunk = None
        angle = seed if index == 0 and seed is not None else arm.start['arm_elbow_deg']['right_arm']
        if stage == 'grasp':
            result,advance,trunk = plan_approach(arm,pose,rotation,plane,preselected=preselected)
            if advance:
                goals[:] = compensated_targets(goals,rotation,advance)
                pose = goals[index][2]
        else:
            result = arm.plan(pose,angle,plane)
            if stage == 'final_retreat' or (stage == 'recover_retreat' and advance):
                trunk = plan_trunk(arm.backend,arm.config,arm.cancel,arm.start,
                                   body_goal(start['poses']['trunk'],stage))
        if len(arm.steps) != 1:
            raise BackendError('软管每段必须仅规划一条MoveL')
        step = arm.steps[0]
        plans.append(dict(stage=stage,pose=pose,start=copy.deepcopy(arm.start),result=result,trunk_plan=trunk))
        arm.start['joints_deg']['right_arm'] = list(step['q'])
        arm.start['poses']['right_arm'] = list(step['pose'])
        arm.start['arm_elbow_deg']['right_arm'] = step['angle']
        arm.start.setdefault('arm_conf_data', {})['right_arm'] = list(step['cart'].confData)
        arm.current,arm.conf = step['cart'],list(step['cart'].confData)
        if trunk:
            arm.start['joints_deg']['trunk'] = list(trunk['q'])
            arm.start['poses']['trunk'] = list(trunk['pose'])
    return advance,goals,plans


def run_tube(job, source, seed, plane, speed, rotation_speed, prepared_result=None):
    moving = None
    closing = False
    service = job.service
    try:
        job._check()
        job._set(phase='opening',stage='open_gripper',message='确认右夹爪预开到130后演算软管流程',
                 protected_arm_stages=STAGES,total_moves=5,lift_mm=0,clearance_lift_mm=0,
                 final_arm_retreat_mm=40,final_trunk_retreat_mm=100,tilt_down_deg=30,gripper_preopen_position=130)
        opened = job._measure('gripper_ready',ensure_gripper_position,service,job.cancel,job._check,130)
        if service.hardware_enabled:
            initial = job._measure('initial_snapshot',HardwareMoveL,service.robot,'right_arm',service.config,job.cancel)
            start = copy.deepcopy(initial.start)
            signature = copy.deepcopy(initial.signature)
        else:
            start = service.robot.read_state()
            signature = {'end':[0.0]*6,'ref':[0.0]*6}
        require_idle(start)
        if start.get('pose_frames',{}).get('right_arm') != 'right_arm_sdk_world':
            raise BackendError('起始右臂位姿坐标系不一致')
        job.service.audit_event('grasp_test_start',sku_typ='tube',module='right_arm',state=start,
                                toolset=signature,gripper=opened,plane=plane,
                                speed_mm_s=speed,rotation_deg_s=rotation_speed)
        result = copy.deepcopy(prepared_result) if prepared_result is not None else job._measure(
            'reproject_pose',service.pose_estimator.reproject_latest_tube_to_right_shoulder,start,start)
        preselected = result.get('preplanned_advance_mm') if prepared_result is not None else None
        if prepared_result is not None:
            from .memory_points import vector
            projected_body = vector(result.get('current_trunk_joints_deg'), 4, '软管冻结目标的躯干关节')
            if max(abs(a-b) for a,b in zip(projected_body,start['joints_deg']['trunk'])) > .1:
                raise BackendError('软管冻结目标投影后躯干状态改变')
        if result['source_result_id'] != source or result.get('sku_typ') != 'tube':
            raise BackendError('软管定位类别或结果已更新，请重新识别')
        service.audit_event('grasp_test_pose_conversion',result=result)
        shoulder = result['shoulder_grasp']
        rotation = shoulder['R_right_shoulder_from_trunk_ref']
        flange = tube_targets(shoulder)
        targets = [(k,label,flange_to_tcp(p,signature)) for k,label,p in flange]
        d = float(shoulder['box_clearance']['d_mm'])
        job._set(phase='planning',stage='full_preflight',message='演算软管全部动作及同步后退',
                 start_pose_mm_deg=start['poses']['right_arm'],start_arm_angle_deg=start['arm_elbow_deg']['right_arm'],
                 box_clearance=shoulder['box_clearance'],shoulder_grasp=shoulder,
                 flange_height_trunk_mm=result['world_grasp']['grasp_height_trunk_mm'],
                 recovery_retreat_mm=d-20,gripper_present=True,gripper_control_enabled=True)
        if service.hardware_enabled:
            advance,targets = job._measure('tube_full_preflight',preflight,job,start,targets,rotation,plane,seed,signature,preselected)
        else:
            advance = 0.0
        flange = compensated_targets(flange,rotation,advance)
        job._set(advance_mm=advance,arm_retreat_mm=d-20-advance,paired_trunk_retreat_mm=advance,
                 targets_flange_mm_deg={k:p for k,_,p in flange},targets_tcp_mm_deg={k:p for k,_,p in targets})
        expected = copy.deepcopy(start)
        body_baseline = copy.deepcopy(start)
        for index,(stage,label,pose) in enumerate(targets):
            job._check()
            job._set(phase='planning',stage=stage,message='演算'+label,protection_scope='endpoint_only')
            pair = None
            if service.hardware_enabled:
                arm = HardwareMoveL(service.robot,'right_arm',service.config,job.cancel)
                job._fixed_body(arm.start,body_baseline)
                if (arm.signature != signature or joint_error(arm.start,expected,('right_arm',))>.5
                        or not poses_match(arm.start['poses']['right_arm'],expected['poses']['right_arm'])):
                    raise BackendError('软管步骤之间手臂位置或工具改变')
                angle = seed if index == 0 and seed is not None else arm.start['arm_elbow_deg']['right_arm']
                trunk = None
                if stage == 'grasp':
                    # Use the full-preflight branch; never invent a new advance.
                    unshifted = shifted(pose,rotation,advance)
                    planned,_,trunk = plan_approach(arm,unshifted,rotation,plane,preselected=advance)
                else:
                    planned = arm.plan(pose,angle,plane)
                    if stage == 'final_retreat' or (stage == 'recover_retreat' and advance):
                        trunk = plan_trunk(service.robot,service.config,job.cancel,arm.start,
                                           body_goal(start['poses']['trunk'],stage))
                if len(arm.steps)!=1:
                    raise BackendError('软管每段必须仅下发一条MoveL')
                if trunk:
                    pair = PairedGraspMove(arm,trunk)
                job._set(selected_arm_angle_deg=planned['angle'],seed_arm_angle_deg=angle,
                         endpoint_clearance_mm=planned.get('clearance'))
                service.audit_event('grasp_test_preflight',stage=stage,sku_typ='tube',pose=pose,
                                    start_state=arm.start,steps=arm.steps,result=planned,toolset=signature)
                moving = pair or arm
                job._check()
                if pair:
                    service.audit_event('grasp_pair_preflight',stage=stage,advance_mm=advance,
                                        start=arm.start,arm_step=arm.steps[0],trunk_plan=trunk,
                                        speed_mm_s=speed,rotation_deg_s=rotation_speed)
                    pair.start_move(speed,rotation_speed)
                else:
                    arm.start_step(0,speed,rotation_speed)
                if index==0:
                    job._set(first_dispatch_ms=round((time.perf_counter()-job.requested_at)*1000,3))
                job._set(phase='moving',message=label)
                if pair:
                    expected = pair.wait_move()
                else:
                    arm.wait_step(0)
                    expected = service.robot.read_state()
                    job._fixed_body(expected,body_baseline)
                moving = None
                # Carry the measured arrival into the next stage/close guard.
                body_baseline = copy.deepcopy(expected)
                arm.start = copy.deepcopy(expected)
            else:
                current = service.robot.read_state()
                job._fixed_body(current,body_baseline)
                if not poses_match(current['poses']['right_arm'],expected['poses']['right_arm']):
                    raise BackendError('软管步骤之间手臂位置改变')
                angle = seed if index==0 and seed is not None else current['arm_elbow_deg']['right_arm']
                service.robot.move_pose('right_arm',pose,speed,angle)
                if stage=='final_retreat':
                    service.robot.move_pose('trunk',body_goal(start['poses']['trunk'],stage),speed)
                expected = service.robot.read_state()
                body_baseline = copy.deepcopy(expected)
                arm = None
            job._check()
            if stage=='tilt_down':
                closing = True
                job._close_gripper(arm,expected)
                closing = False
            job._set(completed_moves=index+1)
            service.audit_event('grasp_test_stage_completed',stage=stage,sku_typ='tube')
        job._set(active=False,phase='completed',stage='completed',
                 message='软管抓取完成：法兰已回正，右臂退40 mm/躯干退100 mm均到位；保持夹持')
        service.audit_event('grasp_test_completed',sku_typ='tube',completed_moves=5,advance_mm=advance,
                            recovery_retreat_mm=d-20,final_arm_retreat_mm=40,final_trunk_retreat_mm=100)
    except Exception as exc:
        errors=[]
        if moving:
            try: errors.extend(moving.stop_and_verify())
            except Exception as stop_error: errors.append(str(stop_error))
        if closing:
            try: service.robot.gripper_stop()
            except Exception as stop_error: errors.append(str(stop_error))
        stop_unconfirmed=bool(errors) or bool(getattr(exc,'stop_unconfirmed',False))
        if stop_unconfirmed:
            with service._lock: service.armed=False
        message=str(exc)+('；停止确认失败：'+'; '.join(errors) if errors else '')
        job._set(active=False,phase='cancelled' if job.cancel.is_set() and not stop_unconfirmed else 'failed',
                 message=message,stop_unconfirmed=stop_unconfirmed)
        service.audit_event('grasp_test_failed',sku_typ='tube',error=message)
