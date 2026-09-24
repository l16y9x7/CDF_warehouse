"""Protected grasp approach/lift, ending at the upper-body retreat pose."""
from __future__ import annotations

import copy
import math
import threading
import time
import numpy as np

from .arm_movel import HardwareMoveL, load_guard_plane, number, pose_values, transform
from .backends import BackendError, JOINT_COUNTS
from .memory_motion import poses_match
from .memory_points import joint_error, require_idle, vector
from .pose_frames import POSE_FRAMES
from .trunk_retreat import HardwareTrunkRetreat
from .control_trace import fields, operation, span, update, worker_thread
from .trunk_frame import TRUNK_FRAME
from .pose_protocol import sku_type
from .grasp_compensation import (PairedGraspMove, plan_approach, plan_trunk,
                                 retreat_trunk_pose, shifted, shifted_targets)


MODULE = "right_arm"
LIFT_MM = 40.0
ARM_RETREAT_EXTRA_MM = 20.0
LEFT_ARM_RETREAT_EXTRA_MM = 30.0
CLEARANCE_LIFT_MM = 75.0
RETREAT_MM = 100.0
# Keep the arm/chest return stages available, but end the current button flow at
# the retreat point. Only change this after a separate, reviewed request.
RUN_RETURN_STAGES = False


def flange_targets(shoulder):
    """Express trunk SDK X/Z displacements in the arm SDK shoulder frame."""
    if shoulder.get("frame") != POSE_FRAMES[MODULE]:
        raise BackendError("抓取结果必须使用右肩 SDK 世界坐标系")
    grasp = vector(shoulder.get("grasp_pose_right_shoulder_mm_deg"), 6, "抓取法兰 Pose")
    pregrasp = vector(shoulder.get("pregrasp_pose_right_shoulder_mm_deg"), 6, "预抓取法兰 Pose")
    clearance = shoulder.get("box_clearance") or {}
    if clearance.get("valid") is not True or clearance.get("frame") != TRUNK_FRAME:
        raise BackendError("缺少本次定位的有效箱体距离，请重新估计位姿")
    try:
        d = float(clearance["d_mm"])
    except (KeyError, TypeError, ValueError) as exc:
        raise BackendError("箱沿到物品的距离 d 无效") from exc
    if not math.isfinite(d) or d < 0:
        raise BackendError("箱沿到物品的距离 d 必须是非负有限数值")
    R = np.asarray(shoulder.get("R_right_shoulder_from_trunk_ref"), dtype=float)
    if (R.shape != (3, 3) or not np.isfinite(R).all()
            or not np.allclose(R.T @ R, np.eye(3), atol=1e-6)
            or not np.isclose(np.linalg.det(R), 1, atol=1e-6)):
        raise BackendError("缺少有效的躯干 SDK 到右肩方向变换，请重新转换抓取点")
    def offset(pose, delta):
        return (np.asarray(pose[:3]) + R @ np.asarray(delta)).tolist() + pose[3:]
    lift = offset(grasp, [0, 0, LIFT_MM])
    retreat = offset(lift, [-(d + ARM_RETREAT_EXTRA_MM), 0, 0])
    clearance_lift = offset(retreat, [0, 0, CLEARANCE_LIFT_MM])
    return [("pregrasp", "预抓取点", pregrasp), ("grasp", "抓取点", grasp),
            ("lift", "提起 40 mm 点", lift),
            ("arm_retreat", f"右臂后退 {d + ARM_RETREAT_EXTRA_MM:.1f} mm 点", retreat),
            ("clearance_lift", "再次抬升 75 mm 点", clearance_lift)]


def left_flange_targets(shoulder):
    if shoulder.get('frame') != POSE_FRAMES['left_arm']:
        raise BackendError('方盒抓取结果必须使用左肩 SDK 世界坐标系')
    clearance = shoulder.get('box_clearance') or {}
    if clearance.get('valid') is not True or clearance.get('frame') != TRUNK_FRAME:
        raise BackendError('方盒预抓取需要有效的箱体前挡板距离')
    targets = [(key, label, vector(shoulder.get(key + '_pose_left_shoulder_mm_deg'), 6, label))
             for key, label in [('pregrasp', '左臂预抓取点'), ('grasp', '左臂抓取点'),
                                ('descend', '左臂法兰下降至下压标定高度'),
                                ('lift', '左臂法兰提起至提起标定高度')]]
    try:
        d = float(clearance['d_mm'])
    except (KeyError, TypeError, ValueError) as exc:
        raise BackendError('左箱进深 d 无效') from exc
    if not math.isfinite(d) or d < 0:
        raise BackendError('左箱进深 d 必须是非负有限数值')
    retreat = shifted(targets[-1][2], shoulder.get('R_left_shoulder_from_trunk_ref'), -(d + LEFT_ARM_RETREAT_EXTRA_MM))
    return targets + [('arm_retreat', f'左臂后退 {d + LEFT_ARM_RETREAT_EXTRA_MM:.1f} mm 点', retreat)]


def flange_to_tcp(pose, signature):
    # Estimation already subtracts the gripper length from the object point.
    # Convert the resulting flange target to the controller's configured TCP.
    end = vector(signature["end"], 6, "工具 TCP")
    end_mm_deg = [v * 1000 for v in end[:3]] + [math.degrees(v) for v in end[3:]]
    return pose_values(transform(pose) @ transform(end_mm_deg))


class GraspTest:
    def __init__(self, service):
        self.service = service
        self.cancel = threading.Event()
        self.thread = None
        self.job = {"active": False, "phase": "idle", "message": "抓取测试待命"}

    def status(self):
        with self.service._lock:
            return {**copy.deepcopy(self.job), "version": "grasp-test-v20"}

    def ensure_idle(self):
        if self.job.get("stop_unconfirmed"):
            raise BackendError("抓取测试停止未确认，请现场确认后重启网页服务")
        if self.job["active"]:
            raise BackendError("抓取测试正在执行，请先停止或等待完成")

    def _set(self, **fields):
        with self.service._lock:
            self.job.update(fields)
        if "stage" in fields:
            update(stage=fields["stage"])
        self.service.audit_event("grasp_test_progress", **fields)

    def _measure(self, name, call, *args):
        with span(self.service.audit_event, name) as timing:
            result = call(*args)
        self.timings[name] = round(timing["duration_ms"], 3)
        self._set(timings_ms=dict(self.timings))
        return result

    def _check(self):
        if self.cancel.is_set() or not self.service.armed:
            raise BackendError("抓取测试已停止")

    @operation("grasp_test")
    def execute(self, payload, *, prepared_result=None, require_gripper=False, prepared_suction_config=None):
        if not isinstance(payload, dict) or set(payload) - {'source_result_id', 'elbow_deg', 'sku_typ'}:
            raise BackendError('抓取请求仅接受 source_result_id、elbow_deg、sku_typ')
        try:
            kind = sku_type(payload.get('sku_typ'))
        except ValueError as exc:
            raise BackendError(str(exc)) from exc
        if kind not in ('bottle', 'box', 'tube'):
            raise BackendError('不支持的抓取类别')
        if kind == 'box' and require_gripper:
            raise BackendError('box 使用左臂吸盘，不使用右手夹爪')
        # Internal handoff only: the Agent reservation spans the confirmed close,
        # complete preflight, trunk approach and this grasp. Web payloads cannot skip it.
        if prepared_suction_config is not None:
            if kind != 'box' or prepared_result is None:
                raise BackendError('预关闭吸盘仅用于接口已准备的盒子抓取')
            from .suction import validate_config
            prepared_suction_config = copy.deepcopy(validate_config(prepared_suction_config))
        requested_at = time.perf_counter()
        source = payload.get("source_result_id")
        if not isinstance(source, str) or not source:
            raise BackendError("请先估计位姿或读取已保存的抓取结果")
        seed = payload.get("elbow_deg")
        seed = None if seed is None else number(seed, "初始臂角 °", -180, 180)
        plane = load_guard_plane(self.service.config)
        with self.service._lock:
            self.service._require_armed()
            self.service.chassis.stop()
            self.cancel.clear()
            self.timings = {}
            self.requested_at = requested_at
            speed = self.service.speed_mm_s
            rotation = self.service.rotation_deg_s
            self.job = {"active": True, "phase": "planning", "stage": "pregrasp",
                        "sku_typ": kind, "module": "left_arm" if kind == "box" else MODULE,
                        **fields(), "timings_ms": {},
                        "message": "读取起始位姿并计算抓取目标", "source_result_id": source,
                        "speed_mm_s": speed, "rotation_deg_s": rotation, "plane": plane, "protection_scope": "endpoint_only",
                        "protected_arm_stages": (["pregrasp", "grasp", "tilt_down", "recover_retreat", "final_retreat"] if kind == "tube" else ["pregrasp", "grasp", "descend", "lift", "arm_retreat"] if kind == "box" else
                                                 ["pregrasp", "grasp", "lift", "arm_retreat", "clearance_lift"]),
                        "offset_frame": TRUNK_FRAME, "completed_moves": 0,
                        "advance_mm": 0.0,
                        "total_moves": 5 if kind == "tube" else 6 if kind == "box" else (8 if RUN_RETURN_STAGES else 6)}
            self.thread = worker_thread(target=self._run, args=(source, seed, plane, speed, rotation, copy.deepcopy(prepared_result), require_gripper, kind, prepared_suction_config),
                                           name="grasp-test", daemon=True)
            self.thread.start()
            return self.status()

    def _fixed_body(self, state, start, module=MODULE):
        require_idle(state)
        if joint_error(state, start, [m for m in JOINT_COUNTS if m != module]) > 0.1:
            raise BackendError("抓取期间躯干、头部或另一手臂改变，后续动作已取消")
        if state.get("toolsets", {}).get("trunk") != start.get("toolsets", {}).get("trunk"):
            raise BackendError("抓取期间躯干 SDK 工具或参考系改变，后续动作已取消")

    def _close_gripper(self, executor, expected):
        self._check()
        self._set(phase="closing", stage="close_gripper", protection_scope="none",
                  message="合上夹爪，等待闭合或接触物品")
        self.service.robot.gripper_start_move(255)
        deadline = time.monotonic() + 30
        while time.monotonic() < deadline:
            self._check()
            if executor:
                with self.service.robot._lock:
                    state = executor._snapshot()
                    self._fixed_body(state, executor.start)
                    if joint_error(state, expected, (MODULE,)) > 0.5:
                        raise BackendError("夹爪闭合期间右臂位置改变")
            state = self.service.robot.gripper_status()
            if state["fault_code"] or state["activation_state"] != 3:
                raise BackendError(f"夹爪状态异常，故障码 {state['fault_code']}")
            if state["going_to_position"] and state["requested_position"] == 255:
                if state["object_state"] in (2, 3):
                    self._set(gripper_result=state)
                    return
                if state["object_state"] == 1:
                    raise BackendError("夹爪报告张开受阻，后续动作已取消")
            self.cancel.wait(0.1)
        raise BackendError("夹爪闭合等待超时，后续动作已取消")

    def _set_suction(self, opened):
        # This task already owns the motion lock. The public manual endpoint
        # intentionally rejects calls during a running grasp.
        self._check()
        self._set(phase='suction', message='打开吸盘并保持' if opened else '确认吸盘关闭')
        with self.service._lock:
            self._check()
            self.service._suction_result = {'commanded_open': None, 'confirmed': False}
            try:
                result = self.service.robot.suction_set(opened, self.suction_config)
                if result.get('confirmed') is not True or result.get('commanded_open') is not opened:
                    raise BackendError('吸盘输出未确认，后续运动已取消')
            except Exception as exc:
                self.service._suction_result['error'] = str(exc)
                self.service.audit_event('suction_command_failed', requested_open=opened, error=str(exc))
                raise
            self.service._suction_result = result
            self.service.audit_event('suction_command', **result)
        self._set(suction_result=result)
        # Cancellation/failure must never automatically release a held box.
        self._check()

    def _run(self, source, seed, plane, speed, rotation, prepared_result=None, require_gripper=False, kind="bottle", prepared_suction_config=None):
        if kind == 'tube':
            from .tube_grasp import run_tube
            return run_tube(self, source, seed, plane, speed, rotation, prepared_result)
        module = "left_arm" if kind == "box" else MODULE
        retreat_extra = LEFT_ARM_RETREAT_EXTRA_MM if kind == "box" else ARM_RETREAT_EXTRA_MM
        executor = None
        moving = None
        closing = False
        gripper_closed = False
        try:
            self._check()
            hardware = self.service.hardware_enabled
            if hardware:
                executor = self._measure("initial_snapshot", HardwareMoveL,
                                         self.service.robot, module, self.service.config, self.cancel)
                start = copy.deepcopy(executor.start)
                signature = executor.signature
            else:
                start = self.service.robot.read_state()
                signature = {"end": [0.0] * 6, "ref": [0.0] * 6}
            require_idle(start)
            if start.get("pose_frames", {}).get(module) != POSE_FRAMES[module]:
                raise BackendError("起始手臂位姿坐标系不一致")
            gripper = None
            if kind == "bottle":
                from .grasp_gripper import ensure_gripper_open
                self._set(phase='opening', stage='open_gripper',
                          message='确认右夹爪完全张开后开始规划')
                gripper = self._measure('gripper_ready', ensure_gripper_open,
                                        self.service, self.cancel, self._check)
                self._set(gripper_present=True, gripper_control_enabled=True)
            else:
                self._set(gripper_control_enabled=False, gripper_message='下压前关闭吸盘，到下压高度后打开并保持')
            self.service.audit_event("grasp_test_start", state=start, toolset=signature, module=module, sku_typ=kind,
                                     plane=plane, speed_mm_s=speed, rotation_deg_s=rotation, gripper=gripper)
            result = copy.deepcopy(prepared_result) if prepared_result is not None else self._measure(
                "reproject_pose", (self.service.pose_estimator.reproject_latest_to_left_shoulder if kind == "box" else
                                   self.service.pose_estimator.reproject_latest_to_right_shoulder), start, start)
            self.service.audit_event("grasp_test_pose_conversion", result=result)
            if result["source_result_id"] != source:
                raise BackendError("保存的抓取结果已更新，请重新估计或转换后再执行")
            if result.get('sku_typ', result.get('world_grasp', {}).get('sku_typ')) != kind:
                raise BackendError('定位结果类别与本次所选手臂不一致')
            targets = (left_flange_targets if kind == 'box' else flange_targets)(result['shoulder_grasp'])
            flange_poses = {key: pose for key, _, pose in targets}
            targets = [(key, label, flange_to_tcp(pose, signature)) for key, label, pose in targets]
            advance = 0.0
            rotation_from_trunk = result['shoulder_grasp'].get(
                'R_left_shoulder_from_trunk_ref' if kind == 'box' else 'R_right_shoulder_from_trunk_ref')
            preselected = result.get('preplanned_advance_mm') if prepared_result is not None else None
            origin = vector(start["poses"][module], 6, "起始手臂 Pose")
            origin_angle = start["arm_elbow_deg"][module]
            self._set(start_pose_mm_deg=origin, start_arm_angle_deg=origin_angle,
                      targets_tcp_mm_deg={key: pose for key, _, pose in targets},
                      targets_flange_mm_deg=flange_poses,
                      recognition_height_trunk_mm=result["world_grasp"].get("recognition_height_trunk_mm"),
                      descend_height_trunk_mm=result["shoulder_grasp"].get("descend_height_trunk_mm"),
                      lift_height_trunk_mm=result["shoulder_grasp"].get("lift_height_trunk_mm"),
                      box_clearance=result["shoulder_grasp"]["box_clearance"],
                      arm_retreat_mm=result["shoulder_grasp"]["box_clearance"]["d_mm"] + retreat_extra,
                      lift_mm=LIFT_MM if kind == "bottle" else None, clearance_lift_mm=CLEARANCE_LIFT_MM if kind == "bottle" else None,
                      shoulder_grasp=result["shoulder_grasp"], current_trunk_joints_deg=result["current_trunk_joints_deg"])
            expected = copy.deepcopy(start)
            body_baseline = copy.deepcopy(start)
            trunk = self._measure("trunk_endpoint_preflight", HardwareTrunkRetreat,
                                  self.service.robot, self.service.config, self.cancel,
                                  start, RETREAT_MM) if hardware else None
            if trunk is not None:
                self.service.audit_event("grasp_test_trunk_preflight", moves=getattr(trunk, "moves", {}),
                                         start=getattr(trunk, "start", start), toolset=getattr(trunk, "signature", None))
            self._set(trunk_retreat_mm=RETREAT_MM, trunk_start_joints_deg=start["joints_deg"]["trunk"])
            if kind == 'box':
                from .suction import validate_config
                self.suction_config = copy.deepcopy(validate_config(
                    prepared_suction_config if prepared_suction_config is not None else self.service.config.get('suction')))
                if prepared_suction_config is None:
                    self._set_suction(False)
            completed = 0
            for index, (key, label, pose) in enumerate(targets):
                self._check()
                self._set(phase="planning", stage=key,
                          protection_scope="endpoint_only",
                          endpoint_clearance_mm=None,
                          message=f"演算{label}的保护 MoveL")
                if hardware:
                    if index:
                        executor = self._measure(key + "_snapshot", HardwareMoveL,
                                                 self.service.robot, module, self.service.config, self.cancel)
                    self._fixed_body(executor.start, body_baseline, module)
                    if executor.signature != signature:
                        raise BackendError("抓取期间工具或工件坐标系改变")
                    if (joint_error(executor.start, expected, (module,)) > 0.5
                            or not poses_match(executor.start["poses"][module], expected["poses"][module])):
                        raise BackendError("步骤之间当前手臂位置改变，后续动作已取消")
                    # Every stage after pregrasp uses a fresh measured arm angle.
                    seed_angle = seed if index == 0 and seed is not None else executor.start["arm_elbow_deg"][module]
                    pair = None
                    if key == 'grasp':
                        planned, advance, trunk_plan = self._measure('grasp_plan', lambda: plan_approach(
                            executor, pose, rotation_from_trunk, plane, preselected=preselected))
                        if advance:
                            targets[:] = shifted_targets(targets, rotation_from_trunk, advance)
                            pose = targets[index][2]
                            flange_poses = {k: shifted(p, rotation_from_trunk, -advance) if k in ('grasp', 'descend', 'lift') else p
                                           for k, p in flange_poses.items()}
                            pair = PairedGraspMove(executor, trunk_plan)
                            self._set(advance_mm=advance,
                                      arm_retreat_mm=result['shoulder_grasp']['box_clearance']['d_mm'] + retreat_extra - advance,
                                      paired_trunk_retreat_mm=advance,
                                      targets_tcp_mm_deg={k: p for k, _, p in targets}, targets_flange_mm_deg=flange_poses,
                                      total_moves=6 if kind == 'box' else (8 if RUN_RETURN_STAGES else 6))
                    else:
                        planned = self._measure(key + "_plan", executor.plan, pose, seed_angle, plane)
                        if key == 'arm_retreat' and advance:
                            trunk_plan = self._measure('paired_retreat_trunk_plan', plan_trunk,
                                                      self.service.robot, self.service.config, self.cancel,
                                                      executor.start, retreat_trunk_pose(start['poses']['trunk']))
                            pair = PairedGraspMove(executor, trunk_plan)
                    if len(executor.steps) != 1:
                        raise BackendError("每个抓取目标必须仅下发一条 MoveL")
                    angle = planned["angle"]
                    self._set(planning_ms=planned.get("planning_ms"), selected_arm_angle_deg=angle,
                              seed_arm_angle_deg=seed_angle,
                              endpoint_clearance_mm=planned.get("clearance"))
                    self.service.audit_event("grasp_test_preflight", stage=key, pose=pose,
                                             seed_arm_angle_deg=seed_angle, torso_guard_enabled=True,
                                             start_state=executor.start, toolset=executor.signature,
                                             steps=executor.steps, plane=plane, result=planned)
                    self._check()
                    moving = pair or executor
                    if pair:
                        self.service.audit_event('grasp_pair_preflight', stage=key, advance_mm=advance,
                                                 start=executor.start, arm_step=executor.steps[0], trunk_plan=trunk_plan,
                                                 speed_mm_s=speed, rotation_deg_s=rotation)
                        self._measure(key + '_dispatch', pair.start_move, speed, rotation)
                    else:
                        self._measure(key + "_dispatch", executor.start_step, 0, speed, rotation)
                    if index == 0:
                        self._set(first_dispatch_ms=round((time.perf_counter() - self.requested_at) * 1000, 3))
                    self._set(phase="moving", message=f"正在到达{label}")
                    if pair:
                        expected = pair.wait_move()
                        body_baseline = copy.deepcopy(expected)
                        # Closing the gripper now guards the reached body position.
                        executor.start = copy.deepcopy(expected)
                    else:
                        executor.wait_step(0)
                    moving = None
                    if not pair:
                        expected["joints_deg"][module] = list(executor.steps[0]["q"])
                else:
                    self._fixed_body(self.service.robot.read_state(), body_baseline, module)
                    angle = seed if index == 0 and seed is not None else self.service.robot.read_state()["arm_elbow_deg"][module]
                    self.service.robot.move_pose(module, pose, speed, angle)
                expected["poses"][module] = list(pose)
                self._check()
                completed += 1
                self._set(completed_moves=completed)
                if key == "grasp" and gripper is not None and self.service.gripper_unlocked:
                    closing = True  # A failed write may still have reached the gripper.
                    self._close_gripper(executor, expected)
                    closing = False
                    gripper_closed = True
                elif key == "grasp" and kind == "bottle":
                    self._set(gripper_result="夹爪未解锁或未识别，未发送开合指令")
                if key == 'descend' and kind == 'box':
                    self._set_suction(True)
                self.service.audit_event("grasp_test_stage_completed", stage=key)
            self._check()
            self._set(phase="moving", stage="trunk_retreat", protection_scope="none", endpoint_clearance_mm=None,
                      message=f"躯干带动整个上身沿躯干 SDK X−后退 {RETREAT_MM:g} mm")
            if hardware:
                if advance:
                    # Both arms return the body by A first; prepare the separate
                    # 100 mm from the arrived state after all arm stages.
                    trunk = self._measure('final_trunk_prepare', HardwareTrunkRetreat,
                                          self.service.robot, self.service.config, self.cancel, expected, RETREAT_MM)
                moving = trunk
                self._measure("trunk_retreat_dispatch", trunk.start_move, "trunk_retreat", expected, speed, rotation)
                expected = trunk.wait_move()
                moving = None
            else:
                body_pose = list(start["poses"]["trunk"])
                body_pose[0] -= RETREAT_MM
                self.service.robot.move_pose("trunk", body_pose, speed)
                expected = self.service.robot.read_state()
            self._check()
            body_baseline = copy.deepcopy(expected)
            completed += 1
            self._set(completed_moves=completed)
            self.service.audit_event("grasp_test_stage_completed", stage="trunk_retreat")
            if kind == 'box':
                self._set(active=False, phase='completed', stage='completed',
                          message='盒子抓取测试完成：已提起并后退100 mm；吸盘保持打开')
                self.service.audit_event('grasp_test_completed', sku_typ=kind, module=module,
                                         completed_moves=completed, suction_enabled=True)
                return
            if not RUN_RETURN_STAGES:
                self._set(active=False, phase="completed", stage="completed",
                          message=(f"抓取测试完成：上身已后退 {RETREAT_MM:g} mm；保持当前位置" +
                                   ("和夹爪闭合" if gripper_closed else "；未执行夹爪闭合")) if hardware else
                                  "MOCK 抓取流程完成（未做实机几何验证）：停在后退位置")
                self.service.audit_event('grasp_test_completed', sku_typ=kind, module=module,
                                         advance_mm=advance, completed_moves=completed,
                                         paired_trunk_retreat_mm=advance,
                                         separate_trunk_retreat_mm=RETREAT_MM)
                return

            # Preserved for a later phase. The current grasp-test button never
            # enters these two return stages while RUN_RETURN_STAGES is False.
            self._check()
            self._set(phase="planning", stage="return", protection_scope="none",
                      endpoint_clearance_mm=None,
                      message="按出发时的位姿和臂角演算普通 MoveL 收回右臂")
            if hardware:
                executor = self._measure("return_snapshot", HardwareMoveL,
                                         self.service.robot, module, self.service.config, self.cancel)
                self._fixed_body(executor.start, body_baseline, module)
                if executor.signature != signature:
                    raise BackendError("抓取期间工具或工件坐标系改变")
                if (joint_error(executor.start, expected, (module,)) > 0.5
                        or not poses_match(executor.start["poses"][module], expected["poses"][module])):
                    raise BackendError("步骤之间当前手臂位置改变，后续动作已取消")
                planned = self._measure("return_plan", executor.plan, origin, origin_angle, None)
                if len(executor.steps) != 1:
                    raise BackendError("右臂收回目标必须仅下发一条 MoveL")
                self._set(planning_ms=planned.get("planning_ms"),
                          selected_arm_angle_deg=planned["angle"], seed_arm_angle_deg=origin_angle,
                          endpoint_clearance_mm=None)
                self.service.audit_event("grasp_test_preflight", stage="return", pose=origin,
                                         seed_arm_angle_deg=origin_angle, torso_guard_enabled=False,
                                         start_state=executor.start, toolset=executor.signature,
                                         steps=executor.steps, plane=None, result=planned)
                self._check()
                moving = executor
                self._measure("return_dispatch", executor.start_step, 0, speed, rotation)
                self._set(phase="moving", message="正在收回右臂到出发位姿")
                executor.wait_step(0)
                moving = None
                expected["joints_deg"][module] = list(executor.steps[0]["q"])
            else:
                self.service.robot.move_pose(module, origin, speed, origin_angle)
            expected["poses"][module] = list(origin)
            self._check()
            completed += 1
            self._set(completed_moves=completed)
            self.service.audit_event("grasp_test_stage_completed", stage="return")

            self._check()
            self._set(phase="moving", stage="trunk_return", protection_scope="none",
                      endpoint_clearance_mm=None, message="手臂已收回，躯干返回出发位置")
            if hardware:
                moving = trunk
                self._measure("trunk_return_dispatch", trunk.start_move, "trunk_return", expected, speed, rotation)
                trunk.wait_move()
                moving = None
            else:
                self.service.robot.move_pose("trunk", start["poses"]["trunk"], speed)
            self._check()
            self._set(completed_moves=completed + 1)
            self.service.audit_event("grasp_test_stage_completed", stage="trunk_return")
            self._set(active=False, phase="completed", stage="completed",
                      message="抓取测试完成，手臂和躯干已回起点；夹爪保持闭合" if hardware else
                              "MOCK 抓取流程完成（未做实机几何验证），手臂和躯干已回起点")
        except Exception as exc:
            errors = []
            if moving:
                try:
                    errors.extend(moving.stop_and_verify())
                except Exception as stop_error:
                    errors.append(str(stop_error))
            if closing:
                try:
                    self.service.robot.gripper_stop()
                except Exception as stop_error:
                    errors.append(str(stop_error))
            if errors:
                with self.service._lock:
                    self.service.armed = False
            message = str(exc) + ("；停止确认失败：" + "; ".join(errors) if errors else "")
            stop_unconfirmed = bool(errors) or bool(getattr(exc, 'stop_unconfirmed', False))
            self._set(active=False, phase="cancelled" if self.cancel.is_set() and not stop_unconfirmed else "failed",
                      message=message, stop_unconfirmed=stop_unconfirmed)
            self.service.audit_event("grasp_test_failed", error=message)

    def stop(self):
        self.cancel.set()
        self.service.audit_event("grasp_test_stop_requested", target_operation_id=self.job.get("operation_id"))
        with self.service._lock:
            if self.job["active"]:
                self.job.update(phase="stopping", message="正在停止抓取测试")
            return self.status()

    def close(self):
        self.stop()
        if self.thread and self.thread.is_alive():
            self.thread.join(timeout=10)
