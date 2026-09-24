"""Realtime camera extrinsics and localization frozen in the PCB4 reference."""
import copy
import time
import numpy as np
from .backends import BackendError
from .box_clearance import front_panel_from_response, clearance_in_trunk
from .grasp_pose import world_grasp_from_4090, world_grasp_to_right_shoulder, world_grasp_to_left_shoulder
from .trunk_frame import trunk_reference_in_chassis, TRUNK_FRAME
from .pose_targets import interpret, basket_right_shoulder, basket_left_shoulder
from .pose_protocol import validate_response_type, shared_grasp_height
from .action_poses import box_heights, tube_height


class AgentGeometry:
    def __init__(self, service):
        self.service = service
        self.estimator = service.pose_estimator

    def extrinsics(self, state=None):
        state = state or self.service.robot.read_memory_state()
        _, camera, _ = self.estimator._load_calibration()
        joints = self.estimator._upper_body_joints(state)
        matrix = self.estimator._kinematics.camera_to_base(joints, camera)
        return dict(T_chassis_camera=np.asarray(matrix).tolist(), t_unit='m',
                    camera='head', camera_frame=self.estimator.CAMERA_FRAME,
                    base_frame='chassis_link', sampled_at_unix_s=time.time(),
                    upper_body_joints_deg=joints.tolist())

    def freeze(self, response, state, kind='bottle'):
        if kind not in ('bottle', 'box', 'tube'):
            raise BackendError('仅支持 bottle、box 或 tube 抓取')
        try:
            validate_response_type(response, kind)
        except ValueError as exc:
            raise BackendError(f'需要 sku_typ={kind} 的完整定位结果：'+str(exc)) from exc
        live = self.extrinsics(state)
        kin = self.estimator._kinematics
        trunk = trunk_reference_in_chassis(state, kin)
        calibration = box_heights(self.estimator.config) if kind == 'box' else None
        tube_calibration = tube_height(self.estimator.config) if kind == 'tube' else None
        height = (tube_calibration['pregrasp_flange_height_mm'] if tube_calibration else
                  calibration['heights_mm']['grasp'] if calibration else shared_grasp_height(self.estimator.config))
        request = dict(live, T_unit='m', T_chassis_trunk_ref_m=trunk.tolist(),
                       grasp_height_trunk_mm=height, sku_typ=kind, grasp_height_sku_typ=kind,
                       box_height_calibration=calibration, tube_height_calibration=tube_calibration)
        world = world_grasp_from_4090(request, response)
        panel = front_panel_from_response(request, response)
        clearance_in_trunk(panel, world, trunk)
        # Persist the reference, point and panel together. No recomputing the
        # height-plane intersection after moving the torso.
        return dict(source_result_id=str(response.get('request_id') or 'agent-localization'),
                    world_grasp=world, front_panel=panel, T_chassis_trunk_ref_m=trunk.tolist(),
                    frozen_frame=TRUNK_FRAME, live_extrinsics=live, sku_typ=kind)

    def project(self, frozen, state):
        kind = frozen.get('sku_typ', 'bottle')
        if kind not in ('bottle', 'box', 'tube'):
            raise BackendError('冻结抓取类别无效')
        kin = self.estimator._kinematics
        reference = np.asarray(frozen['T_chassis_trunk_ref_m'])
        current = trunk_reference_in_chassis(state, kin)
        delta = np.linalg.inv(reference) @ current
        angle = np.arccos(np.clip((np.trace(delta[:3, :3])-1)/2, -1, 1))
        if np.linalg.norm(delta[:3, 3]) > .002 or angle > np.radians(.1):
            raise BackendError('躯干 SDK 参考系改变，冻结目标不能继续执行')
        clearance = clearance_in_trunk(frozen['front_panel'], frozen['world_grasp'], reference)
        side = 'left' if kind == 'box' else 'right'
        shoulder = getattr(kin, side + '_shoulder_sdk_world')(state['joints_deg']['trunk'])
        projector = world_grasp_to_left_shoulder if kind == 'box' else world_grasp_to_right_shoulder
        result = copy.deepcopy(frozen)
        result.update(current_trunk_joints_deg=list(state['joints_deg']['trunk']),
                      box_clearance=clearance, shoulder_grasp=projector(
                          frozen['world_grasp'], shoulder, clearance, reference))
        return result

    def basket(self, response, state, arm='right_arm'):
        if arm not in ('left_arm', 'right_arm'):
            raise BackendError('篮筐投影手臂无效')
        live = self.extrinsics(state)
        result = interpret(response, 'basket', np.asarray(live['T_chassis_camera']))
        if not result.get('valid'):
            raise BackendError(result.get('error') or '篮筐定位无效')
        side = 'left' if arm == 'left_arm' else 'right'
        projector = basket_left_shoulder if arm == 'left_arm' else basket_right_shoulder
        shoulder = getattr(self.estimator._kinematics, side + '_shoulder_sdk_world')(state['joints_deg']['trunk'])
        result.update(projector(result, shoulder))
        self.service.audit_event('agent_basket_projection', input=response, extrinsics=live, result=result, module=arm)
        return result[f'point_{side}_shoulder_mm']
