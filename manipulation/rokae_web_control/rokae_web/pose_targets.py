"""Per-target localization contracts from ROBOT_API_HANDOFF, no motion calls."""
import numpy as np
from .pose_protocol import local_target, validate_response_type

TARGETS = ('bottle', 'box', 'tube', 'basket')
LABELS = dict(bottle='白色罐子（bottle）', box='方盒（box）', tube='软管（tube）', basket='篮筐')


def selection(payload, default_side='RIGHT'):
    if not isinstance(payload, dict) or set(payload) - {'target', 'sku_typ', 'side'}:
        raise ValueError('识别请求仅接受 sku_typ（商品）、target=basket（篮筐）和 side')
    if 'target' in payload and 'sku_typ' in payload:
        raise ValueError('target 与 sku_typ 不能同时提供')
    target = local_target(payload.get('sku_typ', payload.get('target')))
    if target not in TARGETS:
        raise ValueError('请选择 bottle、box、tube 或篮筐')
    fixed_side = {'bottle': 'RIGHT', 'box': 'LEFT', 'tube': 'RIGHT'}.get(target)
    side = payload.get('side', fixed_side or default_side)
    if fixed_side and side != fixed_side:
        raise ValueError(f'{target} 固定使用 {fixed_side} 箱')
    if side not in ('LEFT', 'RIGHT'):
        raise ValueError('商品选箱必须为 LEFT 或 RIGHT')
    return dict(target=target, side=side)


def finite(value, shape):
    try:
        result = np.asarray(value, dtype=float)
        if result.shape != shape or not np.isfinite(result).all():
            raise ValueError()
        return result
    except (TypeError, ValueError):
        raise ValueError('定位字段缺失、维数不符或含非有限数值') from None


def _basket_shoulder(localization, chassis_from_shoulder_m, side, label):
    """Express the basket model center in SDK shoulder axes, without tool offsets."""
    if localization.get('target') != 'basket' or localization.get('valid') is not True:
        raise ValueError(label + '参考点需要本帧有效的篮筐定位')
    point = finite(localization.get('point_chassis_mm'), (3,))
    transform = finite(chassis_from_shoulder_m, (4, 4))
    rotation = transform[:3, :3]
    if (not np.allclose(transform[3], [0, 0, 0, 1], atol=1e-8)
            or not np.allclose(rotation.T @ rotation, np.eye(3), atol=1e-4)
            or not np.isclose(np.linalg.det(rotation), 1, atol=1e-4)):
        raise ValueError(label + '坐标变换无效')
    return {f'{side}_shoulder_frame': f'{side}_arm_sdk_world',
            f'point_{side}_shoulder_mm': (rotation.T @ (point - transform[:3, 3] * 1000)).tolist(),
            f'T_chassis_{side}_shoulder_m': transform.tolist()}


def basket_right_shoulder(localization, chassis_from_shoulder_m):
    return _basket_shoulder(localization, chassis_from_shoulder_m, 'right', '右肩')


def basket_left_shoulder(localization, chassis_from_shoulder_m):
    return _basket_shoulder(localization, chassis_from_shoulder_m, 'left', '左肩')


def interpret(response, target, transform):
    result = dict(target=target, label=LABELS[target], valid=False,
                  camera_frame='head_camera_color_optical_frame', base_frame='chassis_link',
                  point_camera_mm=None, point_chassis_mm=None)
    try:
        if response.get('output_frame') not in (None, result['camera_frame']):
            raise ValueError('定位输出坐标系不符')
        if response.get('output_unit') not in (None, 'mm'):
            raise ValueError('定位输出单位不符')
        if response.get('target_type') not in (None, 'basket' if target == 'basket' else 'sku'):
            raise ValueError('返回的目标类型与请求不一致')
        validate_response_type(response, target)
        if target == 'bottle':
            key = 'reference_point_camera_mm'
            point = finite(response.get(key), (3,))
            extra = dict(point_semantics='可见瓶身轴段中点')
        elif target == 'box':
            key = 'top_point_camera_mm'
            point = finite(response.get(key), (3,))
            extra = dict(point_semantics='拟合顶面参考点')
        elif target == 'tube':
            key = 'top_edge_center_camera_mm'
            point = finite(response.get(key), (3,))
            extra = dict(point_semantics='可见上边缘中点')
            # The center alone defines this grasp. Auxiliary edge geometry is
            # diagnostic and never vetoes a returned center point.
            for name, shape in (('top_edge_endpoints_camera_mm', (2, 3)), ('edge_direction_camera', (3,))):
                if response.get(name) is not None:
                    try:
                        extra[name] = finite(response[name], shape).tolist()
                    except ValueError:
                        pass
        else:
            key = 'model_center_camera_mm'
            point = finite(response.get(key), (3,))
            matrix = finite(response.get('pose_4x4'), (4, 4))
            if (not np.allclose(matrix[3], [0, 0, 0, 1], atol=1e-6)
                    or not np.allclose(matrix[:3, :3].T @ matrix[:3, :3], np.eye(3), atol=1e-4)
                    or not np.isclose(np.linalg.det(matrix[:3, :3]), 1, atol=1e-4)):
                raise ValueError('篮筐中心语义或位姿矩阵无效')
            extra = dict(point_semantics='篮筐 CAD 模型中心', pose_4x4_camera_mm=matrix.tolist(),
                         cad_origin_camera_mm=matrix[:3, 3].tolist())
            for name, shape in (('rotation_euler_zyx_rad', (3,)), ('xyzrxryrz_camera_mm_rad', (6,))):
                if response.get(name) is not None:
                    extra[name] = finite(response[name], shape).tolist()
        chassis = finite(transform, (4, 4))
        result.update(valid=True, point_camera_mm=point.tolist(),
                      point_chassis_mm=(chassis[:3, :3] @ point + chassis[:3, 3] * 1000).tolist(), **extra)
    except ValueError as exc:
        result['error'] = str(exc)
    return result
