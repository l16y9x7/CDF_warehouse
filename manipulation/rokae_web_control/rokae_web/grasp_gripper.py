"""Confirm the right gripper's requested opening before grasp planning."""
import time

from .backends import BackendError


# The 2F-85 may report about 3 at the calibrated open end instead of exactly 0.
OPEN_POSITION_TOLERANCE = 5
OPEN_TIMEOUT_SECONDS = 30.0


def _ready(state):
    if state.get('fault_code') != 0 or state.get('activation_state') != 3:
        raise BackendError('右夹爪未就绪，请初始化并清除故障')


def _at_position(state, target):
    position = state.get('measured_position')
    return (state.get('requested_position') == target and state.get('object_state') == 3
            and isinstance(position, (int, float)) and not isinstance(position, bool)
            and 0 <= position <= 255 and abs(position-target) <= OPEN_POSITION_TOLERANCE)


def ensure_gripper_open(service, cancel, check):
    return ensure_gripper_position(service, cancel, check, 0)


def ensure_gripper_position(service, cancel, check, target):
    """Caller owns the grasp/action reservation. Failure never proceeds to planning."""
    if type(target) is not int or not 0 <= target <= 255:
        raise BackendError('夹爪预开位置必须是0–255整数')
    label = '完全张开' if target == 0 else f'预开到{target}'
    event = 'gripper_open' if target == 0 else 'gripper_preopen'
    def check_active():
        check()
        if not service.armed or not service.gripper_unlocked:
            raise BackendError('右夹爪未解锁或抓取已停止，取消规划')

    attempted = False
    try:
        with service._lock:
            check_active()
            state = service.robot.gripper_status()
            _ready(state)
            if _at_position(state, target):
                check_active()
                service.audit_event(event+'_confirmed', stage='before_pick_planning',
                                    already_open=True, position=target, status=state)
                return state
            if state.get('going_to_position') and state.get('object_state') == 0:
                raise BackendError('右夹爪仍在运动，取消抓取规划')
            service.audit_event(event+'_requested', stage='before_pick_planning', position=target)
            attempted = True  # A failed write may still have reached the gripper.
            service.robot.gripper_start_move(target)
        deadline = time.monotonic() + OPEN_TIMEOUT_SECONDS
        while True:
            with service._lock:
                check_active()
                state = service.robot.gripper_status()
                _ready(state)
                if _at_position(state, target):
                    check_active()
                    service.audit_event(event+'_confirmed', stage='before_pick_planning',
                                        already_open=False, position=target, status=state)
                    return state
                if state.get('requested_position') == target:
                    if state.get('object_state') in (1, 2):
                        raise BackendError(f'右夹爪张开受阻，未{label}，取消抓取规划')
                    if state.get('object_state') == 3:
                        raise BackendError(f'右夹爪回读未到{label}位置，取消抓取规划')
            if time.monotonic() >= deadline:
                raise BackendError(f'右夹爪{label}等待超时，取消抓取规划')
            cancel.wait(.1)
    except Exception as exc:
        service.audit_event(event+'_failed', stage='before_pick_planning', position=target, error=str(exc))
        if attempted:
            try:
                stopped = service.robot.gripper_stop()
                if not isinstance(stopped, dict) or stopped.get('going_to_position') is not False:
                    raise BackendError('夹爪停止回读未确认')
            except Exception as stop_error:
                with service._lock:
                    service.armed = False
                    actions = getattr(service, 'agent_actions', None)
                    if actions is not None:
                        actions.stop_unconfirmed = True
                failure = BackendError(f'{exc}；夹爪停止确认失败：{stop_error}')
                failure.stop_unconfirmed = True
                raise failure from exc
        raise
