"""Read-only sampler adapter for the existing SDK owner, never another connection."""
import math
import time

from .telemetry import TelemetryBusy


def numbers(values, count, factor=1.0):
    values = list(values)
    if len(values) != count or any(isinstance(v, bool) or not math.isfinite(float(v)) for v in values):
        raise ValueError('invalid measured vector')
    result = [float(v)*factor for v in values]
    if not all(math.isfinite(v) for v in result):raise ValueError('nonfinite converted vector')
    return result


def enum_name(value):
    return str(getattr(value, 'name', value))


def common_state(operation, power):
    if power in ('estop', 'gstop', 'unknown') or operation in ('unknown', 'error'):
        return 'error'
    if power == 'off':
        return 'disabled'
    return 'idle' if operation in ('idle', 'jog') else 'moving'


class ManipulatorReader:
    def __init__(self, backend, config, *, motion_pending=lambda: False):
        self.backend, self.config = backend, config
        self.motion_pending = motion_pending
        self.slow, self.last = {}, {}

    def _read(self, module, method, *args):
        # Acquire per getter, not across a batch or any sleep. A waiting motion
        # request takes precedence; an already executing getter cannot be preempted.
        if self.motion_pending() or not self.backend._lock.acquire(blocking=False):
            raise TelemetryBusy()
        try:
            if self.motion_pending():
                raise TelemetryBusy()
            robot = self.backend._robot(module)
            ec = {}
            result = getattr(robot, method)(*args, ec)
            self.backend._check_ec('上报 ' + method, ec)
            return result
        finally:
            self.backend._lock.release()

    def _optional(self, module, key, seconds, query):
        now = time.monotonic()
        cache_key = (module, key)
        if now-self.last.get(cache_key, -float('inf')) >= seconds:
            try:
                result = query()
            except TelemetryBusy:
                pass
            except Exception:
                self.last[cache_key] = now
                self.slow.pop(cache_key, None)
            else:
                self.last[cache_key] = now
                self.slow[cache_key] = result
        if now-self.last.get(cache_key, -float('inf')) > seconds*2:return None
        return self.slow.get(cache_key)

    def _details(self, module, count, sdk):
        result = {}
        info = self._read(module, 'robotInfo')
        result['controller'] = dict(model=str(info.type), version=str(info.version),
                                    controller_id=str(info.id), joint_count=int(info.joint_num),
                                    sdk_version=str(sdk.BaseRobot.sdkVersion()))
        holder = sdk.PyTypeVectorArrayDouble2()
        result['soft_limits_enabled'] = bool(self._read(module, 'getSoftLimit', holder))
        pairs = list(holder.content())
        if len(pairs) != count:
            raise ValueError('invalid controller limits')
        result['joint_limits_deg'] = [numbers(p, 2, 180/math.pi) for p in pairs]
        if module == 'trunk':
            names = sdk.PyTypeVectorString()
            self._read(module, 'getMechUnit', 'u1', 'axes_info', names)
            if len(names.content()) == 2:
                limits = []
                for name in names.content():
                    pair = []
                    for key in ('soft_limit_lower', 'soft_limit_upper'):
                        value = sdk.PyTypeDouble()
                        self._read(module, 'getExtAxisInfo', name, key, value)
                        pair.append(value.content())  # External-axis configuration is degrees.
                    limits.append(numbers(pair, 2))
                result['_head_limits'] = limits
        return result

    def _logs(self, module, sdk):
        logs = self._read(module, 'queryControllerLog', 3, {sdk.LogInfoLevel.warning, sdk.LogInfoLevel.error})
        return [{k: getattr(row, k) for k in ('id','timestamp','content','repair')} for row in list(logs)[:3]]

    def _gripper(self, sdk):
        data = sdk.PyTypeVectorInt([0,0,0])
        self._read('right_arm', 'XPRWModbusRTUReg', 9, 0x03, 0x07D0, 'uint16', 3, data, False)
        words = list(data.content())
        if len(words) != 3 or any(type(v) is not int or not 0 <= v <= 65535 for v in words):
            raise ValueError('invalid gripper status')
        flags = words[0] >> 8
        return dict(activated=bool(flags & 1), action_status=(flags >> 4) & 3,
                    object_status=(flags >> 6) & 3, go_to=bool(flags & 8),
                    fault_code=words[1] >> 8, requested_position=words[1] & 255,
                    position=words[2] >> 8, current_raw=words[2] & 255)

    def sample(self, name):
        module = 'trunk' if name == 'body' else name
        count = 4 if name == 'body' else 7
        sdk = self.backend._load_sdk()
        q = list(self._read(module, 'jointPos'))
        cart = self._read(module, 'cartPosture', sdk.CoordinateType.flangeInBase)
        operation = enum_name(self._read(module, 'operationState'))
        power = enum_name(self._read(module, 'powerState'))
        if power not in ('on','off','estop','gstop','unknown'):
            power = 'unknown'
        part = dict(state=common_state(operation, power), power_state=power,
                    operation_state=operation, joint_positions_deg=numbers(q[:count], count, 180/math.pi),
                    end_pose=numbers(cart.trans, 3, 1000)+numbers(cart.rpy, 3, 180/math.pi))
        head = None
        if name == 'body':
            external = list(getattr(cart, 'external', []))[:2]
            if len(external) != 2 and len(q) >= 6:
                external = q[4:6]
            if len(external) == 2:
                head = {k:part[k] for k in ('state','power_state','operation_state')}
                head['joint_positions_deg'] = numbers(external, 2, 180/math.pi)
        for field, method, factor in (('operate_mode','operateMode',None),
                                      ('joint_velocities_deg_s','jointVel',180/math.pi),
                                      ('joint_torques_nm','jointTorque',1)):
            try:
                value = self._read(module, method)
                if factor is None:
                    value = enum_name(value)
                    part[field] = value if value in ('manual','automatic','unknown') else 'unknown'
                    if head is not None:head[field] = part[field]
                else:
                    values = list(value)
                    part[field] = numbers(values[:count], count, factor)
                    if head is not None and field == 'joint_velocities_deg_s' and len(values) >= 6:
                        head[field] = numbers(values[4:6], 2, factor)
            except Exception:
                pass  # Optional field with no current real value is omitted.
        details = self._optional(module, 'details', 60, lambda:self._details(module,count,sdk))
        if details:
            part.update({k:v for k,v in details.items() if not k.startswith('_')})
            if head is not None:
                if '_head_limits' in details:head['joint_limits_deg'] = details['_head_limits']
                head['controller'] = dict(details['controller'], joint_count=2)
        logs = self._optional(module, 'logs', self.config.get('log_query_interval_sec',5), lambda:self._logs(module,sdk))
        if logs is not None:
            part['controller_logs'] = logs
            if head is not None:head['controller_logs'] = logs
        if module == 'right_arm' and self.config.get('include_grippers',True):
            gripper = self._optional(module, 'gripper', self.config.get('gripper_query_interval_sec',1), lambda:self._gripper(sdk))
            if gripper is not None:part['gripper'] = gripper
        # This robot's left hand is a suction relay, not a Robotiq gripper.
        return {name:part, **({'head':head} if head is not None else {})}
