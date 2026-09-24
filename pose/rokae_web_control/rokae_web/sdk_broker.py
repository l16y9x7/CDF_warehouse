"""Local SDK broker: one hardware owner, restartable web motion planners.

The public HTTP server is read-only. SDK calls travel exclusively over a
mode-0600 Unix socket; one web client owns the command session at a time.
"""
from __future__ import annotations

import json
import os
from pathlib import Path
import socket
import socketserver
import struct
import threading
import time
import uuid
from functools import wraps

from .backends import BackendError, XCoreRobotBackend
from .control_trace import context, fields
from .sdk_wire import SDK, encode, decode, apply_outputs


MAX_FRAME = 4 * 1024 * 1024
READ_METHODS = {"jointPos", "cartPosture", "posture", "operationState", "powerState",
                "operateMode", "toolset", "getSoftLimit", "getMechUnit", "getExtAxisInfo", "checkPath"}
WRITE_METHODS = {"setOperateMode", "setPowerState", "setMotionControlMode", "setDefaultSpeed",
                 "moveReset", "moveAppend", "moveStart", "stop", "enableDrag", "disableDrag", "XPRWModbusRTUReg", "XPRS485SendData"}
METHODS = READ_METHODS | WRITE_METHODS | {"model.calcIk", "model.calcFk"}
MODULES = {"left_arm", "right_arm", "trunk"}


def receive(sock):
    def read(count):
        pieces = bytearray()
        while len(pieces) < count:
            part = sock.recv(count - len(pieces))
            if not part:
                raise EOFError("SDK client disconnected")
            pieces.extend(part)
        return bytes(pieces)
    count = struct.unpack("!I", read(4))[0]
    if not 0 < count <= MAX_FRAME:
        raise ValueError("Invalid SDK message length")
    return json.loads(read(count))


def send(sock, message):
    raw = json.dumps(message, ensure_ascii=False, allow_nan=False, separators=(",", ":")).encode()
    if len(raw) > MAX_FRAME:
        raise ValueError("SDK message too large")
    sock.sendall(struct.pack("!I", len(raw)) + raw)


class BrokerClient:
    def __init__(self, path):
        self.path, self.sock = str(path), None
        self.lock = threading.RLock()
        self.broken = False
        self.session = uuid.uuid4().hex

    def request(self, payload):
        with self.lock:
            if self.broken:
                raise BackendError("SDK 常驻服务连接已中断；指令不会重试，请重启网页后重新回读")
            try:
                if self.sock is None:
                    self.sock = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
                    self.sock.settimeout(30)
                    self.sock.connect(self.path)
                    send(self.sock, {"op": "hello", "client": self.session})
                    hello = receive(self.sock)
                    if not hello.get("ok"):
                        raise BackendError(hello.get("error", "SDK session rejected"))
                send(self.sock, {**payload, "trace": fields()})
                return receive(self.sock)
            except (OSError, EOFError, ValueError, BackendError) as exc:
                self.broken = True
                if self.sock:
                    self.sock.close(); self.sock = None
                raise BackendError(f"SDK 常驻服务不可用（未重发指令）：{exc}") from exc

    def call(self, module, method, args):
        response = self.request({"op": "call", "module": module, "method": method, "args": encode(args)})
        returned = decode(response.get("args", []))
        for old, new in zip(args, returned):
            apply_outputs(old, new)
        if not response.get("ok"):
            raise BackendError(response.get("error", "SDK broker call failed"))
        return decode(response.get("result"))

    def close(self):
        with self.lock:
            if self.sock:
                try:
                    send(self.sock, {"op": "bye"})
                    receive(self.sock)
                except (OSError, EOFError, ValueError):
                    pass
                self.sock.close(); self.sock = None
            self.broken = True


class RemoteRobot:
    def __init__(self, client, module, prefix=""):
        self.client, self.module, self.prefix = client, module, prefix

    def model(self):
        return RemoteRobot(self.client, self.module, "model.")

    def __getattr__(self, name):
        method = self.prefix + name
        if method not in METHODS:
            raise AttributeError(f"SDK broker does not expose {method}")
        def call(*args):
            return self.client.call(self.module, method, args)
        call.__name__ = name
        return call


class BrokerRobotBackend(XCoreRobotBackend):
    """Keep web planners unchanged, but never import/connect the native SDK here."""
    def __init__(self, config, socket_path):
        super().__init__(config)
        self.client = BrokerClient(socket_path)

    def _load_sdk(self):
        return SDK

    def start_arms_synchronized(self, modules):
        response = self.client.request({'op': 'start_arms', 'modules': modules})
        if not response.get('ok'):
            raise BackendError(response.get('error', '同步启动失败'))
        return response['result']

    def _robot(self, module):
        controller = "trunk" if module == "head" else module
        if controller not in MODULES:
            raise BackendError("未知控制器")
        if controller not in self._robots:
            self._robots[controller] = RemoteRobot(self.client, controller)
        return self._robots[controller]

    def close(self):
        # Closing the web stops only its pending actions in the broker and releases
        # its command session. The hardware owner and telemetry remain alive.
        self.client.close()


def foreign_sdk_connections(config):
    """Detect another local SDK owner before the daemon's first connection."""
    own = set()
    for fd in Path('/proc/self/fd').iterdir():
        try:
            target = os.readlink(fd)
            if target.startswith('socket:['):
                own.add(target[8:-1])
        except OSError:
            pass
    addresses = {socket.inet_aton(config[k])[::-1].hex().upper()
                 for k in ('left_arm_ip', 'right_arm_ip', 'trunk_ip')}
    conflicts = []
    for row in Path('/proc/net/tcp').read_text().splitlines()[1:]:
        parts = row.split()
        address, port = parts[2].split(':')
        if (address in addresses and int(port, 16) in (6666, 7777)
                and parts[3] in ('01', '02', '03') and parts[9] not in own):
            conflicts.append(parts[2])
    return conflicts


class OwnedHardwareBackend(XCoreRobotBackend):
    def _robot(self, module):
        controller = 'trunk' if module == 'head' else module
        if controller not in self._robots and foreign_sdk_connections(self._config):
            raise BackendError("其他本机进程仍连接控制器；等待旧网页退出，不抢占 SDK 连接")
        return super()._robot(module)


def motion_request(function):
    """Let cache samplers yield before queued control-session SDK work."""
    @wraps(function)
    def wrapped(self, *args, **kwargs):
        with self._priority_lock:self._pending_requests += 1
        try:
            return function(self, *args, **kwargs)
        finally:
            with self._priority_lock:self._pending_requests -= 1
    return wrapped


class BrokerCore:
    def __init__(self, backend, audit):
        self.backend, self.audit = backend, audit
        self.session_lock = threading.Lock()
        self.session = None
        self.dirty, self.dragging = set(), set()
        self.queued = set()
        self.gripper_dirty = False
        self.cleanup_error = None
        self.stopping = False
        self._priority_lock = threading.Lock()
        self._pending_requests = 0

    def motion_pending(self):
        with self._priority_lock:return self.stopping or self._pending_requests > 0

    def acquire(self, session):
        with self.session_lock:
            if self.stopping:
                raise BackendError("SDK 服务正在退出")
            if self.cleanup_error:
                raise BackendError("上个网页会话停止未确认，请现场核对：" + self.cleanup_error)
            if self.session is not None:
                raise BackendError("已有网页持有运控会话；只读上报请使用独立 HTTP 接口")
            self.session = session
            self.audit("web_client_connected", session=session)

    @motion_request
    def call(self, request):
        module, method = request.get("module"), request.get("method")
        if module not in MODULES or method not in METHODS:
            raise BackendError("SDK 方法不在允许列表中")
        with self.backend._lock:
            if self.stopping:
                raise BackendError("SDK 服务正在退出")
            sdk = self.backend._load_sdk()
            args = decode(request.get("args", []), sdk)
            if method == 'XPRS485SendData':
                from .suction import valid_raw_request
                if module != 'left_arm' or not valid_raw_request(args, sdk):
                    raise BackendError('仅允许左臂继电器状态读取和单通道启停报文')
            robot = self.backend._robot(module)
            target = robot.model() if method.startswith("model.") else robot
            function = getattr(target, method.split('.')[-1])
            # Mark before dispatch: even a failed network reply may have executed.
            if method in ('moveAppend', 'moveStart'):
                self.dirty.add(module)
            if method == 'enableDrag':
                self.dragging.add(module)
            if method == 'XPRWModbusRTUReg' and len(args) > 1 and args[1] != 0x03:
                self.gripper_dirty = True
            started = time.monotonic()
            error, result = None, None
            try:
                result = function(*args)
            except Exception as exc:
                error = str(exc)
            out = {"ok": error is None, "error": error, "result": encode(result), "args": encode(args)}
            if method in ('stop', 'moveReset', 'moveStart'):
                self.queued.discard(module)
            if (method == 'moveAppend' and error is None and args
                    and isinstance(args[-1], dict) and not args[-1].get('ec', 0)):
                self.queued.add(module)
            self.audit("broker_sdk_call", module=module, method=method, args=request.get('args'),
                       args_after=out['args'], result=out['result'], error=error,
                       duration_ms=(time.monotonic()-started)*1000)
            return out

    @motion_request
    def start_arms(self, request):
        from .synchronized_start import start, validate_modules
        modules = request.get('modules')
        validate_modules(modules)
        with self.backend._lock:
            if self.stopping or not set(modules).issubset(self.queued):
                raise BackendError('同步指令未全部就绪，禁止同步启动')
            self.dirty.update(modules)
            self.queued.difference_update(modules)
            result = start(self.backend, modules)
            self.audit('broker_arms_synchronized', **result)
            return {'ok': True, 'result': result}

    @motion_request
    def release(self, session):
        # Do not release the command lease until old-client cleanup finishes.
        # Never disconnect the SDK; the sampler keeps using the same objects.
        with self.session_lock:
            if self.session != session:
                return
            errors = []
            with self.backend._lock:
                for module in self.dirty | self.dragging:
                    robot = self.backend._robots.get(module)
                    if robot is None:
                        continue
                    actions = [('stop', robot.stop), ('moveReset', robot.moveReset)]
                    if module in self.dragging:
                        actions.append(('disableDrag', robot.disableDrag))
                    for name, function in actions:
                        try:
                            self.backend._call(f"网页退出 {module} {name}", function)
                        except Exception as exc:
                            errors.append(str(exc))
                    try:
                        deadline = time.monotonic() + 2.0
                        while self.backend._operation_name(robot, module).lower() != 'idle':
                            if time.monotonic() >= deadline:
                                raise BackendError(f"网页退出后 {module} 停止未确认")
                            time.sleep(.05)
                    except Exception as exc:
                        errors.append(str(exc))
                if self.gripper_dirty:
                    try:
                        self.backend.gripper_stop()
                    except Exception as exc:
                        errors.append(str(exc))
            self.dirty.clear(); self.dragging.clear(); self.gripper_dirty = False
            self.queued.clear()
            self.cleanup_error = '; '.join(errors) or None
            self.session = None
            self.audit("web_client_disconnected", session=session, stop_errors=errors)


class BrokerHandler(socketserver.BaseRequestHandler):
    def handle(self):
        core, session = self.server.core, None
        try:
            if hasattr(socket, 'SO_PEERCRED'):
                _, uid, _ = struct.unpack('3i', self.request.getsockopt(socket.SOL_SOCKET, socket.SO_PEERCRED, 12))
                if uid != os.getuid():
                    raise BackendError("SDK Unix socket requires the service user")
            hello = receive(self.request)
            if hello.get('op') != 'hello' or not isinstance(hello.get('client'), str):
                raise BackendError("Missing SDK session handshake")
            proposed = hello['client']
            core.acquire(proposed)
            session = proposed
            send(self.request, {'ok': True, 'version': 'sdk-broker-v1'})
            while True:
                request = receive(self.request)
                if request.get('op') == 'bye':
                    core.release(session); session = None
                    send(self.request, {'ok': True})
                    return
                if request.get('op') not in ('call', 'start_arms'):
                    raise BackendError("Unsupported SDK request")
                trace = request.get('trace') or {}
                token = context.set({k: v for k, v in trace.items() if k in
                                     ('request_id', 'operation_id', 'operation', 'stage')})
                try:
                    try:
                        result = core.start_arms(request) if request['op'] == 'start_arms' else core.call(request)
                    except Exception as exc:
                        result = {'ok': False, 'error': str(exc)}
                    send(self.request, result)
                finally:
                    context.reset(token)
        except (EOFError, OSError):
            pass
        except Exception as exc:
            try:
                send(self.request, {'ok': False, 'error': str(exc)})
            except (OSError, ValueError):
                pass
        finally:
            if session is not None:
                core.release(session)


class BrokerServer(socketserver.ThreadingUnixStreamServer):
    daemon_threads = True

    def __init__(self, path, core):
        self.core = core
        super().__init__(str(path), BrokerHandler)
        os.chmod(path, 0o600)
