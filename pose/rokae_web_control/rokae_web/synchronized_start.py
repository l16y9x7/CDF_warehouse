"""Dispatch queued commands to arms, or either arm and the trunk, together."""
import threading
import time

from .backends import BackendError
from .control_trace import worker_thread


def validate_modules(modules):
    # Used by the broker before touching queued state, and by direct dispatch.
    if (not isinstance(modules, list) or not 1 <= len(modules) <= 2
            or any(not isinstance(m, str) or m not in ('left_arm', 'right_arm', 'trunk')
                   for m in modules)
            or len(set(modules)) != len(modules) or modules == ['trunk']):
        raise BackendError('同步启动只接受单臂、双臂或任一手臂与躯干控制器')


def start(backend, modules):
    validate_modules(modules)
    # The caller owns backend._lock. Workers touch separate native controllers;
    # no telemetry or other command may interleave with this dispatch batch.
    robots = {name: backend._robot(name) for name in modules}
    gate = threading.Barrier(len(modules) + 1, timeout=5)
    errors, dispatched = {}, {}

    def run(name):
        try:
            gate.wait()
            dispatched[name] = time.monotonic_ns()
            backend._call(f'{name} 同步启动运动', robots[name].moveStart)
        except Exception as exc:
            errors[name] = str(exc)

    threads = []
    try:
        for name in modules:
            thread = worker_thread(target=run, args=(name,), name=f'scan-start-{name}', daemon=True)
            thread.start()
            threads.append(thread)
        gate.wait()
    except Exception as exc:
        gate.abort()
        errors['dispatch'] = str(exc)
    finally:
        for thread in threads:
            thread.join()
    if errors:
        # A failed reply can still have started motion. Stop every participant.
        for name, robot in robots.items():
            for method in ('stop', 'moveReset'):
                try:
                    backend._call(f'{name} 同步启动失败后 {method}', getattr(robot, method))
                except Exception as exc:
                    errors[f'{name}.{method}'] = str(exc)
        raise BackendError('同步启动失败: ' + '; '.join(f'{k}: {v}' for k, v in errors.items()))
    return {'dispatch_skew_ms': (max(dispatched.values()) - min(dispatched.values())) / 1e6,
            'modules': modules}
