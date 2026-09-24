"""Restart the original three-camera boot services, never arbitrary commands."""
import copy
import os
import subprocess
import threading
from concurrent.futures import ThreadPoolExecutor
from datetime import datetime
from pathlib import Path

from .backends import BackendError


SYSTEM_UNITS = (
    'vision-head-rgbd.service', 'vision-head-synced-rgbd.service',
    'vision-head-owner.service', 'vision-head-camera.service',
    'vision-head-media.service', 'vision-left-wrist-rgb.service',
)
USER_UNITS = ('mui-right-wrist-rgb.service',)
SYSTEM_RESTART = ('/usr/bin/sudo', '-n', '/usr/bin/systemctl', 'restart', *SYSTEM_UNITS)
USER_RESTART = ('/usr/bin/systemctl', '--user', 'restart', *USER_UNITS)
CAMERAS = ('head', 'left_wrist', 'right_wrist')


def unit_states(user=False):
    units = USER_UNITS if user else SYSTEM_UNITS
    command = (*(('/usr/bin/systemctl', '--user') if user else ('/usr/bin/sudo', '-n', '/usr/bin/systemctl')), 'show', *units,
               '--property=Id,LoadState,ActiveState,SubState,MainPID')
    done = subprocess.run(command, capture_output=True, text=True, timeout=10)
    if done.returncode:
        hint = '' if user else '；请先运行 system/install_camera_restart_permission.sh 安装固定重启命令授权'
        raise BackendError('读取摄像头服务失败：' + done.stderr.strip()[:1000] + hint)
    records = {}
    for block in done.stdout.strip().split('\n\n'):
        row = dict(line.split('=', 1) for line in block.splitlines() if '=' in line)
        if row.get('Id') in units:
            records[row['Id']] = row
    return records


def manual_camera_processes(proc=Path('/proc')):
    """Do not start a second driver while a manual instance owns the device."""
    modules = {
        'vision.rokae_runtime.driver_supervisor': 'vision-head-rgbd.service',
        'vision.tianji_runtime.synced_rgbd_forwarder': 'vision-head-synced-rgbd.service',
        'vision.rokae_runtime': 'vision-head-owner.service',
        'vision.camera_app': 'vision-head-camera.service',
        'vision.media_app': 'vision-head-media.service',
        'vision.rokae_runtime.realsense_color_publisher': 'vision-left-wrist-rgb.service',
    }
    found = []
    for path in proc.iterdir():
        if not path.name.isdigit():
            continue
        try:
            if path.stat().st_uid != os.getuid():
                continue
            args = path.joinpath('cmdline').read_bytes().decode(errors='replace').strip('\0').split('\0')
            unit = None
            if '-m' in args:
                index = args.index('-m') + 1
                unit = modules.get(args[index]) if index < len(args) else None
            if '/home/admin/mui/rokae_web_control/camera_right_rgb.py' in args:
                unit = USER_UNITS[0]
            if unit and ('/'+unit) not in path.joinpath('cgroup').read_text():
                found.append(dict(pid=int(path.name), service=unit))
        except (FileNotFoundError, PermissionError, ProcessLookupError):
            continue
    return found


class CameraServiceRestart:
    def __init__(self):
        self.lock = threading.Lock()
        self.busy = False
        self.result = None
        self.thread = None

    def status(self):
        with self.lock:
            return dict(recovery_running=self.busy, recovery_result=copy.deepcopy(self.result),
                        recovery_available=True, recovery_scope='all',
                        recovery_version='camera-restart-v1')

    def trigger(self):
        with self.lock:
            if self.busy:
                raise BackendError('三路摄像头服务重启正在进行，请等待完成')
            self.busy, self.result = True, None
            self.thread = threading.Thread(target=self._run, name='rokae-camera-services-restart', daemon=True)
            self.thread.start()
        return dict(accepted=True, cameras=list(CAMERAS),
                    message='已提交头部、左腕和右腕摄像头服务重启')

    @staticmethod
    def _restart(command):
        try:
            result = subprocess.run(command, capture_output=True, text=True, timeout=120)
            return dict(ok=result.returncode == 0, error=result.stderr.strip()[:1000] if result.returncode else '')
        except subprocess.TimeoutExpired:
            return dict(ok=False, error='等待摄像头服务重启超时；systemd任务可能仍在执行，请查看服务状态')
        except OSError as exc:
            return dict(ok=False, error=str(exc))

    def _run(self):
        result = dict(command_ok=False, cameras=list(CAMERAS), services={}, error='')
        try:
            before = {**unit_states(), **unit_states(user=True)}
            missing = [unit for unit in (*SYSTEM_UNITS, *USER_UNITS)
                       if before.get(unit, {}).get('LoadState') != 'loaded']
            if missing:
                raise BackendError('摄像头开机服务未加载：' + '、'.join(missing) + '；请先运行摄像头重启权限安装脚本')
            # The system status read above exercises the same dedicated sudo
            # rule before submitting EITHER system or user restart jobs.
            manual = manual_camera_processes()
            if manual:
                result['manual_processes'] = manual
                raise BackendError('检测到服务之外的手动相机进程（PID ' + ', '.join(str(p['pid']) for p in manual)
                                   + '），请先退出对应手动启动进程，再重启三路服务，避免重复占用设备')
            result['before'] = before
            # Both service groups are submitted together. This never touches MUI
            # control, robot SDK, navigation, health timers or autostart settings.
            with ThreadPoolExecutor(max_workers=2) as pool:
                system = pool.submit(self._restart, SYSTEM_RESTART)
                user = pool.submit(self._restart, USER_RESTART)
                result['system'], result['user'] = system.result(), user.result()
            result['services'] = {**unit_states(), **unit_states(user=True)}
            errors = [row['error'] or '服务重启命令失败' for row in (result['system'], result['user']) if not row['ok']]
            inactive = [unit for unit in (*SYSTEM_UNITS, *USER_UNITS)
                        if result['services'].get(unit, {}).get('ActiveState') != 'active']
            if inactive:
                errors.append('重启后服务未运行：' + '、'.join(inactive))
            result['error'] = '；'.join(errors)
            result['command_ok'] = not errors
        except (BackendError, OSError, subprocess.TimeoutExpired) as exc:
            result['error'] = str(exc)
        finally:
            result['finished_at'] = datetime.now().astimezone().isoformat(timespec='milliseconds')
            with self.lock:
                self.result, self.busy = result, False

    def close(self):
        if self.thread is not None:
            self.thread.join(timeout=1)
