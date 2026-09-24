"""Service-manager doubles only; no service is started or stopped by tests."""
import subprocess
import tempfile
import threading
import unittest
from pathlib import Path
from types import SimpleNamespace as NS
from unittest.mock import patch

from rokae_web.backends import BackendError
from rokae_web.camera_restart import (CameraServiceRestart, SYSTEM_UNITS, USER_UNITS,
    SYSTEM_RESTART, USER_RESTART, manual_camera_processes, unit_states)
from rokae_web.http_camera import HttpCameraManager


def states(user=False):
    return {unit:dict(Id=unit,LoadState='loaded',ActiveState='active',SubState='running',MainPID='42')
            for unit in (USER_UNITS if user else SYSTEM_UNITS)}


class RestartTests(unittest.TestCase):
    def setUp(self):
        self.restart=CameraServiceRestart()
        self.state_patch=patch('rokae_web.camera_restart.unit_states',side_effect=states)
        self.state=self.state_patch.start();self.addCleanup(self.state_patch.stop)
        self.proc_patch=patch('rokae_web.camera_restart.manual_camera_processes',return_value=[])
        self.proc=self.proc_patch.start();self.addCleanup(self.proc_patch.stop)

    def test_system_and_user_groups_run_together_and_check_all_units(self):
        barrier=threading.Barrier(2)
        commands=[]
        def run(command,**kwargs):
            self.assertNotIn('shell',kwargs)
            self.assertEqual(kwargs['timeout'],120)
            commands.append(command);barrier.wait(2)
            return NS(returncode=0,stderr='')
        with patch('rokae_web.camera_restart.subprocess.run',side_effect=run):
            result=self.restart.trigger()
            self.restart.thread.join(3)
        self.assertFalse(self.restart.thread.is_alive())
        self.assertEqual(result['cameras'],['head','left_wrist','right_wrist'])
        self.assertCountEqual(commands,[SYSTEM_RESTART,USER_RESTART])
        self.assertTrue(self.restart.result['command_ok'],self.restart.result)
        self.assertEqual(set(self.restart.result['services']),set(SYSTEM_UNITS+USER_UNITS))
        self.assertEqual(self.restart.status()['recovery_scope'],'all')
        self.assertFalse(self.restart.status()['recovery_running'])

    def test_missing_unit_prevents_both_groups(self):
        self.state.side_effect=lambda user=False: {} if user else states()
        with patch.object(self.restart,'_restart') as run:self.restart._run()
        run.assert_not_called()
        self.assertIn('mui-right-wrist-rgb',self.restart.result['error'])

    def test_permission_failure_prevents_both_groups(self):
        self.state.side_effect=BackendError('sudo: password required')
        with patch.object(self.restart,'_restart') as run:self.restart._run()
        run.assert_not_called()
        self.assertFalse(self.restart.result['command_ok'])
        self.assertIn('sudo',self.restart.result['error'])

    def test_manual_driver_is_not_duplicated_or_killed(self):
        self.proc.return_value=[dict(pid=123,service=SYSTEM_UNITS[0])]
        with patch.object(self.restart,'_restart') as run:self.restart._run()
        run.assert_not_called()
        self.assertIn('PID 123',self.restart.result['error'])

    def test_one_group_failure_is_reported_even_if_other_group_succeeds(self):
        with patch.object(self.restart,'_restart',side_effect=lambda cmd:dict(ok=cmd==SYSTEM_RESTART,error='' if cmd==SYSTEM_RESTART else 'right failed')):
            self.restart._run()
        self.assertFalse(self.restart.result['command_ok'])
        self.assertIn('right failed',self.restart.result['error'])
        self.assertTrue(self.restart.result['system']['ok'])

    def test_inactive_unit_after_restart_is_failure(self):
        readings=[states(),states(True),states(),states(True)]
        readings[-1][USER_UNITS[0]]['ActiveState']='failed'
        self.state.side_effect=readings
        with patch.object(self.restart,'_restart',return_value=dict(ok=True,error='')):self.restart._run()
        self.assertFalse(self.restart.result['command_ok'])
        self.assertIn(USER_UNITS[0],self.restart.result['error'])

    def test_duplicate_is_rejected(self):
        self.restart.busy=True
        with self.assertRaisesRegex(BackendError,'正在进行'):self.restart.trigger()

    def test_timeout_and_missing_executable_report_failure(self):
        for error in (subprocess.TimeoutExpired(SYSTEM_RESTART,120),FileNotFoundError('not found')):
            with self.subTest(error=error),patch('rokae_web.camera_restart.subprocess.run',side_effect=error):
                self.assertFalse(self.restart._restart(SYSTEM_RESTART)['ok'])


class InspectionTests(unittest.TestCase):
    def test_fixed_service_status_commands_and_dedicated_sudo_probe(self):
        for user in (False,True):
            output='\n\n'.join('\n'.join(k+'='+v for k,v in row.items()) for row in states(user).values())
            with patch('rokae_web.camera_restart.subprocess.run',return_value=NS(returncode=0,stdout=output)) as run:
                self.assertEqual(unit_states(user),states(user))
                args=run.call_args.args[0]
                self.assertEqual(args[:2],('/usr/bin/systemctl','--user') if user else ('/usr/bin/sudo','-n'))
                self.assertIn('show',args)
                self.assertNotIn('restart',args)
        with patch('rokae_web.camera_restart.subprocess.run',return_value=NS(returncode=1,stderr='password required')):
            with self.assertRaisesRegex(BackendError,'install_camera_restart_permission'):unit_states()

    def test_proc_inspection_distinguishes_original_unit_from_manual_instance(self):
        with tempfile.TemporaryDirectory() as folder:
            root=Path(folder)
            for pid,unit in ((10,'vision-head-rgbd.service'),(11,'session-36.scope')):
                proc=root/str(pid);proc.mkdir()
                (proc/'cmdline').write_bytes(b'python3\0-m\0vision.rokae_runtime.driver_supervisor\0--camera\0head\0')
                (proc/'cgroup').write_text('0::/system.slice/'+unit+'\n')
            result=manual_camera_processes(root)
            self.assertEqual(result,[dict(pid=11,service=SYSTEM_UNITS[0])])

    def test_http_status_and_restart_share_all_camera_job(self):
        with tempfile.TemporaryDirectory() as folder:
            manager=HttpCameraManager({},folder)
            with patch.object(manager,'_get',side_effect=BackendError('offline')):
                status=manager.status()
            self.assertEqual(set(status),{'head','left_wrist','right_wrist'})
            self.assertTrue(all(row['recovery_scope']=='all' for row in status.values()))
            with patch.object(manager.recovery,'trigger',return_value={'accepted':True}) as trigger:
                self.assertTrue(manager.restart('all')['accepted'])
                trigger.assert_called_once()
                with self.assertRaises(BackendError):manager.restart('head')


if __name__=='__main__':unittest.main()
