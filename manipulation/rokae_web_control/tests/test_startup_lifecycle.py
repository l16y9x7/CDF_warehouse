"""Real subprocess tests in a temporary tree: no ROS, SDK, or system services."""
import os
from pathlib import Path
import signal
import subprocess
import sys
import tempfile
import time
import unittest


ROOT = Path(__file__).resolve().parents[1]


def running(pid):
    try:
        return Path(f'/proc/{pid}/stat').read_text().split(') ', 1)[1][0] != 'Z'
    except FileNotFoundError:
        return False


@unittest.skipUnless(sys.platform.startswith('linux'), 'Linux process groups required')
class StartupLifecycleTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.root = Path(self.temp.name)
        self.process = None
        self.external = None
        self.log = None
        self.env = dict(os.environ, TEST_ROOT=str(self.root), PATH=str(self.root)+':'+os.environ['PATH'])
        script = (ROOT/'start_control_runtime.sh').read_text()
        # Replace only external setup files. All lifecycle logic is unmodified.
        script = script.replace('/opt/ros/humble/setup.bash', str(self.root/'ros-absent.bash'))
        script = script.replace('/home/admin/agvsdk/standard_robots_amr_ros2-v1.3.0/install/setup.bash', str(self.root/'amr-absent.bash'))
        self.write('start_control_runtime.sh', script)
        self.write('systemctl', '#!/bin/bash\necho unexpected > "$TEST_ROOT/systemctl-called"\nexit 1\n')
        self.write('pgrep', '#!/bin/bash\ntest -f "$TEST_ROOT/external"\n')
        # Fail if startup ever waits for ROS discovery again.
        self.write('ros2', '#!/bin/bash\necho unexpected > "$TEST_ROOT/ros2-called"\nsleep 60\n')
        self.write('start_chassis_node.sh', '#!/bin/bash\nexec /usr/bin/python3 "$TEST_ROOT/fake_chassis.py"\n')
        self.write('fake_chassis.py', '''import os,signal,time,subprocess
from pathlib import Path
r=Path(os.environ['TEST_ROOT'])
mode=os.environ.get('CHASSIS_MODE','normal')
signal.signal(signal.SIGINT,signal.SIG_IGN)
if mode=='stubborn':signal.signal(signal.SIGTERM,signal.SIG_IGN)
if mode=='stubborn':
 child=subprocess.Popen(['/usr/bin/python3','-c','import signal,time;signal.signal(signal.SIGTERM,signal.SIG_IGN);time.sleep(60)'])
 (r/'descendant.pid').write_text(str(child.pid))
(r/'chassis.pid').write_text(str(os.getpid()))
(r/'chassis-ready').touch()
if mode=='fail':raise SystemExit(9)
while True:time.sleep(.02)
''')
        self.write('control_runtime.py', '''import os,signal,time
from pathlib import Path
r=Path(os.environ['TEST_ROOT'])
(r/'core.pid').write_text(str(os.getpid()))
(r/'core-ready').touch()
def stop(*_):
 (r/'core-stopped').touch()
 raise SystemExit(0)
signal.signal(signal.SIGTERM,stop)
mode=os.environ.get('CORE_MODE','wait')
if mode=='fail':
 deadline=time.monotonic()+1
 while not (r/'chassis-ready').exists() and time.monotonic()<deadline:time.sleep(.01)
 raise SystemExit(7)
if mode=='exit':raise SystemExit(4)
while True:time.sleep(.02)
''')

    def write(self, name, data):
        p = self.root/name
        p.write_text(data)
        p.chmod(0o700)

    def start(self, **env):
        self.log = open(self.root/'output', 'w')
        self.started = time.monotonic()
        self.process = subprocess.Popen(['/bin/bash', str(self.root/'start_control_runtime.sh')],
                                        env=dict(self.env, **env), stdout=self.log,
                                        stderr=subprocess.STDOUT, start_new_session=True)

    def wait_file(self, name, timeout=2):
        end = time.monotonic()+timeout
        while not (self.root/name).exists() and time.monotonic()<end:
            time.sleep(.01)
        self.assertTrue((self.root/name).exists(), (self.root/'output').read_text())

    def wait_exit(self, expected, timeout=5):
        self.assertEqual(self.process.wait(timeout=timeout), expected, (self.root/'output').read_text())
        return time.monotonic()-self.started

    def assert_owned_stopped(self):
        for p in self.root.glob('*.pid'):
            pid = int(p.read_text())
            deadline = time.monotonic()+1
            while running(pid) and time.monotonic()<deadline:time.sleep(.01)
            self.assertFalse(running(pid), f'{p.name}: {pid} still running')

    def test_core_failure_propagates_without_starting_chassis(self):
        self.start(CORE_MODE='fail')
        elapsed = self.wait_exit(7, 3)
        self.assertLess(elapsed, 2)
        self.assert_owned_stopped()
        self.assertFalse((self.root/'chassis.pid').exists())
        self.assertFalse((self.root/'systemctl-called').exists())

    def test_missing_chassis_does_not_stop_upper_body_or_spawn_node(self):
        self.start(CHASSIS_MODE='fail')
        self.wait_file('core-ready')
        time.sleep(.2)
        self.assertIsNone(self.process.poll())
        self.assertTrue(running(int((self.root/'core.pid').read_text())))
        self.process.terminate()
        self.wait_exit(0)
        self.assertFalse((self.root/'chassis.pid').exists())
        self.assertTrue((self.root/'core-stopped').exists())
        self.assert_owned_stopped()

    def test_core_starts_without_ros_setup_or_discovery(self):
        self.start()
        self.wait_file('core-ready')
        self.assertLess(time.monotonic()-self.started, 1.5)
        self.assertFalse((self.root/'ros2-called').exists())
        self.process.terminate()
        self.wait_exit(0)
        self.assert_owned_stopped()

    def test_reused_external_chassis_is_never_stopped(self):
        (self.root/'external').touch()
        self.external = subprocess.Popen(['/usr/bin/python3', '-c', 'import time;time.sleep(60)'], start_new_session=True)
        self.start(CORE_MODE='exit')
        self.wait_exit(4, 2)
        self.assertFalse((self.root/'chassis.pid').exists())
        self.assertIsNone(self.external.poll())

    def test_sigterm_reaches_core_directly_and_leaves_external_chassis_running(self):
        (self.root/'external').touch()
        self.external = subprocess.Popen(['/usr/bin/python3','-c','import time;time.sleep(60)'],start_new_session=True)
        self.start();self.wait_file('core-ready')
        self.assertEqual(int((self.root/'core.pid').read_text()),self.process.pid)
        self.process.terminate();self.wait_exit(0)
        self.assertIsNone(self.external.poll())
        self.assertTrue((self.root/'core-stopped').exists())

    def tearDown(self):
        # Cleanup only processes started by this test, including failed assertions.
        for p in self.root.glob('*.pid'):
            try:os.kill(int(p.read_text()), signal.SIGKILL)
            except ProcessLookupError:pass
        if self.process:
            try:os.killpg(self.process.pid, signal.SIGKILL)
            except ProcessLookupError:pass
            self.process.wait(timeout=2)
        if self.external:
            self.external.kill()
            self.external.wait(timeout=2)
        if self.log:self.log.close()
        self.temp.cleanup()


if __name__ == '__main__':
    unittest.main()
