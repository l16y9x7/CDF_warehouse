import os
from pathlib import Path
import subprocess
import tempfile
import unittest


class TestLauncher(unittest.TestCase):
    def run_functions(self, code, extra_env=None):
        script = Path(__file__).resolve().parents[1] / 'vision.sh'
        definitions = script.read_text().split('case "${1:-}" in')[0]
        with tempfile.TemporaryDirectory() as tmp:
            env = {**os.environ, 'VISION_RUNTIME_DIR': tmp, **(extra_env or {})}
            return subprocess.run(['bash', '-c', definitions + '\n' + code], env=env,
                                  text=True, capture_output=True, timeout=10)

    def test_required_owner_missing_makes_status_fail(self):
        result = self.run_functions('''
read_pid_file() { echo 99999999; }
is_mod_pid() { [[ "$2" != "vision.rokae_runtime" ]]; }
ps() { return 0; }
do_status
''', {'VISION_START_ROKAE_OWNER': '1'})
        self.assertNotEqual(result.returncode, 0)
        self.assertIn('rokae_owner not running', result.stdout)

    def test_path_python_is_resolved_without_fallback(self):
        result = self.run_functions('''
should_start_rokae_owner() { return 1; }
start_one() { [[ "$PYTHON_BIN" == /* ]] || return 1; }
do_start
''', {'PYTHON_BIN': 'python3', 'VISION_START_MEDIA': '0'})
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertNotIn('fallback', result.stdout)

    def test_invalid_pid_is_never_treated_as_a_process(self):
        for pid in ('0', '-1', '1', 'not-a-pid'):
            with self.subTest(pid=pid):
                self.assertNotEqual(self.run_functions(f'pid_alive {pid}').returncode, 0)
