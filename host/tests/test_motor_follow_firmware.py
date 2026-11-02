"""Native C controller regressions; no probe, serial port or flashing access."""
from pathlib import Path
import os
import shutil
import subprocess
import tempfile
import unittest


ROOT = Path(__file__).resolve().parents[2]
FIRMWARE = ROOT / 'device/mspm0/motor_uart'


class MotorFirmwareTests(unittest.TestCase):
    def test_native_controllers_with_address_and_undefined_sanitizers(self):
        compiler = shutil.which('cc')
        if compiler is None:
            self.skipTest('native C compiler is unavailable')
        with tempfile.TemporaryDirectory(prefix='aic-motor-c-') as output:
            for name in ('remote', 'straight', 'turn', 'follow'):
                with self.subTest(controller=name):
                    binary = Path(output) / name
                    result = subprocess.run([
                        compiler, '-std=c99', '-O1', '-g', '-Wall', '-Wextra', '-Werror',
                        '-fsanitize=address,undefined', '-fno-omit-frame-pointer', '-fno-pie', '-no-pie',
                        '-I', str(FIRMWARE), str(FIRMWARE / 'tests' / f'test_{name}.c'),
                        '-lm', '-o', str(binary),
                    ], text=True, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, timeout=60)
                    self.assertEqual(result.returncode, 0, result.stdout)
                    # These stack-only controllers allocate no heap. LSan cannot
                    # inspect threads inside the ptrace-based task sandbox.
                    env = dict(os.environ, ASAN_OPTIONS='detect_leaks=0:abort_on_error=1',
                               UBSAN_OPTIONS='halt_on_error=1:print_stacktrace=1')
                    result = subprocess.run([str(binary)], text=True, stdout=subprocess.PIPE,
                                            stderr=subprocess.STDOUT, env=env, timeout=30)
                    self.assertEqual(result.returncode, 0, result.stdout)
                    self.assertIn('PASS:', result.stdout)


if __name__ == '__main__':
    unittest.main()
