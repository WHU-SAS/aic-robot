"""Compile the production remote-control/parser headers and test state transitions."""
from pathlib import Path
import shutil
import subprocess
import tempfile
import unittest
ROOT = Path(__file__).resolve().parents[3]

class MotorRemoteTests(unittest.TestCase):
    @unittest.skipUnless(shutil.which('gcc'), 'Host gcc unavailable')
    def test_parser_and_remote_state(self):
        folder = ROOT / 'device/mspm0/motor_uart'
        with tempfile.TemporaryDirectory() as tmp:
            binary = str(Path(tmp) / 'remote_test')
            subprocess.run(['gcc', '-std=c99', '-Wall', '-Wextra', '-Werror',
                            '-fsanitize=undefined,address', '-I', str(folder),
                            str(folder/'tests/test_remote.c'), '-o', binary],
                           check=True, capture_output=True)
            result = subprocess.run([binary], check=True, capture_output=True, text=True)
            self.assertIn('PASS:', result.stdout)
