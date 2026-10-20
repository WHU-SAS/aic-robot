"""Run the actual C request parser against startup-noise regressions."""
from pathlib import Path
import shutil
import subprocess
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[3]


class ParserTests(unittest.TestCase):
    @unittest.skipUnless(shutil.which('gcc'), 'Host gcc unavailable')
    def test_startup_noise_and_frame_recovery(self):
        folder = ROOT / 'device/mspm0/uart_link'
        with tempfile.TemporaryDirectory() as tmp:
            binary = str(Path(tmp) / 'parser_test')
            subprocess.run(['gcc', '-std=c99', '-Wall', '-Wextra', '-Werror',
                            '-I', str(folder), str(folder/'tests/test_parser.c'),
                            '-o', binary], check=True, capture_output=True)
            result = subprocess.run([binary], check=True, capture_output=True, text=True)
            self.assertIn('PASS:', result.stdout)
