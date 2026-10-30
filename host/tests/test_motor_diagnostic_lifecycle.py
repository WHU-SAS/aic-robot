import json
from pathlib import Path
import struct
import tempfile
from types import SimpleNamespace
import unittest
from unittest.mock import patch
from tools.diagnostics import test_k230_motor_uart as motor


class MotorDiagnosticLifecycleTests(unittest.TestCase):
    def run_status(self, restore_error=None, busy=False):
        calls = []
        class Link:
            def __init__(self, port): self.port = SimpleNamespace(close=lambda: calls.append('close'))
            def command(self, *args): pass
            def read(self, size): return struct.pack('<III', 1, 8, 67)
            def execute(self, source, timeout):
                calls.append('status_script')
                yield b'MOTOR_STATE {"protocol_version": 2, "run": 0, "pwm": 0}'
                yield b'MOTOR_STATUS_PASS 20/20'
        with tempfile.TemporaryDirectory() as folder, patch.object(motor, 'CameraLink', Link), \
                patch.object(motor, 'stop_previous_script', side_effect=lambda link: calls.append('idle')), \
                patch.object(motor.subprocess, 'run', return_value=SimpleNamespace(returncode=0 if busy else 1)), \
                patch.object(motor, 'restore_listener', side_effect=restore_error,
                             return_value=['AIC_PICK {"event":"voice_ready"}']) as restore:
            code = motor.main(['--port', '/test-port', '--output', folder])
            result = json.loads((Path(folder)/'result.json').read_text())
            return code, result, calls, restore.call_count

    def test_default_only_queries_status_and_restores_listener(self):
        code, result, calls, restored = self.run_status()
        self.assertEqual(code, 0)
        self.assertEqual(result['mode'], 'status')
        self.assertTrue(result['listener_restored'])
        self.assertFalse(result['servo_commands_sent'])
        self.assertEqual(calls, ['idle', 'status_script', 'close'])
        self.assertEqual(restored, 1)

    def test_busy_port_is_not_opened_or_interrupted(self):
        code, result, calls, restored = self.run_status(busy=True)
        self.assertEqual(code, 1)
        self.assertIn('busy', result['error'])
        self.assertEqual(calls, [])
        self.assertEqual(restored, 0)

    def test_listener_failure_is_not_reported_as_complete_success(self):
        code, result, calls, restored = self.run_status(restore_error=TimeoutError('no voice_ready'))
        self.assertEqual(code, 1)
        self.assertEqual(result['status'], 'listener_restore_failed')
        self.assertEqual(result['motor_test_status'], 'passed')
        self.assertFalse(result['listener_restored'])
        self.assertEqual(calls[-1], 'close')


if __name__ == '__main__': unittest.main()
