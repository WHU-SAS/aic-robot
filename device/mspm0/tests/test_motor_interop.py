"""Run the actual K230 client against a native production C parser/controller."""
import ctypes
import importlib.util
from pathlib import Path
import shutil
import subprocess
import sys
import tempfile
from types import SimpleNamespace
import unittest
from unittest.mock import patch
ROOT = Path(__file__).resolve().parents[3]

class InteropTests(unittest.TestCase):
    @unittest.skipUnless(shutil.which('gcc'), 'Host gcc unavailable')
    def test_k230_bench_and_repeat_invocations(self):
        folder = ROOT/'device/mspm0/motor_uart'
        with tempfile.TemporaryDirectory() as tmp:
            library = str(Path(tmp)/'remote.so')
            subprocess.run(['gcc', '-shared', '-fPIC', '-std=c99', '-Wall', '-Wextra',
                            '-Werror', '-I', str(folder), str(folder/'tests/bridge.c'),
                            '-o', library], check=True, capture_output=True)
            native = ctypes.CDLL(library)
            native.bridge_feed.argtypes = [ctypes.c_uint8]
            native.bridge_feed.restype = ctypes.c_char_p
            native.bridge_reset()
            class Clock:
                ms = 0
                remainder = 0
                button_press_at = None
                def ticks_ms(self): return self.ms
                def ticks_us(self): return self.ms * 1000
                def ticks_diff(self, a, b): return a - b
                def sleep_ms(self, amount):
                    self.ms += amount
                    self.remainder += amount
                    while self.remainder >= 10:
                        if self.button_press_at is not None:
                            native.bridge_button(int(self.button_press_at <= self.ms < self.button_press_at + 200))
                        native.bridge_tick()
                        self.remainder -= 10
            clock = Clock()
            class UART:
                UART2 = 2
                EIGHTBITS, PARITY_NONE, STOPBITS_ONE = 8, None, 1
                def __init__(self, *args, **kwargs): self.buffer = b''
                def write(self, data):
                    for byte in data:
                        self.buffer += native.bridge_feed(byte)
                    return len(data)
                def read(self):
                    data, self.buffer = self.buffer, b''
                    return data
                def deinit(self): pass
            class FPIOA:
                UART2_TXD, UART2_RXD = 5, 6
                def set_function(self, *args): pass
            spec = importlib.util.spec_from_file_location('board_motor_client',
                ROOT/'device/k230/diagnostics/k230_motor_uart.py')
            board = importlib.util.module_from_spec(spec)
            with patch.dict(sys.modules, {'machine': SimpleNamespace(UART=UART, FPIOA=FPIOA)}):
                spec.loader.exec_module(board)
            board.time = clock
            self.assertEqual(board.crc16(b'123456789'), 0x29B1)
            with patch('builtins.print') as output:
                board.MODE = 'status'; board.main()
                # Two runs without reboot: client synchronizes MCU's sequence watermark.
                board.MODE = 'bench'; board.main(); board.main()
                board.MODE = 'stop'; board.main()
                board.MODE = 'forward'; board.main()
                # One short button press while driving, then release with heartbeats.
                clock.button_press_at = clock.ms + 5800
                board.MODE = 'button'; board.main()
                clock.button_press_at = None
                # Missing the button press must fail, stop, and never auto-retry.
                board.MODE = 'button'
                with self.assertRaises(RuntimeError): board.main()

            printed = [args[0][0] for args in output.call_args_list]
            self.assertEqual(printed.count('MOTOR_BENCH_PASS'), 2)
            self.assertIn('MOTOR_STATUS_PASS 20/20', printed)
            self.assertIn('MOTOR_STOP_PASS', printed)
            self.assertEqual(printed.count('MOTOR_FORWARD_PASS'), 1)
            self.assertEqual(printed.count('MOTOR_BUTTON_PASS'), 1)
            valid = b'R,00000001,STATUS,0,10,0,0,0,0,0,00000000'
            frame = b'@'+valid+(',%04X\n'%board.crc16(valid)).encode()
            self.assertEqual(board.decode(b'\x00noise'+frame)['run'], 0)
            with self.assertRaises(ValueError): board.decode(frame[:-5]+b'0000\n')
            self.assertEqual(board.decode(frame)['protocol_version'], 1)
            native.bridge_reset()
            with patch('builtins.print'):
                link = board.MotorLink()
                link.sync()
                self.assertEqual(link.last['protocol_version'], 2)
                for left, right in ((800, 800), (-800, -800), (-800, 800),
                                    (800, -800), (0, 800), (800, 0)):
                    before = link.drive(left, right, 400)
                    clock.sleep_ms(200)
                    state = link.expect('HB')
                    self.assertEqual(state['left_pwm'], min(abs(left), 640) * (1 if left >= 0 else -1))
                    self.assertEqual(state['right_pwm'], min(abs(right), 640) * (1 if right >= 0 else -1))
                    self.assertEqual(state['remaining_ms'], 200)
                    clock.sleep_ms(200)
                    after = link.expect('STATUS')
                    self.assertEqual((after['run'], after['pwm'], after['reason']), (0, 0, 5))
                    self.assertEqual(after['left_pwm'], 0)
                    self.assertEqual(after['right_pwm'], 0)
                    a = after['encoder_a'] - before['encoder_a']
                    b = after['encoder_b'] - before['encoder_b']
                    self.assertEqual((a > 0) - (a < 0), (left > 0) - (left < 0))
                    self.assertEqual((b > 0) - (b < 0), (right < 0) - (right > 0))
                    self.assertFalse(link.expect('HB')['run'])
                board.run_drive_once(link, (-400, 400, 300))
                board.run_drive_once(link, (0, 0, 10))
                link.last['protocol_version'] = 1
                with self.assertRaisesRegex(RuntimeError, 'protocol v2'):
                    link.drive(100, 100, 100)
                for args in ((1601, 0, 100), (0, -1601, 100), (1, 1, 11), (1, 1, 5001)):
                    with self.assertRaises(ValueError): board.encode(1, 'DRIVE', args)
                link.close()
