"""USBDBG takeover uses v2 truth even when legacy SCRIPT_RUNNING says idle."""
import struct
import unittest
from unittest.mock import patch

from host.transport.download_calibration import CameraLink
from tools.diagnostics import test_k230_motor_uart as motor


class FakeSerial:
    def __init__(self, capabilities=(2, 4095), running=True, stop_works=False,
                 interrupt_works=True):
        self.capabilities = capabilities
        self.running = running
        self.stop_works, self.interrupt_works = stop_works, interrupt_works
        self.pending = bytearray()
        self.commands, self.payloads, self.statuses = [], [], []
        self.awaiting = None
        self.resets = 0

    def write(self, data):
        if self.awaiting is not None:
            code, size = self.awaiting
            self.awaiting = None
            assert len(data) == size
            self.payloads.append((code, data))
            if code == 0xA2 and self.capabilities is not None:
                self.pending.extend(struct.pack('<II', *self.capabilities))
            if code == 0x11 and data == b'\x03' and self.interrupt_works:
                self.running = False
        else:
            prefix, code, size = struct.unpack('<BBI', data)
            assert prefix == 0x30
            self.commands.append((code, size))
            if code in (0xA2, 0x11):
                self.awaiting = code, size
            elif code == 0x06 and self.stop_works:
                self.running = False
            elif code in (0xA4, 0x87):
                # The regression firmware's legacy query always lies idle.
                value = int(self.running) if code == 0xA4 else 0
                self.statuses.append((code, value))
                self.pending.extend(struct.pack('<I', value))
            elif code == 0x8E:
                self.pending.extend(struct.pack('<I', 0))
        return len(data)

    def flush(self):
        pass

    def read(self, size):
        result = bytes(self.pending[:size])
        del self.pending[:size]
        return result

    def reset_input_buffer(self):
        self.resets += 1
        self.pending.clear()


class ScriptStopTests(unittest.TestCase):
    def stop(self, port):
        link = CameraLink.__new__(CameraLink)
        link.port = port
        clock = [0.]
        def sleep(seconds):
            clock[0] += seconds
        with patch.object(motor.time, 'monotonic', side_effect=lambda: clock[0]), \
                patch.object(motor.time, 'sleep', side_effect=sleep):
            motor.stop_previous_script(link)
        return clock[0]

    def test_v2_real_running_overrides_legacy_false_idle_and_interrupts(self):
        port = FakeSerial()
        elapsed = self.stop(port)
        self.assertIn((0xA2, 1), port.commands)
        self.assertIn((0x11, 1), port.commands)
        self.assertEqual(port.payloads, [(0xA2, b'\x00'), (0x11, b'\x03')])
        self.assertEqual(port.statuses[0], (0xA4, 1))
        self.assertTrue(all(code == 0xA4 for code, unused in port.statuses))
        self.assertFalse(port.running)
        self.assertGreaterEqual(elapsed, .7)

    def test_unsupported_capabilities_retains_legacy_stop_and_status(self):
        for capabilities in (None, (1, 0)):
            with self.subTest(capabilities=capabilities):
                port = FakeSerial(capabilities=capabilities, stop_works=True)
                self.stop(port)
                self.assertIn((0x06, 0), port.commands)
                self.assertTrue(all(code == 0x87 for code, unused in port.statuses))
                self.assertEqual(port.payloads, [(0xA2, b'\x00')])
                self.assertEqual(port.resets, int(capabilities is None))

    def test_v2_already_idle_does_not_send_terminal_interrupt(self):
        port = FakeSerial(running=False)
        self.stop(port)
        self.assertEqual(port.payloads, [(0xA2, b'\x00')])
        self.assertTrue(all(value == 0 for unused, value in port.statuses))

    def test_v2_unstopped_script_cannot_fall_back_to_false_legacy_idle(self):
        for flags, interrupt_works in ((4095 & ~(1 << 9), True), (4095, False)):
            with self.subTest(flags=flags):
                port = FakeSerial(capabilities=(2, flags), interrupt_works=interrupt_works)
                with self.assertRaisesRegex(TimeoutError, 'Previous K230 script did not stop'):
                    self.stop(port)
                self.assertTrue(all(code == 0xA4 for code, unused in port.statuses))
                self.assertEqual(sum(code == 0x11 for code, unused in port.payloads),
                                 int(bool(flags & (1 << 9))))


if __name__ == '__main__':
    unittest.main()
