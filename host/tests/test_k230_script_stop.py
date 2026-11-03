"""USBDBG stops once and waits for cleanup with authoritative v2 busy polls.

Protocol source: https://github.com/kendryte/canmv-vscode-extension/blob/main/
native/go/internal/boardprotocol/v2.go -- ScriptRunning uses the narrower IDE
lifecycle signal; ScriptBusy uses ScriptStatus, which also covers REPL work.
"""
import struct
import unittest
from unittest.mock import patch

from host.transport.download_calibration import CameraLink
from tools.diagnostics import test_k230_motor_uart as motor


class FakeSerial:
    def __init__(self, capabilities=(2, 4095), running=True, stop_works=False,
                 interrupt_works=True, legacy_running=False):
        self.capabilities = capabilities
        self.running = running
        self.stop_works, self.interrupt_works = stop_works, interrupt_works
        self.legacy_running = legacy_running
        self.pending = bytearray()
        self.stdout = bytearray()
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
                value = int(self.running) if code == 0xA4 else int(self.legacy_running)
                self.statuses.append((code, value))
                self.pending.extend(struct.pack('<I', value))
            elif code == 0x8E:
                self.pending.extend(struct.pack('<I', len(self.stdout)))
            elif code == 0x8F:
                self.pending.extend(self.stdout[:size])
                del self.stdout[:size]
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


class CleanupSerial(FakeSerial):
    """A second interrupt during finally discards the current run's archive."""
    def __init__(self, ide_script, cleanup_polls=4):
        super().__init__(stop_works=False, interrupt_works=False,
                         legacy_running=ide_script)
        self.ide_script=ide_script
        self.cleanup_polls=cleanup_polls
        self.cleaning=self.archived=self.archive_lost=False
        self.interrupts=[]

    def interrupt(self, source):
        self.interrupts.append(source)
        if self.cleaning:
            self.archive_lost=True
            self.running=False
        else:
            self.cleaning=True
            # SCRIPT_STOP has released the narrow IDE lifecycle, but the
            # authoritative busy signal remains true throughout finally.
            self.legacy_running=False

    def write(self,data):
        if self.awaiting is not None:
            if self.awaiting[0]==0x11 and data==b'\x03':self.interrupt('terminal')
        else:
            unused,code,size=struct.unpack('<BBI',data)
            if code==0x06 and self.ide_script:self.interrupt('script_stop')
            if code==0xA4 and self.cleaning and self.running:
                self.cleanup_polls-=1
                if self.cleanup_polls==0:
                    self.archived=True
                    self.running=False
                    self.stdout.extend(b'ARCHIVE_DONE current_run\n')
        return super().write(data)


class ScriptStopTests(unittest.TestCase):
    def stop(self, port, stdout=None):
        link = CameraLink.__new__(CameraLink)
        link.port = port
        if stdout is not None:
            output=link.output
            def retain_output():
                value=output()
                stdout.append(value)
                return value
            link.output=retain_output
        clock = [0.]
        def sleep(seconds):
            clock[0] += seconds
        try:
            with patch.object(motor.time, 'monotonic', side_effect=lambda: clock[0]), \
                    patch.object(motor.time, 'sleep', side_effect=sleep):
                motor.stop_previous_script(link)
        finally:
            self.stop_elapsed=clock[0]
        return clock[0]

    def test_v2_real_running_overrides_legacy_false_idle_and_interrupts(self):
        port = FakeSerial()
        elapsed = self.stop(port)
        self.assertIn((0xA2, 1), port.commands)
        self.assertIn((0x11, 1), port.commands)
        self.assertEqual(port.payloads, [(0xA2, b'\x00'), (0x11, b'\x03')])
        self.assertEqual(port.statuses[0], (0xA4, 1))
        self.assertEqual(port.statuses[1], (0x87, 0))
        self.assertTrue(all(code == 0xA4 for code, unused in port.statuses[2:]))
        self.assertNotIn((0x06, 0),port.commands)
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
        self.assertNotIn((0x06, 0),port.commands)

    def test_v2_ide_stop_allows_finally_to_archive_without_a_second_interrupt(self):
        port=CleanupSerial(ide_script=True,cleanup_polls=105)
        stdout=[]
        elapsed=self.stop(port,stdout)
        self.assertEqual(port.interrupts,['script_stop'])
        self.assertEqual(sum(code==0x06 for code,unused in port.commands),1)
        self.assertFalse(any(code==0x11 for code,unused in port.commands))
        self.assertTrue(port.archived)
        self.assertFalse(port.archive_lost)
        self.assertIn(b'ARCHIVE_DONE current_run\n',b''.join(stdout))
        self.assertGreaterEqual(elapsed,11.)
        self.assertTrue(all(code==0xA4 for code,unused in port.statuses[2:]))

    def test_v2_repl_uses_one_terminal_interrupt_then_waits_for_its_finally(self):
        port=CleanupSerial(ide_script=False,cleanup_polls=6)
        stdout=[]
        self.stop(port,stdout)
        self.assertEqual(port.interrupts,['terminal'])
        self.assertNotIn((0x06, 0),port.commands)
        self.assertEqual(sum(code==0x11 for code,unused in port.commands),1)
        self.assertTrue(port.archived)
        self.assertFalse(port.archive_lost)
        self.assertIn(b'ARCHIVE_DONE current_run\n',b''.join(stdout))

    def test_v2_unstopped_script_cannot_fall_back_to_false_legacy_idle(self):
        for flags, interrupt_works in ((4095 & ~(1 << 9), True), (4095, False)):
            with self.subTest(flags=flags):
                port = FakeSerial(capabilities=(2, flags), interrupt_works=interrupt_works)
                with self.assertRaisesRegex(TimeoutError, 'Previous K230 script did not stop'):
                    self.stop(port)
                polls=port.statuses[2:] if flags & (1 << 9) else port.statuses[1:]
                self.assertTrue(all(code == 0xA4 for code, unused in polls))
                self.assertGreaterEqual(self.stop_elapsed,12.)
                self.assertLessEqual(self.stop_elapsed,12.2)
                self.assertEqual(sum(code == 0x11 for code, unused in port.payloads),
                                 int(bool(flags & (1 << 9))))


if __name__ == '__main__':
    unittest.main()
