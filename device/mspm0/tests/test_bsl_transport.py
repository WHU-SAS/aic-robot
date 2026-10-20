"""Regress TI wire-format vectors and observed ROM handshake behavior."""
import struct
import contextlib
import io
import json
from pathlib import Path
import tempfile
from unittest.mock import patch
import unittest
from tools.diagnostics import mspm0_bsl as m

INFO = bytes.fromhex('00 08 19 00 31 00 01 00 01 00 00 00 00 01 00 c0 3e '
                     '60 01 00 20 01 00 00 00 01 00 00 00 0b 5f 2d d1')
REPEATED_CONNECTION = bytes.fromhex('00 08 02 00 3b 06 0d a7 f7 6b')


class Port:
    def __init__(self, replies):
        self.replies = list(replies)
        self.rx = b''
        self.sent = []

    def reset_input_buffer(self):
        self.rx = b''

    def write(self, data):
        command, reply = self.replies.pop(0)
        assert data == m.packet(command), data.hex()
        self.sent.append(command)
        self.rx += reply
        return len(data)

    def flush(self):
        pass

    def read(self, count):
        result, self.rx = self.rx[:count], self.rx[count:]
        return result


class BSLTests(unittest.TestCase):
    def test_ti_documented_command_vectors(self):
        self.assertEqual(m.packet(0x12).hex(), '800100123a6144de')
        self.assertEqual(m.packet(0x19).hex(), '80010019b2b89649')
        self.assertEqual(m.packet(0x21, b'\xff'*32).hex(),
                         '80210021' + 'ff'*32 + '02aaf03d')

    def test_response_crc_and_length_are_checked(self):
        raw = bytes.fromhex('0802003b0038029482')
        self.assertEqual(m.parse_response(raw), (0x3B, b'\0'))
        for broken in [raw[:-1], raw[:-1] + bytes([raw[-1] ^ 1]), b'\x80' + raw[1:]]:
            with self.assertRaises(ValueError):
                m.parse_response(broken)

    def test_fresh_connection_accepts_rom_buffer_address(self):
        port = Port([(0x12, b'\0'), (0x19, INFO)])
        info = m.BSL(port, lambda e: None).connect(.3)
        self.assertEqual(info['sram_start'], 0x20000160)
        self.assertEqual(info['max_buffer'], 16064)
        self.assertEqual(port.sent, [0x12, 0x19])

    def test_repeated_connection_status_does_not_pollute_device_info(self):
        port = Port([(0x12, REPEATED_CONNECTION), (0x19, INFO)])
        self.assertEqual(m.BSL(port, lambda e: None).connect(.3)['build_id'], 256)
        self.assertEqual(port.rx, b'')

    def test_core_error_stops_transaction(self):
        core = b'\x3b\x09'
        reply = b'\0\x08\x02\0' + core + struct.pack('<I', m.crc32(core))
        port = Port([(0x19, reply)])
        with self.assertRaisesRegex(RuntimeError, 'core error: 09'):
            m.BSL(port, lambda e: None).command(0x19)
        self.assertEqual(port.sent, [0x19])

    def test_mismatching_crc_never_starts_application(self):
        with tempfile.TemporaryDirectory() as tmp:
            image = struct.pack('<II', 0x20208000, 9) + b'\0'*8
            expected = m.crc32(image + b'\xff'*(1024-len(image)))
            argv = ['bsl', '--verify-only', '--start-app', '--hex', 'image.hex', '--output', tmp]
            with patch('sys.argv', argv), patch.object(m, 'load_image', return_value=image), \
                 patch.object(m.serial, 'Serial'), patch.object(m, 'BSL') as cls, \
                 contextlib.redirect_stdout(io.StringIO()):
                cls.return_value.connect.return_value = {'max_buffer': 16064}
                cls.return_value.memory_crc.side_effect = [expected ^ 1, expected]
                self.assertEqual(m.main(), 1)
                cls.return_value.command.assert_not_called()
            result = json.loads((Path(tmp)/'result.json').read_text())
            self.assertFalse(result['application_started'])
            self.assertFalse(result['erase_started'])
