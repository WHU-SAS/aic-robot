#!/usr/bin/env python3
"""MSPM0G3507 UART BSL: inspect, range erase, program, CRC verify, start.
Protocol: TI SLAU887A sections 4.2-4.4, https://www.ti.com/lit/ug/slau887a/slau887a.pdf
Uses the default BSL password once; never attempts factory reset or mass erase.
Run through scripts/mspm0-tools to use pyserial and IntelHex.
"""
import argparse
import binascii
from datetime import datetime
from pathlib import Path
import hashlib
import json
import struct
import time
import serial
from intelhex import IntelHex

ROOT = Path(__file__).resolve().parents[2]
DEFAULT_PORT = '/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0'


def crc32(data):
    # TI CRC32: reflected polynomial, seed FFFFFFFF, no final XOR.
    return binascii.crc32(data) ^ 0xFFFFFFFF


def packet(command, payload=b''):
    core = bytes([command]) + payload
    return b'\x80' + struct.pack('<H', len(core)) + core + struct.pack('<I', crc32(core))


def parse_response(raw):
    if len(raw) < 8 or raw[0] != 0x08:
        raise ValueError('Invalid BSL response header')
    size = struct.unpack_from('<H', raw, 1)[0]
    if not 1 <= size <= 4096 or len(raw) != size + 7:
        raise ValueError('Invalid BSL response length')
    core = raw[3:-4]
    if crc32(core) != struct.unpack_from('<I', raw, len(raw)-4)[0]:
        raise ValueError('BSL response CRC mismatch')
    return core[0], core[1:]


def load_image(path):
    image = IntelHex(str(path))
    segments = image.segments()
    if len(segments) != 1 or segments[0][0] != 0:
        raise ValueError('Expected a contiguous Main Flash image starting at zero')
    end = segments[0][1]
    if not 8 <= end <= 0x20000 or end % 8:
        raise ValueError('Invalid G3507 image size or 8-byte alignment')
    data = image.tobinstr()
    sp, reset = struct.unpack_from('<II', data)
    if not 0x20200000 < sp <= 0x20208000 or sp % 8 or not reset & 1 or reset >= end:
        raise ValueError('Invalid G3507 vector table')
    return data


class BSL:
    def __init__(self, port, log):
        self.port, self.log = port, log

    def read_exact(self, count, timeout):
        deadline = time.monotonic() + timeout
        data = bytearray()
        while len(data) < count:
            if time.monotonic() >= deadline:
                raise TimeoutError('BSL response timeout')
            data.extend(self.port.read(count - len(data)))
        return bytes(data)

    def command(self, cmd, payload=b'', response=True, timeout=3):
        self.log({'command': hex(cmd), 'payload_bytes': len(payload)})
        self.port.write(packet(cmd, payload))
        self.port.flush()
        ack = self.read_exact(1, timeout)[0]
        if ack != 0:
            raise RuntimeError(f'BSL transport NACK 0x{ack:02x} for command 0x{cmd:02x}')
        if not response:
            return None
        header = self.read_exact(3, timeout)
        if header[0] != 8:
            raise RuntimeError('Invalid BSL response header')
        count = struct.unpack_from('<H', header, 1)[0]
        if not 1 <= count <= 4096:
            raise RuntimeError('Invalid BSL response length')
        kind, data = parse_response(header + self.read_exact(count + 4, timeout))
        if kind == 0x3B and data != b'\0':
            raise RuntimeError('BSL core error: ' + data.hex())
        if kind == 0x3A:
            raise RuntimeError('BSL Flash detailed error: ' + data.hex())
        return kind, data

    def status(self, cmd, payload=b''):
        if self.command(cmd, payload) != (0x3B, b'\0'):
            raise RuntimeError(f'Unexpected success response to 0x{cmd:02x}')

    def connect(self, wait):
        deadline = time.monotonic() + wait
        last = ''
        while time.monotonic() < deadline:
            try:
                self.port.reset_input_buffer()
                self.command(0x12, response=False, timeout=.4)
                # An already connected ROM appends MSG 06 to Connection.
                # Consume and validate that packet before requesting device info.
                extra = self.port.read(1)
                if extra:
                    header = extra + self.read_exact(2, 1)
                    if header[0] != 8:
                        raise RuntimeError('Unexpected Connection reply')
                    size = struct.unpack_from('<H', header, 1)[0]
                    if not 1 <= size <= 4096:
                        raise RuntimeError('Invalid Connection reply length')
                    reply = parse_response(header + self.read_exact(size+4, 1))
                    if reply not in [(0x3B, b'\x00'), (0x3B, b'\x06')]:
                        raise RuntimeError('Unexpected Connection core status')
                    self.log({'connection_core_status': reply[1].hex()})
                kind, data = self.command(0x19)
                if kind != 0x31 or len(data) != 24:
                    raise RuntimeError('Unexpected device-info reply')
                info = dict(zip(['command_interpreter', 'build_id', 'application_version',
                                 'plugin_version', 'max_buffer', 'sram_start', 'bcr_config_id',
                                 'bsl_config_id'], struct.unpack('<HHIHHIII', data)))
                # TI's ROM device-info example uses 0x20000160, distinct
                # from this application's linked SRAM address 0x20200000.
                ram = info['sram_start']
                if not 133 <= info['max_buffer'] <= 32768 or ram % 4 or not (
                    0x20000000 <= ram < 0x20008000 or 0x20200000 <= ram < 0x20208000
                ):
                    raise RuntimeError('Invalid BSL buffer metadata')
                return info
            except (TimeoutError, RuntimeError, ValueError) as exc:
                last = str(exc)
                self.log({'connection_error': last})
                time.sleep(.15)
        raise TimeoutError('No valid BSL device-info response: ' + last)

    def memory_crc(self, address, size):
        if not 1024 <= size <= 65536:
            raise ValueError('BSL CRC length must be 1-64 KiB')
        kind, data = self.command(0x26, struct.pack('<II', address, size))
        if kind != 0x32 or len(data) != 4:
            raise RuntimeError('Unexpected memory CRC response')
        return struct.unpack('<I', data)[0]


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--port', default=DEFAULT_PORT)
    ap.add_argument('--hex', type=Path)
    mode = ap.add_mutually_exclusive_group()
    mode.add_argument('--execute', action='store_true', help='Program, verify, then start application')
    mode.add_argument('--verify-only', action='store_true', help='Verify existing image without erase or write')
    ap.add_argument('--start-app', action='store_true', help='Start after successful --verify-only')
    ap.add_argument('--wait-bsl', type=float, default=90)
    ap.add_argument('--output', type=Path)
    args = ap.parse_args()
    if (args.execute or args.verify_only) and args.hex is None:
        ap.error('--execute/--verify-only requires --hex')
    if args.start_app and not args.verify_only:
        ap.error('--start-app requires --verify-only')
    data = load_image(args.hex) if args.hex else None
    out = args.output or ROOT / 'data/logs' / datetime.now().strftime('mspm0_bsl_%Y%m%d_%H%M%S')
    out.mkdir(parents=True, exist_ok=True)
    result = {'status': 'failed', 'port': args.port, 'erase_started': False,
              'application_started': False, 'nonmain_write_commands': False}
    events = []
    def log(event):
        events.append({'host_monotonic': time.monotonic(), **event})
        if 'connection_error' in event and (len(events) < 8 or len(events) % 20 == 0):
            print('WAIT_BSL:', event['connection_error'], flush=True)
    port = None
    try:
        port = serial.Serial(port=None, baudrate=9600, timeout=.05, write_timeout=3, exclusive=True)
        port.dtr = False
        port.rts = False
        port.port = args.port
        port.open()
        bsl = BSL(port, log)
        print('READY: 请按住BSL，按下并松开RST，再松开BSL。正在等待连接。', flush=True)
        result['device_info'] = bsl.connect(args.wait_bsl)
        print('BSL_CONNECTED', json.dumps(result['device_info']), flush=True)
        if not (args.execute or args.verify_only):
            result['status'] = 'inspected'
            return 0
        # A single documented default-password attempt. Failure stops all writes.
        bsl.status(0x21, b'\xff' * 32)
        end = (len(data) + 1023) // 1024 * 1024
        # Preserve and compare CRC of all Main Flash outside the erased pages.
        outside = [(end, min(65536, 0x20000-end))]
        if end + outside[0][1] < 0x20000:
            outside.append((end+outside[0][1], 0x20000-end-outside[0][1]))
        outside = [(a, n) for a, n in outside if n]
        prior = []
        result.update(image_sha256=hashlib.sha256(data).hexdigest(), image_bytes=len(data))
        if args.execute:
            prior = [bsl.memory_crc(a, n) for a, n in outside]
            result['outside_crc_before'] = prior
            result.update(erase_started=True, erased_range=[0, end])
            bsl.status(0x23, struct.pack('<II', 0, end-1))
            for address in range(0, len(data), 128):
                bsl.status(0x20, struct.pack('<I', address) + data[address:address+128])
                print('PROGRAM', min(address+128, len(data)), '/', len(data), flush=True)
            # Verify through a fresh transport/ROM handshake, as used during
            # the independent hardware recheck. Never accept a mismatching CRC.
            result['program_completed'] = True
            port.close()
            port.open()
            bsl = BSL(port, log)
            result['verification_device_info'] = bsl.connect(3)
            bsl.status(0x21, b'\xff' * 32)
            result['verification_session_refreshed'] = True
        time.sleep(.1)
        expected_crc = crc32(data + b'\xff' * (end-len(data)))
        image_crcs = [bsl.memory_crc(0, end), bsl.memory_crc(0, end)]
        result['image_crcs'] = image_crcs
        if image_crcs != [expected_crc, expected_crc]:
            raise RuntimeError(f'Image verification CRC mismatch: {image_crcs} != {expected_crc:08x}')
        result.update(crc_verified=True, image_crc32=f'{expected_crc:08x}')
        if args.execute:
            after = [bsl.memory_crc(a, n) for a, n in outside]
            result['outside_crc_after'] = after
            if after != prior:
                raise RuntimeError('Main Flash outside programmed pages changed')
            result.update(outside_main_flash_crc_unchanged=True, outside_crc_ranges=outside)
        (out/'verification.json').write_text(json.dumps(result, indent=2)+'\n')
        if args.execute or args.start_app:
            bsl.command(0x40, response=False)
            result['application_started'] = True
            port.baudrate = 115200
            capture = bytearray()
            deadline = time.monotonic()+2
            while time.monotonic() < deadline:
                capture.extend(port.read(4096))
            (out/'application_uart0.log').write_bytes(capture)
            print(capture.decode(errors='replace'), flush=True)
            result['uart_ready_seen'] = b'MSPM0_UART1_TEST_READY' in capture
        result['status'] = 'passed'
        return 0
    except (OSError, ValueError, RuntimeError, TimeoutError, serial.SerialException, KeyboardInterrupt) as exc:
        result['error'] = str(exc)
        print('BSL_FAILED:', exc, flush=True)
        return 1
    finally:
        if port is not None and port.is_open:
            port.close()
        (out/'events.json').write_text(json.dumps(events, indent=2)+'\n')
        (out/'result.json').write_text(json.dumps(result, indent=2)+'\n')
        print('RECORD', out, flush=True)


if __name__ == '__main__':
    raise SystemExit(main())
