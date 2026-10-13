#!/usr/bin/env python3
"""Export K230 calibration files through official USBDBG; never modify originals."""
import argparse
import base64
import hashlib
import json
from pathlib import Path
import struct
import time
import serial

PORT = '/dev/serial/by-id/usb-Kendryte_CanMV_001000000-if00'

class CameraLink:
    def __init__(self, port):
        self.port = serial.Serial(port, 921600, timeout=2, exclusive=True)
    def command(self, code, size=0):
        self.port.write(struct.pack('<BBI', 0x30, code, size))
        self.port.flush()
    def read(self, size):
        data = bytearray()
        while len(data) < size:
            chunk = self.port.read(size - len(data))
            if not chunk:
                raise TimeoutError('Incomplete K230 response')
            data.extend(chunk)
        return bytes(data)
    def output(self):
        self.command(0x8E, 4)
        size = struct.unpack('<I', self.read(4))[0]
        if size > 4 * 1024 * 1024:
            raise RuntimeError('Invalid stdout length; serial protocol out of sync')
        data = bytearray()
        while size:
            take = min(size, 4096)
            self.command(0x8F, take)
            data.extend(self.read(take))
            size -= take
        return bytes(data)
    def execute(self, script, timeout=45):
        self.command(0x06)
        time.sleep(.15)
        self.output()
        payload = script.encode()
        if len(payload) % 64 == 0:
            payload += b'\n'
        self.command(0x05, len(payload))
        time.sleep(.06)
        self.port.write(payload)
        self.port.flush()
        start = time.monotonic()
        pending = b''
        while time.monotonic() - start < timeout:
            pending += self.output()
            while b'\n' in pending:
                line, pending = pending.split(b'\n', 1)
                line = line.rstrip(b'\r')
                if line == b'EXPORT_DONE':
                    return
                yield line
            time.sleep(.005)
        raise TimeoutError('K230 export did not finish')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--port', default=PORT)
    parser.add_argument('--list', action='store_true')
    parser.add_argument('--session')
    parser.add_argument('--output', type=Path, default=Path(__file__).parent/'photos')
    args = parser.parse_args()
    link = CameraLink(args.port)
    try:
        link.command(0x80, 12)
        print('FIRMWARE', struct.unpack('<III', link.read(12)), flush=True)
        listing = """import os,json
print('LIST '+json.dumps([(n,len([f for f in os.listdir('/sdcard/calibration/'+n) if f.endswith('.jpg') and '.partial.' not in f])) for n in sorted(os.listdir('/sdcard/calibration')) if n.startswith('session_')]))
print('EXPORT_DONE')
"""
        sessions = None
        for line in link.execute(listing):
            if line.startswith(b'LIST '):
                sessions = json.loads(line[5:])
            else:
                print(line.decode(errors='replace'))
        if sessions is None:
            raise RuntimeError('Cannot list calibration sessions')
        print('SESSIONS', json.dumps(sessions), flush=True)
        if args.list:
            return
        session = args.session or next((n for n,c in reversed(sessions) if c), None)
        if session is None or session not in [n for n,_ in sessions]:
            raise RuntimeError('No captured session selected')
        remote = '/sdcard/calibration/' + session
        files = None
        for line in link.execute("import os,json\nprint('FILES '+json.dumps(sorted(os.listdir(" + repr(remote) + "))))\nprint('EXPORT_DONE')\n"):
            if line.startswith(b'FILES '):
                files = json.loads(line[6:])
        if files is None:
            raise RuntimeError('Cannot list captured files')
        files = [n for n in files if n.endswith(('.jpg', '.json', '.jsonl')) and '.partial.' not in n]
        destination = args.output / session
        destination.mkdir(parents=True, exist_ok=True)
        for index, name in enumerate(files):
            if Path(name).name != name:
                raise RuntimeError('Invalid remote filename')
            source = remote + '/' + name
            script = """import binascii,time
import uhashlib as hashlib
h=hashlib.sha256()
offset=0
with open(%r,'rb') as f:
 while True:
  b=f.read(1536)
  if not b: break
  h.update(b)
  print('DATA '+str(offset)+' '+binascii.b2a_base64(b).decode().strip())
  offset+=len(b)
  time.sleep_ms(15)
print('HASH '+str(offset)+' '+binascii.hexlify(h.digest()).decode())
print('EXPORT_DONE')
""" % source
            data = bytearray()
            expected_hash = None
            expected_size = None
            for line in link.execute(script, timeout=60):
                if line.startswith(b'DATA '):
                    _, offset, encoded = line.split(b' ', 2)
                    if int(offset) != len(data):
                        raise RuntimeError('Missing or duplicated image chunk: ' + name)
                    data.extend(base64.b64decode(encoded, validate=True))
                elif line.startswith(b'HASH '):
                    _, size, expected_hash = line.split(b' ')
                    expected_size = int(size)
                else:
                    print(line.decode(errors='replace'), flush=True)
            digest = hashlib.sha256(data).hexdigest().encode()
            if len(data) != expected_size or digest != expected_hash:
                raise RuntimeError('File integrity check failed: ' + name)
            target = destination / name
            if target.exists() and target.read_bytes() != data:
                raise RuntimeError('Existing file differs; refusing overwrite: ' + str(target))
            target.write_bytes(data)
            print('DOWNLOADED %d/%d %s %d bytes' % (index+1,len(files),name,len(data)), flush=True)
        print('EXPORT_COMPLETE', destination, flush=True)
    finally:
        link.port.close()

if __name__ == '__main__':
    main()
