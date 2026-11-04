#!/usr/bin/env python3
"""K230-only prompted still collection. Default builds a script; no hardware.

During execution keep this command's stdout visible: 拍照N is emitted before
the two-second placement interval. Images stay on SD until all captures finish.
"""
import sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parents[2]))
import argparse
import base64
from datetime import datetime
from zoneinfo import ZoneInfo
import glob
import hashlib
import json
import subprocess
import time

from host.project_paths import ROOT
from host.transport.download_calibration import CameraLink, PORT
from tools.diagnostics.replay_k230_car_tracker import restore_listener


def build_script(remote, count=30, delay_ms=2000):
    return '''import sys,os,time,json
camera=None
try:
 with open('/sdcard/aic/pick/current.json') as f:release=json.load(f)['release']
 sys.path.insert(0,release)
 from camera import CameraSession
 with open(release+'/config.json') as f:vision=json.load(f)['vision']
 if (vision['camera']['width'],vision['camera']['height'])!=(1280,960):raise ValueError('Expected calibrated 1280x960')
 if vision['camera']['hmirror'] or vision['camera']['vflip']:raise ValueError('Expected calibrated unmirrored image')
 for folder in ['/sdcard/car_far',%r]:
  try:os.mkdir(folder)
  except OSError:os.stat(folder)
 camera=CameraSession(vision,preview=False).open()
 for number in range(1,%d+1):
  path=%r+'/frame_%%02d.jpg'%%number
  try:os.stat(path)
  except OSError:pass
  else:raise ValueError('Refusing to overwrite '+path)
  prompted=time.ticks_ms()
  print('CAR_PHOTO_READY '+json.dumps({'number':number,'delay_ms':%d,'prompt_timestamp_ms':prompted}))
  while time.ticks_diff(time.ticks_ms(),prompted)<%d:
   image=camera.snapshot()
   time.sleep_ms(20)
  before=time.ticks_ms()
  image=camera.snapshot()
  after=time.ticks_ms()
  image.save(path,quality=95)
  print('CAR_PHOTO_SAVED '+json.dumps({'number':number,'remote':path,'capture_call_started_ms':before,'capture_call_completed_ms':after,'placement_elapsed_ms':time.ticks_diff(before,prompted),'timestamp_basis':'software snapshot-call interval, not sensor exposure time'}))
except BaseException as exc:
 sys.print_exception(exc)
finally:
 if camera is not None:camera.close()
 print('EXPORT_DONE')
''' % (remote, count, remote, delay_ms, delay_ms)


def download(link, remote, progress=None, script_prefix=''):
    script = '''import binascii,time,uhashlib,os
print('SIZE '+str(os.stat(%r)[6]))
h=uhashlib.sha256();offset=0
with open(%r,'rb') as f:
 while True:
  chunk=f.read(2048)
  if not chunk:break
  h.update(chunk)
  print('DATA '+str(offset)+' '+binascii.b2a_base64(chunk).decode().strip())
  offset+=len(chunk);time.sleep_ms(15)
print('HASH '+str(offset)+' '+binascii.hexlify(h.digest()).decode())
print('EXPORT_DONE')
''' % (remote, remote)
    data = bytearray(); expected = None; size = None; total = None
    for line in link.execute(script_prefix + script, timeout=90):
        if line.startswith(b'SIZE '):
            total = int(line[5:])
            if progress is not None: progress(0, total)
        elif line.startswith(b'DATA '):
            _, offset, encoded = line.split(b' ', 2)
            if int(offset) != len(data): raise RuntimeError('Image chunk offset mismatch')
            data.extend(base64.b64decode(encoded, validate=True))
            if progress is not None: progress(len(data), total)
        elif line.startswith(b'HASH '):
            _, value, digest = line.split(); size = int(value); expected = digest.decode()
    digest = hashlib.sha256(data).hexdigest()
    if expected != digest or size != len(data) or total != size:
        raise RuntimeError('Image hash/size mismatch')
    return bytes(data), digest


def download_progress(number, count):
    last = [-float('inf')]
    def update(received, total):
        now = time.monotonic()
        if now - last[0] < 3 and received != total: return
        last[0] = now
        if total:
            print('下载 %d/%d：%d/%d KB（%.0f%%）' %
                  (number, count, received // 1024, total // 1024, received * 100 / total), flush=True)
    return update


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--execute', action='store_true')
    parser.add_argument('--count', type=int, default=30)
    parser.add_argument('--delay-ms', type=int, default=2000)
    parser.add_argument('--port', default=PORT)
    parser.add_argument('--output', type=Path)
    args = parser.parse_args(argv)
    if not 1 <= args.count <= 100: parser.error('--count must be 1..100')
    if not 1000 <= args.delay_ms <= 30000: parser.error('--delay-ms must be 1000..30000')
    stamp = datetime.now(ZoneInfo('Asia/Shanghai')).strftime('%Y%m%d_%H%M%S_%f')
    output = args.output or ROOT / 'data/car/far_capture' / stamp
    output.mkdir(parents=True, exist_ok=True)
    if (output / 'result.json').exists(): parser.error('Output already contains a session; choose a new directory')
    remote = '/sdcard/car_far/' + stamp
    source = build_script(remote, args.count, args.delay_ms)
    (output / 'board_script.py').write_text(source)
    result = {'status': 'built_no_hardware_access', 'requested_count': args.count,
              'placement_delay_ms': args.delay_ms, 'motion_performed': False,
              'remote_directory': remote, 'frames': [], 'listener_restored': False}
    link = None; lines = []
    try:
        if args.execute:
            port = args.port
            if port == PORT and not Path(port).exists():
                ports = glob.glob('/dev/serial/by-id/usb-Kendryte_CanMV_*-if00')
                if len(ports) != 1: raise RuntimeError('Expected one connected K230')
                port = ports[0]
            owner = subprocess.run(['fuser', port], capture_output=True, text=True)
            if owner.returncode != 1: raise RuntimeError('Port busy or ownership check failed')
            link = CameraLink(port)
            for raw in link.execute(source, timeout=args.count * (args.delay_ms / 1000 + 10) + 60):
                line = raw.decode(errors='replace'); lines.append(line)
                if line.startswith('CAR_PHOTO_READY '):
                    row = json.loads(line[len('CAR_PHOTO_READY '):])
                    print('拍照%d' % row['number'], flush=True)
                elif line.startswith('CAR_PHOTO_SAVED '):
                    row = json.loads(line[len('CAR_PHOTO_SAVED '):]); result['frames'].append(row)
                    print('已拍%d/%d' % (row['number'], args.count), flush=True)
                else: print(line, flush=True)
            if len(result['frames']) != args.count: raise RuntimeError('Capture incomplete; saved SD files retained')
            print('拍摄完成，开始下载 %d 张原图；下载结束后恢复语音。' % args.count, flush=True)
            download_started = time.monotonic()
            downloaded_bytes = 0
            for row in result['frames']:
                data, digest = download(link, row['remote'], download_progress(row['number'], args.count))
                target = output / ('frame_%02d.jpg' % row['number'])
                if target.exists(): raise RuntimeError('Refusing local image overwrite')
                temp = target.with_suffix('.partial'); temp.write_bytes(data); temp.replace(target)
                row.update(local=str(target), sha256=digest, bytes=len(data), quality_review='pending')
                downloaded_bytes += len(data)
                elapsed = time.monotonic() - download_started
                remaining = elapsed / row['number'] * (args.count - row['number'])
                print('已下载 %d/%d，平均 %.1f KB/s，预计剩余 %.0f 秒' %
                      (row['number'], args.count, downloaded_bytes / max(elapsed, .001) / 1024, remaining), flush=True)
            result['status'] = 'captured_and_downloaded'
    except BaseException as exc:
        result.update(status='failed', error=repr(exc))
        print(str(exc), file=sys.stderr)
    finally:
        if link is not None:
            try:
                print('下载/采集阶段已结束，正在恢复语音监听……', flush=True)
                restore = restore_listener(link)
                (output / 'listener_log.txt').write_text('\n'.join(restore) + '\n')
                result['listener_restored'] = True
            except BaseException as exc:
                result.update(status='listener_restore_failed', listener_restore_error=repr(exc))
            link.port.close()
        (output / 'board_log.txt').write_text('\n'.join(lines) + '\n')
        (output / 'result.json').write_text(json.dumps(result, ensure_ascii=False, indent=2) + '\n')
    print('RECORD', output, flush=True)
    return 0 if result['status'] in ('built_no_hardware_access', 'captured_and_downloaded') else 1


if __name__ == '__main__': raise SystemExit(main())
