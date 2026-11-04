#!/usr/bin/env python3
"""Prompted K230 near stills with GPIO26 light; no car/arm commands.

Default builds the capture script only. --execute pauses the existing program,
captures JPEG95 originals, downloads and hash-verifies them, then leaves voice
paused for the inspection session. It never installs a release or edits boot.
"""
import sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

import argparse
import glob
import json
import os
import struct
import subprocess
import termios
from datetime import datetime
from zoneinfo import ZoneInfo

from host.project_paths import ROOT
from host.transport.download_calibration import CameraLink, PORT
from tools.diagnostics.capture_car_far import download, download_progress
from tools.diagnostics.test_k230_motor_uart import stop_previous_script

LIGHT_OFF_DURING_TRANSFER = '''from machine import FPIOA,Pin
_lit_download_mux=FPIOA()
_lit_download_mux.set_function(26,FPIOA.GPIO26)
_lit_download_pin=Pin(26,Pin.OUT,value=0)
_lit_download_mux.set_function(26,FPIOA.GPIO26,pu=0,pd=1,ie=1,oe=1)
_lit_download_pin.value(0)
'''


def build_script(remote, count, delay_ms):
    return '''import os,sys,time,json,gc,binascii,uhashlib
from machine import FPIOA,Pin
camera=None;lamp=None
try:
 with open('/sdcard/main.py') as f:boot=f.read()
 import re
 match=re.search('/sdcard/aic/robot/releases/[0-9a-f]+',boot)
 if match is None:raise ValueError('Cannot identify existing robot camera configuration')
 release=match.group(0);sys.path.insert(0,release)
 with open(release+'/config.json') as f:cfg=json.load(f)
 c=cfg['car'];v=c['vision'];p=c['projection']
 if (v['camera']['width'],v['camera']['height'])!=(1280,960):raise ValueError('Expected1280x960')
 if v['camera']['hmirror'] or v['camera']['vflip']:raise ValueError('Expected unmirrored camera')
 from camera import CameraSession
 from fill_light import FillLight
 from ground_projection import GroundProjection
 from blue_tracker import BlueTracker
 from vision_runtime import track_coarse
 os.mkdir(%r)
 with open(%r+'/config.json','w') as f:json.dump(cfg,f)
 projection=GroundProjection(p['K'],p['D'],p['T'],p['plane'],p.get('forward_axis_Base'),p.get('motion_origin_Base_mm'))
 lamp=FillLight();lamp.on()
 camera=CameraSession(v).open();rows=[]
 for number in range(1,%d+1):
  gc.collect();prompted=time.ticks_ms()
  print('LIT_READY '+json.dumps({'number':number,'delay_ms':%d}))
  while time.ticks_diff(time.ticks_ms(),prompted)<%d:
   camera.snapshot();time.sleep_ms(20)
  img,frame=camera.capture()
  tracker=BlueTracker(dict(c['tracking'],confirmation_frames=1),projection)
  coarse=track_coarse(img,frame,tracker.detect_frame)
  cue=coarse.get('visual_evidence') or {}
  bbox=cue.get('bbox_px') if cue.get('associated') is True and cue.get('ambiguous') is False else None
  jpeg=img.compressed(quality=95);raw=bytes(jpeg.bytearray());name='frame_%%02d'%%number
  row={'number':number,'image_file':name+'.jpg','frame':frame,'paired_coarse':coarse,
   'bbox_px':bbox,'light_on':True,'image_bytes':len(raw),
   'image_sha256':binascii.hexlify(uhashlib.sha256(raw).digest()).decode(),
   'sample_role':'new_fixed_candidate_check; not98percent_acceptance',
   'coarse_state':'new tracker per still; no temporal identity claim',
   'capture_release':release,'motion_performed':False}
  with open(%r+'/'+name+'.jpg','wb') as f:f.write(raw)
  with open(%r+'/'+name+'.json','w') as f:json.dump(row,f)
  rows.append(row);print('LIT_SAVED '+json.dumps({'number':number,'bytes':len(raw),'sha256':row['image_sha256']}))
  img=jpeg=raw=tracker=None
 with open(%r+'/index.json','w') as f:json.dump({'frames':rows,'release':release,'actuator_io':False},f)
 print('LIT_COMPLETE '+str(len(rows)))
except BaseException as exc:
 sys.print_exception(exc)
finally:
 if lamp is not None:lamp.off()
 try:
  if camera is not None:camera.close()
 finally:
  mux=FPIOA();mux.set_function(26,FPIOA.GPIO26);pin=Pin(26,Pin.OUT,value=0)
  mux.set_function(26,FPIOA.GPIO26,pu=0,pd=1,ie=1,oe=1);pin.value(0)
  print('LIT_LIGHT_OFF '+str(pin.value()))
 print('EXPORT_DONE')
''' % (remote, remote, count, delay_ms, delay_ms, remote, remote, remote)


def open_link(port):
    if port == PORT and not Path(port).exists():
        devices = glob.glob('/dev/serial/by-id/usb-Kendryte_CanMV_*-if00')
        if len(devices) != 1:
            raise RuntimeError('请连接一块K230的OTG数据口')
        port = devices[0]
    owner = subprocess.run(['fuser', port], capture_output=True)
    if owner.returncode != 1:
        raise RuntimeError('K230串口被占用，请先关闭IDE串口连接')
    # A CDC close must not reset the board and discard its current session.
    fd = os.open(port, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
    try:
        flags = termios.tcgetattr(fd)
        flags[2] &= ~termios.HUPCL
        termios.tcsetattr(fd, termios.TCSANOW, flags)
    finally:
        os.close(fd)
    return CameraLink(port)


def save_download(link, remote, target, progress=None):
    data, digest = download(link, remote, progress,
                            script_prefix=LIGHT_OFF_DURING_TRANSFER)
    if target.exists():
        raise RuntimeError('拒绝覆盖本地文件: ' + str(target))
    partial = target.with_suffix(target.suffix + '.partial')
    partial.write_bytes(data)
    partial.replace(target)
    return {'file': target.name, 'bytes': len(data), 'sha256': digest}


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--execute', action='store_true')
    parser.add_argument('--count', type=int, default=6)
    parser.add_argument('--delay-ms', type=int, default=2000)
    parser.add_argument('--port', default=PORT)
    parser.add_argument('--output', type=Path)
    args = parser.parse_args(argv)
    if not 1 <= args.count <= 100 or not 1000 <= args.delay_ms <= 30000:
        parser.error('count需为1～100，delay-ms需为1000～30000')
    stamp = datetime.now(ZoneInfo('Asia/Shanghai')).strftime('%Y%m%d_%H%M%S_%f')
    folder = args.output or ROOT / 'data/vision/lit_validation' / stamp
    folder.mkdir(parents=True, exist_ok=False)
    remote = '/sdcard/aic/vision/lit_validation_' + stamp
    source = build_script(remote, args.count, args.delay_ms)
    compile(source, 'K230_lit_capture.py', 'exec')
    (folder / 'board_script.py').write_text(source)
    record = {'status': 'built_no_hardware_access', 'remote_directory': remote,
              'count': args.count, 'placement_delay_ms': args.delay_ms,
              'motion_performed': False, 'listener_restored': False,
              'listener_paused': False,
              'algorithm_tuning_during_batch': False, 'files': [], 'frames': []}
    link = None
    lines = []
    try:
        if args.execute:
            link = open_link(args.port)
            link.command(0x80, 12)
            record['firmware'] = list(struct.unpack('<III', link.read(12)))
            stop_previous_script(link)
            record['listener_paused'] = True
            print('语音已暂停，只开相机和补光；准备按提示摆%d个不同角度／位置。' % args.count, flush=True)
            for raw in link.execute(source, timeout=args.count * (args.delay_ms / 1000 + 10) + 45):
                line = raw.decode(errors='replace');lines.append(line)
                if line.startswith('LIT_READY '):
                    n = json.loads(line[10:])['number']
                    print('拍照%d：现在摆方块，%.1f秒后拍摄，及时把手移开。' % (n, args.delay_ms / 1000), flush=True)
                elif line.startswith('LIT_SAVED '):
                    row = json.loads(line[10:]);record['frames'].append(row)
                    print('已拍%d/%d' % (row['number'], args.count), flush=True)
                elif line.startswith('LIT_LIGHT_OFF '):
                    record['light_off_confirmed'] = line == 'LIT_LIGHT_OFF 0'
                else:
                    print(line, flush=True)
            if len(record['frames']) != args.count or not record.get('light_off_confirmed'):
                raise RuntimeError('采集未完整结束；SD原文件保留，详见board_log.txt')
            print('拍摄结束，补光已关闭。开始下载原图，进度如下：', flush=True)
            for name in ('config.json', 'index.json'):
                record['files'].append(save_download(link, remote + '/' + name, folder / name))
            for row in record['frames']:
                n = row['number'];name = 'frame_%02d' % n
                for extension in ('.json', '.jpg'):
                    info = save_download(link, remote + '/' + name + extension, folder / (name + extension),
                        download_progress(n, args.count) if extension == '.jpg' else None)
                    record['files'].append(info)
                    if extension == '.jpg' and info['sha256'] != row['sha256']:
                        raise RuntimeError('采集和下载原图SHA不一致')
                print('已下载%d/%d' % (n, args.count), flush=True)
            record['status'] = 'captured_and_downloaded'
    except BaseException as exc:
        record.update(status='failed', error=repr(exc))
        print('采集未完成: ' + str(exc), file=sys.stderr, flush=True)
        if link is not None:
            try:
                stop_previous_script(link)
                for raw in link.execute("from machine import FPIOA,Pin\nm=FPIOA();m.set_function(26,FPIOA.GPIO26)\np=Pin(26,Pin.OUT,value=0)\nm.set_function(26,FPIOA.GPIO26,pu=0,pd=1,ie=1,oe=1);p.value(0)\nprint('LIT_LIGHT_OFF '+str(p.value()))\nprint('EXPORT_DONE')\n", timeout=10):
                    if raw == b'LIT_LIGHT_OFF 0':record['light_off_confirmed'] = True
            except BaseException as cleanup:
                record['cleanup_error'] = repr(cleanup)
    finally:
        if link is not None:link.port.close()
        (folder / 'board_log.txt').write_text('\n'.join(lines) + '\n')
        (folder / 'result.json').write_text(json.dumps(record, ensure_ascii=False, indent=2) + '\n')
    print('RECORD ' + str(folder.resolve()), flush=True)
    if record['listener_paused']:
        print('语音暂时保持暂停；请保持K230连接，回复“拍好了”，随后分析并恢复监听。', flush=True)
    return 0 if record['status'] in ('built_no_hardware_access', 'captured_and_downloaded') else 1


if __name__ == '__main__':
    raise SystemExit(main())
