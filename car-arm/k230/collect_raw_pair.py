#!/usr/bin/env python3
"""Save a K230 image on SD and bracket it with raw, read-only servo feedback.

No vision, FK, PID changes, torque changes, or robot motion. Images are
downloaded together later using --download-pending.
"""
import argparse
import base64
from datetime import datetime
import hashlib
import json
import os
from pathlib import Path
import struct
import sys
import time
from zoneinfo import ZoneInfo
from download_calibration import CameraLink, PORT as CAMERA_PORT

HERE = Path(__file__).resolve().parent
PAIRS = HERE / 'grasp_pairs'
PROJECT = Path(__file__).resolve().parents[1]
SERVO_PORT = '/dev/serial/by-id/usb-1a86_USB_Single_Serial_5B79033467-if00'

def stamp():
    return datetime.now(ZoneInfo('Asia/Shanghai')).isoformat()

def save(path, value):
    tmp = path.with_suffix('.tmp')
    tmp.write_text(json.dumps(value, ensure_ascii=False, indent=2) + '\n')
    os.replace(tmp, path)

def sync_collection_storage(pairs=PAIRS):
    """Refresh local/SD storage metadata without opening either serial port."""
    path=pairs/'collection.json'
    if not path.exists(): return {'downloaded':0,'pending':0}
    collection=json.loads(path.read_text())
    downloaded=pending=0
    for group in collection['groups']:
        folder=pairs/group['folder']
        record=json.loads((folder/'record.json').read_text())
        camera=record.get('camera_capture',{})
        image=folder/'frame.jpg'
        if image.exists():
            if camera.get('sha256') and hashlib.sha256(image.read_bytes()).hexdigest()!=camera['sha256']:
                raise RuntimeError('Local image integrity check failed: '+str(image))
            group['image_storage']='Computer frame.jpg; camera SD original retained'
            downloaded+=1
        elif camera.get('downloaded'):
            raise RuntimeError('Exported image missing: '+str(image))
        else:
            group['image_storage']='K230 SD; pending batch download'
            pending+=1
    collection['downloaded_group_count']=downloaded
    collection['pending_download_count']=pending
    save(path,collection)
    return {'downloaded':downloaded,'pending':pending}

def collect():
    sys.path.insert(0, str(PROJECT / 'ros2_ws/src/arm_pkg'))
    from arm_pkg.servo_bus import ServoBus
    folder = PAIRS / datetime.now(ZoneInfo('Asia/Shanghai')).strftime('pair_%Y%m%d_%H%M%S_%f')
    folder.mkdir(parents=True, exist_ok=False)
    remote = '/sdcard/grasp_pairs/' + folder.name + '.jpg'
    record = {'status': 'pending', 'created': stamp(), 'raw_only': True,
              'motion_commands_sent': False, 'pid_writes_sent': False,
              'servo_port': SERVO_PORT, 'samples_before': [], 'samples_after': [],
              'camera_capture': {'remote': remote, 'width': 1280, 'height': 960,
                                 'hmirror': False, 'vflip': False, 'downloaded': False}}
    path = folder / 'record.json'
    save(path, record)
    def sample(bus):
        result = {}
        for sid in range(1, 7):
            started = stamp()
            position, speed = bus.read_position_speed(sid)
            torque = bus.read_torque(sid)
            result[str(sid)] = {'position': position, 'speed': speed, 'torque': torque,
                                'read_started': started, 'read_finished': stamp()}
        return result
    link = None
    try:
        link = CameraLink(CAMERA_PORT)
        link.command(0x80, 12)
        record['camera_capture']['firmware'] = list(struct.unpack('<III', link.read(12)))
        script = '''import os,time,binascii
import uhashlib as hashlib
from media.sensor import Sensor
from media.display import Display
from media.media import MediaManager
sensor=None
path=%r
try:
 try: os.mkdir('/sdcard/grasp_pairs')
 except OSError: pass
 sensor=Sensor()
 sensor.reset()
 sensor.set_framesize(width=1280,height=960)
 sensor.set_pixformat(Sensor.RGB565)
 sensor.set_hmirror(False)
 sensor.set_vflip(False)
 Display.init(Display.VIRT,width=1280,height=960,fps=30,to_ide=True)
 MediaManager.init()
 sensor.run()
 for i in range(30):
  img=sensor.snapshot()
  time.sleep_ms(30)
 print('SNAPSHOT_TICKS_MS '+str(time.ticks_ms()))
 img.save(path,quality=95)
finally:
 if sensor is not None: sensor.stop()
 Display.deinit()
 os.exitpoint(os.EXITPOINT_ENABLE_SLEEP)
 time.sleep_ms(100)
 MediaManager.deinit()
h=hashlib.sha256()
size=0
with open(path,'rb') as f:
 while True:
  b=f.read(4096)
  if not b: break
  h.update(b)
  size+=len(b)
print('SAVED '+str(size)+' '+binascii.hexlify(h.digest()).decode())
print('EXPORT_DONE')
''' % remote
        with ServoBus(SERVO_PORT) as bus:
            for _ in range(3):
                record['samples_before'].append(sample(bus))
                time.sleep(.05)
            record['camera_operation_started'] = stamp()
            save(path, record)
            lines = []
            for line in link.execute(script, timeout=35):
                text = line.decode(errors='replace')
                lines.append(text)
                if text.startswith('SAVED '):
                    _, size, digest = text.split()
                    record['camera_capture'].update(size=int(size), sha256=digest)
                elif text.startswith('SNAPSHOT_TICKS_MS '):
                    record['camera_capture']['device_snapshot_ticks_ms'] = int(text.split()[1])
            record['camera_operation_finished'] = stamp()
            (folder / 'camera_log.txt').write_text('\n'.join(lines) + '\n')
            for _ in range(3):
                record['samples_after'].append(sample(bus))
                time.sleep(.05)
        if not record['camera_capture'].get('size'):
            raise RuntimeError('Camera did not confirm saved image; see camera_log.txt')
        record['status'] = 'raw_pair_saved_on_camera_sd'
        save(path, record)
        collection_path = PAIRS / 'collection.json'
        collection = json.loads(collection_path.read_text()) if collection_path.exists() else {'groups': []}
        group = len(collection['groups']) + 1
        collection['groups'].append({'group': group, 'folder': folder.name, 'raw_only': True,
                                     'image_storage': 'K230 SD; pending batch download'})
        collection['status'] = 'collection_in_progress'
        save(collection_path, collection)
        print('GROUP', group, 'PAIR_SAVED', folder, flush=True)
        print('IMAGE_SAVED_ON_SD', remote, record['camera_capture']['size'], flush=True)
    except Exception as exc:
        record['status'] = 'incomplete'
        record['error'] = str(exc)
        save(path, record)
        raise
    finally:
        if link is not None: link.port.close()

def download_pending():
    paths=[]
    for path in sorted(PAIRS.glob('pair_*/record.json')):
        record=json.loads(path.read_text());camera=record.get('camera_capture',{})
        if record.get('raw_only') and not camera.get('downloaded') and camera.get('sha256'):
            paths.append(path)
    if not paths:
        print('NO_PENDING_IMAGES',sync_collection_storage(),flush=True)
        return
    link = CameraLink(CAMERA_PORT)
    try:
        for path in paths:
            record = json.loads(path.read_text())
            camera = record.get('camera_capture', {})
            if not record.get('raw_only') or camera.get('downloaded') or not camera.get('sha256'):
                continue
            data = bytearray()
            script = '''import time,binascii
offset=0
with open(%r,'rb') as f:
 while True:
  b=f.read(1536)
  if not b: break
  print('DATA '+str(offset)+' '+binascii.b2a_base64(b).decode().strip())
  offset+=len(b)
  time.sleep_ms(15)
print('EXPORT_DONE')
''' % camera['remote']
            for line in link.execute(script, timeout=90):
                if line.startswith(b'DATA '):
                    _, offset, chunk = line.split(b' ', 2)
                    if int(offset) != len(data): raise RuntimeError('Missing image data chunk')
                    data.extend(base64.b64decode(chunk, validate=True))
            if len(data) != camera['size'] or hashlib.sha256(data).hexdigest() != camera['sha256']:
                raise RuntimeError('Image integrity check failed: ' + str(path))
            target = path.parent / 'frame.jpg'
            if target.exists() and target.read_bytes() != data:
                raise RuntimeError('Refusing to overwrite different image: ' + str(target))
            target.write_bytes(data)
            camera.update(downloaded=True, downloaded_at=stamp())
            record['status'] = 'raw_pair_saved_locally'
            save(path, record)
            sync_collection_storage()
            print('DOWNLOADED', target, flush=True)
    finally:
        link.port.close()

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    mode=parser.add_mutually_exclusive_group()
    mode.add_argument('--download-pending', action='store_true')
    mode.add_argument('--refresh-index', action='store_true', help='offline storage-index refresh only')
    args = parser.parse_args()
    if args.refresh_index: print('STORAGE_INDEX',sync_collection_storage())
    elif args.download_pending: download_pending()
    else: collect()
