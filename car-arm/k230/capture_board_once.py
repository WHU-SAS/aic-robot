#!/usr/bin/env python3
"""Capture one current K230 frame; no servo access or robot motion."""
import base64
from datetime import datetime
import hashlib
import json
from pathlib import Path
import struct
from download_calibration import CameraLink, PORT

out = Path(__file__).resolve().parent/'extrinsics'/datetime.now().strftime('board_%Y%m%d_%H%M%S')
out.mkdir(parents=True,exist_ok=False)
link=CameraLink(PORT)
try:
    link.command(0x80,12)
    firmware=struct.unpack('<III',link.read(12))
    remote_script = '''import os,time,binascii
import uhashlib as hashlib
from media.sensor import Sensor
from media.display import Display
from media.media import MediaManager
sensor=None
try:
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
 path='/sdcard/calibration/%s.jpg'
 img.save(path,quality=95)
 print('REMOTE '+path)
finally:
 if sensor is not None: sensor.stop()
 Display.deinit()
 os.exitpoint(os.EXITPOINT_ENABLE_SLEEP)
 time.sleep_ms(100)
 MediaManager.deinit()
h=hashlib.sha256()
offset=0
with open(path,'rb') as f:
 while True:
  b=f.read(1536)
  if not b: break
  h.update(b)
  print('DATA '+str(offset)+' '+binascii.b2a_base64(b).decode().strip())
  offset+=len(b)
  time.sleep_ms(15)
print('HASH '+str(offset)+' '+binascii.hexlify(h.digest()).decode())
print('EXPORT_DONE')
''' % out.name
    data=bytearray()
    digest=None
    size=None
    remote=None
    for line in link.execute(remote_script,timeout=60):
        if line.startswith(b'DATA '):
            _,offset,encoded=line.split(b' ',2)
            if int(offset)!=len(data): raise RuntimeError('Missing camera data chunk')
            data.extend(base64.b64decode(encoded,validate=True))
        elif line.startswith(b'HASH '):
            _,n,digest=line.split(b' ')
            size=int(n)
        elif line.startswith(b'REMOTE '):
            remote=line[7:].decode()
        else:
            print(line.decode(errors='replace'))
    if size!=len(data) or hashlib.sha256(data).hexdigest().encode()!=digest:
        raise RuntimeError('Snapshot integrity check failed')
    (out/'board.jpg').write_bytes(data)
    (out/'capture.json').write_text(json.dumps({'firmware':firmware,'remote':remote,
        'width':1280,'height':960,'sha256':digest.decode(),
        'robot_commands_sent':False},indent=2))
    print('CAPTURE_SAVED',out,flush=True)
finally:
    link.port.close()
