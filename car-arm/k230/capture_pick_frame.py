#!/usr/bin/env python3
"""Capture a fresh K230 JPEG for one pick run, no servo bus or collection mutation."""
import argparse,base64,hashlib,json,struct,time
from pathlib import Path
from download_calibration import CameraLink,PORT

def capture(output,port=PORT):
 output=Path(output);output.mkdir(parents=True,exist_ok=True);remote='/sdcard/pick_tests/'+output.name+'.jpg';link=CameraLink(port);details={};log=[]
 try:
  link.command(0x80,12);details['firmware']=list(struct.unpack('<III',link.read(12)))
  script="""import os,time,binascii
import uhashlib as hashlib
from media.sensor import Sensor
from media.display import Display
from media.media import MediaManager
sensor=None
try:
 try: os.mkdir('/sdcard/pick_tests')
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
 img.save(%r,quality=95)
finally:
 if sensor is not None: sensor.stop()
 Display.deinit()
 os.exitpoint(os.EXITPOINT_ENABLE_SLEEP)
 time.sleep_ms(100)
 MediaManager.deinit()
h=hashlib.sha256()
offset=0
with open(%r,'rb') as f:
 while True:
  b=f.read(2048)
  if not b: break
  h.update(b)
  print('DATA '+str(offset)+' '+binascii.b2a_base64(b).decode().strip())
  offset+=len(b)
  time.sleep_ms(15)
print('HASH '+str(offset)+' '+binascii.hexlify(h.digest()).decode())
print('EXPORT_DONE')
"""%(remote,remote)
  data=bytearray();expected=None;size=None
  for line in link.execute(script,timeout=90):
   if line.startswith(b'DATA '):
    _,offset,b=line.split(b' ',2)
    if int(offset)!=len(data):raise RuntimeError('K230 missing image chunk')
    data.extend(base64.b64decode(b,validate=True))
   elif line.startswith(b'HASH '):_,n,h=line.split();size=int(n);expected=h.decode()
   else:
    message=line.decode(errors='replace');log.append(message)
    if 'Traceback' in message:raise RuntimeError('K230 capture failed: '+message)
  actual=hashlib.sha256(data).hexdigest()
  if size is None or len(data)!=size or actual!=expected:raise RuntimeError('K230 image hash/size verification failed')
  temp=output/'frame.transfer';temp.write_bytes(data);temp.replace(output/'frame.jpg')
  details.update(remote=remote,sha256=actual,size=size,width=1280,height=960,hmirror=False,vflip=False,robot_commands_sent=False)
  (output/'capture.json').write_text(json.dumps(details,indent=2)+'\n');(output/'camera_log.txt').write_text('\n'.join(log));return details
 finally:link.port.close()

if __name__=='__main__':
 p=argparse.ArgumentParser();p.add_argument('--output',type=Path,required=True);p.add_argument('--port',default=PORT);a=p.parse_args();print(json.dumps(capture(a.output,a.port)))
