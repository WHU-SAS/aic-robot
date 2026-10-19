#!/usr/bin/env python3
"""Resume verified K230 session export with larger USBDBG stdout chunks.

Reads SD originals only. Hash-validates every file. Never opens robot serial.
"""

import sys as _sys
from pathlib import Path as _Path
_sys.path.insert(0, str(_Path(__file__).resolve().parents[2]))
from host.project_paths import ROOT as REPO_ROOT, bootstrap as _bootstrap
_bootstrap(include_tools=True)
import argparse
import base64
import hashlib
import json
from pathlib import Path
import re
import struct
import time
from download_calibration import CameraLink,PORT

HERE=(REPO_ROOT/'data/k230')

def write_manifest(path,value):
 temporary=path.with_suffix('.tmp');temporary.write_text(json.dumps(value,ensure_ascii=False,indent=2)+'\n');temporary.replace(path)

def main():
 p=argparse.ArgumentParser(description=__doc__);p.add_argument('--session',required=True);p.add_argument('--output',type=Path,default=HERE/'photos');p.add_argument('--chunk-bytes',type=int,default=2048);p.add_argument('--pause-ms',type=int,default=15);p.add_argument('--max-new-files',type=int);args=p.parse_args()
 if not re.fullmatch(r'session_[0-9]{4}',args.session):p.error('Invalid session')
 if not 1536<=args.chunk_bytes<=2850 or not 5<=args.pause_ms<=30:p.error('Invalid transfer settings')
 if args.max_new_files is not None and args.max_new_files<1:p.error('max-new-files must be positive')
 folder=args.output/args.session;folder.mkdir(parents=True,exist_ok=True);remote='/sdcard/calibration/'+args.session
 manifest={'session':args.session,'remote':remote,'hardware_robot_access':False,'files':[]};manifest_path=folder/'export_manifest.json'
 if manifest_path.exists():manifest=json.loads(manifest_path.read_text())
 records={r['file']:r for r in manifest['files']};link=CameraLink(PORT);new=0
 try:
  link.command(0x80,12);print('FIRMWARE',struct.unpack('<III',link.read(12)),flush=True)
  script="import os,json\nprint('FILES '+json.dumps([(n,os.stat(%r+'/'+n)[6]) for n in sorted(os.listdir(%r)) if (n.endswith('.jpg') or n.endswith('.json') or n.endswith('.jsonl')) and '.partial.' not in n]))\nprint('EXPORT_DONE')\n" % (remote,remote)
  listing=None
  for line in link.execute(script,timeout=15):
   if line.startswith(b'FILES '):listing=json.loads(line[6:])
   elif line:print(line.decode(errors='replace'),flush=True)
  if listing is None:raise RuntimeError('No session listing')
  for index,(name,size) in enumerate(listing):
   if Path(name).name!=name:raise RuntimeError('Unsafe filename')
   target=folder/name;source=remote+'/'+name
   previous=records.get(name)
   if target.exists():
    digest=hashlib.sha256(target.read_bytes()).hexdigest()
    if previous and previous['bytes']==size and previous['sha256']==digest:
     print('VERIFIED_LOCAL',name,flush=True);continue
    # Files completed by the earlier exporter have no persisted manifest.
    remote_hash=None
    check="import uhashlib as h,binascii\ns=h.sha256()\nwith open(%r,'rb') as f:\n while True:\n  b=f.read(65536)\n  if not b:break\n  s.update(b)\nprint('HASH '+binascii.hexlify(s.digest()).decode())\nprint('EXPORT_DONE')\n" % source
    for line in link.execute(check,timeout=15):
     if line.startswith(b'HASH '):remote_hash=line[5:].decode()
    if remote_hash!=digest or target.stat().st_size!=size:raise RuntimeError('Existing file differs from SD: '+name)
    records[name]={'file':name,'bytes':size,'sha256':digest,'remote_hash_verified':True}
    manifest['files']=list(records.values());write_manifest(manifest_path,manifest);print('VERIFIED_SD',name,flush=True);continue
   data=bytearray();remote_hash=None;expected_size=None;started=time.monotonic()
   code="""import binascii,time
import uhashlib as hashlib
h=hashlib.sha256()
offset=0
with open(%r,'rb') as f:
 while True:
  b=f.read(%d)
  if not b:break
  h.update(b)
  print('DATA '+str(offset)+' '+binascii.b2a_base64(b).decode().strip())
  offset+=len(b)
  time.sleep_ms(%d)
print('HASH '+str(offset)+' '+binascii.hexlify(h.digest()).decode())
print('EXPORT_DONE')
""" % (source,args.chunk_bytes,args.pause_ms)
   for line in link.execute(code,timeout=60):
    if line.startswith(b'DATA '):
     _,offset,encoded=line.split(b' ',2)
     if int(offset)!=len(data):raise RuntimeError('Missing data chunk: '+name)
     data.extend(base64.b64decode(encoded,validate=True))
    elif line.startswith(b'HASH '):
     _,n,h=line.split();expected_size=int(n);remote_hash=h.decode()
    elif line:
     print('CAMERA_MESSAGE',line.decode(errors='replace'),flush=True)
     if line.startswith(b'Traceback'):raise RuntimeError('Camera export script failed; see message')
   digest=hashlib.sha256(data).hexdigest()
   if len(data)!=size or expected_size!=size or digest!=remote_hash:raise RuntimeError('Integrity check failed: '+name)
   temporary=target.with_suffix('.transfer');temporary.write_bytes(data);temporary.replace(target)
   records[name]={'file':name,'bytes':size,'sha256':digest,'remote_hash_verified':True,'transfer_seconds':time.monotonic()-started}
   manifest['files']=list(records.values());manifest['chunk_bytes']=args.chunk_bytes;manifest['pause_ms']=args.pause_ms;write_manifest(manifest_path,manifest)
   print('DOWNLOADED',index+1,'/',len(listing),name,size,'seconds',round(time.monotonic()-started,2),flush=True);new+=1
   if args.max_new_files is not None and new>=args.max_new_files:break
  else:
   manifest['status']='complete';manifest['file_count']=len(listing);write_manifest(manifest_path,manifest);print('EXPORT_COMPLETE',folder,flush=True)
 finally:link.port.close()

if __name__=='__main__':main()
