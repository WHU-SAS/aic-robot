#!/usr/bin/env python3
"""Build, hash-verify and activate the user-accepted standalone K230 voice pick."""
import sys
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[2]))
from host.project_paths import ROOT,bootstrap
bootstrap()
import argparse,ast,base64,hashlib,json
from datetime import datetime
from zoneinfo import ZoneInfo
from host.apps.pick_validation_pc import parameters
from tools.diagnostics.k230_arm_ready import config_payload as ready_payload
from device.k230.pick.voice_commands import registered_commands
from host.arm.blue_pick_geometry import PickGeometry
from host.transport.k230_pick_stage import payload
from host.transport.download_calibration import CameraLink,PORT
from host.vision.config import load_vision_config

REMOTE='/sdcard/aic/pick'
STARTUP='''# AIC standalone voice-triggered pick. Boot only listens; no boot motion.
import sys,json
with open('/sdcard/aic/pick/current.json') as f:
    release=json.load(f)['release']
sys.path.insert(0,release)
from app import run_forever
run_forever(release)
'''


def digest(data):return hashlib.sha256(data).hexdigest()
def encoded(value):return (json.dumps(value,ensure_ascii=False,sort_keys=True,indent=2)+'\n').encode()

def build_bundle():
    import numpy as np
    from scipy.spatial import ConvexHull
    pick,config=parameters();vision=load_vision_config(ROOT/pick['vision_config']);loc=vision['localization']
    intrpath=ROOT/loc['intrinsics'];intr=json.loads(intrpath.read_text());ext=json.loads((ROOT/loc['extrinsics']).read_text())
    if digest(intrpath.read_bytes())!=ext['intrinsics_sha256']:raise ValueError('Calibration pair mismatch')
    geometry=PickGeometry(ROOT,config.mapping);model={'joints':[],'points':{}}
    for i in range(1,7):
        item=dict(config.mapping.joint(i));joint=geometry.fk.joints[item['urdf_joint_name']]
        origin=np.eye(4);origin[:3,:3]=geometry.fk._rpy_to_matrix(joint['rpy']);origin[:3,3]=joint['xyz']
        item.update(origin_transform=origin.tolist(),axis=joint['axis'].tolist(),child=joint['child'])
        model['joints'].append(item)
    for link,vertices in geometry.vertices.items():
        model['points'][link]={'tip':geometry.tip[link].tolist(),'distal':geometry.distal[link].tolist(),
                             'hull':vertices[ConvexHull(vertices).vertices].tolist()}
    voice=json.loads((ROOT/'configs/voice/wake_word.json').read_text())
    if not voice.get('enrollment_file'):raise ValueError('No accepted wake word registration')
    commands=registered_commands(voice)
    control=payload(config,[2048]*6)
    cfg={'pick':pick,'vision':vision,'intrinsics':intr,'extrinsics':ext,'model':model,'control':control,'voice':voice,'ready_control':ready_payload()}
    files={p.name:p.read_bytes() for p in (ROOT/'device/k230/pick').glob('*.py')}
    files['camera.py']=(ROOT/'device/k230/vision/camera.py').read_bytes()
    files['wake_word.py']=(ROOT/'device/k230/audio/wake_word.py').read_bytes()
    read=(ROOT/'device/k230/diagnostics/uart1_servo_read.py').read_text().split('\nif __name__ == "__main__":')[0]
    ready=(ROOT/'device/k230/arm/ready_pose.py').read_text()
    text=(ROOT/'host/arm/blue_pick_settle.py').read_text();node=next(n for n in ast.parse(text).body if isinstance(n,ast.ClassDef) and n.name=='PositionSettled')
    settle='\n'.join(text.splitlines()[node.lineno-1:node.end_lineno])
    files['control.py']='\n'.join([read,ready,settle,(ROOT/'device/k230/arm/pick_stage.py').read_text()]).encode()
    files['config.json']=encoded(cfg)
    manifest={'schema':'aic.standalone_pick.v1','trigger':'思达思达','voice_commands':[{'word':c['word'],'action':c['action']} for c in commands],'startup_motion':False,
              'pc_runtime_required':False,'cube_offset_toward_moving_finger_mm':pick['cube_offset_toward_moving_finger_mm'],
              'files':{name:{'bytes':len(data),'sha256':digest(data)} for name,data in sorted(files.items())}}
    release=digest(encoded(manifest))[:16];manifest['release_id']=release
    files['manifest.json']=encoded(manifest)
    return cfg,files,manifest


def execute(link,source,timeout=60):
    wrapped='try:\n exec('+repr(source)+')\nexcept BaseException as exc:\n import sys\n sys.print_exception(exc)\nfinally:\n print("EXPORT_DONE")\n'
    lines=[v.decode(errors='replace') for v in link.execute(wrapped,timeout=timeout)]
    for line in lines:
        if line:print(line,flush=True)
    return lines


def mkdir(link,path):
    source="import os\np=''\nfor part in %r.strip('/').split('/'):\n p+='/'+part\n try:os.mkdir(p)\n except OSError:os.stat(p)\nprint('DIR_OK')\n"%path
    if 'DIR_OK' not in execute(link,source):raise RuntimeError('Cannot create release directory')

def write_file(link,path,data):
    probe="""import binascii,uhashlib
try:
 h=uhashlib.sha256()
 with open(%r,'rb') as f:
  while True:
   b=f.read(4096)
   if not b:break
   h.update(b)
 print('EXISTING '+binascii.hexlify(h.digest()).decode())
except OSError:print('NEW_FILE')
"""%path
    if 'EXISTING '+digest(data) in execute(link,probe):return
    # Files are written in bounded USBDBG scripts, then read from SD and hashed.
    for offset in range(0,len(data),12288):
        chunk=data[offset:offset+12288];mode='wb' if offset==0 else 'ab'
        source="import binascii\nwith open(%r,%r) as f:\n f.write(binascii.a2b_base64(%r))\nprint('CHUNK_OK')\n"%(path+'.partial',mode,base64.b64encode(chunk))
        if 'CHUNK_OK' not in execute(link,source):raise RuntimeError('File chunk write failed')
    source="""import os,binascii,uhashlib
h=uhashlib.sha256();size=0
with open(%r,'rb') as f:
 while True:
  b=f.read(4096)
  if not b:break
  h.update(b);size+=len(b)
actual=binascii.hexlify(h.digest()).decode()
if actual!=%r or size!=%d:raise RuntimeError('SD write hash mismatch')
source=%r;target=%r
try:os.rename(source,target)
except OSError as exc:
 if exc.args[0]!=17:raise
 # CanMV FAT refuses rename over an existing file. Keep the old version until
 # the verified replacement is installed, and restore it if that rename fails.
 backup=target+'.replace_previous'
 try:os.stat(backup)
 except OSError:pass
 else:raise RuntimeError('Unresolved replacement backup: '+backup)
 os.rename(target,backup)
 try:os.rename(source,target)
 except BaseException:
  os.rename(backup,target)
  raise
 os.remove(backup)
print('FILE_OK '+actual)
"""%(path+'.partial',digest(data),len(data),path+'.partial',path)
    if 'FILE_OK '+digest(data) not in execute(link,source):raise RuntimeError('SD verification failed: '+path)


def verify(link,remote,manifest,config):
    # Imports/pose/IK and saved registration checks only: no camera/UART init.
    source="""import os,sys,json,binascii,uhashlib
sys.path.insert(0,%r)
for name in ['app','pose','numeric','kinematics','vision','reflection_faces','face_selection','top_boundaries','camera','wake_word','control','voice_commands']:
 if name in sys.modules:del sys.modules[name]
with open(%r+'/manifest.json') as f:m=json.load(f)
for name,item in m['files'].items():
 h=uhashlib.sha256();size=0
 with open(%r+'/'+name,'rb') as f:
  while True:
   b=f.read(4096)
   if not b:break
   h.update(b);size+=len(b)
 if size!=item['bytes'] or binascii.hexlify(h.digest()).decode()!=item['sha256']:raise RuntimeError('Release corrupt: '+name)
from app import load
from pose import pose_candidates
from kinematics import Geometry
from vision import localize
cfg=load(%r)
from voice_commands import registered_commands
from wake_word import engine
keywords=[]
for command in registered_commands(cfg['voice']):
 with open(command['enrollment_file']) as f:keyword=json.load(f)
 if keyword['kw_id']!=command['keyword_id']:raise ValueError('Keyword ID mismatch')
 keywords.append(keyword)
eng=engine(cfg['voice']['detection_sensitivity'])
try:
 for keyword in keywords:eng.add_keyword(keyword)
finally:eng.deinit()
os.stat('/sdcard/examples/31-OPEN-KWS/model/kws.kmodel')
print('RELEASE_VERIFIED '+m['release_id'])
"""%(remote,remote,remote,remote)
    if 'RELEASE_VERIFIED '+manifest['release_id'] not in execute(link,source):raise RuntimeError('Board import/hash check failed')


def main(argv=None):
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--port',default=PORT)
    p.add_argument('--install',action='store_true');p.add_argument('--activate',action='store_true')
    a=p.parse_args(argv)
    if a.activate and not a.install:p.error('activate requires install')
    config,files,manifest=build_bundle();stamp=datetime.now(ZoneInfo('Asia/Shanghai')).strftime('%Y%m%d_%H%M%S')
    output=ROOT/'data/deployments'/('k230_pick_'+stamp);output.mkdir(parents=True)
    for name,data in files.items():(output/name).write_bytes(data)
    remote=REMOTE+'/releases/'+manifest['release_id'];record={'remote':remote,'manifest':manifest,'activated':False,'status':'built_no_hardware_access'}
    link=None
    try:
        if a.install:
            link=CameraLink(a.port);mkdir(link,remote)
            for name,data in files.items():write_file(link,remote+'/'+name,data)
            verify(link,remote,manifest,config);record['status']='installed_verified_not_active'
            if a.activate:
                # Keep previous startup and pointer locally BEFORE replacing them.
                for path in ['/sdcard/main.py',REMOTE+'/current.json']:
                    source="""import os,binascii
try:
 with open(%r,'rb') as f:b=f.read()
 print('BACKUP '+binascii.b2a_base64(b).decode().strip())
except OSError:print('BACKUP_MISSING')
"""%path
                    lines=execute(link,source);line=next((v for v in lines if v.startswith('BACKUP ')),None)
                    if line:(output/('previous_'+path.rsplit('/',1)[-1])).write_bytes(base64.b64decode(line[7:]))
                    elif 'BACKUP_MISSING' not in lines:raise RuntimeError('Cannot back up startup')
                write_file(link,REMOTE+'/current.json',encoded({'release':remote,'release_id':manifest['release_id']}))
                write_file(link,'/sdcard/main.py',STARTUP.encode())
                record.update(status='activated_next_boot_only',activated=True)
        print('DEPLOYMENT',record['status'],remote,flush=True)
        return 0
    except Exception as exc:
        record.update(status='failed',error=repr(exc));print(str(exc),file=sys.stderr);return 1
    finally:
        if link:link.port.close()
        (output/'deployment.json').write_bytes(encoded(record));print('RECORD',output)

if __name__=='__main__':raise SystemExit(main())
