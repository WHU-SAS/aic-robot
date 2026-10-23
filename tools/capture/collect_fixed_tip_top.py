#!/usr/bin/env python3
"""Collect formal fixed-tip/top-center raw pairs through K230; no servo writes or analysis."""
import sys
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[2]))
import argparse
import ast
import base64
from datetime import datetime
import fcntl
import hashlib
import json
import struct
from zoneinfo import ZoneInfo
from host.project_paths import ROOT
from host.transport.download_calibration import CameraLink, PORT
from host.vision.config import load_vision_config


def stamp():
    return datetime.now(ZoneInfo('Asia/Shanghai')).isoformat()


def save(path, value):
    temporary=path.with_suffix('.tmp')
    temporary.write_text(json.dumps(value,ensure_ascii=False,indent=2)+'\n')
    temporary.replace(path)


def source_without_local_imports(path):
    source=path.read_text()
    lines=source.splitlines(keepends=True)
    removed=set()
    for node in ast.parse(source).body:
        local_import=isinstance(node,ast.ImportFrom) and node.module in ('camera','uart1_servo_read')
        main_guard=isinstance(node,ast.If) and isinstance(node.test,ast.Compare) and isinstance(node.test.left,ast.Name) and node.test.left.id=='__name__'
        if local_import or main_guard:
            removed.update(range(node.lineno-1,node.end_lineno))
    return ''.join(line for n,line in enumerate(lines) if n not in removed)


def build_script(config,remote,protocol='fixed-tip-top'):
    sample_type=protocol_type(protocol)
    paths=('device/k230/diagnostics/uart1_servo_read.py',
           'device/k230/vision/camera.py',
           'device/k230/diagnostics/capture_fixed_tip_top.py')
    return '\n'.join(source_without_local_imports(ROOT/p) for p in paths)+'\ncapture_fixed_tip_top(%r,%r,%r)\n' % (config,remote,sample_type)


def summarize(record):
    samples=record['samples_before']+record['samples_after']
    if len(record['samples_before'])!=3 or len(record['samples_after'])!=3:
        raise RuntimeError('Incomplete six-axis feedback around photograph')
    summary={}
    for sid in range(1,7):
        rows=[sample[str(sid)] for sample in samples]
        positions=sorted(row['position'] for row in rows)
        summary[str(sid)]={'median':(positions[2]+positions[3])/2,
            'min':positions[0],'max':positions[-1], 'span_counts':positions[-1]-positions[0],
            'speeds_raw':[row['speed'] for row in rows],
            'torque_states':[row['torque'] for row in rows]}
    return summary


def download_image(link, metadata):
    source="""import binascii,time
with open(%r,'rb') as source:
 offset=0
 while True:
  chunk=source.read(2048)
  if not chunk:break
  print('IMAGE_DATA '+str(offset)+' '+binascii.b2a_base64(chunk).decode().strip())
  offset+=len(chunk)
  time.sleep_ms(15)
print('EXPORT_DONE')
""" % metadata['remote']
    data=bytearray()
    for line in link.execute(source,timeout=60):
        if line.startswith(b'IMAGE_DATA '):
            _,offset,encoded=line.split(b' ',2)
            if int(offset)!=len(data):
                raise RuntimeError('Missing or duplicate image data')
            data.extend(base64.b64decode(encoded,validate=True))
    if len(data)!=metadata['size'] or hashlib.sha256(data).hexdigest()!=metadata['sha256']:
        raise RuntimeError('Image size or SHA256 mismatch; SD original retained')
    return data


def protocol_type(protocol):
    types={'fixed-tip-top':'fixed_fingertip_cube_top_contact',
           'tool-pivot':'fixed_point_pivot_contact'}
    if protocol not in types:raise ValueError('Unknown capture protocol')
    return types[protocol]


def freeze_session(session,protocol='fixed-tip-top'):
    sample_type=protocol_type(protocol)
    manifest=session/'session.json'
    if manifest.exists() and json.loads(manifest.read_text()).get('sample_type')!=sample_type:
        raise RuntimeError('Cannot mix contact protocols in one session')
    session.mkdir(parents=True,exist_ok=True)
    snapshot=session/'snapshot'
    snapshot.mkdir(exist_ok=True)
    paths=['configs/vision/blue_cube.json','configs/vision/runtime/camera_intrinsics.json',
           'configs/arm/joint_mapping.json','configs/arm/user_selected_pid_demo_baseline.json',
           'configs/arm/motion_defaults.json','models/genkiarm/genkiarm.urdf',
           'host/arm/gripper_geometry.py','host/arm/urdf_fk.py','host/arm/joint_mapping.py',
           'device/k230/diagnostics/uart1_servo_read.py',
           'device/k230/vision/camera.py','device/k230/diagnostics/capture_fixed_tip_top.py',
           'tools/capture/collect_fixed_tip_top.py']
    paths += [str(p.relative_to(ROOT)) for p in sorted((ROOT/'models/genkiarm/meshes').glob('*.stl'))]
    files=[]
    for name in paths:
        content=(ROOT/name).read_bytes()
        target=snapshot/name
        target.parent.mkdir(parents=True,exist_ok=True)
        if target.exists() and target.read_bytes()!=content:
            raise RuntimeError('Session source changed; use a new session: '+name)
        if not target.exists():
            target.write_bytes(content)
        files.append({'path':name,'sha256':hashlib.sha256(content).hexdigest()})
    manifest=session/'session.json'
    if not manifest.exists() and protocol=='tool-pivot':
        save(manifest,{'schema':'aic.tool_pivot_session.v1','created':stamp(),
            'sample_type':sample_type,'contacting_jaw':'fixed',
            'contact_requirement':'Same marked material point on fixed jaw touches same stationary cube top corner in every pose',
            'training_groups':[1,2,3],'validation_groups':[4],
            'planned_groups':4,'files':files,'motion_commands_sent':False})
    elif not manifest.exists():
        save(manifest,{'schema':'aic.fixed_tip_top_session.v1','created':stamp(),
            'sample_type':'fixed_fingertip_cube_top_contact','contacting_jaw':'fixed',
            'cube_edge_mm':20, 'nominal_gripper_opening_deg':30,
            'placement':'manually positioned, naturally self-supported, cube flat on table',
            'contact_region':'fixed distal rounded edge; small pose-dependent contact variation accepted by user',
            'contact_point_is_measured':False,
            'cube_center_definition':'contact minus 10mm along upward table normal; not jaw axis',
            'table_normal_Base':'to be established in analysis; do not assume measured',
            'files':files,'motion_commands_sent':False})
    return load_vision_config(snapshot/'configs/vision/blue_cube.json')


def collect(session,port=PORT,protocol='fixed-tip-top'):
    sample_type=protocol_type(protocol)
    session.mkdir(parents=True,exist_ok=True)
    with (session/'collection.lock').open('a') as lock:
        fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
        config=freeze_session(session,protocol)
        collection_path=session/'collection.json'
        collection=json.loads(collection_path.read_text()) if collection_path.exists() else {'groups':[]}
        completed=[e for e in collection['groups'] if e['status']=='raw_pair_saved_locally']
        group=len(completed)+1 if protocol=='tool-pivot' else len(collection['groups'])+1
        if protocol=='tool-pivot' and group>4:raise RuntimeError('Four pivot groups already collected; use a new session for a new audit')
        name='group_%03d_%s' % (group,datetime.now(ZoneInfo('Asia/Shanghai')).strftime('%Y%m%d_%H%M%S_%f'))
        folder=session/name
        folder.mkdir(exist_ok=False)
        remote=('/sdcard/tool_reference_pivot/' if protocol=='tool-pivot' else '/sdcard/formal_fixed_tip_top/')+name+'.jpg'
        entry={'group':group,'folder':name,'status':'pending'}
        collection['groups'].append(entry)
        save(collection_path,collection)
        record={'schema':'aic.tool_pivot_pair.v1' if protocol=='tool-pivot' else 'aic.fixed_tip_top_pair.v1','group':group,'created':stamp(),
                'status':'pending','raw_only':True,'sample_type':sample_type,
                'contacting_jaw':'fixed','cube_edge_mm':20,'port_requested':port,
                'motion_commands_sent':False,'pid_writes_sent':False,'torque_writes_sent':False,
                'camera_capture':{'remote':remote,'downloaded':False}}
        save(folder/'record.json',record)
        link=None
        lines=[]
        try:
            link=CameraLink(port)
            record['port_resolved']=link.port.port
            link.command(0x80,12)
            record['firmware']=list(struct.unpack('<III',link.read(12)))
            record['capture_started']=stamp()
            save(folder/'record.json',record)
            board_record=None
            for line in link.execute(build_script(config,remote,protocol),timeout=45):
                text=line.decode(errors='replace')
                lines.append(text)
                if text.startswith('PAIR_RECORD '):
                    board_record=json.loads(text[len('PAIR_RECORD '):])
                elif text.startswith('PAIR_STAGE '):
                    print(text,flush=True)
            record['capture_finished']=stamp()
            if board_record is None:
                raise RuntimeError('K230 did not return paired feedback; see board_log.txt')
            save(folder/'board_record.json',board_record)
            if board_record.get('sample_type')!=sample_type:raise RuntimeError('Board returned wrong contact protocol')
            record.update(board_record)
            record['raw_only']=True
            if board_record['status']!='raw_pair_saved_on_camera_sd':
                raise RuntimeError(board_record.get('error','K230 pair incomplete'))
            record['encoder_summary']=summarize(record)
            if protocol=='tool-pivot' and any(any(v!=0 for v in axis['torque_states']) for axis in record['encoder_summary'].values()):
                raise RuntimeError('Pivot capture requires all six axes torque off; no torque writes sent')
            record['camera_capture']['downloaded']=False
            save(folder/'record.json',record)
            image=download_image(link,record['camera_capture'])
            (folder/'frame.jpg').write_bytes(image)
            record['camera_capture']['downloaded']=True
            record['status']='raw_pair_saved_locally'
            entry['status']=record['status']
            print('GROUP',group,'SAVED',folder,flush=True)
            print('ENCODER_MEDIANS',[record['encoder_summary'][str(i)]['median'] for i in range(1,7)],flush=True)
            print('ENCODER_SPANS',[record['encoder_summary'][str(i)]['span_counts'] for i in range(1,7)],flush=True)
        except Exception as exc:
            record['status']='incomplete'
            record['error']=str(exc)
            entry['status']='incomplete'
            raise
        finally:
            if link is not None:
                link.port.close()
            save(folder/'record.json',record)
            save(collection_path,collection)
            (folder/'board_log.txt').write_text('\n'.join(lines)+'\n')


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--session',type=Path)
    parser.add_argument('--protocol',choices=('fixed-tip-top','tool-pivot'),default='fixed-tip-top')
    parser.add_argument('--port',default=PORT)
    args=parser.parse_args()
    default=ROOT/('data/k230/tool_reference_pivot_20261023' if args.protocol=='tool-pivot' else 'data/k230/extrinsics/formal_fixed_tip_top_20261022')
    collect((args.session or default).resolve(),args.port,args.protocol)


if __name__=='__main__':
    main()
