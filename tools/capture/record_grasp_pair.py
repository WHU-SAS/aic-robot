#!/usr/bin/env python3
"""Bracket a K230 photograph with read-only six-servo feedback and offline FK."""

import sys as _sys
from pathlib import Path as _Path
_sys.path.insert(0, str(_Path(__file__).resolve().parents[2]))
from host.project_paths import ROOT as REPO_ROOT, bootstrap as _bootstrap
_bootstrap(include_tools=True)
from datetime import datetime
from zoneinfo import ZoneInfo
import json
import os
from pathlib import Path
import shutil
import subprocess
import sys
import time

PROJECT = REPO_ROOT
HERE = (REPO_ROOT/'data/k230')
from host.transport.servo_bus import ServoBus
from host.arm.joint_mapping import JointMapping
from gripper_geometry import PickGeometry

PORT='/dev/serial/by-id/usb-1a86_USB_Single_Serial_5B79033467-if00'

def stamp():
    return datetime.now(ZoneInfo('Asia/Shanghai')).isoformat()

def sample(bus):
    readings={}
    for sid in range(1,7):
        started=stamp()
        position,speed=bus.read_position_speed(sid)
        torque=bus.read_torque(sid)
        readings[str(sid)]={'position':position,'speed':speed,'torque':torque,
                            'read_started':started,'read_finished':stamp()}
    return readings

def main():
    folder=HERE/'grasp_pairs'/datetime.now(ZoneInfo('Asia/Shanghai')).strftime('pair_%Y%m%d_%H%M%S')
    folder.mkdir(parents=True,exist_ok=False)
    record={'status':'pending','created':stamp(),'servo_port':PORT,
            'motion_commands_sent':False,'pid_writes_sent':False,
            'user_report':'Block stays on table; grasp posture unchanged',
            'samples_before':[],'samples_after':[]}
    path=folder/'record.json'
    def save():path.write_text(json.dumps(record,ensure_ascii=False,indent=2)+'\n')
    save()
    try:
        with ServoBus(PORT) as bus:
            for _ in range(3):
                record['samples_before'].append(sample(bus));time.sleep(.05)
            save()
            record['camera_operation_started']=stamp()
            env=os.environ.copy();env.pop('PYTHONPATH',None)
            result=subprocess.run(['/usr/bin/python3',str(REPO_ROOT/'tools/capture/capture_board_once.py')],
                                  text=True,capture_output=True,env=env)
            record['camera_operation_finished']=stamp()
            (folder/'camera_log.txt').write_text(result.stdout+result.stderr)
            for _ in range(3):
                record['samples_after'].append(sample(bus));time.sleep(.05)
            save()
            if result.returncode:
                raise RuntimeError('Camera capture failed; see camera_log.txt')
            output=next((s.removeprefix('CAPTURE_SAVED ') for s in result.stdout.splitlines()
                         if s.startswith('CAPTURE_SAVED ')),None)
            if output is None:raise RuntimeError('Missing camera capture path')
            camera_folder=Path(output)
            shutil.copy2(camera_folder/'board.jpg',folder/'frame.jpg')
            record['camera_capture']=json.loads((camera_folder/'capture.json').read_text())
            record['camera_source']=str(camera_folder)
        all_samples=record['samples_before']+record['samples_after']
        record['encoder_summary']={}
        representative=[]
        for sid in range(1,7):
            values=[s[str(sid)]['position'] for s in all_samples]
            ordered=sorted(values)
            median=int(round((ordered[2]+ordered[3])/2))
            representative.append(median)
            record['encoder_summary'][str(sid)]={
                'median':median,'min':min(values),'max':max(values),
                'span_counts':max(values)-min(values),
                'speeds_raw':[s[str(sid)]['speed'] for s in all_samples],
                'torque_states':[s[str(sid)]['torque'] for s in all_samples]}
        mapping=JointMapping.load(PROJECT/'configs/arm/joint_mapping.json')
        geometry=PickGeometry(PROJECT,mapping)
        q=geometry.angles(representative)
        mid,tips,lows,transforms=geometry.metrics(q)
        record['joint_angles_deg']=[float(x*180/3.141592653589793) for x in q]
        record['fk']={'reference':'Provisional midpoint of mean distal 2mm STL tip regions; NOT yet established as cube center',
                      'gripper_tip_midpoint_Base_mm':(mid*1000).tolist(),
                      'tip_centers_Base_mm':{k:(v*1000).tolist() for k,v in tips.items()},
                      'link_min_z_Base_mm':{k:float(v*1000) for k,v in lows.items()},
                      'T_Base_from_fixed_jaw':transforms['wan2'].tolist(),
                      'T_Base_from_moving_jaw':transforms['zhua'].tolist()}
        record['vision_status']='Raw photograph saved; center visibility and grasp reference pending analysis'
        record['status']='read_only_pair_recorded'
        save()
        print('PAIR_SAVED',folder,flush=True)
        print('ENCODERS',representative,flush=True)
        print('SPANS',[record['encoder_summary'][str(i)]['span_counts'] for i in range(1,7)],flush=True)
        print('FK_TIP_MIDPOINT_BASE_MM',record['fk']['gripper_tip_midpoint_Base_mm'],flush=True)
    except Exception as exc:
        record['status']='incomplete';record['error']=str(exc);save()
        raise

if __name__=='__main__':main()
