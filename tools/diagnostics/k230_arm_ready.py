#!/usr/bin/env python3
"""K230->URT-2 ready pose using the existing saved baseline/profiles. Default read-only."""
import argparse
from datetime import datetime
import hashlib
import json
from pathlib import Path
import struct
import sys
from zoneinfo import ZoneInfo
sys.path.insert(0,str(Path(__file__).resolve().parents[2]))
from host.project_paths import ROOT,bootstrap
bootstrap()
from host.transport.download_calibration import CameraLink,PORT
from host.arm.task_controller import Configuration,REGISTERS
from host.apps.move_ready import ready_task

def config_payload():
    config=Configuration(ROOT)
    profile=json.loads((ROOT/'configs/arm/ready_pose.json').read_text())
    task=ready_task(config,profile)
    if profile.get('model_floor_check_enabled') is not False:
        raise RuntimeError('Ready-only floor exception configuration changed; review entry')
    return {'goals':task['steps'][0]['positions'],'gripper_opens_deg':profile['angles_deg'][5],
            'profiles':[config.profiles[i] for i in range(1,7)],
            'registers':[{str(a):config.baseline['parameters'][str(i)][name] for a,name in REGISTERS.items()} for i in range(1,7)],
            'software_limits':[config.mapping.joint(i)['software_count_limits'] for i in range(1,7)]}

def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--port',default=PORT)
    parser.add_argument('--execute',action='store_true')
    parser.add_argument('--hold-current',action='store_true')
    parser.add_argument('--hands-free',action='store_true')
    args=parser.parse_args()
    if args.hold_current and not args.execute:parser.error('--hold-current requires --execute')
    if args.execute and not args.hold_current and not args.hands_free:
        parser.error('Ready movement requires --hands-free after current-hold confirmation')
    action='hold' if args.hold_current else 'ready' if args.execute else 'read'
    config=config_payload()
    read_source=(ROOT/'device/k230/diagnostics/uart1_servo_read.py').read_text().split('\nif __name__ == "__main__":')[0]
    board_source=(ROOT/'device/k230/arm/ready_pose.py').read_text()
    source=read_source+'\n'+board_source+'\nrun_arm_ready(ujson.loads('+repr(json.dumps(config))+'),'+repr(action)+')\n'
    script='try:\n exec('+repr(source)+')\nexcept BaseException as exc:\n import sys\n sys.print_exception(exc)\nfinally:\n print("EXPORT_DONE")\n'
    out=ROOT/'data/logs'/datetime.now(ZoneInfo('Asia/Shanghai')).strftime('k230_arm_ready_%Y%m%d_%H%M%S_%f');out.mkdir(parents=True)
    result={'status':'failed','action':action,'config':config,'events':[],
            'pid_write':False,'eeprom_write':False,'automatic_torque_release':False,
            'baseline_sha256':hashlib.sha256((ROOT/'configs/arm/user_selected_pid_demo_baseline.json').read_bytes()).hexdigest()}
    link=None;lines=[]
    try:
        link=CameraLink(args.port);link.command(0x80,12);result['k230_firmware']=list(struct.unpack('<III',link.read(12)))
        for raw in link.execute(script,timeout=45):
            text=raw.decode(errors='replace');lines.append(text);print(text,flush=True)
            if text.startswith('ARM_EVENT '):result['events'].append(json.loads(text[10:]))
        if 'ARM_ACTION_PASS '+action not in lines:raise RuntimeError('K230 arm action failed; no recovery or release sent')
        result['status']='passed';return 0
    except Exception as exc:
        result['error']=str(exc);print('ARM FAILED:',exc,flush=True);return 1
    finally:
        if link is not None:link.port.close()
        (out/'record.json').write_text(json.dumps(result,ensure_ascii=False,indent=2)+'\n')
        (out/'board_log.txt').write_text('\n'.join(lines)+'\n')
        print('RECORD',out,flush=True)

if __name__=='__main__':raise SystemExit(main())
