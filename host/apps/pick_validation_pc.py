#!/usr/bin/env python3
"""PC full-pick field validation: fresh K230 photo, candidate extrinsics, existing IK."""
import sys
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[2]))
from host.project_paths import ROOT,bootstrap
bootstrap()
import argparse
from datetime import datetime
import fcntl
import hashlib
import json
import signal
import subprocess
import time
from zoneinfo import ZoneInfo
from host.arm.task_controller import Configuration
from host.arm.pick_validation_geometry import OpenTipPickGeometry as PickGeometry
from host.transport.download_calibration import PORT
from host.transport.capture_pick_frame import capture
from host.transport.k230_pick_stage import payload,run_action
from host.vision.config import load_vision_config,table_height_at

TRIAL=ROOT/'configs/arm/pick_validation_pc.json'


def parameters(pick_config=None):
    profile=TRIAL if pick_config is None else ROOT/Path(pick_config)
    pick=json.loads(profile.read_text())
    if pick['extra_closing_counts']!=20 or pick['forward_extension_mm']!=0:
        raise ValueError('Validation profile changed; review the selected trial')
    if pick.get('alignment_reference')!='open_fingertip_midpoint':
        raise ValueError('Expected user-selected open-fingertip alignment')
    if 'model_urdf' in pick:
        if not isinstance(pick['model_urdf'],str) or not pick['model_urdf'].strip():
            raise ValueError('model_urdf must be a nonempty path')
        localization=load_vision_config(ROOT/pick['vision_config'])['localization']
        extrinsics=json.loads((ROOT/localization['extrinsics']).read_text())
        for key,path in (('geometry_model_sha256',ROOT/pick['model_urdf']),
                         ('joint_mapping_sha256',ROOT/'configs/arm/joint_mapping.json')):
            actual=hashlib.sha256(path.read_bytes()).hexdigest()
            if extrinsics.get(key)!=actual:
                raise ValueError('Model/extrinsics pair mismatch: '+key)
    return pick,Configuration(ROOT)


def make_plan(vision,start,pick,config,offset=(0,0,0)):
    settings=load_vision_config(ROOT/pick['vision_config'])['localization']
    xy=[float(v)+float(d) for v,d in zip(vision['cube_center_Base_mm'][:2],offset[:2])]
    table_z=table_height_at(settings,xy)
    geometry=PickGeometry(ROOT,config.mapping,table_z_m=table_z/1000,
        hover_tip_height_mm=pick['hover_tip_height_mm']+offset[2],
        descend_tip_height_mm=pick['descend_tip_height_mm']+offset[2],
        lift_tip_height_mm=pick['lift_tip_height_mm']+offset[2],opening_count=pick['opening_count'],
        cube_offset_toward_moving_finger_mm=pick['cube_offset_toward_moving_finger_mm'],
        urdf_path=pick.get('model_urdf'))
    config.check_counts(dict(enumerate(start,1)))
    plan=geometry.plan([v/1000 for v in xy],start)
    if pick.get('model_urdf'):
        model_sha=hashlib.sha256((ROOT/pick['model_urdf']).read_bytes()).hexdigest()
        plan['model_urdf']=pick['model_urdf']
        plan['geometry_model_sha256']=model_sha
        plan['model_correction_applied']=model_sha!=hashlib.sha256((ROOT/'models/genkiarm/genkiarm.urdf').read_bytes()).hexdigest()
    for name in ('hover','descend','close','lift'):
        config.check_counts(dict(enumerate(plan[name],1)))
    plan['target_xy_Base_mm']=xy
    plan['manual_offset_Base_mm']=list(offset)
    plan['forward_extension_mm']=0
    plan['table_plane_Base_mm']=settings['table_plane_Base_mm']
    return geometry,plan


def main(argv=None):
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--execute',action='store_true')
    parser.add_argument('--hands-free',action='store_true')
    parser.add_argument('--detect-only',action='store_true')
    parser.add_argument('--camera-port',default=PORT)
    parser.add_argument('--pick-config',type=Path,help='explicit pick profile; defaults to the accepted profile')
    parser.add_argument('--image',type=Path,help='offline replay; forbidden with execute')
    parser.add_argument('--start-counts',nargs=6,type=int,default=[2048]*6,help='offline start only')
    parser.add_argument('--offset-x-mm',type=float,default=0)
    parser.add_argument('--offset-y-mm',type=float,default=0)
    parser.add_argument('--offset-z-mm',type=float,default=0,help='operator-selected correction to all three fingertip heights')
    args=parser.parse_args(argv)
    if args.execute and (args.image or args.detect_only):parser.error('execute requires a fresh image and cannot use detect-only')
    pick,config=parameters(args.pick_config)
    folder=ROOT/'data/logs'/datetime.now(ZoneInfo('Asia/Shanghai')).strftime('pick_pc_%Y%m%d_%H%M%S_%f')
    folder.mkdir(parents=True,exist_ok=False)
    record={'status':'created','execute':args.execute,'motion_performed':False,
        'torque_enable_attempted':False,'pid_writes_sent':False,'stages':[],
        'pick_profile':pick,'baseline':config.baseline,'motion_defaults':config.defaults,
        'vision_config':pick['vision_config'],'timing_s':{},'arm_transport':'PC -> K230 USBDBG -> UART1 -> URT-2'}
    def save():
        temp=folder/'execution.tmp';temp.write_text(json.dumps(record,ensure_ascii=False,indent=2)+'\n');temp.replace(folder/'execution.json')
    def interrupt(signum,frame):raise KeyboardInterrupt('Stop signal '+str(signum))
    signal.signal(signal.SIGTERM,interrupt)
    save()
    try:
        lockpath=ROOT/'data/run/controller.lock';lockpath.parent.mkdir(parents=True,exist_ok=True)
        with lockpath.open('a') as lock,(folder/'events.jsonl').open('a',buffering=1) as events:
            fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
            def emit(event):
                events.write(json.dumps(event,ensure_ascii=False)+'\n')
                if event['event']=='stage_started':
                    record['motion_performed']=True;record['status']=event['name']+'_running';save()
                    print('STAGE',event['name'],'目标已一次同步发送',flush=True)
                elif event['event']=='gripper_settled':
                    record['gripper_settle']=event['details'];save()
                    print('夹爪连续0.5秒静止，准备抬升。',flush=True)
                elif event['event']=='board_message' and ('Traceback' in event['text'] or 'Error' in event['text']):
                    print(event['text'],file=sys.stderr,flush=True)
            started=time.monotonic()
            record['status']='camera_capture';save()
            if args.image:
                (folder/'frame.jpg').write_bytes(args.image.resolve().read_bytes())
                record['offline_image_source']=str(args.image.resolve())
            else:
                print('K230拍照并回传，PC识图；请保持方块不动，IDE停止并断开。',flush=True)
                record['capture']=capture(folder,args.camera_port)
            record['timing_s']['capture']=time.monotonic()-started
            command=[str(ROOT/'scripts/vision'),str(ROOT/'host/vision/localize_pick_cube.py'),
                '--image',str(folder/'frame.jpg'),'--output',str(folder),
                '--config',str(ROOT/pick['vision_config'])]
            record['status']='vision_localization';save();started=time.monotonic()
            subprocess.run(command,cwd=ROOT,check=True,capture_output=True,text=True)
            record['timing_s']['vision']=time.monotonic()-started
            vision=json.loads((folder/'vision_record.json').read_text());record['vision']=vision
            print('CUBE_CENTER_Base_mm',json.dumps(vision['cube_center_Base_mm']),flush=True)
            if args.detect_only:
                record['status']='localized_no_arm_access';save();return 0
            if args.image:
                start=args.start_counts
            else:
                read=run_action(payload(config,[2048]*6),'read',port=args.camera_port,emit=emit)
                start=[r['position'] for r in read[-1]['rows']]
            offset=(args.offset_x_mm,args.offset_y_mm,args.offset_z_mm)
            started=time.monotonic();geometry,plan=make_plan(vision,start,pick,config,offset)
            record.update(status='planned',start_counts=start,plan=plan)
            record['timing_s']['IK']=time.monotonic()-started;save()
            print('PLAN',json.dumps({name:plan[name] for name in ('hover','descend','close','lift')}),flush=True)
            print('TABLE_Z_AT_TARGET_mm',plan['table_z_Base_mm'],'MANUAL_OFFSET_mm',list(offset),'FORWARD_EXTENSION_mm',0,flush=True)
            if not args.execute:
                record['status']='plan_only_no_motion';save();print('规划完成，未开启扭矩或发送移动指令。');return 0
            record['torque_enable_attempted']=True;save()
            run_action(payload(config,plan['hover']),'hold',port=args.camera_port,emit=emit)
            record['status']='holding_current_pose';save()
            print('六轴扭矩已开启，当前位置已锁定。',flush=True)
            if not args.hands_free and input('请撤掉托扶力、手离开运动区，输入“已放手”开始：').strip()!='已放手':
                raise RuntimeError('未确认放手，未发送抓取目标')
            for name in ('hover','descend','close','lift'):
                duration=pick['stage_durations_s'].get(name,0)
                stage={'name':name,'duration_s':duration,
                    'stationary_s':pick['stage_durations_s']['close_minimum_stationary_s'],
                    'tolerance_counts':pick['gripper_stationary_tolerance_counts']}
                record['motion_command_attempted']=True;record['status']=name+'_dispatching';save()
                result=run_action(payload(config,plan[name]),'stage',stage,port=args.camera_port,emit=emit)
                completed=next(e for e in reversed(result) if e['event']=='stage_complete')
                counts=[row['position'] for row in completed['rows']]
                value={'name':name,'goals':plan[name],'final_feedback':completed['rows'],
                    'final_error_counts':[v-g for v,g in zip(counts,plan[name])],
                    'final_FK':geometry.report(counts)}
                record['stages'].append(value);save()
                print('STAGE_COMPLETE',name,json.dumps({'error_counts':value['final_error_counts'],'FK':value['final_FK']},ensure_ascii=False),flush=True)
            record.update(status='motion_sequence_complete_torque_on',physical_grasp_success='operator_observation_required')
            save();print('流程完成：保留抬升姿态和扭矩，实际夹取结果请现场观察。',flush=True)
            return 0
    except BaseException as exc:
        record.update(status='error_no_extra_motion',error=repr(exc))
        if isinstance(exc,subprocess.CalledProcessError):
            record['vision_error']=exc.stderr or exc.stdout;print(record['vision_error'],file=sys.stderr)
        else:print(str(exc),file=sys.stderr)
        save();print('已停止后续阶段；不自动回位或关闭扭矩。',file=sys.stderr)
        return 1
    finally:print('RECORD',folder/'execution.json',flush=True)


if __name__=='__main__':raise SystemExit(main())
