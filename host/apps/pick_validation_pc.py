#!/usr/bin/env python3
"""PC pick control with live K230 perception or explicit offline image replay."""
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
import math
import signal
import subprocess
import time
from zoneinfo import ZoneInfo
from host.arm.task_controller import Configuration
from host.arm.pick_validation_geometry import OpenTipPickGeometry as PickGeometry
from host.transport.download_calibration import PORT
from host.transport.k230_pick_perception import perceive as perceive_onboard
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
                    if event.get('dispatch_groups')==[[4],[1,2,3,5,6]]:
                        print('STAGE',event['name'],'4号已提前%d毫秒发送，其余五轴随后同步发送'%event['axis4_lead_ms'],flush=True)
                    else:
                        print('STAGE',event['name'],'目标已发送；等待本阶段条件',flush=True)
                elif event['event']=='gripper_settled':
                    record['gripper_settle']=event['details'];save()
                    print('夹爪连续%d次处于±%d计数静止带，准备抬升。'%(event['details']['sample_count'],event['details']['tolerance_counts']),flush=True)
                elif event['event']=='board_message' and ('Traceback' in event['text'] or 'Error' in event['text']):
                    print(event['text'],file=sys.stderr,flush=True)
            started=time.monotonic()
            if args.image:
                record.update(status='offline_image_replay',perception_backend='pc_offline');save()
                (folder/'frame.jpg').write_bytes(args.image.resolve().read_bytes())
                record['offline_image_source']=str(args.image.resolve())
                command=[str(ROOT/'scripts/vision'),str(ROOT/'host/vision/localize_pick_cube.py'),
                    '--image',str(folder/'frame.jpg'),'--output',str(folder),
                    '--config',str(ROOT/pick['vision_config'])]
                subprocess.run(command,cwd=ROOT,check=True,capture_output=True,text=True)
                record['timing_s']['vision']=time.monotonic()-started
                vision=json.loads((folder/'vision_record.json').read_text())
            else:
                record.update(status='onboard_perception',perception_backend='k230');save()
                print('K230拍照并完成识图，仅回传定位结果；请保持方块不动，IDE停止并断开。',flush=True)
                vision=perceive_onboard(folder,pick_config=args.pick_config,port=args.camera_port)
                elapsed=time.monotonic()-started
                record['timing_s'].update(onboard_perception=elapsed,vision=elapsed)
                if isinstance(vision,dict):
                    native_ms=vision.get('perception_timing_ms',{}).get('visual_localization')
                    if type(native_ms) in (int,float) and math.isfinite(native_ms) and native_ms>=0:
                        record['timing_s']['vision']=native_ms/1000
                    for key in ('perception_timing_ms','evidence','release_id','release_path','request_id'):
                        if key in vision:record[key]=vision[key]
            record['vision']=vision
            center=vision.get('cube_center_Base_mm') if isinstance(vision,dict) else None
            if (not isinstance(vision,dict) or vision.get('status')!='localized' or
                not isinstance(center,(list,tuple)) or len(center)!=3 or
                not all(type(v) in (int,float) and math.isfinite(v) for v in center) or
                vision.get('xy_compensation_applied') is not False):
                raise RuntimeError('本轮未得到完整、未补偿的方块定位；未访问机械臂')
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
                    'tolerance_counts':pick['gripper_stationary_tolerance_counts'],
                    'policy':pick['standalone_stage_policy']}
                record['motion_command_attempted']=True;record['status']=name+'_dispatching';save()
                result=run_action(payload(config,plan[name]),'stage_direct',stage,port=args.camera_port,emit=emit)
                completed=next(e for e in reversed(result) if e['event']=='stage_complete' and e['name']==name)
                value={'name':name,'goals':plan[name],'stage_complete':completed,
                    'final_feedback':None,'final_error_counts':None,'final_FK':None,
                    'feedback_source':'unavailable_stage_does_not_read_six_axis_positions'}
                if name in ('hover','descend'):
                    sample=next(e['details'] for e in reversed(result)
                        if e['event']=='position_gate_passed' and e['name']==name)
                    counts=sample['positions']
                    value.update(position_gate_sample=sample,
                        final_feedback=[{'id':i,'position':count} for i,count in enumerate(counts,1)],
                        final_error_counts=[v-g for v,g in zip(counts,plan[name])],
                        final_FK=geometry.report(counts),feedback_source='position_gate_passed')
                    if name=='descend':record['descend_position_sample']=sample
                record['stages'].append(value);save()
                print('STAGE_COMPLETE',name,json.dumps({'error_counts':value['final_error_counts'],'FK':value['final_FK']},ensure_ascii=False),flush=True)
            record.update(status='motion_sequence_complete_torque_on',physical_grasp_success='operator_observation_required')
            save();print('流程完成：保留抬升姿态和扭矩，实际夹取结果请现场观察。',flush=True)
            return 0
    except BaseException as exc:
        record.update(status='error_no_extra_motion',error=repr(exc))
        if record.get('perception_backend')=='k230':
            diagnostic=folder/'onboard_perception.json'
            if diagnostic.is_file():
                try:record['onboard_perception']=json.loads(diagnostic.read_text())
                except (OSError,ValueError) as detail:record['onboard_diagnostic_error']=str(detail)
        if isinstance(exc,subprocess.CalledProcessError):
            record['vision_error']=exc.stderr or exc.stdout;print(record['vision_error'],file=sys.stderr)
        else:print(str(exc),file=sys.stderr)
        save();print('已停止后续阶段；不自动回位或关闭扭矩。',file=sys.stderr)
        return 1
    finally:
        if (folder/'frame.jpg').is_file() and (folder/'vision_candidates.json').is_file():
            try:
                from host.vision.hard_dataset import add_case
                record['hard_case_id']=add_case(folder,'recognition_failure',
                    'PC精定位拒绝，自动保留原图与当轮配置；未赋予实测坐标真值。')
                save()
            except Exception as exc:print('VISION_ARCHIVE_ERROR '+str(exc),file=sys.stderr,flush=True)
        print('RECORD',folder/'execution.json',flush=True)


if __name__=='__main__':raise SystemExit(main())
