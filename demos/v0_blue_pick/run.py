#!/usr/bin/env python3
"""Repeatable K230 capture -> OpenCV -> project1 IK/pick flow; retains final torque."""

import sys as _sys
from pathlib import Path as _Path
_sys.path.insert(0, str(_Path(__file__).resolve().parents[2]))
from host.project_paths import ROOT as REPO_ROOT, bootstrap as _bootstrap
_bootstrap(include_tools=False)
import argparse,fcntl,hashlib,json,math,os,signal,subprocess,sys,termios,time
from datetime import datetime
from pathlib import Path
from zoneinfo import ZoneInfo
ROOT=REPO_ROOT
from task_controller import Configuration,TaskController,IDS
from blue_pick_geometry import PickGeometry
from blue_pick_settle import PositionSettled
from host.transport.servo_bus import ServoBus,ServoError
SERVO_PORT='/dev/serial/by-id/usb-1a86_USB_Single_Serial_5B79033467-if00'
CAMERA_PORT='/dev/serial/by-id/usb-Kendryte_CanMV_001000000-if00'

def forward_pick_target(xy,extra_mm):
 # Apply the user-selected reach correction once to the fresh vision target.
 # Radial Base-XY extension preserves the target bearing, not a camera axis.
 radius=math.hypot(*xy)
 if radius==0 and extra_mm:raise ValueError('Forward extension direction undefined at Base origin')
 return [v*(1+extra_mm/1000/radius) for v in xy] if radius else list(xy)

def drive(controller,geometry,plan,save_stage,clock=time.monotonic,sleep=time.sleep):
 stages=[]
 for name,duration in [('hover',6),('descend',6),('close',3),('lift',8)]:
  goals=dict(enumerate(plan[name],1));controller.send(goals,'single_sync_'+name);save_stage({'event':'stage_started','name':name,'goals':goals});deadline=clock()+duration;samples=[]
  settled=PositionSettled(seconds=.5,tolerance=1) if name=='close' else None
  if settled is not None:print('等待6号连续0.5秒静止（允许±1计数），通过后才抬升。',flush=True)
  while name=='close' or clock()<deadline:
   rows=controller.sample();now=clock();counts=[rows[i]['position'] for i in IDS];samples.append({'t':now,'rows':rows,'FK':geometry.report(counts)})
   if settled is not None and settled.update(rows[6]['position'],now):break
   sleep(.05)
  rows=controller.sample()
  if settled is not None:
   while not settled.update(rows[6]['position'],clock()):
    sleep(.05);rows=controller.sample();now=clock();samples.append({'t':now,'rows':rows,'FK':geometry.report([rows[i]['position'] for i in IDS])})
   save_stage({'event':'gripper_settled','details':settled.report(clock())})
  stage={'name':name,'goals':goals,'final_feedback':rows,'final_FK':geometry.report([rows[i]['position'] for i in IDS]),'samples':samples};stages.append(stage);save_stage({'event':'stage_complete','stage':stage})
  print('STAGE '+name+' '+json.dumps({'FK':stage['final_FK'],'final_error_counts':{i:rows[i]['position']-goals[i] for i in IDS}},ensure_ascii=False),flush=True)
 return stages

def main(argv=None,default_backend='pc'):
 p=argparse.ArgumentParser(description=__doc__);p.add_argument('--execute',action='store_true');p.add_argument('--hands-free',action='store_true',help='operator already removed support force and hands from motion area');p.add_argument('--image',type=Path,help='offline replay image; forbidden with --execute');p.add_argument('--start-counts',nargs=6,type=int,default=[2048]*6,help='offline replay start only; live run reads actual counts');p.add_argument('--port',default=SERVO_PORT);p.add_argument('--camera-port',default=CAMERA_PORT);p.add_argument('--check-env',action='store_true');p.add_argument('--detect-only',action='store_true',help='camera localization only; never opens the servo port');p.add_argument('--debug-roi',action='store_true',help='detect-only diagnostic: export just the cube neighborhood and its global offset');p.add_argument('--debug-image',action='store_true',help='K230 detect-only diagnostic: export the same final recognition frame');p.add_argument('--vision-backend',choices=['pc','k230'],default=default_backend);p.add_argument('--features',type=Path,help='offline K230 feature replay only; forbidden with --execute');args=p.parse_args(argv)
 if args.debug_roi:args.debug_image=True
 if args.debug_image and (not args.detect_only or args.vision_backend!='k230' or args.features or args.image):p.error('--debug-image requires live K230 --detect-only')
 if args.detect_only and args.execute:p.error('--detect-only cannot be combined with --execute')
 if args.image and args.features:p.error('--image and --features are mutually exclusive')
 if args.execute and (args.image or args.features):p.error('--execute requires fresh K230 capture, offline inputs are forbidden')
 if args.features and args.vision_backend!='k230':p.error('--features requires the K230 backend')
 if args.image and args.vision_backend!='pc':p.error('Use board feature replay, or the offline image replay utility, for the K230 backend')
 if args.check_env:
  import numpy,scipy,serial
  from scipy.optimize import least_squares
  least_squares(lambda x:x,[1.]);print('CONTROL',sys.executable,numpy.__version__,scipy.__version__)
  subprocess.run([str(ROOT/'scripts/vision'),'-c','import cv2,numpy; print("VISION",cv2.__version__,numpy.__version__)'],cwd=ROOT,check=True);return 0
 cfg=Configuration(ROOT);pick=json.loads((ROOT/'demos/v0_blue_pick/pick.json').read_text());geometry=PickGeometry(ROOT,cfg.mapping,table_z_m=pick['table_z_Base_mm']/1000,hover_tip_height_mm=pick['hover_tip_height_mm'],descend_tip_height_mm=pick['descend_tip_height_mm'],lift_tip_height_mm=pick['lift_tip_height_mm'],opening_count=pick['opening_count'])
 folder=ROOT/'data/logs'/datetime.now(ZoneInfo('Asia/Shanghai')).strftime('k230_pick_%Y%m%d_%H%M%S_%f');folder.mkdir(parents=True,exist_ok=False)
 record={'status':'created','execute':args.execute,'motion_performed':False,'pid_writes_sent':False,'torque_enable_attempted':False,'task':'hover(descend open)+close+lift; project1 flow','runtime_pick_configuration':pick,'baseline':cfg.baseline,'motion_defaults':cfg.defaults,'per_run_authorization':'User requested project1 full pick flow for supervised personal execution; global manual policy unchanged','stages':[],'vision_backend':args.vision_backend,'timing_s':{}}
 def save():
  temporary=folder/'execution.tmp';temporary.write_text(json.dumps(record,ensure_ascii=False,indent=2)+'\n');temporary.replace(folder/'execution.json')
 def stopped(signum,frame):raise KeyboardInterrupt('process stop signal '+str(signum))
 signal.signal(signal.SIGTERM,stopped);save()
 try:
  phase_started=time.monotonic()
  if args.features:
   (folder/'board_features.json').write_bytes(args.features.resolve().read_bytes());record['source_offline_features']=str(args.features.resolve());record['status']='offline_board_features';save()
  elif args.image:
   source=args.image.resolve()
   (folder/'frame.jpg').write_bytes(source.read_bytes());record['source_offline_image']=str(source);record['status']='offline_vision';save()
  else:
   if args.vision_backend=='k230':
    from capture_board_features import capture
    print('K230正在提取角点并导出同帧诊断原图；本次不驱动机械臂。' if args.debug_image else 'K230正在提取方块角点，仅传坐标；IDE需停止并断开。',flush=True)
   else:
    from capture_pick_frame import capture
    print('重新拍摄K230画面并导出；请保持方块不动，IDE需停止并断开。',flush=True)
   record['status']='camera_capture';save();record['capture']=capture(folder,args.camera_port,debug_image=args.debug_image,debug_roi=args.debug_roi) if args.vision_backend=='k230' else capture(folder,args.camera_port);save()
  record['timing_s']['capture_or_feature_receive']=time.monotonic()-phase_started;phase_started=time.monotonic()
  vision_script='localize_board_faces.py' if args.vision_backend=='k230' else 'localize_pick_cube.py'
  vision_input=['--features',str(folder/'board_features.json')] if args.vision_backend=='k230' else ['--image',str(folder/'frame.jpg')]
  record['status']='vision_localization';save()
  subprocess.run([str(ROOT/'scripts/vision'),str(ROOT/'host/vision'/vision_script),*vision_input,'--output',str(folder)],cwd=ROOT,check=True,capture_output=True,text=True)
  record['status']='vision_localized'
  record['timing_s']['PC_pose_localization']=time.monotonic()-phase_started
  vision=json.loads((folder/'vision_record.json').read_text());record['vision']=vision;xy=[float(v)/1000 for v in vision['cube_center_Base_mm'][:2]];save();print('CUBE_CENTER_Base_mm '+json.dumps(vision['cube_center_Base_mm']),flush=True)
  if args.detect_only:
   record['status']='localization_complete_no_servo_access';save();print('识别完成；未打开舵机串口。TIMING '+json.dumps(record['timing_s']),flush=True);return 0
  original_xy=list(xy);extra_mm=float(pick.get('forward_extension_mm',0));xy=forward_pick_target(original_xy,extra_mm)
  record['target_compensation']={'direction':'radial Base XY from origin','forward_extension_mm':extra_mm,'vision_xy_Base_mm':[v*1000 for v in original_xy],'commanded_xy_Base_mm':[v*1000 for v in xy]};save()
  print('GRASP_TARGET_Base_XY_mm '+json.dumps(record['target_compensation']['commanded_xy_Base_mm']),flush=True)
  if args.image or args.features:
   start=args.start_counts;cfg.check_counts(dict(enumerate(start,1)));phase_started=time.monotonic();plan=geometry.plan(xy,start);record['timing_s']['IK_plan']=time.monotonic()-phase_started
   for name in ['hover','descend','close','lift']:cfg.check_counts(dict(enumerate(plan[name],1)))
   record.update(status='offline_plan_complete_no_hardware',start_counts=start,plan=plan);save();print('PLAN '+json.dumps(plan,ensure_ascii=False));return 0
  lockpath=ROOT/'data/run/controller.lock';lockpath.parent.mkdir(parents=True,exist_ok=True)
  with lockpath.open('a') as lock,(folder/'events.jsonl').open('a',buffering=1) as events:
   fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
   def emit(e):events.write(json.dumps(e,ensure_ascii=False)+'\n')
   with ServoBus(args.port) as bus:
    fcntl.ioctl(bus.port.ser.fileno(),termios.TIOCEXCL)
    try:
     def basic(start,target):cfg.check_start_counts(start,c.limits);cfg.check_counts(target,c.limits);return {'mode':'project1_basic_limits_only'}
     c=TaskController(bus,cfg,basic,emit);c.verify_parameters();fresh=c.sample();start=[fresh[i]['position'] for i in IDS];phase_started=time.monotonic();plan=geometry.plan(xy,start);record['timing_s']['IK_plan']=time.monotonic()-phase_started
     for name in ['hover','descend','close','lift']:cfg.check_counts(dict(enumerate(plan[name],1)),c.limits)
     record.update(status='planned',start_counts=start,plan=plan);save();print('PLAN '+json.dumps({name:plan[name] for name in ['hover','descend','close','lift']}),flush=True)
     if not args.execute:record['status']='fresh_plan_only_no_motion';save();print('PLAN_ONLY 无扭矩或位置写入。');return 0
     step={'name':'hover','goals':dict(enumerate(plan['hover'],1)),'hold_s':0}
     record['torque_enable_attempted']=True;save()
     for attempt in range(30):
      try:c.prepare([step]);break
      except ServoError as exc:
       if 'hold initialization requires fresh stopped position' not in str(exc):raise
       time.sleep(.1)
     else:raise RuntimeError('Cannot initialize fresh current hold')
     record['status']='holding_current_pose';save();print('六轴扭矩已开启，当前位置已锁定。',flush=True)
     if not args.hands_free:
      if input('请撤掉托扶力、手离开运动区；输入“已放手”开始完整流程：').strip()!='已放手':raise RuntimeError('未确认放手，未启动抓取流程')
     print('开始项目1同款四阶段流程：张开并悬停 → 下降 → 闭合停稳 → 抬升。',flush=True)
     def stage_event(event):
      emit(event)
      if event['event']=='stage_started':record.update(status=event['name']+'_running',motion_performed=True)
      elif event['event']=='stage_complete':record['stages'].append(event['stage'])
      elif event['event']=='gripper_settled':record['gripper_settle']=event['details']
      save()
     drive(c,geometry,plan,stage_event);record.update(status='motion_sequence_complete_torque_on',physical_grasp_success='operator_observation_required');record['final_FK']=record['stages'][-1]['final_FK'];save();print('流程完成，保留抬升姿态和扭矩；实际是否抓起请现场观察。',flush=True)
    finally:fcntl.ioctl(bus.port.ser.fileno(),termios.TIOCNXCL)
 except BaseException as exc:
  failed_phase=record['status']
  record.update(status='error_no_extra_motion',error=repr(exc))
  if isinstance(exc,subprocess.CalledProcessError):record['subprocess_stderr']=exc.stderr;record['subprocess_stdout']=exc.stdout;print((exc.stderr or exc.stdout or str(exc)).strip(),file=sys.stderr)
  else:print(str(exc),file=sys.stderr)
  if isinstance(exc,Exception) and failed_phase=='vision_localization' and args.vision_backend=='k230' and not args.features and not args.image:
   try:
    from capture_board_features import fetch_saved_diagnostic
    record['failure_diagnostic']=fetch_saved_diagnostic(folder,args.camera_port)
    if record['failure_diagnostic']['status']=='saved':print('同帧局部诊断图已自动保存：'+str(folder/'frame_roi.jpg'),file=sys.stderr)
   except Exception as diagnostic_error:record['failure_diagnostic_error']=repr(diagnostic_error)
  save()
  if not record['torque_enable_attempted'] and not record['motion_performed']:print('失败发生在定位/规划阶段，未发送移动或扭矩指令。',file=sys.stderr)
  else:print('未继续发送阶段目标；程序未自动关闭扭矩。',file=sys.stderr)
  return 1
 finally:print('RECORD '+str(folder/'execution.json'),flush=True)
 return 0

if __name__=='__main__':raise SystemExit(main())
