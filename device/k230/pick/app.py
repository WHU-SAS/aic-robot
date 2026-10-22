"""Standalone voice commands: pick or the existing ready pose.
No motion at boot: an accepted wake event is required for each run. No chassis
commands, PID writes, automatic return, release or queued extra pick actions.
"""
import os,gc,json,time
from camera import CameraSession
from vision import localize
from kinematics import Geometry
from control import run_pick_stage_direct,run_ready_direct
from voice_commands import registered_commands,resolve_events


def load(root):
    with open(root+'/config.json') as f:return json.load(f)

def log(event,details=None,persist=True):
    record={'event':event,'ticks_ms':time.ticks_ms(),'details':details}
    print('AIC_PICK '+json.dumps(record))
    if not persist:return
    try:
        with open('/sdcard/aic/pick/events.jsonl','a') as f:f.write(json.dumps(record)+'\n')
    except OSError as exc:print('LOG_ERROR '+str(exc))


def wait_for_wake(config):
    # Release audio stream and KWS before camera/media initialization. A fresh
    # engine after each pick discards buffered audio and repeated wake events.
    from wake_word import engine,CHUNK
    from media.pyaudio import PyAudio,paInt16
    eng=audio=stream=None
    try:
        commands=registered_commands(config)
        eng=engine(config['detection_sensitivity']);volume=None
        for command in commands:
            with open(command['enrollment_file']) as f:keyword=json.load(f)
            if keyword['kw_id']!=command['keyword_id']:
                raise ValueError('Keyword ID does not match binding')
            eng.add_keyword(keyword)
            v=keyword.get('mic_volume',100)
            if volume is not None and v!=volume:raise ValueError('Keyword microphone volumes differ')
            volume=v
        audio=PyAudio();stream=audio.open(format=paInt16,channels=1,rate=16000,
             input=True,frames_per_buffer=CHUNK)
        stream.volume(vol=volume)
        for unused in range(32):os.exitpoint();stream.read()
        eng.reset();log('voice_ready',{'words':[c['word'] for c in commands]})
        last_status=time.ticks_ms();chunks=0
        while True:
            os.exitpoint();pcm=stream.read()
            if len(pcm)!=CHUNK*2:raise RuntimeError('Incomplete detection audio chunk')
            events=eng.push(pcm);chunks+=1
            now=time.ticks_ms()
            if time.ticks_diff(now,last_status)>=5000:
                log('voice_listening',{'chunks_read':chunks,'audio_ms':chunks*CHUNK*1000//16000},persist=False)
                last_status=now
            if events:
                command=resolve_events(events,commands)
                if command is None:
                    log('voice_command_ignored',events);continue
                log('wake',{'word':command['word'],'action':command['action'],'events':events})
                return command['action']
    finally:
        if stream is not None:stream.stop_stream();stream.close()
        if audio is not None:audio.terminate()
        if eng is not None:eng.deinit()
        gc.collect()


def record_arm_event(*args):
    message=' '.join(str(v) for v in args)
    if message.startswith('PICK_EVENT ') or message.startswith('ARM_EVENT '):
        log('arm_feedback',json.loads(message.split(' ',1)[1]))
    else:print(message)


def arm_snapshot(config):
    # Only read the six positions needed to select an IK solution.
    from control import FPIOA,UART,read_checked,signed_position
    pins=FPIOA();pins.set_function(3,FPIOA.UART1_TXD);pins.set_function(4,FPIOA.UART1_RXD)
    uart=UART(UART.UART1,baudrate=1000000,bits=UART.EIGHTBITS,
              parity=UART.PARITY_NONE,stop=UART.STOPBITS_ONE)
    try:
        rows=[]
        for i in range(1,7):
            raw=read_checked(uart,i,56,2)
            rows.append({'id':i,'position':signed_position(raw[0],raw[1])})
        return rows
    finally:uart.deinit()


def run_once(config,execute=False):
    vision=config['vision'];intr=config['intrinsics'];K=[list(row) for row in intr['camera_matrix']]
    for j in range(3):K[0][j]*=1280/intr['image_width'];K[1][j]*=960/intr['image_height']
    session=CameraSession(vision,False)
    try:
        session.open();img=session.snapshot()
        # Preserve RGB565 capture and JPEG95 decoding used by the accepted PC
        # pipeline; only the transfer to the computer has been removed.
        img.save('/sdcard/aic/pick/last_frame.jpg',quality=95)
    finally:session.close()
    import cv2
    im=cv2.imread('/sdcard/aic/pick/last_frame.jpg')
    found=localize(im,K,intr['distortion_coefficients'],config['extrinsics']['T_Base_from_Camera_mm'])
    del im;gc.collect();log('localized',found)
    if not execute:return found
    import control as arm_control
    arm_control.print=record_arm_event
    control=config['control'];rows=arm_snapshot(control);start=[r['position'] for r in rows]
    xy=found['cube_center_Base_mm'][:2];plane=vision['localization']['table_plane_Base_mm'];n=plane['normal']
    table_z=(plane['offset_mm']-n[0]*xy[0]-n[1]*xy[1])/n[2]
    geometry=Geometry(config['model'],config['pick'],table_z)
    plan=geometry.plan([v/1000 for v in xy],start);log('planned',plan)
    for name in ('hover','descend','close','lift'):
        action=dict(control);action['goals']=plan[name]
        stage={'name':name,'duration_s':config['pick']['stage_durations_s'].get(name,0),
          'stationary_s':config['pick']['stage_durations_s']['close_minimum_stationary_s'],
          'tolerance_counts':config['pick']['gripper_stationary_tolerance_counts']}
        run_pick_stage_direct(action,stage)
    log('complete_torque_on',{'actual_grasp_success':'requires observation','torque_state':'unchanged_not_checked'})
    return plan


def run_ready(config):
    import control as arm_control
    arm_control.print=record_arm_event
    ready=config['ready_control']
    log('ready_started',{'goals':ready['goals'],'gripper_opens_deg':30})
    run_ready_direct(ready)
    log('ready_dispatched',{'arrival':'not_checked','torque_state':'unchanged_not_read'})


def run_forever(root):
    config=load(root)
    log('startup_waiting_for_wake',{'fixed_jaw_offset_mm':-config['pick']['cube_offset_toward_moving_finger_mm']})
    while True:
        try:action=wait_for_wake(config['voice'])
        except Exception as exc:
            import sys
            sys.print_exception(exc);log('voice_failed',str(exc))
            raise
        try:
            if action=='pick':run_once(config,execute=True)
            elif action=='ready':run_ready(config)
            else:log('voice_command_ignored',action)
        except Exception as exc:
            # Do not dispatch any remaining stages, reverse goals or torque-off.
            import sys
            sys.print_exception(exc);log('ready_failed_no_extra_motion' if action=='ready' else 'pick_failed_no_extra_motion',str(exc))
        gc.collect()
