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


def wait_for_wake(config,warmup_chunks=32):
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
        # Full microphone warmup only on boot; resume has no discard delay.
        for unused in range(warmup_chunks):os.exitpoint();stream.read()
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


def record_descend_position(control,goals):
    # User-approved diagnostic only: one snapshot, no arrival gate or retry.
    started=time.ticks_ms()
    try:
        rows=arm_snapshot(control)
        positions=[row['position'] for row in rows]
        details={'goals':list(goals),'positions':positions,
                 'errors_actual_minus_goal':[a-b for a,b in zip(positions,goals)],
                 'read_ms':time.ticks_diff(time.ticks_ms(),started)}
    except Exception as exc:
        log('descend_position_sample_failed',{'error':str(exc),
            'read_ms':time.ticks_diff(time.ticks_ms(),started)})
        return
    log('descend_position_sample',details)


def run_once(config,execute=False):
    vision=config['vision'];intr=config['intrinsics'];K=[list(row) for row in intr['camera_matrix']]
    for j in range(3):K[0][j]*=1280/intr['image_width'];K[1][j]*=960/intr['image_height']
    session=CameraSession(vision,False);timing={};perception_started=time.ticks_ms()
    try:
        phase=time.ticks_ms();session.open()
        timing['camera_open_warmup']=time.ticks_diff(time.ticks_ms(),phase)
        phase=time.ticks_ms();img=session.snapshot()
        timing['snapshot']=time.ticks_diff(time.ticks_ms(),phase)
        # Preserve RGB565 capture and JPEG95 decoding used by the accepted PC
        # pipeline; only the transfer to the computer has been removed.
        phase=time.ticks_ms();img.save('/sdcard/aic/pick/last_frame.jpg',quality=95)
        timing['jpeg_save']=time.ticks_diff(time.ticks_ms(),phase)
    finally:
        phase=time.ticks_ms();session.close()
        timing['camera_close']=time.ticks_diff(time.ticks_ms(),phase)
    import cv2
    phase=time.ticks_ms();im=cv2.imread('/sdcard/aic/pick/last_frame.jpg')
    timing['jpeg_read']=time.ticks_diff(time.ticks_ms(),phase)
    phase=time.ticks_ms();found=localize(im,K,intr['distortion_coefficients'],config['extrinsics']['T_Base_from_Camera_mm'])
    timing['visual_localization']=time.ticks_diff(time.ticks_ms(),phase)
    timing['capture_and_localize']=time.ticks_diff(time.ticks_ms(),perception_started)
    found['perception_timing_ms']=timing
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
        if name=='descend':record_descend_position(control,plan[name])
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
    warmup_chunks=32
    while True:
        try:action=wait_for_wake(config['voice'],warmup_chunks=warmup_chunks)
        except Exception as exc:
            import sys
            sys.print_exception(exc);log('voice_failed',str(exc))
            raise
        warmup_chunks=0
        try:
            if action=='pick':run_once(config,execute=True)
            elif action=='ready':run_ready(config)
            else:log('voice_command_ignored',action)
        except Exception as exc:
            # Do not dispatch any remaining stages, reverse goals or torque-off.
            import sys
            sys.print_exception(exc);log('ready_failed_no_extra_motion' if action=='ready' else 'pick_failed_no_extra_motion',str(exc))
        gc.collect()
