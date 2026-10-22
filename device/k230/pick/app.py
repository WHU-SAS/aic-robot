"""Standalone voice -> camera -> pose -> IK -> four arm stages.
No motion at boot: an accepted wake event is required for each run. No chassis
commands, PID writes, automatic return, release or queued extra pick actions.
"""
import os,gc,json,time
from camera import CameraSession
from vision import localize
from kinematics import Geometry
from control import run_pick_action


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
        with open(config['enrollment_file']) as f:keyword=json.load(f)
        eng=engine(config['detection_sensitivity']);eng.add_keyword(keyword)
        audio=PyAudio();stream=audio.open(format=paInt16,channels=1,rate=16000,
             input=True,frames_per_buffer=CHUNK)
        stream.volume(vol=keyword.get('mic_volume',100))
        for unused in range(32):os.exitpoint();stream.read()
        eng.reset();log('voice_ready',{'word':config['wake_word']})
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
                event=events[0];log('wake',event);return event
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
    # UART1 is read-only here. Stage runner preflights again before writes.
    from control import FPIOA,UART,preflight
    pins=FPIOA();pins.set_function(3,FPIOA.UART1_TXD);pins.set_function(4,FPIOA.UART1_RXD)
    uart=UART(UART.UART1,baudrate=1000000,bits=UART.EIGHTBITS,
              parity=UART.PARITY_NONE,stop=UART.STOPBITS_ONE)
    try:return preflight(uart,config)
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
    hold=dict(control);hold['goals']=plan['hover'];run_pick_action(hold,'hold');log('torque_on')
    for name in ('hover','descend','close','lift'):
        action=dict(control);action['goals']=plan[name]
        stage={'name':name,'duration_s':config['pick']['stage_durations_s'].get(name,0),
          'stationary_s':config['pick']['stage_durations_s']['close_minimum_stationary_s'],
          'tolerance_counts':config['pick']['gripper_stationary_tolerance_counts']}
        run_pick_action(action,'stage',stage)
    log('complete_torque_on',{'actual_grasp_success':'requires observation'})
    return plan


def run_forever(root):
    config=load(root)
    log('startup_waiting_for_wake',{'fixed_jaw_offset_mm':-config['pick']['cube_offset_toward_moving_finger_mm']})
    while True:
        try:wait_for_wake(config['voice'])
        except Exception as exc:
            import sys
            sys.print_exception(exc);log('voice_failed',str(exc))
            raise
        try:run_once(config,execute=True)
        except Exception as exc:
            # Do not dispatch any remaining stages, reverse goals or torque-off.
            import sys
            sys.print_exception(exc);log('pick_failed_no_extra_motion',str(exc))
        gc.collect()
