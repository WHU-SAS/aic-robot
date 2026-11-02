"""Standalone voice commands: pick or the existing ready pose.
No motion at boot: an accepted wake event is required for each run. No chassis
commands, PID writes, automatic return, release or queued extra pick actions.
"""
import os,gc,json,time,math
from camera import CameraSession
from vision import localize
from vision_runtime import localize_precise
from kinematics import Geometry
from control import run_pick_stage_direct,run_ready_direct
from voice_commands import registered_commands,resolve_events


def load(root):
    with open(root+'/config.json') as f:config=json.load(f)
    # Manifest fingerprints identify the exact deployed source, including local builds.
    try:
        with open(root+'/manifest.json') as f:version=json.load(f)
        config['_evidence_version']={'release_path':root,'manifest':version}
    except OSError as exc:
        config['_evidence_version']={'release_path':root,'manifest_error':str(exc)}
    return config

def log(event,details=None,persist=True):
    record={'event':event,'ticks_ms':time.ticks_ms(),'details':details}
    print('AIC_PICK '+json.dumps(record))
    if not persist:return
    try:
        with open('/sdcard/aic/pick/events.jsonl','a') as f:f.write(json.dumps(record)+'\n')
    except OSError as exc:print('LOG_ERROR '+str(exc))


def wait_for_wake(config,warmup_chunks=32,on_command=None):
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
            if events:
                command=resolve_events(events,commands)
                if command is None:
                    log('voice_command_ignored',events);continue
                # The visible servo response precedes SD logging and audio teardown.
                if on_command is not None:on_command(command['action'])
                log('wake',{'word':command['word'],'action':command['action'],'events':events,
                            'recognition_ticks_ms':now})
                return command['action']
            if time.ticks_diff(now,last_status)>=5000:
                log('voice_listening',{'chunks_read':chunks,'audio_ms':chunks*CHUNK*1000//16000},persist=False)
                last_status=now
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


def _perceive(config,evidence=None):
    vision=config['vision'];intr=config['intrinsics']
    session=CameraSession(vision,False);timing={};perception_started=time.ticks_ms()
    try:
        phase=time.ticks_ms();session.open()
        timing['camera_open_warmup']=time.ticks_diff(time.ticks_ms(),phase)
        phase=time.ticks_ms();img,frame=session.capture()
        timing['snapshot']=time.ticks_diff(time.ticks_ms(),phase)
        # Preserve RGB565 capture and JPEG95 decoding used by the accepted PC
        # pipeline; only the transfer to the computer has been removed.
        image_path='/sdcard/aic/pick/last_frame.jpg'
        if evidence is not None:
            try:image_path=evidence.new_frame('pick',frame)
            except Exception as exc:log('vision_archive_error',str(exc))
        phase=time.ticks_ms();img.save(image_path,quality=95)
        timing['jpeg_save']=time.ticks_diff(time.ticks_ms(),phase)
    finally:
        phase=time.ticks_ms();session.close()
        timing['camera_close']=time.ticks_diff(time.ticks_ms(),phase)
    import cv2
    phase=time.ticks_ms();im=cv2.imread(image_path)
    timing['jpeg_read']=time.ticks_diff(time.ticks_ms(),phase)
    phase=time.ticks_ms();observation=localize_precise(im,frame,intr,config['extrinsics'],
        vision['localization'].get('table_plane_Base_mm') if vision['localization'].get('table_pose_constraints_enabled') else None,
        localizer=localize)
    if evidence is not None:
        try:evidence.record_observation('pick',observation)
        except Exception as exc:log('vision_archive_error',str(exc))
    if observation['status']!='localized':raise ValueError(observation['reason'])
    found=observation.pop('payload');found['observation']=observation
    timing['visual_localization']=time.ticks_diff(time.ticks_ms(),phase)
    timing['capture_and_localize']=time.ticks_diff(time.ticks_ms(),perception_started)
    found['perception_timing_ms']=timing
    del im;gc.collect();log('localized',found)
    return found


def perceive(config):
    # Save the same JPEG used for inference once, retaining successful images too:
    # a later human report may identify an inaccurate but accepted localization.
    evidence=None;status='perception_error';error=None
    try:
        from evidence import EvidenceRun
        snapshot={key:config[key] for key in ('vision','intrinsics','extrinsics')}
        evidence=EvidenceRun('/sdcard/aic/vision/evidence','arm_pick',snapshot,
                             version=config.get('_evidence_version'))
    except Exception as exc:log('vision_archive_error',str(exc))
    try:
        result=_perceive(config,evidence)
        status='localized'
        return result
    except BaseException as exc:
        error=str(exc)
        raise
    finally:
        # Finalize BEFORE arm dispatch; keep all stage rules and the post-lift
        # one-second voice-resume policy untouched by archive I/O.
        if evidence is not None:
            try:
                saved=evidence.finish(status,error)
                log('vision_evidence_saved',{'directory':evidence.directory,
                    'manifest':evidence.manifest_path,'status':saved['status'],
                    'archive_complete':saved['archive_complete']})
            except Exception as exc:log('vision_archive_error',str(exc))


def _table_plane(config):
    if 'table_plane' in config:return config['table_plane']
    return config['vision']['localization']['table_plane_Base_mm']


def _same_calibration(config,source):
    if not isinstance(source,dict):
        raise ValueError('Pick handoff requires source_config calibration')
    try:
        for key in ('camera_matrix','distortion_coefficients','image_width','image_height'):
            if config['intrinsics'][key]!=source['intrinsics'][key]:
                raise ValueError('Pick handoff intrinsics mismatch: '+key)
        if config['extrinsics']['T_Base_from_Camera_mm']!=source['extrinsics']['T_Base_from_Camera_mm']:
            raise ValueError('Pick handoff extrinsics mismatch')
        if _table_plane(config)!=_table_plane(source):
            raise ValueError('Pick handoff table plane mismatch')
        for key in ('width','height','hmirror','vflip'):
            if config['vision']['camera'][key]!=source['vision']['camera'][key]:
                raise ValueError('Pick handoff camera mode mismatch: '+key)
    except (KeyError,TypeError):
        raise ValueError('Pick handoff calibration is incomplete')


def _handoff_target(observation):
    """Check capability and units; the task owns stopped/fresh/window attestation.

    A long static solve retains its capture time. This consumer does not renew
    it or impose a second generic capture-age limit after parking acceptance.
    """
    expected={'schema':'aic.vision_observation.v1','mode':'precise','status':'localized',
              'coordinate_frame':'Base','units':'mm','reference_point':'cube_geometric_center'}
    if not isinstance(observation,dict) or any(observation.get(k)!=v for k,v in expected.items()):
        raise ValueError('Pick handoff requires a complete precise Base observation')
    if (observation.get('xy_compensation_applied') is not False or
            observation.get('odometry_predicted') is True or observation.get('range_only') is True):
        raise ValueError('Pick handoff cannot use compensated, predicted or range-only targets')
    stamp=observation.get('capture_timestamp_ms')
    if (observation.get('image_size_px')!=[1280,960] or observation.get('frame_id') is None or
            type(stamp) is not int or stamp<0 or observation.get('timestamp_ms',stamp)!=stamp or
            observation.get('clock_domain')!='k230_ticks_ms'):
        raise ValueError('Pick handoff requires the original board capture identity')
    center=observation.get('cube_center_Base_mm')
    if (not isinstance(center,(list,tuple)) or len(center)!=3 or
            any(type(v) not in (int,float) or not math.isfinite(v) for v in center)):
        raise ValueError('Pick handoff requires finite raw cube XYZ')
    quality=observation.get('quality') or {}
    rms=quality.get('reprojection_rms_px')
    if type(rms) not in (int,float) or not math.isfinite(rms) or not 0<=rms<=2:
        raise ValueError('Pick handoff does not meet the existing 2px face criterion')
    raw=observation.get('payload') or {}
    if not isinstance(raw,dict):raise ValueError('Pick handoff payload must be a result object')
    found=dict(raw);found['cube_center_Base_mm']=list(center);found['observation']=observation
    return found


def _run_target(config,found,execute,start_counts=None):
    vision=config['vision']
    control=config['control']
    if execute:
        if start_counts is not None:raise ValueError('Live pick requires current arm feedback')
        import control as arm_control
        arm_control.print=record_arm_event
        rows=arm_snapshot(control);start=[r['position'] for r in rows]
    else:
        if (not isinstance(start_counts,(list,tuple)) or len(start_counts)!=6 or
                any(type(v) is not int for v in start_counts)):
            raise ValueError('Offline pick planning requires six start_counts')
        start=list(start_counts)
    xy=found['cube_center_Base_mm'][:2];plane=vision['localization']['table_plane_Base_mm'];n=plane['normal']
    table_z=(plane['offset_mm']-n[0]*xy[0]-n[1]*xy[1])/n[2]
    geometry=Geometry(config['model'],config['pick'],table_z)
    if config['pick'].get('wrist_alignment_enabled') is True:
        from cube_orientation import estimate
        orientation=estimate(found,config['extrinsics']['T_Base_from_Camera_mm'],plane)
        log('cube_orientation',orientation)
        plan=geometry.plan([v/1000 for v in xy],start,orientation)
    else:
        plan=geometry.plan([v/1000 for v in xy],start)
    log('planned',plan)
    if not execute:return plan
    for name in ('hover','descend','close','lift'):
        action=dict(control);action['goals']=plan[name]
        stage={'name':name,'duration_s':config['pick']['stage_durations_s'].get(name,0),
          'stationary_s':config['pick']['stage_durations_s']['close_minimum_stationary_s'],
          'tolerance_counts':config['pick']['gripper_stationary_tolerance_counts'],
          'policy':config['pick']['standalone_stage_policy']}
        arrival=run_pick_stage_direct(action,stage)
        if name=='descend':log('descend_position_sample',arrival)
        if name=='close':
            plan['grip_confirmation']=arrival
            plan['grasp_confirmed']=arrival.get('grasp_confirmed') if isinstance(arrival,dict) else None
    log('complete_torque_on',{'actual_grasp_success':'requires observation','torque_state':'unchanged_not_checked',
        'grasp_confirmed':plan.get('grasp_confirmed'),'grip_confirmation':plan.get('grip_confirmation')})
    return plan



def run_from_observation(config,observation,execute=False,start_counts=None,source_config=None):
    """Plan/pick the exact final static result, without camera or localization.

    Caller must already accept this frame as parked, retain its original capture
    identity and prevent further chassis movement until the pick is dispatched.
    Live mode reads current joints; dry planning requires explicit start counts.
    No PID, torque, stage policy, correction or return behavior is added here.
    """
    _same_calibration(config,source_config)
    found=_handoff_target(observation)
    log('handoff_localized',{'frame_id':observation['frame_id'],
        'capture_timestamp_ms':observation['capture_timestamp_ms'],
        'cube_center_Base_mm':found['cube_center_Base_mm'],'source':'accepted_static_observation'})
    return _run_target(config,found,execute,start_counts)



def begin_from_observation(config,observation,target_version,source_config=None,emit=None):
    """Plan a cooperative pick without taking ownership of camera or motion.

    The task keeps the shared camera and owns target versions. Construction
    reads current joints once and plans, but sends no servo targets. The caller
    must pass its current version to every execution.step(target_version=...).
    invalidate_target() cancels pending stages; the next step sends ready pose2.
    Re-observation may start only when the execution reports can_reobserve.
    close() releases transport; it is not an in-flight servo stop.
    """
    _same_calibration(config,source_config)
    found=_handoff_target(observation)
    import control as arm_control
    factory=arm_control.open_pick_execution
    ready=config['drive_ready_control']
    policy=dict(config['pick']['standalone_stage_policy'])
    policy.update(config['pick'].get('tracking_stage_policy',{}))
    rows=arm_snapshot(config['control'])
    start=[row['position'] for row in rows]
    plan=_run_target(config,found,False,start_counts=start)
    log('tracking_handoff_localized',{'frame_id':observation['frame_id'],
        'capture_timestamp_ms':observation['capture_timestamp_ms'],
        'cube_center_Base_mm':found['cube_center_Base_mm'],
        'target_version':target_version,'source':'accepted_static_observation'})
    if emit is None:emit=lambda event,details:log(event,details)
    return factory(config['control'],ready,plan,policy,target_version,emit=emit)


def run_once(config,execute=False):
    found=perceive(config)
    if not execute:return found
    return _run_target(config,found,True)


def run_ready(config,pose=1,immediate=False):
    if pose not in (1,2):raise ValueError('Ready pose must be1 or2')
    import control as arm_control
    arm_control.print=record_arm_event
    ready=config['ready_control' if pose==1 else 'drive_ready_control']
    if not immediate:
        log('ready_started',{'pose':pose,'goals':ready['goals'],'gripper_opens_deg':ready.get('gripper_opens_deg')})
    result=run_ready_direct(ready)
    log('ready_dispatched' if pose==1 else 'drive_ready_dispatched',
        {'pose':pose,'goals':ready['goals'],'arrival':'not_checked',
         'torque_state':'confirmed_on_before_dispatch','immediate_response':immediate,
         'goal_sent_ticks_ms':result.get('goal_sent_ticks_ms') if isinstance(result,dict) else None})
    return result


def make_wake_response(config):
    """Dispatch the selected ready target before voice cleanup; retain failures."""
    response={'sent':False,'error':None}
    def respond(action):
        if action not in ('pick','ready'):return
        try:
            run_ready(config,pose=2 if action=='pick' else 1,immediate=True)
            response['sent']=True
        except Exception as exc:
            # Let the listener release audio normally. Never start the task
            # when its immediate ready-target response failed.
            response['error']=exc
    return response,respond


def run_forever(root):
    config=load(root)
    log('startup_waiting_for_wake',{'fixed_jaw_offset_mm':-config['pick']['cube_offset_toward_moving_finger_mm']})
    warmup_chunks=32
    while True:
        response,respond=make_wake_response(config)
        try:action=wait_for_wake(config['voice'],warmup_chunks=warmup_chunks,on_command=respond)
        except Exception as exc:
            import sys
            sys.print_exception(exc);log('voice_failed',str(exc))
            raise
        warmup_chunks=0
        try:
            if response['error'] is not None:raise response['error']
            if action=='pick':
                if not response['sent']:run_ready(config,pose=2,immediate=True)
                run_once(config,execute=True)
            elif action=='ready':
                if not response['sent']:run_ready(config)
            else:log('voice_command_ignored',action)
        except Exception as exc:
            # Do not dispatch any remaining stages, reverse goals or torque-off.
            import sys
            sys.print_exception(exc);log('ready_failed_no_extra_motion' if action=='ready' else 'pick_failed_no_extra_motion',str(exc))
        gc.collect()
