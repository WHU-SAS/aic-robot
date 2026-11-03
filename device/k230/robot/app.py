"""Standalone voice entry: continuous approach, park, then grasp the same target.

Tracking boot prepares its camera, then listens without actuator motion.
The ready command sends pose1; pick wake first sends pose2.
Successful execution ends at lift/holding; no automatic release, return or PID writes.
"""
import gc
import json
import os
import time


def emit(event, details=None):
    # No SD traffic in the moving loop or between final vision and arm planning.
    print('AIC_ROBOT ' + json.dumps({'event':event, 'ticks_ms':time.ticks_ms(),
                                   'details':details}))


def load(root):
    with open(root + '/config.json') as stream:
        config = json.load(stream)
    with open(root + '/manifest.json') as stream:
        version = {'release_path':root, 'manifest':json.load(stream)}
    config['arm']['_evidence_version'] = version
    config['_evidence_version'] = version
    return config


def approach(config, frames):
    from ground_projection import GroundProjection
    from motor_link import MotorLink
    from camera import CameraSession
    from blue_tracker import BlueTracker
    from vision_runtime import track_coarse
    from near_runtime import run_parking
    from physical_side_session import FineSession
    from static_pick_session import StaticPickSession
    model = config['projection']
    projection = GroundProjection(model['K'], model['D'], model['T'], model['plane'],
        model.get('forward_axis_Base'), model.get('motion_origin_Base_mm'))
    tracker = BlueTracker(config['tracking'], projection)
    camera = CameraSession(config['vision'])
    link = None
    try:
        link = MotorLink()
        camera.open()
        def capture():
            os.exitpoint()
            return camera.capture()
        return run_parking(capture,
            lambda image, frame: track_coarse(image, frame, tracker.detect_frame),
            link, projection, config, time.ticks_ms, time.sleep_ms,
            fine_factory=StaticPickSession, evidence=frames,
            moving_fine_factory=lambda c, clock: FineSession(c, clock, purpose='car_moving_range'))
    finally:
        try:
            camera.close()
        finally:
            if link is not None:
                link.close()


def archive(config, frames, status, error=None):
    if not frames:
        return None
    from evidence import EvidenceRun
    run = EvidenceRun('/sdcard/aic/vision/evidence', 'robot_task', config,
                      version=config.get('_evidence_version'))
    for item in frames:
        run.save_jpeg(item['label'], item['jpeg'], item['frame'], item['observation'])
    saved = run.finish(status, error)
    return {'directory':run.directory, 'frame_count':len(frames),
            'archive_complete':saved['archive_complete']}


def prepare_tracking_steps(config):
    """Preload pure tracking/vision dependencies for one fixed boot config.

    No UART, camera instance, image, tracker history or task is retained here.
    The Geometry instance is for held-object FK only, not per-scene pick IK.
    """
    started = step_started = time.ticks_ms()
    elapsed = getattr(time, 'ticks_diff', lambda a,b:a-b)
    work_ms = 0
    import app as arm
    import vision  # Static fine's lazy solver, including OpenCV/ulab dependencies.
    duration = elapsed(time.ticks_ms(), step_started)
    work_ms += duration
    yield {'stage':'vision_loaded','ms':duration}
    step_started = time.ticks_ms()
    from ground_projection import GroundProjection
    from motor_link import MotorLink
    from camera import CameraSession
    from blue_tracker import BlueTracker, age_ms
    from vision_runtime import track_coarse
    duration = elapsed(time.ticks_ms(), step_started)
    work_ms += duration
    yield {'stage':'coarse_loaded','ms':duration}
    step_started = time.ticks_ms()
    from physical_side_session import FineSession
    from cooperative_static_session import CooperativeStaticPickSession
    duration = elapsed(time.ticks_ms(), step_started)
    work_ms += duration
    yield {'stage':'fine_loaded','ms':duration}
    # Reuse the existing audio/camera warmup turns. New recovery modules must
    # not be compiled for the first time after the final parking photograph.
    for name in ('edge_profiles', 'separator_profiles', 'profile_recovery',
                 'cube_pose', 'cube_corners', 'cube_observation'):
        step_started = time.ticks_ms()
        __import__(name)
        duration = elapsed(time.ticks_ms(), step_started)
        work_ms += duration
        yield {'stage':'image_math_loaded','module':name,'ms':duration}
    step_started = time.ticks_ms()
    from robot_tracking import TrackingTask, ReobserveRequired
    duration = elapsed(time.ticks_ms(), step_started)
    work_ms += duration
    yield {'stage':'control_loaded','ms':duration}
    step_started = time.ticks_ms()
    model = config['car']['projection']
    projection = GroundProjection(model['K'], model['D'], model['T'], model['plane'],
        model.get('forward_axis_Base'), model.get('motion_origin_Base_mm'))
    prepared = dict(arm=arm, projection=projection,
        MotorLink=MotorLink, CameraSession=CameraSession, BlueTracker=BlueTracker,
        TrackingTask=TrackingTask, ReobserveRequired=ReobserveRequired,
        FineSession=FineSession, StaticSession=CooperativeStaticPickSession,
        track_coarse=track_coarse, age_ms=age_ms,
        prepare_ms=work_ms+elapsed(time.ticks_ms(), step_started),
        prepare_elapsed_ms=age_ms(time.ticks_ms(), started))
    yield {'prepared':prepared}


def prepare_tracking(config):
    """Synchronous fallback for direct callers without boot preparation."""
    for event in prepare_tracking_steps(config):
        if 'prepared' in event:
            return event['prepared']


def tracking_task(config, ready_dispatched=False, camera=None, prepared=None, wake_timing=None,
                  fill_light=None):
    """Keep one camera through parking/grasp; the voice owner may lend it."""
    entered_at = time.ticks_ms()
    if prepared is None:
        import app as arm
    else:
        arm = prepared['arm']
    if not ready_dispatched:
        arm.run_ready(config['arm'], pose=2, immediate=True)
    ready_done = time.ticks_ms()
    reused_preparation = prepared is not None
    prepared = prepare_tracking(config) if prepared is None else prepared
    preparation_done = time.ticks_ms()
    age_ms = prepared['age_ms']
    car = config['car']
    projection = prepared['projection']
    tracker = prepared['BlueTracker'](car['tracking'], projection)
    owns_camera = camera is None
    if owns_camera:
        camera = prepared['CameraSession'](car['vision'])
    tracker_done = time.ticks_ms()
    link = task = None
    frames, result = [], None
    def capture():
        os.exitpoint()
        return camera.capture()
    def begin(observation, version):
        def steps():
            try:
                yield from arm.begin_from_observation_steps(config['arm'], observation,
                    version, source_config=car)
            except arm.PickPlanUnavailable as exc:
                raise prepared['ReobserveRequired'](str(exc))
        return steps()
    try:
        link_started_at = time.ticks_ms()
        link = prepared['MotorLink']()
        link_ready_at = time.ticks_ms()
        camera_started_at = camera_ready_at = None
        if owns_camera:
            camera_started_at = time.ticks_ms()
            camera.open()
            camera_ready_at = time.ticks_ms()
        task_started_at = time.ticks_ms()
        task = prepared['TrackingTask'](capture,
            lambda image, frame:prepared['track_coarse'](image, frame, tracker.detect_frame),
            link, projection, car, time.ticks_ms,
            begin,
            lambda c, clock:prepared['FineSession'](c, clock, purpose='car_moving_range'),
            prepared['StaticSession'], frames, emit=emit,
            reset_tracker=tracker.reset)
        task.metrics.update(task_entered_at_ms=entered_at,
            camera_started_at_ms=camera_started_at, camera_ready_at_ms=camera_ready_at,
            camera_open_ms=age_ms(camera_ready_at, camera_started_at) if owns_camera else 0,
            camera_reused=not owns_camera,
            preparation_reused=reused_preparation,
            ready_dispatch_ms=age_ms(ready_done, entered_at),
            preparation_ms=age_ms(preparation_done, ready_done),
            tracker_setup_ms=age_ms(tracker_done, preparation_done),
            callback_setup_ms=age_ms(link_started_at, tracker_done),
            motor_open_ms=age_ms(link_ready_at, link_started_at),
            task_init_ms=age_ms(time.ticks_ms(), task_started_at),
            task_setup_ms=age_ms(time.ticks_ms(), entered_at))
        if wake_timing is not None:
            task.metrics['wake_timing'] = dict(wake_timing)
        while task.status != 'succeeded':
            task.step()
            if task.status == 'succeeded' and fill_light is not None:
                # This is the existing post-lift confirmation, before archive.
                fill_light.off()
            delay = task.idle_delay_ms()
            if delay:
                time.sleep_ms(delay)
        result = task.snapshot()
        return result
    finally:
        try:
            try:
                if fill_light is not None:
                    fill_light.off()
            finally:
                try:
                    if task is not None:
                        task.close()
                finally:
                    try:
                        if owns_camera:
                            camera.close()
                    finally:
                        if link is not None:
                            link.close()
        finally:
            # An interrupt can arrive inside close(), not only the task loop.
            # Persist this run before the same cleanup exception propagates.
            record = result or (task.snapshot() if task is not None else
                                {'status':'tracking_initialization_interrupted'})
            record.update(ready_pose2_goal_sent=True,ready_pose2_arrival_checked=False,
                arm_started_scope='grasp_after_parking')
            try:
                record['evidence'] = archive(config, frames, record['status'])
            except Exception as exc:
                record['archive_error'] = str(exc)
                emit('vision_archive_error', {'error':str(exc)})
            frames.clear()
            try:
                with open('/sdcard/aic/robot/runs.jsonl','a') as stream:
                    stream.write(json.dumps(record)+'\n')
            except OSError as exc:
                emit('log_error',{'error':str(exc)})


def dispatch(action, config, ready_dispatched=False, camera=None, prepared=None, wake_timing=None,
             fill_light=None):
    import app as arm
    if action == 'ready':
        if fill_light is not None:fill_light.off()
        if not ready_dispatched:arm.run_ready(config['arm'])
        return {'status':'ready_dispatched', 'ready_pose':1, 'arrival_checked':False}
    if action != 'pick':
        raise ValueError('Unknown robot voice action: ' + str(action))
    if config['car'].get('tracking_task', {}).get('enabled'):
        options = {}
        if camera is not None:
            options['camera'] = camera
        if prepared is not None:
            options['prepared'] = prepared
        if wake_timing is not None:
            options['wake_timing'] = wake_timing
        if fill_light is not None:
            options['fill_light'] = fill_light
        return tracking_task(config, ready_dispatched, **options)
    from robot_task import execute_task
    if not ready_dispatched:arm.run_ready(config['arm'],pose=2,immediate=True)
    frames, parking = [], []
    result, error, arm_started = None, None, False
    def drive():
        found = approach(config['car'], frames)
        parking.append(found)
        return found
    def grasp(observation):
        nonlocal arm_started
        arm_started = True
        plan = arm.run_from_observation(config['arm'], observation,
            execute=True, source_config=config['car'])
        if fill_light is not None and isinstance(plan, dict) and plan.get('grasp_confirmed') is True:
            fill_light.off()
        return plan
    try:
        result = execute_task(drive, grasp, emit)
        return result
    except BaseException as exc:
        error = str(exc)
        raise
    finally:
        if fill_light is not None:fill_light.off()
        record = result or {'schema':'aic.robot.task.v1', 'status':'runtime_error',
                            'error':error, 'arm_started':arm_started,
                            'parking':parking[-1] if parking else None}
        # arm_started names the post-parking grasp, not the wake acknowledgement.
        record.update(ready_pose2_goal_sent=True,ready_pose2_arrival_checked=False,
                      arm_started_scope='grasp_after_parking')
        try:
            record['evidence'] = archive(config, frames, record['status'], error)
        except Exception as exc:
            record['archive_error'] = str(exc)
            emit('archive_error', {'error':str(exc)})
        finally:
            frames.clear()
        try:
            with open('/sdcard/aic/robot/runs.jsonl', 'a') as stream:
                stream.write(json.dumps(record) + '\n')
        except OSError as exc:
            emit('log_error', {'error':str(exc)})


def _make_fill_light(config):
    options = config.get('fill_light', {})
    if not options.get('enabled', False):
        return None
    if options.get('gpio') != 26 or options.get('active_high') is not True:
        raise ValueError('Fill light wiring must be active-high GPIO26')
    from fill_light import FillLight
    return FillLight()


class _CameraWarmupFailed(RuntimeError):
    pass


def run_forever(root):
    boot_entered = time.ticks_ms()
    import app as arm
    arm_imported = time.ticks_ms()
    config = load(root)
    fill_light = _make_fill_light(config)
    emit('configured', {'release_path':root, 'trigger':'思达思达',
                        'ready_command':'回准备位', 'startup_motion':False,
                        'ready_command_pose':1,'pick_wake_response_pose':2,
                        'wait_for_ready_arrival':False,
                        'arm_import_ms':time.ticks_diff(arm_imported,boot_entered),
                        'config_load_ms':time.ticks_diff(time.ticks_ms(),arm_imported)})
    timing = config.get('car', {}).get('tracking_task', {})
    tracking = timing.get('enabled')
    if not tracking:
        arm.preload_localization()
    camera = prepared = None
    camera_started = camera_start_ms = None
    startup_pending = bool(tracking)
    startup_steps = prepare_tracking_steps(config) if tracking else None
    camera_warmed = False
    camera_warmup_ms = 0
    def prepare_startup():
        nonlocal prepared, startup_pending, camera_warmed, camera_warmup_ms
        if camera is not None and not camera_warmed:
            started = time.ticks_ms()
            try:
                camera_warmed = camera.warmup_step()
            except Exception as exc:
                # Audio's finally must release its stream before MediaManager
                # can be closed by this owner; preserve the voice fallback.
                raise _CameraWarmupFailed(str(exc))
            camera_warmup_ms += time.ticks_diff(time.ticks_ms(),started)
            if camera_warmed:
                emit('camera_prepared', {'camera_open_ms':camera_start_ms+camera_warmup_ms,
                    'camera_start_ms':camera_start_ms,'camera_warmup_ms':camera_warmup_ms,
                    'camera_ready_since_start_ms':time.ticks_diff(time.ticks_ms(),camera_started),
                    'overlapped_with_audio':True,'reused_between_tasks':True})
        if prepared is None:
            event = next(startup_steps)
            if 'prepared' in event:
                prepared = event['prepared']
                emit('tracking_prepared', {'prepare_ms':prepared['prepare_ms'],
                    'prepare_elapsed_ms':prepared.get('prepare_elapsed_ms'),
                    'before_voice_ready':True})
        startup_pending = prepared is None or camera is not None and not camera_warmed
        return not startup_pending
    try:
        if tracking:
            from camera import CameraSession
            camera = CameraSession(config['car']['vision'])
            camera_started = time.ticks_ms()
            try:
                # Keep the known camera->audio media initialization order.
                # Only snapshots/delays and pure preparation overlap audio.
                camera.start()
                camera_start_ms = time.ticks_diff(time.ticks_ms(),camera_started)
                emit('camera_stream_started',{'camera_start_ms':camera_start_ms})
            except Exception as exc:
                camera.close()
                camera = None
                emit('camera_prepare_error', {'error':str(exc)})
        warmup = timing.get('cold_start_audio_warmup_chunks',32) if tracking else 32
        while True:
            response,respond = arm.make_wake_response(config['arm'])
            on_command = respond
            if fill_light is not None:
                def on_command(action, respond=respond):
                    # Keep the original immediate ready target first. Only a
                    # GPIO write follows, before audio teardown or logging.
                    respond(action)
                    fill_light.set_active(action == 'pick')
            options = {'on_command':on_command}
            if startup_pending:
                options['on_warmup'] = prepare_startup
            try:
                action = arm.wait_for_wake(config['voice'], warmup,**options)
            except _CameraWarmupFailed as exc:
                # wait_for_wake has already closed audio/KWS. A failed camera
                # must not permanently remove ready/voice control.
                camera.close()
                camera = None
                startup_pending = prepared is None
                emit('camera_prepare_error',{'error':str(exc)})
                continue
            voice_returned_at = time.ticks_ms()
            warmup = 0
            try:
                if response['error'] is not None:raise response['error']
                options = {'ready_dispatched':response['sent']}
                if fill_light is not None:options['fill_light'] = fill_light
                if camera is not None:options['camera'] = camera
                if prepared is not None:options['prepared'] = prepared
                if tracking and response.get('recognized_at_ms') is not None:
                    options['wake_timing'] = {key:response[key] for key in
                        ('recognized_at_ms','ready_goal_sent_at_ms')}
                    options['wake_timing']['voice_returned_at_ms'] = voice_returned_at
                result = dispatch(action, config,**options)
                emit('task_finished', {'status':result['status'],'grasp_confirmed':result.get('grasp_confirmed'),
                                       'grip_confirmation':result.get('grip_confirmation')})
            except Exception as exc:
                emit('task_error', {'error':str(exc)})
            finally:
                if fill_light is not None:fill_light.off()
                gc.collect()
    finally:
        try:
            if fill_light is not None:fill_light.off()
        finally:
            try:
                if startup_steps is not None:startup_steps.close()
            finally:
                if camera is not None:
                    camera.close()
