"""Standalone voice entry: continuous approach, park, then grasp the same target.

Boot only listens. The ready command sends pose1; pick wake first sends pose2.
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


def tracking_task(config, ready_dispatched=False):
    """Candidate continuous task; keep one camera through parking and grasp."""
    import app as arm
    from ground_projection import GroundProjection
    from motor_link import MotorLink
    from camera import CameraSession
    from blue_tracker import BlueTracker, age_ms
    from vision_runtime import track_coarse
    from physical_side_session import FineSession
    from cooperative_static_session import CooperativeStaticPickSession
    from robot_tracking import TrackingTask, ReobserveRequired
    from kinematics import Geometry
    if not ready_dispatched:
        arm.run_ready(config['arm'], pose=2, immediate=True)
    car = config['car']
    model = car['projection']
    projection = GroundProjection(model['K'], model['D'], model['T'], model['plane'],
        model.get('forward_axis_Base'), model.get('motion_origin_Base_mm'))
    tracker = BlueTracker(car['tracking'], projection)
    camera = CameraSession(car['vision'])
    link = task = None
    frames, result = [], None
    geometry = Geometry(config['arm']['model'], config['arm']['pick'], 0)
    def capture():
        os.exitpoint()
        return camera.capture()
    def begin(observation, version):
        def steps():
            try:
                yield from arm.begin_from_observation_steps(config['arm'], observation,
                    version, source_config=car)
            except arm.PickPlanUnavailable as exc:
                raise ReobserveRequired(str(exc))
        return steps()
    def held_context(state):
        sample = state.get('last_sample') or {}
        if (state.get('phase') not in ('lift', 'complete') or
                state.get('grasp_confirmed') is not True or
                sample.get('axes') != [1,2,3,4,5,6] or
                not 0 <= age_ms(time.ticks_ms(), sample.get('timestamp_ms', -10000)) <=
                2*car['tracking_task']['period_ms']):
            return state
        # This FK point is a broad visual association reference, never a new
        # measured cube centre or parking/grasp authority.
        midpoint = geometry.metrics(geometry.angles(sample['positions']))[0]
        return dict(state, held_center_pixel=projection.base_to_pixel([p*1000 for p in midpoint]))
    try:
        link = MotorLink()
        camera.open()
        task = TrackingTask(capture,
            lambda image, frame:track_coarse(image, frame, tracker.detect_frame),
            link, projection, car, time.ticks_ms,
            begin,
            lambda c, clock:FineSession(c, clock, purpose='car_moving_range'),
            CooperativeStaticPickSession, frames, emit=emit,
            reset_tracker=tracker.reset)
        task.arm_context = held_context
        while task.status != 'succeeded':
            task.step()
            time.sleep_ms(5)
        result = task.snapshot()
        return result
    finally:
        try:
            if task is not None:
                task.close()
        finally:
            try:
                camera.close()
            finally:
                if link is not None:
                    link.close()
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


def dispatch(action, config, ready_dispatched=False):
    import app as arm
    if action == 'ready':
        if not ready_dispatched:arm.run_ready(config['arm'])
        return {'status':'ready_dispatched', 'ready_pose':1, 'arrival_checked':False}
    if action != 'pick':
        raise ValueError('Unknown robot voice action: ' + str(action))
    if config['car'].get('tracking_task', {}).get('enabled'):
        return tracking_task(config, ready_dispatched)
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
        return arm.run_from_observation(config['arm'], observation,
            execute=True, source_config=config['car'])
    try:
        result = execute_task(drive, grasp, emit)
        return result
    except BaseException as exc:
        error = str(exc)
        raise
    finally:
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


def run_forever(root):
    import app as arm
    config = load(root)
    emit('configured', {'release_path':root, 'trigger':'思达思达',
                        'ready_command':'回准备位', 'startup_motion':False,
                        'ready_command_pose':1,'pick_wake_response_pose':2,
                        'wait_for_ready_arrival':False})
    warmup = 32
    while True:
        response,respond = arm.make_wake_response(config['arm'])
        action = arm.wait_for_wake(config['voice'], warmup,on_command=respond)
        warmup = 0
        try:
            if response['error'] is not None:raise response['error']
            result = dispatch(action, config,ready_dispatched=response['sent'])
            emit('task_finished', {'status':result['status'],'grasp_confirmed':result.get('grasp_confirmed'),
                                   'grip_confirmation':result.get('grip_confirmation')})
        except Exception as exc:
            emit('task_error', {'error':str(exc)})
        finally:
            gc.collect()
