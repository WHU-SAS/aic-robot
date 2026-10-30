"""One wake -> selected coarse approach or continuous near parking task."""
import gc
import json
import os
import time


def archive_fine_frames(config, frames, status, error=None):
    """Called after camera/UART close; original JPEGs never enter JSON logs."""
    if not frames:
        return None
    try:
        try:
            from evidence import EvidenceRun
        except ImportError:
            from device.k230.vision.evidence import EvidenceRun
        run = EvidenceRun('/sdcard/aic/vision/evidence', 'car_near', config,
                          version=config.get('runtime_version'))
        for item in frames:
            run.save_jpeg(item['label'], item['jpeg'], item['frame'], item['observation'])
        run.finish(status, error)
        return {'directory':run.directory, 'frame_count':len(frames)}
    except Exception as exc:
        print('CAR_NEAR archive_error ' + str(exc))
        return {'archive_error':str(exc)}
    finally:
        frames.clear()


def observe(config, projection, camera):
    from vision_runtime import track_coarse
    from blue_tracker import BlueTracker
    tracker = BlueTracker(config['tracking'], projection)
    for unused in range(config['alignment']['observation_frames']):
        os.exitpoint()
        image, frame = camera.capture()
        observation = track_coarse(image, frame, tracker.detect_frame)
        if observation['status'] == 'tracking':
            break
        time.sleep_ms(config['alignment']['frame_interval_ms'])
    return observation


def run_forever(root):
    from straight_voice import wait_for_wake
    from visual_align import align_once, approach_until_near
    from ground_projection import GroundProjection
    from motor_link import MotorLink
    from camera import CameraSession
    with open(root + '/config.json') as f:
        config = json.load(f)
    model = config['projection']
    projection = GroundProjection(model['K'], model['D'], model['T'], model['plane'])
    automatic = config.get('automatic_approach', False)
    continuous = config.get('continuous')
    parking = config.get('near_parking')
    print('CAR_ALIGN configured ' + json.dumps(dict(config['alignment'],
          forward=config.get('forward'), automatic_approach=automatic, continuous=continuous,
          near_parking=parking)))
    warmup = 32
    while True:
        # Release audio before CameraSession initializes the shared media pool.
        wait_for_wake(config['voice'], warmup)
        warmup = 0
        session_start = time.ticks_ms()
        def save_step(result, index, terminal):
            result.update(automatic_approach=True, approach_session_start_ms=session_start,
                          approach_step_index=index, approach_terminal_status=terminal)
            # Between stopped actions only; next action gets a new observation.
            with open('/sdcard/aic/car_align/runs.jsonl', 'a') as f:
                f.write(json.dumps(result) + '\n')
        link = MotorLink()
        camera = CameraSession(config['vision'])
        result, archive_error, fine_frames = None, None, []
        try:
            camera.open()
            if parking:
                from near_runtime import run_parking
                from vision_runtime import track_coarse
                from blue_tracker import BlueTracker
                tracker = BlueTracker(config['tracking'], projection)
                def capture_frame():
                    os.exitpoint()
                    return camera.capture()
                result = run_parking(capture_frame,
                    lambda image, frame: track_coarse(image, frame, tracker.detect_frame),
                    link, projection, config, time.ticks_ms, time.sleep_ms,
                    evidence=fine_frames)
            elif continuous:
                from continuous_approach import run_continuous
                from vision_runtime import track_coarse
                from blue_tracker import BlueTracker
                tracker = BlueTracker(config['tracking'], projection)
                def observe_frame():
                    os.exitpoint()
                    image, frame = camera.capture()
                    return track_coarse(image, frame, tracker.detect_frame)
                result = run_continuous(observe_frame, link, projection, config,
                                        time.ticks_ms, time.sleep_ms)
            elif automatic:
                result = approach_until_near(lambda: observe(config, projection, camera),
                                             link, projection, config, time.ticks_ms, save_step)
            else:
                result = align_once(lambda: observe(config, projection, camera), link,
                                    projection, config, time.ticks_ms)
        except Exception as exc:
            archive_error = str(exc)
            raise
        finally:
            try:
                camera.close()
            finally:
                try:
                    link.close()
                finally:
                    if parking:
                        archive = archive_fine_frames(config, fine_frames,
                            result['status'] if result else 'runtime_error', archive_error)
                        if result is not None and archive:
                            result['evidence'] = archive
        if parking:
            with open('/sdcard/aic/car_near/runs.jsonl', 'a') as f:
                f.write(json.dumps(result) + '\n')
        elif continuous:
            with open('/sdcard/aic/car_follow/runs.jsonl', 'a') as f:
                f.write(json.dumps(result) + '\n')
        elif not automatic:
            with open('/sdcard/aic/car_align/runs.jsonl', 'a') as f:
                f.write(json.dumps(result) + '\n')
        last = result['last'] if automatic else result
        print('CAR_ALIGN complete ' + json.dumps({
            'status': result['status'], 'plan': last.get('plan', last.get('last_plan')),
            'automatic_approach': automatic, 'action_count': result.get('action_count'),
            'nominal_angle_deg': last.get('motion', {}).get('nominal_angle_deg'),
            'nominal_mm': last.get('motion', {}).get('mean_nominal_mm'),
            'after_plan': last.get('after_plan')}))
        gc.collect()
