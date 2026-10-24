"""Stationary-car visual diagnostic. No chassis/arm imports or commands."""
import time


def run_live(tracking, model, duration_ms, interval_ms):
    import os
    import json
    from camera import CameraSession
    from vision_runtime import track_coarse
    with open('/sdcard/aic/pick/current.json') as f:
        release = json.load(f)['release']
    with open(release + '/config.json') as f:
        vision = json.load(f)['vision']
    c = vision['camera']
    if (c['width'], c['height']) != (1280, 960) or c['hmirror'] or c['vflip']:
        raise ValueError('Live tracker requires calibrated unmirrored 1280x960')
    projection = GroundProjection(model['K'], model['D'], model['T'], model['plane'])
    tracker = BlueTracker(tracking, projection)
    camera = CameraSession(vision, preview=True)
    frames = 0
    previous_stamp = None
    try:
        camera.open()
        started = time.ticks_ms()
        print('CAR_LIVE_READY')
        while time.ticks_diff(time.ticks_ms(), started) < duration_ms:
            os.exitpoint()
            image, frame = camera.capture()
            result = track_coarse(image, frame, tracker.detect_frame)
            frames += 1
            stamp = frame['capture_timestamp_ms']
            row = {key: result.get(key) for key in ('frame_id', 'capture_timestamp_ms',
                'clock_domain', 'status', 'bbox_px', 'center_pixel',
                'image_midline_error_px', 'pixels', 'confirmations', 'processing_ms')}
            row['frame_period_ms'] = time.ticks_diff(stamp, previous_stamp) if previous_stamp is not None else None
            previous_stamp = stamp
            row['estimated_table_distance_mm'] = None
            if result.get('bbox_px'):
                x, y, w, h = result['bbox_px']
                color = (0, 255, 0) if result['status'] == 'tracking' else (255, 180, 0)
                image.draw_rectangle((x, y, w, h), color=color, thickness=2)
                cx, cy = [int(v) for v in result['center_pixel']]
                image.draw_line(cx - 5, cy, cx + 5, cy, color=(255, 0, 0), thickness=2)
                image.draw_line(cx, cy - 5, cx, cy + 5, color=(255, 0, 0), thickness=2)
                try:
                    bottom = projection.estimated_bottom_from_image_center(result['center_pixel'])
                    row['estimated_table_distance_mm'] = projection.table_distance(bottom)
                except ValueError:
                    pass
            # Draw only after detecting on the unmarked frame. The native box
            # and cross therefore belong to this actual preview frame.
            camera.show(image)
            print('CAR_LIVE ' + json.dumps(row))
            remaining = interval_ms - time.ticks_diff(time.ticks_ms(), stamp)
            if remaining > 0:
                time.sleep_ms(remaining)
        print('CAR_LIVE_DONE ' + json.dumps({'frames': frames,
            'elapsed_ms': time.ticks_diff(time.ticks_ms(), started)}))
    finally:
        camera.close()
