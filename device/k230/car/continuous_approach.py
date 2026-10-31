"""Continuous coarse visual steering; hardware lifetime belongs to the caller."""
import math
try:
    from .blue_tracker import age_ms
    from .visual_align import alignment_plan
    from .continuous_math import wheel_speeds, predict_bearing, wheel_targets, table_forward, forward_x_per_mm
    from ..pick.pose import undistort
except ImportError:
    from blue_tracker import age_ms
    from visual_align import alignment_plan
    from continuous_math import wheel_speeds, predict_bearing, wheel_targets, table_forward, forward_x_per_mm
    from pose import undistort


def midline_heading(projection, v, width):
    """Planar u=width/2 direction relative to the declared forward axis."""
    ray = undistort([width/2, v], projection.K, projection.D) + [1.]
    direction = [sum(row[j]*ray[j] for j in range(3)) for row in projection.T[:3]]
    n = projection.normal
    forward = table_forward(n, getattr(projection, 'forward_axis_Base', None))
    left = [n[1]*forward[2]-n[2]*forward[1], n[2]*forward[0]-n[0]*forward[2],
            n[0]*forward[1]-n[1]*forward[0]]
    return math.degrees(math.atan2(sum(a*b for a,b in zip(direction,left)),
                                   sum(a*b for a,b in zip(direction,forward))))


def continuous_plan(observation, projection, config, now_ms, speeds):
    raw = alignment_plan(observation, projection, config, now_ms)
    if raw['status'] == 'no_target':
        return {'status': 'no_target'}
    point = observation.get('ground_contact_proxy_Base_mm', [])
    if len(point) != 3 or not all(isinstance(x, (int, float)) and math.isfinite(x) for x in point):
        raise ValueError('Continuous approach requires a valid coarse contact proxy')
    seconds = age_ms(now_ms, observation['capture_timestamp_ms']) / 1000
    dl, dr = speeds['left_mm_s']*seconds, speeds['right_mm_s']*seconds
    travel = (dl+dr)/2
    bearing = predict_bearing(raw['bearing_error_deg'], projection.table_distance(point),
        midline_heading(projection, observation['center_pixel'][1], config['tracking']['image_size_px'][0]),
        dl, dr, config['track_width_mm'])
    remaining = (point[0] - config['forward']['near_Base_X_mm']) / forward_x_per_mm(projection) - travel
    result = {'status': 'follow', 'frame_id': observation['frame_id'],
              'image_error_px': raw['image_error_px'], 'raw_bearing_deg': raw['bearing_error_deg'],
              'predicted_bearing_deg': bearing, 'image_age_ms': seconds*1000,
              'predicted_travel_mm': travel, 'remaining_mm': remaining,
              'prediction_basis': 'short-time nominal wheel motion; uncalibrated axle lever arm'}
    if remaining < config['forward']['min_step_mm']:
        result['centered'] = abs(raw['image_error_px']) <= config['coarse_center_tolerance_px']
        result['status'] = 'near' if result['centered'] else 'near_unaligned'
    else:
        targets = wheel_targets(bearing, remaining, config['continuous'], config['track_width_mm'])
        result.update(left_mm_s=int(targets['left_mm_s']+.5), right_mm_s=int(targets['right_mm_s']+.5))
    return result


def settle(link, state, config, sleep_ms, pending=False):
    if pending or state['run']:
        state = link.expect('STOP')
    states = [state]
    stable_at, counts = state['ms'], (state['encoder_a'], state['encoder_b'])
    while ((states[-1]['ms']-stable_at) & 0xFFFFFFFF) < config['stopped_observation_ms']:
        sleep_ms(50)
        state = link.expect('STATUS')
        new_counts = state['encoder_a'], state['encoder_b']
        if new_counts != counts:
            stable_at, counts = state['ms'], new_counts
        states.append(state)
    return states


def run_continuous(observe, link, projection, config, clock_ms, sleep_ms):
    previous = current = link.sync()
    result = {'schema': 'aic.car.continuous_approach.v1', 'status': 'no_target',
              'automatic_arm_action': False, 'motion_performed': False, 'samples': [],
              'period_ms': config['continuous']['period_ms'], 'physical_distance_measured': False}
    sent = pending = False
    attempts = 0
    try:
        if current['run']:
            result['status'] = 'motor_busy'
            return result
        while True:
            started = clock_ms()
            observation = observe()
            speeds = (wheel_speeds(previous, current, config['mm_per_count'])
                      if previous['ms'] != current['ms'] else {'left_mm_s': 0, 'right_mm_s': 0})
            plan = continuous_plan(observation, projection, config, clock_ms(), speeds)
            result['after'], result['last_plan'] = observation, plan
            if not sent and plan['status'] == 'no_target':
                attempts += 1
                if attempts < config['alignment']['observation_frames']:
                    sleep_ms(config['alignment']['frame_interval_ms'])
                    continue
            if plan['status'] != 'follow':
                result['status'] = plan['status']
                break
            if not sent:
                result['before'] = observation
            command = 'SPEED' if sent else 'FOLLOW'
            pending = sent = True
            reply = link.request('%s,%d,%d' % (command, plan['left_mm_s'], plan['right_mm_s']))
            pending = False
            previous, current = current, reply
            result['motion_performed'] = True
            elapsed = age_ms(clock_ms(), started)
            result['samples'].append({'plan': plan, 'motor': reply, 'cycle_ms': elapsed})
            if reply['result'] != 0 or not reply['run']:
                result['status'], result['motor_stop_reason'] = 'motion_stopped', reply['reason']
                break
            remaining = config['continuous']['period_ms'] - elapsed
            if remaining > 0:
                sleep_ms(remaining)
    finally:
        if sent:
            result['stop_states'] = settle(link, current, config, sleep_ms, pending)
    return result
