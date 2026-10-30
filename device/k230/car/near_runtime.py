"""One moving fine image, coarse steering with encoder-guided distance and at most one correction.

No arm actions or SD writes. Camera/UART lifetimes belong to the caller.
"""
import math
try:
    from .blue_tracker import age_ms
    from .visual_align import alignment_plan
    from .continuous_approach import midline_heading, settle
    from .continuous_math import wheel_speeds, predict_bearing, wheel_targets
    from .near_control import near_plan, verify_parked, correction_plan, distance_anchor
except ImportError:
    from blue_tracker import age_ms
    from visual_align import alignment_plan
    from continuous_approach import midline_heading, settle
    from continuous_math import wheel_speeds, predict_bearing, wheel_targets
    from near_control import near_plan, verify_parked, correction_plan, distance_anchor


def count_delta(a, b):
    return ((b-a+2147483648) % 4294967296)-2147483648


class Odometry:
    """Timestamp UART snapshots on the K230 clock; interpolate capture travel."""
    def __init__(self, mm_per_count):
        self.scale = mm_per_count
        self.rows = []

    def add(self, state, sent_at, received_at):
        # UART midpoint is approximate, not an exposure timestamp.
        import time
        add = getattr(time, 'ticks_add', lambda a, b: a+b)
        stamp = add(sent_at, age_ms(received_at, sent_at)//2)
        self.rows.append((stamp, state['encoder_a'], state['encoder_b']))

    def since(self, stamp, now):
        result = []
        for a, b in zip(self.rows, self.rows[1:]):
            duration = age_ms(b[0], a[0])
            if duration <= 0 or age_ms(b[0], stamp) <= 0:
                continue
            fraction = min(1., age_ms(b[0], stamp)/duration)
            result.append((count_delta(a[1], b[1])*self.scale*fraction,
                           -count_delta(a[2], b[2])*self.scale*fraction))
        if len(self.rows) >= 2:
            a, b = self.rows[-2:]
            duration = age_ms(b[0], a[0])
            tail = min(age_ms(now, b[0]), age_ms(now, stamp))
            if duration > 0 and tail > 0:
                result.append((count_delta(a[1], b[1])*self.scale*tail/duration,
                               -count_delta(a[2], b[2])*self.scale*tail/duration))
        return result


def coarse_drive_plan(observation, projection, config, now_ms, speeds):
    """Cruise with coarse heading updates; no old coarse distance endpoint."""
    raw = alignment_plan(observation, projection, config, now_ms)
    if raw['status'] == 'no_target':
        return {'status': 'no_target', 'mode': 'coarse'}
    point = observation.get('ground_contact_proxy_Base_mm', [])
    if len(point) != 3 or not all(isinstance(v, (int, float)) and math.isfinite(v) for v in point):
        raise ValueError('A valid coarse contact proxy is required')
    seconds = age_ms(now_ms, observation['capture_timestamp_ms'])/1000
    bearing = predict_bearing(raw['bearing_error_deg'], projection.table_distance(point),
        midline_heading(projection, observation['center_pixel'][1], config['tracking']['image_size_px'][0]),
        speeds['left_mm_s']*seconds, speeds['right_mm_s']*seconds, config['track_width_mm'])
    profile = config['continuous']
    # Fine computation itself never selects a slower cruise or an early stop.
    targets = wheel_targets(bearing, profile['cruise_mm_s']/profile['slowdown_gain_per_s'],
                            profile, config['track_width_mm'])
    return {'status': 'follow', 'mode': 'coarse', 'frame_id': observation['frame_id'],
            'image_age_ms': seconds*1000, 'image_error_px': raw['image_error_px'],
            'predicted_bearing_deg': bearing,
            'coarse_Base_X_mm': point[0],
            'left_mm_s': int(targets['left_mm_s']+.5),
            'right_mm_s': int(targets['right_mm_s']+.5)}


def _collect_evidence(session, evidence, label):
    take = getattr(session, 'take_evidence', None)
    if take:
        item = take(label)
        if item:
            evidence.append(item)


def _verify(capture, link, projection, config, clock_ms, fine_factory, stopped_at, state,
            evidence, label):
    image, frame = capture()
    # The shared precise detector finds its own current-frame proposals; no
    # coarse tracking state or proxy is involved in the final acceptance point.
    check = fine_factory(config, clock_ms)
    counts = state['encoder_a'], state['encoder_b']
    stationary = not state['run']
    try:
        check.start(image, frame, None)
        while check.phase:
            check.advance()
            state = link.expect('STATUS')
            stationary = stationary and not state['run'] and (state['encoder_a'], state['encoder_b']) == counts
    finally:
        _collect_evidence(check, evidence, label)
    verdict = verify_parked(check.latest, projection, config['workspace'],
        config['near_parking'], clock_ms(), stopped_at, check.latest_ready_ms,
        stationary_since_capture=stationary)
    return verdict, check.latest, check.events


def run_parking(capture, track, link, projection, config, clock_ms, sleep_ms, fine_factory=None,
                evidence=None, moving_fine_factory=None):
    """Continuous cruise -> one fine anchor -> encoder prediction -> static check.

    The trigger distance is a separately estimated coarse-proxy threshold.
    Original capture times stay intact. `t1_state` is read immediately
    after the trigger image; ordered odometry also accounts approximately for
    capture-to-UART latency. Base-to-axle lever-arm calibration remains absent.
    """
    result = {'schema': 'aic.car.near_parking.v2', 'status': 'no_target',
              'automatic_arm_action': False, 'motion_performed': False,
              'samples': [], 'fine_observations': [], 'verifications': []}
    if evidence is None:
        evidence = []
    distance = config['near_parking'].get('capture_Base_X_mm')
    if distance is None:
        result['status'] = 'capture_distance_unset'
        return result
    if isinstance(distance, bool) or not isinstance(distance, (int, float)) or not math.isfinite(distance):
        raise ValueError('capture_Base_X_mm must be a finite distance or null')
    if fine_factory is None:
        from physical_side_session import FineSession
        fine_factory = FineSession
        if moving_fine_factory is None:
            moving_fine_factory = lambda c, clock: FineSession(c, clock, purpose='car_moving_range')
    if moving_fine_factory is None:
        moving_fine_factory = fine_factory
    fine = moving_fine_factory(config, clock_ms)
    sync_at = clock_ms()
    previous = current = link.sync()
    odometry = Odometry(config['mm_per_count'])
    odometry.add(current, sync_at, clock_ms())
    sent = pending = fine_started = False
    attempts = 0
    anchor = None
    l1 = None

    def command(text):
        nonlocal previous, current
        started = clock_ms()
        reply = link.expect(text)
        odometry.add(reply, started, clock_ms())
        previous, current = current, reply
        return reply

    def speeds():
        return (wheel_speeds(previous, current, config['mm_per_count'])
                if previous['ms'] != current['ms'] else {'left_mm_s': 0, 'right_mm_s': 0})

    try:
        if current['run']:
            result['status'] = 'motor_busy'
            return result
        while True:
            started = clock_ms()
            start_fine = False
            coarse_heading = None
            if anchor is None:
                image, frame = capture()
                frame_state = command('STATUS')  # capture first, l1 immediately afterwards
                if sent and not frame_state['run']:
                    result['status'], result['motor_stop_reason'] = 'motion_stopped', frame_state['reason']
                    break
                coarse = track(image, frame)
                result['after'] = coarse
                plan = coarse_drive_plan(coarse, projection, config, clock_ms(), speeds())
                if not sent and plan['status'] == 'no_target':
                    attempts += 1
                    if attempts < config['alignment']['observation_frames']:
                        sleep_ms(config['alignment']['frame_interval_ms'])
                        continue
                if plan['status'] != 'follow':
                    result['status'], result['last_plan'] = plan['status'], plan
                    break
                if fine_started:
                    fine.advance()
                    # The coarse age is not renewed by an expensive fine step.
                    plan = coarse_drive_plan(coarse, projection, config, clock_ms(), speeds())
                    if plan['status'] != 'follow':
                        result['status'], result['last_plan'] = plan['status'], plan
                        break
                    if not fine.phase:
                        observation = fine.latest
                        result['fine_observations'].append(observation)
                        l2 = command('STATUS')
                        dl = count_delta(l1['encoder_a'], l2['encoder_a'])*config['mm_per_count']
                        dr = -count_delta(l1['encoder_b'], l2['encoder_b'])*config['mm_per_count']
                        result['encoder_handoff'].update(t2_state=l2, left_mm=dl, right_mm=dr,
                                                          mean_mm=(dl+dr)/2)
                        if not l2['run']:
                            result['status'], result['motor_stop_reason'] = 'motion_stopped', l2['reason']
                            break
                        try:
                            distance_anchor(observation)
                        except (ValueError, TypeError):
                            result['status'] = 'precise_unavailable'
                            break
                        anchor = observation
                elif plan['coarse_Base_X_mm'] <= distance:
                    start_fine = True
                    l1 = frame_state
                    result['encoder_handoff'] = {'capture_timestamp_ms': frame['capture_timestamp_ms'],
                        'frame_id': frame['frame_id'], 't1_state': l1,
                        'timestamp_basis': 'software capture and immediate UART snapshot; nominal interpolation'}
                if anchor is None and fine_started:
                    plan['mode'] = 'coarse_waiting_for_fine'
            else:
                # Fine sets distance once; fresh coarse images keep steering.
                # A missing coarse direction cannot discard the known distance.
                image, frame = capture()
                command('STATUS')
                if not current['run']:
                    result['status'], result['motor_stop_reason'] = 'motion_stopped', current['reason']
                    break
                coarse = track(image, frame)
                result['after'] = coarse
            if anchor is not None:
                heading = coarse_drive_plan(coarse, projection, config, clock_ms(), speeds())
                if heading['status'] == 'follow':
                    coarse_heading = heading['predicted_bearing_deg']
                plan = near_plan(anchor, projection, config['workspace'], config['near_parking'],
                    clock_ms(), wheel_deltas=odometry.since(anchor['capture_timestamp_ms'], clock_ms()),
                    speeds=speeds(), encoder_guided=True, steering_bearing_deg=coarse_heading)
                plan['mode'] = 'encoder_guided'
            result['last_plan'] = plan
            if plan['status'] != 'follow':
                result['status'] = plan['status']
                break
            if not sent:
                result['before'] = coarse
            name = 'SPEED' if sent else 'FOLLOW'
            sent = pending = True
            reply = command('%s,%d,%d' % (name, plan['left_mm_s'], plan['right_mm_s']))
            pending = False
            result['motion_performed'] = True
            if reply['result'] != 0 or not reply['run']:
                result['status'], result['motor_stop_reason'] = 'motion_stopped', reply['reason']
            elif start_fine:
                fine_started = True
                fine.start(image, frame, coarse['bbox_px'])
            elapsed = age_ms(clock_ms(), started)
            result['samples'].append({'plan': plan, 'motor': reply, 'cycle_ms': elapsed})
            if result['status'] == 'motion_stopped':
                break
            remaining = config['continuous']['period_ms']-elapsed
            if remaining > 0:
                sleep_ms(remaining)
    finally:
        try:
            if sent:
                result['stop_states'] = settle(link, current, config, sleep_ms, pending)
        finally:
            _collect_evidence(fine, evidence, 'moving_fine')
            result['fine_steps'] = fine.events
    if result['status'] != 'stop_for_verification':
        return result

    state = result['stop_states'][-1]
    for index in range(2):
        verdict, observation, events = _verify(capture, link, projection, config,
            clock_ms, fine_factory, clock_ms(), state, evidence,
            'corrected_verify' if index else 'stopped_verify')
        result['verification'], result['verification_observation'] = verdict, observation
        result['verification_steps'] = events
        result['verifications'].append({'verdict': verdict, 'observation': observation, 'steps': events})
        if verdict['accepted']:
            result['status'] = 'parked'
            return result
        if index:
            result['status'], result['failure_reason'] = 'parking_failed', verdict['status']
            return result
        correction = correction_plan(verdict)
        result['correction'] = correction
        if correction['status'] not in ('move', 'turn'):
            result['status'] = verdict['status']
            return result
        states = []
        pending = True
        try:
            state = link.expect(correction['command'])
            pending = False
            states.append(state)
            while state['run']:
                sleep_ms(config['heartbeat_interval_ms'])
                state = link.expect('HB')
                states.append(state)
        finally:
            result['correction_stop_states'] = settle(link, state, config, sleep_ms, pending)
        correction['states'] = states
        state = result['correction_stop_states'][-1]
        if states[-1]['reason'] != correction['expected_reason']:
            result['status'] = 'correction_motion_stopped'
            return result
    return result
