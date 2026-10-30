"""Camera-midline alignment and staged coarse approach."""
import math
try:
    from .blue_tracker import age_ms
    from .observations import for_approach
    from .straight_voice import run_motion
    from ..pick.pose import undistort
except ImportError:
    from blue_tracker import age_ms
    from observations import for_approach
    from straight_voice import run_motion
    from pose import undistort


def alignment_plan(observation, projection, config, now_ms):
    observation = for_approach(observation, 'coarse')
    stamp = observation['timestamp_ms']
    if (observation['status'] != 'tracking' or stamp is None or
            not 0 <= age_ms(now_ms, stamp) <= config['tracking']['max_age_ms']):
        return {'status': 'no_target', 'angle_deg': 0}
    u, v = observation['center_pixel']
    width, height = config['tracking']['image_size_px']
    if not all(math.isfinite(x) for x in (u, v)) or not (0 <= u < width and 0 <= v < height):
        raise ValueError('Invalid target center')
    error = u - width / 2
    # Compare two undistorted rays at the SAME image row. Project onto the
    # tabletop to account for camera tilt/roll. Reference is u=640, not K.cx
    # or Base Y=0; camera translation cancels in this heading estimate.
    rays = []
    for pixel in ([width / 2, v], [u, v]):
        ray = undistort(pixel, projection.K, projection.D) + [1.]
        direction = [sum(row[j] * ray[j] for j in range(3)) for row in projection.T[:3]]
        normal_part = sum(a * b for a, b in zip(direction, projection.normal))
        rays.append([direction[k] - normal_part * projection.normal[k] for k in range(3)])
    a, b = rays
    cross = [a[1]*b[2]-a[2]*b[1], a[2]*b[0]-a[0]*b[2], a[0]*b[1]-a[1]*b[0]]
    bearing = math.atan2(sum(x*y for x, y in zip(cross, projection.normal)),
                         sum(x*y for x, y in zip(a, b))) * 180 / math.pi
    aligned = abs(error) <= config['coarse_center_tolerance_px']
    gain = config['alignment']['turn_gain']
    angle = 0 if aligned else min(config['alignment']['max_turn_deg'], max(1, int(abs(bearing) * gain + .5)))
    if bearing < 0:
        angle = -angle
    return {'status': 'aligned' if aligned else 'turn', 'angle_deg': angle,
            'bearing_error_deg': bearing, 'image_error_px': error, 'turn_gain': gain,
            'frame_id': observation['frame_id'], 'capture_timestamp_ms': stamp}


def motion_plan(observation, projection, config, now_ms):
    plan = alignment_plan(observation, projection, config, now_ms)
    forward = config.get('forward')
    if forward is None or plan['status'] != 'aligned':
        return plan
    point = observation.get('ground_contact_proxy_Base_mm', [])
    if len(point) != 3 or not all(isinstance(v, (int, float)) and math.isfinite(v) for v in point):
        raise ValueError('Forward approach requires a valid coarse contact proxy')
    remaining = point[0] - forward['near_Base_X_mm']
    plan.update(coarse_forward_Base_X_mm=point[0], near_Base_X_mm=forward['near_Base_X_mm'])
    if remaining < forward['min_step_mm']:
        plan.update(status='near', distance_mm=0, precise_localization_required=True)
    else:
        plan.update(status='move', distance_mm=min(forward['max_step_mm'], int(remaining)))
    return plan


def align_once(observe, link, projection, config, clock_ms):
    before = observe()
    plan = motion_plan(before, projection, config, clock_ms())
    result = {'before': before, 'plan': plan, 'motion_performed': False,
              'automatic_arm_action': False}
    if plan['status'] not in ('turn', 'move'):
        result['status'] = plan['status']
        return result
    baseline = link.sync()
    # Preserve the existing freshness rule across a potentially blocking UART
    # reply. Passing this baseline avoids a second sync after the last check.
    plan = motion_plan(before, projection, config, clock_ms())
    result['plan'] = plan
    if plan['status'] not in ('turn', 'move'):
        result['status'] = plan['status']
        return result
    motion_config = dict(config, action=plan['status'])
    if plan['status'] == 'turn':
        motion_config['angle_deg'] = plan['angle_deg']
    else:
        motion_config['distance_mm'] = plan['distance_mm']
    result['motion'] = run_motion(link, motion_config, baseline=baseline)
    result['motion_performed'] = True
    result['after'] = observe()  # Fresh tracker; the old target is not reused.
    result['after_plan'] = motion_plan(result['after'], projection, config, clock_ms())
    after_status = result['after_plan']['status']
    if after_status == 'near':
        result['status'] = 'near'
    elif plan['status'] == 'move':
        result['status'] = 'move_complete'
    else:
        result['status'] = 'aligned' if after_status in ('aligned', 'move') else 'turn_complete'
    return result


def approach_until_near(observe, link, projection, config, clock_ms, on_step):
    """Repeat existing stopped actions; near is a handoff, not final parking.

    Store each finished step before the next fresh observation so logs do not
    accumulate in RAM. A non-distance/angle stop ends the voice task as well.
    """
    if not config.get('forward'):
        raise ValueError('Automatic approach requires forward settings')
    step_count = action_count = 0
    while True:
        result = align_once(observe, link, projection, config, clock_ms)
        step_count += 1
        terminal = None
        if not result['motion_performed']:
            terminal = result['plan']['status']
        else:
            action_count += 1
            motion = result['motion']
            expected_reason = 7 if motion['action'] == 'turn' else 6
            if motion['stop_reason'] != expected_reason:
                terminal = 'motion_stopped'
            elif result['after_plan']['status'] not in ('turn', 'move'):
                terminal = result['after_plan']['status']
        on_step(result, step_count, terminal)
        if terminal is not None:
            return {'status': terminal, 'step_count': step_count,
                    'action_count': action_count, 'last': result,
                    'automatic_arm_action': False}
