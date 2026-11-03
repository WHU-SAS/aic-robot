"""Continuous fine-position decisions, without camera, UART or arm actions.

Moving distance may use a measured-side range candidate; only complete precise
vision can confirm parking. A range candidate's raw XYZ is retained for motion
propagation and heading audit, not as a table-valid pose or arm target.
Encoder prediction assumes a fixed target and nominal differential-drive motion
about the declared origin (legacy Base zero): the axle lever arm is not calibrated. Neither
prediction nor the initial 0.12s coast model is a physical parking measurement.
"""
import math
try:
    from .blue_tracker import age_ms
    from .observations import for_approach
    from .parking_window import bounds_mm, evaluate
    from .continuous_math import wheel_targets, table_forward, forward_x_per_mm
    from ..pick.pose import undistort
except ImportError:
    from blue_tracker import age_ms
    from observations import for_approach
    from parking_window import bounds_mm, evaluate
    from continuous_math import wheel_targets, table_forward, forward_x_per_mm
    from pose import undistort


def predict_center(center_Base_mm, wheel_deltas, track_width_mm, table_normal,
                   forward_axis_Base=None, motion_origin_Base_mm=None):
    """Integrate ordered (left_mm, right_mm) travel from capture to now.

    The caller supplies increments on the K230 capture time line; this function
    never subtracts absolute timestamps from different boards. Rotation is about
    the table normal at the declared motion origin. Optional paired axes/origin
    preserve that nominal assumption after a rigid Base-coordinate change.
    """
    n = table_normal
    forward = table_forward(n, forward_axis_Base)
    left = [n[1]*forward[2]-n[2]*forward[1], n[2]*forward[0]-n[0]*forward[2],
            n[0]*forward[1]-n[1]*forward[0]]
    origin = [0., 0., 0.] if motion_origin_Base_mm is None else motion_origin_Base_mm
    point = [center_Base_mm[k]-origin[k] for k in range(3)]
    for dl, dr in wheel_deltas:
        distance, yaw = (dl+dr)/2, (dr-dl)/track_width_mm
        if abs(yaw) < 1e-8:
            dx, dy = distance, 0
        else:
            dx = distance*math.sin(yaw)/yaw
            dy = distance*(1-math.cos(yaw))/yaw
        point = [point[k]-dx*forward[k]-dy*left[k] for k in range(3)]
        c, s = math.cos(yaw), math.sin(yaw)
        cross = [n[1]*point[2]-n[2]*point[1], n[2]*point[0]-n[0]*point[2],
                 n[0]*point[1]-n[1]*point[0]]
        height = sum(a*b for a,b in zip(n, point))
        point = [c*point[k]-s*cross[k]+(1-c)*height*n[k] for k in range(3)]
    return [point[k]+origin[k] for k in range(3)]


def _precise(observation):
    if not isinstance(observation, dict):
        raise ValueError('No raw precise cube centre')
    result = for_approach(observation, 'precise')
    point = result.get('cube_center_Base_mm', [])
    if (result['status'] != 'localized' or len(point) != 3 or
            not all(isinstance(v, (int, float)) and math.isfinite(v) for v in point)):
        raise ValueError('No raw precise cube centre')
    if result['capture_timestamp_ms'] is None:
        raise ValueError('Precise frame capture time is required')
    return result


def distance_anchor(observation):
    """Read a moving-distance capability without upgrading it to a full pose."""
    if isinstance(observation, dict) and observation.get('status') == 'localized':
        result = _precise(observation)
        return result, result['cube_center_Base_mm'], 'precise'
    result = for_approach(observation, 'precise')
    candidate = result.get('range_candidate')
    if (not isinstance(candidate, dict) or candidate.get('schema') != 'aic.car.side_range.v1'
            or candidate.get('status') != 'range_candidate'
            or candidate.get('coordinate_frame') != 'Base' or candidate.get('units') != 'mm'
            or candidate.get('not_arm_or_parking_authority') is not True):
        raise ValueError('No physical-side distance candidate')
    point = candidate.get('raw_center_Base_mm')
    if (not isinstance(point, (list, tuple)) or len(point) != 3
            or not all(isinstance(v, (int, float)) and math.isfinite(v) for v in point)
            or candidate.get('Base_X_mm') != point[0]):
        raise ValueError('Invalid raw physical-side distance')
    if result['capture_timestamp_ms'] is None:
        raise ValueError('Distance frame capture time is required')
    return result, point, 'physical_side_range_candidate'


def _geometry(point, projection, workspace):
    pixel = projection.base_to_pixel(point)
    midline = workspace['centerline']['image_midline_x_px']
    ray = undistort([midline, pixel[1]], projection.K, projection.D) + [1.]
    reference = [sum(row[j]*ray[j] for j in range(3)) for row in projection.T[:3]]
    target = [point[k]-projection.origin[k] for k in range(3)]
    n = projection.normal
    directions = []
    for direction in (reference, target):
        component = sum(a*b for a,b in zip(direction,n))
        directions.append([direction[k]-component*n[k] for k in range(3)])
    a, b = directions
    cross = [a[1]*b[2]-a[2]*b[1], a[2]*b[0]-a[0]*b[2], a[0]*b[1]-a[1]*b[0]]
    bearing = math.degrees(math.atan2(sum(x*y for x,y in zip(cross,n)),
                                    sum(x*y for x,y in zip(a,b))))
    return {'image_error_px': pixel[0]-midline, 'bearing_deg': bearing,
            'window': evaluate(point[0], point[1], workspace)}


def target_x(workspace):
    """Shift the inset-window centre a quarter of its depth toward the arm."""
    near, far = bounds_mm(workspace)['Base_X_mm']
    # Smaller Base X brings the cube closer to the arm, so the car drives on.
    return near+(far-near)/4


def near_plan(observation, projection, workspace, config, now_ms,
              wheel_deltas=None, speeds=None, encoder_guided=False, steering_bearing_deg=None):
    """Predict an image-time centre, then steer or cut drive for verification.

    `wheel_deltas` contains actual nominal encoder travel since image capture;
    `speeds` is the latest nominal left_mm_s/right_mm_s for coast estimation.
    A stop request is never a successful parked verdict. Final verification uses
    a new static precise frame. An older absolute anchor is usable only when the
    caller explicitly supplies capture-to-now increments AND encoder_guided=True.
    A limited range candidate can supply moving distance, never parking success.
    After the single moving fine solve, this fixed-target encoder phase does not
    require coarse images for distance. A fresh coarse bearing may override
    steering only. Propagation is not a fresh precise image.
    """
    try:
        observation, raw, distance_source = distance_anchor(observation)
    except (ValueError, TypeError) as exc:
        return {'status': 'no_precise_target', 'reason': str(exc),
                'automatic_arm_action': False}
    age = age_ms(now_ms, observation['capture_timestamp_ms'])
    propagated = wheel_deltas is not None
    if age < 0 or (age > config['max_age_ms'] and
                   not (propagated and encoder_guided is True)):
        return {'status': 'no_precise_target', 'reason': 'Precise image expired',
                'capture_timestamp_ms': observation['capture_timestamp_ms'],
                'image_age_ms': age, 'automatic_arm_action': False}
    axis = getattr(projection, 'forward_axis_Base', None)
    origin = getattr(projection, 'motion_origin_Base_mm', None)
    point = predict_center(raw, wheel_deltas or (), config['track_width_mm'], projection.normal,
                           axis, origin)
    geometry = _geometry(point, projection, workspace)
    speeds = speeds or {'left_mm_s': 0, 'right_mm_s': 0}
    coast = predict_center(point,
        [(speeds['left_mm_s']*config['coast_s'], speeds['right_mm_s']*config['coast_s'])],
        config['track_width_mm'], projection.normal, axis, origin)
    stopped = _geometry(coast, projection, workspace)
    target = target_x(workspace)
    remaining = max(0, (point[0]-target)/forward_x_per_mm(projection))
    result = {'status': 'follow', 'frame_id': observation['frame_id'],
              'capture_timestamp_ms': observation['capture_timestamp_ms'],
              'image_age_ms': age, 'source_capture_age_ms': age,
              'distance_source': distance_source,
              'range_only': distance_source == 'physical_side_range_candidate',
              'odometry_predicted': propagated, 'raw_center_Base_mm': list(raw),
              'predicted_center_Base_mm': point, 'predicted_stop_center_Base_mm': coast,
              'target_Base_X_mm': target,
              'remaining_mm': remaining,
              'predicted_stop_window': stopped['window'],
              'prediction_basis': 'nominal encoder integration; uncalibrated axle lever arm',
              'automatic_arm_action': False}
    result.update(geometry)
    bearing = geometry['bearing_deg'] if steering_bearing_deg is None else steering_bearing_deg
    result.update(steering_bearing_deg=bearing,
                  heading_source='fine_odometry' if steering_bearing_deg is None else 'coarse')
    # Centring is a steering preference, never a reason to drive past the
    # longitudinal destination. Only the subsequent new static image accepts it.
    if coast[0] <= target:
        result['status'] = 'stop_for_verification'
    else:
        targets = wheel_targets(bearing, remaining,
                                config, config['track_width_mm'])
        result.update(left_mm_s=int(targets['left_mm_s']+.5),
                      right_mm_s=int(targets['right_mm_s']+.5))
    return result


def verify_parked(observation, projection, workspace, config, now_ms,
                  stopped_at_ms, result_ready_ms, stationary_since_capture=False):
    """Accept only a new precise frame taken after stopping, consumed promptly.

    The caller records `result_ready_ms` when processing actually finishes and
    attests that the car remained still throughout. A static solve may take more
    than 500ms; its original capture time is never refreshed or replaced. The
    ready-to-consumption age uses the existing max_age_ms, on the K230 clock.
    Only the approved inset window defines acceptance; image-midline centring
    is reported separately as a preferred result. An inside result is final.
    """
    try:
        observation = _precise(observation)
    except (ValueError, TypeError) as exc:
        return {'status': 'no_precise_target', 'accepted': False, 'preferred': False,
                'reason': str(exc), 'reason_code': 'precise_localization_failed',
                'automatic_arm_action': False}
    stamp = observation['capture_timestamp_ms']
    timing = {'stopped_at_ms': stopped_at_ms, 'capture_timestamp_ms': stamp,
              'result_ready_ms': result_ready_ms, 'checked_at_ms': now_ms}
    failure = None
    if stationary_since_capture is not True:
        failure = ('chassis_not_stationary', 'Chassis state or encoder counts changed during static localization')
    elif age_ms(stamp, stopped_at_ms) < 0:
        failure = ('capture_before_stop', 'Precise frame was captured before the stopped reference time')
    elif age_ms(result_ready_ms, stamp) < 0:
        failure = ('result_before_capture', 'Precise result completion precedes frame capture')
    elif age_ms(now_ms, result_ready_ms) < 0:
        failure = ('check_before_result', 'Static result was checked before its completion time')
    elif not 0 <= age_ms(now_ms, result_ready_ms) <= config['max_age_ms']:
        failure = ('static_result_expired', 'Static result exceeded the existing completion-to-use age limit')
    if failure:
        return {'status': 'no_precise_target', 'accepted': False, 'preferred': False,
                'reason_code': failure[0], 'reason': failure[1], 'static_timing': timing,
                'capture_timestamp_ms': stamp, 'automatic_arm_action': False}
    point = observation['cube_center_Base_mm']
    geometry = _geometry(point, projection, workspace)
    target = target_x(workspace)
    accepted = geometry['window']['inside']
    preferred = accepted and abs(geometry['image_error_px']) <= config['center_tolerance_px']
    result = {'status': 'parked' if accepted else 'not_parked', 'accepted': accepted,
            'preferred': preferred,
            'frame_id': observation['frame_id'], 'capture_timestamp_ms': stamp,
            'cube_center_Base_mm': list(point), 'target_Base_X_mm': target,
            'automatic_arm_action': False, 'forward_X_per_mm': forward_x_per_mm(projection),
            'static_timing': timing}
    result.update(geometry)
    return result


def correction_plan(verification, projection, workspace, config):
    """One constant-curvature wheel move toward the agreed depth and midline.

    Solve its nominal wheel distances on the declared table axes, then select
    the integer-millimetre pair closest to that SAME destination. Window margin
    is reported for review, never used as a replacement destination. Retain the
    existing 30-degree turn bound and the user's 100mm per-wheel short-step bound.
    Only the subsequent static image can confirm actual parking.
    """
    if verification.get('accepted'):
        return {'status': 'none'}
    if verification.get('status') != 'not_parked':
        return {'status': 'unavailable'}
    point = verification['cube_center_Base_mm']
    x = target_x(workspace)
    y = sum(bounds_mm(workspace, x)['Base_Y_mm'])/2
    n = projection.normal
    # Preserve this raw observation's table-normal height; it is not a new pose.
    z = (sum(a*b for a,b in zip(n, point))-n[0]*x-n[1]*y)/n[2]
    destination = [x, y, z]
    axis = getattr(projection, 'forward_axis_Base', None)
    origin = getattr(projection, 'motion_origin_Base_mm', None) or [0., 0., 0.]
    forward = table_forward(n, axis)
    left = [n[1]*forward[2]-n[2]*forward[1], n[2]*forward[0]-n[0]*forward[2],
            n[0]*forward[1]-n[1]*forward[0]]
    q, target = [[sum((p[k]-origin[k])*basis[k] for k in range(3))
                  for basis in (forward, left)] for p in (point, destination)]
    yaw = 2*math.atan2(q[1]-target[1], q[0]+target[0])
    yaw = (yaw+math.pi) % (2*math.pi)-math.pi
    yaw = max(-math.pi/6, min(math.pi/6, yaw))
    if abs(yaw) < 1e-8:
        distance = q[0]-target[0]
    else:
        c, s = math.cos(yaw), math.sin(yaw)
        a = [s/yaw, (c-1)/yaw]
        b = [c*q[0]+s*q[1]-target[0], -s*q[0]+c*q[1]-target[1]]
        distance = sum(u*v for u,v in zip(a,b))/sum(u*u for u in a)
    track = config['track_width_mm']
    limit = 100-track*abs(yaw)/2
    distance = max(-limit, min(limit, distance))
    dl, dr = distance-track*yaw/2, distance+track*yaw/2
    best = None
    for il in (math.floor(dl), math.ceil(dl)):
        for ir in (math.floor(dr), math.ceil(dr)):
            if not (il or ir) or max(abs(il), abs(ir)) > 100 or abs(ir-il) > track*math.pi/6:
                continue
            predicted = predict_center(point, [(il, ir)], track, n, axis, origin)
            bounds = bounds_mm(workspace, predicted[0])
            margin = min(predicted[0]-bounds['Base_X_mm'][0], bounds['Base_X_mm'][1]-predicted[0],
                         predicted[1]-bounds['Base_Y_mm'][0], bounds['Base_Y_mm'][1]-predicted[1])
            error = (predicted[0]-x)**2+(predicted[1]-y)**2
            rank = (error, -margin, abs(il)+abs(ir), il, ir)
            if best is None or rank < best[0]:
                best = (rank, il, ir, predicted, margin)
    rank, il, ir, predicted, margin = best
    return {'status': 'arc', 'left_mm': il, 'right_mm': ir,
            'command': 'ARC,%d,%d' % (il, ir), 'expected_reason': 8,
            'target_Base_X_mm': x, 'target_Base_Y_mm': y,
            'nominal_predicted_center_Base_mm': predicted,
            'nominal_predicted_window': evaluate(predicted[0], predicted[1], workspace),
            'nominal_window_margin_mm': margin, 'nominal_target_error_mm': math.sqrt(rank[0]),
            'nominal_yaw_deg': math.degrees((ir-il)/track),
            'prediction_basis': 'nominal signed-wheel arc to agreed destination; uncalibrated axle lever arm; final static verification required'}
