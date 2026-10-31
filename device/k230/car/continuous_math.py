"""Small differential-drive calculations for continuous coarse approach.

Wheel distances and yaw are nominal encoder estimates. Prediction treats the
camera/table-foot origin as following the axle centre: its uncalibrated lever
arm is omitted. It is a short-time approximation, not precise physical pose.
The caller measures image age on the K230 clock, separately from MCU elapsed
time, and keeps the centroid-to-image-midline bearing as the angular reference.
"""
import math


def table_forward(table_normal, forward_axis_Base=None):
    """Declared forward axis projected onto the table, in this Base frame.

    This preserves the nominal motion model; it is not a measured chassis axis.
    """
    axis = [1., 0., 0.] if forward_axis_Base is None else forward_axis_Base
    component = sum(a*b for a, b in zip(axis, table_normal))
    forward = [axis[k]-component*table_normal[k] for k in range(3)]
    length = math.sqrt(sum(v*v for v in forward))
    return [v/length for v in forward]


def forward_x_per_mm(projection):
    """Paired axes convert nominal wheel travel to Base X displacement.

    Omitted axes retain the legacy X-difference-as-distance approximation.
    """
    axis = getattr(projection, 'forward_axis_Base', None)
    return 1. if axis is None else table_forward(projection.normal, axis)[0]


def wheel_speeds(previous_state, current_state, mm_per_count):
    """Nominal mm/s from two MCU replies; callers supply distinct MCU times.

    Only MCU `ms` differences are used. Raw channel A is left-forward positive,
    B is right-forward negative; signed encoder and unsigned ms wraps are kept.
    """
    seconds = ((current_state['ms'] - previous_state['ms']) & 0xFFFFFFFF) / 1000
    def distance(key):
        counts = ((current_state[key] - previous_state[key] + 2147483648)
                  % 4294967296) - 2147483648
        return counts * mm_per_count
    return {'left_mm_s': distance('encoder_a') / seconds,
            'right_mm_s': -distance('encoder_b') / seconds}


def predict_bearing(bearing_deg, range_mm, midline_heading_deg,
                    delta_left_mm, delta_right_mm, track_width_mm):
    """Predict the same centroid bearing after short nominal wheel travel.

    Positive bearings/yaw are left. `range_mm` is the coarse contact proxy's
    planar range, used only as an approximate radius for the centroid direction.
    The image-midline ray may have a nonzero heading relative to declared forward.
    Exposure/buffer age and camera-to-axle lever arm are not calibrated here.
    """
    heading = math.radians(midline_heading_deg)
    target_angle = heading + math.radians(bearing_deg)
    x, y = range_mm * math.cos(target_angle), range_mm * math.sin(target_angle)
    travel = (delta_left_mm + delta_right_mm) / 2
    yaw = (delta_right_mm - delta_left_mm) / track_width_mm
    if abs(yaw) < 1e-8:
        dx, dy = travel, 0
    else:
        dx = travel * math.sin(yaw) / yaw
        dy = travel * (1 - math.cos(yaw)) / yaw
    c, s = math.cos(yaw), math.sin(yaw)
    x, y = x - dx, y - dy
    predicted = math.atan2(-s*x + c*y, c*x + s*y) - heading
    return (math.degrees(predicted) + 180) % 360 - 180


def wheel_targets(bearing_deg, remaining_mm, config, track_width_mm):
    """Forward-only mm/s wheel targets; the caller decides the final stop.

    Config keys: cruise_mm_s, finish_mm_s, max_wheel_mm_s,
    heading_gain_per_s, slowdown_gain_per_s. No discrete MOVE/TURN switching.
    Returned base speed and omega describe the limited targets, not measurements.
    """
    angle = math.radians(bearing_deg)
    distance_speed = min(config['cruise_mm_s'], max(config['finish_mm_s'],
                        config['slowdown_gain_per_s'] * remaining_mm))
    base = max(config['finish_mm_s'], distance_speed * max(0, math.cos(angle)))
    difference = config['heading_gain_per_s'] * angle * track_width_mm / 2
    difference = max(-base, min(base, difference))
    left, right = base - difference, base + difference
    scale = min(1, config['max_wheel_mm_s'] / max(left, right))
    left, right = left * scale, right * scale
    return {'left_mm_s': left, 'right_mm_s': right,
            'base_speed_mm_s': (left + right) / 2,
            'omega_rad_s': (right - left) / track_width_mm}
