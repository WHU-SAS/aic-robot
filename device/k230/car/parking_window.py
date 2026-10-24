"""User-selected linear parking window; pure math, no hardware access.

Input is the raw cube center in mechanical-arm Base millimetres, before any
grasp compensation. A calibrated local line maps the image midline into Base.
"""
import math


def _number(value):
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ValueError('Expected a finite number')
    if not math.isfinite(value):
        raise ValueError('Expected a finite number')
    return value


def _parameters(config):
    window = config['normalized_window']
    forward, lateral = window['longitudinal'], window['lateral']
    near, far = [_number(forward[k]) for k in ('near_Base_X_mm', 'far_Base_X_mm')]
    left, right = [_number(lateral[k]) for k in ('left_Base_Y_mm', 'right_Base_Y_mm')]
    fmin, fmax = [_number(v) for v in forward['allowed']]
    lmin, lmax = [_number(v) for v in lateral['allowed']]
    inset = _number(window.get('inset_mm', 0))
    if far <= near or left == right or fmin > fmax or lmin > lmax or inset < 0:
        raise ValueError('Invalid parking window reference or interval')
    return near, far, left, right, fmin, fmax, lmin, lmax, inset


def _bounds(parameters):
    near, far, left, right, fmin, fmax, lmin, lmax, inset = parameters
    xs = [near + v * (far - near) for v in (fmin, fmax)]
    ys = [left + (v + 1) * (right - left) / 2 for v in (lmin, lmax)]
    result = {'Base_X_mm': [min(xs) + inset, max(xs) - inset],
              'Base_Y_mm': [min(ys) + inset, max(ys) - inset]}
    if any(low > high for low, high in result.values()):
        raise ValueError('Parking window is empty after inset')
    return result


def _center_shift(config, x_mm, parameters):
    centerline = config.get('centerline', {})
    if not centerline.get('image_midline_is_final_centering_reference', False):
        return 0
    line = centerline['local_Base_Y_line']
    center = _number(line['slope_per_X']) * x_mm + _number(line['intercept_mm'])
    return center - (parameters[2] + parameters[3]) / 2


def bounds_mm(config, x_mm=None):
    """Return X limits and Y limits at X (at mid-depth when X is omitted).

    Camera-centered side limits move with depth. Use polygon_mm for all four
    vertices rather than treating the default Y interval as a fixed rectangle.
    """
    parameters = _parameters(config)
    limits = _bounds(parameters)
    if x_mm is None:
        x_mm = sum(limits['Base_X_mm']) / 2
    shift = _center_shift(config, _number(x_mm), parameters)
    limits['Base_Y_mm'] = [v + shift for v in limits['Base_Y_mm']]
    return limits


def polygon_mm(config):
    """Return near-left, near-right, far-right, far-left Base XY vertices."""
    x0, x1 = bounds_mm(config)['Base_X_mm']
    near = bounds_mm(config, x0)['Base_Y_mm']
    far = bounds_mm(config, x1)['Base_Y_mm']
    return [[x0, near[1]], [x0, near[0]], [x1, far[0]], [x1, far[1]]]


def evaluate(x_mm, y_mm, config):
    """Classify one valid vision result without clipping out-of-window values.

    Membership is compared in Base millimetres to keep exact interval endpoints
    inclusive despite floating-point rounding during normalization. This is a
    position test only; it does not confirm the car has stopped or trigger arm
    motion. Missing or failed visual localization must not supply a stale point.
    """
    x_mm, y_mm = _number(x_mm), _number(y_mm)
    parameters = _parameters(config)
    near, far, left, right = parameters[:4]
    limits = bounds_mm(config, x_mm)
    shift = _center_shift(config, x_mm, parameters)
    forward_ok = limits['Base_X_mm'][0] <= x_mm <= limits['Base_X_mm'][1]
    lateral_ok = limits['Base_Y_mm'][0] <= y_mm <= limits['Base_Y_mm'][1]
    return {
        'longitudinal_normalized': (x_mm - near) / (far - near),
        'lateral_normalized': 2 * (y_mm - shift - left) / (right - left) - 1,
        'longitudinal_allowed': forward_ok,
        'lateral_allowed': lateral_ok,
        'inside': forward_ok and lateral_ok,
        'Base_Y_mm': y_mm,
        'centerline_error_mm': y_mm - ((left + right) / 2 + shift)
            if config.get('centerline', {}).get('image_midline_is_final_centering_reference', False)
            else y_mm,
    }
