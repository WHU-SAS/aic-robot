"""Stop-observe-pulse approach decisions. No hardware, automatic arm action or PID."""
import math
try:
    from .blue_tracker import age_ms
    from .parking_window import evaluate, bounds_mm
    from .observations import for_approach
except ImportError:
    from blue_tracker import age_ms
    from parking_window import evaluate, bounds_mm
    from observations import for_approach


class ApproachPlanner:
    def __init__(self, workspace, config):
        self.workspace, self.config = workspace, config

    def _result(self, intent, reason, **details):
        return dict(intent=intent, reason=reason, automatic_arm_action=False, **details)

    def _fresh(self, observation, now_ms):
        if not isinstance(observation, dict):
            return False
        stamp = observation.get('timestamp_ms')
        return isinstance(stamp, (int, float)) and 0 <= age_ms(now_ms, stamp) <= self.config['tracking']['max_age_ms']

    def coarse(self, observation, now_ms, wheels_stopped=True):
        if not wheels_stopped:
            return self._result('wait_for_stop', 'one pulse at a time')
        if isinstance(observation, dict) and 'schema' in observation:
            try:
                observation = for_approach(observation, 'coarse')
            except ValueError as exc:
                return self._result('stop', str(exc))
        if not self._fresh(observation, now_ms) or observation.get('status') != 'tracking':
            return self._result('stop', 'no fresh confirmed coarse target')
        error = observation.get('image_midline_error_px')
        point = observation.get('ground_contact_proxy_Base_mm', [])
        if not isinstance(error, (int, float)) or not isinstance(point, (list, tuple)) or len(point) != 3 or not all(isinstance(v, (int, float)) and math.isfinite(v) for v in list(point) + [error]):
            return self._result('stop', 'invalid coarse geometry')
        if abs(error) > self.config['coarse_center_tolerance_px']:
            return self._result('turn_right_coarse' if error > 0 else 'turn_left_coarse',
                                'align blob with camera midline', image_error_px=error,
                                observation_timestamp_ms=observation['timestamp_ms'])
        if point[0] <= self.config['near_localization_start_Base_X_mm']:
            return self._result('localize', 'stopped near enough for existing precise vision')
        return self._result('forward_coarse', 'approach using a bounded pulse',
                            observation_timestamp_ms=observation['timestamp_ms'])

    def fine(self, observation, now_ms, wheels_stopped=True):
        if not wheels_stopped:
            return self._result('stop', 'precise localization requires stationary wheels')
        if isinstance(observation, dict) and 'schema' in observation:
            try:
                observation = for_approach(observation, 'precise')
            except ValueError as exc:
                return self._result('stop', str(exc))
        if not self._fresh(observation, now_ms) or observation.get('status') != 'localized':
            return self._result('stop', 'no fresh precise localization')
        point = observation.get('cube_center_Base_mm', [])
        try:
            if len(point) != 3 or not all(isinstance(v, (int, float)) and math.isfinite(v) for v in point):
                raise ValueError('Invalid cube center')
            result = evaluate(point[0], point[1], self.workspace)
        except (ValueError, TypeError):
            return self._result('stop', 'invalid precise geometry')
        near, far = bounds_mm(self.workspace, point[0])['Base_X_mm']
        fraction = self.config['preferred_stop_fraction_of_inset_depth']
        if not isinstance(fraction, (int, float)) or not 0 <= fraction <= 1:
            return self._result('stop', 'invalid preferred stopping depth')
        preferred_far = near + fraction * (far - near)
        if result['inside'] and point[0] <= preferred_far:
            return self._result('arrived', 'inside the user-approved inset window', window=result)
        if not result['lateral_allowed']:
            intent = 'turn_right_fine' if result['lateral_normalized'] > 0 else 'turn_left_fine'
        else:
            intent = 'reverse_fine' if point[0] < near else 'forward_fine'
        return self._result(intent, 'correct position with one small pulse, then observe again', window=result,
                            observation_timestamp_ms=observation['timestamp_ms'])


def pulse_for(plan, profiles):
    """Resolve a movement intent only after explicit, reviewed pulse parameters.

    Runtime defaults are null. Offline simulations may supply their own profiles,
    but must not write those assumed numbers back as real motor calibration.
    """
    intent = plan['intent']
    if intent in ('stop', 'wait_for_stop', 'localize', 'arrived'):
        return None
    profile = profiles.get(intent)
    if profile is None:
        raise ValueError('Motion parameters not selected for ' + intent)
    values = [profile[k] for k in ('left_pwm', 'right_pwm', 'duration_ms')]
    left, right, duration = values
    if any(type(v) is not int for v in values) or max(abs(left), abs(right)) > 1600 or not 10 <= duration <= 5000 or duration % 10:
        raise ValueError('Invalid explicit pulse parameters')
    signs = {'forward_coarse': (1, 1), 'forward_fine': (1, 1), 'reverse_fine': (-1, -1),
             'turn_left_coarse': (-1, 1), 'turn_left_fine': (-1, 1),
             'turn_right_coarse': (1, -1), 'turn_right_fine': (1, -1)}
    if intent not in signs or (left > 0) - (left < 0) != signs[intent][0] or (right > 0) - (right < 0) != signs[intent][1]:
        raise ValueError('Pulse signs disagree with the requested direction')
    return tuple(values)
