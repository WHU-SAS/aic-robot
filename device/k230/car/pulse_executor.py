"""One-pulse transport coordinator; caller owns UART, camera and the main loop.

Call poll at roughly 100ms while moving; do not run blocking fine vision then.
The class never initializes hardware, chooses PWM, starts a second pulse, or
invokes the arm. Unexpected stops require explicit release by the caller.
"""
try:
    from .approach import pulse_for
    from .blue_tracker import age_ms
except ImportError:
    from approach import pulse_for
    from blue_tracker import age_ms


class PulseExecutor:
    def __init__(self, link, profiles, max_observation_age_ms=500, clock_ms=None):
        self.link, self.profiles = link, profiles
        self.max_age = max_observation_age_ms
        self.state = 'unknown'
        self.halted = False
        self.last_observation = None
        self.expected_reason = None
        self.clock_ms = clock_ms
        self._stable_ms = None
        self._stable_counts = None

    @property
    def can_localize(self):
        return self.state == 'stopped' and not self.halted

    def synchronize(self):
        state = self.link.expect('STATUS')
        if state.get('protocol_version') != 2:
            raise RuntimeError('Pulse execution requires MSPM0 v2')
        if state['run'] or state['pwm']:
            self.state = 'unknown'
            raise RuntimeError('Motor already running; no new pulse sent')
        self.state = 'stopped'
        return state

    def submit(self, plan, now_ms):
        pulse = pulse_for(plan, self.profiles)
        if pulse is None:
            if self.state == 'moving' and plan['intent'] == 'stop':
                return self.cancel()
            return None
        if self.halted or self.state != 'stopped':
            raise RuntimeError('Pulse executor is not ready for a new motion')
        stamp = plan.get('observation_timestamp_ms')
        if type(stamp) not in (int, float) or not 0 <= age_ms(now_ms, stamp) <= self.max_age:
            raise ValueError('Pulse needs a fresh observation timestamp')
        if self.last_observation is not None and age_ms(stamp, self.last_observation) <= 0:
            raise ValueError('A new observation is required for each pulse')
        self.synchronize()
        # STATUS can block on UART. An injected monotonic clock lets the runtime
        # reject an observation that expired while waiting for that reply.
        current = self.clock_ms() if self.clock_ms is not None else now_ms
        if not 0 <= age_ms(current, stamp) <= self.max_age:
            raise ValueError('Observation expired while synchronizing motors')
        self.last_observation = stamp  # Consume even if the ACK is lost.
        self.expected_reason = 3 if pulse[2] == 5000 else 5
        try:
            result = self.link.drive(*pulse)
            if not result['run']:
                raise RuntimeError('DRIVE did not start')
            self.state = 'moving'
            return result
        except BaseException:
            self.halted = True
            self.cancel()
            raise

    def poll(self):
        if self.state not in ('moving', 'settling'):
            return None
        try:
            result = self.link.expect('STATUS' if self.state == 'settling' else 'HB')
        except BaseException:
            self.halted = True
            self.cancel()
            raise
        if not result['run']:
            completed = not result['pwm'] and result['reason'] == self.expected_reason
            if not completed:
                self.state = 'unknown'
                self.halted = True
            else:
                counts = (result['encoder_a'], result['encoder_b'])
                if self.state != 'settling' or counts != self._stable_counts:
                    self._stable_ms, self._stable_counts = result['ms'], counts
                self.state = 'settling'
                # Output-off is not a mechanical brake. Require unchanged
                # encoder readings for 200 ms before another image/pulse.
                if (result['ms'] - self._stable_ms) & 0xFFFFFFFF < 200:
                    return {'status': 'settling', 'state': result, 'automatic_arm_action': False}
                self.state = 'stopped'
            return {'status': 'pulse_complete' if completed else 'halted',
                    'state': result, 'new_observation_required': True,
                    'automatic_arm_action': False}
        if self.state == 'settling':
            self.halted = True
            self.cancel()
            raise RuntimeError('Motor restarted while waiting for stationary encoders')
        return {'status': 'moving', 'state': result, 'automatic_arm_action': False}

    def cancel(self):
        self.state = 'unknown'
        try:
            result = self.link.expect('STOP')
            confirmed = not result['run'] and not result['pwm']
            self.state = 'settling' if confirmed else 'unknown'
            if confirmed:
                self.expected_reason = 1
                self._stable_ms = result['ms']
                self._stable_counts = (result['encoder_a'], result['encoder_b'])
            if not confirmed: self.halted = True
            return {'stop_acknowledged': confirmed, 'state': result}
        except BaseException as exc:
            self.halted = True
            return {'stop_acknowledged': False, 'error': repr(exc)}

    def release_halt(self):
        """Explicit application/user decision; never called automatically."""
        self.synchronize()
        self.halted = False
