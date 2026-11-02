"""Cooperative arm execution for tracking tasks; no camera or chassis ownership.

Constructing is inert. The owner calls step with its latest target version and
may observe between calls. Cancelling unsent stages is not a physical stop:
recovery explicitly overwrites their goals with the approved ready pose2.
"""

_UNSET_TARGET_VERSION = object()


class PickExecution:
    def __init__(self, plan, policy, io, target_version, clock_ms,
                 ticks_diff=None, emit=None):
        if type(target_version) not in (int, str):
            raise ValueError('A task-owned integer or string target_version is required')
        self.plan = dict(plan)
        for name in ('hover', 'descend', 'close', 'lift'):
            goals = plan.get(name)
            if (not isinstance(goals, (list, tuple)) or len(goals) != 6 or
                    any(type(v) is not int or not 0 <= v <= 4095 for v in goals)):
                raise ValueError('Six encoder goals required for ' + name)
            self.plan[name] = list(goals)
        self.policy = dict(policy)
        defaults = {'hover_tolerance_counts': 30, 'descend_tolerance_counts': 10,
                    'descend_axis4_lead_ms': 200, 'gripper_stable_samples': 5,
                    'gripper_stable_tolerance_counts': 1, 'poll_interval_ms': 50,
                    'grip_confirmation_delta_counts': 30, 'lift_tolerance_counts': 10,
                    'lift_gripper_stable_tolerance_counts': 1,
                    'lift_gripper_stable_ms': 200, 'recovery_observation_delay_ms': 500}
        for key, default in defaults.items():
            value = self.policy.setdefault(key, default)
            if type(value) is not int or value < 0:
                raise ValueError('Invalid stage policy: ' + key)
        if self.policy['gripper_stable_samples'] < 1 or self.policy['poll_interval_ms'] < 1:
            raise ValueError('Positive sample count and poll interval required')
        self.io = io
        self.target_version = target_version
        self.clock_ms = clock_ms
        self.ticks_diff = ticks_diff or (lambda a, b: a - b)
        self.emit = emit or (lambda event, details: None)
        self.status = 'running'
        self.phase = 'hover'
        self.pending_stage = 'hover'
        self.last_positions = [None] * 6
        self.last_sample = None
        self.feedback_status = 'unknown'
        self.error = None
        self.grip_confirmation = None
        self.grasp_confirmed = None
        self._success_ready = False
        self.recovery_reason = None
        self.recovery_goal_sent_ms = None
        self._invalidated = False
        self._last_poll_ms = None
        self._lead_started_ms = None
        self._close_window = []
        self._lift_reference = None
        self._lift_stable_since = None
        self._lift_arrived = False
        self._lift_last_sample_ms = None

    def _event(self, event, details=None):
        payload = {'target_version': self.target_version, 'phase': self.phase}
        if details:
            payload.update(details)
        self.emit(event, payload)

    def invalidate_target(self, reason='target_moved'):
        """Revoke unsent stages now; next step sends ready2, even during wrist lead."""
        if self.status not in ('running',):
            return self.snapshot()
        self._invalidated = True
        self._success_ready = False
        self.recovery_reason = reason
        self.pending_stage = None
        self._close_window = []
        self._lift_stable_since = None
        return self.snapshot()

    def _recover(self, reason):
        # Latch before any IO: a failed write must never resurrect an old plan.
        self.pending_stage = None
        self._invalidated = True
        self._success_ready = False
        self._close_window = []
        self._lift_stable_since = None
        self.recovery_reason = reason
        self._lift_last_sample_ms = None
        self.phase = 'recovery'
        self.status = 'recovering'
        # Ready2 opens the jaw; pre-recovery grip evidence is historical only.
        self.grasp_confirmed = None
        self.feedback_status = 'unknown'
        result = self.io.send_ready2()
        self.recovery_goal_sent_ms = (result or {}).get('goal_sent_ticks_ms')
        if self.recovery_goal_sent_ms is None:
            self.recovery_goal_sent_ms = self.clock_ms()
        self._event('recovery_goal_sent', {'reason': reason,
                    'goal_sent_ticks_ms': self.recovery_goal_sent_ms,
                    'observation_delay_ms': self.policy['recovery_observation_delay_ms'],
                    'arrival_checked': False, 'gripper_opens': True})

    def _refresh_recovery(self, now):
        if (self.status == 'recovering' and self.recovery_goal_sent_ms is not None and
                self.ticks_diff(now, self.recovery_goal_sent_ms) >=
                self.policy['recovery_observation_delay_ms']):
            self.status = 'reobserve_ready'

    def _poll_due(self, now):
        return (self._last_poll_ms is None or
                self.ticks_diff(now, self._last_poll_ms) >= self.policy['poll_interval_ms'])

    def _read(self, ids):
        read_started_ms = self.clock_ms()
        try:
            values = self.io.read_positions(ids)
            if len(values) != len(ids) or any(type(v) is not int for v in values):
                raise ValueError('Incomplete arm feedback round')
        except Exception as exc:
            self._last_poll_ms = self.clock_ms()
            self.feedback_status = 'unknown'
            self.grasp_confirmed = None
            self._success_ready = False
            self.error = str(exc)
            self._close_window = []
            self._lift_stable_since = None
            self._lift_reference = None
            self._lift_arrived = False
            self._lift_last_sample_ms = None
            self._event('feedback_unavailable', {'error': self.error, 'axes': list(ids),
                        'read_started_ms': read_started_ms,
                        'read_ms': self.ticks_diff(self._last_poll_ms, read_started_ms)})
            return None
        now = self.clock_ms()
        self._last_poll_ms = now
        self.feedback_status = 'valid'
        self.error = None
        for axis, value in zip(ids, values):
            self.last_positions[axis - 1] = value
        self.last_sample = {'axes': list(ids), 'positions': list(values),
                            'timestamp_ms': now, 'read_started_ms': read_started_ms,
                            'read_ms': self.ticks_diff(now, read_started_ms)}
        return values

    def _confirmation(self, position):
        goal = self.plan['close'][5]
        delta = position - goal
        return {'axis': 6, 'goal': goal, 'position': position, 'delta_counts': delta,
                'threshold_counts': self.policy['grip_confirmation_delta_counts'],
                'grasp_confirmed': delta >= self.policy['grip_confirmation_delta_counts'],
                'method': 'axis6_stable_goal_delta', 'position_reference': 'last_stable_sample',
                'phase': self.phase, 'sample_time_ms': self._last_poll_ms}

    def _start_stage(self, name):
        self.pending_stage = None
        self.phase = name
        self._success_ready = False
        self._last_poll_ms = None
        if name == 'descend':
            self.io.send_goals(self.plan[name], [4])
            self._lead_started_ms = self.clock_ms()
            self.phase = 'descend_lead'
            self._event('stage_goal_sent', {'name': name, 'axes': [4],
                        'goals': list(self.plan[name])})
        else:
            self.io.send_goals(self.plan[name], [1, 2, 3, 4, 5, 6])
            if name == 'close':
                self._close_window = []
            if name == 'lift':
                self._lift_stable_since = None
                self._lift_arrived = False
                self._lift_last_sample_ms = None
                self._lift_reference = self.last_positions[5]
            self._event('stage_goal_sent', {'name': name, 'axes': [1, 2, 3, 4, 5, 6],
                        'goals': list(self.plan[name])})

    def _arrival(self, positions):
        errors = [actual - goal for actual, goal in zip(positions, self.plan[self.phase])]
        tolerance = self.policy[self.phase + '_tolerance_counts']
        if all(abs(error) <= tolerance for error in errors):
            self._event('position_gate_passed', {'name': self.phase, 'positions': positions,
                        'errors_actual_minus_goal': errors, 'tolerance_counts': tolerance})
            self._event('stage_complete', {'name': self.phase})
            self.pending_stage = 'descend' if self.phase == 'hover' else 'close'

    def _close(self, position):
        self._close_window.append(position)
        required = self.policy['gripper_stable_samples']
        if len(self._close_window) > required:
            self._close_window.pop(0)
        if (len(self._close_window) == required and
                max(self._close_window) - min(self._close_window) <=
                2 * self.policy['gripper_stable_tolerance_counts']):
            self.grip_confirmation = self._confirmation(position)
            self.grasp_confirmed = self.grip_confirmation['grasp_confirmed']
            self._event('gripper_settled', {'positions': list(self._close_window)})
            self._event('grip_confirmation', {'details': dict(self.grip_confirmation)})
            if not self.grasp_confirmed:
                self._recover('empty_grip')
            else:
                self._event('stage_complete', {'name': 'close'})
                self.pending_stage = 'lift'

    def _lift(self, positions, allow_success=True):
        self._success_ready = False
        position = positions[5]
        self.grip_confirmation = self._confirmation(position)
        self.grip_confirmation['method'] = 'axis6_lift_goal_delta'
        self.grip_confirmation['position_reference'] = 'latest_lift_sample'
        self.grasp_confirmed = self.grip_confirmation['grasp_confirmed']
        if not self.grasp_confirmed:
            self._event('grip_confirmation', {'details': dict(self.grip_confirmation)})
            self._recover('grip_lost')
            return
        now = self.last_sample['timestamp_ms']
        # Distinguish scheduler idle from the UART round itself. Six normal
        # checked reads can cost180ms; that is not a missed scheduling turn.
        # Keep real sample spacing visible; do not claim50ms axis6 sampling.
        if self._lift_last_sample_ms is not None:
            idle_gap_ms = self.ticks_diff(self.last_sample['read_started_ms'],
                                          self._lift_last_sample_ms)
            gap_limit_ms = 2 * self.policy['poll_interval_ms']
            if idle_gap_ms > gap_limit_ms:
                self._lift_stable_since = None
                self._lift_reference = position
                self._event('lift_observation_gap', {'idle_gap_ms': idle_gap_ms,
                            'sample_gap_ms': self.ticks_diff(now, self._lift_last_sample_ms),
                            'read_ms': self.last_sample['read_ms'],
                            'cadence_limit_ms': gap_limit_ms})
        self._lift_last_sample_ms = now
        errors = [positions[i] - self.plan['lift'][i] for i in range(5)]
        arrived = all(abs(error) <= self.policy['lift_tolerance_counts'] for error in errors)
        tolerance = self.policy['lift_gripper_stable_tolerance_counts']
        out_of_band = (self._lift_reference is None or
                       abs(position - self._lift_reference) > tolerance)
        if out_of_band:
            self._lift_reference = position
            self._lift_stable_since = None
            self._event('lift_gripper_band_reset', {'position': position,
                        'tolerance_counts': tolerance})
        if not arrived:
            self._lift_stable_since = None
            self._lift_arrived = False
            return
        if not self._lift_arrived:
            self._event('position_gate_passed', {'name': 'lift', 'axes': [1, 2, 3, 4, 5],
                        'errors_actual_minus_goal': errors,
                        'tolerance_counts': self.policy['lift_tolerance_counts']})
        self._lift_arrived = True
        if self._lift_stable_since is None:
            self._lift_stable_since = now
            self._lift_reference = position
        stable_ms = self.ticks_diff(now, self._lift_stable_since)
        if stable_ms >= self.policy['lift_gripper_stable_ms']:
            self._success_ready = True
            if not allow_success:
                return
            self.status = 'succeeded'
            self.phase = 'complete'
            self.grip_confirmation.update(stable_reference=self._lift_reference,
                stable_tolerance_counts=tolerance, stable_ms=stable_ms)
            self._event('stage_complete', {'name': 'lift'})
            self._event('pick_succeeded', {'grip_confirmation': dict(self.grip_confirmation)})

    def step(self, target_version=_UNSET_TARGET_VERSION, allow_success=True):
        """At most one dispatch/feedback turn; empty/lost grip dispatches recovery now.

        allow_success=False defers only final commit for pending task evidence;
        feedback and empty-grip recovery continue without changing thresholds.
        Failure is a reported arm fault for the task owner, not a new task-level
        give-up rule. No retries, camera actions, PID or torque-OFF writes occur.
        """
        now = self.clock_ms()
        self._refresh_recovery(now)
        if self.status != 'running':
            return self.snapshot()
        if target_version is not _UNSET_TARGET_VERSION and target_version != self.target_version:
            self.invalidate_target('target_version_changed')
        try:
            if self._invalidated:
                self._recover(self.recovery_reason)
            elif self.pending_stage is not None:
                self._start_stage(self.pending_stage)
            elif self.phase == 'descend_lead':
                lead_ms = self.ticks_diff(now, self._lead_started_ms)
                if lead_ms >= self.policy['descend_axis4_lead_ms']:
                    self.io.send_goals(self.plan['descend'], [1, 2, 3, 5, 6])
                    self.phase = 'descend'
                    self._last_poll_ms = None
                    self._event('stage_goal_sent', {'name': 'descend', 'axes': [1, 2, 3, 5, 6],
                                'goals': list(self.plan['descend']), 'axis4_lead_ms': lead_ms})
            elif self._poll_due(now):
                ids = [6] if self.phase == 'close' else [1, 2, 3, 4, 5, 6]
                positions = self._read(ids)
                if positions is not None:
                    if self.phase in ('hover', 'descend'):
                        self._arrival(positions)
                    elif self.phase == 'close':
                        self._close(positions[0])
                    elif self.phase == 'lift':
                        self._lift(positions, allow_success=allow_success)
        except Exception as exc:
            self.pending_stage = None
            self.status = 'fault'
            self.error = str(exc)
            try:
                self._event('arm_execution_fault', {'error': self.error,
                            'task_terminal': False, 'torque_off_sent': False})
            except Exception:
                pass
        return self.snapshot()

    def snapshot(self):
        now = self.clock_ms()
        self._refresh_recovery(now)
        wait = 0
        if self.status == 'recovering' and self.recovery_goal_sent_ms is not None:
            wait = max(0, self.policy['recovery_observation_delay_ms'] -
                       self.ticks_diff(now, self.recovery_goal_sent_ms))
        elif self.status == 'running' and not self._invalidated and self.pending_stage is None:
            if self.phase == 'descend_lead':
                wait = max(0, self.policy['descend_axis4_lead_ms'] -
                           self.ticks_diff(now, self._lead_started_ms))
            elif self._last_poll_ms is not None:
                wait = max(0, self.policy['poll_interval_ms'] -
                           self.ticks_diff(now, self._last_poll_ms))
        return {'status': self.status, 'phase': self.phase,
                'target_version': self.target_version, 'pending_stage': self.pending_stage,
                'target_valid': not self._invalidated,
                'grasp_confirmed': self.grasp_confirmed, 'success_ready': self._success_ready,
                'grip_confirmation': dict(self.grip_confirmation) if self.grip_confirmation else None,
                'last_positions': list(self.last_positions), 'last_sample': self.last_sample,
                'feedback_status': self.feedback_status, 'error': self.error,
                'recovery_reason': self.recovery_reason,
                'recovery_goal_sent_ms': self.recovery_goal_sent_ms,
                'can_reobserve': self.status == 'reobserve_ready', 'next_poll_in_ms': wait}

    def close(self):
        """Release UART only. This is not stop, recovery, or torque release."""
        if self.status == 'closed':
            return
        self.pending_stage = None
        self._success_ready = False
        self.status = 'closed'
        self.io.close()


class _PickExecutionTransport:
    """Lazy UART adapter using the unchanged checked packet/ready helpers."""
    def __init__(self, control, ready_control, plan):
        self.control = dict(control)
        self.control['profiles'] = [dict(profile) for profile in control['profiles']]
        self.ready_control = dict(ready_control)
        self.ready_control['goals'] = list(ready_control['goals'])
        self.ready_control['profiles'] = [dict(profile) for profile in ready_control['profiles']]
        self.uart = None
        # Validate every full/subset packet before the first hardware access.
        for name in ('hover', 'descend', 'close', 'lift'):
            sync_frame(plan[name], control['profiles'])
        sync_frame(plan['descend'], control['profiles'], [4])
        sync_frame(plan['descend'], control['profiles'], [1, 2, 3, 5, 6])
        sync_frame(ready_control['goals'], ready_control['profiles'])

    def _port(self):
        if self.uart is None:
            pins = FPIOA()
            pins.set_function(3, FPIOA.UART1_TXD)
            pins.set_function(4, FPIOA.UART1_RXD)
            self.uart = UART(UART.UART1, baudrate=1000000, bits=UART.EIGHTBITS,
                             parity=UART.PARITY_NONE, stop=UART.STOPBITS_ONE)
        return self.uart

    def send_goals(self, goals, servo_ids):
        packet = sync_frame(goals, self.control['profiles'], servo_ids)
        if self._port().write(packet) != len(packet):
            raise RuntimeError('Short cooperative pick goal write')
        # As in the existing direct transport, drain TX before feedback/cleanup.
        time.sleep_ms(2)

    def read_positions(self, servo_ids):
        values = []
        for axis in servo_ids:
            raw = read_checked(self._port(), axis, 56, 2)
            values.append(signed_position(raw[0], raw[1]))
        return values

    def send_ready2(self):
        return dispatch_ready(self._port(), self.ready_control)

    def close(self):
        if self.uart is not None:
            self.uart.deinit()
            self.uart = None


def open_pick_execution(control, ready_control, plan, policy, target_version, emit=None):
    """Build the next-version executor without opening UART or sending targets."""
    transport = _PickExecutionTransport(control, ready_control, plan)
    return PickExecution(plan, policy, transport, target_version,
                         time.ticks_ms, time.ticks_diff, emit)
