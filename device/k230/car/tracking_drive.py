"""Stepped car decisions for a persistent task; no camera, UART or arm IO.

One update consumes current observations and an actual motor reply. Returned
commands are intentions: the task owner sends them and supplies the reply on
the next update. Model distances remain nominal; only a new static complete
observation may authorize the arm.
"""
import math
try:
    from .blue_tracker import age_ms
    from .continuous_math import wheel_speeds
    from .near_control import (near_plan, verify_parked, correction_plan,
                               distance_anchor, predict_center, _geometry)
    from .near_runtime import Odometry, count_delta, coarse_drive_plan
except ImportError:
    from blue_tracker import age_ms
    from continuous_math import wheel_speeds
    from near_control import (near_plan, verify_parked, correction_plan,
                              distance_anchor, predict_center, _geometry)
    from near_runtime import Odometry, count_delta, coarse_drive_plan


class DriveController:
    def __init__(self, projection, config):
        self.projection, self.config = projection, config
        self.near = config['near_parking']
        self.options = config.get('tracking_task', {})
        self.version = None
        self.phase = 'observe'
        self.anchor = self.paired_coarse = self.parking = None
        self._paired_point = None
        self._fine_request = None
        self._last_precise = None
        self._motor = self._previous_motor = None
        self._stable_counts = self._stable_at = self.stopped_at_ms = None
        self._active_action = None
        self._static_ok = True
        self._static_visibility_ok = True
        self._parking_reference = self._parking_ready_ms = None
        self._moving_started = False
        self._motion_hold = None
        self._search_direction = None
        self._searching = False
        self._search_reverse_done = False
        self._adjust_due = False
        self._adjust_count = 0
        self.corrections = 0
        self.fine_requests = 0
        self.odometry = Odometry(config['mm_per_count'])

    def cancel_fine(self):
        """The owner cancelled the job; no result or timestamp is fabricated."""
        self._fine_request = None

    def resume_motion(self):
        """Explicit owner acknowledgement of an existing button/link stop."""
        self._motion_hold = None

    def restart_observation(self, change_view=False):
        """After arm recovery, demand a NEW stopped observation of this target."""
        self.anchor = self.parking = self._paired_point = None
        self._parking_reference = self._parking_ready_ms = None
        self._fine_request = None
        self._adjust_due = bool(change_view)
        self._moving_started = False
        self.phase = 'settling'

    def request_search(self, direction=None):
        self._searching = direction != 'top'
        self._search_direction = direction or self._search_direction or 'left'
        self._search_reverse_done = False
        self.anchor = self.parking = self._paired_point = None
        self.cancel_fine()

    def _changed_target(self, version):
        initial_search = self.version is None and self._searching
        self.version = version
        self.anchor = self.paired_coarse = self.parking = self._paired_point = None
        self._parking_reference = self._parking_ready_ms = None
        self._fine_request = self._last_precise = None
        self._adjust_due = False
        self._moving_started = False
        self._searching = initial_search
        self.phase = 'observe'

    def _observe_motor(self, state, now):
        sent = state.get('_k230_sent_ms', now)
        received = state.get('_k230_received_ms', now)
        if (self._motor is None or state.get('ms') != self._motor.get('ms') or
                any(state[k] != self._motor[k] for k in ('encoder_a', 'encoder_b'))):
            self._previous_motor, self._motor = self._motor, dict(state)
            rows = self.odometry.rows
            # A stopped car may wait indefinitely: constant-count intervals
            # need endpoints, not an ever-growing list of identical samples.
            counts = (state['encoder_a'], state['encoder_b'])
            if len(rows) > 1 and rows[-1][1:] == counts and rows[-2][1:] == counts:
                rows.pop()
            self.odometry.add(state, sent, received)
        counts = (state['encoder_a'], state['encoder_b'])
        if state['run']:
            self._stable_at = self._stable_counts = self.stopped_at_ms = None
        elif counts != self._stable_counts:
            self._stable_counts, self._stable_at = counts, received
            self.stopped_at_ms = None
        elif (self._stable_at is not None and
              age_ms(received, self._stable_at) >= self.config['stopped_observation_ms']):
            self.stopped_at_ms = self._stable_at
        request = self._fine_request
        if request and request['purpose'] == 'static':
            reference = request['motor']
            self._static_ok = self._static_ok and not state['run'] and all(
                abs(count_delta(reference[k], state[k]))*self.config['mm_per_count'] <= .1
                for k in ('encoder_a', 'encoder_b'))
        stamps = []
        if self.anchor:
            stamps.append(self.anchor['capture_timestamp_ms'])
        if request:
            stamps.append(request['capture_timestamp_ms'])
        if stamps:
            while len(self.odometry.rows) > 2 and all(
                    age_ms(self.odometry.rows[1][0], stamp) <= 0 for stamp in stamps):
                self.odometry.rows.pop(0)
        else:
            self.odometry.rows = self.odometry.rows[-2:]

    def _speeds(self):
        a, b = self._previous_motor, self._motor
        if a is None or a['ms'] == b['ms'] or not b['run']:
            return {'left_mm_s': 0, 'right_mm_s': 0}
        return wheel_speeds(a, b, self.config['mm_per_count'])

    def _coarse_plan(self, coarse, now):
        if not isinstance(coarse, dict):
            return None
        evidence = coarse.get('visual_evidence') or {}
        if (evidence.get('ambiguous') or evidence.get('associated') is False or
                evidence.get('distance_valid') is False):
            return None
        try:
            plan = coarse_drive_plan(coarse, self.projection, self.config, now, self._speeds())
            return plan if plan['status'] == 'follow' else None
        except (ValueError, TypeError, KeyError):
            return None

    def _visible(self, coarse, event):
        evidence = (coarse or {}).get('visual_evidence') or {}
        if (event.get('event') == 'suspected' or event.get('movement_state') == 'suspected' or
                event.get('visibility') in ('unknown', 'ambiguous') or evidence.get('ambiguous') or
                evidence.get('associated') is False or event.get('associated') is False):
            return False
        if 'visible' in event:
            return bool(event['visible'])
        return bool(evidence.get('visible', (coarse or {}).get('status') == 'tracking'))

    def _bind_anchor(self, observation, paired):
        distance_anchor(observation)
        self.anchor, self.paired_coarse = observation, paired
        self._paired_point = None
        if (isinstance(paired, dict) and paired.get('frame_id') == observation.get('frame_id') and
                paired.get('capture_timestamp_ms') == observation.get('capture_timestamp_ms')):
            pixel = paired.get('center_pixel')
            if pixel is not None:
                try:
                    # A coarse centroid is not the fine cube centre. This
                    # assumed-height point ONLY propagates its paired ray.
                    self._paired_point = self.projection.pixel_to_plane(pixel, 10)
                except ValueError:
                    pass

    def _near_plan(self, coarse_plan, now):
        deltas = self.odometry.since(self.anchor['capture_timestamp_ms'], now)
        plan = near_plan(self.anchor, self.projection, self.config['workspace'], self.near,
                         now, wheel_deltas=deltas, speeds=self._speeds(), encoder_guided=True)
        if coarse_plan and self._paired_point is not None:
            expected = predict_center(self._paired_point, deltas, self.near['track_width_mm'],
                self.projection.normal, self.projection.forward_axis_Base,
                self.projection.motion_origin_Base_mm)
            expected_heading = _geometry(expected, self.projection, self.config['workspace'])['bearing_deg']
            heading = plan['bearing_deg'] + coarse_plan['predicted_bearing_deg']-expected_heading
            plan = near_plan(self.anchor, self.projection, self.config['workspace'], self.near,
                now, wheel_deltas=deltas, speeds=self._speeds(), encoder_guided=True,
                steering_bearing_deg=heading)
            plan['heading_source'] = 'fine_with_paired_coarse_change'
        return plan

    def _request(self, purpose, coarse, state, now, decision, pending):
        self.phase = 'fine_'+purpose
        if self._fine_request is None:
            self._fine_request = {'purpose': purpose, 'motor': dict(state),
                'capture_timestamp_ms': (coarse or {}).get('capture_timestamp_ms', now),
                'request_id': (self.version, self.fine_requests+1),
                'stopped_at_ms': self.stopped_at_ms,
                'initial': not self._moving_started}
            self._static_ok = not state['run']
            self._static_visibility_ok = bool(decision.get('target_visible'))
            self.fine_requests += 1
        if not pending:
            decision['request_fine'] = purpose
            decision['fine_request_id'] = self._fine_request['request_id']

    def _accept(self, complete, state, now, coarse_plan, decision):
        if complete.get('target_version') != self.version:
            return None
        observation = complete.get('observation')
        purpose = complete.get('purpose')
        if not isinstance(observation, dict) or purpose not in ('moving', 'static'):
            return None
        request = self._fine_request
        paired = complete.get('paired_coarse')
        stamp = observation.get('capture_timestamp_ms')
        ready = complete.get('result_ready_ms', now)
        request_id = complete.get('request_id')
        if isinstance(request_id, list):
            request_id = tuple(request_id)
        if (request is None or request['purpose'] != purpose or
                request_id != request['request_id'] or
                type(stamp) is not int or type(ready) is not int or not isinstance(paired, dict) or
                paired.get('frame_id') != observation.get('frame_id') or
                paired.get('capture_timestamp_ms') != stamp or
                age_ms(stamp, request['capture_timestamp_ms']) < 0 or
                age_ms(ready, stamp) < 0 or
                not 0 <= age_ms(now, ready) <= self.near['max_age_ms']):
            return None
        key = (self.version, observation.get('frame_id'), purpose)
        if key == self._last_precise:
            return None
        self._last_precise = key
        decision['precise_consumed'] = True
        self._fine_request = None
        if purpose == 'moving':
            try:
                self._bind_anchor(observation, paired)
                self.phase = 'following'
            except (ValueError, TypeError):
                self.phase = 'settling'
            return None
        stopped = request.get('stopped_at_ms')
        stationary = (stopped is not None and self._static_ok and
                      complete.get('stationary_since_capture', True))
        verdict = verify_parked(observation, self.projection, self.config['workspace'], self.near,
            now, stopped if stopped is not None else now,
            complete.get('result_ready_ms', now), stationary_since_capture=stationary)
        if verdict['accepted']:
            if not self._static_visibility_ok:
                self.phase = 'settling'
                return None
            self.parking = {'status': 'parked', 'verification': verdict,
                            'verification_observation': observation}
            self._parking_reference = dict(request['motor'])
            self._parking_ready_ms = ready
            self.phase = 'parked'
            return None
        self.parking = None
        if verdict['status'] == 'not_parked':
            self._bind_anchor(observation, paired)
            if request and request['initial']:
                plan = self._near_plan(coarse_plan, now)
                if plan['status'] == 'follow':
                    self.phase = 'following'
                    return None
            correction = correction_plan(verdict, self.projection,
                                         self.config['workspace'], self.near)
            if correction.get('status') == 'arc':
                self.corrections += 1
                self.phase = 'correcting'
                return correction['command']
        # An unavailable measurement never becomes parking authority. A
        # visible target can be observed from a genuinely different view.
        self.phase = 'observing'
        self._adjust_due = True
        return None

    def _arc(self, angle):
        travel = max(1, int(self.near['track_width_mm']*math.radians(abs(angle))/2+.5))
        direction = 1 if angle >= 0 else -1
        return 'ARC,%d,%d' % (-direction*travel, direction*travel)

    def _adjust(self, allow_reverse=True):
        self._adjust_due = False
        self._adjust_count += 1
        if allow_reverse and self._adjust_count % 3 == 0:
            distance = self.options.get('reobserve_reverse_mm', 10)
            return 'ARC,%d,%d' % (-distance, -distance)
        angle = self.options.get('reobserve_angle_deg', 5)
        return self._arc(angle if self._adjust_count % 3 == 1 else -angle)

    def update(self, coarse, motor_state, now_ms, precise=None, target_version=0,
               fine_pending=False, target_event=None, arm_active=False):
        event = target_event or {}
        decision = {'phase': self.phase, 'status': 'running', 'command': None,
                    'request_fine': None, 'allow_arm': False, 'parking': None,
                    'target_version': target_version, 'precise_consumed': False}
        changed = self.version != target_version
        if changed:
            self._changed_target(target_version)
        initial_busy = self._motor is None and motor_state['run']
        self._observe_motor(motor_state, now_ms)
        state, running = motor_state, bool(motor_state['run'])
        if event.get('event') == 'exited' or event.get('search'):
            if not self._searching:
                self.request_search(event.get('last_disappearance_direction'))
        coarse_plan = self._coarse_plan(coarse, now_ms)
        visible = self._visible(coarse, event)
        target_valid = event.get('target_valid', True)
        decision['target_visible'] = visible
        if self._fine_request and self._fine_request['purpose'] == 'static':
            self._static_visibility_ok = self._static_visibility_ok and visible
        if not target_valid:
            self.anchor = self.parking = self._paired_point = None
            self._parking_reference = self._parking_ready_ms = None
            self.cancel_fine()
        if self.parking:
            reference = self._parking_reference
            cache_valid = (visible and not running and reference is not None and
                0 <= age_ms(now_ms, self._parking_ready_ms) <= self.near['max_age_ms'] and
                all(abs(count_delta(reference[k], state[k]))*self.config['mm_per_count'] <= .1
                    for k in ('encoder_a', 'encoder_b')))
            if not cache_valid:
                self.restart_observation()

        def finish(command=None, status=None):
            if command:
                decision['command'] = command
                name = command.split(',')[0]
                if name in ('FOLLOW', 'SPEED'):
                    self._active_action = 'follow'
                    self._moving_started = True
                elif name == 'ARC':
                    self._active_action = 'arc'
                    self._moving_started = True
                elif name == 'STOP':
                    self._active_action = None
            decision.update(phase=self.phase, corrections=self.corrections,
                            fine_requests=self.fine_requests)
            if status:
                decision['status'] = status
            return decision

        if arm_active:
            self.phase = 'arm'
            return finish('STOP' if running else None, 'arm_active')
        if initial_busy:
            self._motion_hold = 'motor_was_running'
        if state.get('result', 0) not in (0, 1):
            self._motion_hold = 'motor_reply_rejected'
        if not running and self._active_action:
            reason = state.get('reason', 0)
            if reason in (2, 4):
                self._motion_hold = 'existing_motor_stop_%d' % reason
            elif self._active_action == 'arc' and reason != 8:
                self._motion_hold = 'arc_stopped_%d' % reason
            self._active_action = None
            if self.phase in ('correcting', 'adjusting'):
                self.anchor = self._paired_point = None
                self.phase = 'settling'
            elif self.phase == 'searching':
                self.phase = 'observing'
        if self._motion_hold:
            decision['motion_wait_reason'] = self._motion_hold
            return finish('STOP' if running else None, 'motion_wait')
        # ARC cannot be retargeted by SPEED. Follow can smoothly use the fresh
        # coarse target, provided it has not entered the static-near region.
        if changed and running and self._active_action == 'arc':
            return finish('STOP', 'target_changed')
        if (changed and running and coarse_plan is not None and
                coarse_plan['coarse_Base_X_mm'] <= self.near['capture_Base_X_mm']):
            self.phase = 'settling'
            return finish('STOP', 'target_changed_near')
        if self._searching:
            evidence = (coarse or {}).get('visual_evidence') or {}
            if coarse_plan or (visible and evidence.get('reason') != 'clipped_blob'):
                self._searching = False
                self.phase = 'observe'
                if running:
                    return finish('STOP', 'reacquired')
            else:
                self.phase = 'searching'
                if running:
                    return finish('HB')
                if self.stopped_at_ms is None:
                    return finish(status='settling')
                if self._search_direction == 'bottom' and not self._search_reverse_done:
                    self._search_reverse_done = True
                    distance = self.options.get('reobserve_reverse_mm', 10)
                    return finish('ARC,%d,%d' % (-distance, -distance), 'searching')
                angle = self.options.get('search_angle_deg', 10)
                if self._search_direction == 'right':
                    angle = -angle
                return finish(self._arc(angle), 'searching')
        # A version change is processed above before accepting a completed job.
        if precise is not None and target_valid:
            command = self._accept(precise, state, now_ms, coarse_plan, decision)
            if command:
                return finish(command)
        if self.parking:
            decision.update(allow_arm=visible, parking=self.parking)
            self.phase = 'parked'
            return finish(status='parked' if visible else 'waiting_for_target')
        if self._active_action == 'arc' and running:
            return finish('HB')
        if self._fine_request is not None and self._fine_request['purpose'] == 'static':
            # A stopped verification owns the car until its result arrives.
            # The old moving anchor may still predict a fraction of a mm to
            # the preferred point; that must not restart FOLLOW mid-exposure.
            if running:
                self.cancel_fine()
                self.phase = 'settling'
                return finish('STOP', 'settling')
            if not fine_pending:
                self.cancel_fine()
                self._request('static', coarse, state, now_ms, decision, False)
            self.phase = 'fine_static'
            return finish(status='localizing_static')
        if not target_valid:
            self.phase = 'observing'
            if coarse_plan is None or coarse_plan['coarse_Base_X_mm'] <= self.near['capture_Base_X_mm']:
                return finish('STOP' if running else None, 'reacquiring')
            name = 'SPEED' if running else 'FOLLOW'
            return finish('%s,%d,%d' % (name, coarse_plan['left_mm_s'],
                                       coarse_plan['right_mm_s']), 'reacquiring')
        evidence = (coarse or {}).get('visual_evidence') or {}
        if (self.anchor is None and coarse_plan is None and visible and not fine_pending and
                evidence.get('reason') == 'clipped_blob'):
            edges = evidence.get('edge_directions', [])
            direction = next((edge for edge in edges if edge in ('left', 'right', 'bottom')), None)
            if direction is not None:
                self.request_search(direction)
                self.phase = 'searching'
                return finish('STOP' if running else None, 'centering_clipped_target')
        if self.phase == 'settling' or self._adjust_due:
            if running:
                return finish('STOP', 'settling')
            if self.stopped_at_ms is None:
                return finish(status='settling')
            if self._adjust_due:
                if not visible:
                    return finish(status='waiting_for_target')
                self.phase = 'adjusting'
                return finish(self._adjust(allow_reverse=coarse_plan is not None))
            if not visible:
                return finish(status='waiting_for_target')
            self._request('static', coarse, state, now_ms, decision, fine_pending)
            return finish()
        if self.anchor:
            plan = self._near_plan(coarse_plan, now_ms)
            decision['plan'] = plan
            if plan['status'] != 'follow':
                self.phase = 'settling'
                return finish('STOP' if running else None, 'settling')
            self.phase = 'following'
            name = 'SPEED' if running else 'FOLLOW'
            return finish('%s,%d,%d' % (name, plan['left_mm_s'], plan['right_mm_s']))
        if coarse_plan is None:
            if visible and evidence.get('reason') != 'clipped_blob':
                if running:
                    self.phase = 'settling'
                    return finish('STOP', 'checking_uncertain_geometry')
                if self.stopped_at_ms is not None:
                    self._request('static', coarse, state, now_ms, decision, fine_pending)
                    return finish(status='checking_uncertain_geometry')
            self.phase = 'observing'
            return finish('STOP' if running else None, 'waiting_for_target')
        decision['plan'] = coarse_plan
        capture_distance = self.near['capture_Base_X_mm']
        if coarse_plan['coarse_Base_X_mm'] <= capture_distance:
            if not running:
                if self.stopped_at_ms is None:
                    return finish(status='settling')
                self._request('static', coarse, state, now_ms, decision, fine_pending)
                return finish()
            self._request('moving', coarse, state, now_ms, decision, fine_pending)
        self.phase = 'fine_moving' if self._fine_request else 'following'
        name = 'SPEED' if running else 'FOLLOW'
        return finish('%s,%d,%d' % (name, coarse_plan['left_mm_s'], coarse_plan['right_mm_s']))
