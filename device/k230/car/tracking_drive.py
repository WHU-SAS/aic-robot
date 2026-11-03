"""Stepped car decisions for a persistent task; no camera, UART or arm IO.

One update consumes current observations and an actual motor reply. Returned
commands are intentions: the task owner sends them and supplies the reply on
the next update. Model distances remain nominal; only a new static complete
observation may authorize the arm.
"""
import math
try:
    from .blue_tracker import age_ms
    from .continuous_math import wheel_speeds, wheel_targets, forward_x_per_mm
    from .near_control import (near_plan, verify_parked, correction_plan,
                               distance_anchor, predict_center, _geometry, target_x)
    from .near_runtime import Odometry, count_delta, coarse_drive_plan
except ImportError:
    from blue_tracker import age_ms
    from continuous_math import wheel_speeds, wheel_targets, forward_x_per_mm
    from near_control import (near_plan, verify_parked, correction_plan,
                              distance_anchor, predict_center, _geometry, target_x)
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
        self._coarse_reference = None
        self._near_recovery = None
        self._awaiting_query_identity = False
        self._moving_retry_frame = None
        self._motor = self._previous_motor = None
        self._stable_counts = self._stable_at = self.stopped_at_ms = None
        self._active_action = None
        self._arc_command = None
        self.arc_timeouts = 0
        self.last_arc_end = None
        self._static_ok = True
        self._parking_reference = self._parking_ready_ms = None
        self._moving_started = False
        self._motion_hold = None
        self._search_direction = None
        self._search_region = None
        self._search_hint = None
        self._searching = False
        self._search_reverse_required = False
        self._search_reverse_done = False
        self._search_inspecting = False
        self._search_view_pending = False
        self._search_arc_active = False
        self._search_follow_active = False
        self._search_speeds = None
        self._search_view_counts = None
        self._search_view_yaw_deg = 0
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

    def restart_observation(self, change_view=False, discard_reference=False):
        """After arm recovery, demand a NEW stopped observation of this target."""
        self.anchor = self.parking = self._paired_point = None
        self._coarse_reference = None
        self._parking_reference = self._parking_ready_ms = None
        self._fine_request = None
        self._search_inspecting = False
        self._search_view_pending = False
        self._search_view_counts = None
        self._awaiting_query_identity = False
        if discard_reference:
            self._near_recovery = self.paired_coarse = None
        self._adjust_due = bool(change_view)
        self._moving_started = False
        self.phase = 'settling'

    def request_search(self, direction=None):
        if self._searching:
            return
        self._searching = direction != 'top'
        self._search_direction = direction or self._search_direction or 'left'
        self._search_reverse_required = direction == 'bottom'
        self._search_reverse_done = False
        self._search_inspecting = self._search_view_pending = False
        self._search_arc_active = False
        self._search_follow_active = False
        self._search_speeds = self._search_view_counts = None
        self._search_hint = None
        self._search_view_yaw_deg = 0
        self.anchor = self.parking = self._paired_point = None
        self._coarse_reference = None
        self._near_recovery = None
        self._awaiting_query_identity = False
        self.cancel_fine()

    def _changed_target(self, version):
        searching = self._searching
        self.version = version
        self.anchor = self.paired_coarse = self.parking = self._paired_point = None
        self._parking_reference = self._parking_ready_ms = None
        self._fine_request = self._last_precise = None
        self._coarse_reference = self._moving_retry_frame = None
        self._near_recovery = None
        self._awaiting_query_identity = False
        self._adjust_due = False
        self._moving_started = False
        self._searching = searching
        self._search_inspecting = False
        self._search_hint = None
        self._search_view_pending = False
        self._search_view_counts = None
        self.phase = 'observe'

    def _end_search(self):
        self._searching = self._search_inspecting = False
        self._search_view_pending = False
        self._search_arc_active = False
        self._search_follow_active = False
        self._search_speeds = self._search_view_counts = None
        self._search_hint = None

    def _canonical_coarse(self, coarse):
        """Evidence and control coordinates must name the same blue candidate."""
        if not isinstance(coarse, dict) or not isinstance(coarse.get('visual_evidence'), dict):
            return coarse
        evidence = dict(coarse['visual_evidence'])
        result = dict(coarse, visual_evidence=evidence)
        keys = ('bbox_px', 'center_pixel', 'pixels', 'ground_contact_proxy_Base_mm',
                'image_midline_error_px', 'confirmations')
        same = all(coarse.get(key) == evidence.get(key) for key in
                   ('bbox_px', 'center_pixel', 'ground_contact_proxy_Base_mm'))
        for key in keys:
            result.pop(key, None)
            if key in evidence:
                result[key] = evidence[key]
        if evidence.get('timestamp_ms') != coarse.get('capture_timestamp_ms'):
            evidence['associated'] = evidence['navigation_confirmed'] = False
        navigation = evidence.get('navigation_confirmed')
        if navigation is True:
            result['status'] = 'tracking'
        elif navigation is False and coarse.get('status') in ('tracking', 'confirming'):
            result['status'] = 'confirming'
        elif navigation is None and not same and coarse.get('status') in ('tracking', 'confirming'):
            result['status'] = 'confirming'
        return result

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
        if self._search_view_pending and self._search_view_counts is not None:
            previous = self._search_view_counts
            dl = count_delta(previous[0], counts[0])*self.config['mm_per_count']
            dr = -count_delta(previous[1], counts[1])*self.config['mm_per_count']
            self._search_view_yaw_deg += math.degrees(abs(dr-dl)/self.near['track_width_mm'])
            self._search_view_counts = counts
            if self._search_view_yaw_deg >= self.options.get('search_angle_deg', 10):
                self._search_view_pending = False
                self._search_view_counts = None
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
        if self._coarse_reference:
            stamps.append(self._coarse_reference['capture_timestamp_ms'])
        if self._near_recovery:
            stamps.append(self._near_recovery['capture_timestamp_ms'])
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
                evidence.get('distance_valid') is False or
                evidence.get('navigation_confirmed') is False):
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

    def _pixel_point(self, coarse):
        evidence = (coarse or {}).get('visual_evidence') or {}
        pixel = (coarse or {}).get('center_pixel', evidence.get('center_pixel'))
        return self.projection.pixel_to_plane(pixel, 10)

    def _remember_coarse(self, coarse):
        self._remember_search_region(coarse)
        try:
            self._coarse_reference = {
                'capture_timestamp_ms': coarse['capture_timestamp_ms'],
                'point': list(coarse['ground_contact_proxy_Base_mm']),
                'ray': self._pixel_point(coarse)}
        except (ValueError, TypeError, KeyError):
            pass

    def _remember_search_region(self, coarse, precise_pair=False):
        """Keep full-target size for direction hints, never a distance anchor."""
        if not isinstance(coarse, dict):
            return
        evidence = (coarse or {}).get('visual_evidence') or coarse or {}
        if (evidence.get('ambiguous') or evidence.get('reason') == 'clipped_blob' or
                evidence.get('associated') is False or
                (not precise_pair and (evidence.get('distance_valid') is not True or
                 evidence.get('navigation_confirmed') is not True))):
            return
        if self._search_shape(evidence):
            self._search_region = {'bbox_px':list(evidence['bbox_px']),
                                   'center_pixel':list(evidence['center_pixel'])}

    def _search_match(self, cue, reference):
        if reference is None or not self._search_shape(cue):
            return False
        tracking = self.config['tracking']
        width, height = tracking['image_size_px']
        a, b = cue['center_pixel'], reference['center_pixel']
        limit = tracking['max_size_ratio']
        return (math.sqrt(((a[0]-b[0])/width)**2+((a[1]-b[1])/height)**2) <=
            tracking['max_center_jump_fraction'] and all(1/limit <=
                cue['bbox_px'][axis]/reference['bbox_px'][axis] <= limit for axis in (2, 3)))

    def _confirmed_search_hint(self, cue, coarse):
        """A new region needs its own three frames, not another cue's count."""
        if self._search_match(cue, self._search_region):
            self._search_hint = None
            return True
        previous = self._search_hint
        stamp, frame = coarse['capture_timestamp_ms'], coarse.get('frame_id')
        required = self.config['tracking'].get('confirmation_frames', 3)
        evidence = coarse.get('visual_evidence') or {}
        native_count = evidence.get('association_confirmations', 0)
        if (evidence.get('associated') is True and not evidence.get('ambiguous') and
                evidence.get('bbox_px') == cue['bbox_px'] and
                evidence.get('center_pixel') == cue['center_pixel'] and
                isinstance(native_count, (int, float)) and native_count >= required):
            self._search_hint = None
            return True
        if previous and (stamp == previous['timestamp_ms'] or frame == previous['frame_id']):
            return (stamp == previous['timestamp_ms'] and
                self._search_match(cue, previous) and previous['confirmations'] >= required)
        same = (previous is not None and
            0 < age_ms(stamp, previous['timestamp_ms']) <= self.config['tracking']['max_age_ms'] and
            self._search_match(cue, previous))
        count = min(required, previous['confirmations']+1) if same else 1
        self._search_hint = {'bbox_px':list(cue['bbox_px']),
            'center_pixel':list(cue['center_pixel']), 'timestamp_ms':stamp,
            'frame_id':frame, 'confirmations':count}
        return count >= required

    def _search_shape(self, cue):
        """Reuse the detector's shape gates; a blue speck is not a heading."""
        try:
            x, y, w, h = cue['bbox_px']
            u, v = cue['center_pixel']
            width, height = self.config['tracking']['image_size_px']
            tracking = self.config['tracking']
            pixels = cue.get('pixels')
            return (all(math.isfinite(k) for k in (x, y, w, h, u, v)) and
                min(w, h) >= 3 and 0 <= u < width and 0 <= v < height and
                max(w/h, h/w) <= tracking['max_aspect_ratio'] and
                (pixels is None or (0 <= pixels <= w*h and
                 pixels/(w*h) >= tracking['min_fill_ratio'])) and
                cue.get('reason', cue.get('measurement_reason')) not in
                    ('sparse_blob', 'implausible_aspect_ratio', 'invalid_or_tiny_blob'))
        except (ValueError, TypeError, KeyError, ZeroDivisionError):
            return False

    def _search_weak_shape(self, cue, associated):
        if not self._search_shape(cue):
            return False
        if cue.get('reason', cue.get('measurement_reason')) == 'clipped_blob':
            # Clipping skips these checks in the detector. Reuse them ONLY
            # for heading; do not publish the recomputed proxy or valid range.
            try:
                geometry = self.projection.blob_geometry(cue['bbox_px'])
                tracking = self.config['tracking']
                return (cue.get('pixels') is not None and
                    tracking['forward_range_mm'][0] <=
                    geometry['ground_contact_proxy_Base_mm'][0] <= tracking['forward_range_mm'][1] and
                    tracking['height_ratio_range'][0] <=
                    geometry['observed_to_expected_height_ratio'] <= tracking['height_ratio_range'][1])
            except (ValueError, TypeError, KeyError):
                return False
        reference = self._search_region
        limit = self.config['tracking']['max_size_ratio']
        return (associated and reference is not None and all(
            1/limit <= cue['bbox_px'][axis]/reference['bbox_px'][axis] <= limit for axis in (2, 3)))

    def _search_heading(self, coarse, event, now, decision):
        """Current visual direction has less authority than navigation/identity."""
        if not isinstance(coarse, dict):
            self._search_hint = None
            return None
        stamp = coarse.get('capture_timestamp_ms')
        if type(stamp) is not int or not 0 <= age_ms(now, stamp) <= self.config['tracking']['max_age_ms']:
            self._search_hint = None
            return None
        self._remember_search_region(event.get('last_seen'))
        if event.get('anchor_valid'):
            self._remember_search_region(event.get('paired_coarse'), precise_pair=True)
        self._remember_search_region(coarse)
        evidence = coarse.get('visual_evidence') or {}
        raw = coarse.get('visual_candidates')
        raw = raw if isinstance(raw, list) else [evidence]
        fresh = [cue for cue in raw if isinstance(cue, dict) and
                 cue.get('timestamp_ms', stamp) == stamp and self._search_shape(cue)]

        def independent(cues):
            def contains(outer, inner):
                x,y,w,h = outer['bbox_px']; a,b,c,d = inner['bbox_px']
                return x<a and y<b and x+w>a+c and y+h>b+d
            return [cue for cue in cues if not any(
                other is not cue and contains(other, cue) for other in cues)]

        canonical = (evidence.get('visible') and evidence.get('associated') is True and
            not evidence.get('ambiguous') and self._visible(coarse, event) and
            self._search_shape(evidence) and (evidence.get('distance_valid') is True or
                self._search_weak_shape(evidence, True)))
        measured = independent([cue for cue in fresh if cue.get('distance_valid') is True])
        if canonical:
            # The associated target A owns its heading. Other measurable blue
            # regions neither replace A nor make A's own direction ambiguous.
            pool = [evidence]
        elif measured:
            pool = measured
            if (len(measured) == 1 and not
                    (evidence.get('bbox_px') == measured[0].get('bbox_px') and
                     evidence.get('center_pixel') == measured[0].get('center_pixel')) and not
                    (evidence.get('associated') is True and
                     self._search_weak_shape(evidence, True))):
                # A unique measurable B may be hidden behind the detector's
                # stale weak A. The owner may restart native association on
                # the next frame; this frame still has direction authority only.
                decision['reset_tracker'] = True
        else:
            pool = [cue for cue in fresh if self._search_weak_shape(cue, False)]
            if (not evidence.get('ambiguous') and evidence.get('visible') and
                    evidence.get('associated') is True and
                    self._search_weak_shape(evidence, True) and not any(
                        cue.get('bbox_px') == evidence.get('bbox_px') and
                        cue.get('center_pixel') == evidence.get('center_pixel') for cue in pool)):
                pool.append(evidence)
            pool = independent(pool)
        if len(pool) != 1:
            self._search_hint = None
            return None
        if not self._confirmed_search_hint(pool[0], coarse):
            return None
        error = pool[0]['center_pixel'][0]-self.config['tracking']['image_size_px'][0]/2
        if abs(error) <= self.config['coarse_center_tolerance_px']:
            return None
        return 'right' if error > 0 else 'left'

    def _inspection_cue(self, coarse, visible, target_valid, precise_candidate, now, centered=True):
        """Current appearance can request a static query without a range."""
        evidence = (coarse or {}).get('visual_evidence') or {}
        stamp = (coarse or {}).get('capture_timestamp_ms')
        if (type(stamp) is not int or not
                0 <= age_ms(now, stamp) <= self.config['tracking']['max_age_ms'] or
                not (precise_candidate or visible and target_valid and self._search_shape(evidence))):
            return False
        if evidence.get('reason') == 'clipped_blob':
            return (self._search_weak_shape(evidence, evidence.get('associated') is True) and
                (not centered or abs(evidence['center_pixel'][0]-
                 self.config['tracking']['image_size_px'][0]/2) <=
                    self.config['coarse_center_tolerance_px']))
        return True

    def _coarse_direction(self, coarse, now):
        """A visible centroid can steer even when its table range is rejected."""
        try:
            point = predict_center(self._pixel_point(coarse),
                self.odometry.since(coarse['capture_timestamp_ms'], now),
                self.near['track_width_mm'], self.projection.normal,
                self.projection.forward_axis_Base, self.projection.motion_origin_Base_mm)
            return _geometry(point, self.projection, self.config['workspace'])['bearing_deg']
        except (ValueError, TypeError, KeyError):
            return None

    def _coarse_prediction(self, coarse, now, visible):
        """Finish only previously observed coarse travel; never a fine authority."""
        reference = self._coarse_reference
        if reference is None:
            return None
        deltas = self.odometry.since(reference['capture_timestamp_ms'], now)
        def propagate(point, increments):
            return predict_center(point, increments, self.near['track_width_mm'],
                self.projection.normal, self.projection.forward_axis_Base,
                self.projection.motion_origin_Base_mm)
        point, ray = propagate(reference['point'], deltas), propagate(reference['ray'], deltas)
        bearing = self._coarse_direction(coarse, now) if visible else None
        source = 'current_coarse_pixel' if bearing is not None else 'coarse_odometry'
        if bearing is None:
            bearing = _geometry(ray, self.projection, self.config['workspace'])['bearing_deg']
        speeds = self._speeds()
        coast = propagate(point, [(speeds['left_mm_s']*self.near['coast_s'],
                                    speeds['right_mm_s']*self.near['coast_s'])])
        destination = target_x(self.config['workspace'])
        targets = wheel_targets(bearing, max(0, (point[0]-destination)/forward_x_per_mm(
            self.projection)), self.near, self.near['track_width_mm'])
        return {'status': 'follow' if coast[0] > destination else 'stop_for_observation',
            'mode': 'coarse_prediction', 'heading_source': source,
            'capture_timestamp_ms': reference['capture_timestamp_ms'],
            'coarse_Base_X_mm': point[0], 'predicted_bearing_deg': bearing,
            'left_mm_s': int(targets['left_mm_s']+.5),
            'right_mm_s': int(targets['right_mm_s']+.5)}

    def _bind_anchor(self, observation, paired):
        distance_anchor(observation)
        paired = self._canonical_coarse(paired)
        self.anchor, self.paired_coarse = observation, paired
        self._paired_point = None
        if (isinstance(paired, dict) and paired.get('frame_id') == observation.get('frame_id') and
                paired.get('capture_timestamp_ms') == observation.get('capture_timestamp_ms')):
            self._remember_search_region(paired, precise_pair=True)
            pixel = paired.get('center_pixel', (paired.get('visual_evidence') or {}).get('center_pixel'))
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
            if purpose == 'static' and not self._searching:
                self._remember_near(now)
            self._fine_request = {'purpose': purpose, 'motor': dict(state),
                'capture_timestamp_ms': (coarse or {}).get('capture_timestamp_ms', now),
                'request_id': (self.version, self.fine_requests+1),
                'stopped_at_ms': self.stopped_at_ms,
                'query_only': purpose == 'static' and not decision.get('target_visible'),
                'initial': not self._moving_started}
            self._static_ok = not state['run']
            self.fine_requests += 1
        if not pending:
            decision['request_fine'] = purpose
            decision['fine_request_id'] = self._fine_request['request_id']

    def _remember_near(self, now, observation=None):
        """Retain nearby recovery geometry, never a parking capability."""
        reference = None
        observation = observation if observation is not None else self.anchor
        if observation is not None:
            observation, point, source = distance_anchor(observation)
            reference = {'capture_timestamp_ms': observation['capture_timestamp_ms'],
                         'point': list(point), 'source': source}
        elif self._near_recovery is None and self._coarse_reference is not None:
            coarse = self._coarse_reference
            reference = {'capture_timestamp_ms': coarse['capture_timestamp_ms'],
                         'point': list(coarse['ray']), 'source': 'coarse_assumed_height'}
        if reference is not None:
            point = self._recovery_point(reference, now)
            self._near_recovery = (reference if point[0] <=
                self.near['capture_Base_X_mm'] else None)
        if self._near_recovery is not None:
            self._near_recovery['last_move'] = None

    def _recovery_point(self, reference, now):
        return predict_center(reference['point'],
            self.odometry.since(reference['capture_timestamp_ms'], now),
            self.near['track_width_mm'], self.projection.normal,
            self.projection.forward_axis_Base, self.projection.motion_origin_Base_mm)

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
        verdict = None
        if purpose == 'static':
            stopped = request.get('stopped_at_ms')
            stationary = (stopped is not None and self._static_ok and
                          complete.get('stationary_since_capture', True))
            verdict = verify_parked(observation, self.projection, self.config['workspace'], self.near,
                now, stopped if stopped is not None else now,
                complete.get('result_ready_ms', now), stationary_since_capture=stationary)
        # An unknown coarse frame is not proof that this successful stopped
        # measurement moved. Keep the same request until association returns;
        # the owner retains the completion and the original motor latch/age.
        if (verdict and verdict['status'] in ('parked', 'not_parked') and
                not decision.get('query_visible')):
            decision['precise_deferred'] = True
            return None
        self._last_precise = key
        decision['precise_consumed'] = True
        self._fine_request = None
        if purpose == 'moving':
            try:
                self._bind_anchor(observation, paired)
                self.phase = 'following'
            except (ValueError, TypeError):
                self.phase = 'following'
                self._moving_retry_frame = decision.get('coarse_frame_id')
            return None
        if verdict['status'] in ('parked', 'not_parked'):
            self._remember_near(now, observation)
            self._awaiting_query_identity = not decision.get('target_visible')
        if verdict['accepted']:
            self._end_search()
            self.parking = {'status': 'parked', 'verification': verdict,
                            'verification_observation': observation}
            self._parking_reference = dict(request['motor'])
            self._parking_ready_ms = ready
            self.phase = 'parked'
            return None
        self.parking = None
        if verdict['status'] == 'not_parked':
            self._end_search()
            self._bind_anchor(observation, paired)
            plan = self._near_plan(coarse_plan, now)
            if self._awaiting_query_identity:
                self.phase = 'following' if plan['status'] == 'follow' else 'settling'
                return None
            if (plan['status'] == 'follow' and
                    not verdict['window']['longitudinal_allowed']):
                if not request['initial']:
                    self.corrections += 1
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
        if self._search_inspecting or request.get('query_only'):
            # Seeing this target and obtaining a full face are different.
            # A failed query changes the local view; it is not a new loss.
            self._searching = False
            self._search_inspecting = True
            self._search_view_pending = True
            self._search_view_counts = (state['encoder_a'], state['encoder_b'])
            self._search_view_yaw_deg = 0
            self._adjust_due = False
            self.phase = 'visual_tracking'
        else:
            self._adjust_due = True
        return None

    def _arc(self, angle):
        travel = max(1, int(self.near['track_width_mm']*math.radians(abs(angle))/2+.5))
        direction = 1 if angle >= 0 else -1
        return 'ARC,%d,%d' % (-direction*travel, direction*travel)

    def _recovery_cue(self, coarse, visible, target_valid, now):
        evidence = (coarse or {}).get('visual_evidence') or {}
        stamp = (coarse or {}).get('capture_timestamp_ms')
        count = evidence.get('association_confirmations', 0)
        return (visible and target_valid and evidence.get('associated') is True and
            type(stamp) is int and
            0 <= age_ms(now, stamp) <= self.config['tracking']['max_age_ms'] and
            isinstance(count, (int, float)) and
            count >= self.config['tracking'].get('confirmation_frames', 3))

    def _reverse_limit(self, reference, state):
        start = reference.get('reverse_start_counts')
        if start is None:
            return reference.get('reverse_exhausted', False)
        travel = max(abs(count_delta(start[index], state[key]))*self.config['mm_per_count']
                     for index,key in enumerate(('encoder_a', 'encoder_b')))
        speeds = self._speeds()
        coast = max(abs(speeds['left_mm_s']),abs(speeds['right_mm_s']))*self.near['coast_s']
        if travel+coast >= 100:
            reference['reverse_exhausted'] = True
        return reference.get('reverse_exhausted', False)

    def _adjust(self, allow_reverse=True, coarse=None, now=None):
        self._adjust_due = False
        self._adjust_count += 1
        recovery = self._near_recovery
        evidence = (coarse or {}).get('visual_evidence') or {}
        edges = evidence.get('edge_directions', []) if evidence.get('reason') == 'clipped_blob' else []
        if recovery is not None and ('bottom' in edges or
                'bottom_view_frame_ms' in recovery):
            recovery['bottom_view_frame_ms'] = coarse['capture_timestamp_ms']
        if (allow_reverse and self._adjust_count % 3 == 0 and
                not (recovery and recovery.get('reverse_exhausted'))):
            distance = self.options.get('reobserve_reverse_mm', 10)
            if recovery is not None:
                recovery['last_move'] = 'reverse'
            return 'ARC,%d,%d' % (-distance, -distance)
        angle = self.options.get('reobserve_angle_deg', 5)
        if recovery is not None:
            recovery['last_move'] = 'turn'
            if 'left' in edges:
                return self._arc(angle)
            if 'right' in edges:
                return self._arc(-angle)
            try:
                geometry = _geometry(self._recovery_point(recovery, now),
                                     self.projection, self.config['workspace'])
            except (ValueError, TypeError, KeyError):
                geometry = None
            if geometry and abs(geometry['image_error_px']) > self.near['center_tolerance_px']:
                return self._arc(angle if geometry['bearing_deg'] > 0 else -angle)
        return self._arc(angle if self._adjust_count % 3 == 1 else -angle)

    def update(self, coarse, motor_state, now_ms, precise=None, target_version=0,
               fine_pending=False, target_event=None, arm_active=False):
        coarse = self._canonical_coarse(coarse)
        event = target_event or {}
        decision = {'phase': self.phase, 'status': 'running', 'command': None,
                    'request_fine': None, 'allow_arm': False, 'parking': None,
                    'reset_tracker': False,
                    'target_version': target_version, 'precise_consumed': False,
                    'precise_deferred': False,
                    'coarse_frame_id': (coarse or {}).get('frame_id')}
        changed = self.version != target_version
        if changed:
            self._changed_target(target_version)
        initial_busy = self._motor is None and motor_state['run']
        self._observe_motor(motor_state, now_ms)
        state, running = motor_state, bool(motor_state['run'])
        visible = self._visible(coarse, event)
        precise_candidate = bool(event.get('precise_candidate'))
        query_visible = visible or precise_candidate
        if (event.get('event') == 'exited' or event.get('search') and not query_visible):
            if not self._searching:
                self.request_search(event.get('last_disappearance_direction'))
        target_valid = event.get('target_valid', True)
        invalidated_view = bool(not target_valid and self._near_recovery is not None and
                                'bottom_view_frame_ms' in self._near_recovery)
        measured_coarse_plan = self._coarse_plan(coarse, now_ms)
        coarse_plan = measured_coarse_plan if visible else None
        if coarse_plan is not None and target_valid and not self._searching:
            self._remember_coarse(coarse)
        elif (running and self._active_action == 'follow' and target_valid and
                not self._searching):
            coarse_plan = self._coarse_prediction(coarse, now_ms, visible)
        decision['target_visible'] = visible
        decision['precise_candidate'] = precise_candidate
        decision['query_visible'] = query_visible
        if not target_valid:
            self.anchor = self.parking = self._paired_point = None
            self._coarse_reference = None
            self._near_recovery = None
            self._parking_reference = self._parking_ready_ms = None
            if not (precise_candidate and self._fine_request and
                    self._fine_request['purpose'] == 'static' and
                    self._fine_request.get('query_only')):
                self.cancel_fine()
        if self.parking:
            reference = self._parking_reference
            cache_valid = (visible and not running and reference is not None and
                0 <= age_ms(now_ms, self._parking_ready_ms) <= self.near['max_age_ms'] and
                all(abs(count_delta(reference[k], state[k]))*self.config['mm_per_count'] <= .1
                    for k in ('encoder_a', 'encoder_b')))
            if not cache_valid:
                self.restart_observation()

        def finish(command=None, status=None, recovery_reverse=False):
            if command:
                decision['command'] = command
                name = command.split(',')[0]
                if name in ('FOLLOW', 'SPEED'):
                    if recovery_reverse or self._active_action == 'recovery_reverse':
                        self._active_action = 'recovery_reverse'
                    elif self._search_follow_active:
                        self._active_action = 'inspection_follow' if self._search_inspecting else 'search_follow'
                    else:
                        self._active_action = 'follow'
                    self._moving_started = True
                elif name == 'ARC':
                    self._active_action = 'arc'
                    self._arc_command = command
                    self._moving_started = True
                elif name == 'STOP':
                    self._active_action = None
                    self._search_arc_active = self._search_follow_active = False
                    self._search_speeds = None
            decision.update(phase=self.phase, corrections=self.corrections,
                            fine_requests=self.fine_requests,
                            arc_timeouts=self.arc_timeouts, last_arc_end=self.last_arc_end)
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
            if self._active_action == 'arc':
                self.last_arc_end = {'at_ms': now_ms, 'command': self._arc_command,
                    'target_version': self.version, 'phase': self.phase, 'state': dict(state)}
                self._search_arc_active = False
            elif self._active_action in ('search_follow', 'inspection_follow', 'recovery_reverse'):
                # Signed FOLLOW retains the MCU's existing segment deadline.
                # Reobserve after its reason-3 stop, then start a new segment.
                self._search_follow_active = False
                self._search_speeds = None
                if self._active_action == 'recovery_reverse':
                    self.phase = 'settling'
            if reason in (2, 4):
                self._motion_hold = 'existing_motor_stop_%d' % reason
            elif self._active_action == 'arc':
                if reason == 3:
                    # The MCU has exhausted this segment, not the whole task.
                    # A fresh stopped image must replace the old ARC geometry.
                    self.arc_timeouts += 1
                    self.restart_observation()
                    self.paired_coarse = None
                elif reason != 8:
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
        if invalidated_view and running:
            self.phase = 'observing'
            return finish('STOP', 'target_invalidated')
        if self._active_action == 'recovery_reverse' and running:
            reference = self._near_recovery
            evidence = (coarse or {}).get('visual_evidence') or {}
            supported = self._recovery_cue(coarse, visible, target_valid, now_ms)
            bottom = (evidence.get('reason') == 'clipped_blob' and
                      'bottom' in evidence.get('edge_directions', []))
            if (reference is None or not supported or not bottom or
                    self._reverse_limit(reference, state)):
                self.phase = 'settling'
                return finish('STOP', 'recovering_view')
            self.phase = 'recovering_view'
            return finish('HB', 'recovering_view')
        # ARC cannot be retargeted by SPEED. Follow can smoothly use the fresh
        # coarse target, provided it has not entered the static-near region.
        if changed and running and self._active_action in ('arc', 'inspection_follow'):
            return finish('STOP', 'target_changed')
        if (changed and running and coarse_plan is not None and
                coarse_plan['coarse_Base_X_mm'] <= self.near['capture_Base_X_mm']):
            self.phase = 'settling'
            return finish('STOP', 'target_changed_near')
        # A version change is processed above before accepting a completed job.
        if precise is not None and (target_valid or
                (precise_candidate and precise.get('purpose') == 'static')):
            command = self._accept(precise, state, now_ms, coarse_plan, decision)
            if command:
                return finish(command)
        if self._awaiting_query_identity:
            if visible and target_valid:
                self._awaiting_query_identity = False
            else:
                self.phase = 'visual_tracking'
                return finish('STOP' if running else None, 'waiting_for_identity')
        if self.parking:
            decision.update(allow_arm=visible, parking=self.parking)
            self.phase = 'parked'
            return finish(status='parked' if visible else 'waiting_for_target')
        if self._fine_request is not None and self._fine_request['purpose'] == 'static':
            # A stopped verification owns the car until its result arrives.
            # The old moving anchor may still predict a fraction of a mm to
            # the preferred point; that must not restart FOLLOW mid-exposure.
            if running:
                self.cancel_fine()
                self.phase = 'settling'
                return finish('STOP', 'settling')
            if not fine_pending and not decision['precise_deferred']:
                self.cancel_fine()
                if not query_visible:
                    self._search_inspecting = False
                    self.phase = 'settling'
                    return finish(status='waiting_for_target')
                self._request('static', coarse, state, now_ms, decision, False)
            self.phase = 'fine_static'
            return finish(status='localizing_static')
        if self._searching:
            evidence = (coarse or {}).get('visual_evidence') or {}
            direction = self._search_heading(coarse, event, now_ms, decision)
            if direction is not None:
                self._search_direction = direction
            if running and not (self._search_arc_active or self._search_follow_active):
                self.phase = 'searching'
                return finish('STOP', 'starting_search')
            if coarse_plan is not None and target_valid:
                self._end_search()
                self._remember_coarse(coarse)
                self.phase = 'observe'
                if running:
                    return finish('STOP', 'reacquired')
            elif (self._inspection_cue(coarse, visible, target_valid, precise_candidate, now_ms) and
                    (precise_candidate or evidence.get('association_confirmations', 0) >=
                     self.config['tracking'].get('confirmation_frames', 3))):
                self._searching = False
                self._search_inspecting = True
                self.phase = 'visual_tracking'
                if running:
                    return finish('STOP', 'visual_tracking')
            else:
                self.phase = 'searching'
                if running:
                    if self._search_follow_active:
                        speed = int(self.options.get('search_speed_mm_s', 20))
                        speeds = (speed, -speed) if self._search_direction == 'right' else (-speed, speed)
                        if speeds != self._search_speeds:
                            self._search_speeds = speeds
                            return finish('SPEED,%d,%d' % speeds, 'searching')
                    return finish('HB')
                if self.stopped_at_ms is None:
                    return finish(status='settling')
                if self._search_reverse_required and not self._search_reverse_done:
                    self._search_reverse_done = True
                    self._search_arc_active = True
                    distance = self.options.get('reobserve_reverse_mm', 10)
                    return finish('ARC,%d,%d' % (-distance, -distance), 'searching')
                speed = int(self.options.get('search_speed_mm_s', 20))
                self._search_speeds = ((speed, -speed) if self._search_direction == 'right'
                                       else (-speed, speed))
                self._search_follow_active = True
                return finish('FOLLOW,%d,%d' % self._search_speeds, 'searching')
        evidence = (coarse or {}).get('visual_evidence') or {}
        recovery = self._near_recovery
        bottom = (evidence.get('reason') == 'clipped_blob' and
                  'bottom' in evidence.get('edge_directions', []))
        if (recovery is not None and (self.phase == 'settling' or self._adjust_due) and
                (bottom or 'bottom_view_frame_ms' in recovery)):
            self.phase = 'settling'
            if running:
                return finish('STOP', 'settling')
            if self.stopped_at_ms is None:
                return finish(status='settling')
            stamp = (coarse or {}).get('capture_timestamp_ms')
            previous = recovery.get('bottom_view_frame_ms')
            supported = (self._recovery_cue(coarse, visible, target_valid, now_ms) and
                age_ms(stamp, self.stopped_at_ms) >= 0 and
                (previous is None or age_ms(stamp, previous) > 0))
            if not supported:
                return finish(status='waiting_for_recovery_view')
            if evidence.get('reason') == 'clipped_blob':
                if not any(edge in ('left', 'right', 'bottom') for edge in
                           evidence.get('edge_directions', [])):
                    return finish(status='waiting_for_recovery_view')
                if bottom and not self._reverse_limit(recovery, state):
                    recovery['bottom_view_frame_ms'] = stamp
                    if 'reverse_start_counts' not in recovery:
                        recovery['reverse_start_counts'] = [state['encoder_a'],state['encoder_b']]
                    self.anchor = self._paired_point = self._coarse_reference = None
                    self._adjust_due = False
                    self.phase = 'recovering_view'
                    speed = int(self.options.get('recovery_reverse_mm_s', 15))
                    return finish('FOLLOW,%d,%d' % (-speed,-speed), 'recovering_view', True)
                self._adjust_due = True
            else:
                recovery.pop('bottom_view_frame_ms', None)
                self._adjust_due = False
        if self._search_inspecting:
            direction = self._search_heading(coarse, event, now_ms, decision)
            if direction is not None:
                self._search_direction = direction
            if coarse_plan is not None and target_valid:
                self._end_search()
                self._remember_coarse(coarse)
                self.phase = 'observe'
                if running:
                    return finish('STOP', 'reacquired')
            else:
                self.phase = 'visual_tracking'
                if not self._inspection_cue(coarse, visible, target_valid, precise_candidate, now_ms, False):
                    self.phase = 'observing'
                    return finish('STOP' if running else None, 'waiting_for_target')
                if (self._search_view_pending or not self._inspection_cue(
                        coarse, visible, target_valid, precise_candidate, now_ms)):
                    if running and not self._search_follow_active:
                        return finish('STOP', 'changing_view')
                    speed = int(self.options.get('search_speed_mm_s', 20))
                    speeds = (speed, -speed) if self._search_direction == 'right' else (-speed, speed)
                    if running:
                        if speeds != self._search_speeds:
                            self._search_speeds = speeds
                            return finish('SPEED,%d,%d' % speeds, 'changing_view')
                        return finish('HB', 'changing_view')
                    if self.stopped_at_ms is None:
                        return finish(status='settling')
                    self._search_speeds = speeds
                    self._search_follow_active = True
                    return finish('FOLLOW,%d,%d' % speeds, 'changing_view')
                if running:
                    return finish('STOP', 'visual_tracking')
                if self.stopped_at_ms is None:
                    return finish(status='settling')
                self._request('static', coarse, state, now_ms, decision, fine_pending)
                return finish(status='visual_tracking')
        if self._active_action == 'arc' and running:
            return finish('HB')
        if precise_candidate and not visible:
            self.phase = 'visual_tracking'
            if running:
                return finish('STOP', 'visual_tracking')
            if self.stopped_at_ms is None:
                return finish(status='settling')
            self._search_inspecting = True
            self._request('static', coarse, state, now_ms, decision, fine_pending)
            return finish(status='visual_tracking')
        if not target_valid:
            self.phase = 'observing'
            if coarse_plan is None or coarse_plan['coarse_Base_X_mm'] <= self.near['capture_Base_X_mm']:
                return finish('STOP' if running else None, 'reacquiring')
            name = 'SPEED' if running else 'FOLLOW'
            return finish('%s,%d,%d' % (name, coarse_plan['left_mm_s'],
                                       coarse_plan['right_mm_s']), 'reacquiring')
        if (self._near_recovery is None and self.anchor is None and coarse_plan is None and
                visible and not fine_pending and
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
                return finish(self._adjust(allow_reverse=coarse_plan is not None,
                                           coarse=coarse, now=now_ms))
            if not visible:
                return finish(status='waiting_for_target')
            self._request('static', coarse, state, now_ms, decision, fine_pending)
            return finish()
        if self.anchor:
            steering = measured_coarse_plan if visible else None
            if measured_coarse_plan is None and visible:
                bearing = self._coarse_direction(coarse, now_ms)
                if bearing is not None:
                    steering = {'predicted_bearing_deg': bearing}
            plan = self._near_plan(steering, now_ms)
            decision['plan'] = plan
            if plan['status'] != 'follow':
                self.phase = 'settling'
                return finish('STOP' if running else None, 'settling')
            self.phase = 'following'
            name = 'SPEED' if running else 'FOLLOW'
            return finish('%s,%d,%d' % (name, plan['left_mm_s'], plan['right_mm_s']))
        if coarse_plan is None:
            if visible and evidence.get('distance_valid') is True:
                self.phase = 'observing'
                return finish(status='confirming_target')
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
        if coarse_plan['status'] != 'follow':
            self.phase = 'settling'
            return finish('STOP' if running else None, 'waiting_for_target')
        capture_distance = self.near['capture_Base_X_mm']
        if (measured_coarse_plan is not None and visible and
                coarse_plan['coarse_Base_X_mm'] <= capture_distance and
                (coarse or {}).get('frame_id') != self._moving_retry_frame):
            if not running:
                if self.stopped_at_ms is None:
                    return finish(status='settling')
                self._request('static', coarse, state, now_ms, decision, fine_pending)
                return finish()
            self._request('moving', coarse, state, now_ms, decision, fine_pending)
        self.phase = 'fine_moving' if self._fine_request else 'following'
        name = 'SPEED' if running else 'FOLLOW'
        return finish('%s,%d,%d' % (name, coarse_plan['left_mm_s'], coarse_plan['right_mm_s']))
