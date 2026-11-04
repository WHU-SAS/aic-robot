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
        self._bearing_only = False
        self._motor = self._previous_motor = None
        self._memory_odom_capture = None
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
        self._bottom_reverse_budget = None
        self._bottom_static_pending = False
        self._search_inspecting = False
        self._search_arc_active = False
        self._search_follow_active = False
        self._search_speeds = None
        self.corrections = 0
        self.fine_requests = 0
        self.odometry = Odometry(config['mm_per_count'])

    def cancel_fine(self):
        """The owner cancelled the job; no result or timestamp is fabricated."""
        self._fine_request = None

    def resume_motion(self):
        """Explicit owner acknowledgement of an existing button/link stop."""
        self._motion_hold = None

    def retarget_following(self, target_version=None, pending_follow=False):
        """Retain owned FOLLOW; pending_follow is an unsent decision rollback."""
        owned = (self._active_action == 'follow' or
                 pending_follow and self._active_action is None)
        if (not owned or self._motor is None or
                not self._motor.get('run') or self._motion_hold):
            return False
        self._changed_target(self.version if target_version is None else target_version)
        self._active_action = 'follow'
        self.phase = 'following'
        self._moving_started = True
        return True

    def restart_observation(self, discard_reference=False):
        """After arm recovery, demand a NEW stopped observation of this target."""
        self.anchor = self.parking = self._paired_point = None
        self._coarse_reference = None
        self._bearing_only = False
        self._parking_reference = self._parking_ready_ms = None
        self._fine_request = None
        self._search_inspecting = False
        self._awaiting_query_identity = False
        if discard_reference:
            self._near_recovery = self.paired_coarse = None
        self._moving_started = False
        self.phase = 'settling'

    def request_search(self, direction=None, bottom_history=False):
        if self._searching and not (direction == 'bottom' and bottom_history and
                                   not self._search_reverse_required):
            return
        self._retain_reverse_budget()
        self._searching = direction != 'top'
        self._search_direction = direction or self._search_direction or 'left'
        self._search_reverse_required = direction == 'bottom' and (
            bottom_history or self._bottom_reverse_budget is not None)
        self._search_reverse_done = False
        self._bottom_static_pending = False
        if self._search_reverse_required and self._bottom_reverse_budget is None:
            self._bottom_reverse_budget = {}
        if self._search_reverse_required:
            self._bottom_reverse_budget.pop('view_recovered',None)
        self._search_inspecting = False
        self._search_arc_active = False
        self._search_follow_active = False
        self._search_speeds = None
        self._search_hint = None
        self.anchor = self.parking = self._paired_point = None
        self._coarse_reference = None
        self._near_recovery = None
        self._awaiting_query_identity = False
        self.cancel_fine()
        self._last_precise = None

    def _changed_target(self, version):
        self._retain_reverse_budget()
        searching = self._searching
        self.version = version
        self.anchor = self.paired_coarse = self.parking = self._paired_point = None
        self._parking_reference = self._parking_ready_ms = None
        self._fine_request = self._last_precise = None
        self._coarse_reference = self._moving_retry_frame = None
        self._bearing_only = False
        self._near_recovery = None
        self._awaiting_query_identity = False
        self._moving_started = False
        self._searching = searching
        self._search_inspecting = False
        self._search_hint = None
        self.phase = 'observe'

    def _end_search(self):
        self._searching = self._search_inspecting = False
        self._search_arc_active = False
        self._search_follow_active = False
        self._search_speeds = None
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
            if self._search_region is not None:
                self._search_region = self._presence_reference(self.odometry.rows[-1][0])
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
        if self._memory_odom_capture is not None:
            stamps.append(self._memory_odom_capture)
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
            if plan['status'] != 'follow':
                return None
            point = predict_center(coarse['ground_contact_proxy_Base_mm'],
                self.odometry.since(coarse['capture_timestamp_ms'],now),
                self.near['track_width_mm'],self.projection.normal,
                self.projection.forward_axis_Base,self.projection.motion_origin_Base_mm)
            plan['remaining_mm'] = max(0,(point[0]-target_x(self.config['workspace']))/
                                      forward_x_per_mm(self.projection))
            return plan
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

    def _presence_reference(self, stamp):
        """Propagate only the existing target region, without retaining old IO rows."""
        if self._search_region is None:return None
        reference=dict(self._search_region)
        point,previous=reference.get('point'),reference.get('motor_stamp_ms')
        if point is None or type(previous) is not int or type(stamp) is not int:
            return reference
        if age_ms(stamp,previous)>=0:
            deltas=self.odometry.since(previous,stamp)
        else:
            deltas=[(-a,-b)for a,b in reversed(self.odometry.since(stamp,previous))]
        point=predict_center(point,deltas,self.near['track_width_mm'],
            self.projection.normal,self.projection.forward_axis_Base,
            self.projection.motion_origin_Base_mm)
        reference.update(point=point,motor_stamp_ms=stamp)
        try:reference['center_pixel']=self.projection.base_to_pixel(point)
        except ValueError:pass
        return reference

    def _visual_presence(self, coarse, now):
        """A current target-sized region can hold observation; stray blue cannot."""
        if not isinstance(coarse,dict):return False
        stamp=coarse.get('capture_timestamp_ms')
        if type(stamp) is not int or not 0 <= age_ms(now,stamp) <= self.config['tracking']['max_age_ms']:
            return False
        evidence=coarse.get('visual_evidence') or {}
        reference=self._presence_reference(stamp)
        # Without target history, only a native selected observation can hold
        # a static query. An arbitrary unselected invalid pool is not a target.
        pool=list(coarse.get('visual_candidates') or []) if reference is not None else []
        if evidence.get('visible'):pool.append(evidence)
        return any(isinstance(cue,dict) and cue.get('visible') is not False and
            cue.get('timestamp_ms',stamp)==stamp and self._search_shape(cue) and
            (reference is None or self._search_match(cue,reference)) for cue in pool)

    def _remember_coarse(self, coarse):
        self._remember_search_region(coarse)
        try:
            self._coarse_reference = {
                'capture_timestamp_ms': coarse['capture_timestamp_ms'],
                'point': list(coarse['ground_contact_proxy_Base_mm']),
                'ray': self._pixel_point(coarse)}
        except (ValueError, TypeError, KeyError):
            pass

    def _remember_search_region(self, coarse, precise_pair=False, reacquired=False,
                                measured_query=False):
        """Keep full-target size for direction hints, never a distance anchor."""
        if not isinstance(coarse, dict):
            return
        evidence = (coarse or {}).get('visual_evidence') or coarse or {}
        if (evidence.get('ambiguous') or evidence.get('reason') == 'clipped_blob' or
                evidence.get('associated') is False and not measured_query or
                (not precise_pair and (evidence.get('distance_valid') is not True or
                 not measured_query and evidence.get('navigation_confirmed') is not True))):
            return
        if self._search_shape(evidence):
            stamp=coarse.get('capture_timestamp_ms')
            reference=self._presence_reference(stamp)
            if not precise_pair and not reacquired and reference is not None:
                previous=reference.get('seen_capture_ms')
                if (not self._search_match(evidence,reference) or
                        type(previous) is int and type(stamp) is int and
                        age_ms(stamp,previous)<=0):
                    return
            self._search_region = {'bbox_px':list(evidence['bbox_px']),
                                   'center_pixel':list(evidence['center_pixel'])}
            if type(stamp) is int:
                height=10.
                if precise_pair and self.anchor is not None:
                    unused,point,unused=distance_anchor(self.anchor)
                    height=sum(a*b for a,b in zip(self.projection.normal,point))-self.projection.offset
                try:
                    self._search_region.update(point=self.projection.pixel_to_plane(
                        evidence['center_pixel'],height),motor_stamp_ms=stamp,seen_capture_ms=stamp)
                    if self.odometry.rows and age_ms(self.odometry.rows[-1][0],stamp)>=0:
                        self._search_region=self._presence_reference(self.odometry.rows[-1][0])
                except ValueError:pass

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

    def _reacquired_presence(self, coarse, now):
        """Confirmed navigation may reacquire the target at a new image position."""
        evidence=(coarse or {}).get('visual_evidence') or {}
        stamp=(coarse or {}).get('capture_timestamp_ms')
        if (type(stamp) is not int or not 0<=age_ms(now,stamp)<=self.config['tracking']['max_age_ms'] or
                evidence.get('associated') is not True or evidence.get('ambiguous') or
                evidence.get('navigation_confirmed') is not True or
                evidence.get('distance_valid') is not True or not self._search_shape(evidence)):
            return False
        tracking=self.config['tracking']
        pixels=evidence.get('pixels')
        return (evidence['bbox_px'][2]*evidence['bbox_px'][3]>=tracking.get('min_area_px2',0) and
            (pixels is None or pixels>=tracking.get('min_pixels',0)))

    def _measurable_presence(self, coarse, now):
        """A complete native measurement is visible before identity confirms."""
        stamp=(coarse or {}).get('capture_timestamp_ms')
        tracking=self.config['tracking']
        if type(stamp) is not int or not 0<=age_ms(now,stamp)<=tracking['max_age_ms']:
            return False
        evidence=(coarse or {}).get('visual_evidence') or {}
        pool=list((coarse or {}).get('visual_candidates') or [])
        if evidence.get('visible'):pool.append(evidence)
        return any(isinstance(cue,dict) and cue.get('visible') is not False and
            cue.get('timestamp_ms',stamp)==stamp and cue.get('distance_valid') is True and
            not cue.get('edge_directions') and self._search_shape(cue) and
            cue['bbox_px'][2]*cue['bbox_px'][3]>=tracking.get('min_area_px2',0) and
            (cue.get('pixels') is None or cue['pixels']>=tracking.get('min_pixels',0))
            for cue in pool)

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
        if event.get('anchor_valid') and self._search_region is None:
            self._remember_search_region(event.get('paired_coarse'), precise_pair=True)
        self._remember_search_region(coarse)
        evidence = coarse.get('visual_evidence') or {}
        raw = coarse.get('visual_candidates')
        raw = raw if isinstance(raw, list) else [evidence]
        fresh = [cue for cue in raw if isinstance(cue, dict) and
                 cue.get('timestamp_ms', stamp) == stamp and self._search_shape(cue)]
        reference=self._presence_reference(stamp)
        if reference is not None:
            fresh=[cue for cue in fresh if self._search_match(cue,reference)]

        def independent(cues):
            def contains(outer, inner):
                x,y,w,h = outer['bbox_px']; a,b,c,d = inner['bbox_px']
                return x<a and y<b and x+w>a+c and y+h>b+d
            return [cue for cue in cues if not any(
                other is not cue and contains(other, cue) for other in cues)]

        canonical = (evidence.get('visible') and evidence.get('associated') is True and
            not evidence.get('ambiguous') and self._visible(coarse, event) and
            (reference is None or self._search_match(evidence,reference)) and
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
                    (reference is None or self._search_match(evidence,reference)) and
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

    def _inspection_cue(self, coarse, visible, target_valid, precise_candidate, now):
        """Current appearance can request a static query without a range."""
        evidence = (coarse or {}).get('visual_evidence') or {}
        stamp = (coarse or {}).get('capture_timestamp_ms')
        if (type(stamp) is not int or not
                0 <= age_ms(now, stamp) <= self.config['tracking']['max_age_ms'] or
                not self._visual_presence(coarse,now) or
                not (precise_candidate or visible and target_valid and self._search_shape(evidence))):
            return False
        if evidence.get('reason') == 'clipped_blob':
            return (self._search_weak_shape(evidence, evidence.get('associated') is True) and
                abs(evidence['center_pixel'][0]-
                 self.config['tracking']['image_size_px'][0]/2) <=
                    self.config['coarse_center_tolerance_px'])
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
        remaining = max(0, (point[0]-destination)/forward_x_per_mm(self.projection))
        targets = wheel_targets(bearing, remaining, self.near, self.near['track_width_mm'])
        return {'status': 'follow' if coast[0] > destination else 'stop_for_observation',
            'mode': 'coarse_prediction', 'heading_source': source,
            'capture_timestamp_ms': reference['capture_timestamp_ms'],
            'coarse_Base_X_mm': point[0], 'predicted_bearing_deg': bearing,
            'remaining_mm': remaining,
            'left_mm_s': int(targets['left_mm_s']+.5),
            'right_mm_s': int(targets['right_mm_s']+.5)}

    def _follow_command(self, name, plan):
        command = '%s,%d,%d' % (name,plan['left_mm_s'],plan['right_mm_s'])
        if 'remaining_mm' in plan:
            # This is uncoasted nominal forward travel. The paired MCU applies
            # the existing speed curve/coast every10ms while Python computes.
            command += ',%d' % int(max(0,plan['remaining_mm'])+.5)
        return command

    def bounded_follow_completed(self, state, now):
        """Latch an ACKed owned endpoint; a moving image stays historical."""
        self._observe_motor(state,now)
        request = self._fine_request
        if request is not None and request['purpose']=='moving':
            request['stopped_for_verification'] = True
            self.anchor = self.parking = self._paired_point = None
            self.phase = 'settling'
        else:
            self.restart_observation()
        self._active_action = None

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

    def pending_fine_motion(self, motor_state, now_ms):
        """Propagate an owned approach while its original fine image computes.

        No new visual authority is invented here. Existing distance/coast and
        speed profiles still bound the remaining travel between camera frames.
        The owner sends this intention instead of its ordinary heartbeat.
        """
        request = self._fine_request
        if (request is not None and request['purpose']=='moving' and
                request.get('stopped_for_verification') and motor_state.get('run')):
            return dict(phase='settling',status='fine_distance_exhausted',command='STOP')
        if (request is None or request['purpose']!='moving' or
                self._active_action!='follow' or not motor_state.get('run') or
                self._motion_hold or self._searching or self._search_inspecting):
            return None
        self._observe_motor(motor_state,now_ms)
        if motor_state.get('result',0) not in (0,1):
            self._motion_hold = 'motor_reply_rejected'
            return dict(phase=self.phase,status='motion_wait',command='STOP',
                        motion_wait_reason=self._motion_hold)
        plan = (self._near_plan(None,now_ms) if self.anchor is not None else
                self._coarse_prediction(None,now_ms,False))
        if plan is not None:
            if plan['status']!='follow':
                request['stopped_for_verification'] = True
                self._active_action = None
                self.phase = 'settling'
                return dict(phase=self.phase,status='fine_distance_exhausted',
                            command='STOP',plan=plan)
            return dict(phase='fine_moving',status='localizing_moving',plan=plan,
                command=self._follow_command('SPEED',plan))
        # A current direction without trustworthy range cannot supply forward
        # travel. Retain the original bearing-only, zero-mean pointing mode.
        bearing = (self._coarse_direction(request.get('coarse'),now_ms)
                   if self._bearing_only else None)
        turn = (int(max(-self.options.get('search_speed_mm_s',20),
            min(self.options.get('search_speed_mm_s',20),
                self.near['heading_gain_per_s']*math.radians(bearing)*
                self.near['track_width_mm']/2))) if bearing is not None else 0)
        return dict(phase='fine_moving',status='localizing_without_range',
                    command='SPEED,%d,%d' % (-turn,turn))

    def _request(self, purpose, coarse, state, now, decision, pending):
        self.phase = 'fine_'+purpose
        if self._fine_request is None:
            if purpose == 'static' and not self._searching:
                self._remember_near(now)
            self._fine_request = {'purpose': purpose, 'motor': dict(state),
                'coarse':coarse,
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

    def _matching_precise_request(self, complete, now):
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
        return request

    def _return_to_coarse(self, coarse, decision):
        """An explicitly distant current target leaves the near-fine cycle."""
        self.anchor = self.paired_coarse = self.parking = self._paired_point = None
        self._coarse_reference = self._near_recovery = None
        self._parking_reference = self._parking_ready_ms = None
        self._fine_request = self._last_precise = self._moving_retry_frame = None
        self._awaiting_query_identity = self._bearing_only = False
        self._moving_started = False
        self._end_search()
        self._remember_coarse(coarse)
        self.phase = 'observe'
        decision['return_to_coarse'] = True

    def _accept(self, complete, state, now, coarse_plan, decision, coarse):
        request = self._matching_precise_request(complete, now)
        if request is None:
            return None
        observation = complete['observation']
        purpose = complete['purpose']
        paired = complete['paired_coarse']
        ready = complete.get('result_ready_ms', now)
        key = (self.version, observation.get('frame_id'), purpose)
        verdict = None
        if purpose == 'static':
            stopped = request.get('stopped_at_ms')
            stationary = (stopped is not None and self._static_ok and
                          complete.get('stationary_since_capture', True))
            verdict = verify_parked(observation, self.projection, self.config['workspace'], self.near,
                now, stopped if stopped is not None else now,
                complete.get('result_ready_ms', now), stationary_since_capture=stationary)
            decision['static_verification'] = verdict
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
            if request.get('stopped_for_verification'):
                # A moving frame taken before this stop remains historical.
                # Demand the normal NEW stopped verification before moving on.
                decision['moving_fine_stopped'] = True
                self.phase = 'settling'
                return None
            try:
                self._bind_anchor(observation, paired)
                self.phase = 'following'
            except (ValueError, TypeError):
                self.phase = 'following'
                self._moving_retry_frame = decision.get('coarse_frame_id')
            return None
        if verdict['status'] in ('parked', 'not_parked'):
            self._bottom_reverse_budget = None
            self._bottom_static_pending = self._search_reverse_required = False
            self._search_reverse_done = False
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
            if (decision.get('coarse_far') and observation['cube_center_Base_mm'][0] >
                    self.near['capture_Base_X_mm']):
                self._return_to_coarse(coarse,decision)
                return None
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
            decision['correction_plan'] = correction
            if correction.get('status') == 'arc':
                decision['correction_reference'] = dict(encoder_a=state['encoder_a'],
                    encoder_b=state['encoder_b'],motor_ms=state.get('ms'),
                    received_ms=state.get('_k230_received_ms'))
                self.corrections += 1
                self.phase = 'correcting'
                return correction['command']
        # Failed recognition grants no motion. Clear historical destinations
        # so the next stopped image cannot be bypassed by an old moving point.
        self.anchor = self._paired_point = self._coarse_reference = None
        self._parking_reference = self._parking_ready_ms = None
        self._bearing_only = False
        self._end_search()
        self.phase = 'settling'
        return None

    def consume_moving_precise(self, complete, motor_state, now_ms, target_version):
        """Apply a same-frame moving result before taking another coarse image.

        The ordinary update keeps the existing request, motor-stop and finite
        endpoint rules. With no current coarse input its first endpoint comes
        only from the accepted precise pair and the latest encoder feedback.
        """
        if (not isinstance(complete, dict) or complete.get('purpose') != 'moving' or
                complete.get('target_version') != target_version or
                self.version != target_version or
                self._matching_precise_request(complete, now_ms) is None):
            return None
        if not motor_state.get('run'):
            # Whether the MCU endpoint or the owner stopped it, this exposure
            # was made in motion and cannot authorize another moving segment.
            self._fine_request['stopped_for_verification'] = True
        return self.update(None, motor_state, now_ms, precise=complete,
            target_version=target_version, target_event={'target_valid': True})

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
        self._retain_reverse_budget()
        start = reference.get('reverse_start_counts')
        if start is None:
            return reference.get('reverse_exhausted', False)
        travel = max(abs(count_delta(start[index], state[key]))*self.config['mm_per_count']
                     for index,key in enumerate(('encoder_a', 'encoder_b')))
        speeds = self._speeds()
        coast = max(abs(speeds['left_mm_s']),abs(speeds['right_mm_s']))*self.near['coast_s']
        if travel+coast >= 100:
            reference['reverse_exhausted'] = True
            if self._bottom_reverse_budget is not None:
                self._bottom_reverse_budget['reverse_exhausted'] = True
        return reference.get('reverse_exhausted', False)

    def _retain_reverse_budget(self):
        """Carry only physical wheel travel across target/search bookkeeping."""
        reference = self._near_recovery
        if reference is None:
            return
        if self._bottom_reverse_budget is None and 'reverse_start_counts' in reference:
            self._bottom_reverse_budget = {key:reference[key] for key in
                ('reverse_start_counts','reverse_exhausted') if key in reference}
        elif self._bottom_reverse_budget is not None:
            if 'reverse_start_counts' in self._bottom_reverse_budget:
                reference['reverse_start_counts'] = self._bottom_reverse_budget['reverse_start_counts']
            if reference.get('reverse_exhausted') or self._bottom_reverse_budget.get('reverse_exhausted'):
                reference['reverse_exhausted'] = self._bottom_reverse_budget['reverse_exhausted'] = True

    def _bottom_full_view(self, coarse, event, now):
        evidence = (coarse or {}).get('visual_evidence') or {}
        if (evidence.get('visible') is not True or evidence.get('ambiguous') or
                evidence.get('reason') == 'clipped_blob' or
                evidence.get('edge_directions') or not self._search_shape(evidence)):
            return False
        stamp = (coarse or {}).get('capture_timestamp_ms')
        # A whole-looking foreign blue patch is not the departed target.
        # Continuity can keep a weak target queryable. A genuinely measured
        # complete candidate can also stop recovery before native navigation
        # confirmation; only NEW static fine may then authorize a pose.
        tracking=self.config['tracking']
        pixels=evidence.get('pixels')
        measurable=(evidence.get('distance_valid') is True and
            evidence['bbox_px'][2]*evidence['bbox_px'][3]>=tracking.get('min_area_px2',0) and
            (pixels is None or pixels>=tracking.get('min_pixels',0)))
        return (type(stamp) is int and evidence.get('timestamp_ms',stamp) == stamp and
            0 <= age_ms(now,stamp) <= tracking['max_age_ms'] and
            (measurable or self._search_region is not None and
             self._visual_presence(coarse,now)))

    def update(self, coarse, motor_state, now_ms, precise=None, target_version=0,
               fine_pending=False, target_event=None, arm_active=False):
        event = target_event or {}
        coarse = self._canonical_coarse(coarse)
        stamp = (coarse or {}).get('capture_timestamp_ms')
        fresh = type(stamp) is int and 0 <= age_ms(now_ms,stamp) <= self.config['tracking']['max_age_ms']
        owned_follow = bool(motor_state['run'] and self._active_action == 'follow' and not arm_active)
        cue = event.get('navigation_cue')
        adopted = (owned_follow and fresh and event.get('tracking_update') and
            isinstance(cue,dict) and isinstance(coarse,dict) and
            cue.get('bbox_px') == (coarse.get('visual_evidence') or {}).get('bbox_px') and
            cue.get('center_pixel') == (coarse.get('visual_evidence') or {}).get('center_pixel') and
            cue.get('timestamp_ms',coarse.get('capture_timestamp_ms')) == coarse.get('capture_timestamp_ms'))
        if adopted:
            # Control may follow a newly selected cue without claiming native
            # old-object association. The input observation remains unchanged.
            control = dict(cue,visible=True,associated=True,ambiguous=False,
                navigation_confirmed=cue.get('distance_valid') is True)
            coarse = self._canonical_coarse(dict(coarse,visual_evidence=control))
        decision = {'phase': self.phase, 'status': 'running', 'command': None,
                    'request_fine': None, 'allow_arm': False, 'parking': None,
                    'reset_tracker': False,
                    'target_version': target_version, 'precise_consumed': False,
                    'precise_deferred': False,
                    'coarse_frame_id': (coarse or {}).get('frame_id')}
        confirmed_moved = bool(event.get('confirmed_moved') or
            event.get('event') == 'moved' and event.get('movement_state') == 'confirmed')
        anchor_locked = (self.anchor is not None and not confirmed_moved and
            not event.get('loss_confirmed') and
            not self._searching and not self._search_inspecting and
            self._active_action not in ('arc', 'recovery_reverse', 'bottom_reverse',
                                        'inspection_follow', 'search_follow'))
        changed = self.version != target_version
        initial_busy = self._motor is None and motor_state['run']
        self._observe_motor(motor_state, now_ms)
        state, running = motor_state, bool(motor_state['run'])
        visible = self._visible(coarse, event)
        presence = self._visual_presence(coarse,now_ms)
        reacquired_presence = self._reacquired_presence(coarse,now_ms)
        if (reacquired_presence and (event.get('confirmed_moved') or
                event.get('event') in ('acquired','reappeared','tracking_update'))):
            self._remember_search_region(coarse,reacquired=True)
            presence=True
        precise_candidate = bool(event.get('precise_candidate'))
        query_visible = visible or precise_candidate
        if (anchor_locked and self.phase in ('settling','visual_tracking','fine_static') and not running and
                self.stopped_at_ms is not None and not presence and
                (event.get('event') == 'exited' or event.get('search'))):
            # Its finite approach and original stable-stop window are over.
            # Explicit current loss may now use the existing search policy;
            # the historical moving point grants no new forward segment.
            anchor_locked = False
        if anchor_locked:
            # A coarse version/identity gap is not a new physical destination.
            # Keep the precise point; new stopped jobs still use today's epoch.
            self.version = target_version
        elif changed and not (owned_follow and fresh and visible and self.retarget_following(target_version)):
            self._changed_target(target_version)
        elif confirmed_moved and self.anchor is not None:
            self._changed_target(target_version)
        if adopted:
            decision['reset_tracker'] = True
        bottom_exit = event.get('event') == 'exited' and event.get('last_disappearance_direction') == 'bottom'
        if (not anchor_locked and (not presence or bottom_exit) and
                (event.get('event') == 'exited' or event.get('search') and not presence)):
            direction = event.get('last_disappearance_direction')
            self.request_search(direction,bottom_history=
                direction == 'bottom' and event.get('event') == 'exited')
        target_valid = event.get('target_valid', True)
        invalidated_view = bool(not target_valid and self._near_recovery is not None and
                                'bottom_view_frame_ms' in self._near_recovery)
        measured_coarse_plan = self._coarse_plan(coarse, now_ms)
        coarse_plan = measured_coarse_plan if visible else None
        if coarse_plan is not None:
            self._bearing_only = False
        if coarse_plan is not None and target_valid and not self._searching:
            self._remember_coarse(coarse)
        elif (running and self._active_action == 'follow' and target_valid and
                not self._searching):
            coarse_plan = self._coarse_prediction(coarse, now_ms, visible)
        decision['target_visible'] = visible
        decision['precise_candidate'] = precise_candidate
        decision['query_visible'] = query_visible
        decision['coarse_far'] = bool(fresh and visible and target_valid and
            (coarse.get('visual_evidence') or {}).get('navigation_confirmed') is True and
            measured_coarse_plan is not None and measured_coarse_plan['coarse_Base_X_mm'] >
            self.near['capture_Base_X_mm'])
        if not target_valid and not anchor_locked:
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

        def finish(command=None, status=None, recovery_reverse=False, bottom_reverse=False):
            if command:
                decision['command'] = command
                name = command.split(',')[0]
                if name in ('FOLLOW', 'SPEED'):
                    if bottom_reverse:
                        self._active_action = 'bottom_reverse'
                    elif recovery_reverse or self._active_action == 'recovery_reverse':
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
            elif self._active_action in ('search_follow', 'inspection_follow', 'recovery_reverse', 'bottom_reverse'):
                # Signed FOLLOW retains the MCU's existing segment deadline.
                # Reobserve after its reason-3 stop, then start a new segment.
                self._search_follow_active = False
                self._search_speeds = None
                if self._active_action in ('recovery_reverse','bottom_reverse'):
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
            if self.phase == 'correcting':
                self.anchor = self._paired_point = None
                self.phase = 'settling'
            elif self.phase == 'searching':
                self.phase = 'observing'
        if self._motion_hold:
            decision['motion_wait_reason'] = self._motion_hold
            return finish('STOP' if running else None, 'motion_wait')
        if (self._bottom_static_pending or
                (self._bottom_reverse_budget or {}).get('view_recovered') and
                not self._bottom_full_view(coarse,event,now_ms)):
            self.phase = 'settling'
            if running:
                return finish('STOP','recovering_bottom_view')
            if self.stopped_at_ms is None:
                return finish(status='settling')
            if self._bottom_full_view(coarse,event,now_ms):
                if age_ms(coarse['capture_timestamp_ms'],self.stopped_at_ms) < 0:
                    return finish(status='waiting_for_post_stop_view')
                self._bottom_static_pending = False
                self._request('static',coarse,state,now_ms,decision,fine_pending)
                return finish(status='recovering_bottom_view')
            # Whole visibility must still hold after the static solve. Losing
            # it during computation revokes that image, not the wheel budget.
            self.restart_observation(discard_reference=True)
            self._bottom_static_pending = self._search_reverse_done = False
            self._search_reverse_required = self._searching = True
            self._bottom_reverse_budget.pop('view_recovered',None)
        if (self._search_reverse_required and
                not (self._bottom_reverse_budget or {}).get('view_recovered') and
                self._bottom_full_view(coarse,event,now_ms)):
            # This new measured region owns only presence during its stopped
            # query. No navigation/fine distance capability is granted here.
            self._remember_search_region(coarse,reacquired=True,measured_query=True)
            self._search_reverse_done = self._bottom_static_pending = True
            self._bottom_reverse_budget['view_recovered'] = True
            self._end_search()
            self.restart_observation(discard_reference=True)
            if (coarse.get('visual_evidence') or {}).get('associated') is not True:
                decision['reset_tracker'] = True
            return finish('STOP' if running else None,'recovering_bottom_view')
        if (self._search_reverse_required and
                (self._bottom_reverse_budget or {}).get('reverse_exhausted') and
                not self._bottom_reverse_budget.get('view_recovered')):
            self.phase = 'observing'
            return finish('STOP' if running else None,'bottom_reverse_limit')
        if self._search_reverse_required and not self._search_reverse_done:
            self.phase = 'recovering_bottom_view'
            reference = self._bottom_reverse_budget
            if reference is None or self._reverse_limit(reference,state):
                self._search_reverse_done = True
                return finish('STOP' if running else None,'bottom_reverse_limit')
            if running:
                if self._active_action not in ('bottom_reverse','recovery_reverse'):
                    return finish('STOP','starting_bottom_recovery')
                self._active_action = 'bottom_reverse'
                return finish('HB','recovering_bottom_view')
            if self.stopped_at_ms is None:
                return finish(status='settling')
            if 'reverse_start_counts' not in reference:
                reference['reverse_start_counts'] = [state['encoder_a'],state['encoder_b']]
            speed = int(self.options.get('recovery_reverse_mm_s',15))
            self._search_follow_active = False
            return finish('FOLLOW,%d,%d' % (-speed,-speed),'recovering_bottom_view',bottom_reverse=True)
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
        # ARC cannot be retargeted by SPEED. Owned FOLLOW can smoothly adopt
        # a current visible target even inside the fine region.
        if changed and running and self._active_action in ('arc', 'inspection_follow'):
            return finish('STOP', 'target_changed')
        # A version change is processed above before accepting a completed job.
        if precise is not None and (target_valid or
                (precise_candidate and precise.get('purpose') == 'static')):
            command = self._accept(precise, state, now_ms, coarse_plan, decision, coarse)
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
        if (self.anchor is not None and
                self._active_action != 'arc' and not self._search_inspecting):
            # A successful fine measurement owns distance until the car stops.
            # Current coarse evidence may steer it, but loss/ambiguity/range
            # noise cannot replace that endpoint or start a search on the way.
            if self.phase == 'settling':
                # The previous coast-aware STOP owns the next static image.
                # Once run=0 its speed estimate becomes zero; recomputing the
                # old moving point must not restart its last few millimetres.
                if running:
                    return finish('STOP', 'settling')
                if self.stopped_at_ms is None:
                    return finish(status='settling')
                if query_visible:
                    self._request('static', coarse, state, now_ms, decision, fine_pending)
                    return finish()
                return finish(status='waiting_for_target')
            steering = measured_coarse_plan if visible and fresh else None
            if steering is None and visible and fresh:
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
            return finish(self._follow_command(name, plan))
        if self._searching:
            evidence = (coarse or {}).get('visual_evidence') or {}
            direction = self._search_heading(coarse, event, now_ms, decision)
            if direction is not None:
                self._search_direction = direction
            if running and not (self._search_arc_active or self._search_follow_active):
                self.phase = 'searching'
                return finish('STOP', 'starting_search')
            if coarse_plan is not None and target_valid and (presence or reacquired_presence):
                self._end_search()
                self._remember_search_region(coarse,reacquired=True)
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
            elif presence and direction is None:
                # Pixels are present even if range/association is unclear.
                # There is no current direction that could justify turning.
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
                speed = int(self.options.get('search_speed_mm_s', 20))
                self._search_speeds = ((speed, -speed) if self._search_direction == 'right'
                                       else (-speed, speed))
                self._search_follow_active = True
                return finish('FOLLOW,%d,%d' % self._search_speeds, 'searching')
        evidence = (coarse or {}).get('visual_evidence') or {}
        recovery = self._near_recovery
        bottom = (evidence.get('reason') == 'clipped_blob' and
                  'bottom' in evidence.get('edge_directions', []))
        if (recovery is not None and self.phase == 'settling' and
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
                    self._retain_reverse_budget()
                    self._search_reverse_required = True
                    self._search_reverse_done = False
                    self.anchor = self._paired_point = self._coarse_reference = None
                    self.phase = 'recovering_view'
                    speed = int(self.options.get('recovery_reverse_mm_s', 15))
                    return finish('FOLLOW,%d,%d' % (-speed,-speed), 'recovering_view', True)
                if bottom and recovery.get('reverse_exhausted'):
                    self._search_reverse_required = self._search_reverse_done = True
                    self.phase = 'observing'
                    return finish(status='bottom_reverse_limit')
            else:
                recovery.pop('bottom_view_frame_ms', None)
        if self._search_inspecting:
            # This is initial visual reacquisition, not a failed fine retry:
            # failed static results leave this state via _end_search above.
            if coarse_plan is not None and target_valid and (presence or reacquired_presence):
                self._end_search()
                self._remember_search_region(coarse,reacquired=True)
                self._remember_coarse(coarse)
                self.phase = 'observe'
                if running:
                    return finish('STOP', 'reacquired')
            else:
                self.phase = 'visual_tracking'
                if running:
                    return finish('STOP', 'visual_tracking')
                if self.stopped_at_ms is None:
                    return finish(status='settling')
                if self._inspection_cue(coarse,visible,target_valid,precise_candidate,now_ms):
                    self._request('static', coarse, state, now_ms, decision, fine_pending)
                return finish(status='visual_tracking' if presence else 'waiting_for_target')
        if self._active_action == 'arc' and running:
            return finish('HB')
        if (self.phase == 'settling' and self.anchor is None and
                decision['coarse_far']):
            # No fine endpoint is being replaced: current confirmed far
            # geometry can resume ordinary coarse approach instead of asking
            # the near-only fine solver for the same distant view forever.
            self._return_to_coarse(coarse,decision)
        if self.phase == 'settling':
            # A failed stopped measurement must be replaced while stopped,
            # before coarse range or visible centering can start another move.
            if running:
                return finish('STOP', 'settling')
            if self.stopped_at_ms is None:
                return finish(status='settling')
            if not query_visible:
                return finish(status='waiting_for_target')
            self._request('static', coarse, state, now_ms, decision, fine_pending)
            return finish()
        if (owned_follow and fresh and visible and target_valid and self.anchor is None and
                coarse_plan is None and (adopted or self._bearing_only)):
            # A new pixel direction supplies no new forward endpoint. Continue
            # turning toward it while measuring, with zero mean translation.
            bearing = self._coarse_direction(coarse, now_ms)
            if bearing is not None:
                self._bearing_only = True
                turn = int(max(-self.options.get('search_speed_mm_s',20),
                    min(self.options.get('search_speed_mm_s',20),
                        self.near['heading_gain_per_s']*math.radians(bearing)*
                        self.near['track_width_mm']/2)))
                if coarse.get('frame_id') != self._moving_retry_frame:
                    self._request('moving',coarse,state,now_ms,decision,fine_pending)
                self.phase = 'fine_moving' if self._fine_request else 'following'
                return finish('SPEED,%d,%d' % (-turn,turn),'tracking_bearing_only')
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
            return finish(self._follow_command(name,coarse_plan), 'reacquiring')
        if (self._near_recovery is None and self.anchor is None and coarse_plan is None and
                visible and not fine_pending and
                evidence.get('reason') == 'clipped_blob'):
            direction = self._search_heading(coarse,event,now_ms,decision)
            if direction is not None:
                self.request_search(direction)
                self.phase = 'searching'
                return finish('STOP' if running else None, 'centering_clipped_target')
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
            return finish(self._follow_command(name,plan))
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
        if (owned_follow and measured_coarse_plan is not None and visible and
                coarse_plan['coarse_Base_X_mm'] <= capture_distance):
            # A retry/current near cue may already be at the destination.
            # Bound THIS command before the owner enters blocking JPEG work.
            bounded = self._coarse_prediction(coarse,now_ms,True)
            if bounded is not None:
                coarse_plan = bounded
                decision['plan'] = bounded
                if bounded['status']!='follow':
                    self.cancel_fine()
                    self.phase = 'settling'
                    return finish('STOP','fine_distance_exhausted')
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
        return finish(self._follow_command(name,coarse_plan))
