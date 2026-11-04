"""Small target history for cooperative tracking; no camera, IO or task policy.

Coarse contact proxies are compared with THEIR encoder-propagated reference,
never with a fine geometric centre. Thresholds are offline starting candidates,
not measured movement-detection accuracy. Unknown vision is not target motion.
"""
import math
try:
    from .blue_tracker import age_ms
    from .near_control import predict_center, distance_anchor
except ImportError:
    from blue_tracker import age_ms
    from near_control import predict_center, distance_anchor


def _point(value, size):
    return (isinstance(value, (list, tuple)) and len(value) == size and
            all(isinstance(x, (int, float)) and math.isfinite(x) for x in value))


class TargetMemory:
    def __init__(self, projection, config):
        self.projection, self.config = projection, config
        task = config.get('tracking_task', {})
        self.pixel_tolerance = task.get('movement_pixel_tolerance_px', 12)
        self.distance_fraction = task.get('movement_distance_fraction', .10)
        self.confirmation_frames = task.get('movement_confirmation_frames', 2)
        self.acquisition_frames = task.get('candidate_confirmation_frames', self.confirmation_frames)
        self.track_width = config.get('track_width_mm',
            config.get('near_parking', {}).get('track_width_mm', 119.2))
        self.version = 0
        self.last_seen = self.last_precise = self.paired_coarse = None
        self.last_disappearance_direction = None
        self._anchor_version = None
        self._expected = self._visual_previous = None
        self._geometry_unknown = False
        self._acquire_count = 0
        self._held_suspect_count = 0
        self._held_candidate = None
        self._held_region = None
        self._invalidated = False
        self._epoch_start_stamp = None
        self._precise_candidate = False
        self._query_visual = None
        self._last_frame = self._last_stamp = None
        self._last_center = None
        # Only a measured target or its original full-pose pair can seed this
        # identity chain. Weak blue may continue it, but cannot start another
        # trajectory after a detector gap or a shape/position discontinuity.
        self._identity_visual = None
        self._exit_direction = None
        self._gap = False
        self._gap_ended_at = None
        self._event = 'none'
        self._moved = self._visible = self._associated = self._distance_valid = False
        self._visibility = 'unknown'
        self._movement_state = 'unknown'
        self._last_reason = None
        self._residual_px = self._residual_fraction = None
        # Bound translation evidence to the existing coarse freshness horizon.
        # A biased first tabletop proxy must not accumulate forever.
        self._motion_visual = self._motion_reference = None
        self._motion_height = 10.
        self._motion_rows = []
        self._motion_window_ms = config.get('tracking', {}).get('max_age_ms', 500)
        self._motion_row_limit = max(3, int(self._motion_window_ms/
            max(1, task.get('period_ms', 100)))+2)
        self._motion_count = 0
        self._motion_direction = None
        self._tracking_update = False
        self._navigation_cue = None
        self._motion_model_uncertain = False
        self._measurement_inconsistent = self._appearance_changed = False

    def _propagate(self, point, deltas):
        return predict_center(point, deltas, self.track_width, self.projection.normal,
            getattr(self.projection, 'forward_axis_Base', None),
            getattr(self.projection, 'motion_origin_Base_mm', None))

    def _evidence(self, observation):
        evidence = observation.get('visual_evidence')
        if isinstance(evidence, dict):
            return evidence
        visible = observation.get('status') in ('tracking', 'confirming')
        return dict(observation, visible=visible, associated=visible,
                    distance_valid=visible and _point(
                        observation.get('ground_contact_proxy_Base_mm'), 3),
                    ambiguous=False, candidate_count=1 if visible else 0)

    def _pixel_limit(self, evidence):
        bbox = evidence.get('bbox_px', [])
        size = max(bbox[2:]) if len(bbox) == 4 else 0
        return max(self.pixel_tolerance, .15*size)

    def _different(self, point, reference, evidence):
        a = self.projection.base_to_pixel(point)
        b = self.projection.base_to_pixel(reference)
        pixel = math.sqrt(sum((x-y)**2 for x, y in zip(a, b)))
        foot = self.projection.camera_table_foot()
        ranges = [math.sqrt(sum((p[k]-foot[k])**2 for k in range(3)))
                  for p in (point, reference)]
        fraction = abs(ranges[0]-ranges[1])/max(20., ranges[1])
        return pixel > self._pixel_limit(evidence) or fraction > self.distance_fraction, pixel, fraction

    def _held_match(self, current, previous):
        """Colour/size continuity using tracker limits, not object identity."""
        if current is None or previous is None:
            return False
        tracking = self.config.get('tracking', {})
        width, height = tracking.get('image_size_px', [1280, 960])
        jump = tracking.get('max_center_jump_fraction', .15)
        ratio_limit = tracking.get('max_size_ratio', 3)
        a, b = current['center_pixel'], previous['center_pixel']
        ratios = [current['bbox_px'][k]/previous['bbox_px'][k] for k in (2,3)]
        return (math.sqrt(((a[0]-b[0])/width)**2+((a[1]-b[1])/height)**2) <= jump
                and all(1/ratio_limit <= ratio <= ratio_limit for ratio in ratios))

    def _size_match(self, current, previous):
        """Shape continuity alone must not reject a genuinely moved target."""
        if current is None or previous is None:
            return False
        limit = self.config.get('tracking', {}).get('max_size_ratio', 3)
        return all(1/limit <= current['bbox_px'][k]/previous['bbox_px'][k] <= limit
                   for k in (2,3))

    def _common_translation(self, current, previous, expected_center):
        """Translation supported by BOTH box edges; expansion alone is not it."""
        shift = [a-b for a,b in zip(expected_center,previous['center_pixel'])]
        current_box, old = current['bbox_px'], previous['bbox_px']
        result = []
        for axis in (0,1):
            a = current_box[axis]-old[axis]-shift[axis]
            b = current_box[axis]+current_box[axis+2]-old[axis]-old[axis+2]-shift[axis]
            result.append(min(a,b) if min(a,b)>0 else (max(a,b) if max(a,b)<0 else 0.))
        return result

    def _visual_jump(self, current, previous):
        """An abrupt image discontinuity, independent of wheel prediction.

        Reuse the tracker's existing colour/size continuity gate; this is not
        proof of human motion or reliable identity among same-colour objects.
        """
        if current is None or previous is None:
            return False
        ratios = [current['bbox_px'][k]/previous['bbox_px'][k] for k in (2,3)]
        limit = self.config.get('tracking', {}).get('max_size_ratio', 3)
        return all(1/limit <= ratio <= limit for ratio in ratios) and not self._held_match(current,previous)

    def _roll_visual(self, visual):
        self._motion_visual = visual
        try:
            self._motion_reference = self.projection.pixel_to_plane(
                visual['center_pixel'],self._motion_height)
            projected = self.projection.base_to_pixel(self._motion_reference)
            if math.sqrt(sum((a-b)**2 for a,b in zip(projected,visual['center_pixel']))) > self._pixel_limit(visual):
                # An ill-conditioned/out-of-view ray is model uncertainty,
                # never evidence that a stationary blue region moves each frame.
                self._motion_reference = None
        except ValueError:
            self._motion_reference = None

    def _full_anchor(self):
        return (self.last_precise is not None and self._anchor_version == self.version and
                self.last_precise.get('status') == 'localized')

    def _distance_anchor(self):
        # bind_precise validates both complete poses and physical-side ranges.
        return self.last_precise is not None and self._anchor_version == self.version

    def release_precise(self):
        """Keep measurement history, without a current distance capability."""
        self._anchor_version = None

    def invalidate(self, reason='target_changed'):
        """One version change until fresh consistent observations reacquire it.

        Repeated notifications during ready2 recovery do not reset a timer or
        create a new version on every frame. No arm action is sent here.
        """
        if not self._invalidated:
            self.version += 1
            self._invalidated = True
            self._acquire_count = 0
            self._epoch_start_stamp = self._last_stamp
        self._precise_candidate = False
        self._geometry_unknown = True
        self._motion_rows = []
        self._motion_count = 0
        self._motion_direction = None
        self._last_reason = reason
        self._event = 'invalidated'
        self._moved = False
        return self.snapshot()

    def retarget_current(self, observation, reason='tracking_update'):
        """Adopt one current usable cue for motion, revoking old geometry.

        This is a following update, not proof of human motion or of object
        identity. The original native association/geometry evidence is kept.
        The coordinator must restrict calls to owned chassis following.
        """
        # A new colour cue may redirect unmeasured following. Once measured,
        # only confirmed same-target movement may revoke the travel distance.
        if self._distance_anchor():
            return None
        evidence = self._evidence(observation)
        box, pixel = evidence.get('bbox_px'), evidence.get('center_pixel')
        stamp = observation.get('capture_timestamp_ms', observation.get('timestamp_ms'))
        if (stamp != self._last_stamp or observation.get('frame_id',stamp) != self._last_frame or
                evidence.get('visible') is not True or evidence.get('ambiguous') is True or
                not _point(box,4) or min(box[2:]) < 3 or not _point(pixel,2) or
                max(box[2]/box[3],box[3]/box[2]) >
                    self.config.get('tracking',{}).get('max_aspect_ratio',4) or
                evidence.get('reason') in ('sparse_blob','implausible_aspect_ratio','invalid_or_tiny_blob',
                                         'inconsistent_with_20mm_cube_shape')):
            return None
        visual = {'bbox_px':list(box),'center_pixel':list(pixel)}
        if not self._tracking_update:
            self.invalidate(reason)
        self._tracking_update = True
        self._navigation_cue = dict(evidence)
        self._query_visual = self._identity_visual = self._visual_previous = visual
        self._expected = (list(evidence['ground_contact_proxy_Base_mm']) if
            evidence.get('distance_valid') is True and
            _point(evidence.get('ground_contact_proxy_Base_mm'),3) else None)
        self._distance_valid = self._expected is not None
        self._visible = self._associated = True
        self._visibility = 'visible'
        self._invalidated = self._gap = False
        self._gap_ended_at = self._exit_direction = None
        self._acquire_count = 1
        self._last_center = list(pixel)
        self._event, self._movement_state = 'tracking_update', 'consistent'
        self._moved = False
        self._residual_px = self._residual_fraction = None
        self._motion_model_uncertain = self._measurement_inconsistent = self._appearance_changed = False
        self._precise_candidate = True
        self._roll_visual(visual)
        self.last_seen = dict(observation)
        return self.snapshot()

    def update(self, observation, wheel_deltas=(), arm_state=None, motion_state=None,
               controlled_correction=False):
        """Use ordered wheel increments SINCE THE PREVIOUS update call.

        arm_state is an arm-engine snapshot. Close uses contact interpretation
        from its first frame; this does not claim a confirmed physical grasp.
        Lift/complete use holding mode.
        held_center_pixel, when supplied by the owner from fresh arm feedback,
        permits conservative external-motion association; without it ordinary
        lift/self-occlusion cannot invalidate the tabletop target. Close can
        use contact_center_pixel, otherwise the original coarse region centre.

        motion_state is the current chassis STATUS. While run is true or wheel
        increments exist, encoder/image disagreement is model uncertainty, not
        target motion. During commanded running, a current visible jump rebases
        following and revokes old fine geometry without a suspected-motion pause.
        A measured travel anchor instead requires confirmed, associated whole-
        region translation; coarse proxy noise or reacquisition cannot revoke it.

        controlled_correction is the owner's near static-verification/correction
        cycle, including ARC completion. Coarse displacement does not classify
        external motion in this cycle: its next precise image gives the next
        correction. Real visibility/loss rules and raw feedback stay unchanged;
        the current coarse image only rebases history and records the residual.
        """
        deltas = tuple(wheel_deltas)
        moving = bool((motion_state or {}).get('run')) or any(abs(a)+abs(b) for a,b in deltas)
        if self._expected is not None and deltas:
            self._expected = self._propagate(self._expected, deltas)
        if self._motion_reference is not None and deltas:
            self._motion_reference = self._propagate(self._motion_reference,deltas)
        self._event, self._moved = 'none', False
        self._tracking_update, self._navigation_cue = False, None
        self._motion_model_uncertain = False
        stamp = observation.get('capture_timestamp_ms', observation.get('timestamp_ms'))
        frame = observation.get('frame_id', stamp)
        if (stamp is None or (self._last_stamp is not None and
                (age_ms(stamp, self._last_stamp) <= 0 or frame == self._last_frame))):
            return self.snapshot()
        self._last_stamp, self._last_frame = stamp, frame
        evidence = self._evidence(observation)
        self._visible = evidence.get('visible') is True
        ambiguous = evidence.get('ambiguous') is True
        self._associated = evidence.get('associated') is True and not ambiguous
        box, pixel = evidence.get('bbox_px'), evidence.get('center_pixel')
        native_count = evidence.get('association_confirmations', 0)
        self._query_visual = ({'bbox_px':list(box),'center_pixel':list(pixel)}
            if self._visible and self._associated and _point(box,4) and
            min(box[2:]) > 0 and _point(pixel,2) else None)
        # Query authority is deliberately weaker than target identity: a
        # stable native colour candidate may request a stopped full pose,
        # without becoming a navigation, motion or arm reference.
        self._precise_candidate = bool(self._visible and self._associated and
            _point(box,4) and min(box[2:]) > 0 and _point(pixel,2) and
            isinstance(native_count,(int,float)) and native_count >=
            self.config.get('tracking',{}).get('confirmation_frames',3))
        point = evidence.get('ground_contact_proxy_Base_mm')
        self._distance_valid = evidence.get('distance_valid') is True and _point(point, 3)
        unique = (self._visible and not ambiguous and
                  (self._associated or evidence.get('candidate_count') == 1))
        arm = arm_state or {}
        holding = (arm.get('phase') in ('close', 'lift', 'complete') or
            (arm.get('grasp_confirmed') is True and
             arm.get('pending_stage') == 'lift'))
        if holding:
            self._visibility, self._movement_state = 'holding', 'holding'
            self._distance_valid = False
            self._event = 'holding'
            if self._held_region is None:
                source = (self.paired_coarse if self._anchor_version == self.version
                          else None) or self.last_seen or {}
                reference = self._evidence(source)
                box, pixel = reference.get('bbox_px'), reference.get('center_pixel')
                if _point(box, 4) and min(box[2:]) > 0 and _point(pixel, 2):
                    # Freeze the whole object before contact. A self-occluded
                    # corner must not shrink the external-motion region, and
                    # an unrelated large candidate must not expand it.
                    self._held_region = {'bbox_px':list(box), 'center_pixel':list(pixel)}
            if unique:
                self.last_seen = dict(observation)
            expected = arm.get('held_center_pixel')
            if not _point(expected, 2) and arm.get('phase') == 'close':
                expected = arm.get('contact_center_pixel')
                if not _point(expected, 2) and self._held_region is not None:
                    expected = self._held_region['center_pixel']
            center = evidence.get('center_pixel')
            bbox = evidence.get('bbox_px', [])
            reference_box = (self._held_region or {}).get('bbox_px', bbox)
            limit = max(48., 2*max(reference_box[2:])) if len(reference_box) == 4 else 48.
            shape_ok = (len(bbox) == 4 and min(bbox[2:]) >= 3 and
                max(bbox[2]/bbox[3], bbox[3]/bbox[2]) <=
                self.config.get('tracking', {}).get('max_aspect_ratio', 4) and
                evidence.get('reason') not in ('sparse_blob', 'implausible_aspect_ratio',
                                               'invalid_or_tiny_blob'))
            if (unique and shape_ok and _point(expected, 2) and _point(center, 2) and
                    math.sqrt(sum((a-b)**2 for a,b in zip(center,expected))) > limit):
                candidate = {'center_pixel':list(center), 'bbox_px':list(bbox)}
                self._held_suspect_count = (min(self.confirmation_frames,
                    self._held_suspect_count+1) if self._held_match(candidate,self._held_candidate)
                    else 1)
                self._held_candidate = candidate
                self._event, self._movement_state = 'suspected', 'suspected'
                if self._held_suspect_count >= self.confirmation_frames:
                    was_invalidated = self._invalidated
                    self.invalidate('held_target_moved')
                    self._event, self._movement_state = 'moved', 'confirmed'
                    self._moved = not was_invalidated
                    self._associated = True
            else:
                self._held_suspect_count = 0
                self._held_candidate = None
            self._exit_direction = None
            return self.snapshot()
        self._held_suspect_count = 0
        self._held_candidate = None
        self._held_region = None
        center = evidence.get('center_pixel')
        bbox = evidence.get('bbox_px', [])
        visual = ({'center_pixel':list(center), 'bbox_px':list(bbox)}
                  if _point(center,2) and _point(bbox,4) and min(bbox[2:]) > 0 else None)
        reference = self._identity_visual if self._gap else self._visual_previous
        source_reference = reference
        if self._motion_reference is not None and self._motion_visual is not None:
            # A fine-compute gap can contain a large commanded turn. Compare
            # with its encoder-propagated image position, not the old pixels.
            try:
                source_reference = self._motion_visual
                reference = dict(self._motion_visual,
                    center_pixel=self.projection.base_to_pixel(self._motion_reference))
            except ValueError:
                reference = None
        elif any(abs(a)+abs(b) for a,b in deltas):
            reference = None
        anchored = self._distance_anchor()
        if anchored and (not self._associated or visual is None or reference is None or
                not self._size_match(visual, reference) or
                evidence.get('reason') in ('sparse_blob', 'implausible_aspect_ratio',
                    'invalid_or_tiny_blob', 'inconsistent_with_20mm_cube_shape')):
            # A different/ambiguous patch, a gap or an expired native identity
            # cannot replace this measured target. Preserve its references;
            # the owner can finish the known travel and then take a new pose.
            self._associated = self._distance_valid = self._precise_candidate = False
            self._query_visual = None
            self._visibility = 'ambiguous' if ambiguous else 'unknown'
            self._movement_state = 'unknown'
            self._event = 'none' if self._gap else 'unknown'
            self._gap = True
            self._visual_previous = None
            self._motion_rows = []
            self._motion_count = 0
            self._motion_direction = None
            return self.snapshot()
        measured_jump = False
        if (self._distance_valid and evidence.get('associated') is False and
                visual is not None and source_reference is not None and reference is not None):
            # A newly measured cube can jump near enough that its box exceeds
            # the old size-ratio gate. Require actual common edge translation,
            # after ego-motion, so mask expansion alone cannot revoke geometry.
            shift = self._common_translation(visual,source_reference,reference['center_pixel'])
            size = self.config.get('tracking',{}).get('image_size_px',[1280,960])
            measured_jump = math.sqrt(sum((value/scale)**2 for value,scale in zip(shift,size))) > \
                self.config.get('tracking',{}).get('max_center_jump_fraction',.15)
        if (not anchored and bool((motion_state or {}).get('run')) and not arm and not ambiguous and
                (self._visual_jump(visual,reference) or measured_jump) and
                self.retarget_current(observation) is not None):
            return self.snapshot()
        unconfirmed_reacquisition = False
        if (not anchored and unique and self._distance_valid and self._gap and visual is not None and
                self._identity_visual is not None and
                not self._held_match(visual,self._identity_visual)):
            if not self._precise_candidate:
                # Native acquisition after a gap starts at one even though
                # its fresh tabletop geometry is measurable. It cannot make
                # another region inherit the previous target's fine result.
                unique = self._associated = False
                unconfirmed_reacquisition = True
            else:
                # The existing native confirmation has acquired a different
                # candidate. Preserve the old fine as evidence, but revoke its
                # version before this new region owns navigation/history.
                self.invalidate('reacquired_different_candidate')
                self._identity_visual = visual
                self._visual_previous = None
                self._motion_visual = self._motion_reference = None
                self._expected = self._last_center = None
                self._exit_direction = None
        if unique and not self._distance_valid:
            # A colour detector's unique/associated flag is not target proof:
            # after loss it may have acquired a tiny unrelated blue edge.
            # Retain visibility, but keep its centre and edges out of motion,
            # exit and reacquisition history until the target supports it.
            rejected_shape = evidence.get('reason') == 'inconsistent_with_20mm_cube_shape'
            self._associated = not rejected_shape and self._held_match(visual,self._identity_visual)
            if rejected_shape:
                # Native blue remains available for direction/a stopped query,
                # but a physically rejected patch cannot rewrite identity or
                # disprove an old fine pose using its unrelated image centre.
                self._query_visual = None
            unique = self._associated
        if unique and visual is not None and (self._identity_visual is None or
                self._held_match(visual,self._identity_visual)):
            self._identity_visual = visual
        self._visibility = 'ambiguous' if ambiguous else ('visible' if unique else 'unknown')
        if unique and _point(center, 2):
            self._exit_direction = None
            if self._last_center is not None:
                dx, dy = center[0]-self._last_center[0], center[1]-self._last_center[1]
                outward = {'left':dx < 0, 'right':dx > 0, 'top':dy < 0, 'bottom':dy > 0}
                for edge in evidence.get('edge_directions', []):
                    if outward.get(edge):
                        self._exit_direction = edge
                        self.last_disappearance_direction = edge
                        break
            self._last_center = list(center)
        if not unique:
            was_gap = self._gap
            self._gap = True
            self._movement_state = 'unknown'
            self._visual_previous = None
            if self._visible and not unconfirmed_reacquisition:
                # Unrelated or ambiguous blue between the edge observation
                # and a blank frame does not prove the target left the view.
                self._exit_direction = None
            if not self._visible and self._exit_direction is not None:
                direction = self._exit_direction
                was_invalidated = self._invalidated
                self.invalidate('left_view_' + direction)
                self._event, self._moved = 'exited', not was_invalidated
                self._expected = None
                self._exit_direction = None
            else:
                self._event = 'unknown' if not was_gap else 'none'
            return self.snapshot()
        previous_visual = self._visual_previous
        same_visual = visual is not None and self._held_match(visual,previous_visual)
        self._visual_previous = visual
        # Compare a short, rolling same-kind image reference. The coarse table
        # proxy may change markedly when another part of the cube turns blue;
        # its range mismatch is diagnostic, not proof that the cube moved.
        self._measurement_inconsistent = False
        if self._distance_valid and self._expected is not None:
            self._measurement_inconsistent, unused, self._residual_fraction = self._different(
                point,self._expected,evidence)
        self._expected = list(point) if self._distance_valid else None
        if visual is not None and self._motion_visual is not None and (
                self._motion_reference is not None or not any(abs(a)+abs(b) for a,b in deltas)):
            expected = (self.projection.base_to_pixel(self._motion_reference)
                        if self._motion_reference is not None else self._motion_visual['center_pixel'])
            shift = self._common_translation(visual,self._motion_visual,expected)
            observed = self._common_translation(visual,self._motion_visual,
                self._motion_visual['center_pixel'])
            if motion_state is not None and not motion_state.get('run'):
                if math.sqrt(sum(value*value for value in observed)) > self._pixel_limit(evidence):
                    # Passive wheel counts cannot erase an independently seen
                    # stopped-target translation. Its residual still subtracts
                    # the measured chassis motion before two-frame confirmation.
                    moving = False
            previous_box = self._motion_visual['bbox_px']
            self._appearance_changed = any(abs(bbox[k]/previous_box[k]-1) >
                self.distance_fraction for k in (2,3))
            moving_evidence = (not controlled_correction and moving and anchored and
                               not self._appearance_changed)
            if controlled_correction:
                # Actual whole-box image translation is expected during a
                # commanded turn, even when its encoder projection is biased.
                # This also owns the first run=0 exposure across ARC completion;
                # the passive-count override above must not call that motion.
                self._motion_rows = []
                self._motion_count = 0
                self._motion_direction = None
                self._residual_px = math.sqrt(sum(value*value for value in shift))
                self._motion_model_uncertain = self._residual_px > self._pixel_limit(evidence)
            elif moving and not moving_evidence:
                # A fixed cube can barely move in the image while an ARC's
                # wheel model predicts65px. Do not stop/relocalize for that
                # disagreement or carry it into the first stationary frame.
                self._motion_rows = []
                self._motion_count = 0
                self._motion_direction = None
                self._residual_px = math.sqrt(sum(value*value for value in shift))
                self._motion_model_uncertain = self._residual_px > self._pixel_limit(evidence)
            elif moving_evidence:
                # Require actual whole-box translation as well as the
                # encoder-compensated residual. A stationary image with a bad
                # wheel/axle model alone cannot prove external target motion.
                if self._motion_rows and len(self._motion_rows[-1]) != 5:
                    self._motion_rows = []
                    self._motion_count = 0
                    self._motion_direction = None
                self._motion_rows.append((stamp,shift[0],shift[1],observed[0],observed[1]))
            else:
                if self._motion_rows and len(self._motion_rows[-1]) != 3:
                    self._motion_rows = []
                    self._motion_count = 0
                    self._motion_direction = None
                self._motion_rows.append((stamp,shift[0],shift[1]))
            self._motion_rows = [row for row in self._motion_rows
                if 0 <= age_ms(stamp,row[0]) <= self._motion_window_ms][-self._motion_row_limit:]
            total = [sum(row[k] for row in self._motion_rows) for k in (1,2)]
            confirmed_evidence = not controlled_correction and not moving
            if moving_evidence:
                observed_total = [sum(row[k] for row in self._motion_rows) for k in (3,4)]
                confirmed_evidence = (math.sqrt(sum(value*value for value in observed_total)) >
                    self._pixel_limit(evidence) and sum(a*b for a,b in zip(total,observed_total)) > 0)
                self._motion_model_uncertain = math.sqrt(sum(value*value for value in total)) > \
                    self._pixel_limit(evidence) and not confirmed_evidence
            if not controlled_correction and (not moving or moving_evidence):
                self._residual_px = math.sqrt(sum(value*value for value in total))
            if confirmed_evidence and self._residual_px > self._pixel_limit(evidence):
                coherent = (self._motion_direction is not None and
                            sum(a*b for a,b in zip(total,self._motion_direction)) > 0)
                self._motion_count = self._motion_count+1 if coherent else 1
                self._motion_direction = total
            else:
                self._motion_count = 0
                self._motion_direction = None
        else:
            self._motion_rows = []
            self._motion_count = 0
            self._motion_direction = None
        if visual is not None:
            self._roll_visual(visual)
        if self._motion_count:
            self._movement_state,self._event = 'suspected','suspected'
            self._geometry_unknown = True
            if self._motion_count >= self.confirmation_frames:
                was_invalidated = self._invalidated
                self.invalidate('target_moved')
                self._moved = not was_invalidated
                self._event,self._movement_state = 'moved','confirmed'
                self._associated = True
                self._identity_visual = visual
            self.last_seen = dict(observation)
            return self.snapshot()
        self._geometry_unknown = (not self._associated or
            (not self._distance_valid and not self._full_anchor()))
        self._movement_state = 'unknown_geometry' if self._geometry_unknown else 'consistent'
        self.last_seen = dict(observation)
        self._acquire_count = self._acquire_count+1 if same_visual else 1
        recovering = (arm_state or {}).get('status') == 'recovering'
        if self._invalidated and self._acquire_count >= self.acquisition_frames and not recovering:
            self._invalidated = False
            self._associated = True
            self._event = 'reappeared' if self._gap else 'acquired'
        elif self._gap:
            self._event = 'reappeared'
        if self._gap:
            self._gap_ended_at = stamp
        self._gap = False
        # Unknown is not evidence of movement. An unchanged, uniquely
        # re-associated full-pose target can resume its existing static plan.
        if self._full_anchor() and self._associated and self._movement_state == 'consistent':
            self._gap_ended_at = None
        return self.snapshot()

    def precise_consistency(self, observation, coarse, wheel_deltas=None,
                            require_current=False):
        """Compare a completed capability with the current native input only.

        This does not assert physical target motion or change any history.
        Unknown/ambiguous input cannot disprove a retained distance. Both full
        poses and range-only results can nevertheless be stale when a current
        unique same-size region contradicts their original native image ray.
        wheel_deltas ends at that current image's capture, never clock-now.
        require_current distinguishes a completed stopped image's freshness
        check from retention of an already accepted travelling distance.
        """
        stamp = observation.get('capture_timestamp_ms')
        if (stamp is None or self._last_stamp is None or self._query_visual is None or
                observation.get('frame_id') != coarse.get('frame_id') or
                coarse.get('capture_timestamp_ms') != stamp or
                age_ms(self._last_stamp,stamp) < 0):
            return 'unknown'
        evidence = self._evidence(coarse)
        pixel, box = evidence.get('center_pixel'), evidence.get('bbox_px')
        if not _point(pixel,2) or not _point(box,4) or min(box[2:]) <= 0:
            return 'unknown'
        ratios = [self._query_visual['bbox_px'][k]/box[k] for k in (2,3)]
        limit = self.config.get('tracking',{}).get('max_size_ratio',3)
        if not all(1/limit <= ratio <= limit for ratio in ratios):
            # A tiny foreign patch is not contradictory target evidence.
            return 'unknown'
        try:
            unused, point, unused = distance_anchor(observation)
            height = (sum(a*b for a,b in zip(self.projection.normal,point))-self.projection.offset
                if observation.get('status') == 'localized' else self._motion_height)
            reference = self.projection.pixel_to_plane(pixel,height)
            if wheel_deltas is not None:
                reference = self._propagate(reference,wheel_deltas)
            elif stamp != self._last_stamp:
                return 'unknown'
            expected = self.projection.base_to_pixel(reference)
            paired = {'center_pixel':pixel,'bbox_px':box}
            shift = self._common_translation(self._query_visual,paired,expected)
            residual = math.sqrt(sum(value*value for value in shift))
        except (ValueError, TypeError):
            return 'unknown'
        if residual > self._pixel_limit(evidence):
            # This single coarse comparison is diagnostic. A bound travel
            # anchor is revoked only by update's confirmed movement rule.
            return 'unknown' if self._distance_anchor() and not require_current else 'changed'
        return 'consistent'

    def bind_precise(self, observation, coarse, version, wheel_deltas=None):
        """Bind only the original same-frame coarse/fine pair, in this version.

        A result after temporary occlusion can retain historical drive range;
        only a pair captured after reappearance clears needs_precise. Parking
        and grasp authority remain the coordinator's fresh static verification.
        For a delayed result, wheel_deltas is capture-to-current-image motion
        (unlike update's per-call increments), used to rebase the paired coarse
        point. Driving separately propagates its remaining distance to now.
        A fresh matched full pose can seed an invalidated current epoch;
        current-image identity is established by the following update.
        """
        if version != self.version:
            return False
        if (observation.get('frame_id') != coarse.get('frame_id') or
                observation.get('capture_timestamp_ms') is None or
                observation.get('capture_timestamp_ms') != coarse.get('capture_timestamp_ms')):
            return False
        if (self._epoch_start_stamp is not None and
                age_ms(observation['capture_timestamp_ms'],self._epoch_start_stamp) <= 0):
            return False
        evidence = self._evidence(coarse)
        recovering = self._invalidated
        if recovering and (observation.get('status') != 'localized' or
                observation.get('target_association',{}).get('status') != 'matched' or
                evidence.get('visible') is not True or evidence.get('ambiguous') is True or
                not _point(evidence.get('center_pixel'),2) or
                not _point(evidence.get('bbox_px'),4) or min(evidence['bbox_px'][2:]) <= 0):
            # A new epoch may acquire a fresh full same-frame pose directly.
            # An old job/range-only result cannot revive the previous target.
            return False
        try:
            unused, fine_point, unused = distance_anchor(observation)
        except (ValueError, TypeError):
            return False
        seeding = observation.get('status') == 'localized' and (
            recovering or self._identity_visual is None or
            not self._associated and not self._full_anchor())
        seed_visual = None
        if seeding:
            # Query-only colour was deliberately excluded from target motion
            # history. Before this delayed pose can seed identity, check its
            # original ray against the current query input without turning
            # that input into target/exit evidence.
            if self.precise_consistency(observation,coarse,wheel_deltas) != 'consistent':
                return False
            seed_visual = self._query_visual
        self.last_precise, self.paired_coarse = dict(observation), dict(coarse)
        self._anchor_version = version
        proxy = evidence.get('ground_contact_proxy_Base_mm')
        if evidence.get('distance_valid') and _point(proxy, 3):
            if wheel_deltas is not None:
                self._expected = self._propagate(proxy, wheel_deltas)
            elif coarse['capture_timestamp_ms'] == self._last_stamp:
                self._expected = list(proxy)
        elif observation.get('status') == 'localized':
            self._expected = None
        pixel = evidence.get('center_pixel')
        bound_source = False
        if observation.get('status') == 'localized' and _point(pixel,2):
            paired_box = evidence.get('bbox_px', [])
            if _point(paired_box,4) and min(paired_box[2:]) > 0:
                paired = {'center_pixel':list(pixel),'bbox_px':list(paired_box)}
                if seeding or coarse['capture_timestamp_ms'] == self._last_stamp:
                    self._identity_visual = seed_visual or paired
                if seeding or coarse['capture_timestamp_ms'] == self._last_stamp:
                    # Full geometry validates this original same-frame weak
                    # source. Public identity remains unchanged until the
                    # next update re-associates a current image to this seed.
                    bound_source = True
                    self._motion_visual = self._visual_previous = seed_visual or paired
                    self._last_center = list(self._motion_visual['center_pixel'])
                    self.last_seen = dict(coarse)
                    if recovering:
                        self._invalidated = False
                        self._associated = False
                        self._gap = True
            height = sum(a*b for a,b in zip(self.projection.normal,fine_point))-self.projection.offset
            try:
                reference = self.projection.pixel_to_plane(pixel,height)
                if wheel_deltas is not None:
                    reference = self._propagate(reference,wheel_deltas)
                elif coarse['capture_timestamp_ms'] != self._last_stamp:
                    reference = None
                current = self._motion_visual
                paired_box = evidence.get('bbox_px', [])
                if (reference is not None and (bound_source or self._associated and self._visible) and
                        current is not None and len(paired_box) == 4):
                    paired = {'center_pixel':pixel,'bbox_px':paired_box}
                    expected = self.projection.base_to_pixel(reference)
                    shift = self._common_translation(current,paired,expected)
                    self._geometry_unknown = (self._movement_state == 'suspected' or
                        math.sqrt(sum(value*value for value in shift)) > self._pixel_limit(evidence))
                    if not self._geometry_unknown:
                        self._motion_height = height
                        self._roll_visual(current)
                        if not self._gap:
                            self._gap_ended_at = None
                        # A new full pose supersedes pre-photo colour changes;
                        # retain only observed motion AFTER that source frame.
                        self._motion_rows = [row for row in self._motion_rows
                            if age_ms(row[0],observation['capture_timestamp_ms']) > 0]
                        self._motion_count = 0
                        self._motion_direction = None
            except ValueError:
                self._geometry_unknown = True
        if (not self._gap and (self._gap_ended_at is None or
                age_ms(observation['capture_timestamp_ms'], self._gap_ended_at) >= 0)):
            self._gap_ended_at = None
        return True

    def snapshot(self):
        anchor = self.last_precise is not None and self._anchor_version == self.version
        return {'version':self.version, 'event':self._event,
            'confirmed_moved':self._moved, 'visibility':self._visibility,
            'tracking_update':self._tracking_update, 'navigation_cue':self._navigation_cue,
            'visible':self._visible, 'associated':self._associated,
            'precise_candidate':self._precise_candidate,
            'table_distance_valid':self._distance_valid and self._associated,
            'movement_state':self._movement_state, 'target_valid':not self._invalidated,
            'anchor_valid':anchor, 'needs_precise':not anchor or self._invalidated or
                self._gap or self._gap_ended_at is not None or self._geometry_unknown,
            'last_seen':self.last_seen, 'last_precise':self.last_precise,
            'paired_coarse':self.paired_coarse,
            'last_disappearance_direction':self.last_disappearance_direction,
            'reason':self._last_reason, 'residual_px':self._residual_px,
            'residual_distance_fraction':self._residual_fraction,
            'measurement_inconsistent':self._measurement_inconsistent,
            'motion_model_uncertain':self._motion_model_uncertain,
            'appearance_changed':self._appearance_changed,
            'motion_thresholds_provisional':True}
