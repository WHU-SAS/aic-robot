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
        self.track_width = config.get('track_width_mm',
            config.get('near_parking', {}).get('track_width_mm', 119.2))
        self.version = 0
        self.last_seen = self.last_precise = self.paired_coarse = None
        self.last_disappearance_direction = None
        self._anchor_version = None
        self._expected = self._suspect = None
        self._visual_reference = self._visual_previous = None
        self._visual_offset = None
        self._visual_suspect_count = 0
        self._geometry_unknown = False
        self._suspect_count = self._acquire_count = 0
        self._held_suspect_count = 0
        self._held_candidate = None
        self._invalidated = False
        self._last_frame = self._last_stamp = None
        self._last_center = None
        self._exit_direction = None
        self._gap = False
        self._gap_ended_at = None
        self._event = 'none'
        self._moved = self._visible = self._associated = self._distance_valid = False
        self._visibility = 'unknown'
        self._movement_state = 'unknown'
        self._last_reason = None
        self._residual_px = self._residual_fraction = None

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
        """Same colour/size continuity rule as BlueTracker, not object identity."""
        if previous is None:
            return False
        tracking = self.config.get('tracking', {})
        width, height = tracking.get('image_size_px', [1280, 960])
        jump = tracking.get('max_center_jump_fraction', .15)
        ratio_limit = tracking.get('max_size_ratio', 3)
        a, b = current['center_pixel'], previous['center_pixel']
        ratio = current['bbox_px'][3]/previous['bbox_px'][3]
        return (math.sqrt(((a[0]-b[0])/width)**2+((a[1]-b[1])/height)**2) <= jump
                and 1/ratio_limit <= ratio <= ratio_limit)

    def invalidate(self, reason='target_changed'):
        """One version change until fresh consistent observations reacquire it.

        Repeated notifications during ready2 recovery do not reset a timer or
        create a new version on every frame. No arm action is sent here.
        """
        if not self._invalidated:
            self.version += 1
            self._invalidated = True
            self._acquire_count = 0
        self._visual_reference = self._visual_offset = None
        self._visual_suspect_count = 0
        self._geometry_unknown = True
        self._last_reason = reason
        self._event = 'invalidated'
        self._moved = False
        return self.snapshot()

    def update(self, observation, wheel_deltas=(), arm_state=None):
        """Use ordered wheel increments SINCE THE PREVIOUS update call.

        arm_state is an arm-engine snapshot. Lift/complete use holding mode.
        held_center_pixel, when supplied by the owner from fresh arm feedback,
        permits conservative external-motion association; without it ordinary
        lift/self-occlusion cannot invalidate the tabletop target.
        """
        deltas = tuple(wheel_deltas)
        if self._expected is not None and deltas:
            self._expected = self._propagate(self._expected, deltas)
        if self._suspect is not None and deltas:
            self._suspect = self._propagate(self._suspect, deltas)
        if self._visual_reference is not None and deltas:
            self._visual_reference = self._propagate(self._visual_reference, deltas)
        self._event, self._moved = 'none', False
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
        point = evidence.get('ground_contact_proxy_Base_mm')
        self._distance_valid = evidence.get('distance_valid') is True and _point(point, 3)
        unique = (self._visible and not ambiguous and
                  (self._associated or evidence.get('candidate_count') == 1))
        arm = arm_state or {}
        holding = (arm.get('phase') in ('lift', 'complete') or
            (arm.get('grasp_confirmed') is True and
             (arm.get('phase') == 'close' or arm.get('pending_stage') == 'lift')))
        if holding:
            self._visibility, self._movement_state = 'holding', 'holding'
            self._distance_valid = False
            self._event = 'holding'
            if unique:
                self.last_seen = dict(observation)
            expected = (arm_state or {}).get('held_center_pixel')
            center = evidence.get('center_pixel')
            bbox = evidence.get('bbox_px', [])
            limit = max(48., 2*max(bbox[2:])) if len(bbox) == 4 else 48.
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
            self._suspect_count = 0
            return self.snapshot()
        self._held_suspect_count = 0
        self._held_candidate = None
        self._visibility = 'ambiguous' if ambiguous else ('visible' if unique else 'unknown')
        center = evidence.get('center_pixel')
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
            self._suspect_count = 0
            self._visual_previous = self._visual_offset = None
            self._visual_suspect_count = 0
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
        bbox = evidence.get('bbox_px', [])
        visual = ({'center_pixel':list(center), 'bbox_px':list(bbox)}
                  if _point(center,2) and len(bbox) == 4 and min(bbox[2:]) > 0 else None)
        same_visual = visual is not None and self._held_match(visual,self._visual_previous)
        self._visual_previous = visual
        # A full fine result can rescue coarse geometry rejection. Its paired
        # coarse centroid ray supplies ONLY an image-change reference, not a
        # newly measured table point or permission to use coarse range.
        if self._visual_reference is not None and visual is not None:
            expected = self.projection.base_to_pixel(self._visual_reference)
            offset = [a-b for a,b in zip(center,expected)]
            self._residual_px = math.sqrt(sum(value*value for value in offset))
            if self._residual_px > self._pixel_limit(evidence):
                coherent = (same_visual and self._visual_offset is not None and
                            sum(a*b for a,b in zip(offset,self._visual_offset)) > 0)
                self._visual_suspect_count = self._visual_suspect_count+1 if coherent else 1
                self._visual_offset = offset
                self._movement_state,self._event = 'suspected','suspected'
                self._geometry_unknown = True
                if self._visual_suspect_count >= self.confirmation_frames:
                    was_invalidated = self._invalidated
                    self.invalidate('target_moved')
                    self._moved = not was_invalidated
                    self._event,self._movement_state = 'moved','confirmed'
                    self._expected = list(point) if self._distance_valid else None
                    self._associated = True
                    self.last_seen = dict(observation)
                return self.snapshot()
            self._visual_suspect_count = 0
            self._visual_offset = None
        # Keep visible geometry failures for direction/search, never invent range.
        if not self._distance_valid:
            self.last_seen = dict(observation)
            self._geometry_unknown = self._visual_reference is None
            self._movement_state = 'unknown_geometry' if self._geometry_unknown else 'consistent'
            self._suspect_count = 0
            self._acquire_count = self._acquire_count+1 if same_visual else 1
            recovering = (arm_state or {}).get('status') == 'recovering'
            if self._invalidated and self._acquire_count >= self.confirmation_frames and not recovering:
                self._invalidated = False
                self._associated = True
                self._event = 'acquired'
            if self._gap:
                self._gap_ended_at = stamp
                self._gap = False
                self._event = 'reappeared'
            return self.snapshot()
        self._geometry_unknown = False
        if self._expected is None:
            self._expected = list(point)
            self._acquire_count = 1
            self._movement_state = 'consistent'
            self.last_seen = dict(observation)
            if not self._invalidated:
                self._event = 'acquired'
            if self._gap:
                self._gap_ended_at = stamp
                self._gap = False
                self._event = 'reappeared'
            return self.snapshot()
        different, self._residual_px, self._residual_fraction = self._different(
            point, self._expected, evidence)
        if different:
            same_candidate = self._suspect is not None and not self._different(
                point, self._suspect, evidence)[0]
            # A target still in motion need not stop at the same pixel for two
            # frames. Persistent residuals in the same direction also confirm
            # movement; opposite isolated frame jumps do not accumulate.
            if self._suspect is not None and not same_candidate:
                current = self.projection.base_to_pixel(point)
                previous = self.projection.base_to_pixel(self._suspect)
                expected = self.projection.base_to_pixel(self._expected)
                same_candidate = sum((a-c)*(b-c) for a,b,c in
                                     zip(current,previous,expected)) > 0
            self._suspect_count = self._suspect_count+1 if same_candidate else 1
            self._suspect = list(point)
            self._acquire_count = 0
            self._movement_state, self._event = 'suspected', 'suspected'
            if self._suspect_count >= self.confirmation_frames:
                was_invalidated = self._invalidated
                self.invalidate('target_moved')
                self._moved = not was_invalidated
                self._event, self._movement_state = 'moved', 'confirmed'
                self._expected = list(point)
                self._suspect = None
                self._suspect_count = 0
                self._associated = True
                self.last_seen = dict(observation)
            return self.snapshot()
        self._suspect = None
        self._suspect_count = 0
        self._movement_state = 'consistent'
        self.last_seen = dict(observation)
        self._acquire_count += 1
        recovering = (arm_state or {}).get('status') == 'recovering'
        if self._invalidated and self._acquire_count >= self.confirmation_frames and not recovering:
            self._invalidated = False
            self._event = 'reappeared' if self._gap else 'acquired'
        elif self._gap:
            self._event = 'reappeared'
        if self._gap:
            self._gap_ended_at = stamp
        self._gap = False
        self._associated = True
        return self.snapshot()

    def bind_precise(self, observation, coarse, version, wheel_deltas=None):
        """Bind only the original same-frame coarse/fine pair, in this version.

        A result after temporary occlusion can retain historical drive range;
        only a pair captured after reappearance clears needs_precise. Parking
        and grasp authority remain the coordinator's fresh static verification.
        For a delayed result, wheel_deltas is capture-to-now motion (unlike
        update's per-call increments), used to rebase the paired coarse point.
        """
        if version != self.version or self._invalidated:
            return False
        if (observation.get('frame_id') != coarse.get('frame_id') or
                observation.get('capture_timestamp_ms') is None or
                observation.get('capture_timestamp_ms') != coarse.get('capture_timestamp_ms')):
            return False
        try:
            unused, fine_point, unused = distance_anchor(observation)
        except (ValueError, TypeError):
            return False
        self.last_precise, self.paired_coarse = dict(observation), dict(coarse)
        self._anchor_version = version
        evidence = self._evidence(coarse)
        proxy = evidence.get('ground_contact_proxy_Base_mm')
        if evidence.get('distance_valid') and _point(proxy, 3):
            if wheel_deltas is not None:
                self._expected = self._propagate(proxy, wheel_deltas)
            elif coarse['capture_timestamp_ms'] == self._last_stamp:
                self._expected = list(proxy)
        elif observation.get('status') == 'localized':
            self._expected = None
        pixel = evidence.get('center_pixel')
        if observation.get('status') == 'localized' and _point(pixel,2):
            height = sum(a*b for a,b in zip(self.projection.normal,fine_point))-self.projection.offset
            try:
                reference = self.projection.pixel_to_plane(pixel,height)
                if wheel_deltas is not None:
                    reference = self._propagate(reference,wheel_deltas)
                elif coarse['capture_timestamp_ms'] != self._last_stamp:
                    reference = None
                if reference is not None:
                    self._visual_reference = reference
                if reference is not None and self._associated and _point(self._last_center,2):
                    expected = self.projection.base_to_pixel(reference)
                    self._geometry_unknown = math.sqrt(sum((a-b)**2 for a,b in
                        zip(self._last_center,expected))) > self._pixel_limit(evidence)
            except ValueError:
                self._visual_reference = None
        if (not self._gap and (self._gap_ended_at is None or
                age_ms(observation['capture_timestamp_ms'], self._gap_ended_at) >= 0)):
            self._gap_ended_at = None
        return True

    def snapshot(self):
        anchor = self.last_precise is not None and self._anchor_version == self.version
        return {'version':self.version, 'event':self._event,
            'confirmed_moved':self._moved, 'visibility':self._visibility,
            'visible':self._visible, 'associated':self._associated,
            'table_distance_valid':self._distance_valid and self._associated,
            'movement_state':self._movement_state, 'target_valid':not self._invalidated,
            'anchor_valid':anchor, 'needs_precise':not anchor or self._invalidated or
                self._gap or self._gap_ended_at is not None or self._geometry_unknown,
            'last_seen':self.last_seen, 'last_precise':self.last_precise,
            'paired_coarse':self.paired_coarse,
            'last_disappearance_direction':self.last_disappearance_direction,
            'reason':self._last_reason, 'residual_px':self._residual_px,
            'residual_distance_fraction':self._residual_fraction,
            'motion_thresholds_provisional':True}
