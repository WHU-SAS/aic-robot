"""Native CanMV blue-blob tracker for coarse approach, with no PnP or motion.

The full image is searched: the grasp detector's lower-half ROI is unsuitable
for a distant cube. A tracked blob is only a coarse candidate, never a validated
20mm physical face or permission to grasp.
"""
import math
import time


def age_ms(now, then):
    ticks_diff = getattr(time, 'ticks_diff', None)
    return ticks_diff(now, then) if ticks_diff else now - then


class BlueTracker:
    def __init__(self, config, projection):
        self.config, self.projection = config, projection
        self._thresholds = [tuple(value) for value in config['lab_thresholds']]
        width, height = config['image_size_px']
        self._roi = (0, 0, width, height)
        self.previous = None
        # Navigation confirmation resets on a gap. Keep the short-lived
        # identity reference separately, so the next weak patch is not a new
        # acquisition merely because one frame was missing or ambiguous.
        self._association_previous = None
        self.confirmations = 0
        self.association_confirmations = 0

    def reset(self):
        self.previous, self.confirmations = None, 0
        self._association_previous = None
        self.association_confirmations = 0

    def _match(self, candidate, old):
        width, height = self.config['image_size_px']
        dx = (candidate['center_pixel'][0]-old['center_pixel'][0])/width
        dy = (candidate['center_pixel'][1]-old['center_pixel'][1])/height
        ratios = [candidate['bbox_px'][k]/old['bbox_px'][k] for k in (2, 3)]
        if (math.sqrt(dx*dx+dy*dy) <= self.config['max_center_jump_fraction'] and
                all(1/self.config['max_size_ratio'] <= ratio <= self.config['max_size_ratio']
                    for ratio in ratios)):
            return dx*dx+dy*dy
        return None

    def _contains(self, outer, inner):
        x,y,w,h = outer['bbox_px'];a,b,c,d = inner['bbox_px']
        return x<a and y<b and x+w>a+c and y+h>b+d

    def _independent(self, candidates):
        # Native thresholding can split one cube into an enclosing measurable
        # region and an interior patch. Retain the actual outer measurement;
        # invalid cues and overlapping/separate objects cannot suppress it.
        return [cue for cue in candidates if not any(
            other is not cue and self._contains(other,cue) for other in candidates)]

    def _select(self, cues, candidates, timestamp_ms):
        """Seeing blue and measuring a tabletop target are separate capabilities.

        A unique unassociated candidate is retained to reveal a possible jump;
        its table-distance validity does not establish that it is our target.
        The task decides target identity/movement, not this color detector.
        """
        evidence = {'visible': bool(cues), 'associated': False, 'ambiguous': False,
            'candidate_count': len(cues), 'timestamp_ms': timestamp_ms,
            'distance_valid': False, 'edge_directions': [], 'reason': 'no_blue_evidence',
            'confirmations': 0, 'association_confirmations': 0, 'navigation_confirmed': False}
        old = self.previous or self._association_previous
        if not cues:
            return None, evidence
        independent = self._independent(candidates)
        # Keep all raw candidates for diagnostics, but not duplicate inner
        # measurable patches in the association/selection pool.
        active = [cue for cue in cues if not cue['distance_valid'] or
                  any(cue is candidate for candidate in independent)]
        matched = [cue for cue in active if old is not None and self._match(cue, old) is not None]
        measured_matches = [cue for cue in matched if cue['distance_valid']]
        if len(measured_matches) == 1:
            matched = [cue for cue in matched if cue['distance_valid'] or
                       not self._contains(measured_matches[0],cue)]
        # At acquisition, original tabletop checks can distinguish a unique
        # cube candidate from unrelated weak blue patches. Do not choose the
        # largest of multiple equally plausible targets.
        pool = matched if matched else (independent if self.previous is None and independent else active)
        if len(pool) != 1:
            evidence.update(ambiguous=True, reason='ambiguous_blue_candidates')
            return None, evidence
        selected = pool[0]
        associated = (bool(matched) or old is None or
                      (self.previous is None and selected['distance_valid']))
        for key in ('bbox_px', 'center_pixel', 'pixels', 'edge_directions'):
            evidence[key] = selected[key]
        width = self.config['image_size_px'][0]
        evidence.update(associated=associated,
            image_midline_error_px=selected['center_pixel'][0]-width/2,
            distance_valid=selected['distance_valid'],
            reason=selected['measurement_reason'] or (None if associated else 'unassociated_blue'))
        if selected['distance_valid']:
            evidence['ground_contact_proxy_Base_mm'] = list(selected['ground_contact_proxy_Base_mm'])
        return selected, evidence

    def detect_frame(self, image, timestamp_ms):
        c = self.config; width, height = c['image_size_px']
        blobs = image.find_blobs(self._thresholds,
            roi=self._roi, pixels_threshold=c['min_pixels'],
            area_threshold=c['min_area_px2'], merge=False)
        candidates, rejected, cues = [], [], []
        for blob in blobs:
            bbox = list(blob.rect()); x, y, w, h = bbox
            pixels = blob.pixels()
            reason = None
            if min(w, h) < 3 or not 0 <= pixels <= w * h:
                reason = 'invalid_or_tiny_blob'
            elif min(x, y) <= 1 or x + w >= width - 1 or y + h >= height - 1:
                reason = 'clipped_blob'
            elif max(w / h, h / w) > c['max_aspect_ratio']:
                reason = 'implausible_aspect_ratio'
            elif pixels / (w * h) < c['min_fill_ratio']:
                reason = 'sparse_blob'
            geometry = None
            if reason is None:
                try:
                    geometry = self.projection.blob_geometry(bbox)
                    forward = geometry['ground_contact_proxy_Base_mm'][0]
                    ratio = geometry['observed_to_expected_height_ratio']
                    if not c['forward_range_mm'][0] <= forward <= c['forward_range_mm'][1]:
                        reason = 'outside_approach_distance'
                    elif not c['height_ratio_range'][0] <= ratio <= c['height_ratio_range'][1]:
                        reason = 'inconsistent_with_20mm_height'
                except ValueError as exc:
                    reason = str(exc)
            if reason != 'invalid_or_tiny_blob':
                edges = []
                if x <= 1: edges.append('left')
                if x+w >= width-1: edges.append('right')
                if y <= 1: edges.append('top')
                if y+h >= height-1: edges.append('bottom')
                cue = {'bbox_px': bbox, 'center_pixel': [float(blob.cx()), float(blob.cy())],
                    'pixels': pixels, 'timestamp_ms': timestamp_ms,
                    'edge_directions': edges, 'distance_valid': reason is None,
                    'measurement_reason': reason}
                if reason is None:
                    cue.update(geometry)
                cues.append(cue)
            if reason:
                rejected.append({'bbox_px': bbox, 'reason': reason})
                continue
            candidates.append(cue)
        if self.previous is not None and not 0 < age_ms(timestamp_ms, self.previous['timestamp_ms']) <= c['max_age_ms']:
            self.previous, self.confirmations = None, 0
            self.association_confirmations = 0
        if (self._association_previous is not None and not
                0 < age_ms(timestamp_ms, self._association_previous['timestamp_ms']) <= c['max_age_ms']):
            self._association_previous = None
        had_measurement = self.confirmations > 0
        selected, evidence = self._select(cues, candidates, timestamp_ms)
        if selected is None or not evidence['associated']:
            self.previous, self.confirmations = None, 0
            self.association_confirmations = 0
            return {'status': 'lost' if had_measurement else 'no_target', 'timestamp_ms': timestamp_ms,
                    'candidates': candidates, 'visual_candidates': cues,
                    'rejected': rejected, 'visual_evidence': evidence}
        self.previous = selected
        self._association_previous = selected
        self.association_confirmations += 1
        evidence['association_confirmations'] = self.association_confirmations
        if not selected['distance_valid']:
            self.confirmations = 0
            return {'status': 'lost' if had_measurement else 'no_target', 'timestamp_ms': timestamp_ms,
                    'candidates': candidates, 'visual_candidates': cues,
                    'rejected': rejected, 'visual_evidence': evidence}
        self.confirmations += 1
        confirmed = self.confirmations >= c['confirmation_frames']
        evidence.update(confirmations=self.confirmations, navigation_confirmed=confirmed)
        return dict(selected, status='tracking' if confirmed else 'confirming',
                    confirmations=self.confirmations, candidates=candidates, visual_candidates=cues,
                    rejected=rejected,
                    visual_evidence=evidence,
                    image_midline_error_px=selected['center_pixel'][0] - width / 2,
                    estimate_basis='blue blob and calibrated table contact proxy; coarse only')
