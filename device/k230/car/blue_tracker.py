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
        self.previous = None
        self.visual_previous = None
        self.confirmations = 0

    def reset(self):
        self.previous, self.confirmations = None, 0
        self.visual_previous = None

    def _match(self, candidate, old):
        width, height = self.config['image_size_px']
        dx = (candidate['center_pixel'][0]-old['center_pixel'][0])/width
        dy = (candidate['center_pixel'][1]-old['center_pixel'][1])/height
        ratio = candidate['bbox_px'][3]/old['bbox_px'][3]
        if (math.sqrt(dx*dx+dy*dy) <= self.config['max_center_jump_fraction'] and
                1/self.config['max_size_ratio'] <= ratio <= self.config['max_size_ratio']):
            return dx*dx+dy*dy
        return None

    def _evidence(self, cues, candidates, timestamp_ms):
        """Seeing blue and measuring a tabletop target are separate capabilities.

        A unique unassociated candidate is retained to reveal a possible jump;
        its table-distance validity does not establish that it is our target.
        The task decides target identity/movement, not this color detector.
        """
        evidence = {'visible': bool(cues), 'associated': False, 'ambiguous': False,
            'candidate_count': len(cues), 'timestamp_ms': timestamp_ms,
            'distance_valid': False, 'edge_directions': [], 'reason': 'no_blue_evidence'}
        old = self.visual_previous
        if old is not None and not 0 <= age_ms(timestamp_ms, old['timestamp_ms']) <= self.config['max_age_ms']:
            self.visual_previous = old = None
        if not cues:
            return evidence
        matched = [cue for cue in cues if old is not None and self._match(cue, old) is not None]
        # At acquisition, original tabletop checks can distinguish a unique
        # cube candidate from unrelated weak blue patches. Do not choose the
        # largest of multiple equally plausible targets.
        pool = matched if matched else (candidates if old is None and candidates else cues)
        if len(pool) != 1:
            evidence.update(ambiguous=True, reason='ambiguous_blue_candidates')
            return evidence
        selected = pool[0]
        associated = old is None or bool(matched)
        for key in ('bbox_px', 'center_pixel', 'pixels', 'edge_directions'):
            evidence[key] = selected[key]
        width = self.config['image_size_px'][0]
        evidence.update(associated=associated,
            image_midline_error_px=selected['center_pixel'][0]-width/2,
            distance_valid=selected['distance_valid'],
            reason=selected['measurement_reason'] or (None if associated else 'unassociated_blue'))
        if selected['distance_valid']:
            evidence['ground_contact_proxy_Base_mm'] = list(selected['ground_contact_proxy_Base_mm'])
        if associated:
            self.visual_previous = selected
        return evidence

    def detect_frame(self, image, timestamp_ms):
        c = self.config; width, height = c['image_size_px']
        blobs = image.find_blobs([tuple(v) for v in c['lab_thresholds']],
            roi=(0, 0, width, height), pixels_threshold=c['min_pixels'],
            area_threshold=c['min_area_px2'], merge=False)
        candidates, rejected, cues = [], [], []
        for blob in blobs:
            bbox = list(blob.rect()); x, y, w, h = bbox
            reason = None
            if min(w, h) < 3 or not 0 <= blob.pixels() <= w * h:
                reason = 'invalid_or_tiny_blob'
            elif min(x, y) <= 1 or x + w >= width - 1 or y + h >= height - 1:
                reason = 'clipped_blob'
            elif max(w / h, h / w) > c['max_aspect_ratio']:
                reason = 'implausible_aspect_ratio'
            elif blob.pixels() / (w * h) < c['min_fill_ratio']:
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
                    'pixels': blob.pixels(), 'timestamp_ms': timestamp_ms,
                    'edge_directions': edges, 'distance_valid': reason is None,
                    'measurement_reason': reason}
                if reason is None:
                    cue.update(geometry)
                cues.append(cue)
            if reason:
                rejected.append({'bbox_px': bbox, 'reason': reason})
                continue
            candidates.append(cue)
        evidence = self._evidence(cues, candidates, timestamp_ms)
        if self.previous is not None and not 0 <= age_ms(timestamp_ms, self.previous['timestamp_ms']) <= c['max_age_ms']:
            self.previous, self.confirmations = None, 0
        matched = []
        if self.previous is not None:
            old = self.previous
            for candidate in candidates:
                distance = self._match(candidate, old)
                if distance is not None:
                    matched.append((distance, candidate))
            if not matched:
                self.previous, self.confirmations = None, 0
                return {'status': 'lost', 'timestamp_ms': timestamp_ms,
                        'candidates': candidates, 'rejected': rejected, 'visual_evidence': evidence}
            matched.sort(key=lambda value: value[0]); selected = matched[0][1]
        elif candidates:
            selected = max(candidates, key=lambda value: value['pixels'])
        else:
            self.previous, self.confirmations = None, 0
            return {'status': 'no_target', 'timestamp_ms': timestamp_ms,
                    'candidates': [], 'rejected': rejected, 'visual_evidence': evidence}
        self.previous = selected
        self.confirmations += 1
        return dict(selected, status='tracking' if self.confirmations >= c['confirmation_frames'] else 'confirming',
                    confirmations=self.confirmations, candidates=candidates, rejected=rejected,
                    visual_evidence=evidence,
                    image_midline_error_px=selected['center_pixel'][0] - width / 2,
                    estimate_basis='blue blob and calibrated table contact proxy; coarse only')
