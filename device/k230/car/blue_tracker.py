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
        self.confirmations = 0

    def reset(self):
        self.previous, self.confirmations = None, 0

    def detect_frame(self, image, timestamp_ms):
        c = self.config; width, height = c['image_size_px']
        blobs = image.find_blobs([tuple(v) for v in c['lab_thresholds']],
            roi=(0, 0, width, height), pixels_threshold=c['min_pixels'],
            area_threshold=c['min_area_px2'], merge=False)
        candidates, rejected = [], []
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
            if reason:
                rejected.append({'bbox_px': bbox, 'reason': reason})
                continue
            candidates.append(dict(geometry, bbox_px=bbox, pixels=blob.pixels(),
                center_pixel=[float(blob.cx()), float(blob.cy())], timestamp_ms=timestamp_ms))
        if self.previous is not None and not 0 <= age_ms(timestamp_ms, self.previous['timestamp_ms']) <= c['max_age_ms']:
            self.reset()
        matched = []
        if self.previous is not None:
            old = self.previous
            for candidate in candidates:
                dx = (candidate['center_pixel'][0] - old['center_pixel'][0]) / width
                dy = (candidate['center_pixel'][1] - old['center_pixel'][1]) / height
                ratio = candidate['bbox_px'][3] / old['bbox_px'][3]
                if math.sqrt(dx * dx + dy * dy) <= c['max_center_jump_fraction'] and 1 / c['max_size_ratio'] <= ratio <= c['max_size_ratio']:
                    matched.append((dx * dx + dy * dy, candidate))
            if not matched:
                self.reset()
                return {'status': 'lost', 'timestamp_ms': timestamp_ms,
                        'candidates': candidates, 'rejected': rejected}
            matched.sort(key=lambda value: value[0]); selected = matched[0][1]
        elif candidates:
            selected = max(candidates, key=lambda value: value['pixels'])
        else:
            self.reset()
            return {'status': 'no_target', 'timestamp_ms': timestamp_ms,
                    'candidates': [], 'rejected': rejected}
        self.previous = selected
        self.confirmations += 1
        return dict(selected, status='tracking' if self.confirmations >= c['confirmation_frames'] else 'confirming',
                    confirmations=self.confirmations, candidates=candidates, rejected=rejected,
                    image_midline_error_px=selected['center_pixel'][0] - width / 2,
                    estimate_basis='blue blob and calibrated table contact proxy; coarse only')
