"""Owned JPEG95 frame, shared physical-side solve, no camera or motion policy."""
import time
try:
    from .physical_side import localize_steps
except ImportError:
    from physical_side import localize_steps


def _age(now, then):
    difference = getattr(time, 'ticks_diff', None)
    return difference(now, then) if difference else now-then


class FineSession:
    def __init__(self, config, clock_ms, purpose='precise'):
        self.config, self.clock = config, clock_ms
        self.include_range = purpose == 'car_moving_range'
        self.phase = self.latest = None
        self.jpeg = self.bgr = self.steps = None
        self.events = []
        self.timing = {}

    def start(self, image, frame, bbox=None):
        self.frame = dict(frame)
        self.bbox = list(bbox) if bbox is not None else None
        started = self.clock()
        self.timing = {'start_at_ms':started, 'advance_calls':0,
                       'solver_calls':0, 'solver_call_ms':0}
        self.jpeg = image.compressed(quality=95)
        self.timing['jpeg_ms'] = _age(self.clock(), started)
        self.events.append({'stage':'jpeg95', 'ms':self.timing['jpeg_ms']})
        self.phase = 'decode'

    def advance(self):
        if self.phase is None:
            return
        started = self.clock()
        self.timing['advance_calls'] += 1
        if 'first_advance_at_ms' not in self.timing:
            self.timing['first_advance_at_ms'] = started
        if self.phase == 'decode':
            import cv2
            try:
                from ulab import numpy as np
            except ImportError:
                import numpy as np
            self.bgr = cv2.imdecode(np.frombuffer(self.jpeg.bytearray(), dtype=np.uint8), cv2.IMREAD_COLOR)
            c = self.config
            self.steps = localize_steps(self.bgr, self.frame, self.bbox,
                c['intrinsics'], c['extrinsics'], c['table_plane'],
                max_candidates=c.get('near_parking', {}).get('max_candidates', 3),
                include_range=self.include_range)
            self.phase = 'geometry'
            self.timing['decode_ms'] = _age(self.clock(), started)
            self.events.append({'stage':'decode', 'ms':self.timing['decode_ms']})
        if self.phase != 'geometry':
            return
        budget = self.config.get('near_parking', {}).get('fine_budget_ms', 40)
        while True:
            step_at = self.clock()
            if 'first_solver_at_ms' not in self.timing:
                self.timing['first_solver_at_ms'] = step_at
            event = next(self.steps)
            duration = _age(self.clock(), step_at)
            self.timing['solver_calls'] += 1
            self.timing['solver_call_ms'] += duration
            self.events.append({'stage':event['stage'], 'ms':duration})
            if event['stage'] == 'complete':
                self.latest = event['observation']
                self.latest_ready_ms = self.clock()
                self.timing['ready_at_ms'] = self.latest_ready_ms
                self.phase = None
                self.bgr = self.steps = None
                return
            if _age(self.clock(), started) >= budget:
                return

    def take_evidence(self, label):
        """Detach original JPEG bytes; the caller archives only after stopping."""
        if self.jpeg is None:
            return None
        # CanMV bytearray() borrows the native image buffer. Copy before its
        # owner or MediaManager is released; the archive outlives both.
        frame = dict(self.frame, localization_timing=dict(self.timing))
        item = {'label':label, 'jpeg':bytes(self.jpeg.bytearray()),
                'frame':frame, 'observation':self.latest or
                    {'status':'processing_interrupted', 'phase':self.phase}}
        self.jpeg = None
        return item
