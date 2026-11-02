"""One owned JPEG95, advanced shared full-face localization, no hardware policy.

Each advance performs decode OR one localizer yield. A stage may exceed the
reported soft budget: this interface does not promise hard preemption. The
caller owns the camera and must detach evidence before its owner is released.
"""
import time
try:
    from .runtime import localize_precise
except ImportError:
    from vision_runtime import localize_precise


def _age(now, then):
    difference = getattr(time, 'ticks_diff', None)
    return difference(now, then) if difference else now-then


def _decode(jpeg):
    import cv2
    try:
        from ulab import numpy as np
    except ImportError:
        import numpy as np
    image = cv2.imdecode(np.frombuffer(jpeg.bytearray(), dtype=np.uint8), cv2.IMREAD_COLOR)
    if image is None:
        raise ValueError('Static precise JPEG decode failed')
    return image


def localize_steps(image, K, D, T, table_plane=None):
    """Resolve the existing raw detector lazily; no detector runs on import."""
    try:
        from ..pick.vision import localize_steps as solve
    except ImportError:
        from vision import localize_steps as solve
    return solve(image, K, D, T, table_plane)


class CooperativeStaticPickSession:
    """FineSession-style stopped localization with one yield per advance.

    start owns one compressed frame until take_evidence. take_evidence during
    processing first cancels the generator, then copies/detaches that JPEG;
    further advance calls cannot finish or publish the cancelled observation.
    """
    def __init__(self, config, clock_ms):
        if config.get('table_plane') is None:
            raise ValueError('Static precise localization requires the calibrated table plane')
        budget = config.get('tracking_task', {}).get('fine_budget_ms',
            config.get('near_parking', {}).get('fine_budget_ms', 20))
        if type(budget) not in (int, float) or not budget > 0:
            raise ValueError('A positive fine soft budget is required')
        self.config, self.clock, self.soft_budget_ms = config, clock_ms, budget
        self.phase = self.latest = self.latest_ready_ms = None
        self.jpeg = self.bgr = self.steps = None
        self.frame = self.bbox = None
        self.events = []
        self._interruption = None
        self._cancelled = False

    def _record(self, stage, started):
        elapsed = _age(self.clock(), started)
        self.events.append({'stage':stage, 'ms':elapsed,
            'soft_budget_ms':self.soft_budget_ms,
            'over_budget':elapsed > self.soft_budget_ms})

    def _release_large(self):
        steps = self.steps
        self.steps = self.bgr = None
        if steps is not None:
            close = getattr(steps, 'close', None)
            if close is not None:
                close()

    def _failed(self, phase, error):
        self.phase = self.latest = self.latest_ready_ms = None
        self._interruption = {'status':'processing_interrupted', 'phase':phase,
                              'error':str(error)}
        # Keep the original resource/solver error if generator cleanup also fails.
        try:
            self._release_large()
        except BaseException as cleanup:
            self._interruption['cleanup_error'] = str(cleanup)

    def start(self, image, frame, bbox=None):
        if self.jpeg is not None:
            raise RuntimeError('Detach the previous frame evidence before restarting')
        self.frame = dict(frame)
        if isinstance(self.frame.get('image_size_px'), (list, tuple)):
            self.frame['image_size_px'] = list(self.frame['image_size_px'])
        # Provenance only: full localization keeps its original candidate range.
        self.bbox = list(bbox) if bbox is not None else None
        self.latest = self.latest_ready_ms = self.phase = None
        self.steps = self.bgr = None
        self.events = []
        self._interruption = None
        self._cancelled = False
        started = self.clock()
        try:
            self.jpeg = image.compressed(quality=95)
        except BaseException as exc:
            self._failed('jpeg95', exc)
            self._record('jpeg95_error', started)
            raise
        self._record('jpeg95', started)
        self.phase = 'decode'

    def _wrap(self, raw=None, rejection=None):
        def complete(image, K, D, T, plane):
            if rejection is not None:
                raise rejection
            return raw
        c = self.config
        return localize_precise(self.bgr, self.frame, c['intrinsics'],
            c['extrinsics'], c['table_plane'], localizer=complete)

    def _publish(self, started, raw=None, rejection=None):
        result = self._wrap(raw, rejection)
        self._release_large()
        self.latest = result
        self.latest_ready_ms = self.clock()
        self.phase = None
        self._record('complete', started)

    def advance(self):
        if self.phase is None:
            return
        phase = self.phase
        started = self.clock()
        try:
            if phase == 'decode':
                self.bgr = _decode(self.jpeg)
                self.phase = 'geometry'
                self._record('decode', started)
                return
            if phase != 'geometry':
                raise RuntimeError('Unknown cooperative precise phase: '+str(phase))
            try:
                if self.steps is None:
                    c = self.config
                    intr = c['intrinsics']
                    K = [list(row) for row in intr['camera_matrix']]
                    for j in range(3):
                        K[0][j] *= 1280/intr['image_width']
                        K[1][j] *= 960/intr['image_height']
                    self.steps = localize_steps(self.bgr, K,
                        intr['distortion_coefficients'],
                        c['extrinsics']['T_Base_from_Camera_mm'], c['table_plane'])
                event = next(self.steps)
            except ValueError as exc:
                self._publish(started, rejection=exc)
                return
            except StopIteration:
                raise RuntimeError('Cooperative localizer ended without a result')
            if (not isinstance(event, dict) or
                    not isinstance(event.get('stage'), str) or not event['stage']):
                raise RuntimeError('Malformed cooperative localizer event')
            if event['stage'] == 'complete':
                self._publish(started, raw=event['result'])
                return
            self._record(event['stage'], started)
        except BaseException as exc:
            self._failed(phase, exc)
            self._record(phase+'_error', started)
            raise

    def cancel(self):
        """Discard this job/result and close its generator, retaining the JPEG."""
        phase = self.phase
        self.phase = self.latest = self.latest_ready_ms = None
        if self._cancelled:
            return
        if self.jpeg is not None and self._interruption is None:
            self._interruption = {'status':'processing_interrupted', 'phase':phase,
                                  'cancelled':True}
        self._cancelled = True
        self._release_large()

    def take_evidence(self, label):
        """Cancel an active job, copy its JPEG, then allow a new start."""
        if self.jpeg is None:
            return None
        if self.phase is not None:
            self.cancel()
        item = {'label':label, 'jpeg':bytes(self.jpeg.bytearray()),
                'frame':dict(self.frame), 'observation':self.latest or
                    self._interruption or {'status':'processing_interrupted', 'phase':None},
                'events':[dict(event) for event in self.events]}
        if isinstance(item['frame'].get('image_size_px'), (list, tuple)):
            item['frame']['image_size_px'] = list(item['frame']['image_size_px'])
        self.jpeg = None
        return item
