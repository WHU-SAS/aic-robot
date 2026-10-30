"""One stopped-frame JPEG for strict shared-side and existing arm recovery.

The caller has already stopped the chassis. A geometry advance can block while
legacy recovery runs; this session must not be used for moving range tracking.
It never captures another frame or requests the car-only distance capability.
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


class StaticPickSession:
    """FineSession-compatible final verification using the full arm detector."""
    def __init__(self, config, clock_ms, purpose='precise'):
        if purpose != 'precise':
            raise ValueError('StaticPickSession is only for stopped precise localization')
        if config.get('table_plane') is None:
            raise ValueError('Static precise localization requires the calibrated table plane')
        self.config, self.clock = config, clock_ms
        self.phase = self.latest = self.latest_ready_ms = None
        self.jpeg = self.bgr = None
        self.events = []

    def start(self, image, frame, bbox=None):
        if self.jpeg is not None:
            raise RuntimeError('Detach the previous frame evidence before restarting')
        self.frame = dict(frame)
        # Kept only for provenance: final strict localization searches the full
        # captured frame, independently of a moving coarse bounding box.
        self.bbox = list(bbox) if bbox is not None else None
        self.latest = self.latest_ready_ms = None
        self.events = []
        started = self.clock()
        self.jpeg = image.compressed(quality=95)
        self.events.append({'stage':'jpeg95', 'ms':_age(self.clock(), started)})
        self.phase = 'decode'

    def advance(self):
        if self.phase is None:
            return
        started = self.clock()
        if self.phase == 'decode':
            self.bgr = _decode(self.jpeg)
            self.events.append({'stage':'decode', 'ms':_age(self.clock(), started)})
            self.phase = 'geometry'
        started = self.clock()
        try:
            c = self.config
            # The default localizer already runs shared fast sides followed by
            # the verified legacy recovery only if necessary, on this same BGR.
            self.latest = localize_precise(self.bgr, self.frame,
                c['intrinsics'], c['extrinsics'], c['table_plane'])
            self.latest_ready_ms = self.clock()
            self.events.append({'stage':'complete', 'ms':_age(self.latest_ready_ms, started)})
            self.phase = None
        finally:
            self.bgr = None

    def take_evidence(self, label):
        """Copy original JPEG bytes before the camera/media owner is released."""
        if self.jpeg is None:
            return None
        item = {'label':label, 'jpeg':bytes(self.jpeg.bytearray()),
                'frame':self.frame, 'observation':self.latest or
                    {'status':'processing_interrupted', 'phase':self.phase}}
        self.jpeg = None
        return item
