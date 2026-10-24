"""Single-owner CanMV camera lifetime. No task or mode scheduling policy."""
import os
import time
from media.sensor import Sensor
from media.display import Display
from media.media import MediaManager


class CameraSession:
    _owner = None
    _frame_sequence = 0

    def __init__(self, config, preview=False):
        self.config = config
        self.preview = preview
        self.sensor = None
        self.display_ready = False
        self.media_ready = False
        self.stopped = False

    def open(self):
        if CameraSession._owner is not None:
            raise RuntimeError('Camera already owned; reuse the active session')
        CameraSession._owner = self
        try:
            camera = self.config['camera']
            self.stopped = False
            self.sensor = Sensor()
            self.sensor.reset()
            self.sensor.set_framesize(width=camera['width'], height=camera['height'])
            self.sensor.set_pixformat(Sensor.RGB565)
            self.sensor.set_hmirror(camera['hmirror'])
            self.sensor.set_vflip(camera['vflip'])
            Display.init(Display.VIRT, width=camera['width'], height=camera['height'],
                         fps=30, to_ide=self.preview)
            self.display_ready = True
            MediaManager.init()
            self.media_ready = True
            self.sensor.run()
            sampling = self.config['sampling']
            for _ in range(sampling['warmup_frames']):
                os.exitpoint()
                self.snapshot()
                time.sleep_ms(sampling['warmup_delay_ms'])
            return self
        except BaseException:
            self.close()
            raise

    def snapshot(self):
        if CameraSession._owner is not self or self.stopped:
            raise RuntimeError('Camera session is not active')
        return self.sensor.snapshot()

    def capture(self):
        """Borrow one native frame and software acquisition-call metadata.

        The image may be reused by the sensor on the next snapshot or close.
        The timestamp is not a sensor exposure timestamp; IDs are boot-local.
        """
        started = time.ticks_ms()
        image = self.snapshot()
        CameraSession._frame_sequence += 1
        camera = self.config['camera']
        return image, {'frame_id':CameraSession._frame_sequence,
            'capture_timestamp_ms':started,'clock_domain':'k230_ticks_ms',
            'timestamp_source':'software_snapshot_call',
            'image_size_px':[camera['width'],camera['height']]}

    def show(self, image):
        Display.show_image(image)

    def stop(self):
        if self.sensor is not None and not self.stopped:
            self.sensor.stop()
            self.stopped = True

    def close(self):
        if CameraSession._owner is not self:
            return
        try:
            self.stop()
        except BaseException:
            pass
        if self.display_ready:
            try:
                Display.deinit()
            except BaseException:
                pass
        try:
            os.exitpoint(os.EXITPOINT_ENABLE_SLEEP)
            time.sleep_ms(100)
            if self.media_ready:
                try:
                    MediaManager.deinit()
                except BaseException:
                    pass
        finally:
            self.display_ready = False
            self.media_ready = False
            CameraSession._owner = None
