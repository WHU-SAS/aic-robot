"""CanMV camera lifetime. One-shot owner; formal shared media scheduling is pending."""
import os
import time
from media.sensor import Sensor
from media.display import Display
from media.media import MediaManager


class CameraSession:
    def __init__(self, config, preview=False):
        self.config = config
        self.preview = preview
        self.sensor = None
        self.display_ready = False
        self.media_ready = False
        self.stopped = False

    def open(self):
        camera = self.config['camera']
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

    def snapshot(self):
        return self.sensor.snapshot()

    def show(self, image):
        Display.show_image(image)

    def stop(self):
        if self.sensor is not None and not self.stopped:
            self.sensor.stop()
            self.stopped = True

    def close(self):
        try:
            self.stop()
        except BaseException:
            pass
        if self.display_ready:
            try:
                Display.deinit()
            except BaseException:
                pass
        os.exitpoint(os.EXITPOINT_ENABLE_SLEEP)
        time.sleep_ms(100)
        if self.media_ready:
            try:
                MediaManager.deinit()
            except BaseException:
                pass
