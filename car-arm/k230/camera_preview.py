"""CanMV K230 camera preview only; no robot commands or automatic capture."""
import time, os, sys
from media.sensor import Sensor
from media.display import Display
from media.media import MediaManager

sensor = None
frames = 0
try:
    sensor = Sensor()
    sensor.reset()
    sensor.set_framesize(width=640, height=480)
    sensor.set_pixformat(Sensor.RGB565)
    Display.init(Display.VIRT, width=640, height=480, fps=30, to_ide=True)
    MediaManager.init()
    sensor.run()
    print('K230_PREVIEW_STARTED 640x480')
    while True:
        os.exitpoint()
        img = sensor.snapshot()
        Display.show_image(img)
        frames += 1
        if frames % 90 == 0:
            print('K230_PREVIEW_FRAMES', frames)
except KeyboardInterrupt:
    print('K230_PREVIEW_STOPPED')
except BaseException as exc:
    sys.print_exception(exc)
finally:
    if sensor is not None:
        sensor.stop()
    Display.deinit()
    os.exitpoint(os.EXITPOINT_ENABLE_SLEEP)
    time.sleep_ms(100)
    MediaManager.deinit()
