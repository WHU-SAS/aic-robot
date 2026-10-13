"""CanMV K230: preview continuously, save a clean image every 3 seconds."""
import os
import sys
import time
import json
from media.sensor import Sensor
from media.display import Display
from media.media import MediaManager

WIDTH = 640
HEIGHT = 480
INTERVAL_MS = 3000
SAVE_ROOT = '/sdcard/calibration'


def new_session():
    if 'calibration' not in os.listdir('/sdcard'):
        os.mkdir(SAVE_ROOT)
    used = set(os.listdir(SAVE_ROOT))
    for n in range(1, 10000):
        name = 'session_%04d' % n
        if name not in used:
            folder = SAVE_ROOT + '/' + name
            os.mkdir(folder)
            return folder
    raise RuntimeError('No unused session folder')


sensor = None
media_started = False
display_started = False
count = 0
try:
    folder = new_session()
    with open(folder + '/session.json', 'w') as stream:
        json.dump({'width': WIDTH, 'height': HEIGHT,
                   'interval_ms': INTERVAL_MS, 'pixel_format': 'RGB565',
                   'hmirror': False, 'vflip': False, 'jpeg_quality': 95,
                   'overlay_saved': False}, stream)
    sensor = Sensor()
    sensor.reset()
    sensor.set_framesize(width=WIDTH, height=HEIGHT)
    sensor.set_pixformat(Sensor.RGB565)
    sensor.set_hmirror(False)
    sensor.set_vflip(False)
    Display.init(Display.VIRT, width=WIDTH, height=HEIGHT, fps=30, to_ide=True)
    display_started = True
    MediaManager.init()
    media_started = True
    sensor.run()
    print('CAPTURE_FOLDER ' + folder)
    print('One photo every 3 seconds. IDE Stop ends capture; no photo limit.')
    started = time.ticks_ms()
    deadline = time.ticks_add(started, INTERVAL_MS)
    while True:
        os.exitpoint()
        img = sensor.snapshot()
        now = time.ticks_ms()
        if time.ticks_diff(now, deadline) >= 0:
            name = 'image_%04d.jpg' % (count + 1)
            temporary = folder + '/image_%04d.partial.jpg' % (count + 1)
            img.save(temporary, quality=95)
            size = os.stat(temporary)[6]
            if size <= 0:
                raise RuntimeError('Empty image: ' + temporary)
            os.rename(temporary, folder + '/' + name)
            count += 1
            with open(folder + '/captures.jsonl', 'a') as stream:
                stream.write(json.dumps({'image': name,
                    'elapsed_ms': time.ticks_diff(now, started),
                    'width': img.width(), 'height': img.height(),
                    'bytes': size}) + '\n')
            print('SAVED %d %s/%s' % (count, folder, name))
            deadline = time.ticks_add(now, INTERVAL_MS)
        # No text, marks, cropping or transforms on preview or saved images.
        Display.show_image(img)
        time.sleep_ms(20)
except KeyboardInterrupt:
    print('CAPTURE_STOPPED; %d photos retained' % count)
except BaseException as exc:
    print('CAPTURE_ERROR; %d photos retained' % count)
    sys.print_exception(exc)
finally:
    if sensor is not None:
        try:
            sensor.stop()
        except Exception as exc:
            print('Sensor cleanup:', exc)
    if display_started:
        try:
            Display.deinit()
        except Exception as exc:
            print('Display cleanup:', exc)
    os.exitpoint(os.EXITPOINT_ENABLE_SLEEP)
    time.sleep_ms(100)
    if media_started:
        MediaManager.deinit()
