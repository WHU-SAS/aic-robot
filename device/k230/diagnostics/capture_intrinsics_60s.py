"""Run in CanMV IDE: preview and save one clean image per second for 60s.

Only the K230 camera and SD card are used. No robot commands.
A new /sdcard/calibration/session_XXXX folder is created on every run.
"""
import os
import sys
import time
import json
from media.sensor import Sensor
from media.display import Display
from media.media import MediaManager

WIDTH = 1280
HEIGHT = 960
INTERVAL_MS = 1000
DURATION_MS = 60000
TARGET_COUNT = DURATION_MS // INTERVAL_MS
JPEG_QUALITY = 95
SAVE_ROOT = '/sdcard/calibration'


def new_session():
    if 'calibration' not in os.listdir('/sdcard'):
        os.mkdir(SAVE_ROOT)
    used = set(os.listdir(SAVE_ROOT))
    for number in range(1, 10000):
        name = 'session_%04d' % number
        if name not in used:
            folder = SAVE_ROOT + '/' + name
            os.mkdir(folder)
            return folder
    raise RuntimeError('No unused session folder')


def save_session(folder, metadata):
    with open(folder + '/session.json', 'w') as stream:
        json.dump(metadata, stream)


sensor = None
sensor_started = False
media_started = False
display_started = False
folder = None
metadata = None
count = 0
missed_slots = 0
last_slot = 0
started = None

try:
    folder = new_session()
    metadata = {
        'purpose': 'intrinsics_recheck_at_current_manual_focus',
        'width': WIDTH, 'height': HEIGHT,
        'interval_ms': INTERVAL_MS, 'duration_ms': DURATION_MS,
        'target_count': TARGET_COUNT, 'pixel_format': 'RGB565',
        'hmirror': False, 'vflip': False, 'jpeg_quality': JPEG_QUALITY,
        'overlay_saved': False, 'robot_commands_sent': False,
        'schedule': 'elapsed-second slots; missed slots are not burst-captured',
        'focus_setting': 'keep the current lens setting; not electronically measured',
        'status': 'starting', 'saved_count': 0, 'missed_slots': 0,
    }
    save_session(folder, metadata)
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
    sensor_started = True

    # Let exposure settle; show the view immediately, without a countdown.
    for _ in range(30):
        os.exitpoint()
        Display.show_image(sensor.snapshot())
        time.sleep_ms(30)

    started = time.ticks_ms()
    metadata['status'] = 'capturing'
    save_session(folder, metadata)
    print('CAPTURE_FOLDER ' + folder)
    print('CAPTURE_STARTED: 1280x960, 1 photo/sec, 60 seconds')

    while last_slot < TARGET_COUNT:
        os.exitpoint()
        img = sensor.snapshot()
        Display.show_image(img)
        now = time.ticks_ms()
        elapsed = time.ticks_diff(now, started)
        slot = elapsed // INTERVAL_MS
        if slot > last_slot:
            if slot > TARGET_COUNT:
                missed_slots += TARGET_COUNT - last_slot
                last_slot = TARGET_COUNT
                break
            missed_slots += slot - last_slot - 1
            last_slot = slot
            name = 'image_%04d.jpg' % (count + 1)
            temporary = folder + '/image_%04d.partial.jpg' % (count + 1)
            # Save the original frame; no text, corner marks or resizing.
            img.save(temporary, quality=JPEG_QUALITY)
            size = os.stat(temporary)[6]
            if size <= 0:
                raise RuntimeError('Empty image: ' + temporary)
            os.rename(temporary, folder + '/' + name)
            count += 1
            with open(folder + '/captures.jsonl', 'a') as stream:
                stream.write(json.dumps({
                    'image': name, 'scheduled_slot': slot,
                    'elapsed_ms': elapsed,
                    'save_finished_elapsed_ms': time.ticks_diff(time.ticks_ms(), started),
                    'width': img.width(), 'height': img.height(), 'bytes': size,
                }) + '\n')
            print('SAVED %02d/60 %s/%s' % (count, folder, name))
        time.sleep_ms(10)

    metadata['status'] = 'completed' if missed_slots == 0 else 'completed_with_missed_slots'
    print('CAPTURE_DONE: %d photos, %d missed seconds, %s' % (count, missed_slots, folder))
except KeyboardInterrupt:
    if metadata is not None:
        metadata['status'] = 'stopped_by_user'
    print('CAPTURE_STOPPED: %d photos retained' % count)
except BaseException as exc:
    if metadata is not None:
        metadata['status'] = 'error'
        metadata['error'] = str(exc)
    print('CAPTURE_ERROR: %d photos retained' % count)
    sys.print_exception(exc)
finally:
    if sensor is not None and sensor_started:
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
        try:
            MediaManager.deinit()
        except Exception as exc:
            print('Media cleanup:', exc)
    if folder is not None and metadata is not None:
        metadata['saved_count'] = count
        metadata['missed_slots'] = missed_slots
        if started is not None:
            metadata['finished_elapsed_ms'] = time.ticks_diff(time.ticks_ms(), started)
        try:
            save_session(folder, metadata)
        except Exception as exc:
            print('Session metadata:', exc)
