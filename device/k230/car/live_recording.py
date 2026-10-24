"""SD-backed demonstration frames; independent of USB/browser transmission."""
import os
import json
import time


class SDFrameRecorder:
    def __init__(self, directory, metadata):
        if not directory.startswith('/sdcard/car_live/') or '..' in directory.split('/'):
            raise ValueError('Recording must use a new /sdcard/car_live/ session')
        try: os.mkdir('/sdcard/car_live')
        except OSError: os.stat('/sdcard/car_live')
        os.mkdir(directory)  # Existing session is an error, never overwritten.
        self.directory = directory
        self.frames = 0
        self.first_stamp = None
        self.last_stamp = None
        with open(directory + '/metadata.json', 'w') as f:
            json.dump(metadata, f)
        self.stream = open(directory + '/frames.jsonl', 'w')

    def write(self, image, observation):
        number = self.frames + 1
        name = 'frame_%06d.jpg' % number
        partial = self.directory + '/frame_%06d.partial.jpg' % number
        started = time.ticks_ms()
        image.save(partial, quality=80)
        os.rename(partial, self.directory + '/' + name)
        row = dict(observation)
        row.update(video_frame=number, filename=name,
                   recording_save_ms=time.ticks_diff(time.ticks_ms(), started))
        self.stream.write(json.dumps(row) + '\n')
        self.stream.flush()
        self.frames = number
        if self.first_stamp is None: self.first_stamp = row['capture_timestamp_ms']
        self.last_stamp = row['capture_timestamp_ms']
        return row

    def close(self):
        self.stream.close()
        with open(self.directory + '/summary.json', 'w') as f:
            json.dump({'frames': self.frames, 'first_capture_timestamp_ms': self.first_stamp,
                       'last_capture_timestamp_ms': self.last_stamp,
                       'format': 'annotated JPEG sequence plus capture-timestamp JSONL',
                       'requires_PC_video_export': True}, f)
