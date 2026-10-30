"""Video-only H.264 capture. No detector, display, audio or robot control.

Uses CanMV's hardware encoder; preserve its Annex-B stream verbatim instead
of using the firmware MP4 muxer (which dropped key frames on this firmware).
API: https://www.kendryte.com/k230_canmv/en/main/example/media/video.html
"""
import json
import os
import time
import uctypes
from media.sensor import Sensor
from media.vencoder import Encoder, ChnAttrStr, StreamData
from media.media import MediaManager, VIDEO_ENCODE_MOD_ID, VENC_DEV_ID


def record_raw_video(directory, duration_ms):
    if not directory.startswith('/sdcard/car_video/') or '..' in directory.split('/'):
        raise ValueError('Expected a new /sdcard/car_video/ session')
    # recording_space/write_video_packet are bundled by record_car_video.py.
    space = recording_space(os.statvfs('/sdcard'), duration_ms)
    print('CAR_VIDEO_STORAGE ' + json.dumps(space))
    if not space['sufficient']:
        raise OSError('SD卡空间不足：可用 %.1f MiB，本次需要至少 %.1f MiB，请先备份并清理SD卡。' %
                      (space['available_bytes'] / 1048576, space['required_bytes'] / 1048576))
    try: os.mkdir('/sdcard/car_video')
    except OSError: os.stat('/sdcard/car_video')
    os.mkdir(directory)  # Never overwrite a previous recording.
    partial = directory + '/raw.partial.h264'
    target = directory + '/raw.h264'
    metadata = {'image_size_px': [1280, 960], 'hmirror': False, 'vflip': False,
                'codec': 'H264_AnnexB', 'requested_fps': 30, 'bitrate_kbps': 4096,
                'duration_requested_ms': duration_ms, 'source': 'K230 camera',
                'recording_only': True, 'audio': False, 'detection_during_recording': False,
                'timestamp_unit': 'microseconds', 'timestamps_file': 'timestamps.jsonl'}
    with open(directory + '/metadata.json', 'w') as f: json.dump(metadata, f)
    sensor = None; encoder = None; binding = None
    encoder_created = False; encoder_started = False; sensor_started = False
    first_pts = None; last_pts = None; frames = 0; max_write_ms = 0
    try:
        sensor = Sensor()
        sensor.reset()
        sensor.set_framesize(width=1280, height=960, alignment=12)
        sensor.set_pixformat(Sensor.YUV420SP)
        sensor.set_hmirror(False)
        sensor.set_vflip(False)
        encoder = Encoder()
        encoder.SetOutBufs(8, 1280, 960)
        encoder.Create(ChnAttrStr(encoder.PAYLOAD_TYPE_H264, encoder.H264_PROFILE_MAIN,
                                  1280, 960, bit_rate=4096, gopLen=30,
                                  src_frame_rate=30, dst_frame_rate=30))
        encoder_created = True
        binding = MediaManager.link(sensor.bind_info()['src'],
                                   (VIDEO_ENCODE_MOD_ID, VENC_DEV_ID, encoder.chn))
        stream = StreamData()
        encoder.Start(); encoder_started = True
        sensor.run(); sensor_started = True
        started = time.ticks_ms()
        print('CAR_VIDEO_READY')
        with open(partial, 'wb') as video, open(directory + '/timestamps.jsonl', 'w') as stamps:
            while time.ticks_diff(time.ticks_ms(), started) < duration_ms:
                os.exitpoint()
                if encoder.GetStream(stream, timeout=1000) != 0:
                    continue
                try:
                    for index in range(stream.pack_cnt):
                        writing = time.ticks_ms()
                        # Include every SPS/PPS header and every key/P frame,
                        # exactly as emitted by the encoder. No image encoding.
                        payload = uctypes.bytearray_at(stream.data[index], stream.data_size[index])
                        write_video_packet(video, payload)
                        max_write_ms = max(max_write_ms, time.ticks_diff(time.ticks_ms(), writing))
                        if stream.stream_type[index] != encoder.STREAM_TYPE_HEADER:
                            if first_pts is None: first_pts = stream.pts[index]
                            last_pts = stream.pts[index]
                            frames += 1
                            stamps.write(json.dumps({'frame_id': frames, 'pts_us': last_pts}) + '\n')
                finally:
                    encoder.ReleaseStream(stream)
        elapsed = time.ticks_diff(time.ticks_ms(), started)
    finally:
        try:
            if sensor_started: sensor.stop()
        finally:
            try:
                if binding is not None: binding.destroy()
            finally:
                try:
                    if encoder_started: encoder.Stop()
                finally:
                    if encoder_created: encoder.Destroy()
    if frames == 0: raise RuntimeError('No video frames recorded; partial retained')
    os.rename(partial, target)
    summary = dict(metadata, frames=frames, elapsed_ms=elapsed,
                   first_pts=first_pts, last_pts=last_pts, max_stream_write_ms=max_write_ms,
                   path=target, bytes=os.stat(target)[6])
    with open(directory + '/summary.json', 'w') as f: json.dump(summary, f)
    print('CAR_VIDEO_DONE ' + json.dumps(summary))
