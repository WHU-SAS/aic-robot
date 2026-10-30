#!/usr/bin/env python3
"""Analyze raw K230 H.264/MP4 on PC with frame-matched demonstration labels."""
import sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parents[2]))
import argparse
import hashlib
import json
import math
import time
import cv2
from device.k230.car.blue_tracker import BlueTracker
from host.car.runtime import load
from host.car.image_adapter import Image
from host.car.distance_demo import visible_table_path
from host.car.video_labels import draw_label


def dashed_path(image, points):
    phase = 0.
    for a, b in zip(points, points[1:]):
        dx, dy = b[0] - a[0], b[1] - a[1]
        length = math.hypot(dx, dy)
        offset = 0.
        while offset < length - 1e-6:
            take = min(length - offset, (12 if phase < 12 else 20) - phase)
            if phase < 12:
                p = tuple(round(a[k] + (b[k] - a[k]) * offset / length) for k in range(2))
                q = tuple(round(a[k] + (b[k] - a[k]) * (offset + take) / length) for k in range(2))
                cv2.line(image, p, q, (51, 207, 255), 2, cv2.LINE_AA)
            offset += take
            phase = (phase + take) % 20


def annotate(source, output, trim_start_s=1., table_contact_intervals=()):
    source, output = Path(source), Path(output)
    if not math.isfinite(trim_start_s) or trim_start_s < 0:
        raise ValueError('Trim start must be nonnegative seconds')
    table_contact_intervals = [tuple(interval) for interval in table_contact_intervals]
    if any(len(interval) != 2 or not all(math.isfinite(v) for v in interval)
           or interval[0] < 0 or interval[1] <= interval[0]
           for interval in table_contact_intervals):
        raise ValueError('Table-contact intervals need increasing nonnegative source seconds')
    if output.suffix.lower() != '.mp4': raise ValueError('Output must end in .mp4')
    partial = output.with_name(output.stem + '.partial.mp4')
    observations = output.with_suffix('.frames.jsonl')
    report = output.with_suffix('.json')
    if any(p.exists() for p in (output, partial, observations, report)):
        raise ValueError('Output already exists; choose a new filename')
    _, config, geometry = load()
    tracker = BlueTracker(config['tracking'], geometry)
    capture = cv2.VideoCapture(str(source))
    if not capture.isOpened(): raise ValueError('Cannot open source video')
    fps = capture.get(cv2.CAP_PROP_FPS)
    raw_timestamps = None
    if source.suffix.lower() in ('.h264', '.264'):
        raw_timestamps = [json.loads(line)['pts_us'] / 1000 for line in
                          (source.parent / 'timestamps.jsonl').read_text().splitlines() if line.strip()]
        if len(raw_timestamps) < 2 or any(b <= a for a, b in zip(raw_timestamps, raw_timestamps[1:])):
            capture.release()
            raise ValueError('Raw H.264 needs increasing hardware timestamps')
        # Elementary H.264 has no container timestamps. Use recorded encoder
        # PTS, rather than the decoder's guessed (often 25fps) playback rate.
        fps = (len(raw_timestamps) - 1) * 1000 / (raw_timestamps[-1] - raw_timestamps[0])
    size = (round(capture.get(cv2.CAP_PROP_FRAME_WIDTH)), round(capture.get(cv2.CAP_PROP_FRAME_HEIGHT)))
    if size != (1280, 960) or not math.isfinite(fps) or not 1 <= fps <= 120:
        capture.release()
        raise ValueError('Expected unmirrored K230 1280x960 video with valid FPS')
    metadata_path = source.parent / 'metadata.json'
    if metadata_path.exists():
        metadata = json.loads(metadata_path.read_text())
        if metadata.get('hmirror') is not False or metadata.get('vflip') is not False:
            capture.release()
            raise ValueError('Video orientation does not match camera calibration')
    output.parent.mkdir(parents=True, exist_ok=True)
    writer = cv2.VideoWriter(str(partial), cv2.VideoWriter_fourcc(*'mp4v'), fps, (1280, 1040))
    if not writer.isOpened():
        capture.release()
        raise RuntimeError('MP4 encoder unavailable')
    count = 0; decoded = 0; states = {}; first_ms = None; previous_ms = None
    clip_start_ms = None
    try:
        with observations.open('w') as events:
            while True:
                ok, image = capture.read()
                if not ok: break
                if image.shape[:2] != (960, 1280): raise ValueError('Video resolution changed')
                if raw_timestamps is not None and decoded >= len(raw_timestamps):
                    raise ValueError('Decoded frame count exceeds recorded timestamps')
                stamp = raw_timestamps[decoded] if raw_timestamps is not None else capture.get(cv2.CAP_PROP_POS_MSEC)
                decoded += 1
                if not math.isfinite(stamp) or (previous_ms is not None and stamp <= previous_ms):
                    raise ValueError('Source video timestamps must increase')
                if first_ms is None: first_ms = stamp
                if stamp - first_ms < trim_start_s * 1000:
                    previous_ms = stamp
                    continue
                if clip_start_ms is None: clip_start_ms = stamp
                started = time.perf_counter()
                result = tracker.detect_frame(Image(image), stamp - first_ms)
                count += 1
                row = {key: result.get(key) for key in ('status', 'bbox_px', 'center_pixel',
                    'pixels', 'image_midline_error_px', 'confirmations')}
                row.update(frame_id=decoded, capture_timestamp_ms=stamp - first_ms,
                           clock_domain='source_video_PTS', PC_offline_analysis=True,
                           processing_ms=round((time.perf_counter() - started) * 1000, 1),
                           frame_period_ms=stamp - previous_ms if previous_ms is not None else None,
                           estimated_table_distance_mm=None)
                row['table_contact_verified'] = any(a <= (stamp - first_ms) / 1000 < b
                                                    for a, b in table_contact_intervals)
                row['table_contact_basis'] = 'Manually reviewed source-time intervals; not automatic height detection'
                previous_ms = stamp
                states[row['status']] = states.get(row['status'], 0) + 1
                if row.get('bbox_px'):
                    x, y, w, h = row['bbox_px']
                    color = (0, 255, 0) if row['status'] == 'tracking' else (0, 180, 255)
                    cv2.rectangle(image, (x, y), (x + w, y + h), color, 2)
                    center = tuple(round(v) for v in row['center_pixel'])
                    cv2.drawMarker(image, center, (0, 0, 255), cv2.MARKER_CROSS, 12, 2)
                    try:
                        if not row['table_contact_verified']:
                            raise ValueError('Support-plane assumption unverified for this frame')
                        bottom = geometry.estimated_bottom_from_image_center(row['center_pixel'])
                        row['estimated_table_distance_mm'] = geometry.table_distance(bottom)
                        path, offscreen = visible_table_path(geometry, bottom, geometry.camera_table_foot(), 1280, 960)
                        row['camera_foot_outside_image'] = offscreen
                        dashed_path(image, path)
                        pixel = tuple(round(v) for v in geometry.base_to_pixel(bottom))
                        cv2.circle(image, pixel, 5, (0, 0, 255), 2, cv2.LINE_AA)
                    except ValueError:
                        pass
                canvas = cv2.copyMakeBorder(image, 80, 0, 0, 0, cv2.BORDER_CONSTANT, value=(245, 245, 245))
                cv2.putText(canvas, 'K230 BLUE CUBE | PC OFFLINE ANALYSIS', (16, 29),
                            cv2.FONT_HERSHEY_SIMPLEX, .7, (60, 40, 20), 1, cv2.LINE_AA)
                cv2.putText(canvas, 'Frame %d | %.2f s | %s' % (decoded, (stamp - clip_start_ms) / 1000, row['status']),
                            (16, 62), cv2.FONT_HERSHEY_SIMPLEX, .65, (60, 40, 20), 1, cv2.LINE_AA)
                writer.write(draw_label(canvas, row))
                events.write(json.dumps(row, ensure_ascii=False) + '\n')
                if count % 150 == 0: print('已处理 %d 帧（%.1f秒）' % (count, count / fps), flush=True)
    finally:
        writer.release()
        capture.release()
    if not count: raise ValueError('No frames remain after trimming')
    if raw_timestamps is not None and decoded != len(raw_timestamps):
        raise ValueError('Decoded frame count does not match recorded timestamps')
    check = cv2.VideoCapture(str(partial))
    valid = check.isOpened() and round(check.get(cv2.CAP_PROP_FRAME_COUNT)) == count
    check.release()
    if not valid: raise RuntimeError('Export validation failed; partial retained')
    partial.replace(output)
    digest = hashlib.sha256()
    with source.open('rb') as raw:
        for block in iter(lambda: raw.read(1024 * 1024), b''): digest.update(block)
    summary = {'source': str(source), 'source_sha256': digest.hexdigest(), 'output': str(output),
               'source_frames': decoded, 'frames': count, 'fps': fps, 'duration_s': count / fps, 'states': states,
               'trim_start_requested_s': trim_start_s, 'trim_start_actual_s': (clip_start_ms - first_ms) / 1000,
               'analysis': 'PC offline LAB approximation, not native K230 timing or real-time validation',
               'playback_speed': 'All frames after initial warmup trim retained; FPS from original hardware PTS for H264',
               'distance': 'Calibrated camera-foot to estimated cube-bottom proxy; not exact grasp position',
               'table_contact_intervals_s': table_contact_intervals,
               'distance_gate': 'Distance only displayed in manually reviewed table-contact intervals; disabled elsewhere',
               'tracking': config['tracking'], 'source_modified': False,
               'calibration': {'K': geometry.K, 'D': geometry.D, 'T': geometry.T,
                               'plane': {'normal': geometry.normal, 'offset_mm': geometry.offset}}}
    report.write_text(json.dumps(summary, ensure_ascii=False, indent=2) + '\n')
    return summary


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--input', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--trim-start', type=float, default=1., help='Trim camera warmup seconds, default 1')
    parser.add_argument('--table-contact-interval', type=float, nargs=2, action='append', default=[],
                        metavar=('START', 'END'), help='Manually verified table-contact source seconds; repeatable. No distance is shown by default.')
    args = parser.parse_args()
    print(json.dumps(annotate(args.input, args.output, args.trim_start, args.table_contact_interval), ensure_ascii=False))


if __name__ == '__main__': main()
