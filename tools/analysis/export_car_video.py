#!/usr/bin/env python3
"""Export copied K230 SD recording to MP4, preserving observed elapsed time."""
import sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parents[2]))
import argparse
import json
import math
import cv2
from host.car.video_timing import frame_times
from host.car.video_labels import draw_label


def export(directory, output, fps=10):
    directory, output = Path(directory), Path(output)
    if output.exists(): raise ValueError('Output exists; choose a new video filename')
    if output.suffix.lower() != '.mp4': raise ValueError('Output must end with .mp4')
    lines = (directory / 'frames.jsonl').read_text().splitlines()
    rows = []
    for index, line in enumerate(lines):
        if not line.strip(): continue
        try: row = json.loads(line)
        except json.JSONDecodeError:
            if index == len(lines) - 1: break  # Interrupted final metadata line.
            raise
        name = row['filename']
        if Path(name).name != name or not name.endswith('.jpg'):
            raise ValueError('Invalid recording frame filename')
        if not (directory / name).is_file(): raise ValueError('Missing video frame: ' + name)
        rows.append(row)
    times, duration = frame_times(rows, fps)
    first = cv2.imread(str(directory / rows[0]['filename']))
    if first is None: raise ValueError('Cannot decode first frame')
    height, width = first.shape[:2]
    output.parent.mkdir(parents=True, exist_ok=True)
    partial = output.with_name(output.stem + '.partial.mp4')
    if partial.exists(): raise ValueError('Partial output exists; use a new output name')
    writer = cv2.VideoWriter(str(partial), cv2.VideoWriter_fourcc(*'mp4v'), fps, (width, height + 80))
    if not writer.isOpened(): raise RuntimeError('MP4 encoder could not start')
    count = math.ceil(duration * fps)
    current = -1; selected = 0; canvas = None
    try:
        for number in range(count):
            second = number / fps
            while selected + 1 < len(rows) and times[selected + 1] <= second + 1e-9:
                selected += 1
            if selected != current:
                row = rows[selected]
                image = first if selected == 0 else cv2.imread(str(directory / row['filename']))
                if image is None or image.shape[:2] != (height, width):
                    raise ValueError('Invalid video frame: ' + row['filename'])
                canvas = cv2.copyMakeBorder(image, 80, 0, 0, 0, cv2.BORDER_CONSTANT, value=(245, 245, 245))
                top = 'K230 - BLUE CUBE LIVE TRACKING'
                bottom = 'Frame %s | %s' % (row.get('frame_id'), row['status'])
                for y, text in ((29, top), (63, bottom)):
                    cv2.putText(canvas, text, (16, y), cv2.FONT_HERSHEY_SIMPLEX, .65, (60, 40, 20), 1, cv2.LINE_AA)
                canvas = draw_label(canvas, row)
                current = selected
            writer.write(canvas)
    finally:
        writer.release()
    check = cv2.VideoCapture(str(partial))
    valid = check.isOpened() and int(check.get(cv2.CAP_PROP_FRAME_COUNT)) == count
    check.release()
    if not valid: raise RuntimeError('Encoded video validation failed; partial retained')
    partial.replace(output)
    summary = {'input_directory': str(directory), 'source_frames': len(rows),
        'output': str(output), 'output_frames': count, 'fps': fps,
        'recorded_duration_s': duration, 'encoded_duration_s': count / fps,
        'timing': 'Actual capture periods resampled; hold preceding frame across gaps',
        'codec': 'MPEG-4 Part 2 (mp4v)', 'source_frames_modified': False,
        'object_labels': 'Box-anchored labels use this exact recorded frame metadata; no labels on lost/no_target'}
    output.with_suffix('.json').write_text(json.dumps(summary, ensure_ascii=False, indent=2) + '\n')
    return summary


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--input', type=Path, required=True, help='Copied SD session with frames.jsonl')
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--fps', type=float, default=10)
    args = parser.parse_args()
    print(json.dumps(export(args.input, args.output, args.fps), ensure_ascii=False))


if __name__ == '__main__': main()
