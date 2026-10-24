#!/usr/bin/env python3
"""Annotate real saved K230 photos with an explicitly estimated table distance."""
import sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parents[2]))
import argparse
import json
import cv2
from device.k230.car.blue_tracker import BlueTracker
from host.car.runtime import load
from host.car.image_adapter import Image
from host.car.distance_demo import render


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--image', type=Path, action='append', required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--native-report', type=Path,
                        help='Use completed K230 saved-image observations instead of PC LAB')
    args = parser.parse_args()
    _, config, geometry = load(); args.output.mkdir(parents=True, exist_ok=True)
    native = None
    if args.native_report:
        report = json.loads(args.native_report.read_text())
        if report.get('status') != 'native_saved_image_replay_completed':
            parser.error('Native replay did not complete')
        if len(report['replays']) != len(args.image): parser.error('Image/report counts differ')
        native = report['replays']
    rows = []
    for number, path in enumerate(args.image, 1):
        photo = cv2.imread(str(path))
        row = {'source': str(path), 'quality_review': 'pending_visual_review', 'PC_LAB_approximation': native is None}
        if photo is None or photo.shape[:2] != (960, 1280):
            row.update(status='unusable_decode_or_resolution', reason='Not the expected calibrated image')
        else:
            if native is not None:
                replay = native[number - 1]
                if Path(replay['path']).name != path.name: parser.error('Image/report order differs')
                observation = replay['observation']
                row.update(native_source=replay['path'], native_frame_ms=replay['frame_ms'])
            else:
                observation = BlueTracker(config['tracking'], geometry).detect_frame(Image(photo), 0)
            row.update(observation=observation, status='detection_failed_retained_for_review')
            if observation.get('bbox_px'):
                # One still is not temporal confirmation. Annotate the candidate,
                # but never change confirming into tracking/localized.
                svg, metadata = render(path.read_bytes(), observation, geometry)
                target = args.output / ('frame_%02d.svg' % number)
                target.write_text(svg)
                row.update(status='candidate_annotated_not_live_tracking_validated', annotation=str(target), **metadata)
        rows.append(row)
    (args.output / 'summary.json').write_text(json.dumps(rows, ensure_ascii=False, indent=2) + '\n')
    print(json.dumps({'images': len(rows), 'annotated': sum('annotation' in row for row in rows)}, ensure_ascii=False))


if __name__ == '__main__': main()
