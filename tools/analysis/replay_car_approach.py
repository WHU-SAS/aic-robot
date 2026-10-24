#!/usr/bin/env python3
"""Replay coarse car vision/decisions on saved photos; never opens hardware."""
import sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parents[2]))
import argparse
import hashlib
import json
import time
import cv2
import numpy as np
from host.project_paths import ROOT
from host.car.runtime import load
from host.car.image_adapter import Image
from device.k230.car.blue_tracker import BlueTracker
from device.k230.car.approach import ApproachPlanner


def synthetic_cube(geometry, x, y):
    n = np.array(geometry.normal, float)
    forward = np.array([1., 0., -n[0] / n[2]])
    forward /= np.linalg.norm(forward)
    right = np.cross(forward, n); right /= np.linalg.norm(right)
    contact = np.array([x, y, (geometry.offset - n[0] * x - n[1] * y) / n[2]])
    vertices = np.array([contact + sx * 10 * forward + sy * 10 * right + z * 20 * n
                         for z in (0, 1) for sx, sy in ((-1, -1), (-1, 1), (1, 1), (1, -1))])
    faces = [([0, 1, 2, 3], -n, (130, 60, 8)), ([4, 5, 6, 7], n, (245, 105, 5)),
             ([0, 4, 5, 1], -forward, (180, 70, 5)), ([3, 2, 6, 7], forward, (150, 55, 4)),
             ([0, 3, 7, 4], -right, (165, 50, 3)), ([1, 5, 6, 2], right, (145, 45, 2))]
    origin = np.array(geometry.origin)
    im = np.full((960, 1280, 3), (110, 160, 190), np.uint8)
    visible = [(np.linalg.norm(np.mean(vertices[ids], axis=0) - origin), ids, color)
               for ids, normal, color in faces if normal @ (origin - np.mean(vertices[ids], axis=0)) > 0]
    for unused, ids, color in sorted(visible, reverse=True):
        q = np.array([geometry.base_to_pixel(v.tolist()) for v in vertices[ids]])
        cv2.fillConvexPoly(im, np.rint(q).astype(np.int32), color)
    return cv2.GaussianBlur(im, (3, 3), .7)


def replay(image, label, folder, config, workspace, geometry, source=None):
    tracker = BlueTracker(config['tracking'], geometry)
    adapter = Image(image); times = []
    # Deliberately reuse a still only for deterministic algorithm testing.
    # This does not validate real temporal tracking or K230 execution time.
    for index in range(config['tracking']['confirmation_frames']):
        started = time.perf_counter(); observation = tracker.detect_frame(adapter, index * 50)
        times.append((time.perf_counter() - started) * 1000)
    plan = ApproachPlanner(workspace, config).coarse(observation,
        (config['tracking']['confirmation_frames'] - 1) * 50)
    marked = image.copy()
    cv2.line(marked, (640, 0), (640, 959), (180, 180, 180), 1)
    if observation.get('bbox_px'):
        x, y, w, h = observation['bbox_px']
        cv2.rectangle(marked, (x, y), (x + w - 1, y + h - 1), (0, 255, 0), 2)
        cv2.circle(marked, tuple(round(v) for v in observation['center_pixel']), 4, (0, 0, 255), -1)
    cv2.putText(marked, label + ': ' + plan['intent'], (20, 35), cv2.FONT_HERSHEY_SIMPLEX, .7, (20, 30, 30), 2)
    output = folder / (label + '.png'); cv2.imwrite(str(output), marked)
    row = {'label': label, 'source': str(source) if source else 'synthetic calibrated 20mm cube',
           'source_sha256': hashlib.sha256(Path(source).read_bytes()).hexdigest() if source else None,
           'observation': observation, 'plan': plan, 'PC_frame_ms': times, 'marked_image': str(output),
           'repeated_still_not_real_temporal_validation': True, 'K230_timing_measured': False}
    (folder / (label + '.json')).write_text(json.dumps(row, ensure_ascii=False, indent=2) + '\n')
    return row


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--image', type=Path, action='append', default=[])
    parser.add_argument('--synthetic', action='store_true')
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args(argv)
    workspace, config, geometry = load(); args.output.mkdir(parents=True, exist_ok=True)
    rows = []
    for index, path in enumerate(args.image):
        image = cv2.imread(str(path))
        if image is None: parser.error('Cannot read ' + str(path))
        rows.append(replay(image, 'saved_%02d' % index, args.output, config, workspace, geometry, path))
    if args.synthetic:
        line = workspace['centerline']['local_Base_Y_line']
        for distance in (500, 750, 1000):
            for side, shift in (('center', 0), ('left', 100), ('right', -100)):
                y = line['slope_per_X'] * distance + line['intercept_mm'] + shift
                image = synthetic_cube(geometry, distance, y)
                row = replay(image, 'synthetic_%d_%s' % (distance, side), args.output, config, workspace, geometry)
                row['truth_Base_XY_mm'] = [distance, y]; rows.append(row)
        for label, image in [('no_target', np.full((960, 1280, 3), (110, 160, 190), np.uint8)),
                             ('uniform_blue', np.full((960, 1280, 3), (200, 60, 5), np.uint8))]:
            rows.append(replay(image, label, args.output, config, workspace, geometry))
    summary = {'hardware_access': False, 'motion_performed': False, 'frames': rows,
               'caveat': 'PC LAB approximation and repeated/synthetic stills; not CanMV parity, real remote detection, or physical stopping validation'}
    (args.output / 'summary.json').write_text(json.dumps(summary, ensure_ascii=False, indent=2) + '\n')
    print(json.dumps({'frames': len(rows), 'decisions': {row['label']: row['plan']['intent'] for row in rows},
                      'record': str(args.output / 'summary.json')}, ensure_ascii=False))


if __name__ == '__main__': main()
