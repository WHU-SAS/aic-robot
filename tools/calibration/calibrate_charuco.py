#!/usr/bin/env python3
"""Calibrate camera intrinsics from full-resolution views of the supplied board."""

import sys as _sys
from pathlib import Path as _Path
_sys.path.insert(0, str(_Path(__file__).resolve().parents[2]))
from host.project_paths import ROOT as REPO_ROOT, bootstrap as _bootstrap
_bootstrap(include_tools=True)

import argparse
import json
import math
from pathlib import Path

import cv2
import numpy as np

from charuco_common import DICTIONARY_NAME, SQUARES_X, SQUARES_Y, detect, make_board

ROOT = REPO_ROOT


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--images', type=Path, default=ROOT / 'calibration' / 'images')
    parser.add_argument('--square-mm', type=float, required=True, help='measured printed square side length')
    parser.add_argument('--output', type=Path, default=ROOT / 'calibration' / 'results' / 'camera_intrinsics.json')
    parser.add_argument('--min-views', type=int, default=12)
    args = parser.parse_args()
    if not math.isfinite(args.square_mm) or not 1 <= args.square_mm <= 100:
        parser.error('--square-mm must be a finite measured length in 1..100 mm')
    if not 8 <= args.min_views <= 100:
        parser.error('--min-views must be 8..100')
    board, detector = make_board(args.square_mm)
    paths = sorted(p for p in args.images.glob('*') if p.suffix.lower() in ('.jpg', '.jpeg', '.png'))
    object_points = []
    image_points = []
    accepted = []
    rejected = []
    image_size = None
    all_corners = board.getChessboardCorners()
    for path in paths:
        image = cv2.imread(str(path))
        if image is None:
            rejected.append((path.name, 'could not read image'))
            continue
        size = (image.shape[1], image.shape[0])
        if image_size is None:
            image_size = size
        if size != image_size:
            rejected.append((path.name, f'resolution {size} differs from {image_size}'))
            continue
        corners, ids, count, markers, _, _ = detect(image, detector)
        if count < 10:
            rejected.append((path.name, f'only {count} corners, {markers} markers'))
            continue
        idx = ids.flatten().astype(np.int32)
        object_points.append(all_corners[idx].astype(np.float32))
        image_points.append(corners.reshape(-1, 2).astype(np.float32))
        accepted.append({'file': path.name, 'corners': count, 'markers': markers})
    if len(accepted) < args.min_views:
        raise SystemExit(f'need at least {args.min_views} accepted views; got {len(accepted)} from {len(paths)} files; rejected={rejected}')
    rms, matrix, distortion, rvecs, tvecs = cv2.calibrateCamera(
        object_points, image_points, image_size, None, None
    )
    if not (math.isfinite(rms) and np.all(np.isfinite(matrix)) and np.all(np.isfinite(distortion))):
        raise RuntimeError('calibration returned non-finite parameters')
    errors = []
    for objects, points, rvec, tvec in zip(object_points, image_points, rvecs, tvecs):
        projected, _ = cv2.projectPoints(objects, rvec, tvec, matrix, distortion)
        delta = projected.reshape(-1, 2) - points
        errors.append(float(np.sqrt(np.mean(np.sum(delta * delta, axis=1)))))
    for view, error in zip(accepted, errors):
        view['reprojection_rms_px'] = error
    result = {
        'image_width': image_size[0], 'image_height': image_size[1],
        'camera_matrix': matrix.tolist(),
        'distortion_coefficients': distortion.reshape(-1).tolist(),
        'rms_reprojection_error_px': float(rms),
        'board': {'squares_x': SQUARES_X, 'squares_y': SQUARES_Y,
                  'square_mm_measured': args.square_mm,
                  'marker_mm_assumed': args.square_mm / 2.0,
                  'dictionary': DICTIONARY_NAME},
        'opencv_version': cv2.__version__,
        'accepted_views': accepted,
        'rejected_views': [{'file': name, 'reason': reason} for name, reason in rejected],
    }
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(result, ensure_ascii=False, indent=2) + '\n', encoding='utf-8')
    print(f'accepted {len(accepted)}/{len(paths)} views; image size {image_size}; RMS {rms:.3f} px')
    print(f'saved {args.output}')
    if rms > 1.0:
        print('check: reprojection RMS exceeds 1 px; review blur, coverage and printed flatness')


if __name__ == '__main__':
    main()
