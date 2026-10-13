#!/usr/bin/env python3
"""PC-side selection for the project's 5x7 DICT_5X5_100 ChArUco board.

Keep all original images. Copy only fully detected patterns to a new directory.
"""
import argparse
from datetime import datetime
import json
from pathlib import Path
import shutil
import cv2
import numpy as np


def inspect(image, board, detector, margin):
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    corners, ids, marker_corners, marker_ids = detector.detectBoard(gray)
    actual_corners = set() if ids is None else set(map(int, ids.flatten()))
    actual_markers = set() if marker_ids is None else set(map(int, marker_ids.flatten()))
    expected_markers = set(map(int, board.getIds().flatten()))
    record = {'corners': len(actual_corners), 'expected_corners': 24,
              'markers': len(actual_markers & expected_markers),
              'expected_markers': len(expected_markers),
              'sharpness_laplacian_variance': float(cv2.Laplacian(gray, cv2.CV_64F).var()),
              'accepted': False}
    if actual_corners != set(range(24)) or not expected_markers <= actual_markers:
        record['reason'] = 'incomplete_or_unreadable_pattern'
        return record
    objects = board.getChessboardCorners()[ids.flatten(), :2]
    homography, _ = cv2.findHomography(objects, corners.reshape(-1, 2), 0)
    if homography is None or not np.isfinite(homography).all():
        record['reason'] = 'invalid_board_projection'
        return record
    # Square length is 1 here: only the board's geometry is needed to screen views.
    outline = np.array([[[0., 0.], [5., 0.], [5., 7.], [0., 7.]]], np.float32)
    projected = cv2.perspectiveTransform(outline, homography).reshape(-1, 2)
    height, width = gray.shape
    inside = (np.isfinite(projected).all()
              and cv2.isContourConvex(projected.astype(np.float32))
              and (projected[:, 0] >= margin).all()
              and (projected[:, 0] <= width - 1 - margin).all()
              and (projected[:, 1] >= margin).all()
              and (projected[:, 1] <= height - 1 - margin).all())
    record['projected_pattern_boundary_px'] = projected.tolist()
    record['accepted'] = bool(inside)
    record['reason'] = 'complete_pattern' if inside else 'pattern_boundary_at_edge'
    return record


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('images', type=Path, help='Exported K230 session directory')
    parser.add_argument('--margin', type=int, default=3)
    args = parser.parse_args()
    if args.margin < 0:
        parser.error('--margin must be nonnegative')
    files = sorted(p for p in args.images.iterdir()
                   if p.suffix.lower() in {'.jpg', '.jpeg', '.png', '.bmp'}
                   and '.partial.' not in p.name)
    if not files:
        parser.error('No completed images found')
    dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_5X5_100)
    board = cv2.aruco.CharucoBoard((5, 7), 1., .5, dictionary)
    board.setLegacyPattern(False)
    detector = cv2.aruco.CharucoDetector(board)
    output = args.images / ('selection_' + datetime.now().strftime('%Y%m%d_%H%M%S_%f'))
    output.mkdir()
    accepted = output / 'accepted'
    accepted.mkdir()
    records = []
    for path in files:
        image = cv2.imread(str(path))
        if image is None:
            record = {'accepted': False, 'reason': 'unreadable_image'}
        else:
            record = inspect(image, board, detector, args.margin)
            record['image_size'] = [image.shape[1], image.shape[0]]
        record['file'] = path.name
        records.append(record)
        if record['accepted']:
            shutil.copy2(path, accepted / path.name)
    count = sum(r['accepted'] for r in records)
    report = {'board': '5x7 ChArUco / DICT_5X5_100 / legacy=False',
              'accepted': count, 'total': len(records), 'margin_px': args.margin,
              'note': 'Completeness screening only; sharpness is reported, not thresholded. Originals retained.',
              'images': records}
    (output / 'selection.json').write_text(json.dumps(report, ensure_ascii=False, indent=2))
    print(f'Accepted {count}/{len(records)}; originals retained. Output: {output}')


if __name__ == '__main__':
    main()
