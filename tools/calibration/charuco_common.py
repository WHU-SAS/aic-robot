"""The exact OpenCV A4 board stored under assets/calibration."""

import sys as _sys
from pathlib import Path as _Path
_sys.path.insert(0, str(_Path(__file__).resolve().parents[2]))
from host.project_paths import ROOT as REPO_ROOT, bootstrap as _bootstrap
_bootstrap(include_tools=True)

import cv2

SQUARES_X = 5
SQUARES_Y = 7
DICTIONARY_NAME = 'DICT_5X5_100'


def make_board(square_mm: float):
    if not 1.0 <= square_mm <= 100.0:
        raise ValueError('measured square size must be between 1 and 100 mm')
    dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_5X5_100)
    board = cv2.aruco.CharucoBoard(
        (SQUARES_X, SQUARES_Y), float(square_mm), float(square_mm) / 2.0, dictionary
    )
    board.setLegacyPattern(False)
    return board, cv2.aruco.CharucoDetector(board)


def detect(image, detector):
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    corners, ids, marker_corners, marker_ids = detector.detectBoard(gray)
    count = 0 if ids is None else len(ids)
    markers = 0 if marker_ids is None else len(marker_ids)
    return corners, ids, count, markers, marker_corners, marker_ids
