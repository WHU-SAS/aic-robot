"""Offline CanMV-blob approximation; this is not board LAB/RGB565 parity."""
import cv2
import numpy as np


class Blob:
    def __init__(self, rect, pixels, center):
        self._rect, self._pixels, self._center = rect, pixels, center
    def rect(self): return self._rect
    def pixels(self): return self._pixels
    def cx(self): return self._center[0]
    def cy(self): return self._center[1]


class Image:
    def __init__(self, bgr):
        if bgr is None or bgr.shape[:2] != (960, 1280):
            raise ValueError('Expected a 1280x960 saved K230 image')
        self.bgr = bgr

    def find_blobs(self, thresholds, roi, pixels_threshold, area_threshold, merge=False):
        if merge:
            raise ValueError('Coarse tracker expects independent components')
        lab = cv2.cvtColor(self.bgr, cv2.COLOR_BGR2LAB)
        x0, y0, width, height = roi
        mask = np.zeros(lab.shape[:2], np.uint8)
        for L0, L1, a0, a1, b0, b1 in thresholds:
            mask |= cv2.inRange(lab, (round(L0 * 255 / 100), a0 + 128, b0 + 128),
                               (round(L1 * 255 / 100), a1 + 128, b1 + 128))
        mask[:y0] = 0; mask[y0 + height:] = 0
        mask[:, :x0] = 0; mask[:, x0 + width:] = 0
        count, unused, stats, centers = cv2.connectedComponentsWithStats(mask, 8)
        blobs = []
        for index in range(1, count):
            x, y, w, h, pixels = [int(v) for v in stats[index]]
            if pixels >= pixels_threshold and w * h >= area_threshold:
                blobs.append(Blob([x, y, w, h], pixels, centers[index].tolist()))
        return blobs
