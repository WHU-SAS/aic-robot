"""SVG demonstration overlay; preserves the original K230 JPEG unchanged."""
import base64
from html import escape
import math


def visible_table_path(geometry, bottom, foot, width, height):
    """Sample a straight TABLE segment, then clip its distorted image curve.

    The camera foot is far below this camera's field of view. Projecting that
    point directly extrapolates lens distortion outside calibration support.
    Stop at the first image boundary instead of drawing to huge coordinates.
    """
    points = [geometry.base_to_pixel(bottom)]
    for index in range(1, 513):
        point = [bottom[k] + (foot[k] - bottom[k]) * index / 512 for k in range(3)]
        try: q = geometry.base_to_pixel(point)
        except ValueError: break
        if not all(math.isfinite(v) for v in q): break
        if not (0 <= q[0] <= width - 1 and 0 <= q[1] <= height - 1):
            p = points[-1]; t = 1.
            for axis, limit in ((0, width - 1), (1, height - 1)):
                if q[axis] < 0: t = min(t, -p[axis] / (q[axis] - p[axis]))
                elif q[axis] > limit: t = min(t, (limit - p[axis]) / (q[axis] - p[axis]))
            points.append([p[k] + t * (q[k] - p[k]) for k in range(2)])
            return list(reversed(points)), True
        points.append(q)
    return list(reversed(points)), False


def render(jpeg, observation, geometry, width=1280, height=960):
    bbox = observation['bbox_px']
    center = observation['center_pixel']
    bottom = geometry.estimated_bottom_from_image_center(center)
    foot = geometry.camera_table_foot()
    distance = geometry.table_distance(bottom)
    path, offscreen = visible_table_path(geometry, bottom, foot, width, height)
    end = geometry.base_to_pixel(bottom)
    if not all(math.isfinite(v) for v in end):
        raise ValueError('Distance line cannot be projected')
    x, y, w, h = bbox
    path_string = ' '.join('%.3f,%.3f' % tuple(p) for p in path)
    label = '桌面平面距离（估计）：%.0f mm' % distance
    foot_label = '摄像头垂足在画面外' if offscreen else '摄像头垂足在画面内'
    encoded = base64.b64encode(jpeg).decode()
    svg = '''<svg xmlns="http://www.w3.org/2000/svg" width="%d" height="%d" viewBox="0 0 %d %d">
<image width="%d" height="%d" href="data:image/jpeg;base64,%s"/>
<rect x="%s" y="%s" width="%s" height="%s" fill="none" stroke="#00ff00" stroke-width="2"/>
<circle cx="%s" cy="%s" r="4" fill="#ff3030"/>
<polyline points="%s" fill="none" stroke="#ffcf33" stroke-width="3" stroke-dasharray="12 8"/>
<circle cx="%.3f" cy="%.3f" r="6" fill="none" stroke="#ff3030" stroke-width="2"/>
<rect x="12" y="12" width="670" height="92" fill="white" fill-opacity="0.88"/>
<g fill="#102040" font-family="sans-serif" font-size="22"><text x="24" y="42">%s</text><text x="24" y="68">中心(%.0f, %.0f) px · 蓝色面积 %s px</text><text x="24" y="94">%s · 红圈为底面中心估计</text></g>
</svg>''' % (width, height, width, height, width, height, encoded, x, y, w, h,
             center[0], center[1], path_string, end[0], end[1],
             escape(label), center[0], center[1], observation['pixels'], escape(foot_label))
    metadata = {'distance_mm': distance, 'distance_basis': 'estimated planar distance from camera optical-center foot to cube bottom-center proxy',
        'camera_table_foot_Base_mm': foot, 'estimated_bottom_center_Base_mm': bottom,
        'camera_foot_outside_image': offscreen, 'exact_bottom_center_measured': False,
        'image_area_unit': 'blue pixels, not physical square mm'}
    return svg, metadata
