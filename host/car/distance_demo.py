"""SVG demonstration overlay; preserves the original K230 JPEG unchanged."""
import base64
from html import escape
import math


def render(jpeg, observation, geometry, width=1280, height=960):
    bbox = observation['bbox_px']
    center = observation['center_pixel']
    bottom = geometry.estimated_bottom_from_image_center(center)
    foot = geometry.camera_table_foot()
    distance = geometry.table_distance(bottom)
    start, end = geometry.base_to_pixel(foot), geometry.base_to_pixel(bottom)
    if not all(math.isfinite(v) for v in start + end):
        raise ValueError('Distance line cannot be projected')
    x, y, w, h = bbox
    # SVG viewport clips an off-screen camera foot without inventing an endpoint.
    offscreen = not (0 <= start[0] < width and 0 <= start[1] < height)
    label = 'Estimated table distance: %.0f mm' % distance
    foot_label = 'Camera foot outside image' if offscreen else 'Camera foot'
    encoded = base64.b64encode(jpeg).decode()
    svg = '''<svg xmlns="http://www.w3.org/2000/svg" width="%d" height="%d" viewBox="0 0 %d %d">
<image width="%d" height="%d" href="data:image/jpeg;base64,%s"/>
<rect x="%s" y="%s" width="%s" height="%s" fill="none" stroke="#00ff00" stroke-width="2"/>
<circle cx="%s" cy="%s" r="4" fill="#ff3030"/>
<line x1="%.3f" y1="%.3f" x2="%.3f" y2="%.3f" stroke="#ffcf33" stroke-width="3" stroke-dasharray="12 8"/>
<circle cx="%.3f" cy="%.3f" r="5" fill="#ff3030"/>
<rect x="12" y="12" width="670" height="92" fill="white" fill-opacity="0.88"/>
<g fill="#102040" font-family="sans-serif" font-size="22"><text x="24" y="42">%s</text><text x="24" y="68">Blue pixels: %s; %s</text><text x="24" y="94">Bottom-center proxy; not a precision measurement</text></g>
</svg>''' % (width, height, width, height, width, height, encoded, x, y, w, h,
             center[0], center[1], start[0], start[1], end[0], end[1], end[0], end[1],
             escape(label), observation['pixels'], escape(foot_label))
    metadata = {'distance_mm': distance, 'distance_basis': 'estimated planar distance from camera optical-center foot to cube bottom-center proxy',
        'camera_table_foot_Base_mm': foot, 'estimated_bottom_center_Base_mm': bottom,
        'camera_foot_outside_image': offscreen, 'exact_bottom_center_measured': False,
        'image_area_unit': 'blue pixels, not physical square mm'}
    return svg, metadata
