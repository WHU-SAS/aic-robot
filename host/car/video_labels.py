"""Frame-matched labels anchored to the detected object's box."""
from pathlib import Path


def label_lines(row, chinese=True):
    if row.get('status') not in ('tracking', 'confirming') or not row.get('bbox_px'):
        return []
    center = row.get('center_pixel')
    center_text = ', '.join(str(round(v)) for v in center) if center else '--'
    distance = row.get('estimated_table_distance_mm')
    distance_text = '%.0f mm' % distance if distance is not None else '--'
    if chinese:
        state = '跟踪中' if row['status'] == 'tracking' else '确认中'
        first = ('%s · 原片帧 %s · 电脑离线分析' % (state, row.get('frame_id'))
                 if row.get('PC_offline_analysis') else
                 '%s · 帧 %s · 检测 %s ms' % (state, row.get('frame_id'), row.get('processing_ms')))
        distance_line = ('距离不适用：离桌或桌面接触未确认'
                         if row.get('table_contact_verified') is False else
                         '桌面平面距离（估计）%s' % distance_text)
        return [first,
                '中心 (%s) px · 蓝色面积 %s px' % (center_text, row.get('pixels')),
                distance_line]
    first = ('%s | source frame %s | PC offline analysis' % (row['status'], row.get('frame_id'))
             if row.get('PC_offline_analysis') else
             '%s | frame %s | detection %s ms' % (row['status'], row.get('frame_id'), row.get('processing_ms')))
    return [first,
            'Center (%s) px | blue area %s px' % (center_text, row.get('pixels')),
            ('Distance N/A: table contact not verified' if row.get('table_contact_verified') is False
             else 'Table distance (estimated): %s' % distance_text)]


def label_position(bbox, panel_size, canvas_size, top_margin=80):
    x, y, w, h = bbox
    width, height = canvas_size; pw, ph = panel_size
    left = max(4, min(x, width - pw - 4))
    top = y + top_margin - ph - 8
    if top < top_margin:
        top = y + top_margin + h + 8
    top = max(top_margin, min(top, height - ph - 4))
    return left, top


def draw_label(bgr, row, top_margin=80):
    if not label_lines(row): return bgr
    # Pillow is available in scripts/vision. Only the offline video export pays
    # this font/rendering cost; K230 detection performance is unchanged.
    from PIL import Image, ImageDraw, ImageFont
    import numpy as np
    fonts = ['/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc',
             '/usr/share/fonts/truetype/wqy/wqy-microhei.ttc']
    path = next((p for p in fonts if Path(p).exists()), None)
    font = ImageFont.truetype(path, 20) if path else ImageFont.load_default()
    lines = label_lines(row, chinese=path is not None)
    image = Image.fromarray(bgr[:, :, ::-1]).convert('RGBA')
    overlay = Image.new('RGBA', image.size, (0, 0, 0, 0))
    draw = ImageDraw.Draw(overlay)
    line_height = 30 if path else 20
    pw = int(max(draw.textlength(s, font=font) for s in lines)) + 24
    ph = len(lines) * line_height + 16
    left, top = label_position(row['bbox_px'], (pw, ph), image.size, top_margin)
    color = (0, 255, 0, 255) if row['status'] == 'tracking' else (255, 180, 0, 255)
    draw.rounded_rectangle((left, top, left + pw, top + ph), radius=6,
                           fill=(14, 38, 56, 225), outline=color, width=2)
    for index, text in enumerate(lines):
        draw.text((left + 12, top + 8 + index * line_height), text, font=font, fill=(244, 255, 249, 255))
    return np.array(Image.alpha_composite(image, overlay).convert('RGB'))[:, :, ::-1].copy()
