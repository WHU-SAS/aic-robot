"""Current-image cube-corner measurements, without pose or motion authority.

Five corners share five strong measured lines. The foot has at most two
independent image hypotheses: a blue/blue crease first, then one observed
exterior side-bottom line. The caller may stop after the first accepted pose;
the second foot line is not measured until this generator is resumed.
"""
import math

_RIGHT_OBJECT = [[-10., 0., 20.], [-10., 20., 20.], [10., 20., 20.],
                 [10., 0., 0.], [-10., 0., 0.]]
_LEFT_OBJECT = [[10., 0., 20.], [10., 20., 20.], [-10., 20., 20.],
                [-10., 0., 0.], [10., 0., 0.]]


def _measure(p, profiles, im, hsv, a, b):
    spread = profiles.spread_edge(im, hsv, a, b, p._fit_edge, p._native_contrasts)
    if spread is not None:
        return spread[0]
    edge = None
    for step in p._edge_steps(im, a, b, hsv=hsv):
        if step['stage'] == 'edge_complete':
            edge = step['edge']
    if edge is None:
        raise ValueError('No measured cube outline')
    return edge


def _polygon(p, hsv):
    mask = p.cv2.inRange(hsv, (90, 120, 35), (140, 255, 255))
    contours, unused = p.cv2.findContours(mask, p.cv2.RETR_EXTERNAL, p.cv2.CHAIN_APPROX_SIMPLE)
    eligible = [c for c in contours if p.cv2.contourArea(c) >= 1000]
    if not eligible:
        raise ValueError('No current blue cube component')
    contour = max(eligible, key=p.cv2.contourArea)
    hull = p.cv2.convexHull(contour)
    vertices = p.cv2.approxPolyDP(hull, .003*p.cv2.arcLength(hull, True), True)
    return contour, vertices.reshape((-1, 2)).tolist()


def _mirrored_layout(p, vertices, bbox):
    """Choose a view from current contour geometry, never pose success."""
    x, y, w, h = bbox
    choices = []
    for j, v in enumerate(vertices):
        if not y+.05*h < v[1] < y+.5*h:
            continue
        side = -1 if v[0] <= x+.12*w else 1 if v[0] >= x+.88*w else 0
        if not side:
            continue
        for other in (vertices[j-1], vertices[(j+1) % len(vertices)]):
            dx, dy = other[0]-v[0], other[1]-v[1]
            length = p.distance(v, other)
            if dy < 0 and length >= 25 and side*dx < -.08*w:
                # A top-depth segment must differ from both a near-vertical
                # side wall and a near-horizontal back edge. Rank proposals
                # between those direction families; this grants no edge proof.
                choices.append((abs(dx*dy)/(length*length), side, v, other))
    if not choices:
        raise ValueError('No current exterior top/depth corner')
    best = max(choices, key=lambda v: v[0])
    return best[1] == 1, {'contour_corner_px': best[2],
                         'upper_neighbor_px': best[3], 'depth_obliqueness': best[0]}


def _upper_back_proposal(vertices, bbox):
    """The one contour proposal rule shared by routing and measurement."""
    x, y, w, h = bbox
    choices = []
    for a, b in zip(vertices, vertices[1:]+vertices[:1]):
        a, b = sorted((a, b), key=lambda v: v[0])
        dx, dy = b[0]-a[0], b[1]-a[1]
        if dx >= .5*w and max(a[1], b[1]) <= y+.16*h and abs(dy) < .3*dx:
            choices.append((dx, a, b))
    if not choices:
        raise ValueError('No current long upper-back contour proposal')
    unused, a, b = max(choices)
    return a, b


def _vertical_slope(p, pixel, intrinsics, extrinsics, plane):
    try:
        from ..pick.pose import undistort
    except ImportError:
        from pose import undistort
    K = [list(row) for row in intrinsics['camera_matrix']]
    K[0] = [v*1280/intrinsics['image_width'] for v in K[0]]
    K[1] = [v*960/intrinsics['image_height'] for v in K[1]]
    D, T, normal = (intrinsics['distortion_coefficients'],
                    extrinsics['T_Base_from_Camera_mm'], plane['normal'])
    up = [sum(T[k][j]*normal[k] for k in range(3)) for j in range(3)]
    length = math.sqrt(sum(v*v for v in up))
    up = [v/length for v in up]
    ray = undistort(pixel, K, D)+[1.]
    a = p.project([ray[k]+up[k]*.0001 for k in range(3)], K, D)
    b = p.project([ray[k]-up[k]*.0001 for k in range(3)], K, D)
    return (a[0]-b[0])/(a[1]-b[1])


def _inner_crease(p, im, hsv, bbox, expected_slope):
    """One64-profile image line; the table only initializes its direction."""
    x, y, w, h = bbox
    a, b = [x+.75*w, y+.35*h], [x+.75*w, y+.85*h]
    radius = int(math.ceil(.30*w))
    stations = p.np.arange(64).reshape((64, 1))
    along = .15+.7*stations/63
    offsets = p.np.arange(-radius, radius+1).reshape((1, 2*radius+1))
    mx, my = a[0]+0*along-offsets, a[1]+(b[1]-a[1])*along+0*offsets
    rows = p._native_contrasts(im, mx, my, -5, 0, hsv)
    peaks = p.np.argmax(p.np.array(rows), axis=1).tolist()
    points, ids = [], []
    for j, peak in enumerate(peaks):
        peak = int(peak)
        if rows[j][peak] <= .08*.08:
            continue
        px = a[0]-(peak-radius+p._peak_fraction(rows[j], peak))
        if x+.50*w < px < x+w-3:
            points.append([px, a[1]+(b[1]-a[1])*(.15+.7*j/63)])
            ids.append(j)
    try:
        edge = p._fit_edge(points, ids)
    except ValueError:
        both = p._blue_grid(hsv, a, b, [-1., 0.], radius)
        if sum(bool(row[radius]) for row in both) < 45:
            raise ValueError('Coherent separator is not internal blue')
        length = p.distance(a, b)
        positions = [(-.35+.7*j/63)*length for j in range(64)]
        peaks = []
        for j, (row, allowed) in enumerate(zip(rows, both)):
            for peak in range(1, len(row)-1):
                if (allowed[peak] and row[peak] > .08*.08 and
                        row[peak] >= row[peak-1] and row[peak] >= row[peak+1]):
                    peaks.append((j, peak-radius+p._peak_fraction(row, peak), row[peak]))
        best = None
        for index in range(-4, 5):
            tilt = -expected_slope+index*.01
            bins = {}
            for j, offset, score in peaks:
                bins.setdefault(int(round((offset-tilt*positions[j])/2)), set()).add(j)
            for key in sorted(bins, key=lambda k: -len(bins[k]))[:2]:
                selected = {}
                for j, offset, score in peaks:
                    if (abs(offset-tilt*positions[j]-key*2) <= 2.5 and
                            (j not in selected or score > selected[j][1])):
                        selected[j] = (offset, score)
                if len(selected) < 45:
                    continue
                ids = sorted(selected)
                points = [[a[0]-selected[j][0], a[1]+(b[1]-a[1])*(.15+.7*j/63)] for j in ids]
                try:
                    candidate = p._fit_edge(points, ids)
                    ends = [p.intersect(candidate[0], [0., 1., -yy]) for yy in (a[1], b[1])]
                    p._separator_evidence(hsv, ends, candidate, im)
                except (ValueError, ZeroDivisionError):
                    continue
                rank = (len(selected), sum(v[1] for v in selected.values()), -candidate[1]['rms_px'])
                if best is None or rank > best[0]:
                    best = rank, candidate
        if best is None:
            raise ValueError('No coherent measured internal edge')
        edge = best[1]
        edge[1]['edge_ownership'] = 'coherent_internal_blue_blue'
    ends = [p.intersect(edge[0], [0., 1., -yy]) for yy in (a[1], b[1])]
    p._separator_evidence(hsv, ends, edge, im)
    return edge


def _front_bottom(p, profiles, im, hsv, bbox, left, right):
    x, y, w, h = bbox
    lower = y+h-1
    lx = p.intersect(left[0], [0., 1., -lower])[0]
    rx = p.intersect(right[0], [0., 1., -lower])[0]
    mask = p.cv2.inRange(hsv, (90, 120, 35), (140, 255, 255))
    contours, unused = p.cv2.findContours(mask, p.cv2.RETR_EXTERNAL, p.cv2.CHAIN_APPROX_SIMPLE)
    center = (int(round((lx+rx)/2)), int(round(y+.65*h)))
    containing = [c for c in contours if p.cv2.pointPolygonTest(c, center, False) >= 0]
    if not containing:
        raise ValueError('No connected blue body at measured front')
    component = p.np.zeros(mask.shape, dtype=p.np.uint8)
    p.cv2.drawContours(component, [max(containing, key=p.cv2.contourArea)], -1, (255,), -1)
    points = []
    for j in range(64):
        px = lx+(rx-lx)*(.15+.7*j/63)
        column = int(round(px))
        if not 0 <= column < component.shape[1]:
            continue
        rows = [v for v in range(max(0, int(lower-20)), min(component.shape[0], int(lower+20))) if component[v, column]]
        if rows:
            points.append([px, float(max(rows))])
    if len(points) < 25:
        raise ValueError('No current front-foot contour seed')
    seed = p.fit(points)
    a, b = p.intersect(right[0], seed), p.intersect(left[0], seed)
    return _measure(p, profiles, im, hsv, a, b)


def _bottom_runs(p, hsv, bbox):
    x, y, w, h = bbox
    values = hsv[max(0, int(y)):min(hsv.shape[0], int(y+h)),
                 max(0, int(x)):min(hsv.shape[1], int(x+w)), 2]
    floor = max(35., .25*float(p.np.max(values)))
    mask = p.cv2.inRange(hsv, (90, 120, int(math.ceil(floor))), (140, 255, 255))
    contours, unused = p.cv2.findContours(mask, p.cv2.RETR_EXTERNAL, p.cv2.CHAIN_APPROX_SIMPLE)
    center = (int(round(x+.5*w)), int(round(y+.6*h)))
    containing = [c for c in contours if p.cv2.pointPolygonTest(c, center, False) >= 0]
    if not containing:
        raise ValueError('No current blue body for side-bottom proposal')
    hull = p.cv2.convexHull(max(containing, key=p.cv2.contourArea))
    poly = p.cv2.approxPolyDP(hull, .003*p.cv2.arcLength(hull, True), True).reshape((-1, 2)).tolist()
    runs = []
    for j in range(len(poly)):
        for count in (1, 2, 3):
            observed = [poly[(j+k) % len(poly)] for k in range(count+1)]
            a, b = sorted((observed[0], observed[-1]), key=lambda v: v[0])
            dx, dy = b[0]-a[0], b[1]-a[1]
            length = p.distance(a, b)
            if length < 25:
                continue
            line = p.fit(observed)
            spread = max(p.residual(v, line) for v in observed)
            if spread > 3:
                continue
            runs.append({'a': a, 'b': b, 'line': line, 'length': length,
                         'dx': dx, 'dy': dy, 'observed': observed,
                         'contour_fit_residual_px': spread})
    return runs, floor


def _side_run(runs, bbox, bottom_line):
    x, y, w, h = bbox
    choices = []
    for run in runs:
        a, b, dx, dy = run['a'], run['b'], run['dx'], run['dy']
        if min(a[1], b[1]) <= y+.65*h or a[0] <= x+.5*w or dx < 8 or dy >= -.2*dx:
            continue
        agreement = abs(sum(run['line'][k]*bottom_line[k] for k in (0, 1)))
        if agreement < .94:
            choices.append((run['length'], a, b, agreement))
    if not choices:
        raise ValueError('No current distinct small-side bottom outline')
    return max(choices)


def _small_bottom(p, profiles, im, hsv, bbox, bottom, runs=None):
    if runs is None:
        runs, unused = _bottom_runs(p, hsv, bbox)
    unused, a, b, agreement = _side_run(runs, bbox, bottom[0])
    return _measure(p, profiles, im, hsv, b, a)


def prepare(bgr, bbox_px, roi_padding_px=24, side_search_height_ratio=1.6):
    """Build reusable same-image ROI data and a cheap image-only route hint.

    The selector measures no line and fits no pose. An observed top/depth
    layout plus the detector's own upper-back proposal nominates the cube
    route; actual edge/pose checks still decide whether it is usable.
    ``raw``, ``hsv`` and ``smooth`` retain ORIGINAL ROI coordinates so another
    same-frame localizer can reuse them without a colour conversion or blur.
    """
    try:
        from . import physical_side as p
    except ImportError:
        import physical_side as p
    if bgr.shape != (960, 1280, 3):
        raise ValueError('Expected full 1280x960 JPEG95-decoded BGR')
    if bbox_px is None:
        hsv = p.cv2.cvtColor(bgr, p.cv2.COLOR_BGR2HSV)
        mask = p.cv2.inRange(hsv, (90, 120, 35), (140, 255, 255))
        contours, unused = p.cv2.findContours(mask, p.cv2.RETR_EXTERNAL, p.cv2.CHAIN_APPROX_SIMPLE)
        eligible = [c for c in contours if p.cv2.contourArea(c) >= 1000 and p.cv2.boundingRect(c)[1]+p.cv2.boundingRect(c)[3] > 432]
        if not eligible:
            raise ValueError('No near blue component for cube measurements')
        bbox_px = list(p.cv2.boundingRect(max(eligible, key=p.cv2.contourArea)))
    ox, oy, x1, y1 = p._roi(bbox_px, roi_padding_px, bgr.shape, side_search_height_ratio)
    raw = p.np.array(bgr[oy:y1, ox:x1], dtype=p.np.uint8)
    box = [bbox_px[0]-ox, bbox_px[1]-oy, bbox_px[2], bbox_px[3]]
    hsv = p.cv2.cvtColor(raw, p.cv2.COLOR_BGR2HSV)
    smooth = p.cv2.GaussianBlur(raw, (3, 3), .7)
    contour, vertices = _polygon(p, hsv)
    mirrored, evidence = None, None
    try:
        mirrored, evidence = _mirrored_layout(p, vertices, box)
    except ValueError:
        pass  # The profile route can still use a near-frontal blue component.
    selector = {'route_hint': 'profile', 'has_top_depth_layout': mirrored is not None,
                'has_upper_back_proposal': False}
    context = {'raw': raw, 'hsv': hsv, 'smooth': smooth, 'contour': contour,
               'vertices': vertices, 'bbox_px': list(bbox_px), 'local_bbox': box,
               'roi_px': [ox, oy, x1-ox, y1-oy], 'mirrored_layout': mirrored,
               'layout_evidence': evidence, 'selector': selector, 'side_bottom_runs': None}
    if mirrored is not None:
        canonical_box = list(box)
        canonical_vertices = vertices
        if mirrored:
            canonical_hsv = p.cv2.flip(hsv, 1)
            canonical_box[0] = raw.shape[1]-box[0]-box[2]
            canonical_contour, canonical_vertices = _polygon(p, canonical_hsv)
            context.update(canonical_hsv=canonical_hsv,
                           canonical_contour=canonical_contour,
                           canonical_vertices=canonical_vertices)
        try:
            a, b = _upper_back_proposal(canonical_vertices, canonical_box)
            context['upper_back_proposal'] = (a, b)
            selector['has_upper_back_proposal'] = True
            selector['route_hint'] = 'cube'
            selector['route_reason'] = 'current_top_depth_and_upper_back_outline'
            selector['upper_back_proposal_px'] = [a, b]
        except ValueError as exc:
            selector['proposal_reason'] = str(exc)
    else:
        selector['proposal_reason'] = 'No current exterior top/depth corner'
    return context


def _measurement(p, edges, foot_name, bbox, origin, width, mirrored, layout):
    pairs = [('front_outer', 'top_depth'), ('top_depth', 'back_top'),
             ('back_top', 'back_outer'), ('front_bottom', foot_name),
             ('front_bottom', 'front_outer')]
    points = [p.intersect(edges[a][0], edges[b][0]) for a, b in pairs]
    halves = [[p.intersect(edges[a][2][half], edges[b][2][half]) for a, b in pairs] for half in (0, 1)]
    errors = [[p.distance(a, b) for a, b in zip(points, group)] for group in halves]
    if max(v for group in errors for v in group) > 3:
        raise ValueError('Cube interleaved corners disagree')
    def restore(point):
        return [origin[0]+width-1-point[0], origin[1]+point[1]] if mirrored else [origin[0]+point[0], origin[1]+point[1]]
    def restore_line(line):
        a, b, c = line
        if mirrored:
            a, c = -a, c+a*(width-1)
        return [a, b, c-a*origin[0]-b*origin[1]]
    full = [restore(v) for v in points]
    if any(not (3 < a < 1277 and 3 < b < 957) for a, b in full):
        raise ValueError('Cube corners clipped')
    A, E, F, C, D = points
    if p.area(points) < 1000 or not p.cv2.isContourConvex(p.np.array(points, dtype=p.FLOAT)):
        raise ValueError('Cube visible hull is not convex')
    anchor = layout['anchor']
    ownership = {'top_triangle': p.component_support([A, E, F, A], anchor),
                 'front_triangle': p.component_support([A, C, D, A], anchor)}
    if not all(ownership.values()):
        raise ValueError('Cube corners do not belong to the current blue component')
    names = ['front_outer', 'top_depth', 'back_top', 'back_outer', 'front_bottom', foot_name]
    observed = []
    for name in names:
        edge = edges[name]
        observed.append(dict(quality=dict(edge[1]), name=name, line_px=restore_line(edge[0]),
                             half_lines_px=[restore_line(v) for v in edge[2]]))
    return {'object_points_mm': [list(v) for v in (_LEFT_OBJECT if mirrored else _RIGHT_OBJECT)],
            'image_points_px': full, 'corner_labels': ['B', 'F', 'E', 'D', 'C'] if mirrored else ['A', 'E', 'F', 'C', 'D'],
            'observed_edges': observed, 'half_corner_errors_px': [v for group in errors for v in group],
            'topology': 'left_side_visible' if mirrored else 'right_side_visible',
            'roi_px': layout['roi_px'], 'component_support': True,
            'component_support_detail': ownership, 'foot_source': foot_name,
            'candidate_window_shared': True, 'odd_even_check': 'independent_measured_point_line_fits'}


def corner_steps(bgr, bbox_px, intrinsics, extrinsics, table_plane,
                 roi_padding_px=24, side_search_height_ratio=1.6, prepared=None):
    """Yield image work and at most two five-corner measurement candidates.

    No cube pose, XYZ, capture, hardware command or vision observation is
    produced here. Consumers decide whether a measured candidate is a valid
    three-dimensional cube, and may close the generator after the first one.
    """
    try:
        from . import physical_side as p
        from . import edge_profiles as profiles
    except ImportError:
        import physical_side as p
        import edge_profiles as profiles
    rejected, emitted = [], 0
    try:
        if bgr.shape != (960, 1280, 3):
            raise ValueError('Expected full 1280x960 JPEG95-decoded BGR')
        if table_plane is None:
            raise ValueError('Cube measurements require the paired calibrated table')
        context = prepared if prepared is not None else prepare(bgr, bbox_px, roi_padding_px, side_search_height_ratio)
        bbox_px = context['bbox_px']
        ox, oy, rw, rh = context['roi_px']
        raw, hsv, im = context['raw'], context['hsv'], context['smooth']
        box = list(context['local_bbox'])
        contour, vertices = context['contour'], context['vertices']
        mirrored = context['mirrored_layout']
        if mirrored is None:
            raise ValueError('No current exterior top/depth corner')
        width = raw.shape[1]
        if mirrored:
            box[0] = width-box[0]-box[2]
            hsv = context['canonical_hsv']
            im = p.cv2.flip(im, 1)
            contour, vertices = context['canonical_contour'], context['canonical_vertices']
        support = p.cv2.approxPolyDP(contour, .003*p.cv2.arcLength(contour, True), True)
        layout = {'roi_px': [ox, oy, rw, rh],
                  'anchor': {'component_contour_px': support.reshape((-1, 2)).tolist()}}
        yield {'stage': 'roi', 'roi_px': layout['roi_px'],
               'topology': 'left_side_visible' if mirrored else 'right_side_visible'}
        pixel = [bbox_px[0]+(.1 if mirrored else .9)*bbox_px[2]-(1 if mirrored else 0), bbox_px[1]+.6*bbox_px[3]]
        slope = _vertical_slope(p, pixel, intrinsics, extrinsics, table_plane)
        if mirrored:
            slope = -slope
        edges = {'inner_crease': _inner_crease(p, im, hsv, box, slope)}
        yield {'stage': 'edge_complete', 'edge_name': 'inner_crease'}
        x, y, w, h = box
        a, b = [x, y+.85*h], [x, y+.35*h]
        spread = profiles.spread_edge(im, hsv, a, b, p._fit_edge, p._native_contrasts)
        if spread is None:
            for event in p._edge_steps(im, a, b, hsv=hsv, coherent_outline=True):
                if event['stage'] == 'edge_complete':
                    left = event['edge']
        else:
            left = spread[0]
        edges['front_outer'] = left
        yield {'stage': 'edge_complete', 'edge_name': 'front_outer'}
        edges['front_bottom'] = _front_bottom(p, profiles, im, hsv, box, left, edges['inner_crease'])
        yield {'stage': 'edge_complete', 'edge_name': 'front_bottom'}
        choices = []
        for j, v in enumerate(vertices):
            if not y+.05*h < v[1] < y+.5*h or p.residual(v, left[0]) > .08*w:
                continue
            for other in (vertices[j-1], vertices[(j+1) % len(vertices)]):
                if other[1] < v[1] and other[0] > v[0]+.08*w and p.distance(v, other) >= 25:
                    choices.append((p.residual(v, left[0]), v, other))
        if not choices:
            raise ValueError('No current silhouette front corner')
        unused, a, b = min(choices, key=lambda v: v[0])
        edges['top_depth'] = _measure(p, profiles, im, hsv, a, b)
        yield {'stage': 'edge_complete', 'edge_name': 'top_depth'}
        a, b = _upper_back_proposal(vertices, box)
        edges['back_top'] = _measure(p, profiles, im, hsv, a, b)
        yield {'stage': 'edge_complete', 'edge_name': 'back_top'}
        edges['back_outer'] = _measure(p, profiles, im, hsv, [x+w-1, y+.10*h], [x+w-1, y+.65*h])
        yield {'stage': 'edge_complete', 'edge_name': 'back_outer'}
        try:
            measured = _measurement(p, edges, 'inner_crease', box, [ox, oy], width, mirrored, layout)
            emitted += 1
            yield {'stage': 'candidate', 'candidate': 0, 'measurement': measured}
        except (ValueError, ZeroDivisionError) as exc:
            rejected.append({'candidate': 0, 'reason': str(exc)})
            yield {'stage': 'candidate_rejected', 'candidate': 0, 'reason': str(exc)}
        # This work is deliberately lazy: a successful primary consumer never
        # resumes the generator and pays no second-foot measurement cost.
        try:
            edges['small_bottom'] = _small_bottom(p, profiles, im, hsv, box, edges['front_bottom'], context['side_bottom_runs'])
            yield {'stage': 'edge_complete', 'edge_name': 'small_bottom'}
            measured = _measurement(p, edges, 'small_bottom', box, [ox, oy], width, mirrored, layout)
            emitted += 1
            yield {'stage': 'candidate', 'candidate': 1, 'measurement': measured}
        except (ValueError, ZeroDivisionError) as exc:
            rejected.append({'candidate': 1, 'reason': str(exc)})
            yield {'stage': 'candidate_rejected', 'candidate': 1, 'reason': str(exc)}
    except (ValueError, ZeroDivisionError) as exc:
        rejected.append({'reason': str(exc)})
        yield {'stage': 'measurement_rejected', 'reason': str(exc)}
    yield {'stage': 'complete', 'candidate_count': emitted, 'rejections': rejected}


_FACE_BACK_OBJECT=[[-10.,0.,20.],[10.,0.,20.],[10.,0.,0.],[-10.,0.,0.],
                   [-10.,20.,20.],[10.,20.,20.]]


def _front_back_upper_proposals(p,vertices,bbox):
    """Current contour only; merged upper segments remain search proposals."""
    x,y,w,h=bbox;depth=[];back=[]
    for j,a in enumerate(vertices):
        for b in(vertices[j-1],vertices[(j+1)%len(vertices)]):
            dx,dy=b[0]-a[0],b[1]-a[1]
            if(a[0]<x+.3*w and y+.05*h<a[1]<y+.5*h and dy<0 and
                    dx>.08*w and p.distance(a,b)>=25):
                depth.append((p.distance(a,b),a,b))
        for direction in(-1,1):
            run=[a]
            for count in(1,2,3):
                b=vertices[(j+direction*count)%len(vertices)]
                if b[0]<=run[-1][0]or max(a[1],b[1])>y+.25*h:break
                run.append(b);dx,dy=b[0]-a[0],b[1]-a[1]
                if dx<.5*w or abs(dy)>.3*dx:continue
                line=p.fit(run)
                rms=math.sqrt(sum(p.residual(v,line)**2 for v in run)/len(run))
                if rms<=3:back.append((dx,-rms,a,b,list(run)))
    if not depth or not back:raise ValueError('No current front/back upper contour proposal pair')
    unused,a,b=max(depth,key=lambda v:v[0]);left=(a,b)
    unused,unused,a,b,run=max(back,key=lambda v:v[:2])
    return left,(a,b),run


def _front_back_upper_window(p,prepared,intrinsics,extrinsics,table_plane):
    """Only the used upper-right window, independent of six-corner topology."""
    try:from .cube_outline import DEFAULT_LAB_THRESHOLDS
    except ImportError:from cube_outline import DEFAULT_LAB_THRESHOLDS
    raw,hsv=prepared['raw'],prepared['hsv'];x,y,w,h=prepared['local_bbox']
    ox,oy=prepared['roi_px'][:2]
    lab=p.cv2.cvtColor(raw,p.cv2.COLOR_BGR2LAB)
    mask=p.np.zeros(hsv.shape[:2],dtype=p.np.uint8)
    for L0,L1,a0,a1,b0,b1 in DEFAULT_LAB_THRESHOLDS:
        part=p.cv2.inRange(lab,(int(round(L0*255/100)),a0+128,b0+128),
            (int(round(L1*255/100)),a1+128,b1+128))
        mask=p.np.maximum(mask,part)
    center=(int(round(x+.5*w)),int(round(y+.6*h)))
    contours,unused=p.cv2.findContours(mask,p.cv2.RETR_EXTERNAL,p.cv2.CHAIN_APPROX_SIMPLE)
    current=[c for c in contours if p.cv2.pointPolygonTest(c,center,False)>=0]
    kind='configured_LAB'
    if not current:
        values=hsv[max(0,int(y)):min(hsv.shape[0],int(y+h)),
                   max(0,int(x)):min(hsv.shape[1],int(x+w)),2]
        floor=max(35.,.25*float(p.np.max(values)))
        mask=p.cv2.inRange(hsv,(90,120,int(math.ceil(floor))),(140,255,255))
        contours,unused=p.cv2.findContours(mask,p.cv2.RETR_EXTERNAL,p.cv2.CHAIN_APPROX_SIMPLE)
        current=[c for c in contours if p.cv2.pointPolygonTest(c,center,False)>=0]
        kind='existing_HSV_body_when_LAB_center_absent'
    if not current:raise ValueError('No current blue component for upper-right window')
    body=p.np.zeros(mask.shape,dtype=p.np.uint8)
    p.cv2.drawContours(body,[max(current,key=p.cv2.contourArea)],-1,(255,),-1)
    ownership=p.np.array(hsv,dtype=p.np.uint8)
    ownership[:,:,2]=ownership[:,:,2]*(body>0)*(mask>0)
    a=[x+w-1,y+.1*h]
    slope=_vertical_slope(p,[a[0]+ox,a[1]+oy],intrinsics,extrinsics,table_plane)
    b=[a[0]+slope*.25*h,a[1]+.25*h]
    return a,b,ownership,dict(method='current_bbox_right_projected_vertical_upper_window',
        body_mask_kind=kind,table_up_slope=slope,window_roi_px=[a,b],
        LAB_thresholds=[list(v)for v in DEFAULT_LAB_THRESHOLDS],
        pose_used=False,table_offset_used=False,six_corner_layout_required=False)


def front_back_steps(p,im,hsv,prepared,front_edges,front_quad,
                     intrinsics,extrinsics,table_plane):
    """Reuse four measured ROI front edges, measure three upper outline edges.

    ``im`` is the same smoothed ROI, ``hsv`` its same-image colours, and
    ``front_quad`` is in ROI pixels. No pose, full-frame acquisition or repeated
    front-edge measurement occurs here. E/F are independently observed upper
    corners; F never comes from extrapolating the front's right edge. This is
    a separate six-point/seven-source layout, not the exterior-six policy.
    """
    try:
        from . import edge_profiles as profiles
    except ImportError:
        import edge_profiles as profiles
    count=0;rejected=[]
    try:
        vertices=prepared.get('vertices')
        if vertices is None:
            unused,vertices=_polygon(p,hsv)
        depth,back,run=_front_back_upper_proposals(p,vertices,prepared['local_bbox'])
        left=_measure(p,profiles,im,hsv,*depth)
        yield {'stage':'front_back_edge_complete','edge_name':'left_top_depth'}
        upper=_measure(p,profiles,im,hsv,*back)
        yield {'stage':'front_back_edge_complete','edge_name':'back_top'}
        # A short UNUSED side-bottom/top branch cannot reject this window.
        # Only the actual upper-right line must pass its64station measurement.
        a,b,ownership,proposal=_front_back_upper_window(p,prepared,
            intrinsics,extrinsics,table_plane)
        right_window=[a,b]
        right=_measure(p,profiles,im,ownership,a,b)
        body_kind=proposal['body_mask_kind']
        right[1].update(edge_ownership=('same_current_LAB_component_upper_outline'
            if body_kind=='configured_LAB'else'same_current_HSV_body_upper_outline'),
            body_mask_kind=body_kind,
            broad_HSV_outside_nonblue_verified=False)
        yield {'stage':'front_back_edge_complete','edge_name':'independent_upper_right_silhouette'}
        points=[list(v)for v in front_quad]+[p.intersect(left[0],upper[0]),p.intersect(upper[0],right[0])]
        halves=[]
        for half in(0,1):
            group=[p.intersect(front_edges[i-1][2][half],front_edges[i][2][half])for i in range(4)]
            group.extend([p.intersect(left[2][half],upper[2][half]),p.intersect(upper[2][half],right[2][half])])
            halves.append(group)
        errors=[p.distance(a,b)for group in halves for a,b in zip(points,group)]
        if max(errors)>3:raise ValueError('Front/back cube interleaved corners disagree')
        ox,oy=prepared['roi_px'][:2]
        full=[[v[0]+ox,v[1]+oy]for v in points]
        if any(not(3<a<1277 and 3<b<957)for a,b in full):
            raise ValueError('Front/back cube corners clipped')
        names=('front_top','front_right','front_bottom','front_outer',
               'left_top_depth','back_top','independent_upper_right_silhouette')
        edges=list(front_edges)+[left,upper,right]
        def restore(line):
            a,b,c=line
            return[a,b,c-a*ox-b*oy]
        observed=[dict(name=name,quality=dict(edge[1]),line_px=restore(edge[0]),
            half_lines_px=[restore(v)for v in edge[2]])for name,edge in zip(names,edges)]
        measurement={'evidence_type':'nonplanar_cube_face_back_top',
            'object_points_mm':[list(v)for v in _FACE_BACK_OBJECT],
            'image_points_px':full,'corner_labels':['A','B','C','D','E','F'],
            'observed_edges':observed,'half_corner_errors_px':errors,
            'topology':'measured_front_face_and_back_top','roi_px':list(prepared['roi_px']),
            'candidate_window_shared':True,'odd_even_check':'independent_measured_point_line_fits',
            'upper_proposals':{'left_depth_roi_px':depth,'back_top_roi_px':back,
                'merged_back_contour_roi_px':run,'right_upper_roi_px':right_window,
                'right_silhouette':proposal,'pose_used':False,'table_offset_used':False}}
        count=1
        yield {'stage':'cube_candidate','candidate':0,'measurement':measurement}
    except(ValueError,ZeroDivisionError)as exc:
        rejected.append({'reason':str(exc)})
        yield {'stage':'front_back_rejected','reason':str(exc)}
    yield {'stage':'front_back_complete','candidate_count':count,'rejections':rejected}
