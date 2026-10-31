"""Shared, bounded physical-side localization for car parking and arm picking.

Input is an owned, full-size BGR image decoded from the existing JPEG95 path.
The coarse box only limits proposal search; it is never a physical face. All
four edges, the 2px pose limit and calibrated-table tests retain pick semantics.
Yields are scheduling opportunities, not a promise of board timing or threads.
"""
import math
import time
import cv2
try:
    from ulab import numpy as np
except ImportError:
    import numpy as np
try:
    from ..pick.reflection_faces import (FLOAT, ordered, distance, fit, residual,
        intersect, validate_seam, blue)
    from ..pick.side_faces import (area, corner_distance, side_pose, select_sides,
        component_support, shared_edge)
    from ..pick.face_selection import internal_seam, shallow_face
    from ..pick.pose import pose_candidates, OBJECT, project
    from ..pick.numeric import dot, norm, matmul, matvec, rotation, solve
    from ..pick.table_pose import context as table_context, constrain_face
    from ..vision.runtime import localize_precise
except ImportError:
    from reflection_faces import (FLOAT, ordered, distance, fit, residual,
        intersect, validate_seam, blue)
    from side_faces import (area, corner_distance, side_pose, select_sides,
        component_support, shared_edge)
    from face_selection import internal_seam, shallow_face
    from pose import pose_candidates, OBJECT, project
    from numeric import dot, norm, matmul, matvec, rotation, solve
    from table_pose import context as table_context, constrain_face
    from vision_runtime import localize_precise


def _now():
    fn = getattr(time, 'ticks_ms', None)
    return fn() if fn else int(time.monotonic()*1000)


def _elapsed(start):
    fn = getattr(time, 'ticks_diff', None)
    return fn(_now(), start) if fn else _now()-start


def _pose_step(R, t, delta):
    angle = norm(delta[:3])
    updated = matmul(rotation([v/angle for v in delta[:3]], angle), R) if angle > 1e-12 else R
    return updated, [t[i]+delta[i+3] for i in range(3)]


def _pose_residuals(R, t, quad, K, D):
    result = []
    for obj, target in zip(OBJECT, quad):
        rp = matvec(R, obj)
        pixel = project([rp[i]+t[i] for i in range(3)], K, D)
        result.extend([pixel[k]-target[k] for k in (0, 1)])
    return result


def _pose_jacobian(R, t, K, D):
    """Exact Brown projection derivatives for the existing left-rotation step."""
    k1, k2, p1, p2, k3 = (list(D)+[0.]*5)[:5]
    columns = [[] for unused in range(6)]
    for obj in OBJECT:
        rp = matvec(R, obj)
        X, Y, Z = [rp[i]+t[i] for i in range(3)]
        if Z <= 0:
            raise ValueError('Point behind camera')
        x, y = X/Z, Y/Z
        r2 = x*x+y*y
        radial = 1+k1*r2+k2*r2*r2+k3*r2*r2*r2
        slope = k1+2*k2*r2+3*k3*r2*r2
        xx = radial+2*x*x*slope+2*p1*y+6*p2*x
        xy = 2*x*y*slope+2*p1*x+2*p2*y
        yy = radial+2*y*y*slope+6*p1*y+2*p2*x
        for a, b, focal in ((xx, xy, K[0][0]), (xy, yy, K[1][1])):
            gx, gy, gz = focal*a/Z, focal*b/Z, -focal*(a*x+b*y)/Z
            row = [-gy*rp[2]+gz*rp[1], gx*rp[2]-gz*rp[0],
                   -gx*rp[1]+gy*rp[0], gx, gy, gz]
            for column, value in zip(columns, row):
                column.append(value)
    return columns


def refine_pose(R, t, quad, K, D):
    """Same two-seed LM objective/20 iterations, with an analytic Jacobian.

    Keeps original damping, limits and convergence rule; removes twelve extra
    reprojections per iteration rather than reducing iterations or accepting a
    worse quadrilateral. The portable default refiner remains unchanged.
    """
    error = _pose_residuals(R, t, quad, K, D)
    cost, damping = dot(error, error), .001
    for unused in range(20):
        columns = _pose_jacobian(R, t, K, D)
        normal = [[dot(a, b) for b in columns] for a in columns]
        for j in range(6):
            normal[j][j] += damping*max(normal[j][j], 1e-6)
        try:
            delta = solve(normal, [-dot(a, error) for a in columns])
        except ValueError:
            break
        scale = max(1., norm(delta[:3])/.2, norm(delta[3:])/30.)
        delta = [v/scale for v in delta]
        rr, tt = _pose_step(R, t, delta)
        try:
            new_error = _pose_residuals(rr, tt, quad, K, D)
            new_cost = dot(new_error, new_error)
        except ValueError:
            damping *= 10
            continue
        if new_cost < cost:
            improvement = cost-new_cost
            R, t, error, cost = rr, tt, new_error, new_cost
            damping = max(1e-9, damping*.3)
            if improvement < 1e-8:
                break
        else:
            damping *= 10
            if damping > 1e8:
                break
    return R, t


_BATCH_EDGES = all(hasattr(cv2, name) for name in ('remap', 'INTER_NEAREST')) and all(
    hasattr(np, name) for name in ('arange', 'maximum', 'sum'))


def _native_contrasts(im, mx, my, dx, dy, hsv=None):
    """Native nearest-pixel pairs; same relative RGB contrast as physical edges."""
    kind = getattr(np, 'float32', FLOAT)
    ux, uy = np.array(mx+dx, dtype=kind), np.array(my+dy, dtype=kind)
    lx, ly = np.array(mx-dx, dtype=kind), np.array(my-dy, dtype=kind)
    upper = cv2.remap(im, ux, uy, cv2.INTER_NEAREST)
    lower = cv2.remap(im, lx, ly, cv2.INTER_NEAREST)
    upper, lower = np.array(upper, dtype=FLOAT), np.array(lower, dtype=FLOAT)
    den = np.maximum((np.sum(upper, axis=2)+np.sum(lower, axis=2))/6, 40.)
    delta = upper-lower
    scores = np.sum(delta*delta, axis=2)/(den*den)
    height, width = im.shape[:2]
    valid = (mx-abs(dx) > 1)*(mx+abs(dx) < width-2)*(my-abs(dy) > 1)*(my+abs(dy) < height-2)
    if hsv is not None:
        for sx, sy in ((ux, uy), (lx, ly)):
            values = cv2.remap(hsv, sx, sy, cv2.INTER_NEAREST)
            valid = valid*(values[:, :, 0] >= 90)*(values[:, :, 0] <= 140)*\
                (values[:, :, 1] >= 80)*(values[:, :, 2] >= 35)
    valid = np.array(valid, dtype=FLOAT)
    return (scores*valid-(1-valid)).tolist()


def _contrast_grid(im, a, b, normal, radius):
    stations = np.arange(64).reshape((64, 1))
    offsets = np.arange(-radius, radius+1).reshape((1, 2*radius+1))
    along = .15+.7*stations/63
    mx = a[0]+(b[0]-a[0])*along+normal[0]*offsets
    my = a[1]+(b[1]-a[1])*along+normal[1]*offsets
    return _native_contrasts(im, mx, my, 2*normal[0], 2*normal[1])


def _peak_fraction(row, peak):
    if not 0 < peak < len(row)-1 or min(row[peak-1:peak+2]) < 0:
        return 0.
    before, score, after = row[peak-1:peak+2]
    curvature = before-2*score+after
    return max(-.5, min(.5, .5*(before-after)/curvature)) if curvature < 0 else 0.


def _fit_edge(points, stations):
    if len(points) < 45:
        raise ValueError('Insufficient edge contrast')
    line = fit(points)
    inliers = [(p, j) for p, j in zip(points, stations) if residual(p, line) <= 3]
    if len(inliers) < 45 or max(j for p, j in inliers)-min(j for p, j in inliers) < 50:
        raise ValueError('Edge lacks distributed consensus')
    line = fit([p for p, j in inliers])
    rms = math.sqrt(sum(residual(p, line)**2 for p, j in inliers)/len(inliers))
    if rms > 3:
        raise ValueError('Physical edge RMS exceeds 3px')
    groups = [[p for p, j in inliers if j % 2 == half] for half in (0, 1)]
    if min(len(g) for g in groups) < 20:
        raise ValueError('Unbalanced edge support')
    return line, {'support': len(inliers), 'sampled_stations': 64, 'rms_px': rms}, [fit(g) for g in groups]


def _blue_grid(hsv, a, b, normal, radius, inside_only=False):
    stations = np.arange(64).reshape((64, 1))
    offsets = np.arange(-radius, radius+1).reshape((1, 2*radius+1))
    along = .15+.7*stations/63
    mx = a[0]+(b[0]-a[0])*along+normal[0]*offsets
    my = a[1]+(b[1]-a[1])*along+normal[1]*offsets
    kind = getattr(np, 'float32', FLOAT)
    both = np.ones(mx.shape, dtype=FLOAT)
    for sign in ((1,) if inside_only else (1, -1)):
        values = cv2.remap(hsv, np.array(mx+sign*2*normal[0], dtype=kind),
            np.array(my+sign*2*normal[1], dtype=kind), cv2.INTER_NEAREST)
        both = both*(values[:, :, 0]>=90)*(values[:, :, 0]<=140)*\
            (values[:, :, 1]>=80)*(values[:, :, 2]>=35)
    return both.tolist()


def _internal_edge(a, b, n, radius, rows, peaks, both, original):
    # Two resolved lines distinguish an actual blue/blue crease from a single
    # external edge's coloured rim. The +/-2px contrast pair spans4pixels;
    # require a further pixel of separation at three positions along the edge.
    if (sum(bool(row[radius]) for row in both) < 45 or
            sum(bool(row[peak]) for row, peak in zip(both, peaks)) >= 20):
        return original
    points, stations = [], []
    for j, (row, allowed) in enumerate(zip(rows, both)):
        candidates = [i for i in range(len(row)) if allowed[i] and row[i] >= 0]
        peak = max(candidates, key=lambda i: row[i]) if candidates else None
        if peak is None or row[peak] <= .08*.08:
            continue
        offset = peak-radius+_peak_fraction(row, peak)
        t = .15+.7*j/63
        points.append([a[k]+(b[k]-a[k])*t+n[k]*offset for k in (0, 1)])
        stations.append(j)
    try:
        internal = _fit_edge(points, stations)
    except ValueError:
        return original
    inner, outer = internal[0], original[0]
    den_i = inner[0]*n[0]+inner[1]*n[1]
    den_o = outer[0]*n[0]+outer[1]*n[1]
    if min(abs(den_i), abs(den_o)) < 1e-6:
        return original
    for t in (.15, .5, .85):
        p = [a[k]+(b[k]-a[k])*t for k in (0, 1)]
        i = -(inner[0]*p[0]+inner[1]*p[1]+inner[2])/den_i
        o = -(outer[0]*p[0]+outer[1]*p[1]+outer[2])/den_o
        if abs(i-o) < 5 or abs(i) >= abs(o):
            return original
    internal[1]['edge_ownership'] = 'internal_blue_blue'
    return internal


def _coherent_internal_edge(a, b, n, radius, rows, both, hsv, im, outline=False):
    """Recover a crease from local peaks in the SAME64 contrast profiles.

    A broad highlight has two flanks: independently choosing the strongest
    pixel at each station can mix them. Vote for one observed line instead;
    the normal edge fit and blue/blue separator checks still own acceptance.
    An outline instead requires a blue interior and prefers the straightest
    distributed peak line, so a face/shadow mixture cannot win by brightness.
    """
    if sum(bool(row[radius]) for row in both) < 45:
        raise ValueError('Coherent outline lacks blue interior' if outline else
                         'Coherent separator is not internal blue')
    length = distance(a, b)
    along = [(-.35+.7*j/63)*length for j in range(64)]
    candidates = []
    for j, (row, allowed) in enumerate(zip(rows, both)):
        for peak in range(1, len(row)-1):
            if (allowed[peak] and row[peak] > .08*.08 and
                    row[peak] >= row[peak-1] and row[peak] >= row[peak+1]):
                candidates.append((j, peak-radius+_peak_fraction(row, peak), row[peak]))
    best = None
    for index in range(-4, 5):
        tilt = index*.05
        bins = {}
        for j, offset, score in candidates:
            key = int(round((offset-tilt*along[j])/2))
            bins.setdefault(key, set()).add(j)
        keys = sorted(bins, key=(lambda k: (-len(bins[k]), k)) if outline else (lambda k: -len(bins[k])))
        for key in keys[:2]:
            selected = {}
            for j, offset, score in candidates:
                if (abs(offset-tilt*along[j]-key*2) <= 2.5 and
                        (j not in selected or score > selected[j][1])):
                    selected[j] = (offset, score)
            if len(selected) < 45:
                continue
            stations = sorted(selected)
            points = [[a[k]+(b[k]-a[k])*(.15+.7*j/63)+n[k]*selected[j][0]
                       for k in (0, 1)] for j in stations]
            try:
                edge = _fit_edge(points, stations)
                rank = ((-edge[1]['rms_px'], edge[1]['support'], sum(v[1] for v in selected.values()))
                        if outline else (len(selected), sum(v[1] for v in selected.values()), -edge[1]['rms_px']))
                if best is not None and rank <= best[0]:
                    continue
                if not outline:
                    upper = [intersect(edge[0], [1., 0., -v[0]]) for v in (a, b)]
                    _separator_evidence(hsv, upper, edge, im)
            except ValueError:
                continue
            best = (rank, edge)
    if best is None:
        raise ValueError('No coherent measured internal edge')
    best[1][1]['edge_ownership'] = 'coherent_blue_outline' if outline else 'coherent_internal_blue_blue'
    return best[1]


def _edge_steps(im, a, b, stations_per_step=8, hsv=None, coherent=False, profile=None):
    """Same64 stations/edge checks; retain a resolved internal-face crease."""
    length = distance(a, b)
    if length < 25:
        raise ValueError('Edge too short')
    u = [(b[k]-a[k])/length for k in (0, 1)]
    n = [-u[1], u[0]]
    radius = min(12, max(5, int(round(length*.06))))
    points, stations, rows, peaks, blue_rows = [], [], [], [], []
    height, width = im.shape[:2]
    grid = _contrast_grid(im, a, b, n, radius) if _BATCH_EDGES else None
    indices = np.argmax(np.array(grid, dtype=FLOAT), axis=1).tolist() if grid is not None else None
    for j in range(64):
        t = .15+.7*j/63
        mid = [a[k]+(b[k]-a[k])*t for k in (0, 1)]
        if grid is not None:
            row = grid[j]
            peak = int(indices[j])
        else:
            row, allowed = [-1.]*(2*radius+1), [False]*(2*radius+1)
            for offset in range(-radius, radius+1):
                p = [mid[k]+offset*n[k] for k in (0, 1)]
                upper = [p[k]+2*n[k] for k in (0, 1)]
                lower = [p[k]-2*n[k] for k in (0, 1)]
                if any(not (1<v[0]<width-2 and 1<v[1]<height-2) for v in (upper, lower)):
                    continue
                c = im[int(round(upper[1])), int(round(upper[0]))].tolist()
                d = im[int(round(lower[1])), int(round(lower[0]))].tolist()
                den = max(40., sum(c+d)/6)
                dr, dg, db = [c[k]-d[k] for k in range(3)]
                score = (dr*dr+dg*dg+db*db)/(den*den)
                row[offset+radius] = score
                if hsv is not None:
                    allowed[offset+radius] = all(blue(hsv[int(round(v[1])), int(round(v[0]))].tolist())
                        for v in (upper, lower))
            peak = max(range(len(row)), key=lambda i: row[i])
            blue_rows.append(allowed)
        rows.append(row); peaks.append(peak)
        if row[peak] > .08*.08:
            offset = peak-radius+_peak_fraction(row, peak)
            points.append([mid[k]+offset*n[k] for k in (0, 1)])
            stations.append(j)
        if (j+1) % stations_per_step == 0:
            yield {'stage': 'edge_samples', 'stations': j+1}
    try:
        edge = _fit_edge(points, stations)
    except ValueError:
        if not coherent or hsv is None:
            raise
        both = _blue_grid(hsv, a, b, n, radius) if grid is not None else blue_rows
        edge = _coherent_internal_edge(a, b, n, radius, rows, both, hsv, im)
    # An internal line separated by5pixels cannot be closer to the seed if
    # the external peak stays within2pixels. Most existing edges exit here.
    if hsv is not None and sum(abs(p-radius)>=3 for p in peaks) >= 45:
        both = _blue_grid(hsv, a, b, n, radius) if grid is not None else blue_rows
        edge = _internal_edge(a, b, n, radius, rows, peaks, both, edge)
    if profile is not None:
        profile.update(a=a, b=b, normal=n, radius=radius, rows=rows)
    yield {'stage': 'edge_complete', 'edge': edge}


def _roi(bbox, padding, shape, side_search_height_ratio=1.6):
    if (len(bbox) != 4 or any(not isinstance(v, (int, float)) or
            not math.isfinite(v) for v in bbox)):
        raise ValueError('Invalid coarse box')
    x, y, w, h = bbox
    if w <= 0 or h <= 0:
        raise ValueError('Empty coarse box')
    height, width = shape[:2]
    x0, y0 = max(0, int(x-padding)), max(0, int(y-padding))
    # The coarse blob may be ONLY the brighter top. Keep enough image below
    # it to propose the side, whose height is roughly width*fy/fx for this
    # camera. This enlarges search only; it never completes a missing edge.
    bottom = max(y+h, y+side_search_height_ratio*w)
    x1, y1 = min(width, int(math.ceil(x+w+padding))), min(height, int(math.ceil(bottom+padding)))
    if x0 >= x1 or y0 >= y1:
        raise ValueError('Coarse box is outside the image')
    return x0, y0, x1, y1


def _separator_evidence(hsv, q, edge, image):
    """One colour check, or the already measured internal-edge evidence.

    Weak colour magnitude is not a missing physical edge: a complete64-station
    fit plus blue on both sides and coherent colour direction can establish a
    separator. An outer silhouette/solid rectangle has no blue on both sides.
    """
    a, b = q[:2]
    length = distance(a, b)
    nx, ny = -(b[1]-a[1])/length, (b[0]-a[0])/length
    kind = getattr(np, 'float32', FLOAT)
    along = np.arange(32).reshape((1, 32))*.6/31+.2
    xx, yy = a[0]+along*(b[0]-a[0]), a[1]+along*(b[1]-a[1])
    maps = [(np.array(xx+sign*5*nx, dtype=kind), np.array(yy+sign*5*ny, dtype=kind))
            for sign in (1, -1)]
    h, w = hsv.shape[:2]
    valid = (xx-abs(5*nx)>0)*(xx+abs(5*nx)<w-1)*(yy-abs(5*ny)>0)*(yy+abs(5*ny)<h-1)
    def pairs(im):
        if _BATCH_EDGES:
            return [np.array(cv2.remap(im, x, y, cv2.INTER_NEAREST), dtype=FLOAT) for x, y in maps]
        return [np.array([[im[max(0, min(h-1, int(round(py)))),
            max(0, min(w-1, int(round(px))))].tolist() for px, py in zip(x.tolist()[0], y.tolist()[0])]],
            dtype=FLOAT) for x, y in maps]
    hi, ho = pairs(hsv)
    both = valid
    for value in (hi, ho):
        both = both*(value[:, :, 0]>=90)*(value[:, :, 0]<=140)*(value[:, :, 1]>=80)*(value[:, :, 2]>=35)
    intensity = both*(abs(hi[:, :, 2]-ho[:, :, 2])>np.maximum(10., .12*np.maximum(hi[:, :, 2], ho[:, :, 2])))
    ci, co = pairs(image)
    delta = ci-co
    den = np.maximum(40., np.sum(ci+co, axis=2)/6)
    colour = both*(np.sum(delta*delta, axis=2)>(.12*den)**2)
    totals = np.sum(delta*colour.reshape((1, 32, 1)), axis=1).tolist()[0]
    channel = max(range(3), key=lambda k: abs(totals[k]))
    coherent = int(np.sum(colour*(delta[:, :, channel]*totals[channel]>0)))
    legacy = max(int(np.sum(intensity)), coherent)
    totals = np.sum(delta*both.reshape((1, 32, 1)), axis=1).tolist()[0]
    channel = max(range(3), key=lambda k: abs(totals[k]))
    direction = int(np.sum(both*(delta[:, :, channel]*totals[channel]>0)))
    internal = int(np.sum(both))
    measured = edge[1]['sampled_stations'] == 64 and edge[1]['support'] >= 45 and edge[1]['rms_px'] <= 3
    alternative = measured and internal >= 24 and direction >= 24
    if legacy < 20 and not alternative:
        raise ValueError('No distributed blue top / side contrast')
    return {'support': legacy if legacy >= 20 else min(internal, direction),
        'method': 'colour' if legacy >= 20 else 'measured_internal_separator',
        'both_blue_stations': internal, 'colour_votes': legacy, 'direction_votes': direction}


def _outline_support(a,b,contour,tolerance=3):
    """Require an observed contour span along a proposed supporting line."""
    if not contour:return False
    length=distance(a,b)
    line=fit([a,b]);u=[(b[k]-a[k])/length for k in (0,1)];intervals=[]
    for c,d in zip(contour,contour[1:]+contour[:1]):
        other=distance(c,d)
        if other<8 or max(residual(p,line) for p in (c,d))>tolerance:continue
        if abs(sum((d[k]-c[k])*u[k] for k in (0,1)))/other<.94:continue
        along=[sum((p[k]-a[k])*u[k] for k in (0,1)) for p in (c,d)]
        lo=max(0.,min(along));hi=min(length,max(along))
        if hi>lo:intervals.append((lo,hi))
    end=coverage=0.
    for lo,hi in sorted(intervals):
        coverage+=max(0.,hi-max(end,lo));end=max(end,hi)
    return coverage>=.5*length


def _recoverable_outline_seeds(seeds, contour):
    """Keep existing internal-face proposals ahead of any new recovery.

    Fifteen pixels cover the original12px edge search plus3px fit tolerance.
    This only classifies search seeds; it never establishes a physical edge.
    """
    for q in seeds:
        top = min(range(4), key=lambda j: q[j][1]+q[(j+1) % 4][1])
        if not _outline_support(q[top], q[(top+1) % 4], contour, tolerance=15):
            return False
    return True


def _bottom_segments(bbox,vertices):
    """Observed lower edges seed a side; rounded endpoints may be extended."""
    x,y,w,h=bbox
    bottoms=[(a,b) for a,b in zip(vertices,vertices[1:]+vertices[:1]) if min(a[1],b[1])>y+.5*h and abs(a[0]-b[0])>=25]
    vertical=[(a,b) for a,b in zip(vertices,vertices[1:]+vertices[:1]) if abs(a[0]-b[0])<.25*abs(a[1]-b[1]) and abs(a[1]-b[1])>=.5*h]
    out=[]
    for a,b in sorted(bottoms,key=lambda item:-abs(item[0][0]-item[1][0]))[:2]:
        line=fit([a,b]);ends=[list(a),list(b)]
        for index,p in enumerate(ends):
            alternatives=[]
            for c,d in vertical:
                try:q=intersect(line,fit([c,d]))
                except ValueError:continue
                if distance(p,q)<.25*w:alternatives.append((distance(p,q),q))
            if alternatives:ends[index]=min(alternatives,key=lambda item:item[0])[1]
        left,right=sorted(ends,key=lambda p:p[0])
        if right[0]-left[0]>=25:out.append((left,right))
    return out


def _recover_seam_steps(im, hsv, bbox, vertices, hypotheses, first, last):
    """Reuse the original seam votes and observed lower outline, static only."""
    center_x = bbox[0]+bbox[2]/2
    tried = []
    bottoms = _bottom_segments(bbox, vertices)
    for count, line in sorted(hypotheses, key=lambda item: -item[0]):
        middle = -(line[0]*center_x+line[2])/line[1]
        if any(abs(middle-old) < 3 for old in tried):
            continue
        tried.append(middle)
        if len(tried) > 3:
            break
        for left, right in bottoms[:1]:
            # The observed lower edge supplies a search direction, never the
            # measured upper edge. All64 upper stations are still sampled.
            slope = (right[1]-left[1])/(right[0]-left[0])
            upper = [[p[0], middle+slope*(p[0]-center_x)] for p in (left, right)]
            if min(left[1]-upper[0][1], right[1]-upper[1][1]) < 25:
                continue
            try:
                for step in _edge_steps(im, upper[0], upper[1], hsv=hsv, coherent=True):
                    if step['stage'] == 'edge_complete':
                        top_edge = step['edge']
                    yield {'stage': 'seam_'+step['stage']}
                upper = [intersect(top_edge[0], [1., 0., -p[0]]) for p in (left, right)]
                measured_middle = -(top_edge[0][0]*center_x+top_edge[0][2])/top_edge[0][1]
                if not first <= measured_middle < last:
                    raise ValueError('Measured separator left its searched band')
                q = upper+[list(right), list(left)]
                _separator_evidence(hsv, q, top_edge, im)
            except ValueError:
                continue
            yield {'stage': 'seam_seed', 'seed': q, 'top_edge': top_edge}


def _seam_seed_steps(im, hsv, bbox, vertices, reuse_local=False, recover_internal=False):
    """Propose a side from observed blue/blue separation and lower contour.

    Colour slices can merge a bright top with a gradually shaded front. A
    distributed RGB step locates their separator without choosing a 3D target.
    Lower contour endpoints only seed edge search; all four edges are remeasured.
    """
    x, y, w, h = bbox
    height, width = im.shape[:2]
    votes = []
    first, last = max(6, int(y+6)), min(height-6, int(y+.55*h))
    columns = [int(round(x+w*(.2+.6*column/23))) for column in range(24)]
    if _BATCH_EDGES and first < last:
        xx = np.array(columns, dtype=FLOAT).reshape((24, 1))
        yy = np.arange(first, last).reshape((1, last-first))
        grid = _native_contrasts(im, xx+0*yy, yy+0*xx, 0, 5, hsv)
    else:
        grid = None
    for column in range(24):
        xx = columns[column]
        if not 0 <= xx < width:
            continue
        peaks = []
        if grid is not None:
            profile = grid[column]
        else:
            profile = [-1.]*(last-first)
            for yy in range(first, last):
                if not (blue(hsv[yy-5, xx].tolist()) and blue(hsv[yy+5, xx].tolist())):
                    continue
                a, b = im[yy-5, xx].tolist(), im[yy+5, xx].tolist()
                den = max(40., sum(a+b)/6)
                profile[yy-first] = sum((u-v)**2 for u, v in zip(a, b))/(den*den)
        # A truncated contrast flank is not an observed separator. Select the
        # existing two votes from complete local peaks in this same RGB grid.
        peaks = [(profile[row], row+first) for row in range(1, len(profile)-1)
                 if min(profile[row-1:row+2]) >= 0 and profile[row] > .12*.12
                 and profile[row] >= profile[row-1]
                 and profile[row] >= profile[row+1]
                 and profile[row] > min(profile[row-1], profile[row+1])]
        peaks.sort(reverse=True)
        chosen = []
        for score, yy in peaks:
            if all(abs(yy-old) >= 8 for old in chosen):
                votes.append([xx, yy])
                chosen.append(yy)
                if len(chosen) == 2:
                    break
        if column % 3 == 2:
            yield {'stage': 'seam_samples', 'columns': column+1}
    hypotheses = []
    center_x = x+w/2
    for index in range(9):
        slope = -.4+.1*index
        bins = {}
        for px, py in votes:
            key = int(round((py-slope*(px-center_x))/2))
            bins[key] = bins.get(key, 0)+1
        for key in sorted(bins, key=lambda k: -bins[k])[:2]:
            support = [p for p in votes if abs(p[1]-slope*(p[0]-center_x)-2*key) <= 2.5]
            if len(set(p[0] for p in support)) >= 12:
                line = fit(support)
                selected = [p for p in support if residual(p, line) <= 3]
                if len(set(p[0] for p in selected)) >= 12 and abs(line[1]) > 1e-6:
                    hypotheses.append((len(set(p[0] for p in selected)), fit(selected)))
    bottoms = [(a, b) for a, b in zip(vertices, vertices[1:]+vertices[:1])
               if min(a[1], b[1]) > y+.5*h and abs(a[0]-b[0]) >= 25]
    tried = []
    seam_tries = 0
    for count, line in sorted(hypotheses, key=lambda item: -item[0]):
        middle = -(line[0]*center_x+line[2])/line[1]
        if any(abs(middle-old) < 8 for old in tried):
            continue
        tried.append(middle)
        if len(tried) > 3:
            break
        for a, b in sorted(bottoms, key=lambda edge: -abs(edge[0][0]-edge[1][0]))[:1]:
            left, right = sorted((a, b), key=lambda p: p[0])
            upper = [[p[0], -(line[0]*p[0]+line[2])/line[1]] for p in (left, right)]
            if min(left[1]-upper[0][1], right[1]-upper[1][1]) < 25:
                continue
            try:
                # Verify the observed separator BEFORE consuming a full-face
                # attempt. Blue background near the silhouette may otherwise
                # outrank the real top/side colour step.
                seam_tries += 1
                for step in _edge_steps(im, upper[0], upper[1], hsv=hsv):
                    if step['stage'] == 'edge_complete':
                        top_edge = step['edge']
                    yield {'stage': 'seam_'+step['stage']}
                upper = [intersect(top_edge[0], [1., 0., -p[0]]) for p in (left, right)]
                measured_middle = -(top_edge[0][0]*center_x+top_edge[0][2])/top_edge[0][1]
                if not first <= measured_middle < last:
                    raise ValueError('Measured separator left its searched band')
                q = upper+[list(right), list(left)]
                _separator_evidence(hsv, q, top_edge, im)
            except ValueError:
                continue
            yield {'stage': 'seam_seed', 'seed': q, 'top_edge': top_edge}
            return  # One verified separator proposal; colours find other sides.
        yield {'stage': 'seam_proposal'}

    if not reuse_local:
        if recover_internal:
            for step in _recover_seam_steps(im, hsv, bbox, vertices, hypotheses, first, last):
                yield step
        return
    # Reuse the original RGB votes; do not scan the image a second time.
    # Fewer local proposal votes never replace the full64-station edge check.
    local_hypotheses=[]
    for left,right in _bottom_segments(bbox,vertices):
        side_columns=[column for column in columns if left[0]<=column<=right[0]]
        if len(set(side_columns))<6:continue
        side_votes=[p for p in votes if min(side_columns)<=p[0]<=max(side_columns)]
        center_x=(left[0]+right[0])/2
        slope0=(right[1]-left[1])/(right[0]-left[0])
        minimum=max(6,min(12,(len(set(side_columns))+1)//2))
        for index in range(9):
            slope=slope0-.4+.1*index
            bins={}
            for px,py in side_votes:
                key=int(round((py-slope*(px-center_x))/2))
                bins[key]=bins.get(key,0)+1
            # Tied bins need the same order on MicroPython and the PC.
            for key in sorted(bins,key=lambda k:(-bins[k],k))[:2]:
                support=[p for p in side_votes if abs(p[1]-slope*(p[0]-center_x)-2*key)<=2.5]
                if len(set(p[0] for p in support))<minimum:continue
                line=fit(support)
                selected=[p for p in support if residual(p,line)<=3]
                if len(set(p[0] for p in selected))>=minimum and abs(line[1])>1e-6:
                    local_hypotheses.append((len(set(p[0] for p in selected)),fit(selected),left,right))
    local_tried=[]
    for count,line,left,right in sorted(local_hypotheses,key=lambda item:-item[0]):
        if seam_tries>=3:break
        center_x=(left[0]+right[0])/2
        middle=-(line[0]*center_x+line[2])/line[1]
        if any(abs(middle-old[0])<8 and abs(center_x-old[1])<8 for old in local_tried):continue
        local_tried.append((middle,center_x))
        if len(local_tried)>3:break
        upper=[[p[0],-(line[0]*p[0]+line[2])/line[1]] for p in (left,right)]
        if min(left[1]-upper[0][1],right[1]-upper[1][1])<25:continue
        try:
            seam_tries+=1
            for step in _edge_steps(im,upper[0],upper[1],hsv=hsv):
                if step['stage']=='edge_complete':top_edge=step['edge']
                yield {'stage':'seam_'+step['stage']}
            upper=[intersect(top_edge[0],[1.,0.,-p[0]]) for p in (left,right)]
            measured_middle=-(top_edge[0][0]*center_x+top_edge[0][2])/top_edge[0][1]
            if not first<=measured_middle<last:raise ValueError('Measured separator left its searched band')
            q=upper+[list(right),list(left)]
            _separator_evidence(hsv,q,top_edge,im)
        except ValueError:continue
        yield {'stage':'seam_seed','seed':q,'top_edge':top_edge}
        return
    if recover_internal:
        recovery = hypotheses or [(count, line) for count, line, left, right in local_hypotheses]
        for step in _recover_seam_steps(im, hsv, bbox, vertices, recovery, first, last):
            yield step


def _contour_side_seed(seed, edge_index, vertices, contour=None):
    """Recover a rounded endpoint using a measured-outline search initialisation.

    Only a long, parallel contour segment alongside the failed side can seed
    this bounded retry. Component extrema alone cannot supply a missing edge.
    Every retry still measures all four sides and passes normal face checks.
    """
    if edge_index not in (1, 2, 3):
        return None
    a, b = seed[edge_index], seed[(edge_index+1) % 4]
    length = distance(a, b)
    if length < 25:
        return None
    u = [(b[k]-a[k])/length for k in (0, 1)]
    n = [-u[1], u[0]]
    center = [sum(p[k] for p in seed)/4 for k in (0, 1)]
    middle = [(a[k]+b[k])/2 for k in (0, 1)]
    inward = sum((center[k]-middle[k])*n[k] for k in (0, 1))
    limit = max(18., .25*distance(seed[0], seed[1]))
    proposals = []
    for c, d in zip(vertices, vertices[1:]+vertices[:1]):
        other = distance(c, d)
        if other < .5*length or abs(sum((d[k]-c[k])*u[k] for k in (0, 1)))/other < .94:
            continue
        along = [sum((p[k]-a[k])*u[k] for k in (0, 1)) for p in (c, d)]
        overlap = max(0., min(length, max(along))-max(0., min(along)))
        if overlap < .5*length:
            continue
        line = fit([c, d])
        denominator = sum(line[k]*n[k] for k in (0, 1))
        if abs(denominator) < 1e-6:
            continue
        gap = -(sum(line[k]*middle[k] for k in (0, 1))+line[2])/denominator
        minimum = min(12, max(5, int(round(length*.06)))) if edge_index == 2 else 3.
        if gap*inward >= 0 or not minimum < abs(gap) <= limit:
            continue
        if edge_index == 2 and not _outline_support(c, d, contour):
            continue
        candidate = [list(p) for p in seed]
        try:
            before = fit([seed[(edge_index-1) % 4], a])
            after = fit([b, seed[(edge_index+2) % 4]])
            candidate[edge_index] = intersect(before, line)
            candidate[(edge_index+1) % 4] = intersect(line, after)
        except ValueError:
            continue
        if corner_distance(seed, candidate) <= limit:
            proposals.append((overlap, -abs(gap), candidate))
    return max(proposals, key=lambda item: item[:2])[2] if proposals else None


def localize_steps(bgr, frame, bbox_px, intrinsics, extrinsics, table_plane,
                   max_candidates=3, roi_padding_px=24, stations_per_step=8,
                   side_search_height_ratio=1.6, include_range=False):
    """Yield bounded work, then one ``complete`` shared precise observation.

    Caller may advance several steps within each control-cycle budget, keeping
    the owned BGR image unchanged until completion/cancellation. Coordinates and
    timestamps belong to the ORIGINAL captured frame, not to completion time.
    ``max_candidates`` limits search rather than weakening face acceptance.
    No top fallback, global recovery, camera ownership, UART or SD operations.
    """
    if bgr.shape != (960, 1280, 3):
        raise ValueError('Expected full 1280x960 JPEG95-decoded BGR')
    if type(max_candidates) is not int or not 1 <= max_candidates <= 8:
        raise ValueError('Choose 1..8 physical-side candidates')
    if type(stations_per_step) is not int or not 1 <= stations_per_step <= 64:
        raise ValueError('Choose 1..64 edge stations per cooperative step')
    if not isinstance(roi_padding_px, (int, float)) or not math.isfinite(roi_padding_px) or roi_padding_px < 16:
        raise ValueError('ROI padding must retain edge search support')
    if (not isinstance(side_search_height_ratio, (int, float)) or
            not math.isfinite(side_search_height_ratio) or side_search_height_ratio < 1):
        raise ValueError('Choose a side search height relative to coarse width')
    frame = dict(frame)
    started = _now()
    raw = {'status': 'no_complete_face', 'reason': 'No verified physical side in bounded ROI'}
    # Public shared wrapper validates frame/calibration shape; no full localizer
    # is imported or called through this injected, already-computed result.
    localize_precise(None, frame, intrinsics, extrinsics, table_plane,
                     localizer=lambda *args: raw)
    K = [list(row) for row in intrinsics['camera_matrix']]
    K[0] = [v*1280/intrinsics['image_width'] for v in K[0]]
    K[1] = [v*960/intrinsics['image_height'] for v in K[1]]
    D, T = intrinsics['distortion_coefficients'], extrinsics['T_Base_from_Camera_mm']
    table = table_context(T, table_plane)
    if table is None:
        raise ValueError('Near parking requires the paired calibrated table plane')
    if bbox_px is None:
        hsv_full = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv_full, (90, 120, 35), (140, 255, 255))
        contours, unused = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        # Select a near component, preserving its FULL outline. Cropping the
        # mask at row432 first can remove a cube's real top/side separator.
        candidates = [c for c in contours if cv2.contourArea(c) >= 1000 and
                      cv2.boundingRect(c)[1]+cv2.boundingRect(c)[3] > 432]
        bbox_px = list(cv2.boundingRect(max(candidates, key=cv2.contourArea))) if candidates else None
        yield {'stage': 'component'}
        if bbox_px is None:
            raw['reason'] = 'No near blue component for physical-side localization'
            observation = localize_precise(None, frame, intrinsics, extrinsics, table_plane,
                                          localizer=lambda *args: raw)
            observation['processing_ms'] = _elapsed(started)
            observation['search'] = {'roi_px': None, 'seed_count': 0, 'measured_seed_count': 0,
                'candidate_limit': max_candidates, 'usable_sides': 0, 'rejected': [],
                'top_fallback': False, 'sampling_backend': 'native_remap' if _BATCH_EDGES else 'portable'}
            yield {'stage': 'complete', 'observation': observation}
            return
    x0, y0, x1, y1 = _roi(bbox_px, roi_padding_px, bgr.shape, side_search_height_ratio)
    im = np.array(bgr[y0:y1, x0:x1], dtype=np.uint8)
    hsv = cv2.cvtColor(im, cv2.COLOR_BGR2HSV)
    smooth = cv2.GaussianBlur(im, (3, 3), .7)
    broad = cv2.inRange(hsv, (90, 120, 35), (140, 255, 255))
    cs, unused = cv2.findContours(broad, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    eligible = [c for c in cs if cv2.contourArea(c) >= 1000]
    seeds, faces, rejected, anchor, top_hints = [], [], [], None, []
    range_raw_faces = []
    yield {'stage': 'roi', 'roi_px': [x0, y0, x1-x0, y1-y0]}
    if eligible:
        contour = max(eligible, key=cv2.contourArea)
        support = cv2.approxPolyDP(contour, .003*cv2.arcLength(contour, True), True)
        anchor = {'component_contour_px': support.reshape((-1, 2)).tolist()}
        hist = cv2.calcHist([hsv], [0], broad, [180], [0., 180.]).reshape((-1,)).tolist()
        modes = []
        for unused in range(4):
            mode = max(range(90, 141), key=lambda i: hist[i])
            if hist[mode] < 100:
                break
            modes.append(mode)
            for i in range(mode-2, mode+3):
                hist[i] = 0
        ranges = [((max(90, m-band), 120, 35), (min(140, m+band), 255, 255))
                  for m in modes for band in (1, 2, 4)]
        ranges += [((90, 120, low), (140, 255, high))
                   for low, high in ((40, 120), (80, 160), (120, 200), (160, 240), (200, 255))]
        values = cv2.calcHist([hsv], [2], broad, [256], [0., 256.]).reshape((-1,)).tolist()
        total, cumulative, cuts = sum(values), 0, []
        for value, count in enumerate(values):
            cumulative += count
            while len(cuts) < 4 and total and cumulative >= total*(len(cuts)+1)/5:
                cuts.append(value)
        ranges += [((90, 120, max(35, low-5)), (140, 255, min(255, high+5)))
                   for low, high in zip([35]+cuts, cuts+[255]) if high-low >= 12]
        ranges.append(((90, 120, 35), (140, 255, 255)))
        for lo, hi in ranges:
            mask = cv2.inRange(hsv, lo, hi)
            mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, np.ones((3, 3), dtype=np.uint8))
            cs, unused = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            for c in cs:
                if cv2.contourArea(c) < 1000:
                    continue
                hull = cv2.convexHull(c)
                poly = cv2.approxPolyDP(hull, .02*cv2.arcLength(hull, True), True)
                if len(poly) != 4:
                    continue
                q = ordered(poly)
                if not component_support(q, anchor):
                    continue
                if shallow_face(q):
                    if not any(corner_distance(q, old) < 3 for old in top_hints):
                        top_hints.append(q)
                    continue
                if not any(corner_distance(q, old) < 3 for old in seeds):
                    seeds.append(q)
            yield {'stage': 'proposals', 'seed_count': len(seeds)}
        # Observed top edges and lower contour vertices may propose a side;
        # these are search seeds only, still requiring four measured edges.
        hull = cv2.convexHull(contour)
        poly = cv2.approxPolyDP(hull, .01*cv2.arcLength(hull, True), True)
        vertices = poly.reshape((-1, 2)).tolist()
        for hint in sorted(top_hints, key=lambda q:
                (max(p[1] for p in q)-min(p[1] for p in q)) /
                max(1., max(p[0] for p in q)-min(p[0] for p in q)))[:2]:
            span = max(max(p[k] for p in hint)-min(p[k] for p in hint) for k in (0, 1))
            for a, b in zip(hint, hint[1:]+hint[:1]):
                if b[0]-a[0] >= -.25*distance(a, b):
                    continue
                lower = [v for v in vertices if v[1] > max(a[1], b[1])+.3*span]
                if not lower:
                    continue
                ends = [min(lower, key=lambda v: abs(v[0]-p[0])) for p in (a, b)]
                if max(abs(v[0]-p[0]) for v, p in zip(ends, (a, b))) > .25*span:
                    continue
                q = ordered(np.array([b, a, ends[0], ends[1]], dtype=FLOAT))
                if not any(corner_distance(q, old) < 3 for old in seeds):
                    seeds.append(q)
            yield {'stage': 'side_proposals', 'seed_count': len(seeds)}
        priority, cached_edges = [], {}
        local_bbox = [bbox_px[0]-x0, bbox_px[1]-y0, bbox_px[2], bbox_px[3]]
        for step in _seam_seed_steps(smooth, hsv, local_bbox, vertices,
                reuse_local=not seeds and not include_range,
                recover_internal=not include_range and _recoverable_outline_seeds(
                    seeds, anchor['component_contour_px'])):
            if step['stage'] == 'seam_seed':
                q = step['seed']
                if component_support(q, anchor):
                    priority.append(q)
                    cached_edges[id(q)] = step['top_edge']
            yield {'stage': step['stage']}
        seeds.sort(key=lambda q: -area(q))
        seeds = priority+[q for q in seeds if not any(corner_distance(q, old) < 3 for old in priority)]
        measured, pose_attempts = 0, 0
        contour_retry = None
        # A corrected seed can use only a later same-side slot already in the
        # original list; existing priority, successes and face budget stay put.
        bottom_retry = None
        for seed in seeds:
            if not include_range and bottom_retry is not None and not faces and corner_distance(seed, bottom_retry[0]) < 18:
                seed = bottom_retry[1]
                bottom_retry = None
            if seed is contour_retry and faces:
                continue  # Any original strict result keeps its old priority.
            full_seed = [[p[0]+x0, p[1]+y0] for p in seed]
            if any(corner_distance(full_seed, f['face_quad_px']) < 18 for f in faces):
                # A verified side already owns this proposal's edge-search
                # neighbourhood; reserve attempts for a different side.
                yield {'stage': 'same_side_proposal'}
                continue
            if faces and not any(shared_edge(full_seed, f['face_quad_px']) for f in faces):
                yield {'stage': 'unrelated_side_proposal'}
                continue
            # Discard a visibly merged silhouette before spending the limited
            # physical-edge attempts. This same test is repeated after fits.
            merged = internal_seam(hsv, seed) is not None
            yield {'stage': 'proposal_seam', 'merged': merged}
            if merged:
                continue
            if measured >= max_candidates:
                break
            index, measured = measured, measured+1
            try:
                edges = []
                failed_edge = None
                outline_profile = {} if not include_range else None
                for edge_index, (a, b) in enumerate(zip(seed, seed[1:]+seed[:1])):
                    if edge_index == 0 and id(seed) in cached_edges:
                        edges.append(cached_edges[id(seed)])
                        yield {'stage': 'edge_reused', 'candidate': index, 'edge_index': 0}
                        continue
                    failed_edge = edge_index
                    profile = outline_profile if edge_index == 1 else None
                    for step in _edge_steps(smooth, a, b, stations_per_step, hsv=hsv, profile=profile):
                        if step['stage'] == 'edge_complete':
                            edges.append(step['edge'])
                        yield {'stage': step['stage'], 'candidate': index,
                               'edge_index': edge_index, 'stations': step.get('stations')}
                    failed_edge = None
                for recovered_outline in (False, True):
                    if recovered_outline:
                        # A failed square pose may expose a right-edge profile
                        # mixing the face crease with its dark side/shadow.
                        # Reuse its64 samples; choose straightness, never a
                        # table/target/PnP residual, to assign those peaks.
                        if (not outline_profile or edges[1][1]['support'] == 64 or
                                outline_profile['b'][1]-outline_profile['a'][1] <=
                                2*abs(outline_profile['b'][0]-outline_profile['a'][0])):
                            break
                        data = outline_profile
                        allowed = _blue_grid(hsv, data['a'], data['b'], data['normal'],
                                             data['radius'], inside_only=True)
                        repaired = _coherent_internal_edge(data['a'], data['b'], data['normal'],
                            data['radius'], data['rows'], allowed, hsv, smooth, outline=True)
                        if repaired[1]['rms_px'] >= edges[1][1]['rms_px']:
                            break
                        edges[1] = repaired
                        yield {'stage': 'outline_profile_recovered', 'candidate': index}
                    q = [intersect(edges[i-1][0], edges[i][0]) for i in range(4)]
                    for half in (0, 1):
                        check = [intersect(edges[i-1][2][half], edges[i][2][half]) for i in range(4)]
                        if max(distance(a, b) for a, b in zip(q, check)) > 3:
                            raise ValueError('Side interleaved corners disagree')
                    if corner_distance(q, seed) > 18:
                        raise ValueError('Side edges too far from proposal')
                    full_q = [[p[0]+x0, p[1]+y0] for p in q]
                    if any(not (3<p[0]<1277 and 3<p[1]<957) for p in full_q):
                        raise ValueError('Side clipped')
                    if area(q) < 1000 or not cv2.isContourConvex(np.array(q, dtype=FLOAT)):
                        raise ValueError('Side not convex')
                    if not component_support(q, anchor) or internal_seam(hsv, q) is not None:
                        raise ValueError('Side proposal contains another face')
                    yield {'stage': 'side_boundary', 'candidate': index}
                    back = min(range(4), key=lambda i: (q[i][1]+q[(i+1)%4][1])/2)
                    separator = _separator_evidence(hsv, q[back:]+q[:back], edges[back], im)
                    yield {'stage': 'side_seam', 'candidate': index}
                    if pose_attempts >= max_candidates:
                        raise ValueError('Physical-side pose attempt budget exhausted')
                    pose_attempts += 1
                    poses = side_pose(pose_candidates(full_q, K, D, refine=True, refiner=refine_pose))
                    if poses:
                        break
                if not poses:
                    raise ValueError('No accurate vertical-side pose')
                face = {'face_quad_px': full_q, 'poses': poses, 'physical_edges': [e[1] for e in edges],
                        'seam_support': separator['support'], 'seam_evidence': separator, 'area_px2': area(q),
                        'boundary_method': 'independent_side_four_measured_edges'}
                if include_range:
                    range_raw_faces.append(face)
                face = constrain_face(face, table)
                if face is None:
                    raise ValueError('No side pose consistent with calibrated table')
                duplicate = next((f for f in faces if corner_distance(full_q, f['face_quad_px']) < 6), None)
                if duplicate is None:
                    faces.append(face)
                elif poses[0]['face_reprojection_rms_px'] < duplicate['poses'][0]['face_reprojection_rms_px']:
                    faces[faces.index(duplicate)] = face
                yield {'stage': 'side_pose', 'candidate': index}
            except (ValueError, ZeroDivisionError) as exc:
                rejected.append({'candidate': index, 'reason': str(exc)})
                if not include_range and str(exc) == 'No accurate vertical-side pose' and bottom_retry is None:
                    corrected = _contour_side_seed(seed, 2, vertices, anchor['component_contour_px'])
                    if corrected is not None and component_support(corrected, anchor):
                        bottom_retry = (seed, corrected)
                if not include_range and contour_retry is None and failed_edge in (1, 3):
                    retry = _contour_side_seed(seed, failed_edge, vertices)
                    if retry is not None and component_support(retry, anchor) and not any(
                            corner_distance(retry, old) < 3 for old in seeds):
                        # Original proposals stay first, and the existing max3
                        # attempt limit remains. No cached edge is reused.
                        seeds.append(retry)
                        contour_retry = retry
                yield {'stage': 'candidate_rejected', 'candidate': index, 'reason': str(exc)}
    try:
        selected = select_sides(faces, K, D, table)
        if selected is not None:
            center = selected['poses'][0]['cube_center_Camera_mm']
            selected.update(cube_center_Camera_mm=center,
                cube_center_Base_mm=[sum(T[i][j]*center[j] for j in range(3))+T[i][3] for i in range(3)],
                vision_path='shared_bounded_physical_sides')
            raw = selected
    except ValueError as exc:
        raw = {'status': 'no_complete_face', 'reason': str(exc)}
    observation = localize_precise(None, frame, intrinsics, extrinsics, table_plane,
                                  localizer=lambda *args: raw)
    if include_range and observation['status'] != 'localized':
        try:
            from .side_range import range_candidate as _range_candidate
        except ImportError:
            from side_range import range_candidate as _range_candidate
        try:
            candidate = _range_candidate(range_raw_faces, K, D, T, table)
            if candidate is not None:
                observation['range_candidate'] = candidate
        except ValueError:
            pass  # The unchanged side-pair checks can reject the range too.
    observation['processing_ms'] = _elapsed(started)
    observation['search'] = {'roi_px': [x0, y0, x1-x0, y1-y0], 'seed_count': len(seeds),
        'measured_seed_count': measured if eligible else 0, 'candidate_limit': max_candidates,
        'usable_sides': len(faces), 'rejected': rejected, 'top_fallback': False,
        'sampling_backend': 'native_remap' if _BATCH_EDGES else 'portable'}
    yield {'stage': 'complete', 'observation': observation}


def localize(im, K, D, T, table_plane, bbox_px=None, **options):
    """Shared raw synchronous interface; no acquisition time is manufactured.

    Matches the existing physical-face localizer signature. Arm callers may
    omit the box and use the existing current-frame near-blue component proposal.
    Car callers should provide their same-frame box and use localize_steps to
    retain acquisition metadata while scheduling motor communication.
    """
    intr = {'camera_matrix': K, 'distortion_coefficients': D,
            'image_width': 1280, 'image_height': 960}
    frame = {'frame_id': 'synchronous_current_image', 'capture_timestamp_ms': None,
             'image_size_px': [1280, 960], 'clock_domain': None,
             'timestamp_source': 'unavailable'}
    for step in localize_steps(im, frame, bbox_px, intr, {'T_Base_from_Camera_mm': T}, table_plane, **options):
        if step['stage'] == 'complete':
            observation = step['observation']
    if observation['status'] != 'localized':
        raise ValueError(observation['reason'])
    return observation['payload']
