"""One same-frame, single-side appearance recovery after static localization.

The bbox proposes a search window, never a face or a precise position. The
profile models choose image measurements; the existing physical-side geometry
and calibrated-table checks remain the authority. No capture or motion here.
"""
import math


def recover_steps(bgr, frame, bbox_px, intrinsics, extrinsics, table_plane,
                  roi_padding_px=24, side_search_height_ratio=1.6, prepared=None):
    """Try one measured face, yielding the ordinary precise observation schema.

    Call only after the original static bounded localizer failed on this same
    owned image. There is no second image, candidate loop, range-only output or
    pose-guided edge selection. Odd/even fits use distinct measured points in
    the common candidate window, not independent global appearance searches.
    """
    # Delayed imports let physical_side call this fallback without an import
    # cycle, and retain the flat module names used by the board deployment.
    try:
        from . import physical_side as p
        from . import edge_profiles, separator_profiles
    except ImportError:
        import physical_side as p
        import edge_profiles
        import separator_profiles
    cv2, np = p.cv2, p.np
    if bgr.shape != (960, 1280, 3):
        raise ValueError('Expected full 1280x960 JPEG95-decoded BGR')
    if (not isinstance(roi_padding_px, (int, float)) or
            not math.isfinite(roi_padding_px) or roi_padding_px < 16):
        raise ValueError('ROI padding must retain edge search support')
    if (not isinstance(side_search_height_ratio, (int, float)) or
            not math.isfinite(side_search_height_ratio) or side_search_height_ratio < 1):
        raise ValueError('Choose a side search height relative to coarse width')
    frame = dict(frame)
    started = p._now()
    raw = {'status': 'no_complete_face',
           'reason': 'No verified physical side in profile recovery'}
    p.localize_precise(None, frame, intrinsics, extrinsics, table_plane,
                       localizer=lambda *args: raw)
    K = [list(row) for row in intrinsics['camera_matrix']]
    K[0] = [v*1280/intrinsics['image_width'] for v in K[0]]
    K[1] = [v*960/intrinsics['image_height'] for v in K[1]]
    D = intrinsics['distortion_coefficients']
    T = extrinsics['T_Base_from_Camera_mm']
    table = p.table_context(T, table_plane)
    if table is None:
        raise ValueError('Near parking requires the paired calibrated table plane')
    metadata = {'method': 'same_frame_single_profile_face', 'candidate_limit': 1,
                'candidate_window_shared': True,
                'odd_even_check': 'independent_measured_point_line_fits'}
    rejected, faces, roi, measured = [], [], None, 0
    if bbox_px is None:
        hsv_full = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv_full, (90, 120, 35), (140, 255, 255))
        contours, unused = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        candidates = [c for c in contours if cv2.contourArea(c) >= 1000 and
                      cv2.boundingRect(c)[1]+cv2.boundingRect(c)[3] > 432]
        bbox_px = list(cv2.boundingRect(max(candidates, key=cv2.contourArea))) if candidates else None
        yield {'stage': 'component'}
    if bbox_px is None:
        raw['reason'] = 'No near blue component for physical-side localization'
    else:
        if prepared is None:
            x0, y0, x1, y1 = p._roi(bbox_px, roi_padding_px, bgr.shape, side_search_height_ratio)
            roi = [x0, y0, x1-x0, y1-y0]
            im = np.array(bgr[y0:y1, x0:x1], dtype=np.uint8)
            hsv = cv2.cvtColor(im, cv2.COLOR_BGR2HSV)
            smooth = cv2.GaussianBlur(im, (3, 3), .7)
            broad = cv2.inRange(hsv, (90, 120, 35), (140, 255, 255))
            contours, unused = cv2.findContours(broad, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            eligible = [c for c in contours if cv2.contourArea(c) >= 1000]
        else:
            roi = prepared['roi_px']
            x0, y0 = roi[:2]
            im, hsv, smooth = prepared['raw'], prepared['hsv'], prepared['smooth']
            eligible = [prepared['contour']]
        yield {'stage': 'roi', 'roi_px': roi}
        if eligible:
            contour = max(eligible, key=cv2.contourArea)
            support = cv2.approxPolyDP(contour, .003*cv2.arcLength(contour, True), True)
            anchor = {'component_contour_px': support.reshape((-1, 2)).tolist()}
            local_bbox = [bbox_px[0]-x0, bbox_px[1]-y0, bbox_px[2], bbox_px[3]]
            try:
                top, details = separator_profiles.measure(
                    smooth, hsv, local_bbox, p._fit_edge, p.fit, p.residual)
                metadata['separator_profile'] = details
                yield {'stage': 'profile_top', 'candidate': 0}
                x, y, w, h = local_bbox
                left, right = [x, y+h-1], [x+w-1, y+h-1]
                upper = [p.intersect(top[0], [1., 0., -point[0]]) for point in (left, right)]
                if min(left[1]-upper[0][1], right[1]-upper[1][1]) < 25:
                    raise ValueError('Edge too short')
                seed = upper+[right, left]
                if not p.component_support(seed, anchor):
                    raise ValueError('Side proposal outside blue component')
                measured = 1
                edges = [top]
                for edge_index in (1, 2, 3):
                    a, b = seed[edge_index], seed[(edge_index+1) % 4]
                    spread = edge_profiles.spread_edge(
                        smooth, hsv, a, b, p._fit_edge, p._native_contrasts)
                    if spread is None:
                        for step in p._edge_steps(smooth, a, b, hsv=hsv,
                                coherent_outline=edge_index in (1, 3) and
                                    abs(b[1]-a[1]) > 2*abs(b[0]-a[0])):
                            if step['stage'] == 'edge_complete':
                                edge = step['edge']
                            yield {'stage': step['stage'], 'candidate': 0,
                                   'edge_index': edge_index, 'stations': step.get('stations')}
                    else:
                        edge, unused = spread
                        yield {'stage': 'edge_complete', 'candidate': 0,
                               'edge_index': edge_index, 'stations': 64}
                    edges.append(edge)
                q = [p.intersect(edges[i-1][0], edges[i][0]) for i in range(4)]
                for half in (0, 1):
                    check = [p.intersect(edges[i-1][2][half], edges[i][2][half]) for i in range(4)]
                    if max(p.distance(a, b) for a, b in zip(q, check)) > 3:
                        raise ValueError('Side interleaved corners disagree')
                if p.corner_distance(q, seed) > 18:
                    raise ValueError('Side edges too far from proposal')
                full_q = [[point[0]+x0, point[1]+y0] for point in q]
                if any(not (3<point[0]<1277 and 3<point[1]<957) for point in full_q):
                    raise ValueError('Side clipped')
                if p.area(q) < 1000 or not cv2.isContourConvex(np.array(q, dtype=p.FLOAT)):
                    raise ValueError('Side not convex')
                if not p.component_support(q, anchor) or p.internal_seam(hsv, q) is not None:
                    raise ValueError('Side proposal contains another face')
                yield {'stage': 'side_boundary', 'candidate': 0}
                back = min(range(4), key=lambda i: (q[i][1]+q[(i+1) % 4][1])/2)
                separator = p._separator_evidence(hsv, q[back:]+q[:back], edges[back], im)
                yield {'stage': 'side_seam', 'candidate': 0}
                poses = p.side_pose(p.pose_candidates(full_q, K, D, refine=True, refiner=p.refine_pose))
                if not poses:
                    raise ValueError('No accurate vertical-side pose')
                face = {'face_quad_px': full_q, 'poses': poses,
                        'physical_edges': [edge[1] for edge in edges],
                        'seam_support': separator['support'], 'seam_evidence': separator,
                        'area_px2': p.area(q), 'boundary_method': 'image_profile_four_measured_edges'}
                face = p.constrain_face(face, table)
                if face is None:
                    raise ValueError('No side pose consistent with calibrated table')
                faces.append(face)
                yield {'stage': 'side_pose', 'candidate': 0}
            except (ValueError, ZeroDivisionError) as exc:
                raw['reason'] = str(exc)
                rejected.append({'candidate': 0, 'reason': str(exc)})
                yield {'stage': 'candidate_rejected', 'candidate': 0, 'reason': str(exc)}
        try:
            selected = p.select_sides(faces, K, D, table)
            if selected is not None:
                center = selected['poses'][0]['cube_center_Camera_mm']
                selected.update(cube_center_Camera_mm=center,
                    cube_center_Base_mm=[sum(T[i][j]*center[j] for j in range(3))+T[i][3] for i in range(3)],
                    vision_path='shared_bounded_physical_sides', profile_recovery=metadata)
                raw = selected
        except ValueError as exc:
            raw = {'status': 'no_complete_face', 'reason': str(exc)}
    observation = p.localize_precise(None, frame, intrinsics, extrinsics, table_plane,
                                     localizer=lambda *args: raw)
    observation['processing_ms'] = p._elapsed(started)
    observation['profile_recovery'] = metadata
    observation['search'] = {'roi_px': roi, 'seed_count': measured,
        'measured_seed_count': measured, 'candidate_limit': 1, 'usable_sides': len(faces),
        'rejected': rejected, 'top_fallback': False, 'profile_recovery': True,
        'sampling_backend': 'native_remap' if p._BATCH_EDGES else 'portable'}
    yield {'stage': 'complete', 'observation': observation}
