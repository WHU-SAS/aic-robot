"""One same-frame static physical-face measurement with typed top evidence.

The bbox proposes a search window, never a face or a precise position. The
profile models choose image measurements; the existing physical-side geometry
and calibrated-table checks remain the authority. No capture or motion here.
"""
import math


def _terminal_face_seed(p,smooth,hsv,box,top,details,origin,
                        intrinsics,extrinsics,table_plane,outline=False):
    """An observed separator end proposes one narrower face, never a pose.

    A gap beyond the existing12px maximum edge-search radius nominates an
    internal crease. Its actual position still comes from64 RGB profiles.
    The lower contour supplies a slanted bottom proposal for that face.
    """
    terminal=details.get('terminal_support',{})
    gaps={side:terminal.get(side+'_gap_px',0.)for side in('left','right')}
    side=max(gaps,key=gaps.get)
    if gaps[side]<=12:return None
    try:from . import cube_corners
    except ImportError:import cube_corners
    x,y,w,h=box
    point=terminal['first_point_px'if side=='left'else'last_point_px']
    slope=cube_corners._vertical_slope(p,[point[0]+origin[0],point[1]+origin[1]],
        intrinsics,extrinsics,table_plane)
    a,b=[[point[0]+slope*(yy-point[1]),yy]for yy in(y+.35*h,y+.85*h)]
    edge_a,edge_b=(b,a)if outline and side=='left'else(a,b)
    for event in p._edge_steps(smooth,edge_a,edge_b,hsv=hsv,
            coherent=not outline,coherent_outline=outline):
        if event['stage']=='edge_complete':crease=event['edge']
    crease_seed=p.fit([a,b])
    left_seed=[1.,0.,-x]if side=='right'else crease_seed
    right_seed=crease_seed if side=='right'else[1.,0.,-(x+w-1)]
    lower=y+h-1
    lx=p.intersect(left_seed,[0.,1.,-lower])[0]
    rx=p.intersect(right_seed,[0.,1.,-lower])[0]
    runs,unused=cube_corners._bottom_runs(p,hsv,box)
    choices=[]
    for run in runs:
        if min(run['a'][1],run['b'][1])<=y+.65*h or run['dx']<=0:continue
        overlap=max(0.,min(rx,run['b'][0])-max(lx,run['a'][0]))
        if overlap>=25:choices.append((overlap*overlap/run['dx'],run))
    if not choices:raise ValueError('No front-bottom contour within the observed ridge face')
    unused,run=max(choices,key=lambda item:item[0])
    seed=[p.intersect(top[0],left_seed),p.intersect(top[0],right_seed),
          p.intersect(right_seed,run['line']),p.intersect(left_seed,run['line'])]
    return seed,{1 if side=='right'else 3:crease},dict(
        method='observed_separator_terminal_wall_and_front_bottom',side=side,
        separator_descriptor=details.get('descriptor'),
        terminal_gap_px=gaps[side],crease_seed_roi_px=[edge_a,edge_b],
        crease_quality=crease[1],bottom_contour_seed_roi_px=[run['a'],run['b']])


def recover_steps(bgr, frame, bbox_px, intrinsics, extrinsics, table_plane,
                  roi_padding_px=24, side_search_height_ratio=1.6, prepared=None,
                  top_method='auto', cue=None, refine_cube=False):
    """Try one measured face with B-step or approved G-ridge top evidence.

    The caller owns static routing and the unchanged captured image. Auto
    chooses one top measurer from the shared cue; it never runs both complete
    models. Prepared ROI data and its separator_cue belong to this same image.
    There is no capture, candidate loop, range output or pose-guided pixels.
    Odd/even fits use distinct measured points in the common candidate window.
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
    if top_method not in ('auto','B','ridge','RGB'):
        raise ValueError('Choose auto, B, ridge or RGB top measurement')
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
                if cue is None and prepared is not None:
                    cue=prepared.get('separator_cue')
                if cue is None:
                    cue=separator_profiles.cue24(smooth,hsv,local_bbox,p.fit,p.residual)
                    if prepared is not None:prepared['separator_cue']=cue
                selected_method=top_method
                if selected_method=='auto':
                    if cue[1].get('method')=='same_frame_RGB_step_cue':
                        selected_method='RGB'
                    else:
                        hint=cue[1].get('B_observability',{})
                        selected_method='B'if hint.get('positive_B_step_observable',True)else'ridge'
                metadata['top_method'] = selected_method
                if selected_method=='ridge':
                    try:from . import ridge_profiles
                    except ImportError:import ridge_profiles
                    measurer=ridge_profiles.measure
                elif selected_method=='RGB':
                    measurer=separator_profiles.measure_rgb_step
                else:measurer=separator_profiles.measure
                top, details = measurer(smooth, hsv, local_bbox,
                    p._fit_edge, p.fit, p.residual, cue=cue)
                metadata['separator_profile'] = details
                yield {'stage': 'profile_top', 'candidate': 0}
                x, y, w, h = local_bbox
                left, right = [x, y+h-1], [x+w-1, y+h-1]
                upper = [p.intersect(top[0], [1., 0., -point[0]]) for point in (left, right)]
                seed = upper+[right, left]
                reused={}
                if selected_method in ('ridge','RGB') or (refine_cube and selected_method=='B'):
                    try:
                        proposal=_terminal_face_seed(p,smooth,hsv,local_bbox,top,details,
                            [x0,y0],intrinsics,extrinsics,table_plane,outline=True)
                    except (ValueError,ZeroDivisionError) as exc:
                        # A highlight can end on a flat face. Its endpoint is
                        # only a proposal until a real wall is measured.
                        metadata['face_proposal_rejected']={'reason':str(exc)}
                        proposal=None
                    if proposal is not None:
                        seed,reused,metadata['face_proposal']=proposal
                if min(seed[3][1]-seed[0][1],seed[2][1]-seed[1][1]) < 25:
                    raise ValueError('Edge too short')
                if not p.component_support(seed, anchor):
                    raise ValueError('Side proposal outside blue component')
                measured = 1
                edges = [top]
                for edge_index in (1, 2, 3):
                    a, b = seed[edge_index], seed[(edge_index+1) % 4]
                    if edge_index in reused:
                        edges.append(reused[edge_index])
                        yield {'stage':'edge_reused','candidate':0,'edge_index':edge_index,'stations':64}
                        continue
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
                if refine_cube and (selected_method in ('ridge','RGB') or
                                    metadata.get('face_proposal') is not None):
                    # The short proposal band can fit a reflection on the
                    # packaging. Measure the actual top-to-bottom wall span
                    # before intersecting half-lines or solving any pose.
                    terminal=details.get('terminal_support',{})
                    endpoint=terminal.get('last_point_px') if 'full_bbox' in terminal.get('method','') else None
                    wall,audit=edge_profiles.wall_edge(p,smooth,hsv,
                        seed[1],seed[2],q[1],q[2],endpoint)
                    metadata['whole_wall_profile']=audit
                    if wall is not None:
                        edges[1]=wall
                        q=[p.intersect(edges[i-1][0],edges[i][0])for i in range(4)]
                    yield {'stage':'wall_profile','candidate':0,'accepted':wall is not None}
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
                if selected_method=='ridge':
                    if back!=0:raise ValueError('Measured G ridge does not own upper face edge')
                    separator=dict(method='same_frame_G_ridge',
                        evidence_type='G_peak_two_flanks_DN',support=top[1]['support'],
                        sampled_stations=64,minimum_prominence_DN=2.,flank_span_px=8.,
                        both_blue_stations=top[1]['support'],
                        prominence_min_DN=details['prominence_min_DN'],
                        prominence_median_DN=details['prominence_median_DN'],
                        original_RGB8_step_evidence=False)
                else:
                    separator = p._separator_evidence(hsv, q[back:]+q[:back], edges[back], im)
                yield {'stage': 'side_seam', 'candidate': 0}
                if refine_cube and (selected_method in ('ridge','RGB') or
                                    metadata.get('face_proposal') is not None):
                    try:
                        from .cube_corners import front_back_steps
                        from .cube_observation import build_result
                    except ImportError:
                        from cube_corners import front_back_steps
                        from cube_observation import build_result
                    context=prepared or {'raw':im,'hsv':hsv,'smooth':smooth,
                        'local_bbox':local_bbox,'roi_px':roi,
                        'contour':contour,'vertices':support.reshape((-1,2)).tolist()}
                    refinement={'candidate_count':0,'rejections':[]}
                    metadata['cube_refinement']=refinement
                    for step in front_back_steps(p,smooth,hsv,context,edges,q,
                            intrinsics,extrinsics,table_plane):
                        if step['stage']=='cube_candidate':
                            refinement['candidate_count']+=1
                            # Once observed nonplanar points contradict a cube,
                            # a weaker planar fit must not erase that evidence.
                            raw=build_result(step['measurement'],K,D,T,table_plane)
                            raw['profile_recovery']=metadata
                            observation=p.localize_precise(None,frame,intrinsics,
                                extrinsics,table_plane,localizer=lambda *args:raw)
                            observation.update(processing_ms=p._elapsed(started),
                                profile_recovery=metadata,search={
                                    'roi_px':roi,'seed_count':1,'measured_seed_count':1,
                                    'candidate_limit':1,'usable_sides':0,
                                    'rejected':rejected,'top_fallback':False,
                                    'profile_recovery':True,'sampling_backend':
                                    'native_remap'if p._BATCH_EDGES else'portable'})
                            yield {'stage':'complete','observation':observation}
                            return
                        elif step['stage']=='front_back_complete':
                            refinement['rejections']=step['rejections']
                        else:
                            yield step
                poses = p.side_pose(p.pose_candidates(full_q, K, D, refine=True, refiner=p.refine_pose))
                if not poses:
                    raise ValueError('No accurate vertical-side pose')
                face = {'face_quad_px': full_q, 'poses': poses,
                        'physical_edges': [edge[1] for edge in edges],
                        'seam_support': separator['support'], 'seam_evidence': separator,
                        'area_px2': p.area(q), 'boundary_method': 'image_profile_four_measured_edges'}
                if selected_method=='ridge':
                    face.update(evidence_type='physical_face_with_G_ridge',
                        boundary_method='four_measured_edges_with_G_ridge')
                elif selected_method=='RGB':
                    face.update(evidence_type='physical_face_with_RGB_step')
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
                if metadata.get('top_method')=='ridge':
                    selected['vision_path']='shared_physical_face_G_ridge'
                elif metadata.get('top_method')=='RGB':
                    selected['vision_path']='shared_physical_face_RGB_step'
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
