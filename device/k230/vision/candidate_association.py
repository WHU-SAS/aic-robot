"""Same-frame 2D association of an existing fine solve, without solving poses.

The cube footprint is projected by the selected pose, not inferred from a face
quad or an audit candidate. The pixel tolerance is an association heuristic;
it is not an RMS bound or evidence of 3D accuracy or physical target identity.
"""


def _number(value):
    return type(value) in (int, float) and abs(value) < float('inf')


def _point(value):
    return (isinstance(value, (list, tuple)) and len(value) == 2 and
            all(_number(v) for v in value))


def _bbox(value):
    return (isinstance(value, (list, tuple)) and len(value) == 4 and
            all(_number(v) for v in value) and min(value[2:]) > 0)


def _cross(a, b, c):
    return (b[0]-a[0])*(c[1]-a[1])-(b[1]-a[1])*(c[0]-a[0])


def _hull(points):
    points = sorted(set((float(p[0]), float(p[1])) for p in points))
    halves = []
    for sequence in (points, points[::-1]):
        half = []
        for point in sequence:
            while len(half) >= 2 and _cross(half[-2], half[-1], point) <= 0:
                half.pop()
            half.append(point)
        halves.append(half[:-1])
    return [list(p) for p in halves[0]+halves[1]]


def _signed_distance(point, hull):
    inside, distances = True, []
    for a, b in zip(hull, hull[1:]+hull[:1]):
        dx, dy = b[0]-a[0], b[1]-a[1]
        along = max(0., min(1., ((point[0]-a[0])*dx+(point[1]-a[1])*dy)/(dx*dx+dy*dy)))
        distances.append(((point[0]-a[0]-along*dx)**2+
                          (point[1]-a[1]-along*dy)**2)**.5)
        if _cross(a, b, point) < 0:
            inside = False
    distance = min(distances)
    return distance if inside else -distance


def _same_candidate(a, b):
    return (list(a['bbox_px']) == list(b['bbox_px']) and
            list(a['center_pixel']) == list(b['center_pixel']))


def _interior_patch(candidate, source):
    x, y, w, h = source['bbox_px']
    a, b, c, d = candidate['bbox_px']
    return x < a and y < b and a+c < x+w and b+d < y+h


def _used_poses(observation):
    if observation.get('status') == 'localized':
        raw = observation.get('payload')
        if not isinstance(raw, dict):
            return 'full_pose', []
        if (raw.get('status')=='complete_cube_candidate' and
                raw.get('evidence_type')=='nonplanar_cube_five_corners'):
            pose=raw.get('cube_pose')
            return 'full_cube_corner_pose', [(0,pose)] if isinstance(pose,dict) else []
        if 'side_faces' in raw:
            faces = raw['side_faces']
            if not isinstance(faces, (list, tuple)):
                return 'full_pose', []
            return 'full_pose', [(index, face.get('selected_pose'))
                for index, face in enumerate(faces)
                if isinstance(face, dict) and face.get('used_for_center') is True]
        poses = raw.get('poses')
        return 'full_pose', [(0, poses[0])] if isinstance(poses, (list, tuple)) and poses else []
    candidate = observation.get('range_candidate')
    if isinstance(candidate, dict) and candidate.get('status') == 'range_candidate':
        faces = candidate.get('selected_faces')
        if isinstance(faces, (list, tuple)):
            return 'moving_range', [(index, face.get('selected_pose'))
                for index, face in enumerate(faces)
                if isinstance(face, dict) and face.get('used_for_center') is True]
        return 'moving_range', []
    return None, []


def associate_fine_candidate(fine_observation, paired_coarse, pixel_tolerance_px=2.0):
    """Require source centroid support from EVERY pose actually used for fine.

    Native current-frame cues disclose competing blue candidates. Navigation
    confirmation and coarse distance validity are not association requirements;
    this result does not grant either of those capabilities to a coarse cue.
    Inputs and their raw pose/audit records are never modified.
    """
    fine = fine_observation if isinstance(fine_observation, dict) else {}
    coarse = paired_coarse if isinstance(paired_coarse, dict) else {}
    result = {'status': 'unknown', 'reason': 'invalid_observation',
        'frame_id': fine.get('frame_id'),
        'capture_timestamp_ms': fine.get('capture_timestamp_ms'),
        'source': 'paired_coarse.visual_evidence centroid in selected 20mm cube pose hulls',
        'geometry_capability': None, 'source_candidate': None, 'used_pose_hulls': [],
        'pixel_tolerance_px': pixel_tolerance_px,
        'tolerance_basis': '2D association heuristic; not reprojection RMS or 3D accuracy'}

    def unknown(reason):
        result['reason'] = reason
        return result

    if not _number(pixel_tolerance_px) or pixel_tolerance_px < 0:
        return unknown('invalid_pixel_tolerance')
    if not fine or not coarse:
        return result
    frame, stamp = fine.get('frame_id'), fine.get('capture_timestamp_ms')
    if (frame is None or type(stamp) is not int or stamp < 0 or coarse.get('frame_id') != frame or
            coarse.get('capture_timestamp_ms') != stamp or
            fine.get('timestamp_ms', stamp) != stamp or
            coarse.get('timestamp_ms', stamp) != stamp or
            not coarse.get('clock_domain') or fine.get('clock_domain') != coarse.get('clock_domain') or
            coarse.get('image_size_px') != [1280, 960] or
            fine.get('image_size_px') != coarse.get('image_size_px')):
        return unknown('source_frame_mismatch')
    source = coarse.get('visual_evidence')
    if not isinstance(source, dict):
        return unknown('missing_source_visual_evidence')
    if (source.get('visible') is not True or source.get('associated') is not True or
            source.get('ambiguous') is not False):
        return unknown('source_not_unambiguously_associated')
    if source.get('timestamp_ms') != stamp or source.get('frame_id', frame) != frame:
        return unknown('source_evidence_frame_mismatch')
    if not _point(source.get('center_pixel')) or not _bbox(source.get('bbox_px')):
        return unknown('invalid_source_candidate')
    if any(key in coarse and coarse[key] != source[key] for key in ('bbox_px', 'center_pixel')):
        return unknown('coarse_candidate_fields_disagree')
    result['source_candidate'] = {'bbox_px': list(source['bbox_px']),
        'center_pixel': list(source['center_pixel']), 'timestamp_ms': stamp,
        'distance_valid': source.get('distance_valid'),
        'navigation_confirmed': source.get('navigation_confirmed')}
    count, pool = source.get('candidate_count'), coarse.get('visual_candidates')
    if (type(count) is not int or count < 1 or not isinstance(pool, (list, tuple)) or
            len(pool) != count):
        return unknown('incomplete_current_candidate_pool')
    for cue in pool:
        if (not isinstance(cue, dict) or not _bbox(cue.get('bbox_px')) or
                not _point(cue.get('center_pixel')) or cue.get('timestamp_ms') != stamp or
                cue.get('frame_id', frame) != frame):
            return unknown('incomplete_current_candidate_pool')
    if not any(_same_candidate(cue, source) for cue in pool):
        return unknown('source_absent_from_current_candidate_pool')
    capability, poses = _used_poses(fine)
    result['geometry_capability'] = capability
    if not poses:
        return unknown('missing_used_pose_geometry')
    for index, pose in poses:
        audit = {'face_index': index, 'hull_px': [], 'supports_source': False}
        result['used_pose_hulls'].append(audit)
        corners = pose.get('cube_projected_corners_px') if isinstance(pose, dict) else None
        if (not isinstance(pose, dict) or pose.get('cube_projection_source') != 'pose_transform' or
                not isinstance(corners, (list, tuple)) or len(corners) != 8 or
                not all(_point(p) for p in corners)):
            audit['reason'] = 'missing_selected_cube_projection'
            continue
        hull = _hull(corners)
        audit['hull_px'] = hull
        if len(hull) < 3:
            audit['reason'] = 'degenerate_selected_cube_projection'
            continue
        signed = _signed_distance(source['center_pixel'], hull)
        audit.update(signed_hull_distance_px=signed,
                     supports_source=signed >= -pixel_tolerance_px)
    support = sum(item['supports_source'] for item in result['used_pose_hulls'])
    if support != len(poses):
        return unknown('partial_used_pose_support' if support else 'no_used_pose_support')
    for index, cue in enumerate(pool):
        if _same_candidate(cue, source) or _interior_patch(cue, source):
            continue
        if any(_signed_distance(cue['center_pixel'], item['hull_px']) >= -pixel_tolerance_px
               for item in result['used_pose_hulls']):
            result['competing_candidate'] = {'pool_index': index,
                'bbox_px': list(cue['bbox_px']), 'center_pixel': list(cue['center_pixel'])}
            return unknown('competing_candidate_in_fine_projection')
    result.update(status='matched', reason='all_used_pose_hulls_support_unique_source')
    return result
