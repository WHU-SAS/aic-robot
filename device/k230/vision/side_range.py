"""Car distance from already measured sides; never complete table/arm authority.

The known object is the same 20mm cube. This does not measure its size or prove
table contact. No image acquisition, pose solve, history or encoder input lives
here. The caller retains the original frame and the strict localization status.
"""
try:
    from ..pick.table_pose import metrics, MAX_NORMAL_DEVIATION_DEG
    from ..pick.side_faces import select_sides, distance, CENTER_AGREEMENT_MM
except ImportError:
    from table_pose import metrics, MAX_NORMAL_DEVIATION_DEG
    from side_faces import select_sides, distance, CENTER_AGREEMENT_MM


def _base(center, T):
    return [sum(T[i][j]*center[j] for j in range(3))+T[i][3] for i in range(3)]


def _range_face(face, table):
    poses, audit = [], []
    for index, pose in enumerate(face['poses']):
        geometry = metrics(pose, table, 'side')
        allowed = (geometry['normal_deviation_deg'] <= MAX_NORMAL_DEVIATION_DEG
                   and pose['face_reprojection_rms_px'] <= 2.)
        audit.append({'original_pose_rank': index,
            'cube_center_Camera_mm': list(pose['cube_center_Camera_mm']),
            'reprojection_rms_px': pose['face_reprojection_rms_px'],
            'table_geometry': geometry, 'range_direction_accepted': allowed})
        if allowed:
            item = dict(pose)
            # side_faces' pair ordering must not inherit table-height scores.
            item.pop('table_geometry', None)
            item['range_geometry'] = geometry
            item['original_pose_rank'] = index
            poses.append(item)
    if not poses:
        return None
    poses.sort(key=lambda p: (p['range_geometry']['normal_deviation_deg'],
                             p['face_reprojection_rms_px']))
    result = dict(face)
    result.update(poses=poses, range_pose_candidates=audit)
    return result


def _possible_centers(selected):
    """Only pose pairs admitted by the existing same-cube side geometry."""
    faces = selected['side_faces']
    if len(faces) == 1:
        return [p['cube_center_Camera_mm'] for p in faces[0]['poses']]
    centers = []
    for a in faces[0]['poses']:
        for b in faces[1]['poses']:
            cosine = abs(sum(x*y for x,y in zip(a['inward_face_normal_Camera'],
                                               b['inward_face_normal_Camera'])))
            if (cosine <= .35 and distance(a['cube_center_Camera_mm'],
                    b['cube_center_Camera_mm']) <= CENTER_AGREEMENT_MM):
                if selected['selected_side_count'] == 2:
                    centers.append([(x+y)/2 for x,y in zip(a['cube_center_Camera_mm'],
                                                           b['cube_center_Camera_mm'])])
                else:
                    centers.append(a['cube_center_Camera_mm'])
    return centers


def range_candidate(raw_faces, K, D, T, table):
    """Reuse free PnP candidates after a strict full-pose failure.

    Four-edge and seam evidence belongs to raw_faces supplied by the shared
    detector. Upright direction, 2px, adjacency, 8mm pair agreement and1.25 area
    rules remain. Height is reported, not used to choose the distance pose.
    X_interval_mm bounds the surviving pose solutions, not all calibration,
    target-size or boundary errors; it is not an accuracy confidence interval.
    """
    faces = [_range_face(face, table) for face in raw_faces]
    faces = [face for face in faces if face is not None]
    selected = select_sides(faces, K, D, None)
    if selected is None:
        return None
    point = _base(selected['poses'][0]['cube_center_Camera_mm'], T)
    xs = [_base(c, T)[0] for c in _possible_centers(selected)]
    if not xs:
        return None
    xs.append(point[0])
    return {'schema': 'aic.car.side_range.v1', 'status': 'range_candidate',
        'coordinate_frame': 'Base', 'units': 'mm', 'Base_X_mm': point[0],
        'raw_center_Base_mm': point, 'X_interval_mm': [min(xs), max(xs)],
        'not_arm_or_parking_authority': True, 'height_used_for_selection': False,
        'side_selection': selected['side_selection'],
        # Preserve only the original faces/poses that actually supplied this
        # centre; audit below deliberately retains every measured candidate.
        'selected_faces': [{'face_quad_px': face['face_quad_px'],
                            'selected_pose': face['selected_pose'],
                            'used_for_center': True}
                           for face in selected['side_faces'] if face['used_for_center']],
        'audit': [{'face_quad_px': face['face_quad_px'],
                   'physical_edges': face['physical_edges'],
                   'area_px2': face['area_px2'],
                   'poses': face['range_pose_candidates']} for face in faces]}
