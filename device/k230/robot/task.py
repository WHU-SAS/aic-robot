"""One car approach followed by the existing arm task, using its final frame.

Callbacks own hardware resources. Approach must close camera and motor UART
before returning. Completion means the motion stages were dispatched according
to their existing feedback rules; it is not an observation of a successful grip.
"""
import math
try:
    from observations import for_approach
except ImportError:
    from device.k230.car.observations import for_approach


def _cube_payload_valid(observation, payload, quality):
    """Consume the shared runtime's explicit cube capability, never a face.

    Reprojection/table verification belongs to that runtime. Here the declared
    capability, finite pose and reported quality must agree before the exact
    accepted parking frame can cross the car-to-arm boundary.
    """
    evidence = 'nonplanar_cube_five_corners'
    pose = payload.get('cube_pose')
    rms = quality.get('reprojection_rms_px')
    def vector(value, count):
        return (isinstance(value, (list, tuple)) and len(value) == count and
                all(type(v) in (int, float) and math.isfinite(v) for v in value))
    def matrix(value, rows, columns):
        return (isinstance(value, (list, tuple)) and len(value) == rows and
                all(vector(row, columns) for row in value))
    if (payload.get('status') != 'complete_cube_candidate' or
            payload.get('evidence_type') != evidence or
            quality.get('evidence_type') != evidence or not isinstance(pose, dict) or
            type(rms) not in (int, float) or not math.isfinite(rms) or not 0 <= rms <= 3 or
            type(quality.get('reprojection_limit_px')) not in (int, float) or
            quality['reprojection_limit_px'] != 3 or
            quality.get('boundary_method') != 'six_measured_edges_five_corners' or
            quality.get('vision_path') != 'shared_nonplanar_cube_corners'):
        return False
    corner_rms = pose.get('corner_reprojection_rms_px')
    R = pose.get('R_Object_to_Camera')
    if (type(corner_rms) not in (int, float) or not math.isfinite(corner_rms) or
            not 0 <= corner_rms <= 3 or abs(corner_rms-rms) > 1e-6 or
            not matrix(R, 3, 3) or not vector(pose.get('t_Object_to_Camera_mm'), 3) or
            not vector(pose.get('object_center_mm'), 3) or
            list(pose['object_center_mm']) != [0, 10, 10] or
            type(pose.get('edge_length_mm')) not in (int, float) or pose['edge_length_mm'] != 20 or
            not matrix(pose.get('object_points_mm'), 5, 3) or
            not matrix(pose.get('image_points_px'), 5, 2) or
            not vector(pose.get('cube_center_Camera_mm'), 3) or
            not matrix(pose.get('cube_projected_corners_px'), 8, 2) or
            pose.get('cube_projection_source') != 'pose_transform' or
            not vector(observation.get('cube_center_Base_mm'), 3) or
            any(key in payload or key in pose for key in
                ('face_reprojection_rms_px', 'face_quad_px', 'physical_edges', 'side_faces', 'poses'))):
        return False
    columns = list(zip(*R))
    determinant = (R[0][0]*(R[1][1]*R[2][2]-R[1][2]*R[2][1]) -
                   R[0][1]*(R[1][0]*R[2][2]-R[1][2]*R[2][0]) +
                   R[0][2]*(R[1][0]*R[2][1]-R[1][1]*R[2][0]))
    return (abs(determinant-1) <= 1e-3 and all(
        abs(sum(a*b for a, b in zip(columns[i], columns[j]))-(1 if i == j else 0)) <= 1e-3
        for i in range(3) for j in range(3)))


def handoff_observation(parking):
    verdict = parking.get('verification', {})
    observation = parking.get('verification_observation')
    checked = for_approach(observation, 'precise')
    point = checked.get('cube_center_Base_mm', [])
    payload = checked.get('payload')
    quality = checked.get('quality')
    quality = quality if isinstance(quality, dict) else {}
    payload_valid = False
    if isinstance(payload, dict):
        evidence = 'nonplanar_cube_five_corners'
        cube_marked = (payload.get('status') == 'complete_cube_candidate' or
            'cube_pose' in payload or payload.get('evidence_type') == evidence or
            quality.get('evidence_type') == evidence)
        payload_valid = (_cube_payload_valid(checked, payload, quality) if cube_marked else
            payload.get('status') == 'complete_face_candidate' and bool(payload.get('poses')))
    if (checked.get('status') != 'localized' or len(point) != 3 or
            not all(isinstance(v, (int, float)) and math.isfinite(v) for v in point) or
            checked.get('capture_timestamp_ms') is None or
            not payload_valid or payload.get('cube_center_Base_mm') != point or
            verdict.get('status') != 'parked' or
            verdict.get('frame_id') != checked['frame_id'] or
            verdict.get('capture_timestamp_ms') != checked['capture_timestamp_ms'] or
            verdict.get('cube_center_Base_mm') != point):
        raise ValueError('Arm handoff requires the accepted final static precise frame')
    return observation


def execute_task(approach, grasp, emit=None):
    emit = emit or (lambda event, details=None: None)
    emit('approach_started', None)
    parking = approach()
    emit('approach_finished', {'status':parking.get('status')})
    result = {'schema':'aic.robot.task.v1', 'status':'parking_failed',
              'parking':parking, 'arm_started':False}
    if parking.get('status') != 'parked' or parking.get('verification', {}).get('accepted') is not True:
        emit('parking_failed', {'status':parking.get('status')})
        return result
    try:
        observation = handoff_observation(parking)
    except (ValueError, TypeError) as exc:
        result.update(status='invalid_handoff', reason=str(exc))
        emit('invalid_handoff', {'reason':str(exc)})
        return result
    result['arm_started'] = True
    emit('grasp_started', {'frame_id':observation['frame_id'],
                          'capture_timestamp_ms':observation['capture_timestamp_ms']})
    result['arm_plan'] = grasp(observation)
    plan=result['arm_plan']
    confirmation=plan.get('grip_confirmation') if isinstance(plan,dict) else None
    confirmed=plan.get('grasp_confirmed') if isinstance(plan,dict) else None
    # Complete means the four stages finished. Encoder classification is separate
    # from any independent observation that the object was actually lifted.
    result.update(status='complete', physical_grasp_confirmed=False,
                  grasp_confirmed=confirmed,grip_confirmation=confirmation)
    emit('complete', {'physical_grasp_confirmed':False,'grasp_confirmed':confirmed,
                      'grip_confirmation':confirmation})
    return result
