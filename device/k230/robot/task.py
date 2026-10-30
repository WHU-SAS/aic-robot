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


def handoff_observation(parking):
    verdict = parking.get('verification', {})
    observation = parking.get('verification_observation')
    checked = for_approach(observation, 'precise')
    point = checked.get('cube_center_Base_mm', [])
    payload = checked.get('payload')
    if (checked.get('status') != 'localized' or len(point) != 3 or
            not all(isinstance(v, (int, float)) and math.isfinite(v) for v in point) or
            checked.get('capture_timestamp_ms') is None or
            not isinstance(payload, dict) or payload.get('status') != 'complete_face_candidate' or
            not payload.get('poses') or payload.get('cube_center_Base_mm') != point or
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
    result.update(status='complete', physical_grasp_confirmed=False)
    emit('complete', {'physical_grasp_confirmed':False})
    return result
