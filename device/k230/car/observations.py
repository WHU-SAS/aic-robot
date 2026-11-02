"""Consume the shared observation contract without owning camera or task policy."""


def for_approach(observation, mode, clock_domain='k230_ticks_ms'):
    """Normalize shared results; never renew their capture time or use payload XYZ.

    The caller's now_ms must come from clock_domain, within the same board boot.
    A clock-domain string cannot detect a board reboot: reset consumers then.
    """
    if not isinstance(observation, dict) or observation.get('schema') != 'aic.vision_observation.v1':
        raise ValueError('Unsupported shared vision result')
    if observation.get('mode') != mode or mode not in ('coarse', 'precise'):
        raise ValueError('Wrong vision capability for this task')
    if observation.get('image_size_px') != [1280, 960] or observation.get('frame_id') is None:
        raise ValueError('Unexpected frame identity or dimensions')
    stamp = observation.get('capture_timestamp_ms')
    if stamp is not None and (type(stamp) is not int or stamp < 0):
        raise ValueError('Invalid capture timestamp')
    if observation.get('timestamp_ms', stamp) != stamp:
        raise ValueError('Capture timestamp alias mismatch')
    if stamp is not None and observation.get('clock_domain') != clock_domain:
        raise ValueError('Observation and controller clocks differ')
    result = dict(observation)
    result['timestamp_ms'] = stamp
    status = result.get('status')
    if mode == 'coarse':
        if 'cube_center_Base_mm' in result:
            raise ValueError('Coarse result cannot supply a precise cube center')
        if status not in ('tracking', 'confirming', 'lost', 'no_target'):
            raise ValueError('Unknown coarse status')
        evidence = result.get('visual_evidence')
        if evidence is not None:
            if (not isinstance(evidence, dict) or evidence.get('timestamp_ms') != stamp or
                    'cube_center_Base_mm' in evidence):
                raise ValueError('Visual evidence must describe the same coarse frame')
            evidence = dict(evidence)
            if evidence.get('distance_valid') is not True:
                evidence.pop('ground_contact_proxy_Base_mm', None)
            result['visual_evidence'] = evidence
        if status in ('lost', 'no_target'):
            for key in ('bbox_px', 'center_pixel', 'image_midline_error_px',
                        'pixels', 'confirmations', 'ground_contact_proxy_Base_mm'):
                result.pop(key, None)
    elif status == 'localized':
        if (result.get('coordinate_frame') != 'Base' or result.get('units') != 'mm'
                or result.get('reference_point') != 'cube_geometric_center'
                or result.get('xy_compensation_applied') is not False):
            raise ValueError('Parking requires raw Base geometric-center millimetres')
    else:
        # Keep failure status, without allowing old payload coordinates to act
        # as a new localization. Planner independently requires localized.
        result.pop('cube_center_Base_mm', None)
    return result
