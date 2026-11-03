"""Shared coarse/precise observation interface; no camera, voice or motion startup.

Callers own task policy. The native RGB565 frame is borrowed until the next
snapshot/close. Precise BGR input must keep the existing JPEG95 decode path.
"""
import math
import time


def _now():
    ticks=getattr(time,'ticks_ms',None)
    return ticks() if ticks else int(time.monotonic()*1000)


def _elapsed(start):
    difference=getattr(time,'ticks_diff',None)
    return difference(_now(),start) if difference else _now()-start


def _envelope(mode,frame):
    if not isinstance(frame,dict) or frame.get('frame_id') is None:
        raise ValueError('Frame identity is required')
    size=frame.get('image_size_px')
    if not isinstance(size,(list,tuple)) or len(size)!=2 or any(type(v) is not int or v<=0 for v in size):
        raise ValueError('Invalid frame dimensions')
    stamp=frame.get('capture_timestamp_ms')
    if stamp is not None and (type(stamp) is not int or stamp<0):raise ValueError('Invalid capture timestamp')
    clock=frame.get('clock_domain')
    if stamp is not None and not clock:raise ValueError('Capture clock domain is required')
    return {'schema':'aic.vision_observation.v1','mode':mode,'frame_id':frame['frame_id'],'capture_timestamp_ms':stamp,
        'timestamp_ms':stamp,'clock_domain':clock,'image_size_px':list(size),
        'timestamp_source':frame.get('timestamp_source','unavailable'),
        'status':'failed','motion_performed':False}


def localize_precise(image,frame,intrinsics,extrinsics,table_plane=None,localizer=None):
    """Shared measured-face or explicitly typed measured-corner result; raw Base centre.

    Missing capture time remains None (e.g. saved-image replay), never replaced
    with processing time. payload retains its own face2px or cube3px evidence.
    """
    result=_envelope('precise',frame)
    if result['image_size_px']!=[1280,960]:raise ValueError('Precise mode requires calibrated 1280x960 input')
    K=[list(row) for row in intrinsics['camera_matrix']]
    for j in range(3):
        K[0][j]*=1280/intrinsics['image_width'];K[1][j]*=960/intrinsics['image_height']
    if localizer is None:
        try:from ..pick.vision import localize as localizer
        except ImportError:from vision import localize as localizer
    started=_now()
    try:
        raw=localizer(image,K,intrinsics['distortion_coefficients'],extrinsics['T_Base_from_Camera_mm'],table_plane)
        if raw.get('status')=='complete_cube_candidate':
            try:from .cube_observation import validate_result,RMS_LIMIT_PX
            except ImportError:from cube_observation import validate_result,RMS_LIMIT_PX
            raw,rms=validate_result(raw,K,intrinsics['distortion_coefficients'],
                                    extrinsics['T_Base_from_Camera_mm'],table_plane)
            center=raw['cube_center_Base_mm']
            quality={'evidence_type':raw['evidence_type'],'reprojection_rms_px':rms,
                'reprojection_limit_px':RMS_LIMIT_PX,'vision_path':raw.get('vision_path'),
                'boundary_method':raw.get('boundary_method')}
        else:
            center=raw.get('cube_center_Base_mm');poses=raw.get('poses')
            if (raw.get('status')!='complete_face_candidate' or not poses or
                not isinstance(center,(list,tuple)) or len(center)!=3 or
                not all(isinstance(v,(int,float)) and math.isfinite(v) for v in center)):
                raise ValueError(raw.get('reason','No valid precise cube center'))
            rms=poses[0].get('face_reprojection_rms_px')
            if not isinstance(rms,(int,float)) or not math.isfinite(rms) or not 0<=rms<=2:
                raise ValueError('Precise face does not meet existing 2px reprojection limit')
            quality={'reprojection_rms_px':rms,'side_selection':raw.get('side_selection'),
                'vision_path':raw.get('vision_path'),'boundary_method':raw.get('boundary_method')}
            if raw.get('evidence_type') in ('physical_face_with_G_ridge','physical_face_with_RGB_step'):
                quality.update(evidence_type=raw['evidence_type'],reprojection_limit_px=2,
                    separator_evidence=('G_ridge_bidirectional_prominence'
                        if raw['evidence_type']=='physical_face_with_G_ridge'
                        else 'original_RGB8_two_blue_flanks'))
        result.update(status='localized',cube_center_Base_mm=list(center),
            coordinate_frame='Base',units='mm',reference_point='cube_geometric_center',
            xy_compensation_applied=False,payload=raw,
            quality=quality)
    except ValueError as exc:
        result.update(status='no_complete_face',reason=str(exc))
    result['processing_ms']=_elapsed(started)
    return result


def track_coarse(image,frame,detector):
    """Call an injected native-frame detector once; never claim precise XYZ."""
    result=_envelope('coarse',frame)
    if result['capture_timestamp_ms'] is None:raise ValueError('Coarse tracking requires a capture timestamp')
    started=_now()
    raw=detector(image,result['capture_timestamp_ms'])
    if not isinstance(raw,dict):raise ValueError('Detector must return an observation dict')
    if 'cube_center_Base_mm' in raw:raise ValueError('A coarse observation cannot claim a precise cube center')
    if raw.get('timestamp_ms',result['capture_timestamp_ms'])!=result['capture_timestamp_ms']:
        raise ValueError('Detector timestamp differs from captured frame')
    evidence=raw.get('visual_evidence')
    if evidence is not None:
        if (not isinstance(evidence,dict) or
                evidence.get('timestamp_ms')!=result['capture_timestamp_ms'] or
                'cube_center_Base_mm' in evidence):
            raise ValueError('Visual evidence must describe the same coarse frame')
        evidence=dict(evidence)
        if evidence.get('distance_valid') is not True:
            evidence.pop('ground_contact_proxy_Base_mm',None)
        raw=dict(raw,visual_evidence=evidence)
    status=raw.get('status')
    if status not in ('tracking','confirming','lost','no_target'):raise ValueError('Unknown coarse observation status')
    raw=dict(raw)
    if status in ('lost','no_target'):
        for key in ('bbox_px','center_pixel','image_midline_error_px','pixels','confirmations','ground_contact_proxy_Base_mm'):
            raw.pop(key,None)
    result.update(status=status,payload=raw,processing_ms=_elapsed(started))
    for key in ('bbox_px','center_pixel','image_midline_error_px','pixels','confirmations','ground_contact_proxy_Base_mm','visual_evidence','visual_candidates'):
        if key in raw:result[key]=raw[key]
    result['quality']={key:raw[key] for key in ('pixels','confirmations') if key in raw}
    return result
