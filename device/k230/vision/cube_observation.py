"""User-approved five-corner cube evidence; independent of face2px results."""
import math
try:
    from .cube_pose import solve_cube
    from ..pick.pose import project
    from ..pick.numeric import dot,cross,matvec,transpose,unit
    from ..pick.table_pose import (context,MAX_NORMAL_DEVIATION_DEG,
        EXPECTED_CENTER_HEIGHT_MM,HEIGHT_TOLERANCE_MM)
except ImportError:
    from cube_pose import solve_cube
    from pose import project
    from numeric import dot,cross,matvec,transpose,unit
    from table_pose import (context,MAX_NORMAL_DEVIATION_DEG,
        EXPECTED_CENTER_HEIGHT_MM,HEIGHT_TOLERANCE_MM)

EVIDENCE_TYPE='nonplanar_cube_five_corners'
RMS_LIMIT_PX=3.
OBJECT=[[-10.,0.,20.],[-10.,20.,20.],[10.,20.,20.],
        [10.,0.,0.],[-10.,0.,0.]]
CORNERS=[[-10.,0.,20.],[10.,0.,20.],[10.,0.,0.],[-10.,0.,0.],
         [-10.,20.,20.],[10.,20.,20.],[10.,20.,0.],[-10.,20.,0.]]


def _vector(value,length):
    if (not isinstance(value,(list,tuple)) or len(value)!=length or
            any(type(v) not in (int,float) or not math.isfinite(v) for v in value)):
        raise ValueError('Invalid five-corner numeric evidence')
    return list(value)


def validate_result(raw,K,D,T,plane):
    """Recompute the five measured-point residual and raw centre from the pose.

    A3px corner fit does not become a2px measured-face fit. No arm, odometry,
    remembered target or grasp offset is used to manufacture image evidence.
    """
    if (not isinstance(raw,dict) or raw.get('status')!='complete_cube_candidate' or
            raw.get('evidence_type')!=EVIDENCE_TYPE):
        raise ValueError('Unknown cube evidence policy')
    pose=raw.get('cube_pose')
    if not isinstance(pose,dict) or pose.get('edge_length_mm')!=20:
        raise ValueError('Five-corner evidence requires the20mm cube model')
    objects=pose.get('object_points_mm');pixels=pose.get('image_points_px')
    if (not isinstance(objects,(list,tuple)) or len(objects)!=5 or
            not isinstance(pixels,(list,tuple)) or len(pixels)!=5):
        raise ValueError('Five measured corner correspondences required')
    objects=[_vector(v,3) for v in objects];pixels=[_vector(v,2) for v in pixels]
    if not any(objects==[[sign*v[0],v[1],v[2]] for v in OBJECT] for sign in (1,-1)):
        raise ValueError('Unknown five-corner cube correspondence layout')
    if any(not(3<v[0]<1277 and 3<v[1]<957) for v in pixels):
        raise ValueError('Five-corner cube is clipped')
    rotation=pose.get('R_Object_to_Camera')
    if not isinstance(rotation,(list,tuple)) or len(rotation)!=3:
        raise ValueError('Invalid cube rotation')
    R=[_vector(row,3) for row in rotation];t=_vector(pose.get('t_Object_to_Camera_mm'),3)
    axes=transpose(R)
    if (any(abs(dot(a,b)-(1. if i==j else 0.))>1e-3
            for i,a in enumerate(axes) for j,b in enumerate(axes)) or
            dot(cross(axes[0],axes[1]),axes[2])<.999):
        raise ValueError('Cube pose is not a proper rigid rotation')
    if _vector(pose.get('object_center_mm'),3)!=[0.,10.,10.]:
        raise ValueError('Cube centre reference differs from the20mm model')
    edges=raw.get('observed_edges');halves=raw.get('half_corner_errors_px')
    if not isinstance(edges,(list,tuple)) or len(edges)!=6:
        raise ValueError('Six observed source edges required for five corners')
    for edge in edges:
        quality=edge.get('quality',{}) if isinstance(edge,dict) else {}
        rms=quality.get('rms_px');support=quality.get('support')
        if (quality.get('sampled_stations')!=64 or type(support) is not int or support<45 or
                type(rms) not in (int,float) or not math.isfinite(rms) or not 0<=rms<=3):
            raise ValueError('Cube source edge lacks measured distributed support')
    if (not isinstance(halves,(list,tuple)) or len(halves) not in (5,10) or
            any(type(v) not in (int,float) or not math.isfinite(v) or not 0<=v<=3 for v in halves)):
        raise ValueError('Five-corner interleaved measurements disagree')
    def transform(obj):
        r=matvec(R,obj)
        return [r[i]+t[i] for i in range(3)]
    predicted=[project(transform(obj),K,D) for obj in objects]
    rms=math.sqrt(sum(sum((a-b)**2 for a,b in zip(p,q))
                      for p,q in zip(predicted,pixels))/5)
    if not math.isfinite(rms) or rms>RMS_LIMIT_PX:
        raise ValueError('Five-corner cube exceeds the approved3px reprojection limit')
    center=transform([0.,10.,10.])
    base=[sum(T[i][j]*center[j] for j in range(3))+T[i][3] for i in range(3)]
    table=context(T,plane)
    if table is None:raise ValueError('Five-corner recovery requires the calibrated table')
    front=axes[1];front_base=matvec([row[:3] for row in T[:3]],front)
    angle=math.degrees(math.asin(min(1.,abs(dot(unit(front_base),table['normal'])))))
    height=dot(base,table['normal'])-table['offset_mm']
    if (angle>MAX_NORMAL_DEVIATION_DEG or
            abs(height-EXPECTED_CENTER_HEIGHT_MM)>HEIGHT_TOLERANCE_MM):
        raise ValueError('Five-corner cube is inconsistent with the calibrated table')
    geometry={'face_kind':'cube','normal_deviation_deg':angle,
        'center_height_mm':height,'height_error_mm':height-EXPECTED_CENTER_HEIGHT_MM,
        'reprojection_limit_px':RMS_LIMIT_PX,'accepted':True}
    normalized=dict(pose,corner_reprojection_rms_px=rms,cube_center_Camera_mm=center,
        cube_center_pixel=project(center,K,D),cube_projected_corners_px=[project(transform(v),K,D) for v in CORNERS],
        cube_projection_source='pose_transform',table_geometry=geometry)
    return dict(raw,cube_pose=normalized,cube_center_Camera_mm=center,
        cube_center_Base_mm=base,table_geometry=geometry),rms


def build_result(measurement,K,D,T,plane):
    """One free6DoF solve for independently measured current-image corners."""
    table=context(T,plane)
    if table is None:raise ValueError('Five-corner recovery requires the calibrated table')
    up=matvec(transpose([row[:3] for row in T[:3]]),table['normal'])
    result=solve_cube(measurement['object_points_mm'],measurement['image_points_px'],K,D,up)
    pose={'R_Object_to_Camera':result['R'],'t_Object_to_Camera_mm':result['t'],
        'object_points_mm':measurement['object_points_mm'],
        'image_points_px':measurement['image_points_px'],
        'object_center_mm':[0.,10.,10.],'edge_length_mm':20,
        'corner_reprojection_rms_px':result['reprojection_RMS_px']}
    raw={'status':'complete_cube_candidate','evidence_type':EVIDENCE_TYPE,
        'cube_pose':pose,'observed_edges':measurement['observed_edges'],
        'half_corner_errors_px':measurement['half_corner_errors_px'],
        'corner_labels':measurement.get('corner_labels'),
        'topology':measurement.get('topology'),
        'boundary_method':'six_measured_edges_five_corners',
        'vision_path':'shared_nonplanar_cube_corners'}
    return validate_result(raw,K,D,T,plane)[0]
