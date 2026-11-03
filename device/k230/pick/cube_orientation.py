"""Tabletop cube axes from verified side faces or nonplanar cube corners."""
import math
try:
    from .numeric import dot,cross,norm,unit,matvec
except ImportError:
    from numeric import dot,cross,norm,unit,matvec


def _vector(value):
    if (not isinstance(value,(list,tuple)) or len(value)!=3 or
            any(type(v) not in (int,float) or not math.isfinite(v) for v in value)):
        raise ValueError('Invalid orientation vector')
    return unit(value)


def plane_basis(plane):
    n=_vector(plane['normal'])
    if n[2]<0:n=[-v for v in n]
    x=unit([a-n[0]*b for a,b in zip([1.,0.,0.],n)])
    return x,cross(n,x),n


_CORNER_EVIDENCE='nonplanar_cube_five_corners'
_CORNER_BOUNDARY='six_measured_edges_five_corners'
_CORNER_PATH='shared_nonplanar_cube_corners'
_OUTLINE_EVIDENCE='nonplanar_cube_six_corners'
_OUTLINE_BOUNDARY='six_measured_outline_edges_six_corners'
_OUTLINE_PATH='shared_nonplanar_cube_outline'
_CORNER_CONTRACTS={
    _CORNER_EVIDENCE:(5,_CORNER_BOUNDARY,_CORNER_PATH,
        'verified_nonplanar_cube_axes_fourfold_mean'),
    _OUTLINE_EVIDENCE:(6,_OUTLINE_BOUNDARY,_OUTLINE_PATH,
        'verified_nonplanar_cube_outline_axes_fourfold_mean'),
    'nonplanar_cube_face_back_top':(6,'seven_measured_edges_front_and_back_top_corners',
        'shared_nonplanar_cube_face_back_top',
        'verified_nonplanar_cube_face_back_top_axes_fourfold_mean')}
_CORNER_BOUNDARIES=tuple(value[1] for value in _CORNER_CONTRACTS.values())
_CORNER_PATHS=tuple(value[2] for value in _CORNER_CONTRACTS.values())


def _finite(value):
    return type(value) in (int,float) and math.isfinite(value)


def _array(value,rows,columns):
    return (isinstance(value,(list,tuple)) and len(value)==rows and
        all(isinstance(row,(list,tuple)) and len(row)==columns and
            all(_finite(v) for v in row) for row in value))


def _point(value,size):
    return (isinstance(value,(list,tuple)) and len(value)==size and
        all(_finite(v) for v in value))


def _corner_marked(found):
    if not isinstance(found,dict):return False
    observation=found.get('observation')
    quality=observation.get('quality') if isinstance(observation,dict) else None
    return (found.get('status')=='complete_cube_candidate' or 'cube_pose' in found or
        any(isinstance(record,dict) and (
            record.get('evidence_type') in _CORNER_CONTRACTS or
            record.get('boundary_method') in _CORNER_BOUNDARIES or
            record.get('vision_path') in _CORNER_PATHS)
            for record in (found,quality)))


def _corner_axes(found,R,n):
    """Validate the distinct corner evidence before deriving its two axes.

    Runtime owns the complete projection/centre/table validation. This wrist
    consumer checks the evidence contract and a proper serialized rotation;
    invalid marked corner evidence cannot borrow a legacy side pose.
    """
    observation=found.get('observation')
    quality=observation.get('quality') if isinstance(observation,dict) else None
    pose=found.get('cube_pose')
    contract=_CORNER_CONTRACTS.get(found.get('evidence_type'))
    if contract is None:raise ValueError('Unknown nonplanar cube evidence')
    count,boundary,path,method=contract
    if (found.get('status')!='complete_cube_candidate' or
            not isinstance(pose,dict) or not isinstance(quality,dict) or
            observation.get('status')!='localized' or
            quality.get('evidence_type')!=found['evidence_type'] or
            quality.get('boundary_method')!=boundary or
            quality.get('vision_path')!=path):
        raise ValueError('Invalid nonplanar cube evidence')
    # Old five-corner callers allowed these raw labels to be absent. If
    # supplied they must agree; the new six-corner contracts require both.
    for key,expected in (('boundary_method',boundary),('vision_path',path)):
        if found.get(key,expected if count==5 else None)!=expected:
            raise ValueError('Nonplanar cube raw marker differs from its quality')
    for record in (found,pose,quality):
        if 'corner_count' in record and (type(record['corner_count']) is not int or
                record['corner_count']!=count):
            raise ValueError('Nonplanar cube corner count differs from its evidence')
    rms=pose.get('corner_reprojection_rms_px')
    reported=quality.get('reprojection_rms_px');limit=quality.get('reprojection_limit_px')
    if (not _finite(rms) or not 0<=rms<=3 or not _finite(reported) or
            not 0<=reported<=3 or abs(rms-reported)>1e-6 or
            not _finite(limit) or abs(limit-3)>1e-6):
        raise ValueError('Invalid nonplanar cube quality')
    rotation=pose.get('R_Object_to_Camera')
    if (not _array(rotation,3,3) or
            not _point(pose.get('t_Object_to_Camera_mm'),3) or
            not _point(pose.get('object_center_mm'),3) or
            not _point(pose.get('cube_center_Camera_mm'),3) or
            not _point(found.get('cube_center_Base_mm'),3) or
            not _array(pose.get('object_points_mm'),count,3) or
            not _array(pose.get('image_points_px'),count,2) or
            not _array(pose.get('cube_projected_corners_px'),8,2) or
            pose.get('cube_projection_source')!='pose_transform'):
        raise ValueError('Invalid nonplanar cube geometry shape')
    if 'cube_center_Base_mm' in observation:
        center=observation['cube_center_Base_mm']
        if (not _point(center,3) or any(abs(a-b)>1e-6
                for a,b in zip(center,found['cube_center_Base_mm']))):
            raise ValueError('Nonplanar cube observation centre differs from its payload')
    edge=pose.get('edge_length_mm')
    if (not _finite(edge) or abs(edge-20)>1e-6 or
            any(abs(a-b)>1e-6 for a,b in zip(pose['object_center_mm'],[0.,10.,10.]))):
        raise ValueError('Invalid nonplanar cube reference')
    # Board float32 and seven-significant-digit JSON can round the matrix.
    # A reflection or a materially scaled/sheared basis is not a cube pose.
    if (any(abs(dot(rotation[i],rotation[j])-(1. if i==j else 0.))>1e-3
            for i in range(3) for j in range(3)) or
            abs(dot(rotation[0],cross(rotation[1],rotation[2]))-1)>1e-3):
        raise ValueError('Nonplanar cube rotation must be proper')
    # Runtime owns the accepted cube/table geometry. Project the object X/Y
    # axes without adding a second tilt or object-Z acceptance condition here.
    axes=[]
    for j in (0,1):
        v=_vector(matvec(R,[row[j] for row in rotation]))
        v=[a-dot(v,n)*b for a,b in zip(v,n)]
        if norm(v)<1e-8:raise ValueError('Degenerate projected cube axis')
        axes.append(_vector(v))
    return axes


def _corner_estimate(found,R,x,y,n):
    samples=[math.atan2(dot(v,y),dot(v,x)) for v in _corner_axes(found,R,n)]
    c=sum(math.cos(4*a) for a in samples);s=sum(math.sin(4*a) for a in samples)
    if math.sqrt(c*c+s*s)<1e-8:
        raise ValueError('Ambiguous nonplanar cube orientation')
    angle=math.atan2(s,c)/4
    axis=[math.cos(angle)*u+math.sin(angle)*v for u,v in zip(x,y)]
    return {'status':'estimated','method':_CORNER_CONTRACTS[found['evidence_type']][3],
        'yaw_table_deg':math.degrees(angle),'period_deg':90,'axis_count':len(samples),
        'axes_Base':[axis,cross(n,axis)],'table_normal_Base':n,
        'axis_residual_deg':[abs((v-angle+math.pi/4)%(math.pi/2)-math.pi/4)*180/math.pi for v in samples]}


def estimate(found,T,plane):
    """Use selected-side yaw modulo90deg, or explicitly keep the current wrist.

    Two selected sides receive equal fourth-angle votes. An unused smaller
    face, a top face, and an unselected planar-pose branch cannot steer a grasp.
    """
    unavailable={'status':'unavailable','reason':'No verified side orientation; keep current wrist'}
    try:
        x,y,n=plane_basis(plane)
        if not isinstance(T,(list,tuple)) or len(T)<3:return unavailable
        R=[]
        for row in T[:3]:
            if (not isinstance(row,(list,tuple)) or len(row)<3 or
                    any(type(v) not in (int,float) or not math.isfinite(v) for v in row[:3])):
                return unavailable
            R.append(list(row[:3]))
        if _corner_marked(found):return _corner_estimate(found,R,x,y,n)
        faces=found.get('side_faces',[]) if isinstance(found,dict) else []
        if not isinstance(faces,(list,tuple)):return unavailable
        faces=[f for f in faces if isinstance(f,dict)]
        selected=[f for f in faces if f.get('used_for_center') is True]
        if not selected:selected=[f for f in faces if 'used_for_center' not in f]
        samples=[]
        for face in selected:
            pose=face.get('selected_pose')
            if not isinstance(pose,dict):continue
            rms=pose.get('face_reprojection_rms_px')
            if type(rms) not in (int,float) or not math.isfinite(rms) or not 0<=rms<=2:continue
            try:v=unit(matvec(R,_vector(pose.get('inward_face_normal_Camera'))))
            except (ValueError,TypeError):continue
            # Same20deg side-vs-table criterion as the precise table validator.
            if abs(dot(v,n))>math.sin(math.radians(20)):continue
            v=[a-dot(v,n)*b for a,b in zip(v,n)]
            if norm(v)<1e-8:continue
            samples.append(math.atan2(dot(v,y),dot(v,x)))
        if not samples:return unavailable
        c=sum(math.cos(4*a) for a in samples);s=sum(math.sin(4*a) for a in samples)
        if math.sqrt(c*c+s*s)<1e-8:return unavailable
        angle=math.atan2(s,c)/4
        a=[math.cos(angle)*u+math.sin(angle)*v for u,v in zip(x,y)]
        return {'status':'estimated','method':'verified_selected_side_normals_fourfold_mean',
            'yaw_table_deg':math.degrees(angle),'period_deg':90,'side_count':len(samples),
            'axes_Base':[a,cross(n,a)],'table_normal_Base':n,
            'side_residual_deg':[abs((v-angle+math.pi/4)%(math.pi/2)-math.pi/4)*180/math.pi for v in samples]}
    except (KeyError,ValueError,TypeError,ZeroDivisionError):
        return unavailable
