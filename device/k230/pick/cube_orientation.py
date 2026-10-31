"""Tabletop cube axes from the physical sides used by precise localization."""
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
