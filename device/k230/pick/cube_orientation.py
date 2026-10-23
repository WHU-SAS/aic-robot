"""Tabletop cube yaw from already verified side poses, modulo 90 degrees."""
import math
from numeric import dot,cross,norm,unit,matvec


def plane_basis(plane):
    n=unit(plane['normal'])
    if n[2]<0:n=[-x for x in n]
    x=unit([a-n[0]*b for a,b in zip([1.,0.,0.],n)])
    return x,cross(n,x),n


def estimate(found,T,plane):
    x,y,n=plane_basis(plane);samples=[]
    # selected_pose is the pair-consistent solution, not the first PnP branch.
    for face in found.get('side_faces',[]):
        p=face.get('selected_pose')
        if p is None:continue
        v=matvec([row[:3] for row in T[:3]],p['inward_face_normal_Camera'])
        v=[a-dot(v,n)*b for a,b in zip(v,n)]
        if norm(v)<1e-6:continue
        angle=math.atan2(dot(v,y),dot(v,x))
        weight=max(1.,face['area_px2'])/(1.+p['face_reprojection_rms_px']**2)
        samples.append({'angle_rad':angle,'weight':weight,'area_px2':face['area_px2']})
    if not samples:return {'status':'unavailable','reason':'No independently verified side pose; retain current wrist angle'}
    # Fourfold circular mean: perpendicular and opposite normals encode the
    # same cube yaw. Averaging the raw normals would aim at a cube diagonal.
    c=sum(s['weight']*math.cos(4*s['angle_rad']) for s in samples)
    s=sum(v['weight']*math.sin(4*v['angle_rad']) for v in samples)
    if math.sqrt(c*c+s*s)<1e-6:return {'status':'unavailable','reason':'Side yaw is ambiguous; retain current wrist angle'}
    a=math.atan2(s,c)/4
    u=[math.cos(a)*xx+math.sin(a)*yy for xx,yy in zip(x,y)]
    v=cross(n,u)
    residuals=[abs((p['angle_rad']-a+math.pi/4)%(math.pi/2)-math.pi/4)*180/math.pi for p in samples]
    return {'status':'estimated','method':'verified_side_normals_fourfold_weighted_mean',
            'yaw_table_deg':a*180/math.pi,'period_deg':90,'side_count':len(samples),
            'axes_Base':[u,v],'table_normal_Base':n,'side_residual_deg':residuals,
            'side_weights':[p['weight'] for p in samples]}
