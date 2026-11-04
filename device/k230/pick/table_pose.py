"""Resolve face poses for an upright 20mm cube on the calibrated table.
No arm feedback, prior target, previous frame or coordinate correction is used.
"""
import math

MAX_NORMAL_DEVIATION_DEG=20.
EXPECTED_CENTER_HEIGHT_MM=10.
HEIGHT_TOLERANCE_MM=10.


def context(T,plane):
    if plane is None:return None
    n=[float(v) for v in plane['normal']];length=math.sqrt(sum(v*v for v in n))
    if length<1e-8:raise ValueError('Invalid table normal')
    sign=1. if n[2]>=0 else -1.
    n=[sign*v/length for v in n];offset=sign*float(plane['offset_mm'])/length
    if not all(math.isfinite(v) for v in n+[offset]):raise ValueError('Invalid table plane')
    return {'T':T,'normal':n,'offset_mm':offset}


def metrics(pose,table,kind):
    T=table['T'];n=table['normal'];c=pose['cube_center_Camera_mm'];v=pose['inward_face_normal_Camera']
    center=[sum(T[i][j]*c[j] for j in range(3))+T[i][3] for i in range(3)]
    normal=[sum(T[i][j]*v[j] for j in range(3)) for i in range(3)]
    length=math.sqrt(sum(x*x for x in normal))
    if length<1e-8:raise ValueError('Invalid face normal')
    cosine=max(-1.,min(1.,sum(a*b for a,b in zip(n,normal))/length))
    # Side normals lie IN the table plane; visible top inward normals point down.
    angle=math.degrees(math.asin(abs(cosine))) if kind=='side' else math.degrees(math.acos(-cosine))
    height=sum(a*b for a,b in zip(n,center))-table['offset_mm']
    height_error=height-EXPECTED_CENTER_HEIGHT_MM
    accepted=(angle<=MAX_NORMAL_DEVIATION_DEG and abs(height_error)<=HEIGHT_TOLERANCE_MM
              and pose['face_reprojection_rms_px']<=2.)
    return {'face_kind':kind,'normal_deviation_deg':angle,'center_height_mm':height,
            'height_error_mm':height_error,'accepted':accepted,
            'score':(angle/MAX_NORMAL_DEVIATION_DEG)**2+(height_error/HEIGHT_TOLERANCE_MM)**2}


def constrain_face(face,table,kind='side'):
    if table is None:return face
    valid=[];audit=[]
    for index,pose in enumerate(face['poses']):
        m=metrics(pose,table,kind);audit.append({'original_rank':index,
          'reprojection_rms_px':pose['face_reprojection_rms_px'],
          'cube_center_Camera_mm':pose['cube_center_Camera_mm'],'table_geometry':m})
        if m['accepted']:
            p=dict(pose);p['table_geometry']=m;p['original_pose_rank']=index;valid.append(p)
    if not valid:return None
    valid.sort(key=lambda p:(p['table_geometry']['score'],p['face_reprojection_rms_px']))
    result=dict(face);result['poses']=valid;result['table_pose_candidates']=audit
    result['pose_selection_rule']='upright_cube_table_geometry_then_reprojection'
    return result


def pair_priority(a,b):
    return (a.get('table_geometry',{}).get('score',0.)+b.get('table_geometry',{}).get('score',0.),
            a['face_reprojection_rms_px']+b['face_reprojection_rms_px'])
