"""Measure individual sides and combine only adjacent, consistent cube faces."""
import cv2
try:
    from ulab import numpy as np
except ImportError:
    import numpy as np
try:
    from .reflection_faces import FLOAT,ordered,measure_edge,intersect,distance,validate_seam
    from .pose import pose_candidates,project
    from .face_selection import internal_seam
except ImportError:
    from reflection_faces import FLOAT,ordered,measure_edge,intersect,distance,validate_seam
    from pose import pose_candidates,project
    from face_selection import internal_seam

AREA_RATIO=1.25
CENTER_AGREEMENT_MM=8.


def area(q):return abs(sum(a[0]*b[1]-b[0]*a[1] for a,b in zip(q,q[1:]+q[:1])))/2


def corner_distance(a,b):
    return min(max(distance(p,q) for p,q in zip(a,b[j:]+b[:j])) for j in range(4))


def shared_edge(a,b):
    """Any edge may be shared; face winding and tabletop yaw are irrelevant."""
    for p,q in zip(a,a[1:]+a[:1]):
        length=distance(p,q)
        if length<18:continue
        u=[(q[k]-p[k])/length for k in (0,1)];n=[-u[1],u[0]]
        for r,s in zip(b,b[1:]+b[:1]):
            other=distance(r,s)
            if other<18:continue
            if abs(sum(u[k]*(s[k]-r[k]) for k in (0,1))/other)<.94:continue
            offsets=[[v[k]-p[k] for k in (0,1)] for v in (r,s)]
            if max(abs(sum(v[k]*n[k] for k in (0,1))) for v in offsets)>8:continue
            along=[sum(v[k]*u[k] for k in (0,1)) for v in offsets]
            overlap=max(0.,min(length,max(along))-max(0.,min(along)))
            if overlap<.75*min(length,other):continue
            centers=[[sum(v[k] for v in face)/4-p[k] for k in (0,1)] for face in (a,b)]
            signed=[sum(v[k]*n[k] for k in (0,1)) for v in centers]
            # Duplicate proposals for the SAME side share supporting lines
            # too; real adjacent faces occupy opposite sides of their seam.
            if signed[0]*signed[1]<0 and min(abs(v) for v in signed)>3:return True
    return False


def side_pose(poses):
    # This fixed camera looks across the tabletop: vertical cube sides have
    # inward normals near the camera XZ plane. Do not use FK/target residuals.
    return [p for p in poses if abs(p['inward_face_normal_Camera'][1])<=.6 and p['face_reprojection_rms_px']<=2]


def measure_side(im,hsv,seed,K,D):
    edges=[measure_edge(im,a,b) for a,b in zip(seed,seed[1:]+seed[:1])]
    q=[intersect(edges[i-1][0],edges[i][0]) for i in range(4)]
    for half in (0,1):
        check=[intersect(edges[i-1][2][half],edges[i][2][half]) for i in range(4)]
        if max(distance(a,b) for a,b in zip(q,check))>3:raise ValueError('Side interleaved corners disagree')
    if corner_distance(q,seed)>18:raise ValueError('Side edges too far from proposal')
    h,w=im.shape[:2]
    if any(not(3<p[0]<w-3 and 3<p[1]<h-3) for p in q):raise ValueError('Side clipped')
    if area(q)<1000 or not cv2.isContourConvex(np.array(q,dtype=FLOAT)):raise ValueError('Side not convex')
    if internal_seam(hsv,q) is not None:raise ValueError('Side proposal contains another face')
    # Identify its upper edge independently of ordered()'s starting corner.
    back=min(range(4),key=lambda i:(q[i][1]+q[(i+1)%4][1])/2)
    support=validate_seam(hsv,q[back:]+q[:back],either_polarity=True)
    ps=side_pose(pose_candidates(q,K,D,refine=True))
    if not ps:raise ValueError('No accurate vertical-side pose')
    return {'face_quad_px':q,'poses':ps,'physical_edges':[e[1] for e in edges],
            'seam_support':support,'area_px2':area(q),'boundary_method':'independent_side_four_measured_edges'}


def detect_sides(im,K,D,anchor):
    if not anchor or not anchor.get('poses') or not anchor.get('face_quad_px'):return []
    q0=anchor['face_quad_px'];xs=[p[0] for p in q0];ys=[p[1] for p in q0]
    span=max(max(xs)-min(xs),max(ys)-min(ys));h,w=im.shape[:2]
    x0=max(0,int(min(xs)-.6*span));x1=min(w,int(max(xs)+.6*span))
    y0=max(0,int(min(ys)-.4*span));y1=min(h,int(max(ys)+1.6*span))
    # Work only around the already selected blue object, not other blue items.
    hsv=cv2.cvtColor(im,cv2.COLOR_BGR2HSV);roi=np.array(hsv[y0:y1,x0:x1],dtype=np.uint8)
    broad=cv2.inRange(roi,(90,120,35),(140,255,255))
    hist=cv2.calcHist([roi],[0],broad,[180],[0.,180.]).reshape((-1,)).tolist();modes=[]
    for unused in range(4):
        mode=max(range(90,141),key=lambda i:hist[i])
        if hist[mode]<100:break
        modes.append(mode)
        for i in range(max(0,mode-2),min(180,mode+3)):hist[i]=0
    ranges=[]
    for mode in modes:
        for band in (1,2,4):
            ranges.append(((max(90,mode-band),120,35),(min(140,mode+band),255,255)))
    # Hue can coincide across top/side; bounded value slices propose the dim
    # side without treating the entire two-side outline as one square.
    for low,high in ((40,120),(80,160),(120,200),(160,240),(200,255)):
        ranges.append(((90,120,low),(140,255,high)))
    seeds=[q0]
    for lo,hi in ranges:
        mask=cv2.inRange(roi,lo,hi)
        mask=cv2.morphologyEx(mask,cv2.MORPH_CLOSE,np.ones((3,3),dtype=np.uint8))
        cs,unused=cv2.findContours(mask,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)
        for c in cs:
            if cv2.contourArea(c)<1000:continue
            hull=cv2.convexHull(c);p=cv2.approxPolyDP(hull,.02*cv2.arcLength(hull,True),True)
            if len(p)!=4:continue
            q=[[x+x0,y+y0] for x,y in ordered(p)]
            if any(corner_distance(q,old)<3 for old in seeds):continue
            seeds.append(q)
    # A top and one side can share the same hue. Use the observed full blue
    # contour's LOWER vertices to seed each exposed side, not a 20mm template.
    # All four edges, including the interior vertical edge, are still measured.
    if abs(anchor['poses'][0]['inward_face_normal_Camera'][1])>.7:
        cs,unused=cv2.findContours(broad,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)
        for contour in cs:
            if cv2.contourArea(contour)<1000:continue
            hull=cv2.convexHull(contour)
            poly=cv2.approxPolyDP(hull,.01*cv2.arcLength(hull,True),True)
            vertices=[[float(x+x0),float(y+y0)] for x,y in poly.reshape((-1,2)).tolist()]
            for a,b in zip(q0,q0[1:]+q0[:1]):
                if b[0]-a[0]>=-.25*distance(a,b):continue
                lower=[v for v in vertices if v[1]>max(a[1],b[1])+.3*span]
                if not lower:continue
                ends=[min(lower,key=lambda v:abs(v[0]-p[0])) for p in (a,b)]
                if max(abs(v[0]-p[0]) for v,p in zip(ends,(a,b)))>.25*span:continue
                seed=ordered(np.array([b,a,ends[0],ends[1]],dtype=FLOAT))
                if not any(corner_distance(seed,old)<3 for old in seeds):seeds.append(seed)
    sm=cv2.GaussianBlur(im,(3,3),.7);found=[]
    for seed in seeds[:32]:
        try:face=measure_side(sm,hsv,seed,K,D)
        except (ValueError,ZeroDivisionError):continue
        if corner_distance(face['face_quad_px'],q0)>18 and not shared_edge(q0,face['face_quad_px']):continue
        existing=next((f for f in found if corner_distance(face['face_quad_px'],f['face_quad_px'])<6),None)
        if existing is None:found.append(face)
        elif face['poses'][0]['face_reprojection_rms_px']<existing['poses'][0]['face_reprojection_rms_px']:
            found[found.index(existing)]=face
    return found


def select_sides(faces,K,D):
    if not faces:return None
    faces=sorted(faces,key=lambda f:-f['area_px2']);first=faces[0];second=None;pair=None
    for candidate in faces[1:]:
        if not shared_edge(first['face_quad_px'],candidate['face_quad_px']):continue
        pairs=[]
        for a in first['poses']:
            for b in candidate['poses']:
                normals=abs(sum(x*y for x,y in zip(a['inward_face_normal_Camera'],b['inward_face_normal_Camera'])))
                separation=distance(a['cube_center_Camera_mm'],b['cube_center_Camera_mm'])
                if normals<=.35 and separation<=CENTER_AGREEMENT_MM:
                    pairs.append((a['face_reprojection_rms_px']+b['face_reprojection_rms_px'],a,b,separation))
        if not pairs:raise ValueError('Adjacent side centers or normals disagree; no average target')
        pairs.sort(key=lambda p:p[0]);pair=pairs[0];second=candidate;break
    result=dict(first);used=[first];rule='single_verified_side'
    pose=first['poses'][0]
    if second is not None:
        pose=pair[1]
        if first['area_px2']/second['area_px2']<=AREA_RATIO:
            used=[first,second];rule='mean_of_similar_area_sides'
            center=[(a+b)/2 for a,b in zip(pair[1]['cube_center_Camera_mm'],pair[2]['cube_center_Camera_mm'])]
            pose=dict(pose);pose['cube_center_Camera_mm']=center;pose['cube_center_pixel']=project(center,K,D)
            pose['face_reprojection_rms_px']=max(pair[1]['face_reprojection_rms_px'],pair[2]['face_reprojection_rms_px'])
        else:rule='larger_verified_side'
    result.update(status='complete_face_candidate',poses=[pose],side_selection=rule,
        side_area_ratio_limit=AREA_RATIO,side_center_agreement_limit_mm=CENTER_AGREEMENT_MM,
        selected_side_count=len(used),side_faces=[{'face_quad_px':f['face_quad_px'],'area_px2':f['area_px2'],
            'poses':f['poses'],'physical_edges':f['physical_edges']} for f in ([first,second] if second else [first])])
    for index,face in enumerate(result['side_faces']):
        face['used_for_center']=index<len(used)
        face['selected_pose']=pair[index+1] if pair else first['poses'][0]
    if second:result['side_area_ratio']=first['area_px2']/second['area_px2']
    if len(used)==2:
        result['boundary_method']='two_sides_measured_edges_mean_center'
        result['center_estimate_method']='arithmetic_mean_of_independent_side_centers'
        result['reprojection_summary']='maximum_selected_face_rms_not_mean_center_reprojection'
    if pair:result['side_center_separation_mm']=pair[3]
    return result


def prefer_sides(im,K,D,anchor):
    result=select_sides(detect_sides(im,K,D,anchor),K,D)
    if result is not None and internal_seam(cv2.cvtColor(im,cv2.COLOR_BGR2HSV),anchor['face_quad_px']) is not None:
        result['rejected_multiface_outline']={'face_quad_px':anchor['face_quad_px'],
            'reason':'Measured an individual side instead of the merged outline'}
    if result is not None and abs(anchor['poses'][0]['inward_face_normal_Camera'][1])>.7:
        try:from .top_boundaries import validate_top_boundary
        except ImportError:from top_boundaries import validate_top_boundary
        top=validate_top_boundary(im,K,D,anchor)
        result['preferred_over_top']={'face_quad_px':anchor['face_quad_px'],
            'reason':'User policy: independently verified sides before top'}
        result['top_validation']={'status':top['status'],'reason':top.get('reason')}
        if top.get('poses'):
            result['top_validation']['face_quad_px']=top['face_quad_px']
            result['top_validation']['center_separation_mm']=distance(
                top['poses'][0]['cube_center_Camera_mm'],result['poses'][0]['cube_center_Camera_mm'])
    return result
