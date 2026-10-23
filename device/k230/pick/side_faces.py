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
    from .table_pose import constrain_face,pair_priority,metrics as table_metrics
except ImportError:
    from reflection_faces import FLOAT,ordered,measure_edge,intersect,distance,validate_seam
    from pose import pose_candidates,project
    from face_selection import internal_seam
    from table_pose import constrain_face,pair_priority,metrics as table_metrics

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


def measure_side(im,hsv,seed,K,D,diagnostic=None):
    edges=[measure_edge(im,a,b) for a,b in zip(seed,seed[1:]+seed[:1])]
    q=[intersect(edges[i-1][0],edges[i][0]) for i in range(4)]
    if diagnostic is not None:diagnostic.update(face_quad_px=q,physical_edges=[e[1] for e in edges])
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
    support=validate_seam(hsv,q[back:]+q[:back],either_polarity=True,image=im)
    poses=pose_candidates(q,K,D,refine=True)
    if diagnostic is not None:diagnostic['pose_reprojection_rms_px']=[p['face_reprojection_rms_px'] for p in poses]
    ps=side_pose(poses)
    if not ps:raise ValueError('No accurate vertical-side pose')
    return {'face_quad_px':q,'poses':ps,'physical_edges':[e[1] for e in edges],
            'seam_support':support,'area_px2':area(q),'boundary_method':'independent_side_four_measured_edges'}


def component_anchor(hsv):
    """Locate the blue object without repeatedly solving a square pose."""
    height,width=hsv.shape[:2]
    mask=cv2.inRange(hsv,(90,120,35),(140,255,255));mask[:int(height*.45),:]=0
    cs,unused=cv2.findContours(mask,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)
    eligible=[c for c in cs if cv2.contourArea(c)>=1000]
    if not eligible:return None
    contour=max(eligible,key=cv2.contourArea);x,y,w,h=cv2.boundingRect(contour)
    if min(w,h)<30:return None
    hull=cv2.convexHull(contour)
    outline=cv2.approxPolyDP(hull,.01*cv2.arcLength(hull,True),True)
    support=cv2.approxPolyDP(contour,.003*cv2.arcLength(contour,True),True)
    vertices=[[float(x),float(y)] for x,y in outline.reshape((-1,2)).tolist()]
    return {'face_quad_px':[[float(x),float(y)],[float(x+w-1),float(y)],
            [float(x+w-1),float(y+h-1)],[float(x),float(y+h-1)]],
            'component_bbox_px':[x,y,w,h],'component_vertices_px':vertices,
            'component_contour_px':[[float(x),float(y)] for x,y in support.reshape((-1,2)).tolist()],
            'unposed_component':True}


def inside_polygon(point,polygon):
    inside=False;x,y=point
    for a,b in zip(polygon,polygon[1:]+polygon[:1]):
        if (a[1]>y)!=(b[1]>y):
            crossing=a[0]+(y-a[1])*(b[0]-a[0])/(b[1]-a[1])
            if x<crossing:inside=not inside
    return inside


def component_support(q,anchor):
    # Interior samples must belong to the original connected component, not a
    # second blue item that happened to fall in the expanded search rectangle.
    contour=anchor['component_contour_px']
    for u,v in ((.5,.5),(.25,.25),(.75,.25),(.25,.75),(.75,.75)):
        p=[(1-v)*((1-u)*q[0][k]+u*q[1][k])+v*((1-u)*q[3][k]+u*q[2][k]) for k in (0,1)]
        if not inside_polygon(p,contour):return False
    return True


def detect_sides(im,K,D,anchor,hsv=None,diagnostics=None):
    unposed=bool(anchor and anchor.get('unposed_component'))
    if not anchor or not anchor.get('face_quad_px') or not (unposed or anchor.get('poses')):return []
    q0=anchor['face_quad_px'];xs=[p[0] for p in q0];ys=[p[1] for p in q0]
    span=max(max(xs)-min(xs),max(ys)-min(ys));h,w=im.shape[:2]
    if unposed:
        x,y,rw,rh=anchor['component_bbox_px']
        x0=max(0,x-24);x1=min(w,x+rw+24);y0=max(0,y-24);y1=min(h,y+rh+24)
    else:
        x0=max(0,int(min(xs)-.6*span));x1=min(w,int(max(xs)+.6*span))
        y0=max(0,int(min(ys)-.4*span));y1=min(h,int(max(ys)+1.6*span))
    if hsv is None:hsv=cv2.cvtColor(im,cv2.COLOR_BGR2HSV)
    roi=np.array(hsv[y0:y1,x0:x1],dtype=np.uint8)
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
    for low,high in ((40,120),(80,160),(120,200),(160,240),(200,255)):
        ranges.append(((90,120,low),(140,255,high)))
    # Adapt value slices to the actual blue component so a dim side still
    # receives a proposal. These windows do not change edge/pose acceptance.
    values=cv2.calcHist([roi],[2],broad,[256],[0.,256.]).reshape((-1,)).tolist()
    total=sum(values);cumulative=0;cuts=[]
    for value,count in enumerate(values):
        cumulative+=count
        while len(cuts)<4 and total and cumulative>=total*(len(cuts)+1)/5:cuts.append(value)
    for low,high in zip([35]+cuts,cuts+[255]):
        if high-low>=12:ranges.append(((90,120,max(35,low-5)),(140,255,min(255,high+5))))
    seeds=[] if unposed else [q0];top_hints=[]
    if unposed and len(anchor['component_vertices_px'])==4:seeds.append(ordered(np.array(anchor['component_vertices_px'],dtype=FLOAT)))
    try:from .face_selection import shallow_face
    except ImportError:from face_selection import shallow_face
    if not unposed and abs(anchor['poses'][0]['inward_face_normal_Camera'][1])>.7:top_hints.append(q0)
    for lo,hi in ranges:
        mask=cv2.inRange(roi,lo,hi)
        mask=cv2.morphologyEx(mask,cv2.MORPH_CLOSE,np.ones((3,3),dtype=np.uint8))
        cs,unused=cv2.findContours(mask,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)
        for c in cs:
            if cv2.contourArea(c)<1000:continue
            hull=cv2.convexHull(c);p=cv2.approxPolyDP(hull,.02*cv2.arcLength(hull,True),True)
            if len(p)!=4:continue
            q=[[x+x0,y+y0] for x,y in ordered(p)]
            if unposed and not component_support(q,anchor):continue
            if unposed and shallow_face(q) and not any(corner_distance(q,old)<3 for old in top_hints):top_hints.append(q)
            if any(corner_distance(q,old)<3 for old in seeds):continue
            seeds.append(q)
    # Top hints and observed lower contour vertices only propose side search
    # windows; each output still needs all FOUR measured physical boundaries.
    cs,unused=cv2.findContours(broad,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)
    for hint in sorted(top_hints,key=lambda q:-area(q))[:8]:
        hint_span=max(max(v[k] for v in hint)-min(v[k] for v in hint) for k in (0,1))
        for contour in cs:
            if cv2.contourArea(contour)<1000:continue
            hull=cv2.convexHull(contour);poly=cv2.approxPolyDP(hull,.01*cv2.arcLength(hull,True),True)
            vertices=[[float(x+x0),float(y+y0)] for x,y in poly.reshape((-1,2)).tolist()]
            for a,b in zip(hint,hint[1:]+hint[:1]):
                if b[0]-a[0]>=-.25*distance(a,b):continue
                lower=[v for v in vertices if v[1]>max(a[1],b[1])+.3*hint_span]
                if not lower:continue
                ends=[min(lower,key=lambda v:abs(v[0]-p[0])) for p in (a,b)]
                if max(abs(v[0]-p[0]) for v,p in zip(ends,(a,b)))>.25*hint_span:continue
                seed=ordered(np.array([b,a,ends[0],ends[1]],dtype=FLOAT))
                if not any(corner_distance(seed,old)<3 for old in seeds):seeds.insert(0,seed)
    sm=cv2.GaussianBlur(im,(3,3),.7);found=[]
    for seed in seeds[:32]:
        detail={'seed_quad_px':seed} if diagnostics is not None else None
        if detail is not None:diagnostics.setdefault('attempts',[]).append(detail)
        try:face=measure_side(sm,hsv,seed,K,D,detail)
        except (ValueError,ZeroDivisionError) as exc:
            if detail is not None:detail['rejection']=str(exc)
            continue
        if detail is not None:detail['measured_side_passed']=True
        q=face['face_quad_px']
        if unposed:
            if not component_support(q,anchor):continue
        elif corner_distance(q,q0)>18 and not shared_edge(q0,q):continue
        existing=next((f for f in found if corner_distance(q,f['face_quad_px'])<6),None)
        if existing is None:found.append(face)
        elif face['poses'][0]['face_reprojection_rms_px']<existing['poses'][0]['face_reprojection_rms_px']:
            found[found.index(existing)]=face
    search={'seed_count':len(seeds),'measured_seed_count':min(32,len(seeds)),'usable_sides':len(found),'value_cuts':cuts}
    if diagnostics is not None:diagnostics.update(search)
    for face in found:face['side_search_diagnostics']=search
    return found


def fast_sides(im,K,D,hsv,table=None,diagnostics=None):
    anchor=component_anchor(hsv)
    if anchor is None:return None
    result=select_sides(detect_sides(im,K,D,anchor,hsv,diagnostics),K,D,table)
    if result is not None:result['vision_path']='direct_component_sides'
    return result


def select_sides(faces,K,D,table=None):
    if table is not None:
        faces=[v for v in (constrain_face(f,table) for f in faces) if v is not None]
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
                    pairs.append((pair_priority(a,b),a,b,separation))
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
            if table is not None:pose['table_geometry']=table_metrics(pose,table,'side')
        else:rule='larger_verified_side'
    result.update(status='complete_face_candidate',poses=[pose],side_selection=rule,
        side_area_ratio_limit=AREA_RATIO,side_center_agreement_limit_mm=CENTER_AGREEMENT_MM,
        selected_side_count=len(used),side_faces=[{'face_quad_px':f['face_quad_px'],'area_px2':f['area_px2'],
            'poses':f['poses'],'physical_edges':f['physical_edges'],
            'table_pose_candidates':f.get('table_pose_candidates',[])} for f in ([first,second] if second else [first])])
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


def prefer_sides(im,K,D,anchor,table=None,diagnostics=None):
    result=select_sides(detect_sides(im,K,D,anchor,diagnostics=diagnostics),K,D,table)
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
