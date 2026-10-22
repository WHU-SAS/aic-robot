"""Verify shallow face proposals against four edges in the original image."""
import math
import cv2
try:
    from .reflection_faces import blue,distance,fit,residual,intersect,sample
    from .pose import pose_candidates
    from .face_selection import internal_seam
except ImportError:
    from reflection_faces import blue,distance,fit,residual,intersect,sample
    from pose import pose_candidates
    from face_selection import internal_seam


def measure(im,hsv,a,b,radius,background_required):
    length=distance(a,b)
    if length<18:raise ValueError('Top edge too short for boundary verification')
    n=[-(b[1]-a[1])/length,(b[0]-a[0])/length]
    points=[];stations=[];height,width=im.shape[:2]
    for j in range(64):
        t=.2+.6*j/63;mid=[a[k]+t*(b[k]-a[k]) for k in (0,1)];best=None
        for offset in range(-radius,radius+1):
            p=[mid[k]+offset*n[k] for k in (0,1)]
            inside=[p[k]+3*n[k] for k in (0,1)];outside=[p[k]-3*n[k] for k in (0,1)]
            if any(not(2<v[0]<width-3 and 2<v[1]<height-3) for v in (inside,outside)):continue
            hin,hout=sample(hsv,inside),sample(hsv,outside)
            if not blue(hin) or (background_required and blue(hout)):continue
            c=sample(im,inside);d=sample(im,outside)
            score=distance(c,d)/max(50.,sum(c+d)/6)
            if score<.12:continue
            # Same-colour faces can differ by <35 brightness levels in shade.
            # Use relative brightness, alongside the measured RGB contrast.
            value_jump=abs(hin[2]-hout[2])/max(25.,hin[2],hout[2])
            if blue(hout) and abs(hin[0]-hout[0])<3 and value_jump<.12:continue
            if best is None or score>best[0]:best=(score,p,j)
        if best:points.append(best[1]);stations.append(j)
    if len(points)<45:raise ValueError('Top edge lacks distributed colour/gradient support')
    line=fit(points);pairs=[(p,j) for p,j in zip(points,stations) if residual(p,line)<=3]
    if len(pairs)<45 or max(j for p,j in pairs)-min(j for p,j in pairs)<50:raise ValueError('Top edge samples do not support one line')
    line=fit([p for p,j in pairs]);rms=math.sqrt(sum(residual(p,line)**2 for p,j in pairs)/len(pairs))
    if rms>3:raise ValueError('Top edge line residual exceeds 3px')
    groups=[[p for p,j in pairs if j%2==half] for half in (0,1)]
    if min(len(g) for g in groups)<20:raise ValueError('Top edge sampling is unbalanced')
    return line,{'support':len(pairs),'sampled_stations':64,'rms_px':rms,'background_required':background_required},[fit(g) for g in groups]


def validate_top_boundary(im,K,D,candidate):
    """Return measured corners or an explicit failure; never accept raw fallback."""
    seed=candidate['face_quad_px'];previous={'face_quad_px':seed,'boundary_method':candidate.get('boundary_method'),
        'reprojection_rms_px':candidate['poses'][0]['face_reprojection_rms_px'] if candidate.get('poses') else None}
    try:
        hsv=cv2.cvtColor(im,cv2.COLOR_BGR2HSV);sm=cv2.GaussianBlur(im,(3,3),.7)
        width=(distance(seed[0],seed[1])+distance(seed[3],seed[2]))/2
        radius=int(min(24,max(8,round(width*.18))))
        # A diamond has TWO upper silhouette edges. For positive corner
        # winding, edges pointing right have an upward exterior normal.
        # Near-vertical lateral edges are excluded; always retain the highest
        # edge requirement as well. Cyclic corner order changes none of this.
        back={min(range(4),key=lambda i:(seed[i][1]+seed[(i+1)%4][1])/2)}
        for i,(a,b) in enumerate(zip(seed,seed[1:]+seed[:1])):
            if b[0]-a[0]>.25*distance(a,b):back.add(i)
        edges=[measure(sm,hsv,a,b,radius,i in back) for i,(a,b) in enumerate(zip(seed,seed[1:]+seed[:1]))]
        q=[intersect(edges[i-1][0],edges[i][0]) for i in range(4)]
        for half in (0,1):
            check=[intersect(edges[i-1][2][half],edges[i][2][half]) for i in range(4)]
            if max(distance(a,b) for a,b in zip(q,check))>3:raise ValueError('Top edge interleaved corner fits disagree')
        if max(distance(a,b) for a,b in zip(q,seed))>min(32,max(12,.25*width)):raise ValueError('Measured top boundary too far from proposal')
        height,image_width=im.shape[:2]
        if any(not(3<p[0]<image_width-3 and 3<p[1]<height-3) for p in q):raise ValueError('Measured top is clipped')
        # Four positively oriented convex corners prevent a crossed solution.
        cross=[(q[(i+1)%4][0]-q[i][0])*(q[(i+2)%4][1]-q[(i+1)%4][1])-(q[(i+1)%4][1]-q[i][1])*(q[(i+2)%4][0]-q[(i+1)%4][0]) for i in range(4)]
        if min(cross)<=0:raise ValueError('Measured top is not a convex face')
        area=abs(sum(a[0]*b[1]-b[0]*a[1] for a,b in zip(q,q[1:]+q[:1])))/2
        if area<1000 or min(distance(a,b) for a,b in zip(q,q[1:]+q[:1]))<18:raise ValueError('Measured top has insufficient image extent')
        if internal_seam(hsv,q) is not None:raise ValueError('Measured top contains an interior physical seam')
        good=0
        for u in (.2,.4,.6,.8):
            for v in (.2,.4,.6,.8):
                point=[(1-v)*((1-u)*q[0][k]+u*q[1][k])+v*((1-u)*q[3][k]+u*q[2][k]) for k in (0,1)]
                good+=blue(sample(hsv,point))
        if good<14:raise ValueError('Measured top interior lacks blue support')
        ps=pose_candidates(q,[[float(v) for v in row] for row in K],[float(v) for v in D],refine=True)
        if not ps or ps[0]['face_reprojection_rms_px']>2:raise ValueError('Measured top pose reprojection exceeds 2px')
        result=dict(candidate)
        result.pop('cube_center_Camera_mm',None);result.pop('cube_center_Base_mm',None)
        result.update(status='complete_face_candidate',reason=None,
            face_quad_px=q,poses=ps,boundary_method='measured_top_four_edges_refined_pose',
            physical_edges=[e[1] for e in edges],threshold_proposal=previous,
            corner_adjustments_px=[distance(a,b) for a,b in zip(q,seed)],top_boundary_verified=True)
        return result
    except (ValueError,ZeroDivisionError) as exc:
        return {'status':'no_complete_face','reason':'Top face boundary not verified: '+str(exc),'rejected_top_proposal':previous}
