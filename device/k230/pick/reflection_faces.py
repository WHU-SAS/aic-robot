"""Measure a blue face independently of a colour blob's reflected extension.

Colour slices propose search windows only. Every output corner is the
intersection of four supported image edges; neither FK nor a desired pose is
used. Shared by desktop and board, with an injected planar pose solver.
"""
import math
import cv2
try:
    from ulab import numpy as np
except ImportError:
    import numpy as np
FLOAT=getattr(np,'float',None) or np.float32


def distance(a,b):return math.sqrt(sum((x-y)**2 for x,y in zip(a,b)))

def ordered(contour):
    q=[[float(v) for v in p] for p in contour.reshape((-1,2)).tolist()]
    c=[sum(p[k] for p in q)/4 for k in (0,1)]
    q.sort(key=lambda p:math.atan2(p[1]-c[1],p[0]-c[0]))
    i=min(range(4),key=lambda j:sum(q[j]))
    return q[i:]+q[:i]


def fit(points):
    values=cv2.fitLine(np.array(points,dtype=FLOAT),cv2.DIST_HUBER,0,.01,.01)
    # CanMV returns a tuple; desktop OpenCV returns a 4x1 ndarray.
    if hasattr(values,'reshape'):values=values.reshape((-1,)).tolist()
    vx,vy,x,y=[float(v) for v in values]
    return [-vy,vx,vy*x-vx*y]


def residual(p,line):return abs(p[0]*line[0]+p[1]*line[1]+line[2])


def intersect(a,b):
    d=a[0]*b[1]-a[1]*b[0]
    if abs(d)<1e-6:raise ValueError('Parallel measured edges')
    return [(a[1]*b[2]-a[2]*b[1])/d,(a[2]*b[0]-a[0]*b[2])/d]


def sample(im,p):return im[int(round(p[1])),int(round(p[0]))].tolist()


def measure_edge(im,a,b):
    length=distance(a,b)
    if length<25:raise ValueError('Edge too short')
    u=[(b[k]-a[k])/length for k in (0,1)];n=[-u[1],u[0]]
    radius=min(12,max(5,int(round(length*.06))))
    points=[];stations=[];height,width=im.shape[:2]
    for j in range(64):
        t=.15+.7*j/63;mid=[a[k]+(b[k]-a[k])*t for k in (0,1)];best=None;responses={}
        for offset in range(-radius,radius+1):
            p=[mid[k]+offset*n[k] for k in (0,1)]
            upper=[p[k]+2*n[k] for k in (0,1)];lower=[p[k]-2*n[k] for k in (0,1)]
            if any(not(1<v[0]<width-2 and 1<v[1]<height-2) for v in (upper,lower)):continue
            c=sample(im,upper);d=sample(im,lower)
            # Relative contrast preserves exposure variation; no fixed blue brightness.
            # Compare squared relative RGB contrast: same ordering and gate,
            # without a sqrt, generator and concatenated list per pixel.
            den=max(40.,(c[0]+c[1]+c[2]+d[0]+d[1]+d[2])/6)
            dr=c[0]-d[0];dg=c[1]-d[1];db=c[2]-d[2]
            score=(dr*dr+dg*dg+db*db)/(den*den)
            responses[offset]=score
            if best is None or score>best[0]:best=(score,p,j,offset)
        if best and best[0]>.08*.08:
            peak=best[3];before=responses.get(peak-1);after=responses.get(peak+1);sub=0.
            # Locate the observed gradient peak between pixels. Do not move
            # corners to fit a square or use pose error to choose this shift.
            if before is not None and after is not None:
                curvature=before-2*best[0]+after
                if curvature<0:sub=max(-.5,min(.5,.5*(before-after)/curvature))
            points.append([best[1][k]+sub*n[k] for k in (0,1)]);stations.append(best[2])
    if len(points)<45:raise ValueError('Insufficient edge contrast')
    line=fit(points)
    inliers=[(p,j) for p,j in zip(points,stations) if residual(p,line)<=3]
    if len(inliers)<45 or max(j for p,j in inliers)-min(j for p,j in inliers)<50:
        raise ValueError('Edge lacks distributed consensus')
    line=fit([p for p,j in inliers])
    rms=math.sqrt(sum(residual(p,line)**2 for p,j in inliers)/len(inliers))
    if rms>3:raise ValueError('Physical edge RMS exceeds 3px')
    # Independently fit alternating sampling stations. Corners must agree
    # within 3px, so one group of stray reflection/rounded-corner votes cannot
    # determine a missing physical edge.
    groups=[[p for p,j in inliers if j%2==half] for half in (0,1)]
    if min(len(g) for g in groups)<20:raise ValueError('Unbalanced edge support')
    halves=[fit(g) for g in groups]
    return line,{'support':len(inliers),'sampled_stations':64,'rms_px':rms},halves


def blue(h):return 90<=h[0]<=140 and h[1]>=80 and h[2]>=35


def validate_seam(hsv,q,either_polarity=False,image=None):
    a,b=q[:2];length=distance(a,b);n=[-(b[1]-a[1])/length,(b[0]-a[0])/length]
    support=0;colour_votes=[];height,width=hsv.shape[:2]
    for j in range(32):
        t=.2+.6*j/31;p=[a[k]+t*(b[k]-a[k]) for k in (0,1)]
        inside=[p[k]+5*n[k] for k in (0,1)];outside=[p[k]-5*n[k] for k in (0,1)]
        if any(not(0<v[0]<width-1 and 0<v[1]<height-1) for v in (inside,outside)):continue
        lo=sample(hsv,inside);hi=sample(hsv,outside)
        contrast=abs(lo[2]-hi[2]) if either_polarity else lo[2]-hi[2]
        if blue(lo) and blue(hi):
            if contrast>max(10,.12*max(lo[2],hi[2])):support+=1
            if image is not None and either_polarity:
                c=sample(image,inside);d=sample(image,outside)
                delta=[x-y for x,y in zip(c,d)]
                den=max(40.,sum(c+d)/6)
                # Bright top/side faces can have equal V but distinct colour.
                # Use the existing top-boundary RGB contrast level, retaining
                # blue on BOTH sides and distributed, same-direction support.
                if sum(v*v for v in delta)>(.12*den)**2:colour_votes.append(delta)
    if support>=20:return support
    if len(colour_votes)>=20:
        total=[sum(v[k] for v in colour_votes) for k in range(3)]
        channel=max(range(3),key=lambda k:abs(total[k]))
        coherent=sum(v[channel]*total[channel]>0 for v in colour_votes)
        if coherent>=20:return coherent
    raise ValueError('No distributed blue top / side contrast')


def detect_reflection_face(im,K,D,pose_solver,roi=None,adjacent_top=None):
    if adjacent_top is not None:
        try:from .face_selection import adjacent_side
        except ImportError:from face_selection import adjacent_side
    hsv=cv2.cvtColor(im,cv2.COLOR_BGR2HSV);height,width=hsv.shape[:2]
    x0,y0,rw,rh=roi if roi is not None else (0,int(round(height*.45)),width,height-int(round(height*.45)))
    broad=cv2.inRange(hsv,(90,160,35),(140,255,255));broad[:y0,:]=0;broad[y0+rh:,:]=0;broad[:,:x0]=0;broad[:,x0+rw:]=0
    hist=cv2.calcHist([hsv],[0],broad,[180],[0.,180.]).reshape((-1,)).tolist();modes=[]
    for unused in range(4):
        mode=max(range(90,141),key=lambda i:hist[i])
        if hist[mode]<300:break
        modes.append(mode)
        for i in range(max(0,mode-2),min(180,mode+3)):hist[i]=0
    if not modes:return None
    proposals=[]
    for mode in modes:
        for band in (1,2,4):
            for floor in (80,140,200):
                mask=cv2.inRange(hsv,(max(90,mode-band),160,floor),(min(140,mode+band),255,255))
                mask[:y0,:]=0;mask[y0+rh:,:]=0;mask[:,:x0]=0;mask[:,x0+rw:]=0
                cs,unused=cv2.findContours(mask,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)
                for c in cs:
                    area=cv2.contourArea(c);x,y,w,h=cv2.boundingRect(c)
                    if area<1000 or min(w,h)<30 or x<=x0+2 or y<=y0+2 or x+w>=x0+rw-2 or y+h>=y0+rh-2:continue
                    hull=cv2.convexHull(c);p=cv2.approxPolyDP(hull,.02*cv2.arcLength(hull,True),True)
                    if len(p)!=4:continue
                    q=ordered(p)
                    if adjacent_top is not None and not adjacent_side(adjacent_top,q):continue
                    if any(max(distance(a,b) for a,b in zip(q,old[1]))<1 for old in proposals):continue
                    proposals.append((area,q))
    proposals.sort(key=lambda p:p[0],reverse=True)
    sm=cv2.GaussianBlur(im,(3,3),.7);found=[]
    for area,seed in proposals[:24]:
        try:
            edges=[measure_edge(sm,a,b) for a,b in zip(seed,seed[1:]+seed[:1])]
            q=[intersect(edges[i-1][0],edges[i][0]) for i in range(4)]
            for half in (0,1):
                check=[intersect(edges[i-1][2][half],edges[i][2][half]) for i in range(4)]
                if max(distance(a,b) for a,b in zip(q,check))>3:raise ValueError('Interleaved edge fits disagree')
            if max(distance(a,b) for a,b in zip(q,seed))>18:continue
            if any(not(x0+3<p[0]<x0+rw-3 and y0+3<p[1]<y0+rh-3) for p in q):continue
            qa=np.array(q,dtype=FLOAT)
            if not cv2.isContourConvex(qa) or abs(cv2.contourArea(qa))<1000:continue
            if adjacent_top is not None and not adjacent_side(adjacent_top,q):continue
            support=validate_seam(hsv,q,either_polarity=adjacent_top is not None)
            poses=pose_solver(q,K,D)
            if not poses or poses[0]['face_reprojection_rms_px']>2:continue
            found.append({'face_quad_px':q,'poses':poses,'physical_edges':[v[1] for v in edges],'seam_support':support})
        except (ValueError,ZeroDivisionError):continue
    if not found:return None
    # Prefer agreement between independently seeded searches. Each candidate
    # has already passed the alternating-station check on all four edges.
    for f in found:f['near_support']=sum(max(distance(a,b) for a,b in zip(f['face_quad_px'],g['face_quad_px']))<3 for g in found)
    found.sort(key=lambda f:(-f['near_support'],f['poses'][0]['face_reprojection_rms_px']))
    result=found[0];result.update(status='complete_face_candidate',boundary_method='colour_slice_four_measured_edges' if adjacent_top is None else 'adjacent_side_four_measured_edges_refined_pose',selection_uses_fk=False,selection_uses_extrinsics=False)
    return result
