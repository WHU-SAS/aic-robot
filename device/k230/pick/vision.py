"""Board OpenCV blue-face pipeline matching the PC threshold/refinement order.

Only contours, pixel measurements and image reprojection rank candidates. No
robot FK or target residual participates in image selection. Uses ulab rather
than desktop NumPy, and portable two-pose geometry rather than desktop PnP.
"""
import math
import time
import cv2
try:
    from ulab import numpy as np
except ImportError:
    import numpy as np
FLOAT=getattr(np,'float',None) or np.float32
try:
    from .pose import pose_candidates
    from .numeric import norm,dot,solve
    from .reflection_faces import detect_reflection_face
    from .top_boundaries import validate_top_boundary
    from .side_faces import prefer_sides,fast_sides
    from .table_pose import context as table_context,constrain_face
    from .face_selection import internal_seam,separated_face_inside,adjacent_side,independent_adjacent_side,shallow_face
except ImportError:
    from pose import pose_candidates
    from numeric import norm,dot,solve
    from reflection_faces import detect_reflection_face
    from top_boundaries import validate_top_boundary
    from side_faces import prefer_sides,fast_sides
    from table_pose import context as table_context,constrain_face
    from face_selection import internal_seam,separated_face_inside,adjacent_side,independent_adjacent_side,shallow_face

def median(values):
    v=sorted(values);n=len(v)
    return (v[n//2]+v[(n-1)//2])/2 if n else 0.
def points(contour):return [[float(v[0]),float(v[1])] for v in contour.reshape((-1,2)).tolist()]
def ordered(q):
    q=points(q) if hasattr(q,'reshape') else q
    center=[sum(p[k] for p in q)/4 for k in (0,1)]
    q=sorted(q,key=lambda p:math.atan2(p[1]-center[1],p[0]-center[0]))
    first=min(range(4),key=lambda i:sum(q[i]));return q[first:]+q[:first]
def fit_line(p):
    v=cv2.fitLine(np.array(p,dtype=FLOAT),cv2.DIST_HUBER,0,.01,.01)
    if hasattr(v,'reshape'):v=v.reshape((-1,)).tolist()
    vx,vy,x,y=[float(t) for t in v]
    return [-vy,vx,vy*x-vx*y]
def intersect(a,b):
    return solve([[a[0],a[1]],[b[0],b[1]]],[-a[2],-b[2]])
def refine(c,q):
    p=points(c);lines=[]
    for a,b in zip(q,q[1:]+q[:1]):
        u=[b[i]-a[i] for i in (0,1)];length=norm(u)
        if length==0:return q
        u=[v/length for v in u];n=[-u[1],u[0]]
        selected=[]
        for v in p:
            delta=[v[i]-a[i] for i in (0,1)];along=dot(delta,u)
            if abs(dot(delta,n))<7 and length*.1<along<length*.9:selected.append(v)
        if len(selected)<8:return q
        lines.append(fit_line(selected))
    try:result=[intersect(lines[i-1],lines[i]) for i in range(4)]
    except ValueError:return q
    return result if max(norm([a[i]-b[i] for i in (0,1)]) for a,b in zip(result,q))<15 else q

def contours(hsv,s,v,close=True):
    mask=cv2.inRange(hsv,(90,s,v),(140,255,255))
    if close:mask=cv2.morphologyEx(mask,cv2.MORPH_CLOSE,np.ones((3,3),dtype=np.uint8))
    cs,unused=cv2.findContours(mask,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_NONE)
    return cs

def near(a,b,limit=5):return max(norm([v-w for v,w in zip(x,y)]) for x,y in zip(a,b))<limit

def basic_face(hsv,K,D):
    height,width=hsv.shape[:2];accepted=[]
    for value in (160,180,200,220):
        cs=contours(hsv,160,value)
        if not cs:continue
        c=max(cs,key=cv2.contourArea)
        if cv2.contourArea(c)<1000:continue
        hull=cv2.convexHull(c);q=cv2.approxPolyDP(hull,.02*cv2.arcLength(hull,True),True)
        x,y,w,h=cv2.boundingRect(c)
        if len(q)==4 and x>2 and y>2 and x+w<width-2 and y+h<height-2:accepted.append(ordered(q))
    if not accepted:return None
    q=[[median([v[i][j] for v in accepted]) for j in (0,1)] for i in range(4)]
    span=max(max(v[i][j] for v in accepted)-min(v[i][j] for v in accepted) for i in range(4) for j in (0,1))
    if span>12 or min(v[1] for v in q)<432:return None
    ps=pose_candidates(q,K,D)
    if ps and ps[0]['face_reprojection_rms_px']<=2:return {'face_quad_px':q,'poses':ps,'boundary_method':'existing_threshold_median'}
    return None

def dim_face(hsv,K,D):
    height,width=hsv.shape[:2];found=[]
    for s in (80,120,160,200):
        for v in (40,60,80,100,120,140,160,180,200,220):
            for c in contours(hsv,s,v):
                area=cv2.contourArea(c);x,y,w,h=cv2.boundingRect(c)
                if area<6000 or y<height*.45 or x<=2 or x+w>=width-2 or y+h>=height-2:continue
                hull=cv2.convexHull(c);q=cv2.approxPolyDP(hull,.02*cv2.arcLength(hull,True),True)
                if len(q)!=4:continue
                q=refine(c,ordered(q));ps=pose_candidates(q,K,D)
                if ps:found.append({'q':q,'poses':ps,'area':area,'rms':ps[0]['face_reprojection_rms_px']})
    if not found:return None
    largest=max(f['area'] for f in found);found=[f for f in found if f['area']>=largest*.65]
    found.sort(key=lambda f:f['rms']);best=found[0];support=sum(near(f['q'],best['q']) for f in found)
    if best['rms']<=2 and support>=3:return {'face_quad_px':best['q'],'poses':best['poses'],'near_support':support,'boundary_method':'dim_blue_line_refinement'}
    return None

def smooth5(p):
    n=len(p)
    def index(i):
        if i<0:return -i
        if i>=n:return 2*n-2-i
        return i
    return [sum(w*p[index(i+d)] for d,w in zip((-2,-1,0,1,2),(1,4,6,4,1)))/16 for i in range(n)]

def split_face(hsv,K,D):
    height,width=hsv.shape[:2];eligible=[c for c in contours(hsv,120,40) if cv2.contourArea(c)>6000 and cv2.boundingRect(c)[1]>=height*.45]
    if not eligible:return None
    c=max(eligible,key=cv2.contourArea);x,y,w,h=cv2.boundingRect(c)
    if x<=2 or y<=2 or x+w>=width-2 or y+h>=height-2:return None
    x0=x+int(round(w*.2));x1=x+int(round(w*.8));lo=max(5,int(round(h*.08)));hi=min(h-12,int(round(h*.45)))
    if hi<=lo:return None
    seams=[]
    for ch,minimum,jump_min in ((0,.3,3),(2,1.,35)):
        profile=[median([float(hsv[yy,xx,ch]) for xx in range(x0,x1)]) for yy in range(y,y+h)]
        smooth=smooth5(profile);grad=[abs(smooth[i+2]-smooth[i])/2 for i in range(h-2)]
        j=max(range(lo,hi),key=lambda i:grad[i])+1;sy=y+j
        upper=[float(hsv[yy,xx,ch]) for yy in range(sy-12,sy-4) for xx in range(x0,x1)]
        lower=[float(hsv[yy,xx,ch]) for yy in range(sy+4,sy+12) for xx in range(x0,x1)]
        if grad[j-1]>=minimum and abs(median(upper)-median(lower))>=jump_min:seams.append((ch,minimum,sy))
    if not seams:return None
    ch,minimum,sy=seams[0];edges=[]
    for xx in range(x0,x1):
        col=smooth5([float(hsv[yy,xx,ch]) for yy in range(sy-10,sy+11)])
        grad=[abs(col[i+2]-col[i])/2 for i in range(len(col)-2)];k=max(range(len(grad)),key=lambda i:grad[i])+1
        if grad[k-1]>=minimum:edges.append([xx,sy-10+k])
    if len(edges)<max(15,(x1-x0)//3):return None
    line=fit_line(edges)
    if abs(line[1])<1e-6:return None
    slope=-line[0]/line[1];intercept=-line[2]/line[1]
    if abs(slope)>.35 or median([abs(p[1]-p[0]*slope-intercept) for p in edges])>3:return None
    left=max(0,x-2);top=max(0,y-2);right=min(width,x+w+2);bottom=min(height,y+h+2)
    cropped=hsv[top:bottom,left:right,:].copy();found=[]
    for s in (80,120,160,200):
        for val in (40,80,120,160):
            mask=cv2.inRange(cropped,(90,s,val),(140,255,255));mask=cv2.morphologyEx(mask,cv2.MORPH_CLOSE,np.ones((3,3),dtype=np.uint8))
            for yy in range(bottom-top):
                for xx in range(right-left):
                    if yy+top<(xx+left)*slope+intercept:mask[yy,xx]=0
            cs,unused=cv2.findContours(mask,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_NONE)
            if not cs:continue
            c=max(cs,key=cv2.contourArea)
            if cv2.contourArea(c)<6000:continue
            cp=points(c);c=np.array([[p[0]+left,p[1]+top] for p in cp],dtype=FLOAT)
            hull=cv2.convexHull(c);q=cv2.approxPolyDP(hull,.02*cv2.arcLength(hull,True),True)
            if len(q)!=4:continue
            q=refine(c,ordered(q));ps=pose_candidates(q,K,D)
            if ps:found.append({'q':q,'poses':ps,'rms':ps[0]['face_reprojection_rms_px']})
    if not found:return None
    found.sort(key=lambda f:f['rms']);best=found[0];support=sum(near(f['q'],best['q']) for f in found)
    if best['rms']<=2 and support>=3:return {'face_quad_px':best['q'],'poses':best['poses'],'boundary_method':'exposed_cube_top_side_seam_then_side_edges','near_support':support}
    return None

def measured_line(p):
    if len(p)<8:raise ValueError('Insufficient edge support')
    line=fit_line(p);length=norm(line[:2]);line=[v/length for v in line]
    rms=math.sqrt(sum((dot(v,line[:2])+line[2])**2 for v in p)/len(p))
    if rms>3:raise ValueError('Physical edge RMS exceeds 3px')
    return line

def is_blue(hsv,y,x):
    h,s,v=[int(hsv[y,x,c]) for c in range(3)]
    return 90<=h<=140 and s>=80 and v>=35

def seam(hsv,bbox):
    x,y,w,h=bbox;height=hsv.shape[0]
    rows=range(max(8,y-int(round(h*.2))),min(height-8,y+int(round(h*.45))))
    columns=range(x+int(.18*w),x+int(.82*w))
    if len(rows)<12 or len(columns)<15:raise ValueError('Insufficient seam area')
    votes=[]
    for xx in columns:
        scores=[]
        for yy in rows:
            good=all(is_blue(hsv,yy+d,xx) for d in (-5,-4,-3,2,3,4))
            score=0.
            if good:
                upper=[median([int(hsv[yy+d,xx,c]) for d in (-6,-5,-4,-3)]) for c in (0,2)]
                lower=[median([int(hsv[yy+d,xx,c]) for d in (2,3,4,5)]) for c in (0,2)]
                score=abs(upper[0]-lower[0])/3+max(lower[1]-upper[1],0)/30
            scores.append(score)
        for unused in range(3):
            peak=max(range(len(scores)),key=lambda i:scores[i])
            if scores[peak]<1:break
            votes.append((xx,rows[peak],scores[peak]))
            for i in range(max(0,peak-7),min(len(scores),peak+8)):scores[i]=0
    if not votes:raise ValueError('No blue/blue seam')
    best=None
    for k in range(71):
        slope=-.35+k*.01;values=[p[1]-slope*(p[0]-(x+w/2)) for p in votes]
        bins={}
        for intercept,p in zip(values,votes):
            index=int(round(intercept/2));bins[index]=bins.get(index,0)+min(p[2],4)
        for unused in range(4):
            if not bins:break
            peak=max(bins,key=lambda i:bins[i])
            if bins[peak]<=0:break
            selected=[p for p,intercept in zip(votes,values) if abs(intercept-peak*2)<2.5]
            support=len(set(p[0] for p in selected))
            for i in range(peak-3,peak+4):bins[i]=0
            if support<len(columns)*.55 or max(p[0] for p in selected)-min(p[0] for p in selected)<w*.5:continue
            if best is None or support>best[0]:best=(support,selected)
    if best is None:raise ValueError('Seam lacks column consensus')
    return measured_line([p[:2] for p in best[1]])

def gradient_side(im,hsv,K,D):
    height,width=hsv.shape[:2];y0=int(round(height*.45))
    broad=cv2.inRange(hsv,(90,80,35),(140,255,255));broad[:y0,:]=0
    if cv2.countNonZero(broad)<1000:return None
    hist=cv2.calcHist([hsv],[2],broad,[256],[0.,256.]).reshape((-1,)).tolist()
    total=sum(hist);rank=(total-1)*.85;sofar=0;level=0
    for level,n in enumerate(hist):
        sofar+=n
        if sofar>rank:break
    floors=[]
    for factor in (.6,.75,.9):
        v=int(max(35,min(240,level*factor)))
        if v not in floors:floors.append(v)
    proposals=[]
    for saturation in (160,120,80):
        for floor in floors:
            mask=cv2.inRange(hsv,(90,saturation,floor),(140,255,255));mask[:y0,:]=0
            cs,unused=cv2.findContours(mask,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)
            for c in cs:
                area=cv2.contourArea(c);x,y,w,h=cv2.boundingRect(c)
                if area<1000 or w<30 or h<30 or h>3*w or x<=2 or x+w>=width-2 or y<=y0+2 or y+h>=height-2:continue
                box=(x,y,w,h)
                if any(max(abs(a-b) for a,b in zip(box,v[1]))<=5 for v in proposals):continue
                proposals.append((area,box))
    proposals.sort(reverse=True)
    b,g,r=[np.array(v,dtype=FLOAT) for v in cv2.split(im)]
    chroma=cv2.GaussianBlur((b-r)/(b+g+r+1),(3,3),.7)
    gx=cv2.Sobel(chroma,5,1,0,ksize=3)/8;gy=cv2.Sobel(chroma,5,0,1,ksize=3)/8
    for unused_area,bbox in proposals[:12]:
        try:
            x,y,w,h=bbox;top=seam(hsv,bbox);middle=-(top[0]*(x+w/2)+top[2])/top[1]
            expected=w*K[1][1]/K[0][0];radius=max(12,int(round(w*.1)));left=[];right=[];bottom=[]
            first=max(1,int(middle+expected*.2));last=min(height-1,int(middle+expected*.75))
            for yy in range(first,last):
                for center,sign,pts in ((x,1,left),(x+w-1,-1,right)):
                    cols=range(max(1,center-radius),min(width-1,center+radius+1));xx=max(cols,key=lambda v:float(gx[yy,v])*sign)
                    if float(gx[yy,xx])*sign>.02:pts.append([xx,yy])
            radius=max(20,int(round(w*.22)));columns=range(x+int(.2*w),x+int(.8*w))
            for xx in columns:
                rows=range(max(1,y+h-radius),min(height-1,y+h+radius))
                if not rows:continue
                yy=max(rows,key=lambda v:-float(gy[v,xx]))
                if -float(gy[yy,xx])>.02:bottom.append([xx,yy])
            if min(len(left),len(right))<max(12,(last-first)*.6) or len(bottom)<max(12,len(columns)*.6):continue
            l=measured_line(left);r=measured_line(right);b=measured_line(bottom)
            q=[intersect(l,top),intersect(r,top),intersect(r,b),intersect(l,b)]
            if any(not(2<p[0]<width-2 and 2<p[1]<height-2) for p in q):continue
            qa=np.array(q,dtype=FLOAT)
            if not cv2.isContourConvex(qa) or abs(cv2.contourArea(qa))<1000:continue
            ps=pose_candidates(q,K,D)
            if ps and ps[0]['face_reprojection_rms_px']<=2:return {'face_quad_px':q,'poses':ps,'boundary_method':'distributed_colour_seam_and_gradient_edges'}
        except (ValueError,ZeroDivisionError):continue
    return None

def localize(im,K,D,T,table_plane=None):
    """Shared fast sides first; preserve verified arm recovery and v0 paths."""
    if table_plane is None:
        return _localize_legacy(im,K,D,T,table_plane)
    try:
        from ..vision.physical_side import localize as fast_side
    except ImportError:
        try:
            from physical_side import localize as fast_side
        except ImportError:
            from device.k230.vision.physical_side import localize as fast_side
    clock=getattr(time,'ticks_ms',None) or (lambda:int(time.monotonic()*1000))
    difference=getattr(time,'ticks_diff',None) or (lambda a,b:a-b)
    start=clock()
    try:
        result=fast_side(im,K,D,T,table_plane)
    except ValueError as exc:
        fast_ms=difference(clock(),start)
        recovery=clock()
        result=_localize_legacy(im,K,D,T,table_plane)
        result['shared_side_recovery']={'fast_side_reason':str(exc),
            'fast_side_ms':fast_ms,'legacy_recovery_ms':difference(clock(),recovery)}
        return result
    result['shared_side_processing_ms']=difference(clock(),start)
    return result


def _localize_legacy(im,K,D,T,table_plane=None):
    # Synchronous compatibility entry drains the exact same candidate sequence.
    steps=_localize_legacy_steps(im,K,D,T,table_plane)
    try:
        for step in steps:
            if step['stage']=='complete':return step['result']
    finally:_close_localize_steps(steps)
    raise RuntimeError('Legacy localization ended without a result')


def _localize_legacy_steps(im,K,D,T,table_plane=None):
    if im.shape[:2]!=(960,1280):raise ValueError('Expected 1280x960 BGR frame')
    hsv=cv2.cvtColor(im,cv2.COLOR_BGR2HSV)
    table=table_context(T,table_plane)
    yield {'stage':'legacy_hsv'}
    direct=fast_sides(im,K,D,hsv,table)
    yield {'stage':'legacy_direct_sides'}
    if direct is not None:
        center=direct['poses'][0]['cube_center_Camera_mm']
        direct['cube_center_Camera_mm']=center
        direct['cube_center_Base_mm']=[sum(T[i][j]*center[j] for j in range(3))+T[i][3] for i in range(3)]
        yield {'stage':'complete','result':direct}
        return
    result=basic_face(hsv,K,D)
    yield {'stage':'legacy_basic_face'}
    if not result:
        result=dim_face(hsv,K,D)
        yield {'stage':'legacy_dim_face'}
    initial=result
    selected=prefer_sides(im,K,D,result,table)
    yield {'stage':'legacy_prefer_sides'}
    if selected is not None:
        center=selected['poses'][0]['cube_center_Camera_mm']
        selected['cube_center_Camera_mm']=center
        selected['cube_center_Base_mm']=[sum(T[i][j]*center[j] for j in range(3))+T[i][3] for i in range(3)]
        yield {'stage':'complete','result':selected}
        return
    if result is not None and shallow_face(result['face_quad_px']):
        side=split_face(hsv,K,D)
        yield {'stage':'legacy_split_side'}
        if side is None or not adjacent_side(result['face_quad_px'],side['face_quad_px']):
            independent=independent_adjacent_side(im,K,D,result['face_quad_px'])
            yield {'stage':'legacy_adjacent_side'}
            if independent is not None:side=independent
        if side is not None and adjacent_side(result['face_quad_px'],side['face_quad_px']):
            side['preferred_over_top']={'face_quad_px':result['face_quad_px'],
                'reprojection_rms_px':result['poses'][0]['face_reprojection_rms_px'],
                'reason':'Verified adjacent physical side has larger image extent than bright top'}
            result=side
    evidence=internal_seam(hsv,result['face_quad_px']) if result and 'preferred_over_top' not in result else None
    yield {'stage':'legacy_internal_seam'}
    if evidence is not None:
        whole=result;separated=split_face(hsv,K,D)
        yield {'stage':'legacy_separate_outline'}
        result=separated if separated and separated_face_inside(whole['face_quad_px'],separated['face_quad_px']) else None
        if result is not None:
            result['rejected_multiface_outline']={'boundary_method':whole['boundary_method'],
                'face_quad_px':whole['face_quad_px'],'reprojection_rms_px':whole['poses'][0]['face_reprojection_rms_px'],
                'interior_seam':evidence}
    elif result is None:
        result=split_face(hsv,K,D)
        yield {'stage':'legacy_split_fallback'}
    if not result:
        result=gradient_side(im,hsv,K,D)
        yield {'stage':'legacy_gradient_side'}
    if not result:
        result=detect_reflection_face(im,K,D,pose_candidates)
        yield {'stage':'legacy_reflection_face'}
    if result is not initial:
        selected=prefer_sides(im,K,D,result,table)
        yield {'stage':'legacy_prefer_sides'}
        if selected is not None:
            for key in ('rejected_multiface_outline','preferred_over_top'):
                if key in result:selected.setdefault(key,result[key])
            result=selected
    if result is not None and 'side_selection' not in result and shallow_face(result['face_quad_px']):
        result=validate_top_boundary(im,K,D,result)
        yield {'stage':'legacy_top_boundary'}
        if result['status']!='complete_face_candidate':raise ValueError(result['reason'])
    if result is None:raise ValueError('No reliable physical blue20mm face')
    if table is not None and 'side_selection' not in result:
        result=constrain_face(result,table,'top' if shallow_face(result['face_quad_px']) else 'side')
        yield {'stage':'legacy_table_constraint'}
        if result is None:raise ValueError('No face pose consistent with upright cube on calibrated table')
    result.setdefault('status','complete_face_candidate')
    result.setdefault('vision_path','legacy_recovery')
    center=result['poses'][0]['cube_center_Camera_mm']
    result['cube_center_Camera_mm']=center
    result['cube_center_Base_mm']=[sum(T[i][j]*center[j] for j in range(3))+T[i][3] for i in range(3)]
    yield {'stage':'complete','result':result}
    return


def _close_localize_steps(steps):
    close=getattr(steps,'close',None)
    if close is not None:close()


def _shared_side_steps(im,K,D,T,table_plane):
    try:from ..vision.physical_side import localize_steps as sides
    except ImportError:
        try:from physical_side import localize_steps as sides
        except ImportError:from device.k230.vision.physical_side import localize_steps as sides
    intr={'camera_matrix':K,'distortion_coefficients':D,'image_width':1280,'image_height':960}
    # Match the old synchronous shared-side wrapper. The outer session retains
    # the actual capture identity; internal results never renew its timestamp.
    frame={'frame_id':'synchronous_current_image','capture_timestamp_ms':None,
           'image_size_px':[1280,960],'clock_domain':None,'timestamp_source':'unavailable'}
    steps=sides(im,frame,None,intr,{'T_Base_from_Camera_mm':T},table_plane)
    try:
        for event in steps:yield event
    finally:_close_localize_steps(steps)


def localize_steps(im,K,D,T,table_plane=None):
    """Yield the existing full localizer, ending with one raw ``result``.

    Same shared-side default search, then the same legacy recovery order and
    thresholds. No partial step supplies a target. A cv2/pose or legacy candidate
    primitive is still synchronous: yields are cooperative, not a time limit.
    Closing the generator cancels remaining work without camera/UART actions.
    """
    if table_plane is None:
        steps=_localize_legacy_steps(im,K,D,T,table_plane)
        try:
            for event in steps:yield event
        finally:_close_localize_steps(steps)
        return
    clock=getattr(time,'ticks_ms',None) or (lambda:int(time.monotonic()*1000))
    difference=getattr(time,'ticks_diff',None) or (lambda a,b:a-b)
    start=clock();observation=None
    try:
        steps=_shared_side_steps(im,K,D,T,table_plane)
        try:
            for event in steps:
                if event['stage']=='complete':
                    observation=event['observation']
                else:
                    detail=dict(event);detail['stage']='shared_'+event['stage']
                    yield detail
        finally:_close_localize_steps(steps)
        if observation is None:raise RuntimeError('Shared localization ended without a result')
        if observation['status']!='localized':raise ValueError(observation['reason'])
        result=observation['payload']
    except ValueError as exc:
        fast_ms=difference(clock(),start);recovery=clock()
        steps=_localize_legacy_steps(im,K,D,T,table_plane)
        try:
            for event in steps:
                if event['stage']=='complete':
                    result=event['result']
                    result['shared_side_recovery']={'fast_side_reason':str(exc),
                        'fast_side_ms':fast_ms,'legacy_recovery_ms':difference(clock(),recovery)}
                    yield {'stage':'complete','result':result}
                else:yield event
        finally:_close_localize_steps(steps)
        return
    result['shared_side_processing_ms']=difference(clock(),start)
    yield {'stage':'complete','result':result}
