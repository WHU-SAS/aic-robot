"""Physical blue-cube side from a colour seam and independently measured edges.

Image-only fallback. No FK, target coordinates, extrinsics or relaxed PnP gate.
"""
import cv2
import numpy as np
from host.vision.face_geometry import pose_candidates


def fit_edge(points):
    points=np.asarray(points,np.float32)
    if len(points)<8:raise ValueError('Insufficient physical edge support')
    vx,vy,x,y=cv2.fitLine(points,cv2.DIST_HUBER,0,.01,.01).ravel()
    normal=np.array([-vy,vx],float)
    normal/=np.linalg.norm(normal)
    line=np.r_[normal,-normal@np.array([x,y])]
    residual=points@normal+line[2]
    rms=float(np.sqrt(np.mean(residual**2)))
    if rms>3:raise ValueError('Physical edge RMS exceeds 3px')
    return line,{'support':len(points),'rms_px':rms}


def intersection(a,b):
    value=np.cross(a,b)
    if abs(value[2])<1e-6:raise ValueError('Parallel face edges')
    return value[:2]/value[2]


def consensus_seam(hsv,bbox):
    x,y,w,h=bbox
    rows=np.arange(max(8,y-round(h*.2)),min(hsv.shape[0]-8,y+round(h*.45)))
    columns=np.arange(x+int(.18*w),x+int(.82*w))
    if len(rows)<12 or len(columns)<15:raise ValueError('Insufficient seam search area')
    blue=(hsv[:,:,0]>=90)&(hsv[:,:,0]<=140)&(hsv[:,:,1]>=80)&(hsv[:,:,2]>=35)
    # A seam requires blue material above AND below it, excluding the silhouette.
    above=rows[:,None]+np.arange(-6,-2)
    below=rows[:,None]+np.arange(2,6)
    upper=np.median(hsv[above[:,:,None],columns],axis=1).astype(float)
    lower=np.median(hsv[below[:,:,None],columns],axis=1).astype(float)
    good=blue[(rows[:,None]+np.arange(-5,-2))[:,:,None],columns].all(1)
    good &= blue[(rows[:,None]+np.arange(2,5))[:,:,None],columns].all(1)
    score=np.abs(upper[:,:,0]-lower[:,:,0])/3+np.maximum(lower[:,:,2]-upper[:,:,2],0)/30
    score[~good]=0
    points=[]
    for index,column in enumerate(columns):
        votes=score[:,index].copy()
        for unused in range(3):
            peak=int(votes.argmax())
            if votes[peak]<1:break
            points.append((column,rows[peak],votes[peak]))
            votes[max(0,peak-7):peak+8]=0
    if not points:raise ValueError('No supported blue/blue colour seam')
    points=np.asarray(points,float)
    best=None
    for slope in np.linspace(-.35,.35,71):
        intercept=points[:,1]-slope*(points[:,0]-(x+w/2))
        histogram=np.bincount(np.rint(intercept/2).astype(int),weights=np.minimum(points[:,2],4))
        # A local paint/shadow edge can have stronger votes on one side only.
        # Evaluate competing line modes before requiring distributed support.
        for unused in range(4):
            peak=int(histogram.argmax())
            if histogram[peak]<=0:break
            selected=points[np.abs(intercept-peak*2)<2.5]
            support=len(np.unique(selected[:,0]))
            histogram[max(0,peak-3):peak+4]=0
            if support<len(columns)*.55 or np.ptp(selected[:,0])<w*.5:continue
            if best is None or support>best[0]:best=(support,selected)
    if best is None:raise ValueError('Seam lacks distributed column consensus')
    support,selected=best
    line,quality=fit_edge(selected[:,:2])
    quality['column_support']=support
    quality['searched_columns']=len(columns)
    return line,quality


def side_from_seed(hsv,gx,gy,bbox,K,D):
    x,y,w,h=bbox
    top,tq=consensus_seam(hsv,bbox)
    middle_y=-(top[0]*(x+w/2)+top[2])/top[1]
    # Only sets an interior search band, never supplies a corner or measured size.
    expected_height=w*K[1,1]/K[0,0]
    left=[];right=[];bottom=[]
    radius=max(12,round(w*.1))
    first=max(1,int(middle_y+expected_height*.2))
    last=min(hsv.shape[0]-1,int(middle_y+expected_height*.75))
    for row in range(first,last):
        for center,sign,points in ((x,1,left),(x+w-1,-1,right)):
            columns=np.arange(max(1,center-radius),min(hsv.shape[1]-1,center+radius+1))
            contrast=gx[row,columns]*sign
            peak=int(contrast.argmax())
            if contrast[peak]>.02:points.append([float(columns[peak]),row])
    bottom_radius=max(20,round(w*.22))
    columns=range(x+int(.2*w),x+int(.8*w))
    for column in columns:
        rows=np.arange(max(1,y+h-bottom_radius),min(hsv.shape[0]-1,y+h+bottom_radius))
        if not len(rows):continue
        contrast=-gy[rows,column]
        peak=int(contrast.argmax())
        if contrast[peak]>.02:bottom.append([column,float(rows[peak])])
    if min(len(left),len(right))<max(12,(last-first)*.6) or len(bottom)<max(12,len(columns)*.6):
        raise ValueError('Side edges lack measured gradient support')
    l,lq=fit_edge(left);r,rq=fit_edge(right);b,bq=fit_edge(bottom)
    quad=np.array([intersection(l,top),intersection(r,top),intersection(r,b),intersection(l,b)])
    if not np.isfinite(quad).all() or not cv2.isContourConvex(quad.astype(np.float32)):
        raise ValueError('Invalid physical face quadrilateral')
    if (quad[:,0]<=2).any() or (quad[:,0]>=hsv.shape[1]-2).any() or (quad[:,1]<=2).any() or (quad[:,1]>=hsv.shape[0]-2).any():
        raise ValueError('Physical face corner clipped')
    if abs(cv2.contourArea(quad.astype(np.float32)))<1000:raise ValueError('Physical face too small')
    poses=pose_candidates(quad,K,D)
    if not poses or poses[0]['face_reprojection_rms_px']>2:
        raise ValueError('Physical face reprojection exceeds 2px')
    return {'status':'complete_face_candidate','boundary_method':'distributed_colour_seam_and_gradient_edges',
        'face_identity':'side below measured blue/blue seam','face_quad_px':quad.tolist(),
        'poses':poses,'bbox_px':list(bbox),'physical_edges':[tq,rq,bq,lq],
        'seam_line_px':top.tolist(),'near_support':tq['column_support'],
        'selection_uses_fk':False,'selection_uses_extrinsics':False}


def detect_blue_side(image,K,D,roi=None):
    """Return measured side-face diagnostics; a plain blue rectangle is insufficient."""
    hsv=cv2.cvtColor(image,cv2.COLOR_BGR2HSV)
    height,width=hsv.shape[:2]
    x0,y0,rw,rh=roi if roi is not None else (0,round(height*.45),width,height-round(height*.45))
    broad=cv2.inRange(hsv,np.array([90,80,35]),np.array([140,255,255]))
    region=np.zeros_like(broad);region[y0:y0+rh,x0:x0+rw]=broad[y0:y0+rh,x0:x0+rw]
    values=hsv[:,:,2][region>0]
    if len(values)<1000:return {'status':'no_complete_face','reason':'Insufficient blue support','attempts':[]}
    level=float(np.percentile(values,85))
    floors=list(dict.fromkeys(int(max(35,min(240,level*f))) for f in (.6,.75,.9)))
    proposals=[]
    for saturation in (160,120,80):
        for floor in floors:
            mask=cv2.inRange(hsv,np.array([90,saturation,floor]),np.array([140,255,255]))
            mask[:y0]=0;mask[y0+rh:]=0;mask[:,:x0]=0;mask[:,x0+rw:]=0
            contours,_=cv2.findContours(mask,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)
            for contour in contours:
                area=cv2.contourArea(contour);x,y,w,h=cv2.boundingRect(contour)
                if area<1000 or w<30 or h<30 or h>3*w:continue
                if x<=x0+2 or x+w>=x0+rw-2 or y<=y0+2 or y+h>=y0+rh-2:continue
                box=(x,y,w,h)
                if any(max(abs(a-b) for a,b in zip(box,p[1]))<=5 for p in proposals):continue
                proposals.append((area,box))
    proposals.sort(reverse=True)
    b,g,r=cv2.split(image.astype(np.float32))
    chroma=cv2.GaussianBlur((b-r)/(b+g+r+1),(3,3),.7)
    gx=cv2.Sobel(chroma,cv2.CV_32F,1,0,ksize=3)/8
    gy=cv2.Sobel(chroma,cv2.CV_32F,0,1,ksize=3)/8
    attempts=[]
    for area,bbox in proposals[:12]:
        try:
            result=side_from_seed(hsv,gx,gy,bbox,K,D)
            result['attempts']=attempts
            return result
        except (ValueError,cv2.error,np.linalg.LinAlgError) as exc:
            attempts.append({'bbox_px':list(bbox),'reason':str(exc)})
    # Broad masks can connect the cube to a blue table reflection. Colour
    # slices are only seeds; the shared fallback measures all four real edges.
    from device.k230.pick.reflection_faces import detect_reflection_face
    supported=detect_reflection_face(image,K,D,
        lambda q,k,d:pose_candidates(np.asarray(q),k,d),roi)
    if supported is not None:
        supported['attempts']=attempts
        return supported
    return {'status':'no_complete_face','reason':'No independently supported side face','attempts':attempts}
