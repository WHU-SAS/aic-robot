"""Two planar poses from homography differential constraints; no NumPy/OpenCV.

A centered square and normalized image rays give a 2x2 homography Jacobian.
Its maximum singular value fixes scale; the missing third row has two signs.
Both proper rotations receive a linear least-squares translation, then are
ranked using the same distorted pixel reprojection error as the PC reference.
"""
import math
try:
    from .numeric import dot,cross,norm,unit,transpose,matmul,matvec,solve,least_squares,rotation
except ImportError:
    from numeric import dot,cross,norm,unit,transpose,matmul,matvec,solve,least_squares,rotation

OBJECT=[[-10.,-10.,0.],[10.,-10.,0.],[10.,10.,0.],[-10.,10.,0.]]


def undistort(pixel,K,D):
    x0=(pixel[0]-K[0][2])/K[0][0];y0=(pixel[1]-K[1][2])/K[1][1]
    x,y=x0,y0;k1,k2,p1,p2,k3=(list(D)+[0.]*5)[:5]
    # Same five fixed-point iterations as cv2.undistortPoints' default.
    for unused in range(5):
        r2=x*x+y*y;rad=1+k1*r2+k2*r2*r2+k3*r2*r2*r2
        if rad<=0:raise ValueError('Invalid distortion inverse')
        dx=2*p1*x*y+p2*(r2+2*x*x);dy=p1*(r2+2*y*y)+2*p2*x*y
        x,y=(x0-dx)/rad,(y0-dy)/rad
    return [x,y]


def project(point,K,D):
    if point[2]<=0:raise ValueError('Point behind camera')
    x,y=point[0]/point[2],point[1]/point[2]
    k1,k2,p1,p2,k3=(list(D)+[0.]*5)[:5];r2=x*x+y*y
    rad=1+k1*r2+k2*r2*r2+k3*r2*r2*r2
    u=x*rad+2*p1*x*y+p2*(r2+2*x*x)
    v=y*rad+p1*(r2+2*y*y)+2*p2*x*y
    return [u*K[0][0]+K[0][2],v*K[1][1]+K[1][2]]


def pose_candidates(quad,K,D,refine=False,refiner=None):
    rays=[undistort(p,K,D) for p in quad];a=[];b=[]
    for obj,(u,v) in zip(OBJECT,rays):
        x,y=obj[:2]
        a.append([x,y,1.,0.,0.,0.,-u*x,-u*y]);b.append(u)
        a.append([0.,0.,0.,x,y,1.,-v*x,-v*y]);b.append(v)
    h=solve(a,b);p,q=h[2],h[5]
    jac=[[h[0]-h[6]*p,h[1]-h[7]*p],[h[3]-h[6]*q,h[4]-h[7]*q]]
    view=unit([p,q,1.]);axis=cross([0.,0.,1.],view);angle=math.acos(max(-1.,min(1.,view[2])))
    Q=rotation(unit(axis),angle) if norm(axis)>1e-12 else [[1.,0.,0.],[0.,1.,0.],[0.,0.,1.]]
    B=matmul([[1.,0.,-p],[0.,1.,-q]],Q)
    cols=[solve([row[:2] for row in B],col) for col in transpose(jac)]
    A=transpose(cols);ata=matmul(transpose(A),A)
    tr=ata[0][0]+ata[1][1];disc=math.sqrt(max(0.,(ata[0][0]-ata[1][1])**2+4*ata[0][1]**2))
    scale=math.sqrt((tr+disc)/2)
    if scale<1e-12:raise ValueError('Degenerate planar face')
    cols=[[v/scale for v in col] for col in cols]
    z0=math.sqrt(max(0.,1-dot(cols[0],cols[0])))
    z1=math.sqrt(max(0.,1-dot(cols[1],cols[1])))
    if dot(cols[0],cols[1])>0:z1=-z1
    result=[]
    for sign in (1.,-1.):
        c0=cols[0]+[sign*z0];c1=cols[1]+[sign*z1]
        R=matmul(Q,transpose([c0,c1,cross(c0,c1)]))
        a=[];b=[]
        for obj,(u,v) in zip(OBJECT,rays):
            rp=matvec(R,obj)
            a.extend([[1.,0.,-u],[0.,1.,-v]]);b.extend([u*rp[2]-rp[0],v*rp[2]-rp[1]])
        t=least_squares(a,b)
        if refine:R,t=(refiner or refine_transform)(R,t,quad,K,D)
        predicted=[]
        try:
            for obj in OBJECT:
                rp=matvec(R,obj);predicted.append(project([rp[i]+t[i] for i in range(3)],K,D))
        except ValueError:continue
        rms=math.sqrt(sum(sum((v-w)**2 for v,w in zip(a,b)) for a,b in zip(quad,predicted))/4)
        inward=[row[2] for row in R]
        if dot(inward,t)<0:inward=[-v for v in inward]
        center=[t[i]+10*inward[i] for i in range(3)]
        pose={'face_reprojection_rms_px':rms,'cube_center_Camera_mm':center,
              'cube_center_pixel':project(center,K,D),'inward_face_normal_Camera':inward}
        # Optional rigid-cube evidence: front corners retain the reprojection
        # used above; the opposite corners lie 20mm along the inward normal.
        try:
            corners=predicted[:]
            for obj in OBJECT:
                rp=matvec(R,obj)
                corners.append(project([rp[i]+t[i]+20*inward[i] for i in range(3)],K,D))
            if all(math.isfinite(v) for corner in corners for v in corner):
                pose['cube_projected_corners_px']=corners
                pose['cube_projection_source']='pose_transform'
        except (ValueError,ZeroDivisionError,OverflowError):pass
        result.append(pose)
    return sorted(result,key=lambda v:v['face_reprojection_rms_px'])


def refine_transform(R,t,quad,K,D):
    """Bounded LM refinement of a planar seed against the SAME measured corners.

    Optimizes six rigid-pose parameters, never changes the measured quadrilateral
    or the acceptance limit. Both planar solutions are refined independently.
    """
    def residuals(rot,tr):
        values=[]
        for obj,target in zip(OBJECT,quad):
            rp=matvec(rot,obj);p=project([rp[i]+tr[i] for i in range(3)],K,D)
            values.extend([p[k]-target[k] for k in (0,1)])
        return values
    def step(rot,tr,d):
        angle=norm(d[:3])
        updated=matmul(rotation([v/angle for v in d[:3]],angle),rot) if angle>1e-12 else rot
        return updated,[tr[i]+d[i+3] for i in range(3)]
    error=residuals(R,t);cost=dot(error,error);damping=.001
    for unused in range(20):
        columns=[]
        for j in range(6):
            # Central differences with resolvable steps also work with the
            # board's float32 arithmetic; tiny one-sided steps lose precision.
            h=.001 if j<3 else .01;delta=[0.]*6;delta[j]=h
            rr,tt=step(R,t,delta);plus=residuals(rr,tt)
            delta[j]=-h;rr,tt=step(R,t,delta);minus=residuals(rr,tt)
            columns.append([(v-w)/(2*h) for v,w in zip(plus,minus)])
        normal=[[dot(a,b) for b in columns] for a in columns]
        for j in range(6):normal[j][j]+=damping*max(normal[j][j],1e-6)
        try:delta=solve(normal,[-dot(a,error) for a in columns])
        except ValueError:break
        scale=max(1.,norm(delta[:3])/.2,norm(delta[3:])/30.)
        delta=[v/scale for v in delta];rr,tt=step(R,t,delta)
        try:new_error=residuals(rr,tt);new_cost=dot(new_error,new_error)
        except ValueError:damping*=10;continue
        if new_cost<cost:
            improvement=cost-new_cost;R,t,error,cost=rr,tt,new_error,new_cost
            damping=max(1e-9,damping*.3)
            if improvement<1e-8:break
        else:
            damping*=10
            if damping>1e8:break
    return R,t
