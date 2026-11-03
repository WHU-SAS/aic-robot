"""Portable multi-corner rigid pose math, shared by near-vision callers.

Object points and corresponding unchanged image observations belong to the
caller. Table-up supplies only an initial orientation; LM then refines all six
pose coordinates freely. This module grants no image or control authority and
contains no quality, parking, table or grasp acceptance policy.
"""
import math
try:
    from ..pick.numeric import (cross, dot, unit, transpose, matmul, matvec,
        least_squares, solve, norm, rotation)
    from ..pick.pose import project, undistort
except ImportError:
    from numeric import (cross, dot, unit, transpose, matmul, matvec,
        least_squares, solve, norm, rotation)
    from pose import project, undistort


def _horizontal_pair(objects):
    best=None
    for i,a in enumerate(objects):
        for j in range(i+1,len(objects)):
            b=objects[j]
            if abs(a[2]-b[2])>1e-9:continue
            dx,dy=b[0]-a[0],b[1]-a[1];length=dx*dx+dy*dy
            if length<=1e-12:continue
            # Lowest observed horizontal edge, then largest object-space span.
            # This is independent of pixel order, image fit and final pose.
            rank=(-a[2],length)
            if best is None or rank>best[0]:best=(rank,i,j,dx,dy)
    if best is None:raise ValueError('Pose initialization needs a horizontal object pair')
    return best[1:]


def initialize(objects,pixels,K,D,up):
    rays=[undistort(v,K,D) for v in pixels]
    i,j,dx,dy=_horizontal_pair(objects)
    first,second=rays[i],rays[j]
    ez=unit(up);line=cross(first+[1.],second+[1.]);horizontal=unit(cross(ez,line))
    if dot(horizontal,[second[0]-first[0],second[1]-first[1],0.])<0:
        horizontal=[-v for v in horizontal]
    # horizontal follows the observed object delta(dx,dy,0). A negative dx
    # (opposite-yaw B/F/E/D/C ordering) MUST reverse the inferred object-X.
    span=math.sqrt(dx*dx+dy*dy);c,s=dx/span,dy/span
    perpendicular=unit(cross(ez,horizontal))
    ex=[c*horizontal[k]-s*perpendicular[k] for k in range(3)]
    ey=unit(cross(ez,ex));ex=unit(cross(ey,ez));R=transpose([ex,ey,ez])
    a=[];b=[]
    for obj,(u,v) in zip(objects,rays):
        rp=matvec(R,obj)
        a.extend([[1.,0.,-u],[0.,1.,-v]])
        b.extend([u*rp[2]-rp[0],v*rp[2]-rp[1]])
    return R,least_squares(a,b)


def update(R,t,delta):
    angle=norm(delta[:3])
    rr=matmul(rotation([v/angle for v in delta[:3]],angle),R) if angle>1e-12 else R
    return rr,[t[k]+delta[k+3] for k in range(3)]


def residuals(R,t,objects,pixels,K,D):
    values=[]
    for obj,target in zip(objects,pixels):
        rp=matvec(R,obj);pixel=project([rp[k]+t[k] for k in range(3)],K,D)
        values.extend([pixel[k]-target[k] for k in range(2)])
    return values


def jacobian(R,t,objects,K,D):
    k1,k2,p1,p2,k3=(list(D)+[0.]*5)[:5];columns=[[] for unused in range(6)]
    for obj in objects:
        rp=matvec(R,obj);X,Y,Z=[rp[k]+t[k] for k in range(3)]
        if Z<=0:raise ValueError('Cube point behind camera')
        x,y=X/Z,Y/Z;r2=x*x+y*y
        radial=1+k1*r2+k2*r2*r2+k3*r2*r2*r2;slope=k1+2*k2*r2+3*k3*r2*r2
        xx=radial+2*x*x*slope+2*p1*y+6*p2*x
        xy=2*x*y*slope+2*p1*x+2*p2*y
        yy=radial+2*y*y*slope+6*p1*y+2*p2*x
        for a,b,focal in ((xx,xy,K[0][0]),(xy,yy,K[1][1])):
            gx,gy,gz=focal*a/Z,focal*b/Z,-focal*(a*x+b*y)/Z
            row=[-gy*rp[2]+gz*rp[1],gx*rp[2]-gz*rp[0],-gx*rp[1]+gy*rp[0],gx,gy,gz]
            for column,value in zip(columns,row):column.append(value)
    return columns


def solve_cube(objects,pixels,K,D,up,iterations=25,center_object_mm=None):
    if center_object_mm is None:center_object_mm=[0.,10.,10.]
    R,t=initialize(objects,pixels,K,D,up)
    error=residuals(R,t,objects,pixels,K,D);cost=dot(error,error);damping=.001
    for unused in range(iterations):
        columns=jacobian(R,t,objects,K,D)
        normal=[[dot(a,b) for b in columns] for a in columns]
        for j in range(6):normal[j][j]+=damping*max(normal[j][j],1e-6)
        try:delta=solve(normal,[-dot(column,error) for column in columns])
        except ValueError:break
        scale=max(1.,norm(delta[:3])/.2,norm(delta[3:])/30.)
        delta=[v/scale for v in delta];rr,tt=update(R,t,delta)
        try:new_error=residuals(rr,tt,objects,pixels,K,D);new_cost=dot(new_error,new_error)
        except ValueError:damping*=10;continue
        if new_cost<cost:
            improvement=cost-new_cost;R,t,error,cost=rr,tt,new_error,new_cost
            damping=max(1e-9,damping*.3)
            if improvement<1e-8:break
        else:
            damping*=10
            if damping>1e8:break
    center=[v+t[k] for k,v in enumerate(matvec(R,center_object_mm))]
    return dict(R=R,t=t,reprojection_RMS_px=math.sqrt(cost/len(objects)),
        cube_center_Camera_mm=center,up_Camera=[row[2] for row in R])
