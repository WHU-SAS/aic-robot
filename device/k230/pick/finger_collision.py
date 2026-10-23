"""Conservative finger-volume checks in the same Base frame as FK and vision.

2mm link-local boxes enclose the individual convex finger hulls. The cube is
20mm, oriented with the table and measured sides. Empirical IK target offsets
are NOT additional physical translations of the links.
"""
import math
from numeric import dot,cross,unit,transform


class FingerCollision(ValueError):pass


def boxes_overlap(center,rotation,half,cube_half):
    """Complete 15-axis SAT: oriented finger box against cube-frame AABB.

    rotation rows express the finger's local axes in cube coordinates. Touch
    counts as overlap. No vertex-inside shortcut (it misses edge crossings).
    """
    absolute=[[abs(v)+1e-10 for v in row] for row in rotation]
    for i in range(3):
        if abs(center[i])>cube_half[i]+sum(half[j]*absolute[i][j] for j in range(3)):return False
    for j in range(3):
        if abs(sum(center[i]*rotation[i][j] for i in range(3)))>half[j]+sum(cube_half[i]*absolute[i][j] for i in range(3)):return False
    for i in range(3):
        a=(i+1)%3;b=(i+2)%3
        for j in range(3):
            c=(j+1)%3;d=(j+2)%3
            distance=abs(center[b]*rotation[a][j]-center[a]*rotation[b][j])
            radius=cube_half[a]*absolute[b][j]+cube_half[b]*absolute[a][j]+half[c]*absolute[i][d]+half[d]*absolute[i][c]
            if distance>radius:return False
    return True


def hull_overlap(points,half,neighbors=None):
    """GJK intersection against the complete convex finger hull, after SAT.

    A separating support plane proves clearance. Degenerate or unconverged
    cases remain conservative overlaps, rather than inventing clearance.
    """
    sub=lambda a,b:[a[i]-b[i] for i in range(3)]
    neg=lambda a:[-v for v in a]
    previous=0
    def support(d):
        nonlocal previous
        if neighbors is None:p=max(points,key=lambda p:dot(p,d))
        else:
            # Linear support on a convex polytope: an edge-local maximum is
            # global. Warm-start from the previous GJK support vertex.
            value=dot(points[previous],d)
            while True:
                best=previous
                for index in neighbors[previous]:
                    candidate=dot(points[index],d)
                    if candidate>value:best=index;value=candidate
                if best==previous:break
                previous=best
            p=points[previous]
        return [p[i]+(half[i] if d[i]>=0 else -half[i]) for i in range(3)]
    def line(a,b):
        ab=sub(b,a);ao=neg(a)
        if dot(ab,ao)>0:return [b,a],cross(cross(ab,ao),ab)
        return [a],ao
    simplex=[support([1.,0.,0.])];d=neg(simplex[0])
    for unused in range(48):
        size=math.sqrt(dot(d,d))
        if size<1e-14:return True
        a=support(d)
        if dot(a,d)<-1e-12*size:return False
        simplex.append(a);ao=neg(a)
        if len(simplex)==2:
            simplex,d=line(a,simplex[0])
        elif len(simplex)==3:
            b=simplex[-2];c=simplex[-3];ab=sub(b,a);ac=sub(c,a);abc=cross(ab,ac)
            if dot(cross(abc,ac),ao)>0:
                if dot(ac,ao)>0:simplex=[c,a];d=cross(cross(ac,ao),ac)
                else:simplex,d=line(a,b)
            elif dot(cross(ab,abc),ao)>0:simplex,d=line(a,b)
            elif dot(abc,ao)>0:d=abc
            else:simplex=[b,c,a];d=neg(abc)
        else:
            b,c,e=simplex[-2],simplex[-3],simplex[-4]
            for u,v,opposite in ((b,c,e),(c,e,b),(e,b,c)):
                normal=cross(sub(u,a),sub(v,a))
                if dot(normal,sub(opposite,a))>0:normal=neg(normal)
                if dot(normal,ao)>0:
                    simplex=[v,u,a];d=normal;break
            else:return True
    return True


class CubeEnvelope:
    def __init__(self,g,xy,orientation):
        n=unit(orientation['table_normal_Base']) if orientation and 'table_normal_Base' in orientation else [0.,0.,1.]
        if abs(n[2])<1e-6:raise FingerCollision('Invalid table normal for cube envelope')
        # xy is the center coordinate; table_z is evaluated at exactly that xy.
        self.center=[xy[0],xy[1],g.table_z_m+.010/n[2]]
        known=orientation and orientation.get('status')=='estimated'
        x=orientation['axes_Base'][0] if known else [1.,0.,0.]
        x=unit([x[i]-dot(x,n)*n[i] for i in range(3)])
        self.axes=[x,unit(cross(n,x)),n]
        # Unknown yaw: contain every rotation of the 20mm square on the table.
        self.half=[.010 if known else .010*math.sqrt(2)]*2+[.010]
        self.orientation_known=bool(known)
        self.checks=0

    def pose(self,g,q,inflate=0.):
        self.checks+=1;ts=g.transforms(q)
        for link in ('wan2','zhua'):
            if not g.points[link].get('collision_boxes'):raise FingerCollision('Missing finger collision geometry: '+link)
            t=ts[link];R=[[sum(ax[k]*t[k][j] for k in range(3)) for j in range(3)] for ax in self.axes]
            # Inflation encloses motion between samples, not encoder errors.
            cube_half=[v+inflate for v in self.half]
            bound=g.points[link]['collision_bounds'];p=transform(t,bound['center'])
            delta=[p[i]-self.center[i] for i in range(3)];center=[dot(delta,ax) for ax in self.axes]
            if not boxes_overlap(center,R,bound['half'],cube_half):continue
            for index,box in enumerate(g.points[link]['collision_boxes']):
                p=transform(t,box['center']);delta=[p[i]-self.center[i] for i in range(3)]
                center=[dot(delta,ax) for ax in self.axes]
                if boxes_overlap(center,R,box['half'],cube_half):
                    points=[]
                    for vertex in g.points[link]['hull']:
                        world=transform(t,vertex);delta=[world[i]-self.center[i] for i in range(3)]
                        points.append([dot(delta,ax) for ax in self.axes])
                    if hull_overlap(points,cube_half,g.points[link].get('collision_neighbors')):
                        return {'finger':link,'box':index,'cube_local_center_mm':[v*1000 for v in center],
                                'motion_enclosure_mm':inflate*1000}
                    break
        return None


def segment(g,cube,a,b,label):
    # The maximum downstream distance from each joint bounds its contribution
    # to any finger point's displacement, independent of the current pose.
    tip_radius=max(math.sqrt(dot(v,v)) for link in ('wan2','zhua') for v in g.points[link]['hull'])
    lengths=[math.sqrt(sum(j['origin_transform'][i][3]**2 for i in range(3))) for j in g.joints]
    radii=[sum(lengths[i+1:])+tip_radius for i in range(6)]
    stack=[(a,b,0.,1.)];checks=0
    while stack:
        left,right,u0,u1=stack.pop();q=[(left[i]+right[i])/2 for i in range(6)]
        inflate=sum(abs(right[i]-left[i])*radii[i] for i in range(6))/2
        hit=cube.pose(g,q,inflate);checks+=1
        if not hit:continue  # Entire segment enclosed and separated.
        actual=cube.pose(g,q)
        if actual:
            actual.update(path=label,progress=(u0+u1)/2)
            raise FingerCollision('Open finger intersects cube: '+str(actual))
        if inflate<=.00005:
            hit.update(path=label,progress=(u0+u1)/2)
            raise FingerCollision('Open finger sweep clearance unresolved within 0.05mm: '+str(hit))
        if checks>=4096:raise FingerCollision('Descent sweep exceeds bounded planning work')
        middle=(u0+u1)/2
        stack.append((q,right,middle,u1));stack.append((left,q,u0,middle))
    return checks


def validate_plan(g,plan,cube):
    """Check targets and two nominal descents; not measured servo trajectories.

    Both synchronous interpolation and axis4-completes-first interpolation
    are checked. Actual asynchronous/PID motion is not certified by this model.
    """
    start_checks=cube.checks
    hover=g.angles(plan['hover']);down=g.angles(plan['descend'])
    for name,q in (('hover',hover),('descend',down)):
        hit=cube.pose(g,q)
        if hit:raise FingerCollision('Open finger intersects cube at '+name+': '+str(hit))
    segment(g,cube,hover,down,'synchronous')
    ahead=list(hover);ahead[3]=down[3]
    segment(g,cube,hover,ahead,'axis4_first')
    segment(g,cube,ahead,down,'remaining_axes')
    return {'status':'model_clear','frame':'unshifted_Base_FK','cube_center_Base_mm':[v*1000 for v in cube.center],
            'cube_size_mm':20,'orientation_known':cube.orientation_known,'geometry':'separate_convex_finger_hulls_with_slice_box_broadphase',
            'pose_checks':cube.checks-start_checks,'paths':['synchronous','axis4_first_then_remaining'],
            'sweep_checked':True,
            'actual_motion_certified':False,'encoder_error_included':False}
