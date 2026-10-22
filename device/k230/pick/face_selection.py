"""Do not treat a multi-face blue silhouette as one physical square."""
import math


def _blue(p):
    return 90<=p[0]<=140 and p[1]>=80 and p[2]>=35


def internal_seam(hsv,quad):
    """Cheap distributed colour-step probe inside a candidate, not at its rim.

    A strong interior step requires the existing physical-face separator before
    this candidate can be accepted. A low square reprojection error alone does
    not establish that a silhouette contains only one plane.
    """
    height,width=hsv.shape[:2]
    def at(u,v):
        p=[(1-v)*((1-u)*quad[0][k]+u*quad[1][k])+v*((1-u)*quad[3][k]+u*quad[2][k]) for k in (0,1)]
        x,y=int(round(p[0])),int(round(p[1]))
        if not(0<=x<width and 0<=y<height):return None
        return [float(hsv[y,x,k]) for k in (0,1,2)]
    votes=[]
    for column in range(24):
        u=.2+.6*column/23;best=None
        for row in range(7,30):
            v=row/64
            a=at(u,v-.025);b=at(u,v+.025)
            if a is None or b is None or not(_blue(a) and _blue(b)):continue
            contrast=max(abs(a[0]-b[0])/4,abs(a[2]-b[2])/max(25.,.18*max(a[2],b[2])))
            if contrast>=1 and (best is None or contrast>best[0]):best=(contrast,row)
        if best is not None:votes.append(best[1])
    for row in range(7,30):
        support=sum(abs(r-row)<=1 for r in votes)
        if support>=18:return {'row_fraction':row/64,'column_support':support,'searched_columns':24}
    return None


def separated_face_inside(whole,part):
    """A verified side must cut through the old candidate, not move its rim."""
    def line_distance(p,a,b):
        dx,dy=b[0]-a[0],b[1]-a[1];length=math.sqrt(dx*dx+dy*dy)
        if length<1e-6:return -1e9
        return (dx*(p[1]-a[1])-dy*(p[0]-a[0]))/length
    edges=list(zip(whole,whole[1:]+whole[:1]))
    for u in (.25,.5,.75):
        p=[part[0][k]*(1-u)+part[1][k]*u for k in (0,1)]
        if min(line_distance(p,a,b) for a,b in edges)<=5:return False
    for p in part[2:]:
        d=[line_distance(p,a,b) for a,b in edges]
        if min(d)<-8 or min(abs(v) for v in d)>8:return False
    return True


def adjacent_side(top,side):
    """Associate a shallow top with its larger side using their shared edge.

    Thresholds can trim a top corner ALONG the seam without changing the seam.
    Match supporting line direction, normal gap and overlapping edge intervals,
    rather than requiring its trimmed endpoint to coincide with a side corner.
    Both inputs must already have passed their physical-face quality checks.
    """
    def distance(a,b):return math.sqrt(sum((x-y)**2 for x,y in zip(a,b)))
    def area(q):return abs(sum(a[0]*b[1]-b[0]*a[1] for a,b in zip(q,q[1:]+q[:1])))/2
    tw=(distance(top[0],top[1])+distance(top[3],top[2]))/2
    th=(distance(top[0],top[3])+distance(top[1],top[2]))/2
    sw=distance(side[0],side[1])
    if tw<1 or th/tw>.65 or not .7<sw/tw<1.45:return False
    if area(side)<1.5*area(top):return False
    a,b=side[:2];length=distance(a,b)
    u=[(b[k]-a[k])/length for k in (0,1)];n=[-u[1],u[0]]
    t0,t1=top[3],top[2];edge_length=distance(t0,t1)
    if edge_length<1:return False
    direction=sum((t1[k]-t0[k])*u[k] for k in (0,1))/edge_length
    if direction<.94:return False
    offsets=[[t[k]-a[k] for k in (0,1)] for t in (t0,t1)]
    if max(abs(sum(v[k]*n[k] for k in (0,1))) for v in offsets)>max(8.,.08*tw):return False
    along=[sum(v[k]*u[k] for k in (0,1)) for v in offsets]
    overlap=max(0.,min(length,max(along))-max(0.,min(along)))
    if overlap<.75*min(length,edge_length):return False
    return (side[2][1]+side[3][1])/2>(top[2][1]+top[3][1])/2+th


def independent_adjacent_side(im,K,D,top):
    """Separate side colours, measure four real edges and refine rigid pose."""
    try:
        from .reflection_faces import detect_reflection_face
        from .pose import pose_candidates
    except ImportError:
        from reflection_faces import detect_reflection_face
        from pose import pose_candidates
    return detect_reflection_face(im,K,D,
        lambda q,k,d:pose_candidates(q,k,d,refine=True),adjacent_top=top)


def shallow_face(q):
    """Image-shape eligibility only; not a claim that the face is horizontal."""
    def length(a,b):return math.sqrt(sum((x-y)**2 for x,y in zip(a,b)))
    width=(length(q[0],q[1])+length(q[3],q[2]))/2
    height=(length(q[0],q[3])+length(q[1],q[2]))/2
    return width>1 and height/width<=.65
