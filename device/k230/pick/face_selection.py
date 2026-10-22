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
