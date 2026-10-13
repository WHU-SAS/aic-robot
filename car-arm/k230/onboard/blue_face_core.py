"""Portable physical blue-cube edge extraction for CanMV and offline replay.
No OpenCV, NumPy, robot access or brightness-cutoff face sizing.
"""
import math

def median(values):
    values = sorted(values)
    n = len(values)
    return (values[n//2] + values[(n-1)//2]) / 2 if n else 0

def blue_hv(rgb):
    r,g,b = rgb
    high = max(r,g,b); low = min(r,g,b); delta = high-low
    if high < 40 or delta*255 < 80*high or not delta:
        return None
    if high == r: hue = ((g-b)/delta) % 6
    elif high == g: hue = (b-r)/delta+2
    else: hue = (r-g)/delta+4
    hue *= 30
    return (hue, high) if 90 <= hue <= 140 else None

def line(points, swap=False):
    """Robust y=a*x+b, or x=a*y+b, represented as Ax+By+C=0."""
    if len(points) < 6: raise ValueError('Insufficient physical edge samples')
    p = [(v[1],v[0]) if swap else (v[0],v[1]) for v in points]
    used = p
    for _ in range(3):
        n = len(used); sx=sum(v[0] for v in used); sy=sum(v[1] for v in used)
        denom=sum(v[0]*v[0] for v in used)-sx*sx/n
        if abs(denom)<1e-6: raise ValueError('Degenerate edge')
        a=(sum(v[0]*v[1] for v in used)-sx*sy/n)/denom; b=(sy-a*sx)/n
        errors=[abs(v[1]-a*v[0]-b) for v in p]
        cutoff=max(2.5,3*median(errors))
        selected=[v for v,e in zip(p,errors) if e <= cutoff]
        if len(selected)<6: break
        used=selected
    rms=math.sqrt(sum((v[1]-a*v[0]-b)**2 for v in used)/len(used))
    return ((1,-a,-b) if swap else (-a,1,-b)), {'support':len(used),'rms_px':rms}

def intersect(u,v):
    a,b,c=u; d,e,f=v; det=a*e-b*d
    if abs(det)<1e-6: raise ValueError('Parallel edges')
    return [(b*f-c*e)/det,(c*d-a*f)/det]

def extract(get_pixel,bbox,width=1280,height=960,step=2):
    x,y,w,h = [int(v) for v in bbox]
    bottom_clipped=y+h>=height-2
    if x<=2 or y<=2 or x+w>=width-2:
        return {'status':'clipped','bbox_px':[x,y,w,h],'candidates':[],'reason':'Top or horizontal physical boundaries are outside frame'}
    xs=list(range(x,x+w,step)); ys=list(range(y,y+h,step))
    grid=[]; profiles=[]; edges=[]
    for yy in ys:
        row=[blue_hv(get_pixel(xx,yy)) for xx in xs]; grid.append(row)
        valid=[i for i,v in enumerate(row) if v is not None]
        edges.append((xs[valid[0]],xs[valid[-1]]) if valid else None)
        central=[v for i,v in enumerate(row) if v and .2*w <= xs[i]-x <= .8*w]
        profiles.append((median([v[0] for v in central]),median([v[1] for v in central])) if len(central)>=5 else None)
    pixel_reads=len(xs)*len(ys)
    # LAB may join low-saturation background to the cube. Split the stricter
    # HSV-blue samples again and retain the largest connected component.
    nx=len(xs);ny=len(ys);seen=bytearray(nx*ny);largest=[]
    for start in range(nx*ny):
        if seen[start] or grid[start//nx][start%nx] is None:continue
        stack=[start];seen[start]=1;component=[]
        while stack:
            pos=stack.pop();component.append(pos);ii=pos//nx;jj=pos%nx
            for di,dj in [(-1,-1),(-1,0),(-1,1),(0,-1),(0,1),(1,-1),(1,0),(1,1)]:
                ai=ii+di;aj=jj+dj
                if 0<=ai<ny and 0<=aj<nx:
                    nxt=ai*nx+aj
                    if not seen[nxt] and grid[ai][aj] is not None:seen[nxt]=1;stack.append(nxt)
        if len(component)>len(largest):largest=component
    active=bytearray(nx*ny)
    for pos in largest:active[pos]=1
    edges=[];profiles=[]
    for i,row in enumerate(grid):
        for j in range(nx):
            if not active[i*nx+j]:row[j]=None
        valid=[j for j,v in enumerate(row) if v is not None]
        edges.append((xs[valid[0]],xs[valid[-1]]) if valid else None)
        central=[v for j,v in enumerate(row) if v and .2*w<=xs[j]-x<=.8*w]
        profiles.append((median([v[0] for v in central]),median([v[1] for v in central])) if len(central)>=5 else None)
    # LAB blob bounds can include a thin blue reflection/background connection.
    # Restrict vertical support to the dense HSV-blue component, not that spur.
    dense=[i for i,row in enumerate(grid) if sum(v is not None for v in row)>=max(6,len(xs)*.15)]
    if not dense:return {'status':'no_blue_support','candidates':[]}
    first,last=dense[0],dense[-1]
    grid=grid[first:last+1];edges=edges[first:last+1];profiles=profiles[first:last+1];ys=ys[first:last+1]
    y=ys[0];h=ys[-1]-y+step
    columns=[]
    for j,xx in enumerate(xs):
        valid=[i for i,row in enumerate(grid) if row[j] is not None]
        columns.append((xx,ys[valid[0]],ys[valid[-1]]) if valid else None)
    candidates=[]; rejected=[]
    seam=None; seam_quality=None
    search=[]
    for i in range(3,len(ys)-3):
        if not .08*h <= ys[i]-y <= .45*h:continue
        before=[p for p in profiles[i-3:i] if p];after=[p for p in profiles[i+1:i+4] if p]
        if len(before)<2 or len(after)<2:continue
        hv=abs(median([p[0] for p in before])-median([p[0] for p in after]))
        vv=abs(median([p[1] for p in before])-median([p[1] for p in after]))
        if vv>=25 or hv>=3:search.append((max(vv/25,hv/3),i,0 if hv/3>vv/25 else 1))
    if search:
        _,idx,ch=max(search); samples=[]
        for j,xx in enumerate(xs):
            if not .2*w<=xx-x<=.8*w:continue
            options=[]
            for i in range(max(2,idx-5),min(len(ys)-2,idx+6)):
                if all(grid[k][j] for k in [i-2,i-1,i+1,i+2]):
                    jump=abs((grid[i+1][j][ch]+grid[i+2][j][ch]-grid[i-1][j][ch]-grid[i-2][j][ch])/2)
                    options.append((jump,i))
            if options:
                jump,i=max(options)
                if jump>=(2 if ch==0 else 15):samples.append((xx,ys[i]))
        try:
            seam,seam_quality=line(samples)
            if abs(seam[0])>.35 or seam_quality['rms_px']>3:seam=None
        except ValueError:pass
    def face(kind,lo,hi,top_line=None,bottom_line=None,value_floor=40):
        try:
            left=[];right=[]
            # Select the interior of each face, away from rounded physical corners.
            for yy,row in zip(ys,grid):
                valid=[j for j,v in enumerate(row) if v and v[1]>=value_floor]
                edge=(xs[valid[0]],xs[valid[-1]]) if valid else None
                if edge and lo+.15*(hi-lo)<yy<hi-.15*(hi-lo):
                    left.append((edge[0],yy));right.append((edge[1],yy))
            l,lq=line(left,True); rr,rq=line(right,True)
            col=[]
            for j,xx in enumerate(xs):
                if not x+.2*w<xx<x+.8*w:continue
                valid=[i for i,row in enumerate(grid) if row[j] and row[j][1]>=value_floor]
                if valid:col.append((xx,ys[valid[0]],ys[valid[-1]]))
            if top_line is None:top_line,tq=line([(v[0],v[1]) for v in col])
            else:tq=seam_quality
            if bottom_line is None:bottom_line,bq=line([(v[0],v[2]) for v in col])
            else:bq=seam_quality
            q=[intersect(l,top_line),intersect(rr,top_line),intersect(rr,bottom_line),intersect(l,bottom_line)]
            quality=[tq,rq,bq,lq]
            if max(v['rms_px'] for v in quality)>3:raise ValueError('Physical edges not straight enough')
            if any(not(x-8<=a<=x+w+8 and y-8<=b<=y+h+8) for a,b in q):raise ValueError('Edge intersection outside component')
            if any(a<=2 or a>=width-2 or b<=2 or b>=height-2 for a,b in q):raise ValueError('Face corner touches frame boundary')
            area=abs(sum(q[i][0]*q[(i+1)%4][1]-q[(i+1)%4][0]*q[i][1] for i in range(4)))/2
            if area<1000:raise ValueError('Face area too small')
            cross=[(q[(i+1)%4][0]-q[i][0])*(q[(i+2)%4][1]-q[(i+1)%4][1])-(q[(i+1)%4][1]-q[i][1])*(q[(i+2)%4][0]-q[(i+1)%4][0]) for i in range(4)]
            if not(all(v>0 for v in cross) or all(v<0 for v in cross)):raise ValueError('Nonconvex physical face')
            candidates.append({'kind':kind,'value_floor':value_floor,'quad_px':q,'area_px':area,'edges':quality})
        except ValueError as exc:rejected.append({'kind':kind,'reason':str(exc)})
    if seam:
        seam_y=-(seam[0]*(x+w/2)+seam[2])/seam[1]
        for floor in [40,120,160,180,200,220]:face('top',y,seam_y,bottom_line=seam,value_floor=floor)
        if not bottom_clipped:face('side',seam_y,y+h,top_line=seam)
        else:rejected.append({'kind':'side','reason':'Side bottom edge is clipped; complete top remains eligible'})
    elif not bottom_clipped:face('unknown',y,y+h)
    else:rejected.append({'kind':'unknown','reason':'Bottom clipped and no reliable complete top seam'})
    boundary=[p for yy,e in zip(ys,edges) if e for p in [(e[0],yy),(e[1],yy)]]
    boundary.extend(p for c in columns if c for p in [(c[0],c[1]),(c[0],c[2])])
    extra,three_diagnostic=three_face_candidates(get_pixel,[x,y,w,h],boundary,width,height)
    candidates.extend(extra)
    primary_parts=[three_diagnostic]+[three_diagnostic[key] for key in ['original_orientation','opposite_orientation'] if key in three_diagnostic]
    variants=[three_diagnostic]
    # Do not discard a genuine short cube edge merely because paint/rounded
    # corners introduce several smaller contour segments. Retain an alternative
    # measured six-edge simplification for the host's full 3D-cube validation.
    if any(len(part.get('simplified_hull_px',[]))>6 for part in primary_parts):
        _,merged=three_face_candidates(get_pixel,[x,y,w,h],boundary,width,height,True)
        three_diagnostic['merged_hull_alternative']=merged
        variants.append(merged)
    cube_outlines=[]
    for variant in variants:
        parts=[variant]+[variant[key] for key in ['original_orientation','opposite_orientation'] if key in variant]
        evidence=max([sum(bool(edge) and edge.get('support',0)>=12 and edge.get('rms_px',99)<=3 for edge in part.get('internal_quality',[])) for part in parts]+[0])
        if evidence<1:continue
        for part in parts:
            corners=part.get('outer_corners_px');quality=part.get('outer_edge_quality',[])
            if corners and len(corners)==6 and len(quality)==6:
                if not any(max(abs(a[k]-b[k]) for a,b in zip(corners,old['corners_px']) for k in range(2))<.01 for old in cube_outlines):
                    cube_outlines.append({'corners_px':corners,'edges':quality,'verified_internal_edges':evidence,
                                          'detector':'cube_outline','edge_size_mm':20,'hull_strategy':part.get('hull_strategy')})
                break
    return {'cube_outlines':cube_outlines,'three_face_diagnostic':three_diagnostic,'status':'faces_extracted' if candidates else 'no_reliable_face','bbox_px':[x,y,w,h],'bottom_clipped':bottom_clipped,'candidates':candidates,'rejected':rejected,'seam_line':list(seam) if seam else None,'pixel_reads':pixel_reads,'method':'broad-blue physical boundary and top/side seam; no bright-patch square sizing'}


def component_boxes(get_pixel,bbox,width=1280,height=960,step=4):
    """Split a broad LAB blob in HSV before deciding whether the cube is clipped.
    Returns only component descriptors, not an image. A clipped background
    component must not suppress a separate complete cube in the same LAB box.
    """
    x,y,w,h=[int(v) for v in bbox]
    x=max(0,x);y=max(0,y);w=min(w,width-x);h=min(h,height-y)
    if w<=0 or h<=0:return []
    xs=list(range(x,x+w,step));ys=list(range(y,y+h,step));nx=len(xs);ny=len(ys)
    flags=bytearray(nx*ny)
    for i,yy in enumerate(ys):
        for j,xx in enumerate(xs):
            if blue_hv(get_pixel(xx,yy)) is not None:flags[i*nx+j]=1
    result=[]
    for start in range(nx*ny):
        if flags[start]!=1:continue
        flags[start]=2;stack=[start];n=0;min_i=ny;min_j=nx;max_i=-1;max_j=-1
        while stack:
            pos=stack.pop();ii=pos//nx;jj=pos%nx;n+=1
            min_i=min(min_i,ii);max_i=max(max_i,ii);min_j=min(min_j,jj);max_j=max(max_j,jj)
            for di,dj in [(-1,-1),(-1,0),(-1,1),(0,-1),(0,1),(1,-1),(1,0),(1,1)]:
                ai=ii+di;aj=jj+dj
                if 0<=ai<ny and 0<=aj<nx:
                    nxt=ai*nx+aj
                    if flags[nxt]==1:flags[nxt]=2;stack.append(nxt)
        if n<100:continue
        left=xs[min_j];right=min(width,xs[max_j]+step);top=ys[min_i];bottom=min(height,ys[max_i]+step)
        if (right-left)*(bottom-top)<6000:continue
        clipped_sides=[]
        if left<=2:clipped_sides.append('left')
        if top<=2:clipped_sides.append('top')
        if right>=width-2:clipped_sides.append('right')
        if bottom>=height-2:clipped_sides.append('bottom')
        clipped=bool(clipped_sides)
        # Include the original broad bounds when close; two-pixel coarse padding
        # recovers border pixels before the finer extraction, without enlarging
        # a truly clipped component into a valid one.
        bx=max(x,left-step);by=max(y,top-step);br=min(x+w,right+step);bb=min(y+h,bottom+step)
        result.append({'bbox_px':[bx,by,br-bx,bb-by],'sampled_pixels':n,
                       'sample_step':step,'clipped':clipped,'clipped_sides':clipped_sides,'coarse_pixel_reads':nx*ny})
    return sorted(result,key=lambda c:c['sampled_pixels'],reverse=True)


def ordered_corners(points):
    cx=sum(p[0] for p in points)/4;cy=sum(p[1] for p in points)/4
    q=sorted([[float(p[0]),float(p[1])] for p in points],key=lambda p:math.atan2(p[1]-cy,p[0]-cx))
    first=min(range(4),key=lambda i:q[i][0]+q[i][1])
    return q[first:]+q[:first]

def native_rect_faces(img,bbox,width=1280,height=960):
    """Native CanMV quadrilateral edges, filtered by the cube's blue interior.
    Does not pretend a whole color silhouette is one physical square face.
    Rect edge strength is reported honestly, rather than a fabricated line RMS.
    """
    if not hasattr(img,'find_rects'):
        return [],{'available':False,'reason':'Firmware has no native find_rects'}
    x,y,w,h=[int(v) for v in bbox];x0=max(0,x-6);y0=max(0,y-6);right=min(width,x+w+6);bottom=min(height,y+h+6)
    roi=(x0,y0,right-x0,bottom-y0)
    try:rects=img.find_rects(roi=roi,threshold=3000)
    except Exception as exc:return [],{'available':True,'error':str(exc),'roi':list(roi)}
    found=[];rejected=[]
    for rect in rects:
        q=ordered_corners(rect.corners())
        if any(a<=2 or a>=width-2 or b<=2 or b>=height-2 for a,b in q):
            rejected.append({'quad_px':q,'reason':'Native face corner touches frame boundary'});continue
        if any(not(x-8<=a<=x+w+8 and y-8<=b<=y+h+8) for a,b in q):continue
        area=abs(sum(q[i][0]*q[(i+1)%4][1]-q[(i+1)%4][0]*q[i][1] for i in range(4)))/2
        if area<1000:continue
        crosses=[(q[(i+1)%4][0]-q[i][0])*(q[(i+2)%4][1]-q[(i+1)%4][1])-(q[(i+1)%4][1]-q[i][1])*(q[(i+2)%4][0]-q[(i+1)%4][0]) for i in range(4)]
        if not(all(v>0 for v in crosses) or all(v<0 for v in crosses)):continue
        blue=0;total=0
        for u in [.15,.3,.5,.7,.85]:
            for v in [.15,.3,.5,.7,.85]:
                # Sample well inside a quad; perspective affine distortion does
                # not change whether these bilinear samples lie inside it.
                px=(1-v)*((1-u)*q[0][0]+u*q[1][0])+v*((1-u)*q[3][0]+u*q[2][0])
                py=(1-v)*((1-u)*q[0][1]+u*q[1][1])+v*((1-u)*q[3][1]+u*q[2][1])
                total+=1
                if blue_hv(img.get_pixel(int(px),int(py))) is not None:blue+=1
        if blue/total<.7:
            rejected.append({'quad_px':q,'reason':'Native quadrilateral interior is not blue','blue_fraction':blue/total});continue
        if any(max(math.sqrt((a[0]-b[0])**2+(a[1]-b[1])**2) for a,b in zip(q,f['quad_px']))<3 for f in found):continue
        found.append({'kind':'unknown','detector':'native_rects','value_floor':-1,
                      'quad_px':q,'area_px':area,'edge_magnitude':int(rect.magnitude()),
                      'blue_fraction':blue/total})
    return found,{'available':True,'roi':list(roi),'rectangle_count':len(rects),'accepted_count':len(found),'rejected':rejected}


def convex_hull(points):
    points=sorted(set(points))
    def turn(a,b,c):return (b[0]-a[0])*(c[1]-a[1])-(b[1]-a[1])*(c[0]-a[0])
    low=[];high=[]
    for p in points:
        while len(low)>1 and turn(low[-2],low[-1],p)<=0:low.pop()
        low.append(p)
    for p in reversed(points):
        while len(high)>1 and turn(high[-2],high[-1],p)<=0:high.pop()
        high.append(p)
    return low[:-1]+high[:-1]


def oriented_three_faces(get_pixel,bbox,boundary,width=1280,height=960,radius=10,partial=False,merge_to_six=False):
    """Six measured outer edges and measured internal cube seams.
    Projective parallel-edge geometry only seeds the seam search. Each face
    requires two outer edges and its own two verified internal edges; a weak
    third seam must not suppress a complete independent face. All image
    processing stays on K230, including the alternative visible-side layout.
    """
    diagnostic={'method':'six outer edges + measured internal gradient seams','hull_strategy':'merged_six' if merge_to_six else 'long_segments'}
    def fail(reason):
        diagnostic['reason']=reason
        return [],diagnostic
    def norm(v):return math.sqrt(sum(k*k for k in v))
    def distance(a,b,p):
        dx=b[0]-a[0];dy=b[1]-a[1];length=norm([dx,dy])
        return abs(dx*(p[1]-a[1])-dy*(p[0]-a[0]))/length if length else 0
    def cross(a,b):return [a[1]*b[2]-a[2]*b[1],a[2]*b[0]-a[0]*b[2],a[0]*b[1]-a[1]*b[0]]
    def join(a,b):return cross(a+[1],b+[1])
    x,y,w,h=bbox
    if x<=2 or y<=2 or x+w>=width-2 or y+h>=height-2:return fail('Six outer edges require an unclipped component')
    try:
        poly=convex_hull(boundary)
        while len(poly)>6:
            costs=[distance(poly[i-1],poly[(i+1)%len(poly)],poly[i]) for i in range(len(poly))]
            if not merge_to_six and min(costs)>6:break
            del poly[costs.index(min(costs))]
        diagnostic['simplified_hull_px']=[list(p) for p in poly]
        if not 6<=len(poly)<=9:return fail('Not a six-edge cube silhouette')
        segments=sorted(range(len(poly)),key=lambda i:norm([poly[(i+1)%len(poly)][k]-poly[i][k] for k in range(2)]),reverse=True)[:6]
        segments.sort();outer=[];quality=[]
        for i in segments:
            a,b=poly[i],poly[(i+1)%len(poly)];dx=b[0]-a[0];dy=b[1]-a[1];length=norm([dx,dy])
            if length<25:return fail('One outer edge is too short to measure')
            points=[]
            for point in boundary:
                u=((point[0]-a[0])*dx+(point[1]-a[1])*dy)/(length*length)
                if .15<u<.85 and distance(a,b,point)<5:points.append(point)
            edge,qq=line(points,abs(dy)>abs(dx));qq['rms_px']/=norm(edge[:2])
            if qq['rms_px']>3:return fail('Outer physical edge is not straight')
            outer.append(edge);quality.append(qq)
        q=[intersect(outer[i-1],outer[i]) for i in range(6)]
        if sum(q[i][0]*q[(i+1)%6][1]-q[(i+1)%6][0]*q[i][1] for i in range(6))<0:q.reverse()
        first=min(range(6),key=lambda i:q[i][0]+q[i][1]);q=q[first:]+q[:first]
        if any(not(x-8<=a<=x+w+8 and y-8<=b<=y+h+8) for a,b in q):return fail('Outer intersection exceeds component')
        turns=[(q[(i+1)%6][0]-q[i][0])*(q[(i+2)%6][1]-q[(i+1)%6][1])-(q[(i+1)%6][1]-q[i][1])*(q[(i+2)%6][0]-q[(i+1)%6][0]) for i in range(6)]
        if not all(v>0 for v in turns):return fail('Outer intersections are not convex')
        diagnostic['outer_corners_px']=q
        ordered_quality=[]
        for i in range(6):
            a,b=q[i],q[(i+1)%6]
            best=min(range(6),key=lambda j:sum(abs(outer[j][0]*point[0]+outer[j][1]*point[1]+outer[j][2])/norm(outer[j][:2]) for point in [a,b]))
            ordered_quality.append(quality[best])
        diagnostic['outer_edge_quality']=ordered_quality
        vp=[cross(join(q[i],q[(i+1)%6]),join(q[(i+3)%6],q[(i+4)%6])) for i in range(3)]
        seed_h=cross(cross(q[0]+[1],vp[1]),cross(q[2]+[1],vp[0]))
        if abs(seed_h[2])<1e-8:return fail('Degenerate seam seed')
        seed=[seed_h[0]/seed_h[2],seed_h[1]/seed_h[2]]
        diagnostic['outer_corners_px']=q;diagnostic['junction_seed_px']=seed
        if not x+5<seed[0]<x+w-5 or not y+5<seed[1]<y+h-5:return fail('Seam seed is outside cube interior')
        seams=[];seam_quality=[];reads=0
        diagnostic['internal_quality']=seam_quality
        def pixel_hsv(px,py):
            px=int(px+.5);py=int(py+.5)
            if not 0<=px<width or not 0<=py<height:return None
            rgb=get_pixel(px,py);hv=blue_hv(rgb)
            if hv is None:return None
            high=max(rgb)
            return (hv[0],255*(high-min(rgb))/high,hv[1])
        for index in [0,2,4]:
            dx=q[index][0]-seed[0];dy=q[index][1]-seed[1];length=norm([dx,dy]);nx=-dy/length;ny=dx/length;points=[]
            if length<30:
                if not partial:return fail('Internal seam is too short to verify')
                seams.append(None);seam_quality.append(None);continue
            for j in range(24):
                t=.2+.55*j/23;px=seed[0]+t*dx;py=seed[1]+t*dy;options=[]
                for offset in range(-radius,radius+1,2):
                    xx=px+nx*offset;yy=py+ny*offset
                    samples=[pixel_hsv(xx+nx*k,yy+ny*k) for k in [-4,-2,2,4]];reads+=4
                    if all(samples):
                        delta=[abs((samples[0][k]+samples[1][k]-samples[2][k]-samples[3][k])/2) for k in range(3)]
                        score=max(delta[0]/3,delta[1]/35,delta[2]/20);options.append((score,xx,yy))
                if options:
                    best=max(options)
                    if best[0]>=.65:points.append((best[1],best[2]))
            if len(points)<12:
                if not partial:return fail('Missing internal photometric seam')
                seams.append(None);seam_quality.append(None);continue
            try:edge,qq=line(points,abs(dy)>abs(dx))
            except ValueError:
                if not partial:raise
                seams.append(None);seam_quality.append(None);continue
            qq['rms_px']/=norm(edge[:2])
            if qq['support']<12 or qq['rms_px']>3:
                if not partial:return fail('Internal seam lacks consistent straight support')
                seams.append(None);seam_quality.append(None);continue
            seams.append(edge);seam_quality.append(qq)
        junctions=[intersect(seams[i],seams[(i+1)%3]) if seams[i] and seams[(i+1)%3] else None for i in range(3)]
        valid=[p for p in junctions if p]
        if not valid:return fail('No face has two measured internal edges')
        junction=[median([p[k] for p in valid]) for k in range(2)]
        diagnostic.update(internal_seams=[list(s) if s else None for s in seams],internal_quality=seam_quality,
                          measured_junction_px=junction,junction_intersections_px=junctions,pixel_reads=reads,
                          per_face_junction=partial)
        if not partial and (norm([junction[k]-seed[k] for k in range(2)])>radius+2 or max(norm([p[k]-junction[k] for k in range(2)]) for p in valid)>8):
            return fail('Three measured internal seams do not meet consistently')
        # Map measured outer quality after ordering/rotating the six corners.
        oq=[]
        for i in range(6):
            a,b=q[i],q[(i+1)%6]
            best=min(range(6),key=lambda j:sum(abs(outer[j][0]*p[0]+outer[j][1]*p[1]+outer[j][2])/norm(outer[j][:2]) for p in [a,b]))
            oq.append(quality[best])
        found=[]
        for index in range(3):
            start=index*2;corner=junctions[index] if partial else junction
            if corner is None or norm([corner[k]-seed[k] for k in range(2)])>radius+2:continue
            face=[q[start],q[(start+1)%6],q[(start+2)%6],corner]
            area=abs(sum(face[i][0]*face[(i+1)%4][1]-face[(i+1)%4][0]*face[i][1] for i in range(4)))/2
            if area<1000 or any(a<=2 or a>=width-2 or b<=2 or b>=height-2 for a,b in face):continue
            found.append({'kind':'unknown','detector':'cube_edges','face_index':index,'value_floor':-1,
                          'quad_px':face,'area_px':area,'internal_seams_verified':True,
                          'edges':[oq[start],oq[(start+1)%6],seam_quality[(index+1)%3],seam_quality[index]]})
        diagnostic['accepted_count']=len(found)
        return found,diagnostic
    except (ValueError,ZeroDivisionError,IndexError) as exc:return fail(str(exc))


def three_face_candidates(get_pixel,bbox,boundary,width=1280,height=960,merge_to_six=False):
    """Try both visible-side arrangements; output original sensor coordinates.
    Reflection is only an internal pixel accessor and does not change camera
    mirror settings, intrinsics, extrinsics, or the outgoing coordinate frame.
    """
    found,diagnostic=oriented_three_faces(get_pixel,bbox,boundary,width,height,merge_to_six=merge_to_six)
    diagnostic['orientation']='primary'
    if found:return found,diagnostic
    x,y,w,h=bbox;axis=2*x+w-1
    def mirrored_pixel(px,py):return get_pixel(axis-px,py)
    mirrored=[(axis-p[0],p[1]) for p in boundary]
    # Rounded narrow faces make the projective seed less accurate. Search a
    # wider pixel neighborhood, while retaining measured-line and PnP gates.
    faces,other=oriented_three_faces(mirrored_pixel,bbox,mirrored,width,height,20,True,merge_to_six)
    for f in faces:
        f['quad_px']=[[axis-p[0],p[1]] for p in f['quad_px']]
        f['face_index']+=3
    # All diagnostic corners and lines are restored to full sensor coordinates.
    for name in ['simplified_hull_px','outer_corners_px','junction_intersections_px']:
        if other.get(name) is not None:other[name]=[[axis-p[0],p[1]] if p else None for p in other[name]]
    for name in ['junction_seed_px','measured_junction_px']:
        if other.get(name) is not None:other[name]=[axis-other[name][0],other[name][1]]
    if other.get('internal_seams') is not None:
        other['internal_seams']=[[-v[0],v[1],v[0]*axis+v[2]] if v else None for v in other['internal_seams']]
    other['orientation']='opposite';other['coordinate_system']='original full sensor pixels'
    return faces,{'original_orientation':diagnostic,'opposite_orientation':other,'accepted_count':len(faces)}
