"""Outer-edge spread fitting using NumPy/ulab matrix projection.

Pure image math: no pose/table objective or hardware operations. For each
sigmoid template g, eliminate the two RGB plateaus analytically. Its centred,
unit-norm column projects each channel profile; maximum summed projection
energy is exactly minimum two-platform RGB least-squares error. The selected
locations still pass the original +/-2px contrast and physical-edge checks.
"""
import math
import cv2
try:
    from ulab import numpy as np
except ImportError:
    import numpy as np

FLOAT=getattr(np,'float',None) or getattr(np,'float64',None) or np.float32
_TEMPLATES={}
_TEMPLATE_ORDER=[]


def _templates(radius):
    cached=_TEMPLATES.get(radius)
    if cached is not None:
        _TEMPLATE_ORDER.remove(radius)
        _TEMPLATE_ORDER.append(radius)
        return cached
    centers=[];widths=[]
    for width in (.7,1.,1.5,2.,3.):
        for j in range(8*radius-20):
            widths.append(width);centers.append(-radius+3+.25*j)
    length=2*radius+1;count=len(centers)
    t=np.array([v-radius for v in range(length)],dtype=FLOAT).reshape((length,1))
    c=np.array(centers,dtype=FLOAT).reshape((1,count))
    w=np.array(widths,dtype=FLOAT).reshape((1,count))
    g=1/(1+np.exp(-(t-c)/w))
    u=g-(np.sum(g,axis=0)/length).reshape((1,count))
    projection=u/np.sqrt(np.sum(u*u,axis=0)).reshape((1,count))
    cached=(projection,centers,widths)
    if len(_TEMPLATE_ORDER)==3:
        del _TEMPLATES[_TEMPLATE_ORDER.pop(0)]
    _TEMPLATES[radius]=cached
    _TEMPLATE_ORDER.append(radius)
    return cached


def _project(centered,projection):
    if hasattr(np,'dot'):
        return np.dot(centered,projection)
    # Same native two-dimensional arithmetic when this ulab lacks dot.
    # The loop has at most25 offsets, never64*templates Python fits.
    rows,length=centered.shape;count=projection.shape[1]
    result=np.zeros((rows,count),dtype=FLOAT)
    for k in range(length):
        result=result+centered[:,k].reshape((rows,1))*projection[k,:].reshape((1,count))
    return result


def spread_edge(im,hsv,a,b,fit_edge,native_contrasts,audit=False):
    """Return None for internal/unsupported ownership; otherwise original edge.

    fit_edge and native_contrasts are the original production helpers injected
    by the caller. No quality threshold is duplicated with a weaker value.
    One RGB remap and one HSV ownership remap sample all64 profiles at once.
    """
    length=math.sqrt(sum((b[k]-a[k])**2 for k in (0,1)))
    if length<25:raise ValueError('Edge too short')
    normal=[-(b[1]-a[1])/length,(b[0]-a[0])/length]
    radius=min(12,max(5,int(round(length*.06))))
    offsets=np.array([j-radius for j in range(2*radius+1)],dtype=FLOAT).reshape((1,2*radius+1))
    along=np.array([.15+.7*j/63 for j in range(64)],dtype=FLOAT).reshape((64,1))
    mx=a[0]+(b[0]-a[0])*along+normal[0]*offsets
    my=a[1]+(b[1]-a[1])*along+normal[1]*offsets
    kind=getattr(np,'float32',FLOAT)
    maps=[np.array(v,dtype=kind) for v in (mx,my)]
    values=np.array(cv2.remap(im,maps[0],maps[1],cv2.INTER_NEAREST),dtype=FLOAT)
    colors=cv2.remap(hsv,maps[0],maps[1],cv2.INTER_NEAREST)
    blue=(colors[:,:,0]>=90)*(colors[:,:,0]<=140)*(colors[:,:,1]>=80)*(colors[:,:,2]>=35)
    outside,inside=blue[:,0],blue[:,-1]
    if float(np.sum(outside*inside))>24 or float(np.sum(inside*(1-outside)))<45:
        return None
    projection,centers,widths=_templates(radius)
    count=len(centers);energy=np.zeros((64,count),dtype=FLOAT)
    variance=np.zeros((64,1),dtype=FLOAT)
    length=2*radius+1
    for channel in range(3):
        row=values[:,:,channel]
        centered=row-(np.sum(row,axis=1)/length).reshape((64,1))
        projected=_project(centered,projection)
        energy=energy+projected*projected
        if audit:variance=variance+np.sum(centered*centered,axis=1).reshape((64,1))
    indices=np.argmax(energy,axis=1).tolist()
    points=[];stations=[];rows=[];coordinates=[]
    for j,index in enumerate(indices):
        index=int(index);center=centers[index]
        t=.15+.7*j/63
        coordinates.append([a[k]+(b[k]-a[k])*t+center*normal[k] for k in (0,1)])
    px=np.array([p[0] for p in coordinates],dtype=FLOAT).reshape((64,1))
    py=np.array([p[1] for p in coordinates],dtype=FLOAT).reshape((64,1))
    strengths=native_contrasts(im,px,py,2*normal[0],2*normal[1])
    energies=energy.tolist() if audit else None
    variances=variance.tolist() if audit else None
    for j,index in enumerate(indices):
        index=int(index);strength=float(strengths[j][0])
        if audit:
            mse=max(0,(variances[j][0]-energies[j][index])/(3*length))
            rows.append([j,centers[index],widths[index],mse,strength])
        if strength>.08*.08:points.append(coordinates[j]);stations.append(j)
    edge=fit_edge(points,stations)
    edge[1].update(appearance_method='colour_adaptive_outer_edge_spread')
    return edge,rows


def _wall_hist_middle(hist,count):
    lower=(count-1)//2;upper=count//2;seen=0;lo=None
    for value in sorted(hist):
        seen+=hist[value]
        if lo is None and seen>lower:lo=value
        if seen>upper:return lo,value


def _wall_noise_sigma(gray):
    """Exact HaarHH median/MAD, sorting at most1021 weighted integer bins."""
    values=np.array(gray,dtype=FLOAT)
    hh=(values[:-1:2,:-1:2]-values[1::2,:-1:2]-values[:-1:2,1::2]+values[1::2,1::2])/2
    hist={};count=0
    for row in hh.tolist():
        for value in row:
            key=int(2*value);hist[key]=hist.get(key,0)+1;count+=1
    if not count:return 0.
    lo,hi=_wall_hist_middle(hist,count);middle_sum=lo+hi;deviations={}
    for key,weight in hist.items():
        distance=abs(2*key-middle_sum)
        deviations[distance]=deviations.get(distance,0)+weight
    lo,hi=_wall_hist_middle(deviations,count)
    return((lo+hi)/8)/.67448975


def wall_edge(p,im,hsv,seed_a,seed_b,front_B,front_C,observed_top_endpoint=None):
    """One image-only wall line; no pose, capture time or motion authority.

    The nominated wall uses its full observed top-to-bottom span. At most3
    top-adjacent Hough proposals share ONE25x64RGB contrast bank. Only actual
    local contrast maxima enter the existing distributed45/64/3px fitter.
    Blue material is required inward; external blue packaging is permitted.
    Image rejection returns(None,audit); an unavailable native API propagates.
    """
    audit={'method':'bounded_Hough_shared_RGB8_whole_wall_peak_consensus',
        'capture_timestamp_ms':None,'clock_domain':None,
        'timestamp_source':'unavailable_to_wall_measurement',
        'candidate_budget':3,'native_RGB_profile_batches':0,
        'profile_shape':[25,64],'sample_fraction':[.15,.85],
        'full_wall_endpoints_roi_px':[list(front_B),list(front_C)],
        'pose_used':False,'table_height_used':False}
    length=p.distance(seed_a,seed_b)
    if length<25:
        audit['reason']='Wall proposal too short';return None,audit
    u=[(seed_b[k]-seed_a[k])/length for k in(0,1)];seed=p.fit([seed_a,seed_b])
    anchor=front_B if observed_top_endpoint is None else observed_top_endpoint
    audit['observed_top_endpoint_roi_px']=list(anchor)
    gray=cv2.cvtColor(im,cv2.COLOR_BGR2GRAY);sigma=_wall_noise_sigma(gray)
    high=max(6.4,5*math.sqrt(12)*sigma);low=.4*high
    audit.update(canny_low=low,canny_high=high,noise_sigma=sigma)
    binary=cv2.Canny(gray,low,high,None,3,True)
    yy=np.arange(gray.shape[0]).reshape((gray.shape[0],1))
    xx=np.arange(gray.shape[1]).reshape((1,gray.shape[1]))
    wall_length=p.distance(front_B,front_C)
    if wall_length<25:
        audit['reason']='Observed whole wall too short';return None,audit
    wu=[(front_C[k]-front_B[k])/wall_length for k in(0,1)]
    along=(xx-front_B[0])*wu[0]+(yy-front_B[1])*wu[1]
    band=(abs(seed[0]*xx+seed[1]*yy+seed[2])<=12)*(along>=0)*(along<=wall_length)
    lines=cv2.HoughLinesP(np.array(binary*band,dtype=np.uint8),1.,math.pi/180.,15,None,25.,5.)
    if lines is None or lines.shape[0]==0:
        audit['reason']='No observed Hough wall segment';return None,audit
    candidates=[]
    for values in lines.reshape((-1,4)).tolist():
        a,b=values[:2],values[2:];size=p.distance(a,b)
        if size<25:continue
        if abs(sum((b[k]-a[k])*u[k]for k in(0,1))/size)<1/math.sqrt(1+.2*.2):continue
        line=p.fit([a,b]);gap=p.residual(anchor,line)
        if gap<=12:candidates.append({'segment_roi_px':[a,b],'line':line,'length_px':size,'top_gap_px':gap})
    candidates.sort(key=lambda item:(item['top_gap_px'],-item['length_px']))
    proposed=[]
    for item in candidates:
        if len(proposed)==3:break
        if any(max(p.residual(q,old['line'])for q in item['segment_roi_px'])<2 for old in proposed):continue
        proposed.append(item)
    audit['proposal_count']=len(proposed)
    if not proposed:
        audit['reason']='No wall segment adjacent to the measured top endpoint';return None,audit
    def project(point):
        delta=sum(seed[k]*point[k]for k in(0,1))+seed[2]
        return[point[k]-delta*seed[k]for k in(0,1)]
    a,b=project(front_B),project(front_C);size=p.distance(a,b)
    normal=[-(b[1]-a[1])/size,(b[0]-a[0])/size]
    t=np.array([[.15+.7*j/63 for j in range(64)]],dtype=p.FLOAT)
    offsets=np.arange(-12,13).reshape((25,1))
    cx=a[0]+(b[0]-a[0])*t;cy=a[1]+(b[1]-a[1])*t
    mx=cx+offsets*normal[0];my=cy+offsets*normal[1]
    rows=np.array(p._native_contrasts(im,mx,my,2*normal[0],2*normal[1])).tolist()
    audit['native_RGB_profile_batches']=1
    kind=getattr(np,'float32',FLOAT)
    colors=cv2.remap(hsv,np.array(mx+5*normal[0],dtype=kind),np.array(my+5*normal[1],dtype=kind),cv2.INTER_NEAREST)
    blue=(colors[:,:,0]>=90)*(colors[:,:,0]<=140)*(colors[:,:,1]>=80)*(colors[:,:,2]>=35)
    best=None;rejected=[]
    for item in proposed:
        line=item['line'];den=line[0]*normal[0]+line[1]*normal[1];points=[];stations=[]
        for j in range(64):
            mid=[float(cx[0,j]),float(cy[0,j])];pred=-(line[0]*mid[0]+line[1]*mid[1]+line[2])/den
            row=[rows[k][j]for k in range(25)]
            peaks=[k for k in range(1,24)if abs(k-12-pred)<=3 and blue[k,j]and
                row[k]>.08*.08 and row[k]>=row[k-1]and row[k]>=row[k+1]]
            if not peaks:continue
            k=max(peaks,key=lambda k:(row[k],-abs(k-12-pred)))
            offset=k-12+p._peak_fraction(row,k)
            points.append([mid[i]+normal[i]*offset for i in(0,1)]);stations.append(j)
        try:edge=p._fit_edge(points,stations)
        except ValueError as exc:
            rejected.append(str(exc));continue
        rank=(edge[1]['support'],-item['top_gap_px'],item['length_px'],-edge[1]['rms_px'])
        if best is None or rank>best[0]:best=rank,edge,item
    if best is None:
        audit.update(reason='No Hough wall has45distributed originalRGB8 peaks',rejections=rejected);return None,audit
    unused,edge,item=best
    edge[1].update(appearance_method=audit['method'],edge_ownership='blue_inside_material_boundary',
        broad_HSV_outside_nonblue_verified=False,original_RGB8_step_evidence=True,
        physical_sample_span='front_top_to_bottom_middle70percent')
    audit['selected']={'segment_roi_px':item['segment_roi_px'],'top_gap_px':item['top_gap_px'],
        'support':edge[1]['support'],'rms_px':edge[1]['rms_px']}
    return edge,audit
