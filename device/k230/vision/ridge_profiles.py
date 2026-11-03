"""Observed G highlight ridge; separate from the original RGB8%step evidence.

One current-frame cue proposes the window.64 measured peak positions require
blue on both fixed8px flanks and>=2DN prominence on each flank. The injected
original line fitter retains its45/64/distribution/RMS/odd-even requirements.
No pose/table score, previous frame, image identity or hardware operation.
"""
import math
import cv2
try:
    from ulab import numpy as np
except ImportError:
    import numpy as np

FLOAT=getattr(np,'float',None) or np.float32
MAP_FLOAT=getattr(np,'float32',FLOAT)


def _G(im,mx,my,tangent):
    values=np.zeros(mx.shape,dtype=FLOAT)
    for shift in (-1,0,1):
        xx=np.array(mx+shift*tangent[0],dtype=MAP_FLOAT)
        yy=np.array(my+shift*tangent[1],dtype=MAP_FLOAT)
        sample=cv2.remap(im,xx,yy,cv2.INTER_LINEAR)
        values=values+np.array(sample[:,:,1],dtype=FLOAT)/3.
    return values


def measure(im,hsv,bbox,fit_edge,fit,residual,cue=None,audit=False):
    """One fixed+/-3px local G peak; no search over appearance policies."""
    if cue is None:
        try:from .separator_profiles import cue24
        except ImportError:from separator_profiles import cue24
        cue=cue24(im,hsv,bbox,fit,residual)
    line,cue_audit=cue
    x,y,w,h=bbox;slope=-line[0]/line[1];length=math.sqrt(1+slope*slope)
    normal=[-slope/length,1/length];tangent=[1/length,slope/length]
    xx=np.array([[x+(w-1)*(.15+.7*j/63)for j in range(64)]],dtype=FLOAT)
    yy=-(line[0]*xx+line[2])/line[1]
    offsets=np.array([[j]for j in range(-4,5)],dtype=FLOAT)
    profile=_G(im,xx+offsets*normal[0],yy+offsets*normal[1],tangent)
    selected=[]
    for j in range(64):
        k=max(range(1,8),key=lambda k:(float(profile[k,j]),-abs(k-4)))
        before,value,after=[float(profile[i,j])for i in (k-1,k,k+1)]
        curvature=before-2*value+after
        fraction=max(-.5,min(.5,.5*(before-after)/curvature))if curvature<0 else 0.
        selected.append(float(k-4)+fraction)
    offsets=np.array([selected],dtype=FLOAT)
    cx,cy=xx+offsets*normal[0],yy+offsets*normal[1]
    center=_G(im,cx,cy,tangent)
    upper=_G(im,cx-8*normal[0],cy-8*normal[1],tangent)
    lower=_G(im,cx+8*normal[0],cy+8*normal[1],tangent)
    points,stations,prominences,rows=[],[],[],[]
    for j in range(64):
        both=True
        for sign in (-1,1):
            px=int(round(float(cx[0,j]+sign*8*normal[0])))
            py=int(round(float(cy[0,j]+sign*8*normal[1])))
            value=hsv[py,px]
            both=both and 90<=value[0]<=140 and value[1]>=80 and value[2]>=35
        rise=float(center[0,j]-upper[0,j]);fall=float(center[0,j]-lower[0,j])
        prominence=min(rise,fall);point=[float(cx[0,j]),float(cy[0,j])]
        supported=both and prominence>=2.
        if supported:
            points.append(point);stations.append(j);prominences.append(prominence)
        if audit:
            rows.append(dict(station=j,point=point,rise_DN=rise,fall_DN=fall,
                prominence_DN=prominence,both_blue=both,supported=supported))
    edge=fit_edge(points,stations)
    values=sorted(prominences);count=len(values)
    median=values[count//2]if count%2 else(values[count//2-1]+values[count//2])/2
    edge[1].update(appearance_method='same_frame_G_ridge',
        station_evidence='G_peak_two_flanks_DN',minimum_prominence_DN=2.,
        flank_span_px=8.,original_RGB8_step_evidence=False)
    details=dict(cue=cue_audit,descriptor='G_ridge',sampled_stations=64,
        support=edge[1]['support'],minimum_prominence_DN=2.,flank_span_px=8.,
        local_peak_window_px=3.,tangent_average_pixels=3,
        prominence_min_DN=min(prominences),prominence_median_DN=median,
        prominence_max_DN=max(prominences),original_RGB8_step_evidence=False)
    kept=[(station,point)for station,point in zip(stations,points)
          if residual(point,edge[0])<=3]
    first=min(kept,key=lambda item:item[1][0]);last=max(kept,key=lambda item:item[1][0])
    details['terminal_support']=dict(first_station=first[0],last_station=last[0],
        first_point_px=first[1],last_point_px=last[1],
        left_gap_px=first[1][0]-(x+(w-1)*.15),
        right_gap_px=x+(w-1)*.85-last[1][0])
    if audit:details.update(measured_points=points,station_ids=stations,station_rows=rows)
    return edge,details
