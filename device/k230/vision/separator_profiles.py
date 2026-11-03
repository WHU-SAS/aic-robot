"""Current-image separator:24column G cue,64station B transition.

No camera, hardware, SciPy, dynamic experiment import, pose/table input, or
known image coordinates. Input im is the production3x3/.7 smoothed ROI.
"""
import math
import cv2
try:
    from ulab import numpy as np
except ImportError:
    import numpy as np

FLOAT=getattr(np,'float',None) or getattr(np,'float64',None) or np.float32
MAP_FLOAT=getattr(np,'float32',FLOAT)


def _maps(mx,my):return [np.array(mx,dtype=MAP_FLOAT),np.array(my,dtype=MAP_FLOAT)]


def cue24(im,hsv,bbox,fit,residual):
    """Only a proposal; this12/24 cue never substitutes for45/64 edge proof."""
    x,y,w,h=bbox;first=max(8,int(y+6));last=min(im.shape[0]-8,int(y+.55*h))
    columns=[x+w*(.15+.7*j/23) for j in range(24)]
    xx=np.array(columns,dtype=FLOAT).reshape((24,1))
    yy=np.array(list(range(first-8,last+8)),dtype=FLOAT).reshape((1,last-first+16))
    maps=_maps(xx+0*yy,yy+0*xx)
    rgb=cv2.remap(im,maps[0],maps[1],cv2.INTER_NEAREST)
    colors=cv2.remap(hsv,maps[0],maps[1],cv2.INTER_NEAREST)
    values=rgb[:,:,1].tolist();hs=colors.tolist();votes=[]
    for j,row in enumerate(values):
        for k in range(8,len(row)-8):
            if row[k]<row[k-1] or row[k]<row[k+1] or row[k]<=min(row[k-1],row[k+1]):continue
            score=min(row[k]-row[k-8],row[k]-row[k+8])
            if score<2:continue
            if not all(90<=hs[j][i][0]<=140 and hs[j][i][1]>=80 and hs[j][i][2]>=35 for i in (k-8,k+8)):continue
            votes.append((j,columns[j],float(first-8+k),float(score)))
    best=None;cx=x+w/2
    for k in range(-4,5):
        slope=k*.05;bins={}
        for j,px,py,score in votes:bins.setdefault(round((py-slope*(px-cx))/2),set()).add(j)
        for key in sorted(bins,key=lambda v:-len(bins[v]))[:3]:
            chosen={}
            for j,px,py,score in votes:
                if abs(py-slope*(px-cx)-key*2)<=2.5 and (j not in chosen or score>chosen[j][2]):chosen[j]=(px,py,score)
            if len(chosen)<12:continue
            line=fit([list(v[:2]) for v in chosen.values()])
            kept=[(j,v) for j,v in chosen.items() if residual(v[:2],line)<=3]
            if len(kept)<12 or max(j for j,v in kept)-min(j for j,v in kept)<18:continue
            line=fit([list(v[:2]) for j,v in kept]);rms=math.sqrt(sum(residual(v[:2],line)**2 for j,v in kept)/len(kept))
            rank=(len(kept),sum(v[2] for j,v in kept),-rms)
            if best is None or rank>best[0]:best=(rank,line)
    if best is None:raise ValueError('No distributed same-frame G-band cue')
    return best[1],dict(columns=24,support=best[0][0],rms_px=-best[0][2],method='same_frame_G_peak_cue')


def _sample(im,hsv,mx,my,normal):
    linear=[];nearest=[];blue=None
    for sign in (-1,1):
        maps=_maps(mx+sign*5*normal[0],my+sign*5*normal[1])
        linear.append(np.array(cv2.remap(im,maps[0],maps[1],cv2.INTER_LINEAR),dtype=FLOAT))
        nearest.append(np.array(cv2.remap(im,maps[0],maps[1],cv2.INTER_NEAREST),dtype=FLOAT))
        colors=cv2.remap(hsv,maps[0],maps[1],cv2.INTER_NEAREST)
        valid=(colors[:,:,0]>=90)*(colors[:,:,0]<=140)*(colors[:,:,1]>=80)*(colors[:,:,2]>=35)
        blue=valid if blue is None else blue*valid
    delta=linear[1]-linear[0];den=np.maximum(40.,np.sum(linear[0]+linear[1],axis=2)/6)
    physical_delta=nearest[1]-nearest[0]
    physical_den=np.maximum(40.,np.sum(nearest[0]+nearest[1],axis=2)/6)
    supported=blue*(np.sum(physical_delta*physical_delta,axis=2)/(physical_den*physical_den)>.08*.08)
    return np.maximum(delta[:,:,0],0)/den*blue,supported,delta[:,:,0]


def measure(im,hsv,bbox,fit_edge,fit,residual,cue=None,audit=False):
    """Fixed energy descriptor; arrays bounded to45models x64stations.

    fit_edge is the original45/64/distribution/RMS/odd-even authority. This
    function grants no pose, parking or arm capability. Current-frame G cue
    merely brackets the measured B transition; no remembered coordinates.
    """
    cue_line,cue_audit=cue if cue is not None else cue24(im,hsv,bbox,fit,residual)
    x,y,w,h=bbox
    def at(px):return [px,-(cue_line[0]*px+cue_line[2])/cue_line[1]]
    a,b=at(x),at(x+w-1);length=math.sqrt(sum((b[k]-a[k])**2 for k in (0,1)))
    normal=[-(b[1]-a[1])/length,(b[0]-a[0])/length]
    fraction=np.array([.15+.7*j/63 for j in range(64)],dtype=FLOAT).reshape((1,64))
    cx=a[0]+(b[0]-a[0])*fraction;cy=a[1]+(b[1]-a[1])*fraction
    along=(fraction-.5)*length
    offset_bank=[-13+.25*i for i in range(45)]
    offset_column=np.array([[value] for value in offset_bank],dtype=FLOAT)
    best=None
    for j in range(41):
        slope=-.2+.01*j
        offsets=offset_column+slope*along
        response,support,unused=_sample(im,hsv,cx+normal[0]*offsets,cy+normal[1]*offsets,normal)
        scores=(np.sum(response,axis=1)/64).tolist();counts=np.sum(support,axis=1).tolist()
        for i,offset in enumerate(offset_bank):
            if (counts[i]<45 or offset-abs(slope)*.35*length < -14 or
                    offset+abs(slope)*.35*length > -1):continue
            if best is None or scores[i]>best[0]:best=(scores[i],offset,slope,int(counts[i]))
    if best is None:raise ValueError('No joint B line with45 original contrast stations')
    score,offset,slope,count=best
    model=offset+slope*along
    local=model+np.array([[-2+.25*j] for j in range(17)],dtype=FLOAT)
    response,observed,db=_sample(im,hsv,cx+normal[0]*local,cy+normal[1]*local,normal)
    responses=response.tolist();supported=observed.tolist();jumps=db.tolist();locations=local.tolist()
    coordinates=[];stations=[]
    for j in range(64):
        choices=[k for k in range(17) if supported[k][j] and jumps[k][j]>0]
        if not choices:continue
        k=max(choices,key=lambda k:(responses[k][j],-abs(-2+.25*k)));v=locations[k][j]
        t=.15+.7*j/63
        coordinates.append([a[axis]+(b[axis]-a[axis])*t+normal[axis]*v for axis in (0,1)])
        stations.append(j)
    edge=fit_edge(coordinates,stations)
    edge[1]['appearance_method']='joint_energy_B_transition_after_G_cue'
    details=dict(cue=cue_audit,descriptor='energy',model_offset=offset,model_relative_slope=slope,
                 original_RGB_support=count,model_score=float(score),max_sample_models=45)
    if audit:details.update(measured_points=coordinates,station_ids=stations)
    return edge,details
