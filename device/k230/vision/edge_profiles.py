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
