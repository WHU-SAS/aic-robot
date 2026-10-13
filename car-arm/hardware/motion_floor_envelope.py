"""Conservative continuous floor bound for independent joint count intervals."""
import numpy as np
from gripper_geometry import LINK_MESHES,mesh_floor_clearance


def certify_floor(fk,mapping,start,target,cache,check_cancel,max_cells=256,finger_floor_m=.03):
    if not np.isfinite(finger_floor_m) or not .01 <= finger_floor_m <= .03:
        raise ValueError('finger model floor must be between 10 and 30 mm')
    thresholds={link:(finger_floor_m if link in ('wan2','zhua') else (.03 if link=='wan' else .02)) for link in LINK_MESHES}
    # Geometry uncertainty of +/-12 counts; this is not a runtime abort threshold.
    lower=np.minimum(start,target).astype(float)-12
    upper=np.maximum(start,target).astype(float)+12
    names=[mapping.joint(i)['urdf_joint_name'] for i in range(1,7)]
    slopes_signed=np.array([mapping.count_to_radians(i+1,2049)-mapping.count_to_radians(i+1,2048) for i in range(6)])
    zeros=np.array([mapping.joint(i+1)['zero_count'] for i in range(6)])
    mid=(lower+upper)/2
    angles={n:float((mid[i]-zeros[i])*slopes_signed[i]) for i,n in enumerate(names)}
    check_cancel();mesh_floor_clearance(fk,angles,cache)
    radii={}
    for link,filename in LINK_MESHES.items():
        _,_,chain=fk.forward_kinematics(angles,target_link=link)
        radius=float(np.linalg.norm(cache[filename],axis=1).max())
        r={}
        for name in reversed(chain):
            j=fk.joints[name]
            if j['type'] not in ('revolute','continuous','fixed'):
                raise ValueError('floor certificate requires fixed/revolute joints')
            if name in names:r[name]=radius
            radius+=float(np.linalg.norm(j['xyz']))
        radii[link]=np.array([r.get(n,0.) for n in names])
    # Rotation of a root joint about world Z cannot lower a mesh point.
    # Bound its tiny nonvertical component rather than full spatial radius.
    # Non-root axes keep the original bound since parent rotations may tilt them.
    for name in names:
        joint=fk.joints[name]
        if joint['parent']==fk.root_link and joint['type'] in ('revolute','continuous'):
            transform,_,_=fk.forward_kinematics(angles,target_link=joint['child'])
            axis=transform[:3,:3] @ joint['axis']
            axis=axis/np.linalg.norm(axis)
            horizontal=float(np.linalg.norm(axis[:2]))
            if horizontal<1e-10:
                index=names.index(name)
                for radius in radii.values():radius[index]*=horizontal
    slopes=np.abs(slopes_signed)
    pending=[(lower,upper)];minimum={};visited=0
    while pending:
        check_cancel();visited+=1
        if visited>max_cells:raise ValueError('floor envelope could not be certified within bounded cell budget')
        lo,hi=pending.pop();mid=(lo+hi)/2
        angles={n:float((mid[i]-zeros[i])*slopes_signed[i]) for i,n in enumerate(names)}
        center=mesh_floor_clearance(fk,angles,cache)
        # Telescoping one-joint rotations displace any vertex by <= radius*|delta|.
        bounds={link:z-float(np.dot(radii[link],slopes*(hi-lo)/2)) for link,z in center.items()}
        safe=all(z>=thresholds[link] for link,z in bounds.items())
        if safe:
            for link,z in bounds.items():minimum[link]=min(minimum.get(link,float('inf')),z)
            continue
        if any(z<thresholds[link] for link,z in center.items()):
            raise ValueError('route fails model floor clearance')
        scores=np.max(np.array(list(radii.values())),axis=0)*slopes*(hi-lo)
        axis=int(np.argmax(scores))
        if scores[axis]<1e-8:raise ValueError('floor envelope cannot be certified')
        hi_left=hi.copy();hi_left[axis]=mid[axis]
        lo_right=lo.copy();lo_right[axis]=mid[axis]
        pending.extend(((lo,hi_left),(lo_right,hi)))
    return minimum,visited
