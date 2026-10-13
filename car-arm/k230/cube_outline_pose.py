"""Joint known-20mm-cube pose from six measured image boundary lines.
The six-edge silhouette is a 3D cube, never interpreted as one square plane.
Uses K230 coordinates only; no image pixels, hardware, or table-height prior.
"""
from itertools import combinations, product
import cv2
import numpy as np

BASE_VERTICES=np.array([[0,0,0],[0,20,0],[20,20,0],[20,20,20],[20,0,20],[0,0,20]],dtype=float)
ALL_VERTICES=np.array([[x,y,z] for x in [0,20] for y in [0,20] for z in [0,20]],dtype=float)


def aligned_outline(points,reference):
    points=np.asarray(points,float);reference=np.asarray(reference,float)
    choices=[np.roll(p,i,axis=0) for p in [points,points[::-1]] for i in range(6)]
    return min(choices,key=lambda q:float(np.max(np.linalg.norm(q-reference,axis=1))))


def fit_cube_lines(corners,K,D):
    q=np.asarray(corners,float);target=[]
    for a,b in zip(q,np.roll(q,-1,axis=0)):
        dx,dy=b-a;line=np.array([-dy,dx,dy*a[0]-dx*a[1]],float);line/=np.linalg.norm(line[:2]);target.extend([line]*3)
    target=np.array(target);solutions=[]
    for parity in [0,1]:
        vertices=BASE_VERTICES.copy()
        if parity:vertices[:,0]=20-vertices[:,0]
        ok,rv,tv=cv2.solvePnP(vertices,q,K,D,flags=cv2.SOLVEPNP_EPNP)
        if not ok:continue
        ok,rv,tv=cv2.solvePnP(vertices,q,K,D,rv,tv,True,flags=cv2.SOLVEPNP_ITERATIVE)
        if not ok:continue
        parameters=np.r_[rv.ravel(),tv.ravel()]
        samples=np.concatenate([[(1-t)*vertices[i]+t*vertices[(i+1)%6] for t in [.25,.5,.75]] for i in range(6)])
        def evaluate(p):
            projected,jac=cv2.projectPoints(samples,p[:3],p[3:],K,D);projected=projected.reshape(-1,2)
            residual=(projected*target[:,:2]).sum(axis=1)+target[:,2]
            jacobian=target[:,0,None]*jac[::2,:6]+target[:,1,None]*jac[1::2,:6]
            return residual,jacobian
        damping=.001
        for iteration in range(80):
            residual,jac=evaluate(parameters);normal=jac.T@jac
            try:delta=np.linalg.solve(normal+damping*np.diag(np.maximum(1,np.diag(normal))),-jac.T@residual)
            except np.linalg.LinAlgError:break
            trial=parameters+delta;next_residual,_=evaluate(trial)
            if next_residual@next_residual<residual@residual:
                parameters=trial;damping=max(1e-8,damping/3)
            else:damping=min(1e12,damping*5)
            if np.linalg.norm(delta)<1e-8:break
        residual,_=evaluate(parameters);rotation=cv2.Rodrigues(parameters[:3])[0]
        depths=(rotation@ALL_VERTICES.T+parameters[3:,None])[2]
        if not np.isfinite(parameters).all() or depths.min()<=0:continue
        projected=cv2.projectPoints(vertices,parameters[:3],parameters[3:],K,D)[0].reshape(-1,2)
        projected_all=cv2.projectPoints(ALL_VERTICES,parameters[:3],parameters[3:],K,D)[0].reshape(-1,2)
        hull=cv2.convexHull(projected.astype(np.float32))
        # Both omitted vertices must project inside the cube's own silhouette.
        if any(cv2.pointPolygonTest(hull,tuple(p),True)<-.5 for p in projected_all):continue
        edge_rms=float(np.sqrt(np.mean(residual**2)));edge_max=float(np.max(np.abs(residual)))
        corner_rms=float(np.sqrt(np.mean(np.sum((projected-q)**2,axis=1))))
        center=rotation@np.array([10.,10.,10.])+parameters[3:]
        solutions.append({'edge_fit_rms_px':edge_rms,'edge_fit_max_px':edge_max,
            'corner_reprojection_rms_px':corner_rms,'projected_outline_px':projected.tolist(),
            'cube_center_Camera_mm':center.tolist(),'rotation_Camera_from_Cube':rotation.tolist(),
            'translation_Camera_from_Cube_mm':parameters[3:].tolist(),'layout_parity':parity,
            'edge_size_mm':20,'fit_iterations':iteration+1})
    return sorted(solutions,key=lambda s:s['edge_fit_rms_px'])


def outline_records(frame):
    records=frame.get('cube_outlines',[])
    if records:return records
    # Replay old logs: these corners were emitted only after all six outer
    # lines passed the original onboard <=3px straightness check.
    diagnostics=[frame.get('three_face_diagnostic',{})]+[a.get('three_face_diagnostic',{}) for a in frame.get('component_attempts',[])]
    found=[]
    for d in diagnostics:
        parts=[d]+[d[k] for k in ['original_orientation','opposite_orientation'] if k in d]
        evidence=max([sum(bool(e) and e.get('support',0)>=12 and e.get('rms_px',99)<=3 for e in part.get('internal_quality',[])) for part in parts]+[0])
        if evidence<1:continue
        for part in parts:
            q=part.get('outer_corners_px')
            if q and len(q)==6 and not any(np.max(np.abs(np.asarray(q)-f['corners_px']))<.01 for f in found):
                found.append({'corners_px':q,'verified_internal_edges':evidence,'legacy_diagnostic_replay':True})
                break
    return found


def localize_cube_outlines(frames,K,D,T):
    prepared=[];diagnostic={'method':'known20mm cube, six physical image edge lines','frames':[],'triplets':[]}
    for frame_index,frame in enumerate(frames):
        usable=[]
        for record in outline_records(frame):
            q=np.asarray(record.get('corners_px'),float)
            if q.shape!=(6,2) or not np.isfinite(q).all() or (q[:,0]<=2).any() or (q[:,0]>=1278).any() or (q[:,1]<432).any() or (q[:,1]>=958).any():continue
            if record.get('verified_internal_edges',0)<1:continue
            if not record.get('legacy_diagnostic_replay'):
                edges=record.get('edges',[])
                if len(edges)!=6 or any(e.get('support',0)<6 or e.get('rms_px',99)>3 for e in edges):continue
            if not cv2.isContourConvex(q.astype(np.float32)) or abs(cv2.contourArea(q.astype(np.float32)))<6000:continue
            usable.append(q)
        prepared.append(usable);diagnostic['frames'].append({'frame_index':frame_index,'outline_count':len(usable)})
    accepted=[];seen=set()
    for current_index,records in enumerate(prepared):
        for q in records:
            previous=[]
            for i,old in enumerate(prepared[:current_index]):
                aligned=[aligned_outline(p,q) for p in old];near=[p for p in aligned if np.max(np.linalg.norm(p-q,axis=1))<=5]
                if near:previous.append((i,near))
            # Nearby contour alternatives can differ geometrically even when
            # their vertex distance is smaller. Validate every matching pair
            # across two distinct frames, rather than greedily keeping one.
            pairs=[((a[0],p),(b[0],r)) for a,b in combinations(previous,2) for p,r in product(a[1],b[1])]
            for pair in pairs:
                median=np.median([q,pair[0][1],pair[1][1]],axis=0);key=tuple(median.ravel())
                if key in seen:continue
                seen.add(key);indices=[pair[0][0],pair[1][0],current_index]
                try:solutions=fit_cube_lines(median,K,D)
                except (cv2.error,np.linalg.LinAlgError,ValueError):solutions=[]
                diagnostic['triplets'].append({'selected_frame_indices':indices,'corners_px':median.tolist(),
                    'solutions':[{'edge_fit_rms_px':s['edge_fit_rms_px'],'edge_fit_max_px':s['edge_fit_max_px'],
                        'corner_reprojection_rms_px':s['corner_reprojection_rms_px'],'layout_parity':s['layout_parity']} for s in solutions]})
                for s in solutions:
                    # The metric is line-normal image residual, explicitly
                    # distinct from the original single-face point PnP RMS.
                    if s['edge_fit_rms_px']>2 or s['edge_fit_max_px']>4:continue
                    center=T[:3,:3]@np.asarray(s['cube_center_Camera_mm'])+T[:3,3]
                    accepted.append({'detector':'cube_outline','kind':'cube','pose':s,
                        'center_Base_mm':center.tolist(),'outline_px':median.tolist(),
                        'selected_frame_indices':indices,'area_px':abs(float(cv2.contourArea(median.astype(np.float32))))})
    if not accepted:diagnostic['reason']='No three-frame whole-cube fit with line RMS<=2px and max<=4px';return None,diagnostic
    chosen=min(accepted,key=lambda s:s['pose']['edge_fit_rms_px']);diagnostic['selected']=chosen
    return chosen,diagnostic
