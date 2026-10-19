#!/usr/bin/env python3
"""Offline OpenCV analysis of raw grasp correspondences. Never controls hardware.

Fits are diagnostic: the FK tip midpoint is not a measured cube-center TCP.
Original images, feedback, intrinsics and robot calibration are not overwritten.
"""

import sys as _sys
from pathlib import Path as _Path
_sys.path.insert(0, str(_Path(__file__).resolve().parents[2]))
from host.project_paths import ROOT as REPO_ROOT, bootstrap as _bootstrap
_bootstrap(include_tools=True)
import csv
import itertools
import json
from pathlib import Path
import cv2
import numpy as np
from face_geometry import ordered_quad, face_candidates, pose_candidates

HERE = (REPO_ROOT/'data/k230')
ROOT = HERE/'grasp_pairs'
OUT = ROOT/'analysis_20261013'

def write(name, value):
    OUT.mkdir(parents=True,exist_ok=True)
    (OUT/name).write_text(json.dumps(value, ensure_ascii=False, indent=2)+'\n')




def background_pair(a,b,descriptors):
    ka,da = descriptors[a];kb,db = descriptors[b]
    if da is None or db is None: return {'pair':[a,b],'status':'insufficient_features'}
    matches = cv2.BFMatcher().knnMatch(da,db,k=2)
    matches = [m[0] for m in matches if len(m)==2 and m[0].distance<.65*m[1].distance]
    if len(matches)<8: return {'pair':[a,b],'status':'insufficient_matches','matches':len(matches)}
    p1 = np.array([ka[m.queryIdx].pt for m in matches]);p2 = np.array([kb[m.trainIdx].pt for m in matches])
    H,flags = cv2.findHomography(p1,p2,cv2.RANSAC,2)
    if H is None: return {'pair':[a,b],'status':'no_background_transform'}
    selected = flags.ravel().astype(bool);p1=p1[selected];p2=p2[selected]
    predicted = cv2.perspectiveTransform(p1.reshape(-1,1,2),H).reshape(-1,2)
    displacement = np.linalg.norm(p2-p1,axis=1)
    return {'pair':[a,b], 'status':'background_matches', 'inlier_count':len(p1),
            'inlier_reference_bbox_px':np.r_[p1.min(axis=0),p1.max(axis=0)].tolist(),
            'median_delta_xy_px':np.median(p2-p1,axis=0).tolist(),
            'median_displacement_px':float(np.median(displacement)),
            'p90_displacement_px':float(np.percentile(displacement,90)),
            'homography_reprojection_rms_px':float(np.sqrt(np.mean(np.sum((predicted-p2)**2,axis=1)))),
            'H_destination_from_source':H.tolist(),
            'interpretation':'Background displacement alone does not establish a change in the camera-to-robot-base relative pose.'}

def fit_affine(x,y):
    design = np.c_[x,np.ones(len(x))]
    coef,_,rank,_ = np.linalg.lstsq(design,y,rcond=None)
    if rank<3: raise ValueError('Collinear training positions')
    return coef

def rigid_fit(x,y):
    xc=x-x.mean(axis=0);yc=y-y.mean(axis=0)
    U,s,Vt=np.linalg.svd(xc.T@yc)
    R=Vt.T@U.T
    if np.linalg.det(R)<0:
        Vt[-1]*=-1;R=Vt.T@U.T
    t=y.mean(axis=0)-R@x.mean(axis=0)
    return R,t,s

def diagnostic_fits(rows, camera_pose_evidence=None):
    usable=[r for r in rows if r['vision']['status']=='complete_face_candidate' and r['vision'].get('poses')]
    if len(usable)<4:return {'status':'insufficient_complete_face_candidates','usable_groups':[r['group'] for r in usable]}
    ids=[r['group'] for r in usable]
    xy=np.array([r['fk']['tip_midpoint_Base_mm'][:2] for r in usable])
    rays=np.array([r['vision']['poses'][0]['cube_center_normalized_ray'] for r in usable])
    coef=fit_affine(rays,xy);prediction=np.c_[rays,np.ones(len(rays))]@coef
    errors=np.linalg.norm(prediction-xy,axis=1)
    loo=[]
    for i,row in enumerate(usable):
        train=np.arange(len(usable))!=i
        coeff=fit_affine(rays[train],xy[train]);p=np.r_[rays[i],1]@coeff
        loo.append({'test_group':row['group'],'training_groups':[n for j,n in enumerate(ids) if j!=i],
                    'predicted_fk_tip_reference_xy_mm':p.tolist(),'error_xy_mm':float(np.linalg.norm(p-xy[i]))})
    homography,_=cv2.findHomography(rays,xy,0)
    projected=cv2.perspectiveTransform(rays.reshape(-1,1,2),homography).reshape(-1,2)
    hlo=[]
    if len(usable)>=5:
        for i,row in enumerate(usable):
            train=np.arange(len(usable))!=i
            H,_=cv2.findHomography(rays[train],xy[train],0)
            p=cv2.perspectiveTransform(rays[i].reshape(1,1,2),H).reshape(2)
            hlo.append({'test_group':row['group'],'error_xy_mm':float(np.linalg.norm(p-xy[i]))})
    xyz=np.array([r['fk']['tip_midpoint_Base_mm'] for r in usable])
    camera=np.array([r['vision']['poses'][0]['cube_center_Camera_mm'] for r in usable])
    R,t,s=rigid_fit(camera,xyz);re=np.linalg.norm(camera@R.T+t-xyz,axis=1)
    return {'status':'diagnostic_only_not_deployment_calibration',
            'limitations':['FK labels are provisional gripper-tip reference, not measured cube center.',
                           'Side-face pose ambiguity retained; fits use the independently lowest-reprojection candidate.',
                           'No servo/calibration configuration has been changed.'],
            'camera_to_base_pose_evidence':camera_pose_evidence,
            'usable_groups':ids,
            'affine':{'coefficients_from_normalized_ray_to_tip_xy_mm':coef.tolist(),
                      'training_errors_mm':dict(zip(map(str,ids),errors.tolist())),
                      'training_rms_mm':float(np.sqrt(np.mean(errors**2))),
                      'leave_one_out':loo,
                      'loo_rms_mm':float(np.sqrt(np.mean([r['error_xy_mm']**2 for r in loo]))),
                      'loo_max_mm':max(r['error_xy_mm'] for r in loo)},
            'homography':{'H_normalized_ray_to_tip_xy_mm':homography.tolist(),
                          'training_errors_mm':dict(zip(map(str,ids),np.linalg.norm(projected-xy,axis=1).tolist())),
                          'leave_one_out':hlo},
            'rigid_3d':{'R_Base_from_Camera':R.tolist(),'translation_mm':t.tolist(),
                        'training_errors_mm':dict(zip(map(str,ids),re.tolist())),
                        'training_rms_mm':float(np.sqrt(np.mean(re**2))),
                        'cross_covariance_singular_values_mm2':s.tolist()},
            'gripper_reference_xy_range_mm':np.ptp(xy,axis=0).tolist(),
            'gripper_reference_z_range_mm':float(np.ptp(xyz[:,2]))}

def main():
    OUT.mkdir(parents=True,exist_ok=True)
    collection=json.loads((ROOT/'collection.json').read_text())
    fk_rows={r['group']:r for r in json.loads((OUT/'encoder_fk.json').read_text())}
    intr=json.loads((HERE/'calibration_results/camera_intrinsics.json').read_text())
    K=np.array(intr['camera_matrix'],float);K[0]*=2;K[1]*=2
    D=np.array(intr['distortion_coefficients'],float)
    rows=[];descriptors={};sift=cv2.SIFT_create(nfeatures=3000)
    for g in collection['groups']:
        im=cv2.imread(str(ROOT/g['folder']/'frame.jpg'))
        if im is None:raise RuntimeError('Missing image '+g['folder'])
        if im.shape[:2]!=(960,1280):raise RuntimeError('Unexpected capture resolution')
        vision=face_candidates(im,g['group'])
        if g['group']==1:
            confirmed=json.loads((ROOT/g['folder']/'face_candidate.json').read_text())
            vision.update(status='complete_face_candidate',face_quad_px=confirmed['face_quad_px'],
                          face_identity='side_face_confirmed_by_user')
        if vision['status']=='complete_face_candidate':
            vision['poses']=pose_candidates(np.array(vision['face_quad_px']),K,D)
            if len(vision['poses'])==2:
                vision['pose_ambiguity_center_separation_mm']=float(np.linalg.norm(np.array(vision['poses'][0]['cube_center_Camera_mm'])-vision['poses'][1]['cube_center_Camera_mm']))
        rows.append({'group':g['group'],'folder':g['folder'],'fk':fk_rows[g['group']],'vision':vision})
        gray=cv2.cvtColor(im,cv2.COLOR_BGR2GRAY)
        mask=np.zeros(gray.shape,np.uint8);mask[:400,:]=255
        descriptors[g['group']]=sift.detectAndCompute(gray,mask)
        annot=im.copy()
        if vision.get('face_quad_px'):
            q=np.rint(vision['face_quad_px']).astype(np.int32)
            cv2.polylines(annot,[q],True,(0,255,0),3)
            if vision.get('poses'):
                for i,p in enumerate(vision['poses']):
                    cv2.drawMarker(annot,tuple(np.rint(p['cube_center_pixel']).astype(int)),(0,255,255) if i==0 else (0,128,255),cv2.MARKER_CROSS,20,2)
        elif vision['variants']:
            q=np.array(vision['variants'][-1]['polygon_px'],np.int32)
            cv2.polylines(annot,[q],True,(0,0,255),3)
        cv2.putText(annot,'Group %d: %s'%(g['group'],vision['status']),(20,35),cv2.FONT_HERSHEY_SIMPLEX,.8,(0,0,0),4)
        cv2.putText(annot,'Group %d: %s'%(g['group'],vision['status']),(20,35),cv2.FONT_HERSHEY_SIMPLEX,.8,(255,255,255),2)
        cv2.imwrite(str(OUT/('group_%02d_annotated.jpg'%g['group'])),annot)
    background=[background_pair(a,b,descriptors) for a,b in itertools.combinations(sorted(descriptors),2)]
    evidence=collection.get('camera_to_base_pose_evidence')
    for item in background:
        item['camera_to_base_pose_evidence']=evidence
    fits=diagnostic_fits(rows,evidence)
    write('groups_analysis.json',rows);write('background_consistency.json',background);write('diagnostic_fits.json',fits)
    with (OUT/'group_summary.csv').open('w',newline='') as f:
        writer=csv.writer(f);writer.writerow(['group','image_status','encoder_max_span_counts','tip_x_mm','tip_y_mm','tip_z_mm','face_reprojection_rms_px','pose_center_ambiguity_mm'])
        for row in rows:
            v=row['vision'];writer.writerow([row['group'],v['status'],max(s['span_counts'] for s in row['fk']['feedback_summary']),*row['fk']['tip_midpoint_Base_mm'],v['poses'][0]['face_reprojection_rms_px'] if v.get('poses') else '',v.get('pose_ambiguity_center_separation_mm','')])
    for row in rows:
        v=row['vision'];print(json.dumps({'group':row['group'],'status':v['status'],'reason':v.get('reason'),'quad':v.get('face_quad_px'),'pose_rms':[p['face_reprojection_rms_px'] for p in v.get('poses',[])],'ambiguity_mm':v.get('pose_ambiguity_center_separation_mm')}))
    print('FITS',json.dumps(fits))
    print('BACKGROUND_ADJACENT',json.dumps([r for r in background if r['pair'][1]-r['pair'][0]==1]))
    print('OUTPUT',OUT)

if __name__=='__main__':main()
