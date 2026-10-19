#!/usr/bin/env python3
"""Offline current-focus ChArUco check; save candidates, never replace defaults."""

import sys as _sys
from pathlib import Path as _Path
_sys.path.insert(0, str(_Path(__file__).resolve().parents[2]))
from host.project_paths import ROOT as REPO_ROOT, bootstrap as _bootstrap
_bootstrap(include_tools=True)
import argparse
import hashlib
import json
from pathlib import Path
import shutil
import cv2
import numpy as np


def save(path,value):path.write_text(json.dumps(value,ensure_ascii=False,indent=2)+'\n')
def read(path):return json.loads(path.read_text())

def main():
 p=argparse.ArgumentParser(description=__doc__);p.add_argument('--images',type=Path,required=True);p.add_argument('--selection',type=Path,required=True);p.add_argument('--baseline',type=Path,default=REPO_ROOT/'configs/vision/runtime/camera_intrinsics.json');p.add_argument('--output',type=Path,required=True);args=p.parse_args()
 source=args.images.resolve();selection=read(args.selection);out=args.output.resolve();out.mkdir(parents=True,exist_ok=False)
 old_path=args.baseline.resolve();old=read(old_path);shutil.copy2(old_path,out/'previous_intrinsics.json');shutil.copy2(args.selection,out/'selection.json')
 board=cv2.aruco.CharucoBoard((5,7),28.54,14.27,cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_5X5_100));board.setLegacyPattern(False);detector=cv2.aruco.CharucoDetector(board)
 objects=[];points=[];names=[];size=None;hashes=[];corner_sets=[]
 for item in selection['images']:
  if not item['accepted']:continue
  path=source/item['file'];im=cv2.imread(str(path));assert im is not None
  dims=(im.shape[1],im.shape[0])
  if size is None:size=dims
  if dims!=size:raise RuntimeError('Mixed image sizes')
  corners,ids,_,_=detector.detectBoard(cv2.cvtColor(im,cv2.COLOR_BGR2GRAY));assert ids is not None and len(ids)==24
  ix=ids.ravel();order=np.argsort(ix);ix=ix[order];corners=corners.reshape(-1,2)[order]
  objects.append(board.getChessboardCorners()[ix].astype(np.float32));points.append(corners.astype(np.float32));names.append(path.name);corner_sets.append(corners)
  hashes.append({'file':path.name,'sha256':hashlib.sha256(path.read_bytes()).hexdigest()})
 if not names:save(out/'report.json',{'status':'no_complete_patterns','hardware_access':False});print('NO_COMPLETE_PATTERNS');return
 Kold=np.array(old['camera_matrix'],float);Kold[0]*=size[0]/old['image_width'];Kold[1]*=size[1]/old['image_height'];Dold=np.array(old['distortion_coefficients'],float)
 def evaluate(K,D,indices):
  result=[]
  for i in indices:
   ok,rv,tv=cv2.solvePnP(objects[i],points[i],K,D)
   if not ok:raise RuntimeError('Board pose solve failed')
   projected,_=cv2.projectPoints(objects[i],rv,tv,K,D);delta=projected.reshape(-1,2)-points[i]
   result.append({'file':names[i],'rms_px':float(np.sqrt(np.mean(np.sum(delta**2,axis=1)))),'mean_residual_xy_px':np.mean(delta,axis=0).tolist(),
     'residuals_xy_px':delta.tolist(),'board_tilt_deg':float(np.degrees(np.arccos(np.clip(abs(cv2.Rodrigues(rv)[0][2,2]),0,1))))})
  return {'rms_px':float(np.sqrt(np.mean([r['rms_px']**2 for r in result]))),'views':result}
 idx=list(range(len(names)));old_result=evaluate(Kold,Dold,idx)
 combined=np.concatenate(points);coverage={'corner_min_px':combined.min(0).tolist(),'corner_max_px':combined.max(0).tolist(),'hull_fraction':float(cv2.contourArea(cv2.convexHull(combined))/np.prod(size))}
 # Mark almost identical viewpoints; no automatic deletion based on residuals.
 diverse=[]
 for i,q in enumerate(corner_sets):
  if not diverse or min(float(np.sqrt(np.mean(np.sum((q-corner_sets[j])**2,axis=1)))) for j in diverse)>=10.:diverse.append(i)
 report={'status':'precheck_only','capture_session':source.name,'image_size':list(size),'complete_views':len(names),'complete_files':names,
 'old_intrinsics_on_new_images':old_result,'coverage':coverage,'viewpoint_diversity':{'corner_displacement_threshold_px':10,'representative_files':[names[i] for i in diverse]},
 'input_hashes':hashes,'hardware_access':False,'canonical_intrinsics_updated':False,'candidate_only':True,
 'limitations':['Held-out board pose is fitted per image; pixel RMS is not independent measured physical accuracy.',
 'Focus changed relative to old calibration; sample collection keeps current manual focus.',
 'Low pixel residual alone does not prove depth accuracy; pose/edge coverage and parameter stability also matter.']}
 if len(names)<12:
  report['status']='insufficient_complete_views_for_candidate';save(out/'report.json',report);print('INSUFFICIENT',len(names),'OLD_RMS',old_result['rms_px']);return
 fit=cv2.calibrateCameraExtended(objects,points,size,None,None);rms,K,D,rv,tv,std_intr,std_extr,perview=fit
 if not np.isfinite(K).all() or not np.isfinite(D).all():raise RuntimeError('Nonfinite calibration')
 normal=[]
 for rot in rv:
  n=cv2.Rodrigues(rot)[0][:,2]
  if n[2]<0:n=-n
  normal.append(n)
 normal=np.array(normal);coverage['normal_x_range']=[float(normal[:,0].min()),float(normal[:,0].max())];coverage['normal_y_range']=[float(normal[:,1].min()),float(normal[:,1].max())]
 coverage['tilt_deg_range']=[float(np.degrees(np.arccos(np.clip(normal[:,2],0,1))).min()),float(np.degrees(np.arccos(np.clip(normal[:,2],0,1))).max())]
 candidate={'status':'current_focus_candidate_not_adopted','image_width':size[0],'image_height':size[1],'camera_matrix':K.tolist(),'distortion_coefficients':D.ravel().tolist(),
 'rms_reprojection_error_px':float(rms),'intrinsic_standard_deviations':std_intr.ravel().tolist(),'source_session':source.name,
 'accepted_views':[{'file':n,'reprojection_rms_px':float(e)} for n,e in zip(names,perview.ravel())],
 'board':{'squares_x':5,'squares_y':7,'square_mm_measured':28.54,'marker_mm_assumed':14.27,'dictionary':'DICT_5X5_100'},
 'capture_settings':read(source/'session.json'),'applicability':'current manual lens focus, source sensor mode, RGB565, hmirror=False, vflip=False',
 'source_hashes':hashes,'default_replaced':False}
 folds=[]
 for withheld in np.array_split(np.arange(len(names)),3):
  test=list(map(int,withheld));train=[i for i in idx if i not in test]
  fr,fk,fd,*_=cv2.calibrateCamera([objects[i] for i in train],[points[i] for i in train],size,None,None)
  folds.append({'training_files':[names[i] for i in train],'test_files':[names[i] for i in test],
    'candidate_training_rms_px':float(fr),'candidate_K':fk.tolist(),'candidate_D':fd.ravel().tolist(),
    'old':evaluate(Kold,Dold,test),'candidate':evaluate(fk,fd,test)})
 old_cv=float(np.sqrt(np.mean([r['rms_px']**2 for f in folds for r in f['old']['views']])))
 new_cv=float(np.sqrt(np.mean([r['rms_px']**2 for f in folds for r in f['candidate']['views']])))
 report.update(status='candidate_and_blocked_validation_complete',candidate_training_rms_px=float(rms),candidate_K=K.tolist(),candidate_D=D.ravel().tolist(),
  old_K_scaled=Kold.tolist(),intrinsic_standard_deviations=std_intr.ravel().tolist(),
  focal_change_vs_old_percent={'fx':float((K[0,0]/Kold[0,0]-1)*100),'fy':float((K[1,1]/Kold[1,1]-1)*100)},
  principal_point_change_px={'cx':float(K[0,2]-Kold[0,2]),'cy':float(K[1,2]-Kold[1,2])},
  blocked_validation={'old_rms_px':old_cv,'candidate_rms_px':new_cv,'folds':folds,'description':'Three chronological blocks; each block withheld while the others fit candidate intrinsics.'},
  maximum_focal_change_across_folds_percent=float(max(max(abs(f['candidate_K'][0][0]/K[0,0]-1),abs(f['candidate_K'][1][1]/K[1,1]-1)) for f in folds)*100))
 save(out/'camera_intrinsics_candidate.json',candidate);save(out/'report.json',report)
 print(json.dumps({k:report[k] for k in ['complete_views','candidate_training_rms_px','focal_change_vs_old_percent','principal_point_change_px','maximum_focal_change_across_folds_percent','coverage','blocked_validation'] if k!='blocked_validation'},ensure_ascii=False));print('VALIDATION_OLD',old_cv,'CANDIDATE',new_cv)
 # The current runtime file is never modified by this script.
 assert read(old_path)==old

if __name__=='__main__':main()
