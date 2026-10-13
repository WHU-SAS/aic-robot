#!/usr/bin/env python3
"""Compare frozen fixed-tip pairs under two intrinsics; no hardware access."""
import argparse
import hashlib
import json
from pathlib import Path
import shutil
import cv2
import numpy as np
from analyze_grasp_batch import pose_candidates

ROOT=Path(__file__).resolve().parents[1]
def load(p):return json.loads(p.read_text())
def save(p,value):p.write_text(json.dumps(value,ensure_ascii=False,indent=2)+'\n')
def sha(p):return hashlib.sha256(p.read_bytes()).hexdigest()

def fit(camera,refs,groups):
 x=np.array([camera[g] for g in groups]);y=np.array([refs[g] for g in groups]);xc=x-x.mean(0);yc=y-y.mean(0)
 if len(groups)<3 or np.linalg.matrix_rank(xc)<2 or np.linalg.matrix_rank(yc)<2:raise ValueError('Insufficient non-collinear pairs')
 U,s,Vt=np.linalg.svd(xc.T@yc);R=Vt.T@U.T
 if np.linalg.det(R)<0:Vt[-1]*=-1;R=Vt.T@U.T
 assert np.allclose(R.T@R,np.eye(3),atol=1e-10) and np.isclose(np.linalg.det(R),1)
 t=y.mean(0)-R@x.mean(0);T=np.eye(4);T[:3,:3]=R;T[:3,3]=t
 return T,s

def metrics(rows):
 d=np.array([r['delta_mm'] for r in rows])
 return {'groups':rows,'rms_xyz_mm':float(np.sqrt(np.mean(np.sum(d*d,axis=1)))),
 'rms_xy_mm':float(np.sqrt(np.mean(np.sum(d[:,:2]**2,axis=1)))),
 'rms_z_mm':float(np.sqrt(np.mean(d[:,2]**2))),
 'mean_delta_mm':d.mean(0).tolist(),'max_xyz_mm':float(np.linalg.norm(d,axis=1).max())}

def evaluate(camera,refs,T,groups):
 rows=[]
 for g in groups:
  pred=T[:3,:3]@camera[g]+T[:3,3];d=pred-refs[g]
  rows.append({'group':g,'predicted_Base_mm':pred.tolist(),'reference_Base_mm':refs[g].tolist(),
   'delta_mm':d.tolist(),'xy_mm':float(np.linalg.norm(d[:2])),'xyz_mm':float(np.linalg.norm(d))})
 return metrics(rows)

def analyze(camera,refs,ids,holdout):
 T,s=fit(camera,refs,ids);result={'full_fit':{**evaluate(camera,refs,T,ids),'T_Base_from_Camera_mm':T.tolist(),'singular_values':s.tolist()}}
 loo=[]
 for g in ids:
  tr=[n for n in ids if n!=g];A,_=fit(camera,refs,tr);row=evaluate(camera,refs,A,[g])['groups'][0];row['training_groups']=tr;loo.append(row)
 result['leave_one_out']=metrics(loo)
 tr=[g for g in ids if g not in holdout];test=[g for g in holdout if g in ids];A,_=fit(camera,refs,tr)
 result['fixed_split']={'training_groups':tr,'validation_groups':test,**evaluate(camera,refs,A,test),'T_Base_from_Camera_mm':A.tolist()}
 # Keep the earlier 8/9 test set unchanged too, with the same eight training pairs.
 tr=[g for g in ids if g not in [8,9]];A,_=fit(camera,refs,tr)
 result['same_holdout_8_9']={'training_groups':tr,**evaluate(camera,refs,A,[8,9])}
 return result

def main():
 p=argparse.ArgumentParser(description=__doc__);p.add_argument('--previous',type=Path,required=True);p.add_argument('--intrinsics',type=Path,required=True);p.add_argument('--output',type=Path,required=True);args=p.parse_args()
 previous=args.previous.resolve();source=args.intrinsics.resolve();out=args.output.resolve();out.mkdir(parents=True,exist_ok=False)
 m=load(previous/'input_manifest.json');a=load(previous/'analysis.json');v={r['group']:r for r in load(previous/'vision.json')};fk={r['group']:r for r in load(previous/'encoder_fk.json')};ids=a['usable_groups'];refs={g:np.array(fk[g]['cube_center_Base_mm']) for g in ids}
 if ids!=list(range(3,13)):raise ValueError('Unexpected sample set: review frozen input')
 protected=[ROOT/'k230/calibration_results/camera_intrinsics.json',ROOT/'k230/calibration_results/camera_in_base_provisional.json',ROOT/'hardware/user_selected_pid_demo_baseline.json',ROOT/'hardware/motion_defaults.json']
 unchanged={str(f):sha(f) for f in protected}
 for item in m['files']:
  assert sha(previous/item['snapshot'])==sha(Path(item['source']))==item['sha256']
 for item in m['model_hashes']:assert sha(Path(item['path']))==item['sha256']
 frozen_files=['input_manifest.json','analysis.json','vision.json','encoder_fk.json','fixed_point_definition.json','snapshot/intrinsics.json']
 previous_hashes={str(previous/f):sha(previous/f) for f in frozen_files}
 for name in ['encoder_fk.json','fixed_point_definition.json']:shutil.copy2(previous/name,out/name)
 shutil.copy2(previous/'snapshot/intrinsics.json',out/'old_intrinsics.json');shutil.copy2(source,out/'new_intrinsics.json');save(out/'frozen_vision_corners.json',list(v.values()))
 allposes={};results={};scaled={}
 for label,intr in [('old',load(out/'old_intrinsics.json')),('new',load(out/'new_intrinsics.json'))]:
  K=np.array(intr['camera_matrix'],float);D=np.array(intr['distortion_coefficients'],float);K[0]*=1280/intr['image_width'];K[1]*=960/intr['image_height'];scaled[label]={'K_at_1280x960':K.tolist(),'D':D.ravel().tolist()};camera={};poses=[]
  for group in m['groups']:
   item=v[group];row={'group':group,'status':item['status'],'used_for_comparison':group in ids,'face_quad_px':item.get('face_quad_px'),'boundary_method':item['boundary_method']}
   if item.get('face_quad_px'):
    candidates=pose_candidates(np.array(item['face_quad_px'],float),K,D);row['poses']=candidates
    if len(candidates)==2:row['pose_ambiguity_mm']=float(np.linalg.norm(np.array(candidates[0]['cube_center_Camera_mm'])-candidates[1]['cube_center_Camera_mm']))
    if group in ids:
     if not candidates:raise RuntimeError('Lost physical PnP solution for group '+str(group))
     camera[group]=np.array(candidates[0]['cube_center_Camera_mm'])
   poses.append(row)
  allposes[label]=poses;results[label]=analyze(camera,refs,ids,m['holdout_groups'])
 # Reproduction is a gate: same old image points, poses, fit and split.
 assert np.isclose(results['old']['fixed_split']['rms_xyz_mm'],a['fixed_split']['new']['rms_xyz_mm'],atol=1e-7)
 assert np.isclose(results['old']['leave_one_out']['rms_xyz_mm'],a['leave_one_out']['rms_xyz_mm'],atol=1e-7)
 report={'status':'paired_intrinsics_comparison_complete','requested_groups':m['groups'],'usable_groups':ids,'excluded_groups':[g for g in m['groups'] if g not in ids],
 'comparison_design':'Only intrinsics changed. Frozen image quadrilaterals, quality decisions, FK references and training/validation split unchanged; refit extrinsics independently for each intrinsics.',
 'pose_selection':'Lowest image reprojection IPPE solution independently of FK residual; both solutions retained.',
 'reference_definition':load(out/'fixed_point_definition.json'),'input_previous':str(previous),'new_intrinsics_source':str(source),'new_intrinsics_sha256':sha(source),'scaled_intrinsics':scaled,
 'results':results,'hardware_access':False,'defaults_updated':False,'opencv_version':cv2.__version__,
 'limitations':['Manual fixed-tip touch/top-center alignment and FK endpoint proxy are approximate, not surveyed ground truth.',
 'A 20mm square face yields two IPPE poses; choosing by image residual does not remove depth ambiguity.',
 'This evaluates correspondence consistency, not powered pick accuracy.',
 'Cube faces lie near lower image edge beyond part of the accepted new board-corner coverage.']}
 report['change_percent']={k:100*(results['old'][k]['rms_xyz_mm']-results['new'][k]['rms_xyz_mm'])/results['old'][k]['rms_xyz_mm'] for k in ['fixed_split','leave_one_out','same_holdout_8_9','full_fit']}
 save(out/'poses.json',allposes);save(out/'comparison.json',report)
 T=np.array(results['new']['full_fit']['T_Base_from_Camera_mm'])
 save(out/'camera_in_base_candidate.json',{'status':'current_focus_fixed_tip_candidate_not_adopted','T_Base_from_Camera_mm':T.tolist(),'T_Camera_from_Base_mm':np.linalg.inv(T).tolist(),'intrinsics_file':str(out/'new_intrinsics.json'),'intrinsics_sha256':sha(source),'source_groups':ids,'point_definition':report['reference_definition'],'validation':results['new'],'default_replaced':False})
 for f,h in {**unchanged,**previous_hashes}.items():assert sha(Path(f))==h
 for item in m['files']:assert sha(previous/item['snapshot'])==sha(Path(item['source']))==item['sha256']
 assert sha(source)==report['new_intrinsics_sha256']
 save(out/'verification.json',{'old_results_reproduced':True,'raw_and_snapshot_hashes_verified':len(m['files']),'protected_files':unchanged,'previous_analysis_hashes':previous_hashes,'source_manifest':m,'canonical_files_unchanged':True,'hardware_access':False})
 print(json.dumps({'usable_groups':ids,'old':{k:{x:results['old'][k][x] for x in ['rms_xyz_mm','rms_xy_mm','rms_z_mm','max_xyz_mm']} for k in ['fixed_split','leave_one_out','same_holdout_8_9','full_fit']},'new':{k:{x:results['new'][k][x] for x in ['rms_xyz_mm','rms_xy_mm','rms_z_mm','max_xyz_mm']} for k in ['fixed_split','leave_one_out','same_holdout_8_9','full_fit']},'change_percent':report['change_percent'],'old_results_reproduced':True},ensure_ascii=False))

if __name__=='__main__':main()
