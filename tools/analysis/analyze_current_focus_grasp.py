#!/usr/bin/env python3
"""Freeze and analyze current-focus action-1 pairs; camera/robot ports are never opened."""

import sys as _sys
from pathlib import Path as _Path
_sys.path.insert(0, str(_Path(__file__).resolve().parents[2]))
from host.project_paths import ROOT as REPO_ROOT, bootstrap as _bootstrap
_bootstrap(include_tools=True)
import argparse,hashlib,json,shutil,subprocess,sys
from pathlib import Path
ROOT=REPO_ROOT
def load(p):return json.loads(p.read_text())
def save(p,d):p.write_text(json.dumps(d,ensure_ascii=False,indent=2)+'\n')
def sha(p):return hashlib.sha256(p.read_bytes()).hexdigest()

def vision_and_fit(out):
 import cv2,numpy as np
 from face_geometry import face_candidates,pose_candidates
 from grasp_face_quality import assess
 from compare_fixed_top_intrinsics import fit,evaluate,metrics
 m=load(out/'input_manifest.json');groups=m['new_groups'];intr=load(out/'snapshot/current_intrinsics.json');old=load(out/'snapshot/intrinsics.json')
 def matrices(i):
  K=np.array(i['camera_matrix'],float);K[0]*=1280/i['image_width'];K[1]*=960/i['image_height'];return K,np.array(i['distortion_coefficients'],float)
 K,D=matrices(intr);Ko,Do=matrices(old);rows=[];tiles=[]
 for g in groups:
  im=cv2.imread(str(out/'snapshot'/f'group_{g:02d}'/'frame.jpg'));assert im is not None and im.shape[:2]==(960,1280)
  r=face_candidates(im,g);r['boundary_method']='existing_threshold_median';ok=False
  if r['status']=='complete_face_candidate':
   ps=pose_candidates(np.array(r['face_quad_px']),K,D);ok=bool(ps and ps[0]['face_reprojection_rms_px']<=2 and np.array(r['face_quad_px'])[:,1].min()>=432)
   if ok:r['poses']=ps
  if not ok:r=assess(im,K,D);r['boundary_method']='dim_blue_line_refinement'
  r['group']=g
  if r.get('face_quad_px'):
   q=np.array(r['face_quad_px']);r['old_intrinsics_poses_same_corners']=pose_candidates(q,Ko,Do)
   cv2.polylines(im,[np.rint(q).astype(np.int32)],True,(0,255,0) if r['status']=='complete_face_candidate' else (0,128,255),3)
  if len(r.get('poses',[]))==2:r['pose_ambiguity_mm']=float(np.linalg.norm(np.array(r['poses'][0]['cube_center_Camera_mm'])-r['poses'][1]['cube_center_Camera_mm']))
  cv2.imwrite(str(out/f'group_{g:02d}_annotated.jpg'),im);tile=cv2.resize(im,(480,360));cv2.putText(tile,f"Group {g}: {r['status']}",(6,25),cv2.FONT_HERSHEY_SIMPLEX,.45,(255,255,255),2);tiles.append(tile);rows.append(r)
  print('QUALITY',g,r['status'],[round(p['face_reprojection_rms_px'],3) for p in r.get('poses',[])],flush=True)
 while len(tiles)%3:tiles.append(np.zeros_like(tiles[0]))
 cv2.imwrite(str(out/'contact_sheet.jpg'),np.vstack([np.hstack(tiles[i:i+3]) for i in range(0,len(tiles),3)]));save(out/'vision.json',rows)
 ids=[r['group'] for r in rows if r['status']=='complete_face_candidate' and r.get('poses')];fk={r['group']:r for r in load(out/'encoder_fk.json')};refs={g:np.array(fk[g]['cube_center_Base_mm']) for g in ids};test=[g for g in m['holdout_new_groups'] if g in ids];train=[g for g in ids if g not in m['holdout_new_groups']]
 result={'requested_groups':groups,'usable_groups':ids,'training_groups':train,'validation_groups':test,'predeclared_holdout_groups':m['holdout_new_groups'],
 'protocol':'action1: modeled fingertip midpoint approximates cube bottom center; cube center = midpoint + BaseZ10mm',
 'focus_provenance':'User supplemented four pairs in response to keeping current focus; focus not electronically recorded.',
 'quality_selection':'Existing image-only screen with current K; no filtering by FK residual; both K variants reuse accepted points and the exact image quadrilaterals.',
 'feedback':[{'group':g,'encoder_spans':fk[g]['encoder_spans'],'max_speed_raw':max(abs(x) for s in fk[g]['speeds'] for x in s)} for g in groups],
 'hardware_access':False,'defaults_updated':False,'results':{},'limitations':['FK midpoint and manual bottom-center alignment are approximate, not surveyed ground truth.',
 'Single-face IPPE two-pose ambiguity retained; lowest image error selected independently of FK.',
 'Current intrinsics have less board-corner coverage near image bottom.','No powered grasp accuracy measured.']}
 if len(ids)<3:
  result['status']='insufficient_current_focus_pairs';save(out/'analysis.json',result);return
 for label,key in [('old','old_intrinsics_poses_same_corners'),('new','poses')]:
  cam={r['group']:np.array(r[key][0]['cube_center_Camera_mm']) for r in rows if r['group'] in ids};A,s=fit(cam,refs,ids);d={'full_fit':{**evaluate(cam,refs,A,ids),'T_Base_from_Camera_mm':A.tolist(),'singular_values':s.tolist()}}
  if len(ids)>=4:
   loo=[]
   for g in ids:
    tr=[n for n in ids if n!=g];B,_=fit(cam,refs,tr);row=evaluate(cam,refs,B,[g])['groups'][0];row['training_groups']=tr;loo.append(row)
   d['leave_one_out']=metrics(loo)
  if len(train)>=3 and test:
   B,sb=fit(cam,refs,train);d['fixed_split']={**evaluate(cam,refs,B,test),'T_Base_from_Camera_mm':B.tolist(),'singular_values':sb.tolist()}
  result['results'][label]=d
 point=np.array([refs[g] for g in ids]);sv=np.linalg.svd(point-point.mean(0),compute_uv=False);result['reference_coverage_mm']={'min':point.min(0).tolist(),'max':point.max(0).tolist(),'range':np.ptp(point,axis=0).tolist(),'centered_singular_values':sv.tolist()}
 result['status']='current_focus_pairs_analyzed';save(out/'analysis.json',result)
 A=np.array(result['results']['new']['full_fit']['T_Base_from_Camera_mm'])
 save(out/'camera_in_base_candidate.json',{'status':'current_focus_action1_candidate_not_adopted','source_namespace':'grasp_pairs','source_groups':ids,'intrinsics_file':str(out/'snapshot/current_intrinsics.json'),'intrinsics_sha256':sha(out/'snapshot/current_intrinsics.json'),
 'T_Base_from_Camera_mm':A.tolist(),'T_Camera_from_Base_mm':np.linalg.inv(A).tolist(),'reference':result['protocol'],'validation':result['results']['new'],'default_replaced':False})
 print('METRICS',json.dumps({'usable':ids,'train':train,'test':test,'results':{label:{method:{k:r[k] for k in ['rms_xyz_mm','rms_xy_mm','rms_z_mm','max_xyz_mm']} for method,r in res.items()} for label,res in result['results'].items()},'coverage':result['reference_coverage_mm']},ensure_ascii=False),flush=True)

def main():
 p=argparse.ArgumentParser(description=__doc__);p.add_argument('--groups',type=int,nargs='+');p.add_argument('--holdout-groups',type=int,nargs='+');p.add_argument('--intrinsics',type=Path);p.add_argument('--output',type=Path,required=True);p.add_argument('--vision-phase',action='store_true');args=p.parse_args();out=args.output.resolve()
 if args.vision_phase:vision_and_fit(out);return
 if not args.groups or not args.holdout_groups or not args.intrinsics:p.error('groups, holdout-groups and intrinsics required')
 if len(set(args.groups))!=len(args.groups) or not set(args.holdout_groups)<=set(args.groups):p.error('Invalid groups')
 import analyze_calibration_comparison as core
 core.OLD_GROUPS=[];core.NEW_GROUPS=args.groups;core.HOLDOUT_NEW_GROUPS=args.holdout_groups
 protected=[ROOT/'data/k230/calibration_results/camera_intrinsics.json',ROOT/'data/k230/calibration_results/camera_in_base_provisional.json',ROOT/'configs/arm/user_selected_pid_demo_baseline.json',ROOT/'configs/arm/motion_defaults.json'];hashes={str(f):sha(f) for f in protected}
 core.freeze(out);core.fk(out);source=args.intrinsics.resolve();target=out/'snapshot/current_intrinsics.json';h=sha(source);shutil.copy2(source,target);assert sha(target)==h
 m=load(out/'input_manifest.json');m['files'].append({'source':str(source),'snapshot':str(target.relative_to(out)),'sha256':h});m['sample_namespace']='grasp_pairs';m['reference_definition']='User midpoint = cube bottom center; BaseZ+10mm';save(out/'input_manifest.json',m)
 subprocess.run([str(ROOT/'scripts/vision'),str(Path(__file__).resolve()),'--output',str(out),'--vision-phase'],cwd=ROOT,check=True)
 for item in m['files']:assert sha(out/item['snapshot'])==sha(Path(item['source']))==item['sha256']
 for item in m['model_hashes']:assert sha(Path(item['path']))==item['sha256']
 for f,h in hashes.items():assert sha(Path(f))==h
 save(out/'verification.json',{'raw_source_snapshot_files_verified':len(m['files']),'model_hashes_verified':len(m['model_hashes']),'protected_file_hashes':hashes,'canonical_files_unchanged':True,'hardware_access':False})
 print('RAW_INPUTS_MODELS_DEFAULTS_UNCHANGED',flush=True)

if __name__=='__main__':main()
