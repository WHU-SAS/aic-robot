#!/usr/bin/env python3
"""Offline action-1 comparison: old captures keep old K; refocused captures use new K."""

import sys as _sys
from pathlib import Path as _Path
_sys.path.insert(0, str(_Path(__file__).resolve().parents[2]))
from host.project_paths import ROOT as REPO_ROOT, bootstrap as _bootstrap
_bootstrap(include_tools=True)
import argparse,hashlib,json,shutil
from pathlib import Path
import numpy as np
from face_geometry import pose_candidates
from compare_fixed_top_intrinsics import fit,evaluate,metrics
ROOT=REPO_ROOT
def load(p):return json.loads(p.read_text())
def save(p,v):p.write_text(json.dumps(v,ensure_ascii=False,indent=2)+'\n')
def sha(p):return hashlib.sha256(p.read_bytes()).hexdigest()

def main():
 p=argparse.ArgumentParser(description=__doc__);p.add_argument('--previous',type=Path,required=True);p.add_argument('--intrinsics',type=Path,required=True);p.add_argument('--quality',type=Path,required=True);p.add_argument('--output',type=Path,required=True);args=p.parse_args()
 prev=args.previous.resolve();out=args.output.resolve();out.mkdir(parents=True,exist_ok=False)
 m=load(prev/'input_manifest.json');a=load(prev/'comparison.json');vision={r['group']:r for r in load(prev/'vision.json')['rows']};fk={r['group']:r for r in load(prev/'encoder_fk.json')};fresh={r['group']:r for r in load(args.quality)};ids=a['usable_groups'];oldids=a['old_groups'];newids=a['usable_new_groups'];train=a['fixed_split_comparison']['expanded_training_groups'];tests=a['fixed_split_comparison']['validation_groups']
 assert ids==[1,2,4,5,8,9,22,24];assert [g for g in m['new_groups'] if fresh[g]['status']=='complete_face_candidate']==newids
 verify={str(prev/f):sha(prev/f) for f in ['input_manifest.json','comparison.json','vision.json','encoder_fk.json']}
 protected=[ROOT/'data/k230/calibration_results/camera_intrinsics.json',ROOT/'data/k230/calibration_results/camera_in_base_provisional.json',ROOT/'configs/arm/user_selected_pid_demo_baseline.json',ROOT/'configs/arm/motion_defaults.json',args.intrinsics.resolve()]
 verify.update({str(f):sha(f) for f in protected})
 for item in m['files']:assert sha(prev/item['snapshot'])==sha(Path(item['source']))==item['sha256']
 for item in m['model_hashes']:assert sha(Path(item['path']))==item['sha256']
 intr=load(args.intrinsics);K=np.array(intr['camera_matrix']);K[0]*=1280/intr['image_width'];K[1]*=960/intr['image_height'];D=np.array(intr['distortion_coefficients']);refs={g:np.array(fk[g]['cube_center_Base_mm']) for g in ids};oldcam={g:np.array(vision[g]['poses'][0]['cube_center_Camera_mm']) for g in ids};newcam=dict(oldcam);newposes=[]
 for g in newids:
  quad=np.array(vision[g]['face_quad_px']);assert np.allclose(quad,np.array(fresh[g]['face_quad_px']),atol=1e-7),'Boundary changed unexpectedly'
  ps=pose_candidates(quad,K,D);assert ps
  newcam[g]=np.array(ps[0]['cube_center_Camera_mm']);newposes.append({'group':g,'face_quad_px':quad.tolist(),'poses':ps,'old_intrinsics_poses':vision[g]['poses']})
 def measure(cam):
  T,_=fit(cam,refs,train);A,_=fit(cam,refs,ids);original,_=fit(cam,refs,oldids);loo=[]
  for g in newids:
   tr=[n for n in ids if n!=g];B,_=fit(cam,refs,tr);r=evaluate(cam,refs,B,[g])['groups'][0];r['training_groups']=tr;loo.append(r)
  return {'same_holdout':{'training_groups':train,'validation_groups':tests,**evaluate(cam,refs,T,tests)},
   'old_8_9_holdout':evaluate(cam,refs,T,[8,9]),'new_24_holdout':evaluate(cam,refs,T,[24]),
   'old_six_predict_new':evaluate(cam,refs,original,newids),'leave_one_new_out':metrics(loo),
   'full_fit':{**evaluate(cam,refs,A,ids),'T_Base_from_Camera_mm':A.tolist(),'T_Camera_from_Base_mm':np.linalg.inv(A).tolist()}}
 old=measure(oldcam);new=measure(newcam)
 assert np.isclose(old['same_holdout']['rms_xyz_mm'],a['fixed_split_comparison']['expanded']['rms_xyz_mm'],atol=1e-7)
 assert np.isclose(old['old_six_predict_new']['rms_xyz_mm'],a['old_six_predictions_on_new']['rms_xyz_mm'],atol=1e-7)
 record={'status':'current_focus_action1_reanalysis_complete','protocol':'two fingertip endpoint midpoint ~= cube bottom center, Base +Z 10mm',
 'requested_new_groups':m['new_groups'],'usable_new_groups':newids,'usable_all_groups':ids,
 'intrinsics_by_capture':'old groups 1,2,4,5,8,9 keep old pre-refocus intrinsics; groups19..24 use current-focus intrinsics',
 'comparison_design':'same frozen image corners/FK/accepted pairs/holdout split; only refocused captures change intrinsics; refit rigid extrinsics',
 'camera_base_pose_assumption':'User confirmed mounting unchanged; optical-center changes from lens refocus are not independently measured.',
 'old':old,'new':new,'quality':list(fresh.values()),'hardware_access':False,'defaults_updated':False,'source_previous':str(prev),'new_intrinsics_source':str(args.intrinsics.resolve()),
 'limitations':['Only two newly usable current-focus pairs cannot independently fit a rigid external calibration requiring three non-collinear pairs.',
 'Candidate fit combines pre-focus and current-focus data with respective intrinsics, assuming the same Camera coordinate datum after refocus.',
 'Physical reference is modeled fingertip midpoint plus10mm, not surveyed cube center; no powered grasp accuracy measured.',
 'Current-focus cube faces near image bottom exceed part of board corner coverage; residual depth distortion remains possible.',
 'Current and fixed-tip contact protocols are reported separately, not pooled.']}
 for f in ['encoder_fk.json','snapshot/collection.json']:shutil.copy2(prev/f,out/Path(f).name)
 shutil.copy2(prev/'snapshot/intrinsics.json',out/'old_intrinsics.json');shutil.copy2(args.intrinsics,out/'new_intrinsics.json');save(out/'poses.json',newposes);save(out/'comparison.json',record)
 save(out/'camera_in_base_candidate.json',{'status':'action1_mixed_focus_diagnostic_not_adopted','source_groups':ids,'current_focus_intrinsics':str(out/'new_intrinsics.json'),'T_Base_from_Camera_mm':new['full_fit']['T_Base_from_Camera_mm'],'T_Camera_from_Base_mm':new['full_fit']['T_Camera_from_Base_mm'],'reference':record['protocol'],'limitations':record['limitations'],'default_replaced':False})
 for f,h in verify.items():assert sha(Path(f))==h
 for item in m['files']:assert sha(prev/item['snapshot'])==sha(Path(item['source']))==item['sha256']
 save(out/'verification.json',{'old_result_reproduced':True,'input_source_manifest':m,'frozen_and_protected_hashes':verify,'canonical_files_unchanged':True,'hardware_access':False})
 print(json.dumps({'usable_new':newids,'metrics':{name:{'old':old[name]['rms_xyz_mm'],'new':new[name]['rms_xyz_mm']} for name in ['same_holdout','old_8_9_holdout','new_24_holdout','old_six_predict_new','leave_one_new_out','full_fit']},'old_result_reproduced':True},ensure_ascii=False))

if __name__=='__main__':main()
