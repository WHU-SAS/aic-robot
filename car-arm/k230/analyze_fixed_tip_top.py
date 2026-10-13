#!/usr/bin/env python3
"""Offline fixed-fingertip cube-top calibration. Never opens a serial port."""
import argparse
from datetime import datetime
import hashlib
import json
from pathlib import Path
import shutil
import subprocess
import sys
from zoneinfo import ZoneInfo
PROJECT=Path(__file__).resolve().parents[1]
GROUPS=[1,2,3,4,5]
HOLDOUT_GROUPS=[4,5]

def load(p):return json.loads(p.read_text())
def save(p,d):p.write_text(json.dumps(d,ensure_ascii=False,indent=2)+'\n')
def sha(p):return hashlib.sha256(p.read_bytes()).hexdigest()

def snapshot(out):
 root=PROJECT/'k230/fixed_tip_top_pairs';collection=load(root/'collection.json');mapping={r['group']:r for r in collection['groups']}
 assert all(g in mapping for g in GROUPS)
 out.mkdir(exist_ok=False,parents=True);snap=out/'snapshot';snap.mkdir();inventory=[]
 for group in GROUPS:
  source=root/mapping[group]['folder'];target=snap/f'group_{group:02d}';target.mkdir();r=load(source/'record.json')
  assert r['sample_type']=='fixed_fingertip_cube_top_contact' and r['contacting_jaw']=='fixed'
  assert len(r['samples_before'])==3 and len(r['samples_after'])==3
  assert sha(source/'frame.jpg')==r['camera_capture']['sha256']
  for name in ['record.json','frame.jpg']:
   p=source/name;d=sha(p);shutil.copy2(p,target/name);assert sha(p)==sha(target/name)==d
   inventory.append({'source':str(p),'snapshot':str((target/name).relative_to(out)),'sha256':d})
 for source,name in [(PROJECT/'k230/calibration_results/camera_intrinsics.json','intrinsics.json'),(PROJECT/'k230/calibration_results/camera_in_base_provisional.json','old_baseline.json'),(PROJECT/'hardware/joint_mapping.json','joint_mapping.json')]:
  d=sha(source);shutil.copy2(source,snap/name);assert sha(snap/name)==d;inventory.append({'source':str(source),'snapshot':str((snap/name).relative_to(out)),'sha256':d})
 save(snap/'collection.json',{**collection,'groups':[mapping[g] for g in GROUPS]})
 model=[PROJECT/'hardware/gripper_geometry.py',PROJECT/'ros2_ws/src/arm_pkg/urdf/genkiarm.urdf']+list((PROJECT/'ros2_ws/src/arm_pkg/urdf/meshes').glob('*.stl'))
 save(out/'input_manifest.json',{'created':datetime.now(ZoneInfo('Asia/Shanghai')).isoformat(),'groups':GROUPS,'holdout_groups':HOLDOUT_GROUPS,'sample_namespace':'fixed_tip_top_pairs','hardware_access':False,'files':inventory,'model_hashes':[{'path':str(p),'sha256':sha(p)} for p in model]})
 print('SNAPSHOT',out,flush=True)

def encoder_fk(out):
 import numpy as np
 sys.path[:0]=[str(PROJECT/'hardware'),str(PROJECT/'ros2_ws/src/arm_pkg')]
 from gripper_geometry import PickGeometry
 from arm_pkg.joint_mapping import JointMapping
 geometry=PickGeometry(PROJECT,JointMapping.load(out/'snapshot/joint_mapping.json'))
 vertices=geometry.vertices['wan2'];maxz=float(vertices[:,2].max());extreme=vertices[np.abs(vertices[:,2]-maxz)<1e-8]
 local=extreme.astype(float).mean(0);local[2]=maxz
 # Explicitly keep a constant local point across every pose. It is a proxy,
 # not a claim to have measured the user's physical endpoint.
 proxy={'fixed_jaw_link':'wan2','local_point_m':local.tolist(),'definition':'Mean XY of maximum local-Z STL vertices, with exact maximum local Z',
        'physical_point_surveyed':False,'other_proxy_local_m':geometry.tip['wan2'].tolist(),
        'local_maxZ_vertices_m':extreme.tolist(),'table_normal_Base':[0,0,1],'cube_center_offset_Base_mm':[0,0,-10]}
 rows=[]
 for group in GROUPS:
  r=load(out/'snapshot'/f'group_{group:02d}'/'record.json');samples=r['samples_before']+r['samples_after']
  counts=[int(round(float(np.median([s[str(i)]['position'] for s in samples])))) for i in range(1,7)]
  spans=[int(np.ptp([s[str(i)]['position'] for s in samples])) for i in range(1,7)]
  q=geometry.angles(counts);mid,tips,lows,T=geometry.metrics(q);matrix=T['wan2'];point=(matrix@np.r_[local,1])[:3]*1000
  alternative=(matrix@np.r_[geometry.tip['wan2'],1])[:3]*1000
  rows.append({'group':group,'counts':counts,'encoder_spans':spans,'speeds_raw':[[s[str(i)]['speed'] for s in samples] for i in range(1,7)],
   'torque_states':[[s[str(i)]['torque'] for s in samples] for i in range(1,7)],'joint_angles_deg':np.rad2deg(q).tolist(),
   'fixed_contact_point_Base_mm':point.tolist(),'cube_center_Base_mm':(point+[0,0,-10]).tolist(),
   'table_z_Base_mm_estimate':float(point[2]-20),'old_distal2mm_proxy_contact_Base_mm':alternative.tolist(),
   'proxy_difference_mm':float(np.linalg.norm(point-alternative)),
   'fixed_jaw_transform':matrix.tolist(),'fixed_jaw_tilt_deg':float(np.degrees(np.arccos(np.clip(-matrix[2,2],-1,1))))})
 save(out/'fixed_point_definition.json',proxy);save(out/'encoder_fk.json',rows)

def vision(out):
 import cv2,numpy as np
 sys.path.insert(0,str(PROJECT/'k230'))
 from analyze_grasp_batch import face_candidates,pose_candidates
 from grasp_face_quality import assess
 intr=load(out/'snapshot/intrinsics.json');K=np.array(intr['camera_matrix'],float);K[:2]*=2;D=np.array(intr['distortion_coefficients'],float)
 rows=[];tiles=[]
 for group in GROUPS:
  im=cv2.imread(str(out/'snapshot'/f'group_{group:02d}'/'frame.jpg'));assert im is not None and im.shape[:2]==(960,1280)
  v=face_candidates(im,group);v['boundary_method']='existing_threshold_median';ok=False
  if v['status']=='complete_face_candidate':
   ps=pose_candidates(np.array(v['face_quad_px']),K,D)
   ok=bool(ps and ps[0]['face_reprojection_rms_px']<=2. and np.array(v['face_quad_px'])[:,1].min()>=432)
   if ok:v['poses']=ps
  if not ok:
   v=assess(im,K,D);v['boundary_method']='dim_blue_line_refinement'
  v['group']=group
  if len(v.get('poses',[]))==2:v['center_pose_ambiguity_mm']=float(np.linalg.norm(np.array(v['poses'][0]['cube_center_Camera_mm'])-v['poses'][1]['cube_center_Camera_mm']))
  annot=im.copy()
  if v.get('face_quad_px'):cv2.polylines(annot,[np.rint(v['face_quad_px']).astype(np.int32)],True,(0,255,0) if v['status']=='complete_face_candidate' else (0,128,255),3)
  for pose in v.get('poses',[])[:1]:cv2.drawMarker(annot,tuple(np.rint(pose['cube_center_pixel']).astype(int)),(0,255,255),cv2.MARKER_CROSS,20,2)
  if not v.get('poses'):
   clips=v.get('dim_face_assessment',{}).get('clipped',[])
   if clips:
    _,x,y,w,h=max(clips);cv2.rectangle(annot,(x,y),(x+w-1,y+h-1),(0,0,255),3)
  cv2.imwrite(str(out/f'fixed_top_{group:02d}_annotated.jpg'),annot);tile=cv2.resize(annot,(480,360))
  cv2.putText(tile,f"Fixed-top {group}: {v['status']}",(7,24),cv2.FONT_HERSHEY_SIMPLEX,.48,(0,0,0),3)
  cv2.putText(tile,f"Fixed-top {group}: {v['status']}",(7,24),cv2.FONT_HERSHEY_SIMPLEX,.48,(255,255,255),1)
  rows.append(v);tiles.append(tile);print('VISION',group,v['status'],[round(x['face_reprojection_rms_px'],3) for x in v.get('poses',[])],flush=True)
 while len(tiles)%3:tiles.append(np.zeros_like(tiles[0]))
 cv2.imwrite(str(out/'contact_sheet.jpg'),np.vstack([np.hstack(tiles[i:i+3]) for i in range(0,len(tiles),3)]))
 save(out/'vision.json',rows)

def analyze(out):
 import numpy as np
 f={r['group']:r for r in load(out/'encoder_fk.json')};v={r['group']:r for r in load(out/'vision.json')}
 ids=[g for g in GROUPS if v[g]['status']=='complete_face_candidate' and v[g].get('poses')]
 camera={g:np.array(v[g]['poses'][0]['cube_center_Camera_mm']) for g in ids}
 base={g:np.array(f[g]['cube_center_Base_mm']) for g in ids}
 alternate={g:np.array(f[g]['old_distal2mm_proxy_contact_Base_mm'])+np.array([0,0,-10]) for g in ids}
 def fit(groups,refs=base):
  x=np.array([camera[g] for g in groups]);y=np.array([refs[g] for g in groups]);xc=x-x.mean(0);yc=y-y.mean(0)
  if len(groups)<3 or np.linalg.matrix_rank(xc)<2 or np.linalg.matrix_rank(yc)<2:raise ValueError('Insufficient non-collinear training pairs')
  U,s,Vt=np.linalg.svd(xc.T@yc);R=Vt.T@U.T
  if np.linalg.det(R)<0:Vt[-1]*=-1;R=Vt.T@U.T
  assert np.allclose(R.T@R,np.eye(3),atol=1e-10) and np.isclose(np.linalg.det(R),1.)
  return R,y.mean(0)-R@x.mean(0),s
 def evaluate(R,t,groups,refs=base):
  rows=[]
  for g in groups:
   pred=R@camera[g]+t;delta=pred-refs[g]
   rows.append({'group':g,'predicted_Base_mm':pred.tolist(),'reference_Base_mm':refs[g].tolist(),'delta_mm':delta.tolist(),
    'xy_mm':float(np.linalg.norm(delta[:2])),'xyz_mm':float(np.linalg.norm(delta))})
  if not rows:return {'groups':[],'rms_xy_mm':None,'rms_xyz_mm':None}
  return {'groups':rows,'rms_xy_mm':float(np.sqrt(np.mean([r['xy_mm']**2 for r in rows]))),'rms_xyz_mm':float(np.sqrt(np.mean([r['xyz_mm']**2 for r in rows]))),
   'max_xy_mm':max(r['xy_mm'] for r in rows),'max_xyz_mm':max(r['xyz_mm'] for r in rows)}
 old=load(out/'snapshot/old_baseline.json');oldT=np.array(old['T_Base_from_Camera_mm']);oldpred=evaluate(oldT[:3,:3],oldT[:3,3],ids)
 result={'sample_type':'fixed_fingertip_cube_top_contact','requested_groups':GROUPS,'usable_groups':ids,'reference':'same fixed local wan2 endpoint proxy, Base -Z 10mm, not midpoint of two jaws',
 'fixed_point_definition':load(out/'fixed_point_definition.json'),
 'quality':[{'group':g,'status':v[g]['status'],'reason':v[g].get('reason'),'boundary_method':v[g]['boundary_method'],
  'face_reprojection_rms_px':v[g]['poses'][0]['face_reprojection_rms_px'] if v[g].get('poses') else None,
  'pose_ambiguity_mm':v[g].get('center_pose_ambiguity_mm'),'encoder_spans':f[g]['encoder_spans']} for g in GROUPS],
 'existing_old6_baseline_on_new_contacts':oldpred,'old_baseline_original_fixed_split_xyz_rms_mm':old['fixed_split_validation']['rms_3d_mm'],
 'note':'Old 4.324mm validation is a different set with a different reference; not a same-test-set improvement comparison.',
 'intrinsics':'Old pre-refocus intrinsics reused; current focus calibration validity not established',
 'canonical_baseline_updated':False,'hardware_access':False}
 if len(ids)>=3:
  R,t,s=fit(ids);T=np.eye(4);T[:3,:3]=R;T[:3,3]=t;res=evaluate(R,t,ids)
  result['full_fit']={**res,'T_Base_from_Camera_mm':T.tolist(),'T_Camera_from_Base_mm':np.linalg.inv(T).tolist(),'singular_values':s.tolist()}
  candidate={'status':'fixed_top_contact_diagnostic_candidate_not_adopted','source_namespace':'fixed_tip_top_pairs','source_groups':ids,
   'T_Base_from_Camera_mm':T.tolist(),'T_Camera_from_Base_mm':np.linalg.inv(T).tolist(),
   'point_definition':result['fixed_point_definition'],'intrinsic_validity':result['intrinsics'],'training_metrics':res,'default_replaced':False}
  save(out/'camera_in_base_candidate.json',candidate)
  if len(ids)>=4:
   loo=[];altloo=[]
   for g in ids:
    tr=[i for i in ids if i!=g];a,b,_=fit(tr);loo.extend(evaluate(a,b,[g])['groups'])
    a,b,_=fit(tr,alternate);altloo.extend(evaluate(a,b,[g],alternate)['groups'])
   result['leave_one_out']={'groups':loo,'rms_xy_mm':float(np.sqrt(np.mean([r['xy_mm']**2 for r in loo]))),'rms_xyz_mm':float(np.sqrt(np.mean([r['xyz_mm']**2 for r in loo]))),'max_xyz_mm':max(r['xyz_mm'] for r in loo)}
   result['endpoint_proxy_sensitivity']={'alternative':'existing mean distal2mm fixed-jaw proxy; same offset -10mm','reference_difference_mm':[f[g]['proxy_difference_mm'] for g in ids],
    'alternate_leave_one_out':{'groups':altloo,'rms_xyz_mm':float(np.sqrt(np.mean([r['xyz_mm']**2 for r in altloo]))),'rms_xy_mm':float(np.sqrt(np.mean([r['xy_mm']**2 for r in altloo])))}}
  train=[g for g in ids if g not in HOLDOUT_GROUPS];test=[g for g in HOLDOUT_GROUPS if g in ids]
  if len(train)>=3 and test:
   a,b,_=fit(train);result['fixed_split']={'training_groups':train,'validation_groups':test,'old':evaluate(oldT[:3,:3],oldT[:3,3],test),'new':evaluate(a,b,test)}
  points=np.array([f[g]['fixed_contact_point_Base_mm'] for g in ids]);result['coverage_Base_mm']={'min':points.min(0).tolist(),'max':points.max(0).tolist(),'range':np.ptp(points,axis=0).tolist()}
  table=np.array([f[g]['table_z_Base_mm_estimate'] for g in ids]);result['table_z_Base_mm_model']={'median':float(np.median(table)),'range':[float(table.min()),float(table.max())]}
 save(out/'analysis.json',result)
 print('ANALYSIS',json.dumps(result,ensure_ascii=False),flush=True)

def main():
 global GROUPS,HOLDOUT_GROUPS
 p=argparse.ArgumentParser(description=__doc__);p.add_argument('--output',type=Path,required=True);p.add_argument('--vision-phase',action='store_true');p.add_argument('--resume-frozen',action='store_true');p.add_argument('--groups',type=int,nargs='+');p.add_argument('--holdout-groups',type=int,nargs='+');args=p.parse_args();out=args.output.resolve()
 if args.vision_phase or args.resume_frozen:
  frozen=load(out/'input_manifest.json');GROUPS=frozen['groups'];HOLDOUT_GROUPS=frozen.get('holdout_groups',GROUPS[-2:])
 else:
  if args.groups is not None:GROUPS=args.groups
  HOLDOUT_GROUPS=args.holdout_groups if args.holdout_groups is not None else GROUPS[-2:]
  if len(set(GROUPS))!=len(GROUPS) or not set(HOLDOUT_GROUPS)<=set(GROUPS):raise ValueError('Invalid group or holdout IDs')
 if args.vision_phase:vision(out);return
 if not args.resume_frozen:snapshot(out);encoder_fk(out)
 subprocess.run([str(PROJECT/'scripts/vision'),str(Path(__file__).resolve()),'--output',str(out),'--vision-phase'],cwd=PROJECT,check=True)
 analyze(out);m=load(out/'input_manifest.json')
 for file in m['files']:assert sha(out/file['snapshot'])==sha(Path(file['source']))==file['sha256']
 for file in m['model_hashes']:assert sha(Path(file['path']))==file['sha256']
 save(out/'verification.json',{'original_and_snapshot_hashes_unchanged':len(m['files']),'model_hashes_unchanged':len(m['model_hashes']),
  'default_extrinsics_unchanged':load(out/'snapshot/old_baseline.json')==load(PROJECT/'k230/calibration_results/camera_in_base_provisional.json'),'hardware_access':False})
 print('INPUTS_AND_DEFAULT_BASELINE_UNCHANGED',flush=True)

if __name__=='__main__':main()
