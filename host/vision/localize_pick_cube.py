#!/usr/bin/env python3
"""Use the existing K230 OpenCV face recognition and 20mm IPPE cube pose."""

import sys as _sys
from pathlib import Path as _Path
_sys.path.insert(0, str(_Path(__file__).resolve().parents[2]))
from host.project_paths import ROOT as REPO_ROOT, bootstrap as _bootstrap
_bootstrap(include_tools=False)
import argparse,hashlib,json,sys
from pathlib import Path
import cv2,numpy as np
from face_geometry import face_candidates,pose_candidates
from grasp_face_quality import assess
from runtime_cube_faces import split_exposed_cube
from host.vision.config import load_vision_config,table_height_at
ROOT=REPO_ROOT

def localize(image,output,vision_config=None):
 output=Path(output);output.mkdir(parents=True,exist_ok=True);cfg=load_vision_config(vision_config)['localization'];ip=ROOT/cfg['intrinsics'];ep=ROOT/cfg['extrinsics'];intr=json.loads(ip.read_text());ext=json.loads(ep.read_text())
 if hashlib.sha256(ip.read_bytes()).hexdigest()!=ext['intrinsics_sha256']:raise RuntimeError('Candidate intrinsics and extrinsics do not match')
 im=cv2.imread(str(image))
 if im is None or im.shape[:2]!=(960,1280):raise RuntimeError('Expected a completed1280x960 K230 frame')
 K=np.array(intr['camera_matrix'],float);K[0]*=1280/intr['image_width'];K[1]*=960/intr['image_height'];D=np.array(intr['distortion_coefficients'],float)
 v=face_candidates(im,0);v['boundary_method']='existing_threshold_median';ok=False
 if v['status']=='complete_face_candidate':
  ps=pose_candidates(np.array(v['face_quad_px']),K,D);ok=bool(ps and ps[0]['face_reprojection_rms_px']<=2 and np.array(v['face_quad_px'])[:,1].min()>=432)
  if ok:v['poses']=ps
 if not ok:v=assess(im,K,D);v['boundary_method']='dim_blue_line_refinement'
 if v['status']!='complete_face_candidate' or not v.get('poses'):
  initial=v;separated=split_exposed_cube(im,K,D)
  if separated['status']=='complete_face_candidate' and separated.get('poses'):
   v=separated;v['previous_whole_outline_diagnostic']={'status':initial['status'],'reason':initial.get('reason'),'face_quad_px':initial.get('face_quad_px'),'face_rms_px':initial['poses'][0]['face_reprojection_rms_px'] if initial.get('poses') else None}
  else:v['exposed_cube_separation_diagnostic']=separated
 if v['status']!='complete_face_candidate' or not v.get('poses'):
  from host.vision.blue_side_edges import detect_blue_side
  supported=detect_blue_side(im,K,D)
  if supported['status']=='complete_face_candidate':
   supported['previous_attempt']=v;v=supported
  else:v['gradient_side_attempt']=supported
 if v['status']!='complete_face_candidate' or not v.get('poses'):
  (output/'vision_candidates.json').write_text(json.dumps(v,ensure_ascii=False,indent=2)+'\n');raise RuntimeError('未识别到完整可靠的蓝色20mm方块面：'+str(v.get('reason',v['status'])))
 T=np.array(ext['T_Base_from_Camera_mm']);cent=np.array(v['poses'][0]['cube_center_Camera_mm']);base=T[:3,:3]@cent+T[:3,3]
 cv2.polylines(im,[np.rint(v['face_quad_px']).astype(np.int32)],True,(0,255,0),3);cv2.drawMarker(im,tuple(np.rint(v['poses'][0]['cube_center_pixel']).astype(int)),(0,255,255),cv2.MARKER_CROSS,20,2);cv2.imwrite(str(output/'camera_detected.jpg'),im)
 table_z=table_height_at(cfg,base[:2])
 result={'status':'localized','method':'existing K230 OpenCV blue face +20mm IPPE; lowest image reprojection candidate','cube_center_Base_mm':base.tolist(),'cube_center_Camera_mm':cent.tolist(),'table_z_Base_mm':table_z,'cube_center_z_above_assumed_table_mm':float(base[2]-table_z),'vision':v,'intrinsics':str(ip),'extrinsics':str(ep),'motion_performed':False,'xy_compensation_applied':False}
 if cfg.get('table_plane_Base_mm') is not None:result['table_plane_Base_mm']=cfg['table_plane_Base_mm']
 (output/'vision_record.json').write_text(json.dumps(result,ensure_ascii=False,indent=2)+'\n');return result

if __name__=='__main__':
 p=argparse.ArgumentParser();p.add_argument('--image',type=Path,required=True);p.add_argument('--output',type=Path,required=True);p.add_argument('--config',type=Path);a=p.parse_args()
 try:print(json.dumps(localize(a.image,a.output,a.config),ensure_ascii=False))
 except RuntimeError as exc:
  print('方块定位失败：'+str(exc)+'；诊断文件：'+str(a.output/'vision_candidates.json'),file=sys.stderr);raise SystemExit(2)
