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
from host.vision.config import load_vision_config,table_height_at
ROOT=REPO_ROOT

def localize(image,output,vision_config=None):
 output=Path(output);output.mkdir(parents=True,exist_ok=True);profile=load_vision_config(vision_config);cfg=profile['localization'];ip=ROOT/cfg['intrinsics'];ep=ROOT/cfg['extrinsics'];intr=json.loads(ip.read_text());ext=json.loads(ep.read_text())
 if hashlib.sha256(ip.read_bytes()).hexdigest()!=ext['intrinsics_sha256']:raise RuntimeError('Candidate intrinsics and extrinsics do not match')
 # Freeze the actual calibration and source fingerprints before attempting
 # detection, so rejected images remain reproducible after config updates.
 sources=sorted((ROOT/'device/k230/pick').glob('*.py'))+sorted((ROOT/'device/k230/vision').glob('*.py'))
 context={'schema':'aic.vision_context.v1','vision':profile,'intrinsics':intr,'extrinsics':ext,
          'table_plane':cfg.get('table_plane_Base_mm') if cfg.get('table_pose_constraints_enabled') else None,
          'version':{'source_sha256':{str(p.relative_to(ROOT)):hashlib.sha256(p.read_bytes()).hexdigest() for p in sources}},
          'calibration_sources':{'intrinsics':str(ip),'extrinsics':str(ep),
            'intrinsics_sha256':hashlib.sha256(ip.read_bytes()).hexdigest(),
            'extrinsics_sha256':hashlib.sha256(ep.read_bytes()).hexdigest()}}
 (output/'vision_context.json').write_text(json.dumps(context,ensure_ascii=False,indent=2)+'\n')
 im=cv2.imread(str(image))
 if im is None or im.shape[:2]!=(960,1280):raise RuntimeError('Expected a completed1280x960 K230 frame')
 from device.k230.vision.runtime import localize_precise
 frame={'frame_id':str(Path(image).resolve()),'capture_timestamp_ms':None,
        'clock_domain':None,'timestamp_source':'unavailable','image_size_px':[1280,960]}
 observation=localize_precise(im,frame,intr,ext,cfg.get('table_plane_Base_mm') if cfg.get('table_pose_constraints_enabled') else None)
 v=observation.pop('payload',{'status':'no_complete_face','reason':observation.get('reason')})
 if v['status']!='complete_face_candidate' or not v.get('poses'):
  (output/'vision_candidates.json').write_text(json.dumps(v,ensure_ascii=False,indent=2)+'\n');raise RuntimeError('未识别到完整可靠的蓝色20mm方块面：'+str(v.get('reason',v['status'])))
 T=np.array(ext['T_Base_from_Camera_mm']);cent=np.array(v['poses'][0]['cube_center_Camera_mm']);base=T[:3,:3]@cent+T[:3,3]
 for index,face in enumerate(v.get('side_faces') or [v]):
  cv2.polylines(im,[np.rint(face['face_quad_px']).astype(np.int32)],True,((0,255,0) if index==0 else (255,255,0)),3)
 cv2.drawMarker(im,tuple(np.rint(v['poses'][0]['cube_center_pixel']).astype(int)),(0,255,255),cv2.MARKER_CROSS,20,2);cv2.imwrite(str(output/'camera_detected.jpg'),im)
 table_z=table_height_at(cfg,base[:2])
 result={'status':'localized','method':'shared PC/K230 measured sides; larger side or mean of similar-area side centers; verified top fallback','cube_center_Base_mm':base.tolist(),'cube_center_Camera_mm':cent.tolist(),'table_z_Base_mm':table_z,'cube_center_z_above_assumed_table_mm':float(base[2]-table_z),'vision':v,'intrinsics':str(ip),'extrinsics':str(ep),'motion_performed':False,'xy_compensation_applied':False,'observation':observation}
 if cfg.get('table_plane_Base_mm') is not None:result['table_plane_Base_mm']=cfg['table_plane_Base_mm']
 (output/'vision_record.json').write_text(json.dumps(result,ensure_ascii=False,indent=2)+'\n');return result

if __name__=='__main__':
 p=argparse.ArgumentParser();p.add_argument('--image',type=Path,required=True);p.add_argument('--output',type=Path,required=True);p.add_argument('--config',type=Path);a=p.parse_args()
 try:print(json.dumps(localize(a.image,a.output,a.config),ensure_ascii=False))
 except RuntimeError as exc:
  print('方块定位失败：'+str(exc)+'；诊断文件：'+str(a.output/'vision_candidates.json'),file=sys.stderr);raise SystemExit(2)
