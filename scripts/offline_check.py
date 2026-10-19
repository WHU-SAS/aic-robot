#!/usr/bin/env python3
"""Verify project independence, raw files and numerical FK without hardware."""

import sys as _sys
from pathlib import Path as _Path
_sys.path.insert(0, str(_Path(__file__).resolve().parents[1]))
from host.project_paths import ROOT as REPO_ROOT, bootstrap as _bootstrap
_bootstrap(include_tools=True)
import ast
import hashlib
import json
from pathlib import Path
import subprocess
import sys
import numpy as np

ROOT=REPO_ROOT
from host.transport import servo_bus
from host.arm import joint_mapping,urdf_fk
from gripper_geometry import PickGeometry
from task_controller import Configuration

def verify_raw_pairs(root,collection):
    if not collection['groups']:raise RuntimeError('No raw image/encoder pairs')
    ids=[g['group'] for g in collection['groups']]
    if len(ids)!=len(set(ids)):raise RuntimeError('Duplicate collection group IDs')
    verified=pending=0
    for group in collection['groups']:
        folder=root/'data/k230/grasp_pairs'/group['folder']
        record=json.loads((folder/'record.json').read_text())
        image=folder/'frame.jpg'
        if image.exists():
            digest=hashlib.sha256(image.read_bytes()).hexdigest()
            if digest!=record['camera_capture']['sha256']:
                raise RuntimeError('Raw image changed: '+str(folder))
            verified+=1
        elif record['camera_capture'].get('downloaded'):
            raise RuntimeError('Exported raw image missing: '+str(folder))
        else: pending+=1
        if len(record['samples_before'])!=3 or len(record['samples_after'])!=3:
            raise RuntimeError('Missing raw feedback: '+str(folder))
    return {'verified':verified,'pending':pending}

def check_local_data():
    for module in [servo_bus,joint_mapping,urdf_fk]:
        if not Path(module.__file__).resolve().is_relative_to(ROOT):
            raise RuntimeError('Cross-project import: '+module.__file__)
    for base in [ROOT/'host',ROOT/'tools',ROOT/'scripts']:
        for path in base.rglob('*.py'):
            ast.parse(path.read_text(),filename=str(path))
            if '/home/xd/Desktop/机械臂项目' in path.read_text():
                raise RuntimeError('Old-project runtime path: '+str(path))
    config=Configuration(ROOT)
    geometry=PickGeometry(ROOT,config.mapping)
    rows=json.loads((ROOT/'data/k230/grasp_pairs/analysis_20261013/encoder_fk.json').read_text())
    largest=0.
    for row in rows:
        q=geometry.angles(row['encoder_counts'])
        mid,tips,lows,transforms=geometry.metrics(q)
        error=float(np.max(np.abs(mid*1000-np.array(row['tip_midpoint_Base_mm']))))
        largest=max(largest,error)
        if error>1e-8:raise RuntimeError('FK changed during split: group '+str(row['group']))
    collection=json.loads((ROOT/'data/k230/grasp_pairs/collection.json').read_text())
    storage=verify_raw_pairs(ROOT,collection)
    verified,pending=storage['verified'],storage['pending']
    baseline=json.loads((ROOT/'data/k230/calibration_results/camera_in_base_provisional.json').read_text())
    T=np.array(baseline['T_Base_from_Camera_mm'])
    if not np.allclose(T@np.array(baseline['T_Camera_from_Base_mm']),np.eye(4),atol=1e-8):
        raise RuntimeError('Extrinsics inverse mismatch')
    snippet='''import json,sys
from pathlib import Path
import cv2,numpy
root=Path.cwd()
sys.path.insert(0,str(root/'host/vision'))
from host.vision.face_geometry import face_candidates,pose_candidates
intr=json.loads((root/'data/k230/calibration_results/camera_intrinsics.json').read_text())
K=numpy.array(intr['camera_matrix']);K[0]*=2;K[1]*=2
D=numpy.array(intr['distortion_coefficients'])
rows=json.loads((root/'data/k230/grasp_pairs/analysis_20261013/groups_analysis.json').read_text())
for row in rows:
 im=cv2.imread(str(root/'data/k230/grasp_pairs'/row['folder']/'frame.jpg'))
 assert im.shape[:2]==(960,1280)
 if row['vision'].get('poses'):
  poses=pose_candidates(numpy.array(row['vision']['face_quad_px']),K,D)
  assert numpy.allclose(poses[0]['cube_center_Camera_mm'],row['vision']['poses'][0]['cube_center_Camera_mm'],atol=1e-5)
print(json.dumps({'opencv':cv2.__version__,'numpy':numpy.__version__,'pose_reproduction':'passed'}))
'''
    result=subprocess.run([str(ROOT/'scripts/vision'),'-c',snippet],text=True,capture_output=True,check=True)
    print(result.stdout.strip())
    print(json.dumps({'status':'passed','hardware_access':False,
                      'collection_groups':len(collection['groups']),'raw_pairs_verified':verified,
                      'images_pending_download':pending,
                      'fk_groups_verified':len(rows),'fk_max_delta_mm':largest,
                      'local_arm_imports':True,'pid_parameters':config.baseline['parameters']},ensure_ascii=False))

def main():
    import argparse
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--with-local-data',action='store_true',help='also verify locally retained historical calibration samples')
    args=parser.parse_args()
    config=Configuration(ROOT)
    for module in [servo_bus,joint_mapping,urdf_fk]:
        if not Path(module.__file__).resolve().is_relative_to(ROOT):raise RuntimeError('Cross-project module import')
    for base in [ROOT/'host',ROOT/'tools',ROOT/'scripts',ROOT/'device/k230']:
        for p in base.rglob('*.py'):
            if 'runtime_checks' not in p.parts:ast.parse(p.read_text(),filename=str(p))
    cfg=json.loads((ROOT/'configs/arm/k230_pick_config.json').read_text())
    ip=ROOT/cfg['intrinsics'];ep=ROOT/cfg['extrinsics'];intr=json.loads(ip.read_text());ext=json.loads(ep.read_text())
    if hashlib.sha256(ip.read_bytes()).hexdigest()!=ext['intrinsics_sha256']:raise RuntimeError('Runtime camera pair hash mismatch')
    T=np.array(ext['T_Base_from_Camera_mm']);inverse=np.array(ext['T_Camera_from_Base_mm'])
    if not np.allclose(T@inverse,np.eye(4),atol=1e-8):raise RuntimeError('Runtime camera pair inverse mismatch')
    from blue_pick_geometry import PickGeometry as RuntimeGeometry
    geometry=RuntimeGeometry(ROOT,config.mapping,table_z_m=cfg['table_z_Base_mm']/1000,hover_tip_height_mm=cfg['hover_tip_height_mm'],descend_tip_height_mm=cfg['descend_tip_height_mm'],lift_tip_height_mm=cfg['lift_tip_height_mm'],opening_count=cfg['opening_count'])
    geometry.report([2048]*6)
    for counts in [[2048]*6,[2100,2200,2300,2400,2048,2389]]:
        if geometry.counts(geometry.angles(counts))!=counts:raise RuntimeError('Encoder/angle mapping roundtrip mismatch')
    snippet="import sys,json;from pathlib import Path;sys.path.insert(0,str(Path.cwd()/'host/vision'));import cv2,numpy,localize_board_faces,localize_pick_cube;print(json.dumps({'opencv':cv2.__version__,'numpy':numpy.__version__,'runtime_vision_imports':'passed'}))"
    result=subprocess.run([str(ROOT/'scripts/vision'),'-c',snippet],text=True,capture_output=True,check=True)
    print(result.stdout.strip())
    print(json.dumps({'status':'passed','hardware_access':False,'local_arm_imports':True,'runtime_calibration_hash_and_inverse':'passed','encoder_mapping_and_fk':'passed','raw_data_required':False},ensure_ascii=False))
    if args.with_local_data:check_local_data()

if __name__=='__main__':main()
