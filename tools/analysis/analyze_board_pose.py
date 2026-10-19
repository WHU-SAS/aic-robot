#!/usr/bin/env python3
"""Detect the board, estimate its camera pose, and label its physical corners."""

import sys as _sys
from pathlib import Path as _Path
_sys.path.insert(0, str(_Path(__file__).resolve().parents[2]))
from host.project_paths import ROOT as REPO_ROOT, bootstrap as _bootstrap
_bootstrap(include_tools=True)
import argparse
import json
from pathlib import Path
import cv2
import numpy as np

p=argparse.ArgumentParser(description=__doc__)
p.add_argument('image',type=Path)
p.add_argument('--intrinsics',type=Path,default=REPO_ROOT/'configs/vision/runtime/camera_intrinsics.json')
a=p.parse_args()
cal=json.loads(a.intrinsics.read_text())
K=np.array(cal['camera_matrix'],dtype=float)
D=np.array(cal['distortion_coefficients'],dtype=float)
im=cv2.imread(str(a.image))
if im is None: raise RuntimeError('Cannot read captured image')
sx=im.shape[1]/cal['image_width'];sy=im.shape[0]/cal['image_height']
if abs(sx-sy)>1e-9:
    raise RuntimeError('Aspect ratio differs from intrinsic calibration')
K[0,:] *= sx
K[1,:] *= sy
square=cal['board']['square_mm_measured']
board=cv2.aruco.CharucoBoard((5,7),square,square/2,
                            cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_5X5_100))
board.setLegacyPattern(False)
c,ids,mc,mi=cv2.aruco.CharucoDetector(board).detectBoard(cv2.cvtColor(im,cv2.COLOR_BGR2GRAY))
method='charuco_marker_ids'
if ids is None or len(ids)<6:
    gray=cv2.cvtColor(im,cv2.COLOR_BGR2GRAY)
    ok,c=cv2.findChessboardCorners(gray,(4,6),cv2.CALIB_CB_ADAPTIVE_THRESH|cv2.CALIB_CB_NORMALIZE_IMAGE)
    if not ok:
        raise RuntimeError('Neither marker IDs nor the complete chess grid could be detected')
    c=cv2.cornerSubPix(gray,c,(5,5),(-1,-1),(cv2.TERM_CRITERIA_EPS|cv2.TERM_CRITERIA_MAX_ITER,40,.001))
    ids=np.arange(24,dtype=np.int32).reshape(-1,1)
    method='complete_chess_grid_labels_define_board_orientation'
objects=board.getChessboardCorners()[ids.flatten()]
solutions=cv2.solvePnPGeneric(objects,c.reshape(-1,2),K,D,flags=cv2.SOLVEPNP_IPPE)
candidates=[]
for r,t in zip(solutions[1],solutions[2]):
    R,_=cv2.Rodrigues(r)
    depths=(objects@R.T+t.reshape(1,3))[:,2]
    if (depths<=0).any(): continue
    projected,_=cv2.projectPoints(objects,r,t,K,D)
    err=float(np.sqrt(np.mean(np.sum((projected.reshape(-1,2)-c.reshape(-1,2))**2,axis=1))))
    candidates.append((err,r,t,R))
if not candidates: raise RuntimeError('No board pose with positive camera depths')
candidates.sort(key=lambda x:x[0])
error,r,t,R=candidates[0]
r,t=cv2.solvePnPRefineLM(objects,c.reshape(-1,2),K,D,np.ascontiguousarray(r),np.ascontiguousarray(t))
R,_=cv2.Rodrigues(r)
projected,_=cv2.projectPoints(objects,r,t,K,D)
error=float(np.sqrt(np.mean(np.sum((projected.reshape(-1,2)-c.reshape(-1,2))**2,axis=1))))
outer=np.array([[0,0,0],[5*square,0,0],[5*square,7*square,0],[0,7*square,0]],np.float32)
pixels,_=cv2.projectPoints(outer,r,t,K,D)
pixels=pixels.reshape(-1,2)
T=np.eye(4);T[:3,:3]=R;T[:3,3]=t.flatten()
record={'detection_method':method,'corner_indices_are_marker_confirmed':method=='charuco_marker_ids',
        'corners_detected':len(ids),'markers_detected':0 if mi is None else len(mi),
        'image_size':[im.shape[1],im.shape[0]],'intrinsics_scale':[sx,sy],
        'reprojection_rms_px':error,'alternative_pose_rms_px':[s[0] for s in candidates[1:]],
        'T_camera_from_board_mm':T.tolist(),
        'board_frame':'A=(0,0,0); AB=+X; AD=+Y; right-handed +Z; units mm',
        'outer_corners':{label:{'board_mm':obj.tolist(),'image_px':px.tolist()} for label,obj,px in zip('ABCD',outer,pixels)},
        'detected_internal_corners':[{'id':int(i),'board_mm':obj.tolist(),'image_px':px.tolist()}
                                     for i,obj,px in zip(ids.flatten(),objects,c.reshape(-1,2))],
        'status':'camera-to-board diagnostic only; this does not replace the current camera-to-base baseline'}
(a.image.parent/'board_pose.json').write_text(json.dumps(record,indent=2))
annot=im.copy()
cv2.aruco.drawDetectedCornersCharuco(annot,c,ids)
cv2.polylines(annot,[np.rint(pixels).astype(np.int32)],True,(0,255,255),2)
for label,px in zip('ABCD',pixels):
    x,y=map(int,np.rint(px))
    cv2.circle(annot,(x,y),5,(0,0,255),-1)
    tx=max(0,min(im.shape[1]-25,x+8));ty=max(22,min(im.shape[0]-4,y-8))
    cv2.putText(annot,label,(tx,ty),cv2.FONT_HERSHEY_SIMPLEX,.8,(0,0,0),4)
    cv2.putText(annot,label,(tx,ty),cv2.FONT_HERSHEY_SIMPLEX,.8,(0,255,255),2)
cv2.imwrite(str(a.image.parent/'board_labeled.jpg'),annot)
print(json.dumps({'corners':len(ids),'markers':record['markers_detected'],
                  'rms_px':error,'outer_corners':record['outer_corners']},indent=2))
print('LABELED',a.image.parent/'board_labeled.jpg')
