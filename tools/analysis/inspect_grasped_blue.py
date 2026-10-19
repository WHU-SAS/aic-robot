#!/usr/bin/env python3
"""Propose a blue object and visible face for operator confirmation; no robot commands."""

import sys as _sys
from pathlib import Path as _Path
_sys.path.insert(0, str(_Path(__file__).resolve().parents[2]))
from host.project_paths import ROOT as REPO_ROOT, bootstrap as _bootstrap
_bootstrap(include_tools=True)
import argparse,json
from pathlib import Path
import cv2,numpy as np

p=argparse.ArgumentParser(description=__doc__);p.add_argument('folder',type=Path);a=p.parse_args()
im=cv2.imread(str(a.folder/'frame.jpg'));h,w=im.shape[:2]
hsv=cv2.cvtColor(im,cv2.COLOR_BGR2HSV)
mask=cv2.inRange(hsv,np.array([90,160,50]),np.array([140,255,255]))
mask=cv2.morphologyEx(mask,cv2.MORPH_CLOSE,np.ones((3,3),np.uint8))
contours,_=cv2.findContours(mask,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)
choices=[]
for c in contours:
 area=cv2.contourArea(c);x,y,bw,bh=cv2.boundingRect(c)
 if area<1000 or x<=0 or y<=0 or x+bw>=w or y+bh>=h:continue
 if not .35<bw/bh<3 or area/(bw*bh)<.5:continue
 choices.append((area,c))
if len(choices)!=1:raise RuntimeError('Main blue region is not unique: '+str(len(choices)))
_,contour=choices[0]
x,y,bw,bh=cv2.boundingRect(contour)
component=np.zeros(mask.shape,np.uint8);cv2.drawContours(component,[contour],-1,255,-1)
quads=[]
for value in [160,180,200,220]:
 bright=np.uint8((component>0)&(hsv[:,:,2]>=value))*255
 cs,_=cv2.findContours(bright,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)
 if not cs:continue
 face=max(cs,key=cv2.contourArea)
 if cv2.contourArea(face)<.3*cv2.contourArea(contour):continue
 q=cv2.approxPolyDP(cv2.convexHull(face),.02*cv2.arcLength(cv2.convexHull(face),True),True)
 if len(q)!=4:continue
 q=q.reshape(-1,2).astype(float)
 center=q.mean(axis=0);angle=np.arctan2(q[:,1]-center[1],q[:,0]-center[0]);q=q[np.argsort(angle)]
 q=np.roll(q,-np.argmin(q.sum(axis=1)),axis=0)
 quads.append(q)
if not quads:raise RuntimeError('No stable four-corner face candidate')
q=np.median(quads,axis=0)
def line(a,b):return np.cross(np.r_[a,1],np.r_[b,1])
center=np.cross(line(q[0],q[2]),line(q[1],q[3]));center=center[:2]/center[2]
record={'status':'operator_confirmation_needed','cube_size_mm_user_confirmed':20,
        'main_blue_bbox':[x,y,bw,bh],'face_quad_px':q.tolist(),'face_center_candidate_px':center.tolist(),
        'face_corner_span_px':np.ptp(np.array(quads),axis=0).tolist(),
        'brightness_thresholds_used':[160,180,200,220],
        'note':'Candidate is a visible blue face, not yet identified as top or side. No calibration correspondence accepted.'}
(a.folder/'face_candidate.json').write_text(json.dumps(record,indent=2))
annot=im.copy();cv2.rectangle(annot,(x,y),(x+bw,y+bh),(0,255,0),2)
cv2.polylines(annot,[np.rint(q).astype(np.int32)],True,(0,0,255),3)
for i,(cx,cy) in enumerate(q):
 cv2.putText(annot,str(i+1),(round(cx)+5,round(cy)-5),cv2.FONT_HERSHEY_SIMPLEX,.7,(0,0,255),2)
cv2.drawMarker(annot,tuple(np.rint(center).astype(int)),(0,255,255),cv2.MARKER_CROSS,20,2)
cv2.putText(annot,'Confirm red outline: TOP face or SIDE face?',(20,h-18),cv2.FONT_HERSHEY_SIMPLEX,.7,(0,0,0),4)
cv2.putText(annot,'Confirm red outline: TOP face or SIDE face?',(20,h-18),cv2.FONT_HERSHEY_SIMPLEX,.7,(255,255,255),2)
cv2.imwrite(str(a.folder/'face_candidate.jpg'),annot)
print(json.dumps(record,indent=2))
