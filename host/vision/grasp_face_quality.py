"""Offline dim-blue face assessment. Does not use FK to select image corners."""

import sys as _sys
from pathlib import Path as _Path
_sys.path.insert(0, str(_Path(__file__).resolve().parents[2]))
from host.project_paths import ROOT as REPO_ROOT, bootstrap as _bootstrap
_bootstrap(include_tools=False)
import cv2
import numpy as np
from face_geometry import ordered_quad,pose_candidates

def refine(c,q):
 points=c.reshape(-1,2).astype(float);lines=[]
 for a,b in zip(q,np.roll(q,-1,axis=0)):
  u=b-a;length=np.linalg.norm(u);u/=length;n=np.array([-u[1],u[0]]);dist=np.abs((points-a)@n);along=(points-a)@u
  selected=points[(dist<7)&(along>length*.1)&(along<length*.9)]
  if len(selected)<8:return q
  vx,vy,x,y=cv2.fitLine(selected.astype(np.float32),cv2.DIST_HUBER,0,.01,.01).ravel();lines.append((np.array([x,y]),np.array([vx,vy])))
 result=[]
 for i in range(4):
  a,u=lines[(i-1)%4];b,v=lines[i]
  try:t=np.linalg.solve(np.c_[u,-v],b-a)[0]
  except np.linalg.LinAlgError:return q
  result.append(a+t*u)
 result=np.array(result)
 return result if np.max(np.linalg.norm(result-q,axis=1))<15 else q

def detect(im,K,D):
 hsv=cv2.cvtColor(im,cv2.COLOR_BGR2HSV);height,width=im.shape[:2];candidates=[];clipped=[]
 for s in [80,120,160,200]:
  for v in [40,60,80,100,120,140,160,180,200,220]:
   mask=cv2.inRange(hsv,np.array([90,s,v]),np.array([140,255,255]));mask=cv2.morphologyEx(mask,cv2.MORPH_CLOSE,np.ones((3,3),np.uint8));cont,_=cv2.findContours(mask,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_NONE)
   for c in cont:
    area=cv2.contourArea(c);x,y,w,h=cv2.boundingRect(c)
    if area<6000 or y<height*.45:continue
    if x<=2 or x+w>=width-2 or y+h>=height-2:
     clipped.append((area,x,y,w,h));continue
    hull=cv2.convexHull(c);q=cv2.approxPolyDP(hull,.02*cv2.arcLength(hull,True),True)
    if len(q)!=4:continue
    q=refine(c,ordered_quad(q));poses=pose_candidates(q,K,D)
    if not poses:continue
    candidates.append({'s':s,'v':v,'q':q.tolist(),'area':area,'bbox':[x,y,w,h],'rms':poses[0]['face_reprojection_rms_px'],'poses':poses})
 if not candidates:return {'status':'clipped' if clipped else 'no_complete_face','clipped':clipped}
 # Highest area cluster first, avoids small blue clutter. Within that cluster,
 # minimize purely image reprojection score, with no use of FK residuals.
 largest=max(x['area'] for x in candidates);candidates=[x for x in candidates if x['area']>=largest*.65]
 candidates.sort(key=lambda x:x['rms']);best=candidates[0]
 close=[x for x in candidates if np.max(np.linalg.norm(np.array(x['q'])-best['q'],axis=1))<5]
 return {'status':'complete_face_candidate','best':best,'near_support':len(close),'all_count':len(candidates),'candidate_diagnostics':[{'S':x['s'],'V':x['v'],'quad_px':x['q'],'reprojection_rms_px':x['rms']} for x in candidates]}

def assess(im,K,D):
    result=detect(im,K,D)
    if result['status']!='complete_face_candidate':
        return {'status':'no_complete_face','reason':'Blue face contour reaches image boundary' if result['status']=='clipped' else 'No complete four-corner face',
                'dim_face_assessment':result,'variants':[]}
    best=result['best']
    accepted=best['rms']<=2. and result['near_support']>=3
    poses=best['poses']
    return {'status':'complete_face_candidate' if accepted else 'poor_boundary_quality',
            'reason':None if accepted else 'Refined face reprojection >2px or insufficient stable boundary support',
            'face_quad_px':best['q'],'poses':poses,
            'face_identity':'automatic dim-blue face candidate; no FK-based selection',
            'pose_ambiguity_mm':float(np.linalg.norm(np.array(poses[0]['cube_center_Camera_mm'])-poses[1]['cube_center_Camera_mm'])) if len(poses)==2 else None,
            'dim_face_assessment':result,'variants':[]}
