"""Separate an exposed blue cube's top/side color seam before side-face IPPE.

Runtime only. No FK, known target position, hardware, or calibration sample edits.
"""

import sys as _sys
from pathlib import Path as _Path
_sys.path.insert(0, str(_Path(__file__).resolve().parents[2]))
from host.project_paths import ROOT as REPO_ROOT, bootstrap as _bootstrap
_bootstrap(include_tools=False)
import cv2
import numpy as np
from face_geometry import ordered_quad,pose_candidates
from grasp_face_quality import refine

def split_exposed_cube(im,K,D):
 hsv=cv2.cvtColor(im,cv2.COLOR_BGR2HSV);H,W=hsv.shape[:2];broad=cv2.inRange(hsv,np.array([90,120,40]),np.array([140,255,255]));broad=cv2.morphologyEx(broad,cv2.MORPH_CLOSE,np.ones((3,3),np.uint8));contours,_=cv2.findContours(broad,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_NONE)
 eligible=[c for c in contours if cv2.contourArea(c)>6000 and cv2.boundingRect(c)[1]>=.45*H]
 if not eligible:return {'status':'no_complete_face','reason':'No exposed blue cube component'}
 blob=max(eligible,key=cv2.contourArea);x,y,w,h=cv2.boundingRect(blob)
 if x<=2 or y<=2 or x+w>=W-2 or y+h>=H-2:return {'status':'no_complete_face','reason':'Blue cube is clipped at image boundary'}
 x0=x+round(w*.2);x1=x+round(w*.8)
 lo=max(5,round(h*.08));hi=min(h-12,round(h*.45));seams=[]
 # Hue separates faces under bright light; value handles similarly colored faces in shade.
 # Compare local contrasts, not a fixed absolute exposure/brightness threshold.
 for channel,label,minimum_gradient,minimum_jump in [(0,'hue',.3,3),(2,'value',1.,35)]:
  profile=np.median(hsv[y:y+h,x0:x1,channel],axis=1).astype(np.float32)
  smooth=cv2.GaussianBlur(profile.reshape(-1,1),(1,5),0).ravel();gradient=np.abs(smooth[2:]-smooth[:-2])/2
  j=int(np.argmax(gradient[lo:hi]))+lo+1;candidate_y=y+j
  upper=hsv[candidate_y-12:candidate_y-4,x0:x1];lower=hsv[candidate_y+4:candidate_y+12,x0:x1]
  hue_jump=float(abs(np.median(upper[:,:,0])-np.median(lower[:,:,0])));value_jump=float(abs(np.median(upper[:,:,2])-np.median(lower[:,:,2])))
  jump=hue_jump if channel==0 else value_jump
  if float(gradient[j-1])>=minimum_gradient and jump>=minimum_jump:
   seams.append({'channel':channel,'label':label,'y':candidate_y,'minimum_gradient':minimum_gradient,'score':float(gradient[j-1]),'hue_jump':hue_jump,'value_jump':value_jump})
 if not seams:return {'status':'no_complete_face','reason':'No stable hue or brightness top/side seam'}
 # Preserve hue results where they already had a coherent color change.
 selected=seams[0];seam=selected['y'];seam_channel=selected['channel'];hue_jump=selected['hue_jump'];value_jump=selected['value_jump']
 # Estimate the same seam across columns; require spatial support, not a single-pixel edge.
 edges=[]
 for xx in range(x0,x1):
  col=hsv[seam-10:seam+11,xx,seam_channel].astype(np.float32);col=cv2.GaussianBlur(col.reshape(-1,1),(1,5),0).ravel();g=np.abs(col[2:]-col[:-2])/2;k=int(np.argmax(g))+1
  if g[k-1]>=selected['minimum_gradient']:edges.append([xx,seam-10+k])
 if len(edges)<max(15,(x1-x0)//3):return {'status':'no_complete_face','reason':'Top/side seam lacks coherent column support'}
 points=np.array(edges,np.float32);vx,vy,px,py=cv2.fitLine(points,cv2.DIST_HUBER,0,.01,.01).ravel();slope=float(vy/vx) if abs(vx)>1e-6 else float('inf')
 if not np.isfinite(slope) or abs(slope)>.35:return {'status':'no_complete_face','reason':'No near-horizontal exposed side seam'}
 intercept=float(py-slope*px);deviation=float(np.median(np.abs(points[:,1]-(points[:,0]*slope+intercept))))
 if deviation>3:return {'status':'no_complete_face','reason':'Top/side seam is unstable','seam_deviation_px':deviation}
 yy=np.arange(H)[:,None];xx=np.arange(W)[None,:];below=yy>=xx*slope+intercept;found=[]
 for s in [80,120,160,200]:
  for val in [40,80,120,160]:
   mask=cv2.inRange(hsv,np.array([90,s,val]),np.array([140,255,255]));mask=cv2.morphologyEx(mask,cv2.MORPH_CLOSE,np.ones((3,3),np.uint8));mask[~below]=0
   # Keep only the current component neighborhood, excludes other blue objects.
   keep=np.zeros_like(mask);keep[max(0,y-2):min(H,y+h+2),max(0,x-2):min(W,x+w+2)]=mask[max(0,y-2):min(H,y+h+2),max(0,x-2):min(W,x+w+2)]
   cs,_=cv2.findContours(keep,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_NONE)
   if not cs:continue
   c=max(cs,key=cv2.contourArea)
   if cv2.contourArea(c)<6000:continue
   hull=cv2.convexHull(c);q=cv2.approxPolyDP(hull,.02*cv2.arcLength(hull,True),True)
   if len(q)!=4:continue
   q=refine(c,ordered_quad(q));ps=pose_candidates(q,K,D)
   if not ps:continue
   found.append({'S':s,'V':val,'q':q.tolist(),'rms':float(ps[0]['face_reprojection_rms_px']),'poses':ps})
 if not found:return {'status':'no_complete_face','reason':'No complete side after separating top face'}
 found.sort(key=lambda a:a['rms']);best=found[0];support=sum(np.max(np.linalg.norm(np.array(c['q'])-best['q'],axis=1))<5 for c in found)
 accepted=best['rms']<=2 and support>=3
 return {'status':'complete_face_candidate' if accepted else 'poor_boundary_quality','reason':None if accepted else 'Separated side still fails image boundary quality',
 'boundary_method':'exposed_cube_top_side_seam_then_side_edges','face_quad_px':best['q'],'poses':best['poses'],'near_support':int(support),'candidate_count':len(found),
 'seam_line_px':{'channel':selected['label'],'score':selected['score'],'slope':slope,'intercept':intercept,'column_support':len(edges),'median_deviation_px':deviation,'hue_jump':hue_jump,'value_jump':value_jump},
 'bbox_px':[x,y,w,h],'candidates':[{'S':f['S'],'V':f['V'],'quad_px':f['q'],'reprojection_rms_px':f['rms']} for f in found]}


def verify_single_face(im,K,D,candidate):
 """Check interior physical seams even when an outline passed square PnP."""
 from device.k230.pick.face_selection import internal_seam,separated_face_inside,adjacent_side,independent_adjacent_side,shallow_face
 if candidate.get('status')!='complete_face_candidate' or not candidate.get('poses'):return candidate
 separated=None
 if shallow_face(candidate['face_quad_px']):
  separated=split_exposed_cube(im,K,D)
  if not (separated.get('status')=='complete_face_candidate' and separated.get('poses') and adjacent_side(candidate['face_quad_px'],separated['face_quad_px'])):
   independent=independent_adjacent_side(im,K,D,candidate['face_quad_px'])
   if independent is not None:separated=independent
  if separated.get('status')=='complete_face_candidate' and separated.get('poses') and adjacent_side(candidate['face_quad_px'],separated['face_quad_px']):
   separated['preferred_over_top']={'face_quad_px':candidate['face_quad_px'],
      'reprojection_rms_px':candidate['poses'][0]['face_reprojection_rms_px'],
      'reason':'Verified adjacent physical side has larger image extent than bright top'}
   return separated
 evidence=internal_seam(cv2.cvtColor(im,cv2.COLOR_BGR2HSV),candidate['face_quad_px'])
 if evidence is None:return candidate
 if separated is None:separated=split_exposed_cube(im,K,D)
 previous={'boundary_method':candidate.get('boundary_method'),'face_quad_px':candidate['face_quad_px'],
           'reprojection_rms_px':candidate['poses'][0]['face_reprojection_rms_px'],'interior_seam':evidence}
 if separated.get('status')=='complete_face_candidate' and separated.get('poses') and separated_face_inside(candidate['face_quad_px'],separated['face_quad_px']):
  separated['rejected_multiface_outline']=previous
  return separated
 return {'status':'no_complete_face','reason':'Interior blue seam requires a verified individual physical face',
         'rejected_multiface_outline':previous,'separation_attempt':separated}
