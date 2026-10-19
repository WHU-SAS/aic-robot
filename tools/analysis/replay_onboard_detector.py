
import sys as _sys
from pathlib import Path as _Path
_sys.path.insert(0, str(_Path(__file__).resolve().parents[2]))
from host.project_paths import ROOT as REPO_ROOT, bootstrap as _bootstrap
_bootstrap(include_tools=True)
import sys,json,time,cv2,numpy as np
from pathlib import Path
r=REPO_ROOT;sys.path[:0]=[str(r/'host/vision'),str(r/'device/k230/vision')]
from blue_face_core import extract
from localize_board_faces import localize
out=Path('/tmp/k230_onboard_validation');out.mkdir(exist_ok=True);summary=[]
for suffix in ['162836_777041','163107_169299','161939_257286','162114_721702','162458_735322','160424_793307']:
 folder=r/'data/logs'/('k230_pick_20261013_'+suffix);im=cv2.imread(str(folder/'frame.jpg'));lab=cv2.cvtColor(im,cv2.COLOR_BGR2LAB)
 # Replay approximates CanMV LAB blob selection; firmware color conversion is not identical.
 mask=cv2.inRange(lab,np.array([0,108,0]),np.array([255,255,120]));mask[:432]=0
 cs,_=cv2.findContours(mask,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE);cs=[c for c in cs if cv2.contourArea(c)>=1000 and cv2.boundingRect(c)[2]*cv2.boundingRect(c)[3]>=6000];co=max(cs,key=cv2.contourArea);bbox=cv2.boundingRect(co)
 rgb=cv2.cvtColor(im,cv2.COLOR_BGR2RGB);rgb[:,:,0]=(rgb[:,:,0]>>3)<<3;rgb[:,:,1]=(rgb[:,:,1]>>2)<<2;rgb[:,:,2]=(rgb[:,:,2]>>3)<<3
 result=extract(lambda x,y:tuple(int(v) for v in rgb[y,x]),bbox);payload={'schema':'aic.blue_faces.v1','backend':'canmv_native','width':1280,'height':960,'hmirror':False,'vflip':False,'frames':[result,result,result],'robot_commands_sent':False,'image_exported':False,'replay_note':'Single saved JPEG repeated for localization integration only; not a real three-frame stability test or firmware LAB parity test.'}
 p=out/suffix;p.mkdir(exist_ok=True);feature=p/'board_features.json';feature.write_text(json.dumps(payload));t=time.perf_counter()
 try:
  v=localize(feature,p);row={'run':suffix,'status':'localized','Base_mm':v['cube_center_Base_mm'],'selected_kind':v['vision']['selected']['kind'],'selected_value_floor':v['vision']['selected']['value_floor'],'face_disagreement_mm':v['vision']['face_center_disagreement_mm'],'payload_bytes':len(json.dumps(payload).encode()),'PC_pose_s':time.perf_counter()-t};print(row)
 except RuntimeError as exc:row={'run':suffix,'status':'failed','reason':str(exc)};print(row)
 summary.append(row)
(out/'summary.json').write_text(json.dumps(summary,indent=2))
