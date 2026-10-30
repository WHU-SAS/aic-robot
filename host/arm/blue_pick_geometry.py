"""K230 pick IK/FK; align the nominal20mm contact midpoint per user choice."""

import sys as _sys
from pathlib import Path as _Path
_sys.path.insert(0, str(_Path(__file__).resolve().parents[2]))
from host.project_paths import ROOT as REPO_ROOT, bootstrap as _bootstrap
_bootstrap(include_tools=False)
import math
import numpy as np
from scipy.optimize import least_squares,brentq
from host.arm.urdf_fk import URDFForwardKinematics
from gripper_geometry import stl_vertices

class PickGeometry:
 def __init__(self,root,mapping,table_z_m=0.,hover_tip_height_mm=80.,descend_tip_height_mm=25.,lift_tip_height_mm=80.,opening_count=2389,urdf_path=None):
  self.table_z_m=float(table_z_m)
  self.hover_height_m=float(hover_tip_height_mm)/1000
  self.descend_height_m=float(descend_tip_height_mm)/1000
  self.lift_height_m=float(lift_tip_height_mm)/1000
  if not all(math.isfinite(v) and v>=0 for v in [self.hover_height_m,self.descend_height_m,self.lift_height_m]):raise ValueError("Tip heights must be finite nonnegative model distances")
  self.opening_count=int(opening_count)
  self.opening_angle_rad=mapping.count_to_radians(6,self.opening_count)
  mapping.radians_to_count(6,self.opening_angle_rad)
  self.mapping=mapping;self.names=[mapping.joint(i)['urdf_joint_name'] for i in range(1,7)]
  model_file=_Path(urdf_path) if urdf_path is not None else _Path('models/genkiarm/genkiarm.urdf')
  self.fk=URDFForwardKinematics(str(root/model_file))
  self.vertices={link:np.unique(stl_vertices(root/'models/genkiarm/meshes'/name),axis=0) for link,name in [('wan2','EEE.stl'),('zhua','FFF.stl')]}
  self.distal={link:v[v[:,2]>=v[:,2].max()-.002] for link,v in self.vertices.items()}
  self.tip={link:v.mean(axis=0) for link,v in self.distal.items()}
 def angles(self,counts):return np.array([self.mapping.count_to_radians(i,int(counts[i-1])) for i in range(1,7)])
 def counts(self,q):return [self.mapping.radians_to_count(i,float(q[i-1])) for i in range(1,7)]
 def metrics(self,q):
  ang=dict(zip(self.names,q));points={};low={};trans={}
  for link,v in self.vertices.items():
   t=self.fk.forward_kinematics(ang,target_link=link)[0];trans[link]=t
   points[link]=(t@np.r_[self.tip[link],1])[:3];low[link]=float(np.min(v@t[2,:3]+t[2,3]))
  return (points['wan2']+points['zhua'])/2,points,low,trans
 def nominal_contact(self,q):
  # Fixed-jaw orientation is independent of the actual gripper opening.
  T=self.fk.forward_kinematics(dict(zip(self.names,q)),target_link='wan2')[0]
  axis=T[:2,1];length=float(np.linalg.norm(axis))
  if length<1e-8:raise RuntimeError('Gripper across-axis has no horizontal projection')
  axis=axis/length;width=.020*(abs(axis[0])+abs(axis[1]))
  fixed=self.distal['wan2'];moving=self.distal['zhua']
  def gap(a):return float(fixed[:,1].min()-np.max(moving[:,1]*math.cos(a)-moving[:,2]*math.sin(a)-.0132))
  # Preserve project1's20mm projected-width formula and0.7mm nominal compression.
  angle=brentq(lambda a:gap(a)-(width-.0007),0,math.pi/2)
  return angle,width
 def alignment_midpoint(self,q):
  # User-authorized reference: expected midpoint when gripping the20mm cube.
  # Use a continuous angle inside IK; keep actual opening/extra tightening separate.
  reference_q=np.array(q,dtype=float,copy=True);reference_q[5]=self.nominal_contact(reference_q)[0]
  return self.metrics(reference_q)[0]
 def report(self,counts):
  q=self.angles(counts);mid,pts,low,_=self.metrics(q);angle,width=self.nominal_contact(q)
  return {'reference':'actual midpoint of mean distal2mm STL tip regions, provisional',
   'alignment_reference':'nominal_20mm_contact_midpoint_without_legacy_offset',
   'alignment_nominal_contact_count':self.mapping.radians_to_count(6,angle),
   'estimated_cube_projection_mm':width*1000,'alignment_center_Base_mm':(self.alignment_midpoint(q)*1000).tolist(),
   'gripper_midpoint_Base_mm':(mid*1000).tolist(),'fixed_tip_center_Base_mm':(pts['wan2']*1000).tolist(),
   'moving_tip_center_Base_mm':(pts['zhua']*1000).tolist(),'link_min_z_mm':{k:v*1000 for k,v in low.items()},
   'table_z_Base_mm':self.table_z_m*1000,'link_clearance_above_table_mm':{k:(v-self.table_z_m)*1000 for k,v in low.items()}}
 def solve(self,xy,height,tilt,seed,q5,q6):
  az=math.atan2(xy[1],xy[0]+.013);sumgoal=math.radians(180-tilt);results=[]
  def residual(a):
   q=np.r_[a,q5,q6];alignment=self.alignment_midpoint(q);low=self.metrics(q)[2]
   return np.r_[(alignment[:2]-xy)/.001,(min(low.values())-(self.table_z_m+height))/.001,(sum(a[1:])-sumgoal)/.005]
  for init in [seed[:4],np.r_[az,np.radians([30,50,180-tilt-80])],np.r_[az,np.radians([45,40,180-tilt-85])]]:
   sol=least_squares(residual,np.clip(init,-math.pi/2+1e-5,math.pi/2-1e-5),bounds=([-math.pi/2]*4,[math.pi/2]*4),max_nfev=200)
   q=np.r_[sol.x,q5,q6];counts=self.counts(q);qr=self.angles(counts);low=self.metrics(qr)[2];alignment=self.alignment_midpoint(qr)
   if np.linalg.norm(alignment[:2]-xy)<.002 and abs(min(low.values())-(self.table_z_m+height))<.002:
    results.append((float(np.linalg.norm(alignment[:2]-xy)),counts,qr))
  if not results:raise RuntimeError('No IK solution within joint limits for requested stage')
  results.sort(key=lambda v:v[0]);return results[0][1],results[0][2]
 def plan(self,xy,start):
  q0=self.angles(start);open_count=self.opening_count;q6=self.mapping.count_to_radians(6,open_count);q5=q0[4];failures=[]
  # Same tilt choices/preference as project1; require a complete four-stage solution.
  for tilt in sorted([20,25,30,35,40,45],key=lambda t:abs(t-25)):
   try:
    hover,qh=self.solve(xy,self.hover_height_m,tilt,q0,q5,q6);low,ql=self.solve(xy,self.descend_height_m,tilt,qh,q5,q6)
    angle,width=self.nominal_contact(ql);nominal=self.mapping.radians_to_count(6,angle);close_count=nominal-20
    close=low.copy();close[5]=close_count
    lift,qu=self.solve(xy,self.lift_height_m,tilt,ql,q5,q6);lift[5]=close_count
    return {'hover':hover,'descend':low,'close':close,'lift':lift,'tilt_deg':tilt,'opening_count':open_count,
     'opening_angle_deg':math.degrees(q6),'closing_count':close_count,'nominal_closing_count':nominal,'extra_closing_counts':20,
     'alignment_reference':'nominal_20mm_contact_midpoint_without_legacy_offset','cube_offset_toward_moving_finger_mm':0,
     'estimated_cube_projection_mm':width*1000,'low_min_tip_model_mm':self.descend_height_m*1000,'hover_min_tip_model_mm':self.hover_height_m*1000,'lift_min_tip_model_mm':self.lift_height_m*1000,'model_correction_applied':False,
     'physical_low_height_is_estimate':True,'table_z_Base_mm':self.table_z_m*1000,
     'target_FK':{name:self.report(c) for name,c in [('hover',hover),('descend',low),('close',close),('lift',lift)]}}
   except (RuntimeError,ValueError) as exc:failures.append({'tilt':tilt,'error':str(exc)})
  raise RuntimeError('No complete hover/descend/close/lift IK solution: '+str(failures))
