#!/usr/bin/env python3
"""Offline, fixed-split validation against a provisional FK tip reference."""

import sys as _sys
from pathlib import Path as _Path
_sys.path.insert(0, str(_Path(__file__).resolve().parents[2]))
from host.project_paths import ROOT as REPO_ROOT, bootstrap as _bootstrap
_bootstrap(include_tools=True)
import json
from pathlib import Path
import numpy as np
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation

OUT = (REPO_ROOT/'data/k230')/'grasp_pairs/analysis_20261013'

def rigid(x,y):
    U,_,Vt=np.linalg.svd((x-x.mean(0)).T@(y-y.mean(0)))
    R=Vt.T@U.T
    if np.linalg.det(R)<0:
        Vt[-1]*=-1;R=Vt.T@U.T
    return R,y.mean(0)-R@x.mean(0)

def main():
    rows=json.loads((OUT/'groups_analysis.json').read_text())
    rows=[r for r in rows if r['vision'].get('poses')]
    ids=[r['group'] for r in rows]
    C=np.array([r['vision']['poses'][0]['cube_center_Camera_mm'] for r in rows])
    B=np.array([r['fk']['tip_midpoint_Base_mm'] for r in rows])
    # Fixed split, chosen from collection order, not validation error.
    train=[ids.index(i) for i in [1,2,4,5]]
    test=[ids.index(i) for i in [8,9]]
    R,t=rigid(C[train],B[train])
    prediction=C@R.T+t
    err=np.linalg.norm(prediction-B,axis=1)
    loo=[]
    for i,g in enumerate(ids):
        ix=[j for j in range(len(ids)) if j!=i]
        rr,tt=rigid(C[ix],B[ix]);delta=rr@C[i]+tt-B[i]
        loo.append({'test_group':g,'error_3d_mm':float(np.linalg.norm(delta)),
                    'error_xy_mm':float(np.linalg.norm(delta[:2]))})
    _,_,Vt=np.linalg.svd(C-C.mean(0))
    normal=Vt[-1]
    distances=(C-C.mean(0))@normal
    fixed_R=np.array([r['fk']['T_Base_from_fixed_jaw_m'] for r in rows])[:,:3,:3]
    rr,tt=rigid(C,B)
    def residual(a):
        return (C@Rotation.from_rotvec(a[:3]).as_matrix().T+a[3:6]-B-np.einsum('nij,j->ni',fixed_R,a[6:9])).ravel()
    a0=np.r_[Rotation.from_matrix(rr).as_rotvec(),tt,np.zeros(3)]
    opt=least_squares(residual,a0,max_nfev=1500)
    # Compare offsets as a sensitivity study, not an accepted physical TCP.
    sensitivity=[]
    for dz in [0,-5,-10,-15]:
        reference=B+np.einsum('nij,j->ni',fixed_R,np.array([0,0,dz]))
        r,trans=rigid(C[train],reference[train]);pred=C@r.T+trans
        sensitivity.append({'assumed_offset_local_mm':[0,0,dz],
                            'translation_mm':trans.tolist(),
                            'test_errors_3d_mm':np.linalg.norm(pred[test]-reference[test],axis=1).tolist()})
    collection=json.loads((OUT.parent/'collection.json').read_text())
    result={'analysis_role':'historical_tip_reference_diagnostic',
            'current_baseline_reference':'data/k230/calibration_results/camera_in_base_provisional.json',
            'camera_to_base_pose_evidence':collection.get('camera_to_base_pose_evidence'),
            'training_groups':[1,2,4,5],'validation_groups':[8,9],
            'reference':'Provisional FK gripper-tip midpoint, not verified physical cube center',
            'R':R.tolist(),'translation_mm':t.tolist(),
            'per_group':[{'group':g,'predicted_reference_Base_mm':pred.tolist(),
                          'reference_Base_mm':base.tolist(),'error_3d_mm':float(e),
                          'error_xy_mm':float(np.linalg.norm((pred-base)[:2]))}
                         for g,pred,base,e in zip(ids,prediction,B,err)],
            'validation_rms_3d_mm':float(np.sqrt(np.mean(err[test]**2))),
            'leave_one_out':loo,
            'cube_center_Camera_plane_normal':normal.tolist(),
            'cube_center_Camera_plane_rms_mm':float(np.sqrt(np.mean(distances**2))),
            'grasp_offset_fit_diagnostic':{'offset_local_mm':opt.x[6:].tolist(),
                    'residual_rms_3d_mm':float(np.sqrt(np.mean(np.sum(residual(opt.x).reshape(-1,3)**2,axis=1)))),
                    'not_accepted':True,
                    'reason':'An unconstrained fitted offset is not a physical TCP measurement; posture-dependent model and grasp errors can be absorbed into it.'},
            'offset_sensitivity':sensitivity,
            'deployment_approved':False,'robot_configuration_changed':False}
    (OUT/'rigid_holdout_validation.json').write_text(json.dumps(result,indent=2)+'\n')
    print('VALIDATION_RMS_3D_MM',result['validation_rms_3d_mm'])
    print('TEST_ERRORS',[(ids[i],float(err[i])) for i in test])

if __name__=='__main__':main()
