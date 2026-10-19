#!/usr/bin/env python3
"""Offline initial extrinsics using the user's cube-bottom midpoint assumption."""

import sys as _sys
from pathlib import Path as _Path
_sys.path.insert(0, str(_Path(__file__).resolve().parents[2]))
from host.project_paths import ROOT as REPO_ROOT, bootstrap as _bootstrap
_bootstrap(include_tools=True)
import json
from pathlib import Path
import numpy as np
from validate_grasp_correspondences import rigid

HERE=(REPO_ROOT/'data/k230')
OUT=HERE/'grasp_pairs/analysis_20261013'

def main():
    collection=json.loads((HERE/'grasp_pairs/collection.json').read_text())
    assumption=collection['grasp_reference_assumption']
    if not assumption['tip_end_midpoint_is_cube_bottom_center']:
        raise RuntimeError('Cube bottom-center reference is not confirmed')
    rows=[r for r in json.loads((OUT/'groups_analysis.json').read_text())
          if r['group'] in assumption['groups'] and r['vision'].get('poses')]
    ids=[r['group'] for r in rows]
    camera=np.array([r['vision']['poses'][0]['cube_center_Camera_mm'] for r in rows])
    bottom=np.array([r['fk']['tip_midpoint_Base_mm'] for r in rows])
    offset=np.array(assumption['cube_center_offset_Base_mm'],float)
    center=bottom+offset
    R,t=rigid(camera,center)
    T=np.eye(4);T[:3,:3]=R;T[:3,3]=t
    assert np.allclose(R.T@R,np.eye(3),atol=1e-8) and abs(np.linalg.det(R)-1)<1e-8
    predicted=camera@R.T+t
    errors=np.linalg.norm(predicted-center,axis=1)
    train=[ids.index(g) for g in [1,2,4,5]]
    test=[ids.index(g) for g in [8,9]]
    R4,t4=rigid(camera[train],center[train])
    test_prediction=camera[test]@R4.T+t4
    deltas=test_prediction-center[test]
    validation=[{'group':ids[i],'predicted_cube_center_Base_mm':p.tolist(),
                  'assumed_cube_center_Base_mm':center[i].tolist(),
                  'error_xy_mm':float(np.linalg.norm(d[:2])),
                  'error_3d_mm':float(np.linalg.norm(d))}
                 for i,p,d in zip(test,test_prediction,deltas)]
    Tm=T.copy();Tm[:3,3]/=1000
    baseline={'status':'provisional_baseline_under_user_selected_approximation',
              'camera_device':'CanMV K230 / USB serial 001000000',
              'source_groups':ids,
              'coordinate_convention':'p_Base_mm = R @ p_Camera_mm + translation_mm; Base is current URDF root; Camera is optical X-right Y-down Z-forward',
              'grasp_reference_assumption':assumption,
              'T_Base_from_Camera_mm':T.tolist(),
              'T_Camera_from_Base_mm':np.linalg.inv(T).tolist(),
              'arm_camera_optical_to_Base_m':Tm.tolist(),
              'table_z_Base_m':float(np.median(bottom[:,2])/1000),
              'table_z_Base_mm_model_range':[float(bottom[:,2].min()),float(bottom[:,2].max())],
              'cube_center_z_Base_m_approx':float((np.median(bottom[:,2])+10)/1000),
              'intrinsics_source':str((HERE/'calibration_results/camera_intrinsics.json').resolve()),
              'source_image_size':[1280,960],
              'intrinsics_scale_for_source_images':2,
              'camera_to_base_pose_evidence':collection['camera_to_base_pose_evidence'],
              'training_all6_rms_3d_mm':float(np.sqrt(np.mean(errors**2))),
              'per_group_training_errors_3d_mm':dict(zip(map(str,ids),errors.tolist())),
              'fixed_split_validation':{'training_groups':[1,2,4,5],'validation_groups':[8,9],
                          'groups':validation,'rms_3d_mm':float(np.sqrt(np.mean(np.sum(deltas**2,axis=1))))},
              'limitations':['User-selected approximation links modeled fingertip midpoint to cube-bottom center.',
                             'STL distal-region mean is a proxy for the physical finger endpoints.',
                             'Base Z perpendicular to the tabletop is assumed; table height comes from current FK.',
                             'Single-face planar pose ambiguity and internal FK height variation remain.',
                             'Errors are against assumed correspondences, not measured powered grasp accuracy.'],
              'hardware_commands_sent':False,'formal_project_configuration_changed':False,
              'next_step':'Use practical alignment observations to refine this initial baseline.'}
    target=HERE/'calibration_results/camera_in_base_provisional.json'
    target.write_text(json.dumps(baseline,ensure_ascii=False,indent=2)+'\n')
    (OUT/'cube_bottom_center_calibration.json').write_text(json.dumps(baseline,ensure_ascii=False,indent=2)+'\n')
    print('BASELINE',target)
    print('TRANSLATION_BASE_MM',t.tolist())
    print('TABLE_Z_BASE_MM',float(np.median(bottom[:,2])))
    print('VALIDATION',validation)

if __name__=='__main__':main()
