#!/usr/bin/env python3
"""Offline tabletop-contact constraints confirmed by the operator.

Known: at least one fingertip contacts the table, cube remains on table,
cube edge is 20mm. Cube XY still uses a provisional gripper reference.
No robot control, joint-map changes or adopted calibration writes.
"""
import json
from pathlib import Path
import sys
import numpy as np

HERE=Path(__file__).resolve().parent
OUT=HERE/'grasp_pairs/analysis_20261013'
PROJECT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(PROJECT/'hardware'))
sys.path.insert(0,str(PROJECT/'ros2_ws/src/arm_pkg'))
from arm_pkg.joint_mapping import JointMapping
from gripper_geometry import PickGeometry

def main():
    confirmed=[1,2,4,5,8,9]
    rows=[r for r in json.loads((OUT/'groups_analysis.json').read_text()) if r['group'] in confirmed]
    geometry=PickGeometry(PROJECT,JointMapping.load(PROJECT/'hardware/joint_mapping.json'))
    contacts=[]
    for row in rows:
        q=geometry.angles(row['fk']['encoder_counts'])
        midpoint,tips,lows,transforms=geometry.metrics(q)
        per_finger=[]
        for link,vertices in geometry.vertices.items():
            T=transforms[link]
            points=vertices@T[:3,:3].T+T[:3,3]
            index=int(np.argmin(points[:,2]))
            distal=vertices[:,2]>=vertices[:,2].max()-.002
            per_finger.append({'link':link,'lowest_point_Base_mm':(points[index]*1000).tolist(),
                               'lowest_is_distal_tip':bool(distal[index]),
                               'tip_mean_Base_mm':(tips[link]*1000).tolist()})
        contacts.append({'group':row['group'],
                         'user_confirmed':'Fingertip touches table; exact contacting finger(s) not individually identified',
                         'fingers':per_finger,
                         'table_z_Base_mm_inferred_from_lowest_model_tip':float(min(lows.values())*1000),
                         'lowest_model_tip_used':'Minimum across both STL fingers; actual finger not measured'})
    ids=[r['group'] for r in rows]
    C=np.array([r['vision']['poses'][0]['cube_center_Camera_mm'] for r in rows])
    B=np.array([r['fk']['tip_midpoint_Base_mm'] for r in rows])
    floor=np.array([r['table_z_Base_mm_inferred_from_lowest_model_tip'] for r in contacts])
    train=np.array([g in [1,2,4,5] for g in ids])
    Ct=C[train]
    _,_,Vt=np.linalg.svd(Ct-Ct.mean(0))
    normal=Vt[-1]
    prior=np.array(json.loads((OUT/'rigid_holdout_validation.json').read_text())['R'])[2]
    if normal@prior<0:normal*=-1
    # Tabletop is horizontal in the assumed Base frame. Determine roll/pitch
    # from the camera cube-center plane, yaw and XY translation from FK labels.
    axis=np.array([1.,0,0]);u=axis-(axis@normal)*normal;u/=np.linalg.norm(u)
    v=np.cross(normal,u);R0=np.array([u,v,normal]);X=C@R0.T
    U,_,Vt=np.linalg.svd((X[train,:2]-X[train,:2].mean(0)).T@(B[train,:2]-B[train,:2].mean(0)))
    Q=Vt.T@U.T
    if np.linalg.det(Q)<0:Vt[-1]*=-1;Q=Vt.T@U.T
    R=np.eye(3);R[:2,:2]=Q;R=R@R0
    table_z=float(np.median(floor[train]));cube_z=table_z+10
    translation=np.r_[B[train,:2].mean(0)-(C@R.T)[train,:2].mean(0),cube_z-Ct.mean(0)@normal]
    predicted=C@R.T+translation
    result={'status':'height_constrained_candidate_only',
            'analysis_role':'historical_before_bottom_center_approximation',
            'current_baseline_reference':'k230/calibration_results/camera_in_base_provisional.json',
            'confirmed_groups':confirmed,'cube_edge_mm':20,'cube_center_above_table_mm':10,
            'contacts':contacts,
            'table_z_Base_mm_all_groups_median':float(np.median(floor)),
            'model_contact_z_min_mm':float(floor.min()),'model_contact_z_max_mm':float(floor.max()),
            'model_contact_z_span_mm':float(np.ptp(floor)),
            'contact_rms_about_median_mm':float(np.sqrt(np.mean((floor-np.median(floor))**2))),
            'training_groups':[1,2,4,5],'validation_groups':[8,9],
            'table_z_Base_mm_training_median':table_z,
            'cube_center_z_Base_mm_training_constraint':cube_z,
            'table_up_normal_Camera':normal.tolist(),
            'camera_origin_height_above_table_candidate_mm':float(10-Ct.mean(0)@normal),
            'R_Base_from_Camera_candidate':R.tolist(),'translation_Base_mm_candidate':translation.tolist(),
            'validation':[{'group':g,'predicted_center_Base_mm':p.tolist(),
                           'xy_error_to_provisional_tip_reference_mm':float(np.linalg.norm(p[:2]-b[:2])),
                           'center_height_plane_residual_mm':float(p[2]-cube_z)}
                          for g,p,b in zip(ids,predicted,B) if g in [8,9]],
            'limitations':['At least one finger contacts the table; which finger(s) is not separately confirmed.',
                           'Table Z inferred through current CAD/FK has a roughly 5mm spread.',
                           'Base Z perpendicular to tabletop is a model assumption.',
                           'XY references remain gripper-tip midpoints, not independently measured cube centers.',
                           'Single-face camera pose ambiguity remains.',
                           'The constrained height is not an independent measured robot-accuracy check.'],
            'robot_configuration_changed':False,'deployment_approved':False}
    (OUT/'tip_table_constraints.json').write_text(json.dumps(result,ensure_ascii=False,indent=2)+'\n')
    print(json.dumps({'table_z_median_mm':result['table_z_Base_mm_all_groups_median'],
                      'contact_spread_mm':result['model_contact_z_span_mm'],
                      'validation':result['validation']},ensure_ascii=False))

if __name__=='__main__':main()
