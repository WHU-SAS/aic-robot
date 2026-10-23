#!/usr/bin/env python3
"""Offline fixed-point pivot audit. Never updates runtime geometry or calibration."""
import sys
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[2]))
import argparse
import hashlib
import json
import numpy as np
from host.project_paths import ROOT
from tools.capture.collect_fixed_tip_top import summarize, save


def fit_pivot(transforms, training=(1,2,3), validation=(4,)):
    if len(training)<3 or not validation or len(set(training+validation))!=len(training+validation):
        raise ValueError('Need at least three training poses and disjoint independent validation')
    matrices={i:np.asarray(T,dtype=float) for i,T in transforms.items()}
    for T in matrices.values():
        if (T.shape!=(4,4) or not np.isfinite(T).all() or
            not np.allclose(T[3],[0,0,0,1]) or
            not np.allclose(T[:3,:3].T@T[:3,:3],np.eye(3),atol=1e-6) or
            not np.isclose(np.linalg.det(T[:3,:3]),1,atol=1e-6)):
            raise ValueError('Invalid rigid transform')
    # T translations are metres; solve and report both points in millimetres.
    A=np.vstack([np.c_[matrices[i][:3,:3],-np.eye(3)] for i in training])
    b=np.concatenate([-matrices[i][:3,3]*1000 for i in training])
    x,unused,rank,singular=np.linalg.lstsq(A,b,rcond=None)
    if rank!=6:
        raise ValueError('Pivot not observable: vary both wrist roll and pitch, not just one rotation axis')
    rows=[]
    for i in training+validation:
        T=matrices[i]
        predicted=T[:3,:3]@x[:3]+T[:3,3]*1000
        delta=predicted-x[3:]
        rows.append({'group':i,'split':'training' if i in training else 'validation',
            'predicted_contact_Base_mm':predicted.tolist(),
            'delta_mm':delta.tolist(),'error_mm':float(np.linalg.norm(delta))})
    return {'fixed_jaw_link':'wan2','fitted_local_point_mm':x[:3].tolist(),
            'fitted_pivot_Base_mm':x[3:].tolist(),'rank':int(rank),
            'singular_values':singular.tolist(),'condition_number':float(singular[0]/singular[-1]),
            'training_groups':list(training),'validation_groups':list(validation),'groups':rows}


def audit(session):
    from host.arm.joint_mapping import JointMapping
    from host.arm.gripper_geometry import PickGeometry
    load=lambda p:json.loads(p.read_text())
    sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
    info=load(session/'session.json')
    if info.get('schema')!='aic.tool_pivot_session.v1' or info.get('sample_type')!='fixed_point_pivot_contact':
        raise ValueError('Only a dedicated same-point/same-pivot session can be fitted')
    frozen=session/'snapshot'
    for item in info['files']:
        if sha(frozen/item['path'])!=item['sha256']:raise ValueError('Changed frozen source: '+item['path'])
    for name in ('host/arm/gripper_geometry.py','host/arm/urdf_fk.py','host/arm/joint_mapping.py'):
        if sha(ROOT/name)!=sha(frozen/name):raise ValueError('Current FK differs from frozen implementation: '+name)
    geometry=PickGeometry(frozen,JointMapping.load(frozen/'configs/arm/joint_mapping.json'))
    transforms={};feedback={};inputs=[]
    for entry in load(session/'collection.json')['groups']:
        if entry['status']!='raw_pair_saved_locally':continue
        group=entry['group'];folder=session/entry['folder'];record=load(folder/'record.json')
        if group in transforms:raise ValueError('Duplicate completed group')
        if record.get('sample_type')!='fixed_point_pivot_contact' or record['status']!='raw_pair_saved_locally':
            raise ValueError('Wrong or incomplete sample')
        if sha(folder/'frame.jpg')!=record['camera_capture']['sha256']:raise ValueError('Raw image changed')
        summary=summarize(record)
        if any(any(t!=0 for t in axis['torque_states']) for axis in summary.values()):
            raise ValueError('Pivot sample has torque enabled')
        # Preserve half-count medians by converting individual integer feedback then taking median.
        q=np.array([np.median([geometry.mapping.count_to_radians(i,s[str(i)]['position'])
                   for s in record['samples_before']+record['samples_after']]) for i in range(1,7)])
        transforms[group]=geometry.metrics(q)[3]['wan2']
        feedback[group]={'encoder_summary':summary,'joint_angles_deg':np.degrees(q).tolist()}
        inputs.extend({'path':str(folder/name),'sha256':sha(folder/name)} for name in ('record.json','frame.jpg','board_record.json'))
    training=tuple(info['training_groups']);validation=tuple(info['validation_groups'])
    if set(transforms)!=set(training+validation):raise ValueError('All declared training and validation groups are required')
    result=fit_pivot(transforms,training,validation)
    vertices=geometry.vertices['wan2'];cap=vertices[np.abs(vertices[:,2]-vertices[:,2].max())<1e-8]
    references={'calibration_tip':cap.mean(0)*1000,'runtime_distal_2mm_tip':geometry.tip['wan2']*1000}
    p=np.array(result['fitted_local_point_mm'])
    result.update({'schema':'aic.tool_pivot_audit.v1','session':str(session),
        'reference_comparison':{name:{'local_point_mm':point.tolist(),
            'fitted_minus_reference_mm':(p-point).tolist(),'distance_mm':float(np.linalg.norm(p-point))}
            for name,point in references.items()},'feedback':feedback,'input_files':inputs,
        'runtime_changed':False,'camera_used_for_fit':False,
        'interpretation':'Valid only if the marked material point and target corner remained identical; FK, backlash and compliance errors can appear as reference error. A four-pose fit alone does not validate whole finger geometry.'})
    return result


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--session',type=Path,default=ROOT/'data/k230/tool_reference_pivot_20261023')
    parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args()
    if args.output.exists():raise RuntimeError('Choose a new output path to preserve earlier evidence')
    result=audit(args.session.resolve())
    args.output.parent.mkdir(parents=True,exist_ok=True)
    save(args.output,result)
    print(json.dumps(result,ensure_ascii=False,indent=2))


if __name__=='__main__':main()
