#!/usr/bin/env python3
"""Offline formal fixed-tip/top-contact extrinsics with a predeclared holdout."""
import sys
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[2]))
from host.project_paths import ROOT,bootstrap
bootstrap(include_tools=True)
import argparse
from datetime import datetime
import hashlib
import json
from zoneinfo import ZoneInfo
import numpy as np


def load(path):return json.loads(path.read_text())
def sha(path):return hashlib.sha256(path.read_bytes()).hexdigest()
def save(path,value):path.write_text(json.dumps(value,ensure_ascii=False,indent=2)+'\n')


def rigid_fit(camera,base):
    x=np.asarray(camera,float);y=np.asarray(base,float)
    if x.shape!=y.shape or x.ndim!=2 or x.shape[1]!=3 or len(x)<3 or not np.isfinite([x,y]).all():
        raise ValueError('Need at least three finite corresponding 3D points')
    xc=x-x.mean(0);yc=y-y.mean(0)
    if np.linalg.matrix_rank(xc)<2 or np.linalg.matrix_rank(yc)<2:
        raise ValueError('Calibration points are collinear')
    u,s,vt=np.linalg.svd(xc.T@yc)
    rotation=vt.T@u.T
    if np.linalg.det(rotation)<0:
        vt[-1]*=-1
        rotation=vt.T@u.T
    transform=np.eye(4)
    transform[:3,:3]=rotation
    transform[:3,3]=y.mean(0)-rotation@x.mean(0)
    return transform,s


def fit_from_split(camera,base,training,validation):
    if len(set(training))!=len(training) or len(set(validation))!=len(validation) or set(training)&set(validation):
        raise ValueError('Training and validation must be unique and disjoint')
    train=[g for g in training if g in camera and g in base]
    test=[g for g in validation if g in camera and g in base]
    transform,singular=rigid_fit([camera[g] for g in train],[base[g] for g in train])
    return transform,singular,train,test


def contact_plane(points):
    points=np.asarray(points,float)
    _,s,vt=np.linalg.svd(points-points.mean(0))
    if len(points)<3 or s[1]<1e-8:
        raise ValueError('Insufficient non-collinear contacts for table normal')
    normal=vt[-1]
    if normal[2]<0:normal=-normal
    offset=float(points.mean(0)@normal)
    return normal,offset,s


def evaluate(transform,camera,base,groups):
    rows=[]
    for group in groups:
        predicted=transform[:3,:3]@np.asarray(camera[group])+transform[:3,3]
        delta=predicted-base[group]
        rows.append({'group':group,'predicted_Base_mm':predicted.tolist(),
            'reference_Base_mm':base[group].tolist(),'delta_mm':delta.tolist(),
            'xy_mm':float(np.linalg.norm(delta[:2])),'xyz_mm':float(np.linalg.norm(delta))})
    if not rows:return {'groups':[],'count':0}
    delta=np.array([r['delta_mm'] for r in rows])
    return {'groups':rows,'count':len(rows),
        'rms_xyz_mm':float(np.sqrt(np.mean(np.sum(delta**2,axis=1)))),
        'rms_xy_mm':float(np.sqrt(np.mean(np.sum(delta[:,:2]**2,axis=1)))),
        'rms_z_mm':float(np.sqrt(np.mean(delta[:,2]**2))),
        'axis_rms_mm':np.sqrt(np.mean(delta**2,axis=0)).tolist(),
        'mean_delta_mm':delta.mean(0).tolist(),
        'max_xyz_mm':max(r['xyz_mm'] for r in rows),'max_xy_mm':max(r['xy_mm'] for r in rows)}


def image_pose(image,K,D,group):
    # Replay exactly the board image pipeline; identity is used only to leave
    # camera coordinates untouched before the external calibration fit.
    from device.k230.pick.vision import localize as runtime_localize
    try:value=runtime_localize(image,K.tolist(),D.tolist(),np.eye(4).tolist())
    except ValueError as exc:value={'status':'no_complete_face','reason':str(exc)}
    if value.get('poses') and len(value['poses'])>1:
        value['pose_ambiguity_mm']=float(np.linalg.norm(np.array(value['poses'][0]['cube_center_Camera_mm'])-value['poses'][1]['cube_center_Camera_mm']))
    return value


def analyze(session,out):
    import cv2
    from host.arm.joint_mapping import JointMapping
    from host.arm.gripper_geometry import PickGeometry
    from tools.capture.collect_fixed_tip_top import summarize
    out.mkdir(parents=True,exist_ok=True)
    if (out/'analysis.json').exists():raise RuntimeError('Output already fitted; choose a new output directory')
    info=load(session/'session.json');split=load(session/'validation_split.json')
    collection=load(session/'collection.json')
    training=split['training_groups'];validation=split['validation_groups']
    if set(training)&set(validation) or sorted(training+validation)!=list(range(1,17)):
        raise RuntimeError('Invalid predeclared 12/4 split')
    sources=[{'path':str(session/name),'sha256':sha(session/name)} for name in ('session.json','collection.json','validation_split.json')]
    for item in info['files']:
        frozen=session/'snapshot'/item['path']
        if sha(frozen)!=item['sha256']:raise RuntimeError('Frozen file changed: '+str(frozen))
        sources.append({'path':str(frozen),'sha256':item['sha256']})
    for name in ('host/arm/gripper_geometry.py','host/arm/urdf_fk.py'):
        if sha(ROOT/name)!=sha(session/'snapshot'/name):raise RuntimeError('FK implementation differs from session snapshot')
    frozen=session/'snapshot'
    geometry=PickGeometry(frozen,JointMapping.load(frozen/'configs/arm/joint_mapping.json'))
    vertices=geometry.vertices['wan2']
    cap=vertices[np.abs(vertices[:,2]-vertices[:,2].max())<1e-8]
    endpoint=cap.astype(float).mean(0)
    # Match the V0 fixed-top proxy; do not silently optimize this point from residuals.
    intr=load(frozen/'configs/vision/runtime/camera_intrinsics.json')
    K=np.array(intr['camera_matrix'],float);K[0]*=1280/intr['image_width'];K[1]*=960/intr['image_height']
    D=np.array(intr['distortion_coefficients'],float)
    fk={};records={};paths={}
    for item in collection['groups']:
        group=item['group'];folder=session/item['folder'];record=load(folder/'record.json')
        if record['status']!='raw_pair_saved_locally' or record['sample_type']!='fixed_fingertip_cube_top_contact' or record['contacting_jaw']!='fixed':
            raise RuntimeError('Incomplete or wrong contact semantics: '+str(group))
        if sha(folder/'frame.jpg')!=record['camera_capture']['sha256']:raise RuntimeError('Raw image changed')
        summary=summarize(record)
        counts=[int(round(summary[str(i)]['median'])) for i in range(1,7)]
        q=geometry.angles(counts);transforms=geometry.metrics(q)[3];transform=transforms['wan2']
        point=(transform@np.r_[endpoint,1])[:3]*1000
        fk[group]={'group':group,'counts':counts,'joint_angles_deg':np.degrees(q).tolist(),
            'fixed_contact_point_Base_mm':point.tolist(),
            'fixed_jaw_transform':transform.tolist(),'encoder_summary':summary,
            'fixed_jaw_tilt_deg':float(np.degrees(np.arccos(np.clip(-transform[2,2],-1,1))))}
        records[group]=record;paths[group]=folder
        sources += [{'path':str(folder/name),'sha256':sha(folder/name)} for name in ('frame.jpg','record.json','board_record.json')]
    if sorted(fk)!=list(range(1,17)):raise RuntimeError('All 16 declared groups must be present')
    save(out/'input_manifest.json',{'session':str(session),'split':split,'inputs':sources,
        'hardware_access':False,'created':datetime.now(ZoneInfo('Asia/Shanghai')).isoformat(),
        'analyzer_sha256':sha(Path(__file__)),
        'vision_source_hashes':{name:sha(ROOT/'host/vision'/name) for name in ('face_geometry.py','grasp_face_quality.py','runtime_cube_faces.py','blue_side_edges.py')}})
    save(out/'encoder_fk.json',list(fk.values()))
    print('RAW_AND_FK_VERIFIED 16; independent holdouts '+str(validation),flush=True)
    vision=[];tiles=[]
    for group in sorted(fk):
        image=cv2.imread(str(paths[group]/'frame.jpg'))
        if image is None or image.shape[:2]!=(960,1280):raise RuntimeError('Wrong image dimensions')
        value=image_pose(image,K,D,group);value['group']=group
        vision.append(value)
        annotation=image.copy()
        if value.get('face_quad_px'):
            quad=np.rint(value['face_quad_px']).astype(np.int32)
            cv2.polylines(annotation,[quad],True,(0,255,0),3)
            for i,pixel in enumerate(quad):cv2.putText(annotation,str(i),tuple(pixel),cv2.FONT_HERSHEY_SIMPLEX,.6,(0,0,255),2)
        cv2.imwrite(str(out/f'group_{group:02d}_annotated.jpg'),annotation)
        tile=cv2.resize(annotation,(480,360));cv2.putText(tile,str(group)+(' VALID' if group in validation else ' TRAIN'),(10,25),cv2.FONT_HERSHEY_SIMPLEX,.6,(0,0,255),2);tiles.append(tile)
        print('VISION',group,value['status'],[round(p['face_reprojection_rms_px'],3) for p in value.get('poses',[])],flush=True)
    cv2.imwrite(str(out/'vision_contact_sheet.jpg'),np.vstack([np.hstack(tiles[i:i+4]) for i in range(0,16,4)]))
    save(out/'vision.json',vision)
    usable={v['group']:v for v in vision if v['status']=='complete_face_candidate' and v.get('poses')}
    train=[g for g in training if g in usable];test=[g for g in validation if g in usable]
    if len(train)<3:raise RuntimeError('Insufficient image-valid training samples; diagnostics retained')
    # Infer the tabletop normal from training contact labels only, never held-out samples.
    normal,offset,plane_sv=contact_plane([fk[g]['fixed_contact_point_Base_mm'] for g in train])
    base={g:np.array(fk[g]['fixed_contact_point_Base_mm'])-10*normal for g in usable}
    camera={g:np.array(usable[g]['poses'][0]['cube_center_Camera_mm']) for g in usable}
    T,s,train,test=fit_from_split(camera,base,training,validation)
    train_metrics=evaluate(T,camera,base,train)
    test_metrics=evaluate(T,camera,base,test)
    oldT=np.array(load(ROOT/'configs/vision/runtime/camera_in_base.json')['T_Base_from_Camera_mm'])
    point=np.array([fk[g]['fixed_contact_point_Base_mm'] for g in train])
    deviations=point@normal-offset
    definition={'fixed_jaw_link':'wan2','local_point_m':endpoint.tolist(),
        'method':'mean of maximum local-Z STL vertices, same V0 fixed-top proxy',
        'physical_point_surveyed':False,'contact_region_variation_accepted_by_user':True,
        'center_offset_mm':(-10*normal).tolist(),'table_up_normal_Base':normal.tolist(),
        'normal_source':'SVD of training-only modeled contact positions, not surveyed',
        'table_plane_equation':'normal dot Base_mm = offset_mm','table_plane_offset_mm':offset-20,
        'plane_contact_rms_mm':float(np.sqrt(np.mean(deviations**2))),
        'plane_contact_max_abs_mm':float(np.max(np.abs(deviations))),
        'normal_tilt_from_BaseZ_deg':float(np.degrees(np.arccos(normal[2]))),
        'plane_singular_values_mm':plane_sv.tolist()}
    save(out/'fixed_point_definition.json',definition)
    result={'status':'candidate_fitted_not_adopted','source_session':str(session),
        'predeclared_training_groups':training,'predeclared_validation_groups':validation,
        'training_groups_used':train,'validation_groups_used':test,
        'image_rejected_groups':[g for g in sorted(fk) if g not in usable],
        'pose_selection':'lowest image reprojection IPPE solution; no FK-residual-based selection',
        'holdout_integrity':'No held-out point used in rigid fit or table normal; no resampling',
        'reference_definition':definition,'training':train_metrics,'validation':test_metrics,
        'T_Base_from_Camera_mm':T.tolist(),'T_Camera_from_Base_mm':np.linalg.inv(T).tolist(),
        'fit_singular_values':s.tolist(),
        'coverage_training_Base_mm':{'min':point.min(0).tolist(),'max':point.max(0).tolist(),'range':np.ptp(point,axis=0).tolist()},
        'old_runtime_on_same_labels':{'training':evaluate(oldT,camera,base,train),'validation':evaluate(oldT,camera,base,test)},
        'limitations':['Manual contacts and CAD endpoint are approximate labels, not surveyed ground truth.',
            'Rounded contact-region shift and joint compliance can cause pose-dependent systematic error.',
            'Image reprojection quality is not independently measured robot accuracy.',
            'Single planar face has two IPPE candidates; both retained, chosen by image score only.',
            'Table normal is inferred from training FK contacts, not independently measured.'],
        'runtime_updated':False,'hardware_access':False}
    save(out/'analysis.json',result)
    save(out/'camera_in_base_candidate.json',{'status':result['status'],
        'T_Base_from_Camera_mm':T.tolist(),'T_Camera_from_Base_mm':np.linalg.inv(T).tolist(),
        'source_training_groups':train,'independent_validation_groups':test,
        'intrinsics_sha256':sha(frozen/'configs/vision/runtime/camera_intrinsics.json'),
        'reference_definition':definition,'training_metrics':train_metrics,
        'validation_metrics':test_metrics,'default_replaced':False})
    for item in sources:
        if sha(Path(item['path']))!=item['sha256']:raise RuntimeError('Input changed during analysis')
    save(out/'verification.json',{'input_hashes_verified':len(sources),'hardware_access':False,
        'validation_excluded_from_fit':not set(train)&set(test),'runtime_updated':False})
    print('FIT_RESULT',json.dumps({k:result[k] for k in ('training_groups_used','validation_groups_used','image_rejected_groups','training','validation','reference_definition')},ensure_ascii=False),flush=True)


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--session',type=Path,required=True)
    parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args()
    analyze(args.session.resolve(),args.output.resolve())


if __name__=='__main__':main()
