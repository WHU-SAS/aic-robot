#!/usr/bin/env python3
"""Compare frozen K230 grasp pairs offline; never opens a hardware port.

Uses the existing OpenCV face/PnP and fingertip FK definitions unchanged.
Keeps old baseline, raw data and live collection index untouched. Host and
vision phases run in their separate project environments.
"""

import sys as _sys
from pathlib import Path as _Path
_sys.path.insert(0, str(_Path(__file__).resolve().parents[2]))
from host.project_paths import ROOT as REPO_ROOT, bootstrap as _bootstrap
_bootstrap(include_tools=True)
import argparse
from datetime import datetime
import hashlib
import json
from pathlib import Path
import shutil
import subprocess
import sys
from zoneinfo import ZoneInfo

PROJECT = REPO_ROOT
OLD_GROUPS = [1, 2, 4, 5, 8, 9]
NEW_GROUPS = list(range(11, 19))
HOLDOUT_NEW_GROUPS = [17, 18]

def read(path): return json.loads(path.read_text())
def write(path, value): path.write_text(json.dumps(value, ensure_ascii=False, indent=2) + '\n')
def digest(path): return hashlib.sha256(path.read_bytes()).hexdigest()

def freeze(out):
    root = PROJECT / 'data/k230/grasp_pairs'
    collection = read(root / 'collection.json')
    wanted = OLD_GROUPS + NEW_GROUPS
    mapping = {g['group']: g for g in collection['groups']}
    if not all(g in mapping for g in wanted): raise RuntimeError('Requested group missing')
    out.mkdir(parents=True, exist_ok=False)
    snap = out / 'snapshot'; snap.mkdir()
    files=[]
    for group in wanted:
        source = root / mapping[group]['folder']
        target = snap / f'group_{group:02d}'; target.mkdir()
        record=read(source/'record.json')
        if len(record['samples_before'])!=3 or len(record['samples_after'])!=3:
            raise RuntimeError(f'Incomplete feedback group {group}')
        if digest(source/'frame.jpg') != record['camera_capture']['sha256']:
            raise RuntimeError(f'Image hash mismatch group {group}')
        for name in ['record.json','frame.jpg','face_candidate.json']:
            src=source/name
            if src.exists():
                before=digest(src);shutil.copy2(src,target/name)
                if digest(target/name)!=before or digest(src)!=before:
                    raise RuntimeError(f'Input changed during snapshot: {src}')
                files.append({'source':str(src),'snapshot':str((target/name).relative_to(out)), 'sha256':before})
    for src, name in [(PROJECT/'data/k230/calibration_results/camera_intrinsics.json','intrinsics.json'),
                      (PROJECT/'data/k230/calibration_results/camera_in_base_provisional.json','old_baseline.json'),
                      (PROJECT/'configs/arm/joint_mapping.json','joint_mapping.json')]:
        shutil.copy2(src,snap/name);files.append({'source':str(src),'snapshot':str((snap/name).relative_to(out)),'sha256':digest(src)})
    model_files=[PROJECT/'host/arm/gripper_geometry.py',PROJECT/'models/genkiarm/genkiarm.urdf']
    model_files += sorted((PROJECT/'models/genkiarm/meshes').glob('*.stl'))
    frozen={k:v for k,v in collection.items() if k!='groups'}
    frozen.update(groups=[mapping[g] for g in wanted],analysis_requested_groups=wanted)
    write(snap/'collection.json',frozen)
    write(out/'input_manifest.json',{'created':datetime.now(ZoneInfo('Asia/Shanghai')).isoformat(),
          'old_groups':OLD_GROUPS,'new_groups':NEW_GROUPS,'holdout_new_groups':HOLDOUT_NEW_GROUPS,'hardware_access':False,
          'files':files,'model_hashes':[{'path':str(p),'sha256':digest(p)} for p in model_files]})
    print('SNAPSHOT_SAVED',out,flush=True)

def fk(out):
    import numpy as np
    from host.arm.joint_mapping import JointMapping
    from gripper_geometry import PickGeometry
    geometry=PickGeometry(PROJECT,JointMapping.load(out/'snapshot/joint_mapping.json'))
    rows=[]
    for group in OLD_GROUPS+NEW_GROUPS:
        folder=out/'snapshot'/f'group_{group:02d}'
        record=read(folder/'record.json');samples=record['samples_before']+record['samples_after']
        counts=[int(round(float(np.median([s[str(i)]['position'] for s in samples])))) for i in range(1,7)]
        spans=[int(np.ptp([s[str(i)]['position'] for s in samples])) for i in range(1,7)]
        speeds=[[s[str(i)]['speed'] for s in samples] for i in range(1,7)]
        torques=[[s[str(i)]['torque'] for s in samples] for i in range(1,7)]
        q=geometry.angles(counts);mid,tips,lows,transforms=geometry.metrics(q)
        rows.append({'group':group,'counts':counts,'encoder_spans':spans,'speeds':speeds,'torques':torques,
                     'tip_midpoint_Base_mm':(mid*1000).tolist(),
                     'cube_center_Base_mm':(mid*1000+np.array([0,0,10.])).tolist(),
                     'fixed_jaw_tilt_deg':float(np.degrees(np.arccos(np.clip(-transforms['wan2'][2,2],-1,1)))),
                     'joint_angles_deg':np.rad2deg(q).tolist()})
    write(out/'encoder_fk.json',rows)

def vision(out):
    import cv2
    import numpy as np
    from face_geometry import face_candidates,pose_candidates
    from grasp_face_quality import assess
    intr=read(out/'snapshot/intrinsics.json');K=np.array(intr['camera_matrix'],float);K[0]*=2;K[1]*=2
    D=np.array(intr['distortion_coefficients'],float)
    rows=[];tiles=[]
    for group in OLD_GROUPS+NEW_GROUPS:
        folder=out/'snapshot'/f'group_{group:02d}';im=cv2.imread(str(folder/'frame.jpg'))
        if im is None or im.shape[:2]!=(960,1280):raise RuntimeError(f'Bad image group {group}')
        v=face_candidates(im,group)
        v['boundary_method']='existing_threshold_median'
        if group in NEW_GROUPS:
            legacy_ok=False
            if v['status']=='complete_face_candidate':
                quad=np.array(v['face_quad_px'])
                poses=pose_candidates(quad,K,D)
                legacy_ok=bool(poses and poses[0]['face_reprojection_rms_px']<=2. and quad[:,1].min()>=im.shape[0]*.45)
            if not legacy_ok:
                v=assess(im,K,D)
                v['boundary_method']='dim_blue_line_refinement'
        v['group']=group
        if group==1:
            confirmed=read(folder/'face_candidate.json')
            v.update(status='complete_face_candidate',face_quad_px=confirmed['face_quad_px'],face_identity='side_face_confirmed_by_user')
        if v['status']=='complete_face_candidate':
            v['poses']=pose_candidates(np.array(v['face_quad_px']),K,D)
            if len(v['poses'])==2:
                v['pose_ambiguity_mm']=float(np.linalg.norm(np.array(v['poses'][0]['cube_center_Camera_mm'])-v['poses'][1]['cube_center_Camera_mm']))
        annot=im.copy()
        if v.get('face_quad_px'):
            cv2.polylines(annot,[np.rint(v['face_quad_px']).astype(np.int32)],True,(0,255,0),3)
        for pose in v.get('poses',[])[:1]:
            cv2.drawMarker(annot,tuple(np.rint(pose['cube_center_pixel']).astype(int)),(0,255,255),cv2.MARKER_CROSS,20,2)
        cv2.imwrite(str(out/f'group_{group:02d}_annotated.jpg'),annot)
        tile=cv2.resize(annot,(480,360));label=f"Group {group}: {v['status']}"
        cv2.putText(tile,label,(8,24),cv2.FONT_HERSHEY_SIMPLEX,.47,(0,0,0),3)
        cv2.putText(tile,label,(8,24),cv2.FONT_HERSHEY_SIMPLEX,.47,(255,255,255),1)
        tiles.append(tile);rows.append(v)
        print('VISION',group,v['status'],'PnP_RMS', [round(p['face_reprojection_rms_px'],3) for p in v.get('poses',[])],flush=True)
    while len(tiles)%3:tiles.append(np.zeros_like(tiles[0]))
    cv2.imwrite(str(out/'all_groups_contact_sheet.jpg'),np.vstack([np.hstack(tiles[i:i+3]) for i in range(0,len(tiles),3)]))
    write(out/'vision.json',{'opencv':cv2.__version__,'numpy':np.__version__,'rows':rows})

def compare(out):
    import numpy as np
    fkrows={r['group']:r for r in read(out/'encoder_fk.json')}
    visionrows={r['group']:r for r in read(out/'vision.json')['rows']}
    usable=[g for g in OLD_GROUPS+NEW_GROUPS if visionrows[g]['status']=='complete_face_candidate' and visionrows[g].get('poses')]
    if not all(g in usable for g in OLD_GROUPS):raise RuntimeError('Old usable set no longer reproduced')
    ids=usable;camera=np.array([visionrows[g]['poses'][0]['cube_center_Camera_mm'] for g in ids]);base=np.array([fkrows[g]['cube_center_Base_mm'] for g in ids])
    def fit(groups):
        if len(groups)<3:raise ValueError('Not enough training points')
        ix=[ids.index(g) for g in groups];x=camera[ix];y=base[ix]
        U,s,Vt=np.linalg.svd((x-x.mean(0)).T@(y-y.mean(0)));R=Vt.T@U.T
        if np.linalg.det(R)<0:Vt[-1]*=-1;R=Vt.T@U.T
        return R,y.mean(0)-R@x.mean(0),s
    def evaluate(R,t,groups):
        rows=[]
        for g in groups:
            i=ids.index(g);p=R@camera[i]+t;d=p-base[i]
            rows.append({'group':g,'predicted_Base_mm':p.tolist(),'reference_Base_mm':base[i].tolist(),
                        'delta_Base_mm':d.tolist(),'xy_mm':float(np.linalg.norm(d[:2])),'xyz_mm':float(np.linalg.norm(d))})
        if not rows:return {'groups':[],'rms_xyz_mm':None,'rms_xy_mm':None}
        return {'groups':rows,'rms_xyz_mm':float(np.sqrt(np.mean([r['xyz_mm']**2 for r in rows]))),
                'rms_xy_mm':float(np.sqrt(np.mean([r['xy_mm']**2 for r in rows]))),
                'max_xyz_mm':max(r['xyz_mm'] for r in rows),'max_xy_mm':max(r['xy_mm'] for r in rows)}
    new=[g for g in NEW_GROUPS if g in ids]
    tests=[g for g in ([8,9]+HOLDOUT_NEW_GROUPS) if g in ids]
    oldtrain=[1,2,4,5];newtrain=oldtrain+[g for g in new if g not in tests]
    R0,t0,_=fit(oldtrain);R1,t1,s1=fit(newtrain)
    independent={'comparison':'Same holdout groups, excluded from both fitted transforms. New holdout groups are selected by ID before fitting, not assumed to be repetitions.',
                 'old_training_groups':oldtrain,'expanded_training_groups':newtrain,'validation_groups':tests,
                 'old':evaluate(R0,t0,tests),'expanded':evaluate(R1,t1,tests),
                 'old_holdout_only':{'old':evaluate(R0,t0,[8,9]),'expanded':evaluate(R1,t1,[8,9])},
                 'new_holdout_only':{'old':evaluate(R0,t0,[g for g in HOLDOUT_NEW_GROUPS if g in ids]),'expanded':evaluate(R1,t1,[g for g in HOLDOUT_NEW_GROUPS if g in ids])}}
    R6,t6,_=fit(OLD_GROUPS)
    saved=read(out/'snapshot/old_baseline.json');T=np.array(saved['T_Base_from_Camera_mm'])
    assert np.allclose(R6,T[:3,:3],atol=1e-8) and np.allclose(t6,T[:3,3],atol=1e-7),'Old calibration does not reproduce'
    # Strict old-six predictions on all new samples were independent of old fitting.
    new_prediction=evaluate(R6,t6,new)
    loo=[]
    for g in new:
        train=[i for i in ids if i!=g];R,t,_=fit(train)
        loo.append({'test_group':g,'training_groups':train,'prediction':evaluate(R,t,[g])['groups'][0]})
    loo_xy=float(np.sqrt(np.mean([r['prediction']['xy_mm']**2 for r in loo]))) if loo else None
    loo_xyz=float(np.sqrt(np.mean([r['prediction']['xyz_mm']**2 for r in loo]))) if loo else None
    R,t,s=fit(ids);allfit=evaluate(R,t,ids);fullT=np.eye(4);fullT[:3,:3]=R;fullT[:3,3]=t
    oldpoints=np.array([fkrows[g]['tip_midpoint_Base_mm'] for g in OLD_GROUPS]);newpoints=np.array([fkrows[g]['tip_midpoint_Base_mm'] for g in ids])
    stale=[]
    for group in ids:
        row=fkrows[group]
        if max(row['encoder_spans'])>2 or any(v!=0 for arr in row['speeds'] for v in arr):stale.append(group)
    result={'status':'offline_comparison_only','old_groups':OLD_GROUPS,'new_requested_groups':NEW_GROUPS,
            'usable_groups':ids,'usable_new_groups':new,'rejected_groups':[{'group':g,'reason':visionrows[g].get('reason',visionrows[g]['status'])} for g in NEW_GROUPS if g not in ids],
            'image_and_encoder_quality':[{ 'group':g,'status':visionrows[g]['status'],'encoder_spans':fkrows[g]['encoder_spans'],
                'max_speed_raw':max(abs(v) for a in fkrows[g]['speeds'] for v in a),'fixed_jaw_tilt_deg':fkrows[g]['fixed_jaw_tilt_deg'],
                'boundary_method':visionrows[g].get('boundary_method'),'face_reprojection_rms_px':visionrows[g]['poses'][0]['face_reprojection_rms_px'] if visionrows[g].get('poses') else None,
                'pose_ambiguity_mm':visionrows[g].get('pose_ambiguity_mm')} for g in OLD_GROUPS+NEW_GROUPS],
            'feedback_variation_flags_not_automatic_exclusion':stale,'old_baseline_reproduced':True,
            'fixed_split_comparison':independent,'old_six_predictions_on_new':new_prediction,
            'expanded_leave_one_new_out':{'rms_xy_mm':loo_xy,'rms_xyz_mm':loo_xyz,'groups':loo},
            'full_fit':{'training_groups':ids,**allfit,'T_Base_from_Camera_mm':fullT.tolist(),'T_Camera_from_Base_mm':np.linalg.inv(fullT).tolist(),
                        'cross_covariance_singular_values':s.tolist(),'table_z_Base_mm_median':float(np.median(newpoints[:,2]))},
            'coverage':{'old_range_Base_mm':np.ptp(oldpoints,axis=0).tolist(),'combined_range_Base_mm':np.ptp(newpoints,axis=0).tolist(),
                        'old_min_max_Base_mm':[oldpoints.min(0).tolist(),oldpoints.max(0).tolist()],
                        'combined_min_max_Base_mm':[newpoints.min(0).tolist(),newpoints.max(0).tolist()]},
            'limitations':['References are the user-selected modeled fingertip midpoint plus Base Z 10mm, not surveyed cube centers.',
               'New groups inherit the same capture protocol; physical contact and centering cannot be proven by encoder reads.',
               'Single visible square face gives pose ambiguity; lowest reprojection candidate remains selected independently of FK.',
               'No claim of powered grasp accuracy; more points cannot remove systematic FK/TCP bias.',
               'The last two requested groups are held out by ID, not confirmed identical-position repetitions.'],
            'canonical_baseline_updated':False,'hardware_access':False,
            'offline_image_quality_screen':{'refined_face_reprojection_rms_max_px':2.,'min_near_boundary_support':3,'purpose':'Conservative offline review of image corners, not a hardware protection or proof of real grasp accuracy'},
            'quality_decision':'No new samples accepted; improvement cannot be assessed' if not new else 'New samples admitted after image-only quality assessment',
            'user_evidence':{'prior_groups_11_to_18_without_extra_lighting':True,'old_ten_with_extra_lighting':True,'latest_batch_capture_condition_notes':read(out/'snapshot/collection.json').get('capture_condition_notes'),'current_batch_focus_change_not_documented_in_raw_records':True}}
    write(out/'comparison.json',result)
    b=independent['old'];e=independent['expanded']
    fmt=lambda value: '不可计算' if value is None else f'{value:.3f}'
    report=['# K230补采前后标定对照','',f'输入：原6组 {OLD_GROUPS}；新增{len(NEW_GROUPS)}组 {NEW_GROUPS}。可用新增组：{new}。',
            '采用冻结快照，未读取分支后续采集，未改写原始数据、采集索引、既有分析或当前默认外参。','',
            '## 同一留出集对照','',f'验证组：{tests}，均不参与本对照拟合。旧训练组：{oldtrain}；扩展训练组：{newtrain}。','',
            '| 指标 | 旧样本训练 | 加入新样本训练 |','|---|---:|---:|',
            f"| XY RMS/mm | {b['rms_xy_mm']:.3f} | {e['rms_xy_mm']:.3f} |",
            f"| XYZ RMS/mm | {b['rms_xyz_mm']:.3f} | {e['rms_xyz_mm']:.3f} |",
            f"| 最大XY误差/mm | {b['max_xy_mm']:.3f} | {e['max_xy_mm']:.3f} |",'',
            '### 原验证组8、9','',
            f"XYZ RMS：{independent['old_holdout_only']['old']['rms_xyz_mm']:.3f} → {independent['old_holdout_only']['expanded']['rms_xyz_mm']:.3f} mm。",'',
            '## 新位置预测与覆盖','',f"原6组外参预测新增有效样本：XY RMS {fmt(new_prediction['rms_xy_mm'])}mm，XYZ RMS {fmt(new_prediction['rms_xyz_mm'])}mm。",
            f'加入其他新组、逐个留出新组：XY RMS {fmt(loo_xy)}mm，XYZ RMS {fmt(loo_xyz)}mm。',
            f"原XYZ跨度(mm)：{np.round(np.ptp(oldpoints,axis=0),2).tolist()}；合并后：{np.round(np.ptp(newpoints,axis=0),2).tolist()}。",'',
            '## 全部有效样本拟合（仅候选）','',f"训练XYZ RMS {allfit['rms_xyz_mm']:.3f}mm；桌面Base Z中位值 {np.median(newpoints[:,2]):.3f}mm。训练误差不作为独立精度证明。",'',
            '## 使用边界','',
            '误差相对于用户选定的指尖中点+10mm近似。并非通电抓取实测精度；夹持参考偏差、关节/杆件模型和单面PnP歧义仍可造成系统误差。',
            f'新验证组{HOLDOUT_NEW_GROUPS}按编号事先留出，尚不声称是旧位置的重复摆位。',
            '全部矩阵、逐组误差、编码跨度与识别图见本目录；当前默认外参不自动替换。']
    if not new:
        report.insert(2,f'**结论：新增{len(NEW_GROUPS)}组目前未达到本轮角点质量复核要求，不能据此证明标定精度改善。表中的前后值相同，是因为没有新组入选，不是14组拟合后的测试结果。**')
        report += ['', f'## 新{len(NEW_GROUPS)}组图像质量', '',
                   '原6组的现有面重投影RMS为0.39～1.17px。暗光复核降低亮度阈值，并拟合边线；离线复核暂以RMS≤2px且至少3个近似边界候选为筛选标准。该标准只用于本轮图像分析，不涉及机械臂控制。', '',
                   '| 组别 | 状态 | 暗光边线复核RMS/px |', '|---|---|---:|']
        for g in NEW_GROUPS:
            v=visionrows[g]
            value=v['poses'][0]['face_reprojection_rms_px'] if v.get('poses') else None
            report.append(f"| {g} | {'轮廓到图像底边，缺完整面' if not v.get('poses') else '有完整面候选，但角点精度不足'} | {fmt(value)} |")
        report += ['', '保留原始数据作识别调试，需补拍的组别以本轮逐组质量表为准，先验证一组边界质量。',
                   '旧11～18组的光照和失焦说明不自动作为本次补拍的事实；本次采集条件以新的用户确认或采集记录为准。',
                   '分支可继续采集：本轮仅用冻结快照，分析不占任何串口、不改collection索引。',
                   '当前默认外参完整保留：原留出XY RMS 3.358mm、XYZ RMS 4.324mm；它们是近似对应点残差，不是通电抓取精度。']
    (out/'报告.md').write_text('\n'.join(report)+'\n')
    print('COMPARISON',json.dumps({'usable_new':new,'old_same_holdout':b,'expanded_same_holdout':e,'old_holdout':independent['old_holdout_only'],'new_prediction_xyz':new_prediction['rms_xyz_mm'],'new_loo_xyz':loo_xyz,'full_training_xyz':allfit['rms_xyz_mm'],'flags':stale},ensure_ascii=False),flush=True)

def main():
    global NEW_GROUPS,HOLDOUT_NEW_GROUPS
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--output',type=Path,required=True);p.add_argument('--vision-phase',action='store_true');p.add_argument('--resume-frozen',action='store_true');p.add_argument('--new-groups',type=int,nargs='+');p.add_argument('--holdout-new-groups',type=int,nargs='+');args=p.parse_args();out=args.output.resolve()
    if args.resume_frozen or args.vision_phase:
        frozen=read(out/'input_manifest.json');NEW_GROUPS=frozen['new_groups'];HOLDOUT_NEW_GROUPS=frozen.get('holdout_new_groups',NEW_GROUPS[-2:])
    else:
        if args.new_groups is not None:NEW_GROUPS=args.new_groups
        HOLDOUT_NEW_GROUPS=args.holdout_new_groups if args.holdout_new_groups is not None else NEW_GROUPS[-2:]
        if len(NEW_GROUPS)!=len(set(NEW_GROUPS)) or set(NEW_GROUPS)&set(OLD_GROUPS):raise RuntimeError('New group IDs must be unique and disjoint from old groups')
        if not set(HOLDOUT_NEW_GROUPS)<=set(NEW_GROUPS):raise RuntimeError('Holdout IDs must belong to the requested batch')
    if args.vision_phase:vision(out);return
    if not args.resume_frozen:freeze(out);fk(out)
    subprocess.run([str(PROJECT/'scripts/vision'),str(Path(__file__).resolve()),'--output',str(out),'--vision-phase'],cwd=PROJECT,check=True)
    compare(out)
    manifest=read(out/'input_manifest.json')
    for f in manifest['files']:
        if digest(out/f['snapshot'])!=f['sha256']:raise RuntimeError('Snapshot changed during analysis')
    for f in manifest['model_hashes']:
        if digest(Path(f['path']))!=f['sha256']:raise RuntimeError('FK model changed during analysis')
    print('SNAPSHOT_AND_MODEL_HASHES_VERIFIED',flush=True)

if __name__=='__main__':main()
