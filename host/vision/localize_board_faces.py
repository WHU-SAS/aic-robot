"""PC pose from K230 physical faces: match corners first, then validate 3-frame pose."""

import sys as _sys
from pathlib import Path as _Path
_sys.path.insert(0, str(_Path(__file__).resolve().parents[2]))
from host.project_paths import ROOT as REPO_ROOT, bootstrap as _bootstrap
_bootstrap(include_tools=False)
import argparse,hashlib,json
from itertools import combinations
from pathlib import Path
import cv2,numpy as np
from face_geometry import pose_candidates,ordered_quad
from cube_outline_pose import localize_cube_outlines
ROOT=REPO_ROOT

def localize(features,output):
    output=Path(output);output.mkdir(parents=True,exist_ok=True)
    data=json.loads(Path(features).read_text());cfg=json.loads((ROOT/'configs/arm/k230_pick_config.json').read_text())
    ip=ROOT/cfg['intrinsics'];ep=ROOT/cfg['extrinsics'];intr=json.loads(ip.read_text());ext=json.loads(ep.read_text())
    if hashlib.sha256(ip.read_bytes()).hexdigest()!=ext['intrinsics_sha256']:raise RuntimeError('内外参不匹配')
    if data.get('schema')!='aic.blue_faces.v1' or (data.get('width'),data.get('height'))!=(1280,960) or data.get('hmirror') is not False or data.get('vflip') is not False:
        raise RuntimeError('K230角点协议、画面尺寸或镜像设置不符')
    K=np.array(intr['camera_matrix'],float);K[0]*=1280/intr['image_width'];K[1]*=960/intr['image_height'];D=np.array(intr['distortion_coefficients'],float);T=np.array(ext['T_Base_from_Camera_mm'],float)
    frames=data.get('frames',[])
    if not 3<=len(frames)<=9:raise RuntimeError('需要K230至少三帧、最多九帧的角点记录')
    prepared=[];diagnostics=[]
    for index,frame in enumerate(frames):
        usable=[];checks=[];rejected=[]
        for f in frame.get('candidates',[]):
            try:
                q=np.array(f['quad_px'],float)
                if q.shape!=(4,2) or not np.isfinite(q).all() or (q[:,0]<=2).any() or (q[:,0]>=1278).any() or (q[:,1]<432).any() or (q[:,1]>=958).any():continue
                q=ordered_quad(q);source=f.get('detector','fitted_edges')
                if source=='native_rects':
                    if f.get('edge_magnitude',0)<3000 or f.get('blue_fraction',0)<.7:continue
                else:
                    if f.get('kind')=='unknown' and source!='cube_edges':
                        rejected.append({'reason':'整块蓝色轮廓缺少面分界，不作为完整物理面','quad_px':q.tolist()});continue
                    if source=='cube_edges' and f.get('internal_seams_verified') is not True:continue
                    if len(f.get('edges',[]))!=4 or max(e['rms_px'] for e in f['edges'])>3:continue
                area=abs(float(cv2.contourArea(q.astype(np.float32))))
                if area<1000 or not cv2.isContourConvex(q.astype(np.float32)):continue
                poses=pose_candidates(q,K,D)
                checks.append({'kind':f['kind'],'detector':source,'value_floor':f.get('value_floor',40),'quad_px':q.tolist(),'reprojection_rms_px':[p['face_reprojection_rms_px'] for p in poses]})
                # Keep a stable physical face even if one frame's PnP is noisy.
                # The same <=2px criterion is checked after combining its corners.
                usable.append({'declared_kind':f['kind'],'detector':source,'face_index':f.get('face_index'),'value_floor':f.get('value_floor',40),'quad_px':q.tolist(),'area_px':area})
            except (ValueError,KeyError,TypeError,cv2.error) as exc:rejected.append({'reason':str(exc)})
        diagnostics.append({'frame':index,'candidates_for_temporal_match':usable,'pose_checks':checks,'PC_rejected':rejected,
            'board_rejected':frame.get('rejected',[]),'board_status':frame.get('status'),'board_bbox_px':frame.get('bbox_px'),
            'board_reason':frame.get('reason'),'board_bottom_clipped':frame.get('bottom_clipped'),
            'three_face_diagnostic':frame.get('three_face_diagnostic'),'component_attempts':frame.get('component_attempts',[]),'native_rect_diagnostic':frame.get('native_rect_diagnostic'),'LAB_blobs':frame.get('LAB_blobs',[])})
        prepared.append(usable)
    stable=[];combined_checks=[];seen_quads=set()
    for frame_index,candidates in enumerate(prepared):
        for f in candidates:
            previous_matches=[]
            for previous_index,previous in enumerate(prepared[:frame_index]):
                near=[p for p in previous if (p['declared_kind'],p['detector'],p['value_floor'],p.get('face_index'))==(f['declared_kind'],f['detector'],f['value_floor'],f.get('face_index')) and np.max(np.linalg.norm(np.array(p['quad_px'])-f['quad_px'],axis=1))<=5]
                if near:
                    closest=min(near,key=lambda p:float(np.max(np.linalg.norm(np.array(p['quad_px'])-f['quad_px'],axis=1))))
                    previous_matches.append((previous_index,closest))
            for pair in combinations(previous_matches,2):
                q=np.median([f['quad_px'],pair[0][1]['quad_px'],pair[1][1]['quad_px']],axis=0)
                key=(f['detector'],f.get('face_index'),f['value_floor'],tuple(q.ravel()))
                if key in seen_quads:continue
                seen_quads.add(key);selected_frames=[pair[0][0],pair[1][0],frame_index];poses=pose_candidates(q,K,D)
                combined_checks.append({'detector':f['detector'],'declared_kind':f['declared_kind'],'value_floor':f['value_floor'],'face_index':f.get('face_index'),'selected_frame_indices':selected_frames,'quad_px':q.tolist(),'rms_px':[p['face_reprojection_rms_px'] for p in poses]})
                for pose in poses:
                    if pose['face_reprojection_rms_px']>2:continue
                    normal=T[:3,:3]@np.array(pose['inward_face_normal_Camera'])
                    kind=f['declared_kind']
                    if kind=='unknown':kind='top' if normal[2]<-.7 else ('side' if abs(normal[2])<=.7 else 'unsupported')
                    if kind=='unsupported' or (kind=='top' and normal[2]>-.7) or (kind=='side' and abs(normal[2])>.7):continue
                    center=T[:3,:3]@np.array(pose['cube_center_Camera_mm'])+T[:3,3]
                    stable.append(dict(f,kind=kind,quad_px=q.tolist(),selected_frame_indices=selected_frames,pose=pose,center_Base_mm=center.tolist(),normal_Base=normal.tolist()))
    diagnosis={'schema':'aic.blue_faces.diagnostic.v2','frames':diagnostics,'combined_pose_checks':combined_checks,'stable_candidates':stable}
    def fail(reason):
        diagnosis['reason']=reason;(output/'vision_candidates.json').write_text(json.dumps(diagnosis,ensure_ascii=False,indent=2)+'\n');raise RuntimeError(reason)
    def finish(chosen,consistency=None):
        center=chosen['center_Base_mm'];diagnosis.update(selected=chosen,face_center_disagreement_mm=consistency)
        result={'status':'localized','method':('K230 measured six-edge known20mm cube pose' if chosen['detector']=='cube_outline' else 'K230 physical quadrilaterals; bounded burst, match three valid frames before PC OpenCV20mm pose'),
                'cube_center_Base_mm':center,'cube_center_Camera_mm':chosen['pose']['cube_center_Camera_mm'],
                'cube_center_z_above_assumed_table_mm':center[2]-cfg['table_z_Base_mm'],'table_z_Base_mm':cfg['table_z_Base_mm'],
                'vision':diagnosis,'intrinsics':str(ip),'extrinsics':str(ep),'motion_performed':False,
                'xy_compensation_applied':False,'vision_backend':'k230','image_pixels_processed_on_PC':False}
        (output/'vision_record.json').write_text(json.dumps(result,ensure_ascii=False,indent=2)+'\n');return result
    def cube_fallback():
        joint,joint_diagnostic=localize_cube_outlines(frames,K,D,T)
        diagnosis['cube_outline_diagnostic']=joint_diagnostic
        return joint
    joint=cube_fallback()
    diagnosis['selection_policy']='whole verified20mm cube first, then independent physical faces'
    if joint is not None:return finish(joint)
    if not stable:
        clipped_sets=[]
        for frame in frames:
            components=[c for blob in frame.get('LAB_blobs',[]) for c in blob.get('HSV_components',[])]
            if not components or any(not c.get('clipped') for c in components):break
            sides=set(components[0].get('clipped_sides',[]))
            for component in components[1:]:sides.intersection_update(component.get('clipped_sides',[]))
            clipped_sets.append(sides)
        if len(clipped_sets)==len(frames):
            common=set.intersection(*clipped_sets)
            if common.intersection({'left','right','top'}):
                names={'left':'左侧','right':'右侧','top':'上侧','bottom':'下侧'};text='、'.join(names[s] for s in ['left','right','top','bottom'] if s in common)
                fail('蓝色候选区域在画面'+text+'出框，未取得完整方块面；请仅移动方块，让一个完整面进入画面')
        if combined_checks:
            rms=[min(p['rms_px'],default=float('inf')) for p in combined_checks]
            fail('物理面角点稳定，但三帧合并后的解算质量不足；最小重投影误差'+format(min(rms),'.2f')+'px')
        if any(f.get('candidates') for f in frames):fail('采样'+str(len(frames))+'帧后仍不足三帧匹配的完整物理面；请查看component_attempts中的棱线诊断')
        fail('K230未提取到完整物理面；板端状态：'+','.join(str(f.get('status')) for f in frames))
    # Cross-check faces from the same extraction method. Do not mix a
    # legacy two-face fit with the new three-face junction geometry.
    chosen=None;consistency=None;source_checks=[]
    for source in ['native_rects','cube_edges','fitted_edges']:
        pool=[f for f in stable if f['detector']==source]
        if not pool:continue
        top=[f for f in pool if f['kind']=='top'];side=[f for f in pool if f['kind']=='side'];pairs=[]
        for a in top:
            for b in side:
                delta=float(np.linalg.norm(np.array(a['center_Base_mm'])-b['center_Base_mm']))
                if delta<=8:pairs.append((delta,a,b))
        if top and side:
            if not pairs:
                source_checks.append({'detector':source,'status':'rejected','reason':'Top and side centers disagree >8mm'})
                continue
            pairs.sort(key=lambda p:p[0]);consistency,a,b=pairs[0]
            chosen=a if a['pose']['face_reprojection_rms_px']<=b['pose']['face_reprojection_rms_px'] else b
        else:
            if source=='fitted_edges' and not top:
                source_checks.append({'detector':source,'status':'rejected','reason':'Legacy side-only fit has no independent cube/top verification'})
                continue
            chosen=max(pool,key=lambda f:(f['area_px'],-f['pose']['face_reprojection_rms_px']))
        source_checks.append({'detector':source,'status':'selected'})
        break
    diagnosis['source_consistency_checks']=source_checks
    if chosen is None:
        fail('未得到可靠的整块方块或独立物理面；旧侧面候选缺少验证或相互矛盾，诊断已保留')
    return finish(chosen,consistency)

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--features',type=Path,required=True);p.add_argument('--output',type=Path,required=True);args=p.parse_args()
    try:print(json.dumps(localize(args.features,args.output),ensure_ascii=False))
    except (RuntimeError,ValueError,KeyError,TypeError) as exc:
        print('K230方块定位失败：'+str(exc));raise SystemExit(2)
