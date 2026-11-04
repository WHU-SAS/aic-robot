#!/usr/bin/env python3
"""Measure a saved ChArUco board plane; never adopt it as robot calibration.

Use scripts/vision. A board can be hand-held in any orientation. Only an
explicit contact declaration labels it as user-declared table-contact data.
Both IPPE seeds are independently refined, then ranked by image RMS only.
"""
import sys
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[2]))
from host.project_paths import ROOT,bootstrap
bootstrap(include_tools=True)

import argparse
import hashlib
import json
import math
import shutil
from host.vision.config import load_vision_config
from device.k230.pick.numeric import dot,solve,transpose

DEFAULT_PROFILE=ROOT/'configs/vision/trials/model_alignment_20261030.json'


def normalized_plane(plane):
    normal=[float(v)for v in plane['normal']]
    length=math.sqrt(dot(normal,normal))
    if length==0:raise ValueError('Plane normal is zero')
    return {'normal':[v/length for v in normal],
            'offset_mm':float(plane['offset_mm'])/length}


def transform_plane(plane,T):
    """n_C dot p_C=d_C, p_B=T_BC p_C; use inverse transpose exactly."""
    p=normalized_plane(plane)
    R=[row[:3]for row in T[:3]];translation=[row[3]for row in T[:3]]
    normal=solve(transpose(R),p['normal'])
    return normalized_plane({'normal':normal,
        'offset_mm':p['offset_mm']+dot(normal,translation)})


def compare_planes(observed,reference,center):
    observed,reference=normalized_plane(observed),normalized_plane(reference)
    cosine=min(1.,abs(dot(observed['normal'],reference['normal'])))
    residual=dot(reference['normal'],center)-reference['offset_mm']
    return {'unoriented_normal_angle_deg':math.degrees(math.acos(cosine)),
        'reference_plane_signed_residual_at_board_center_mm':residual,
        'reference_plane_abs_residual_at_board_center_mm':abs(residual),
        'observed_plane_residual_at_board_center_mm':
            dot(observed['normal'],center)-observed['offset_mm'],
        'plane_sign_is_geometrically_arbitrary':True}


def contact_role(declared=False,entry=None):
    entry=entry or {}
    declared=declared or entry.get('role')=='user_declared_table_contact'
    return 'user_declared_table_contact'if declared else'observed_board_plane'


def digest(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def detector_for(intrinsics):
    from tools.calibration import charuco_common as common
    spec=intrinsics['board'];square=float(spec['square_mm_measured'])
    if (spec['squares_x']!=common.SQUARES_X or spec['squares_y']!=common.SQUARES_Y or
            spec['dictionary']!=common.DICTIONARY_NAME or
            abs(float(spec['marker_mm_assumed'])-square/2)>1e-9):
        raise ValueError('The shared detector supports the configured5x7 half-marker DICT_5X5_100 board')
    board,detector=common.make_board(square)
    return board,detector,common.detect


def detect_pose(image,intrinsics,board,detector,detect):
    import cv2
    import numpy as np
    K=np.array(intrinsics['camera_matrix'],dtype=float)
    sx=image.shape[1]/intrinsics['image_width'];sy=image.shape[0]/intrinsics['image_height']
    if abs(sx-sy)>1e-9:raise ValueError('Image aspect differs from the paired intrinsic calibration')
    K[0]*=sx;K[1]*=sy;D=np.array(intrinsics['distortion_coefficients'],dtype=float)
    corners,ids,count,markers,marker_corners,marker_ids=detect(image,detector)
    found={'charuco_ids':[]if ids is None else ids.reshape(-1).tolist(),
        'marker_ids':[]if marker_ids is None else marker_ids.reshape(-1).tolist(),
        'charuco_corner_count':count,'marker_count':markers,
        'image_size_px':[image.shape[1],image.shape[0]],'intrinsics_scale':[sx,sy],
        'method':'marker_confirmed_charuco_ids',
        'K_used':K.tolist(),'D_used':D.tolist()}
    if count<6:
        return dict(found,status='board_not_detected',reason='Fewer than6 marker-confirmed ChArUco corners'),image.copy()
    objects=np.ascontiguousarray(board.getChessboardCorners()[ids.reshape(-1)],dtype=float)
    pixels=np.ascontiguousarray(corners.reshape(-1,2),dtype=float)
    seeds=cv2.solvePnPGeneric(objects,pixels,K,D,flags=cv2.SOLVEPNP_IPPE)
    candidates=[]
    for rank,(r,t)in enumerate(zip(seeds[1],seeds[2])):
        r,t=cv2.solvePnPRefineLM(objects,pixels,K,D,
            np.ascontiguousarray(r,dtype=float),np.ascontiguousarray(t,dtype=float),
            criteria=(cv2.TERM_CRITERIA_EPS|cv2.TERM_CRITERIA_MAX_ITER,100,1e-10))
        R=cv2.Rodrigues(r)[0];translation=t.reshape(3)
        depths=(objects@R.T+translation)[:,2]
        projected=cv2.projectPoints(objects,r,t,K,D)[0].reshape(-1,2)
        rms=float(np.sqrt(np.mean(np.sum((projected-pixels)**2,axis=1))))
        candidates.append({'IPPE_seed_rank':rank,'reprojection_rms_px':rms,
            'all_detected_points_positive_depth':bool(np.all(depths>0)),
            'R_Board_to_Camera':R.tolist(),'t_Board_to_Camera_mm':translation.tolist()})
    valid=[c for c in candidates if c['all_detected_points_positive_depth']]
    if not valid:raise ValueError('No positive-depth IPPE+LM board pose')
    selected=min(valid,key=lambda c:c['reprojection_rms_px'])
    R=np.array(selected['R_Board_to_Camera']);translation=np.array(selected['t_Board_to_Camera_mm'])
    square=float(intrinsics['board']['square_mm_measured'])
    width=intrinsics['board']['squares_x']*square;height=intrinsics['board']['squares_y']*square
    board_center=np.array([width/2,height/2,0.]);center=R@board_center+translation
    normal=R[:,2]
    annotated=image.copy();cv2.aruco.drawDetectedCornersCharuco(annotated,corners,ids)
    outer=np.array([[0.,0.,0.],[width,0.,0.],[width,height,0.],[0.,height,0.]])
    rvec=cv2.Rodrigues(R)[0]
    outline=cv2.projectPoints(outer,rvec,translation,K,D)[0].reshape(-1,2)
    cv2.polylines(annotated,[np.rint(outline).astype(np.int32)],True,(0,255,255),2)
    for label,point in zip('ABCD',outline):
        px,py=np.rint(point).astype(int)
        cv2.putText(annotated,label,(max(0,min(image.shape[1]-25,int(px))),
            max(25,min(image.shape[0]-5,int(py)))),cv2.FONT_HERSHEY_SIMPLEX,.7,(0,255,255),2)
    return dict(found,status='observed_board_plane',pose_candidates=candidates,
        selected_IPPE_seed_rank=selected['IPPE_seed_rank'],
        pose_selection='minimum_refined_image_RMS_among_positive_depth_IPPE_seeds',
        reprojection_rms_px=selected['reprojection_rms_px'],
        R_Board_to_Camera=R.tolist(),t_Board_to_Camera_mm=translation.tolist(),
        plane_Camera_mm=normalized_plane({'normal':normal.tolist(),'offset_mm':float(normal@translation)}),
        board_center_Camera_mm=center.tolist(),
        detected_corners=[{'id':int(i),'Board_mm':obj.tolist(),'image_px':px.tolist()}
            for i,obj,px in zip(ids.reshape(-1),objects,pixels)]),annotated


def manifest_entry(manifest,path):
    if not manifest:return {}
    image_sha=digest(path)
    for entry in manifest.get('images',[]):
        declared_sha=entry.get('image_sha256')
        if declared_sha:
            if declared_sha==image_sha:return entry
            continue
        source=entry.get('image')or entry.get('image_path')
        same_name=source and(Path(source).name==path.name or str(path.resolve())==source)
        if same_name:return entry
    return {}


def verify(images,output,vision_config=DEFAULT_PROFILE,table_contact=False,manifest_path=None):
    import cv2
    output=Path(output);output.mkdir(parents=True,exist_ok=False)
    profile_path=Path(vision_config).resolve();profile=load_vision_config(profile_path)
    localization=profile['localization']
    intr_path=ROOT/localization['intrinsics'];ext_path=ROOT/localization['extrinsics']
    intr=json.loads(intr_path.read_text());ext=json.loads(ext_path.read_text())
    T=ext['T_Base_from_Camera_mm'];prior=localization['table_plane_Base_mm']
    snapshots={}
    for name,path in(('vision_profile.json',profile_path),('intrinsics.json',intr_path),('extrinsics.json',ext_path)):
        shutil.copy2(path,output/name);snapshots[name]={'source':str(path),'sha256':digest(path)}
    manifest=json.loads(Path(manifest_path).read_text())if manifest_path else None
    if manifest_path:shutil.copy2(manifest_path,output/'input_manifest.json')
    board,detector,detect=detector_for(intr);rows=[]
    for index,source in enumerate(map(Path,images),1):
        folder=output/('%04d_%s'%(index,source.stem));folder.mkdir()
        saved=folder/('original'+source.suffix.lower());shutil.copy2(source,saved)
        entry=manifest_entry(manifest,source);role=contact_role(table_contact,entry)
        image=cv2.imread(str(saved))
        if image is None:raise ValueError('Cannot decode saved original: '+str(source))
        measurement,annotated=detect_pose(image,intr,board,detector,detect)
        record=dict(measurement,input_image=str(source.resolve()),original_copy=str(saved.resolve()),
            image_sha256=digest(saved),input_role=entry.get('role'),role=role,
            table_contact_user_declared=role=='user_declared_table_contact',
            declaration_source=('command_line'if table_contact else'input_manifest')
                if role=='user_declared_table_contact'else None,
            capture_timestamp_ms=entry.get('capture_timestamp_ms'),
            board_spec=intr['board'],calibration_sources=snapshots,calibration_updated=False,
            physical_grasp_accuracy_validated=False,
            capture_profile_pairing='supplied_profile; historical_same_mount_not_inferred',
            limitation='A detected board plane is not table truth. Contact label is only the user declaration; board flatness/size and camera pairing still matter.')
        if measurement['status']=='observed_board_plane':
            plane=transform_plane(measurement['plane_Camera_mm'],T)
            center=[sum(T[i][j]*measurement['board_center_Camera_mm'][j]for j in range(3))+T[i][3]for i in range(3)]
            record.update(plane_Base_mm=plane,board_center_Base_mm=center,
                configured_table_plane_Base_mm=prior,
                plane_comparison=compare_planes(plane,prior,center))
        text=role+' / '+measurement['status']
        cv2.putText(annotated,text,(12,28),cv2.FONT_HERSHEY_SIMPLEX,.6,(0,255,255),2)
        if not cv2.imwrite(str(folder/'annotated.jpg'),annotated):raise OSError('Cannot save board annotation')
        (folder/'result.json').write_text(json.dumps(record,indent=2,ensure_ascii=False)+'\n')
        rows.append({'image':str(source.resolve()),'result':str((folder/'result.json').resolve()),
            'image_sha256':record['image_sha256'],'role':role,'status':record['status'],
            'reprojection_rms_px':record.get('reprojection_rms_px')})
    summary={'schema':'aic.observed_board_plane_review.v1','images':rows,
        'profile':snapshots,'OpenCV_version':cv2.__version__,
        'analysis_tool':{'source':str(Path(__file__).resolve()),'sha256':digest(__file__)},
        'observed_board_planes':sum(r['status']=='observed_board_plane'for r in rows),
        'user_declared_table_contact_images':sum(r['role']=='user_declared_table_contact'for r in rows),
        'calibration_updated':False,'physical_grasp_accuracy_validated':False}
    (output/'index.json').write_text(json.dumps(summary,indent=2,ensure_ascii=False)+'\n')
    return summary


def main(argv=None):
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('images',type=Path,nargs='+')
    parser.add_argument('--vision-config',type=Path,default=DEFAULT_PROFILE)
    parser.add_argument('--output',type=Path,required=True,help='New output directory; existing paths are not overwritten')
    parser.add_argument('--table-contact',action='store_true',help='Explicit user declaration: all inputs show the board flat on the table')
    parser.add_argument('--manifest',type=Path,help='Optional images entries with image/image_sha256 and explicit role')
    args=parser.parse_args(argv)
    result=verify(args.images,args.output,args.vision_config,args.table_contact,args.manifest)
    print(json.dumps({'output':str(args.output.resolve()),'images':len(result['images']),
        'observed_board_planes':result['observed_board_planes'],
        'user_declared_table_contact_images':result['user_declared_table_contact_images'],
        'calibration_updated':False},ensure_ascii=False))


if __name__=='__main__':main()
