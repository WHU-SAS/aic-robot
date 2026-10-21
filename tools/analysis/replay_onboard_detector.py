#!/usr/bin/env python3
"""Replay saved K230 features or images offline; never accesses camera or servo ports."""
import sys as _sys
from pathlib import Path as _Path
_sys.path.insert(0, str(_Path(__file__).resolve().parents[2]))
from host.project_paths import ROOT as REPO_ROOT, bootstrap as _bootstrap
_bootstrap(include_tools=True)
import argparse
import json
from pathlib import Path
import sys
import time
import cv2
import numpy as np
sys.path.insert(0,str(REPO_ROOT/'device/k230/vision'))
from blue_face_core import extract
from host.vision.localize_board_faces import localize


def replay_image(folder, output):
    image=cv2.imread(str(folder/'frame.jpg'))
    if image is None:raise RuntimeError('Cannot read '+str(folder/'frame.jpg'))
    if image.shape[:2]!=(960,1280):raise RuntimeError('Replay requires 1280x960 images')
    lab=cv2.cvtColor(image,cv2.COLOR_BGR2LAB)
    mask=cv2.inRange(lab,np.array([0,108,0]),np.array([255,255,120]));mask[:432]=0
    contours,_=cv2.findContours(mask,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)
    contours=[c for c in contours if cv2.contourArea(c)>=1000 and cv2.boundingRect(c)[2]*cv2.boundingRect(c)[3]>=6000]
    if not contours:raise RuntimeError('No blue replay candidate')
    bbox=cv2.boundingRect(max(contours,key=cv2.contourArea))
    rgb=cv2.cvtColor(image,cv2.COLOR_BGR2RGB)
    rgb[:,:,0]=(rgb[:,:,0]>>3)<<3;rgb[:,:,1]=(rgb[:,:,1]>>2)<<2;rgb[:,:,2]=(rgb[:,:,2]>>3)<<3
    result=extract(lambda x,y:tuple(int(v) for v in rgb[y,x]),bbox)
    payload={'schema':'aic.blue_faces.v1','backend':'canmv_native','width':1280,'height':960,
             'hmirror':False,'vflip':False,'frames':[result,result,result],
             'robot_commands_sent':False,'image_exported':False,
             'replay_note':'One saved JPEG repeated; not a temporal stability or firmware LAB parity test.'}
    output.mkdir(parents=True,exist_ok=True)
    feature=output/'board_features.json';feature.write_text(json.dumps(payload))
    started=time.perf_counter();value=localize(feature,output)
    selected=value['vision']['selected']
    return {'run':folder.name,'status':'localized','Base_mm':value['cube_center_Base_mm'],
            'selected_kind':selected['kind'],'selected_value_floor':selected.get('value_floor'),
            'face_disagreement_mm':value['vision']['face_center_disagreement_mm'],
            'payload_bytes':len(json.dumps(payload).encode()),'PC_pose_s':time.perf_counter()-started}


def replay(folder, output, source='auto'):
    features = folder/'board_features.json'
    if source == 'features' or (source == 'auto' and features.exists()):
        if not features.exists():
            raise RuntimeError('Cannot read '+str(features))
        started = time.perf_counter()
        result = localize(features, output)
        selected = result['vision']['selected']
        return {'run':folder.name, 'status':'localized',
                'sample_source':'recorded_board_features',
                'Base_mm':result['cube_center_Base_mm'],
                'selected_kind':selected['kind'],
                'selected_detector':selected['detector'],
                'selected_frames':selected.get('selected_frame_indices'),
                'PC_pose_s':time.perf_counter()-started}
    result = replay_image(folder, output)
    result['sample_source'] = 'single_jpeg_approximation'
    return result


def main(argv=None):
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('runs',type=Path,nargs='+',help='saved run directories containing board_features.json or frame.jpg')
    parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--source',choices=('auto','features','image'),default='auto',
                        help='Prefer recorded multi-frame features; image replay approximates firmware extraction')
    args=parser.parse_args(argv);summary=[]
    for index,folder in enumerate(args.runs,1):
        try:row=replay(folder,args.output/f'{index:02d}_{folder.name}',args.source)
        except RuntimeError as exc:row={'run':str(folder),'status':'failed','reason':str(exc)}
        summary.append(row);print(json.dumps(row,ensure_ascii=False))
    args.output.mkdir(parents=True,exist_ok=True)
    (args.output/'summary.json').write_text(json.dumps(summary,ensure_ascii=False,indent=2)+'\n')
    return int(any(row['status']=='failed' for row in summary))

if __name__=='__main__':raise SystemExit(main())
