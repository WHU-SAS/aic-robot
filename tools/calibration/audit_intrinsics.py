#!/usr/bin/env python3
"""Fit and check K230 ChArUco intrinsics; original captures remain unchanged."""

import sys as _sys
from pathlib import Path as _Path
_sys.path.insert(0, str(_Path(__file__).resolve().parents[2]))
from host.project_paths import ROOT as REPO_ROOT, bootstrap as _bootstrap
_bootstrap(include_tools=True)
import json
from pathlib import Path
import cv2
import numpy as np

import argparse

def main(argv=None):
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--images', type=Path, required=True, help='directory of accepted complete-board JPG views')
    parser.add_argument('--output', type=Path, default=REPO_ROOT/'data/k230/calibration_results/intrinsics_quality.json')
    args=parser.parse_args(argv)
    IMAGES=args.images
    board = cv2.aruco.CharucoBoard((5,7),28.54,14.27,
                                  cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_5X5_100))
    board.setLegacyPattern(False)
    detector = cv2.aruco.CharucoDetector(board)
    objects, points, names = [], [], []
    size = None
    for path in sorted(IMAGES.glob('*.jpg')):
        im = cv2.imread(str(path))
        if im is None: raise RuntimeError('Cannot read image: '+str(path))
        current_size = (im.shape[1], im.shape[0])
        if size is not None and current_size != size:
            raise RuntimeError('Mixed image resolutions')
        size = current_size
        c, ids, _, _ = detector.detectBoard(cv2.cvtColor(im,cv2.COLOR_BGR2GRAY))
        if ids is None or len(ids) != 24:
            raise RuntimeError('Full pattern no longer detected: '+path.name)
        objects.append(board.getChessboardCorners()[ids.flatten()].astype(np.float32))
        points.append(c.reshape(-1,2).astype(np.float32))
        names.append(path.name)

    if not objects: raise RuntimeError('No complete ChArUco views in '+str(IMAGES))
    fit = cv2.calibrateCameraExtended(objects,points,size,None,None)
    rms,K,D,rv,tv,std_intr,std_extr,per_view = fit
    combined=np.concatenate(points)
    coverage = {
        'corner_x_range_px':[float(combined[:,0].min()),float(combined[:,0].max())],
        'corner_y_range_px':[float(combined[:,1].min()),float(combined[:,1].max())],
        'corner_hull_image_fraction': float(cv2.contourArea(cv2.convexHull(combined)) / (size[0]*size[1])),
    }
    tilts=[]
    normal_vectors=[]
    for r in rv:
        R,_=cv2.Rodrigues(r)
        normal=R[:,2]
        if normal[2]<0: normal=-normal
        normal_vectors.append(normal)
        tilts.append(float(np.degrees(np.arccos(np.clip(abs(R[2,2]),0,1)))))
    coverage['tilt_from_frontal_degrees_range']=[min(tilts),max(tilts)]
    coverage['normal_x_range']=[float(min(n[0] for n in normal_vectors)),float(max(n[0] for n in normal_vectors))]
    coverage['normal_y_range']=[float(min(n[1] for n in normal_vectors)),float(max(n[1] for n in normal_vectors))]

    # Three contiguous held-out groups reduce leakage from adjacent timed captures.
    folds=[]
    for withheld in np.array_split(np.arange(len(names)),3):
        ids=set(map(int,withheld))
        training=[i for i in range(len(names)) if i not in ids]
        train_rms,k,d,_,_=cv2.calibrateCamera([objects[i] for i in training],
                                            [points[i] for i in training],size,None,None)
        errors=[]
        for i in withheld:
            ok,r,t=cv2.solvePnP(objects[i],points[i],k,d)
            if not ok: raise RuntimeError('Held-out pose solve failed')
            projected,_=cv2.projectPoints(objects[i],r,t,k,d)
            residual=projected.reshape(-1,2)-points[i]
            errors.append({'file':names[i],'rms_px':float(np.sqrt(np.mean(np.sum(residual**2,axis=1))))})
        folds.append({'train_views':len(training),'held_out_views':len(ids),
                      'train_rms_px':float(train_rms),'camera_matrix':k.tolist(),
                      'held_out_rms_px':float(np.sqrt(np.mean([v['rms_px']**2 for v in errors]))),
                      'views':errors})
    focal_changes=[max(abs(f['camera_matrix'][0][0]/K[0,0]-1),
                       abs(f['camera_matrix'][1][1]/K[1,1]-1)) for f in folds]
    held_out_rms=float(np.sqrt(np.mean([v['rms_px']**2 for f in folds for v in f['views']])))
    report={
        'image_size':list(size),'views':len(names),'rms_px':float(rms),
        'camera_matrix':K.tolist(),'distortion_coefficients':D.flatten().tolist(),
        'intrinsic_standard_deviations':std_intr.flatten().tolist(),
        'coverage':coverage,'blocked_validation_folds':folds,
        'held_out_rms_px':held_out_rms,
        'max_focal_change_across_folds_percent':float(max(focal_changes)*100),
        'per_view_rms_px':[{'file':n,'rms_px':float(e)} for n,e in zip(names,per_view.flatten())],
        'note':'Held-out errors still fit board pose per image; this is not independent measured physical accuracy.',
    }
    args.output.parent.mkdir(parents=True,exist_ok=True)
    args.output.write_text(json.dumps(report,ensure_ascii=False,indent=2))
    print(json.dumps({k:report[k] for k in ['rms_px','held_out_rms_px','max_focal_change_across_folds_percent','coverage','intrinsic_standard_deviations']},indent=2))


if __name__=='__main__':main()
