#!/usr/bin/env python3
"""Offline feedback quality and provisional gripper FK; no hardware access."""

import sys as _sys
from pathlib import Path as _Path
_sys.path.insert(0, str(_Path(__file__).resolve().parents[2]))
from host.project_paths import ROOT as REPO_ROOT, bootstrap as _bootstrap
_bootstrap(include_tools=True)
import json
from pathlib import Path
import sys
import numpy as np

HERE = (REPO_ROOT/'data/k230')
PROJECT = REPO_ROOT
from host.arm.joint_mapping import JointMapping
from gripper_geometry import PickGeometry

def main():
    root = HERE/'grasp_pairs'
    out = root/'analysis_20261013'
    out.mkdir(exist_ok=True)
    groups = json.loads((root/'collection.json').read_text())['groups']
    geometry = PickGeometry(PROJECT, JointMapping.load(PROJECT/'configs/arm/joint_mapping.json'))
    result = []
    for group in groups:
        record = json.loads((root/group['folder']/'record.json').read_text())
        samples = record['samples_before']+record['samples_after']
        if len(samples) != 6:
            raise RuntimeError('Expected three feedback samples before and after image capture')
        counts = [int(round(np.median([s[str(i)]['position'] for s in samples]))) for i in range(1,7)]
        q = geometry.angles(counts)
        midpoint, tips, lows, transforms = geometry.metrics(q)
        summary = []
        for sid in range(1,7):
            positions = [s[str(sid)]['position'] for s in samples]
            summary.append({'id': sid, 'median_count': counts[sid-1],
                            'min_count': min(positions), 'max_count': max(positions),
                            'span_counts': max(positions)-min(positions),
                            'speeds_raw': [s[str(sid)]['speed'] for s in samples],
                            'torque_states': [s[str(sid)]['torque'] for s in samples]})
        result.append({'group': group['group'], 'folder': group['folder'],
                       'encoder_counts': counts, 'feedback_summary': summary,
                       'joint_angles_deg': np.rad2deg(q).tolist(),
                       'tip_midpoint_Base_mm': (midpoint*1000).tolist(),
                       'tip_centers_Base_mm': {k: (v*1000).tolist() for k,v in tips.items()},
                       'mesh_lowest_z_Base_mm': {k: float(v*1000) for k,v in lows.items()},
                       'T_Base_from_fixed_jaw_m': transforms['wan2'].tolist(),
                       'T_Base_from_moving_jaw_m': transforms['zhua'].tolist(),
                       'camera_capture': record['camera_capture'],
                       'reference_is_physical_cube_center': False,
                       'reference_note': 'Mean midpoint of distal 2mm STL tip regions. Unknown grasp insertion depth and geometry bias remain. No old virtual-center or XY correction applied.'})
    path = out/'encoder_fk.json'
    path.write_text(json.dumps(result, ensure_ascii=False, indent=2)+'\n')
    print('SAVED', path)
    for row in result:
        print(json.dumps({'group':row['group'], 'counts':row['encoder_counts'],
                          'spans':[s['span_counts'] for s in row['feedback_summary']],
                          'tip_midpoint_Base_mm':row['tip_midpoint_Base_mm']}))

if __name__ == '__main__': main()
