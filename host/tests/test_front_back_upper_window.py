"""The used upper-right edge is independent of unused short outline sides.

H034/H100 retain original paired calibration. This checks source layouts and
the user-selected0..20mm table policy, not physical corner truth. In particular
H100's stored A/B measurements were later found on the wrong image feature;
even their low modelRMS cannot validate that physical-edge interpretation.
"""
import importlib.util
import json
import unittest
from unittest.mock import patch
from host.project_paths import ROOT
HAS_VISION=importlib.util.find_spec('cv2')is not None
if HAS_VISION:
    import cv2
    from device.k230.vision import physical_side as p,cube_corners,cube_outline,cube_pose,cube_observation
LOCAL=ROOT/'data/vision/acceptance_20261104'


@unittest.skipUnless(HAS_VISION,'Dedicated local CV environment')
class FrontBackUpperWindowTests(unittest.TestCase):
    def materials(self,name):
        inventory=LOCAL/'inventory.json';snapshot=LOCAL/'residual_followup_20261104/table_cases/upper_vertical_window.json'
        if not inventory.exists()or not snapshot.exists():self.skipTest('Unversioned real source measurements unavailable')
        item=next(v for v in json.loads(inventory.read_text())['images']if v['id']==name)
        case=next(c for c in item['cases']if c['calibration_role']=='original')
        cfg=json.loads((ROOT/case['calibration_json']).read_text());image=cv2.imread(item['image_path'])
        if image is None:self.skipTest('Unversioned original JPEG unavailable')
        saved=next(v for v in json.loads(snapshot.read_text())['rows']if v['id']==name)
        measured=saved['replay']['candidate_geometry'][0]['measurement']
        return cfg,cube_corners.prepare(image,None),measured

    def test_real_upper_line_does_not_need_a_complete_six_corner_hull(self):
        for name in('H034','H100'):
            with self.subTest(image=name):
                cfg,ctx,unused=self.materials(name)
                with patch.object(cube_outline,'_proposal',side_effect=AssertionError('Unused six-corner topology must not run')):
                    a,b,hsv,audit=cube_corners._front_back_upper_window(p,ctx,
                        cfg['intrinsics'],cfg['extrinsics'],cfg['table_plane'])
                self.assertFalse(audit['six_corner_layout_required'])
                self.assertFalse(audit['pose_used']);self.assertFalse(audit['table_offset_used'])
                x,y,w,h=ctx['local_bbox']
                self.assertEqual(a,[x+w-1,y+.1*h])
                self.assertAlmostEqual(b[1],y+.35*h)
                from device.k230.vision import edge_profiles
                edge=cube_corners._measure(p,edge_profiles,ctx['smooth'],hsv,a,b)
                self.assertEqual(edge[1]['sampled_stations'],64)
                self.assertEqual(edge[1]['support'],64)
                self.assertLess(edge[1]['rms_px'],.5)

    def test_independent_source_layout_and_current_table_policy_are_not_physical_truth(self):
        for name in('H034','H100'):
            with self.subTest(image=name):
                cfg,ctx,saved=self.materials(name);ox,oy=ctx['roi_px'][:2]
                def local(line):
                    a,b,c=line;return[a,b,c+a*ox+b*oy]
                edges=[(local(e['line_px']),dict(e['quality']),[local(v)for v in e['half_lines_px']])for e in saved['observed_edges'][:4]]
                quad=[[v[0]-ox,v[1]-oy]for v in saved['image_points_px'][:4]]
                steps=list(cube_corners.front_back_steps(p,ctx['smooth'],ctx['hsv'],ctx,
                    edges,quad,cfg['intrinsics'],cfg['extrinsics'],cfg['table_plane']))
                candidates=[e['measurement']for e in steps if e['stage']=='cube_candidate']
                self.assertEqual(len(candidates),1,steps);measurement=candidates[0]
                self.assertEqual(measurement['evidence_type'],'nonplanar_cube_face_back_top')
                self.assertEqual(len(measurement['observed_edges']),7)
                self.assertLessEqual(max(measurement['half_corner_errors_px']),3.)
                observed=measurement['observed_edges']
                actual=p.intersect(observed[5]['line_px'],observed[6]['line_px'])
                self.assertAlmostEqual(actual[0],measurement['image_points_px'][5][0],places=5)
                self.assertAlmostEqual(actual[1],measurement['image_points_px'][5][1],places=5)
                intr=cfg['intrinsics'];K=intr['camera_matrix'];D=intr['distortion_coefficients'];T=cfg['extrinsics']['T_Base_from_Camera_mm']
                normal=cfg['table_plane']['normal'];up=[sum(T[k][j]*normal[k]for k in range(3))for j in range(3)]
                solved=cube_pose.solve_cube(measurement['object_points_mm'],measurement['image_points_px'],K,D,up)
                self.assertLess(solved['reprojection_RMS_px'],1.1)
                if name=='H034':
                    raw=cube_observation.build_result(measurement,K,D,T,cfg['table_plane'])
                    self.assertGreaterEqual(raw['table_geometry']['center_height_mm'],0.)
                    self.assertLessEqual(raw['table_geometry']['center_height_mm'],20.)
                else:
                    with self.assertRaisesRegex(ValueError,'calibrated table'):
                        cube_observation.build_result(measurement,K,D,T,cfg['table_plane'])
