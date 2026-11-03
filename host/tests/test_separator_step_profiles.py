"""Arbitrary-sign RGB-step proposals retain independent64station evidence.

Known captures check pixel/quality provenance, not physical XYZ/grasp accuracy.
RGB24 support is never substituted for the original45of64 measured edge.
"""
import importlib.util
import json
import unittest
from unittest.mock import patch

from host.project_paths import ROOT
HAS_VISION=importlib.util.find_spec('cv2')is not None
if HAS_VISION:
    import cv2
    import numpy as np
    from device.k230.vision import separator_profiles as s,physical_side as p,cube_corners,ridge_profiles

BASE=ROOT/'data/car/full_process_tracking_20261103/post_static_anchor_corrections_01'
CAPTURE=ROOT/'data/vision/near_supplement/20261104_005624'
AUDIT=ROOT/'data/vision/acceptance_20261104/residual_before_six'


@unittest.skipUnless(HAS_VISION,'Dedicated local CV environment')
class SeparatorStepProfilesTests(unittest.TestCase):
    def synthetic_step(self,reverse=False):
        image=np.zeros((96,180,3),np.uint8)
        a,b=([90,35,0],[210,85,0])if reverse else([210,85,0],[90,35,0])
        image[:42,:]=a;image[42:,:]=b
        hsv=cv2.cvtColor(image,cv2.COLOR_BGR2HSV)
        return cv2.GaussianBlur(image,(3,3),.7),hsv,[4,4,170,90]

    def test_both_RGB_step_polarities_have_real64station_evidence(self):
        for reverse in(False,True):
            with self.subTest(reverse=reverse):
                im,hsv,box=self.synthetic_step(reverse)
                cue=s.cue24(im,hsv,box,p.fit,p.residual)
                self.assertEqual(cue[1]['method'],'same_frame_RGB_step_cue')
                self.assertLessEqual(cue[1]['support'],24)
                edge,details=s.measure_rgb_step(im,hsv,box,p._fit_edge,p.fit,p.residual,cue)
                self.assertEqual(edge[1]['sampled_stations'],64)
                self.assertEqual(edge[1]['support'],64)
                self.assertLessEqual(edge[1]['rms_px'],3)
                self.assertEqual(edge[1]['appearance_method'],'same_frame_original_RGB8_step')
                self.assertTrue(edge[1]['original_RGB8_step_evidence'])
                self.assertEqual(details['separator_evidence']['both_blue_stations'],32)
                self.assertEqual(details['signed_median_BGR_jump_DN'][0]>0,reverse)
                self.assertNotIn('native_contrast_profile',details)
                self.assertEqual(details['terminal_support']['gap_reference'],'full_source_bbox')
                self.assertEqual(details['terminal_support']['left_gap_px'],0)
                self.assertEqual(details['terminal_support']['right_gap_px'],0)

    def test_RGB_fallback_uses_the_two_already_captured24column_arrays(self):
        im,hsv,box=self.synthetic_step();remap=cv2.remap
        with patch.object(cv2,'remap',wraps=remap)as calls:
            cue=s.cue24(im,hsv,box,p.fit,p.residual)
        self.assertEqual(cue[1]['method'],'same_frame_RGB_step_cue')
        self.assertEqual(calls.call_count,2)
        self.assertTrue(all(call.args[1].shape[0]==24 for call in calls.call_args_list))
        with patch.object(cv2,'remap',wraps=remap)as calls:
            s.measure_rgb_step(im,hsv,box,p._fit_edge,p.fit,p.residual,cue)
        self.assertFalse(any(call.args[1].shape[0]==24 for call in calls.call_args_list))

    def test_a_proposed_line_cannot_replace45measured_stations(self):
        im,hsv,box=self.synthetic_step()
        im[:,60:]=[130,40,0]
        hsv=cv2.cvtColor(im,cv2.COLOR_BGR2HSV)
        cue=([0.,1.,-41.5],{'method':'same_frame_RGB_step_cue','support':24,'columns':24})
        with self.assertRaisesRegex(ValueError,'contrast|consensus|coherent measured internal edge'):
            s.measure_rgb_step(im,hsv,box,p._fit_edge,p.fit,p.residual,cue)

    def test_uniform_blue_does_not_manufacture_an_internal_step(self):
        image=np.zeros((96,180,3),np.uint8);image[:]=[130,40,0]
        hsv=cv2.cvtColor(image,cv2.COLOR_BGR2HSV)
        with self.assertRaisesRegex(ValueError,'RGB-step cue'):
            s.cue24(image,hsv,[4,4,170,90],p.fit,p.residual)
        self.assertIsNone(s._rgb_terminal(image,hsv,[4,4,170,90],[0.,1.,-42.],p._native_contrasts))

    def test_real_negative_B_G_steps_have_distinct_cue_and_measurement_support(self):
        index=ROOT/'data/vision/acceptance_20261104/inventory.json'
        if not index.exists():self.skipTest('Unversioned original capture inventory unavailable')
        inventory={v['id']:v for v in json.loads(index.read_text())['images']}
        for name,support,rms in(('H061',61,1.226898234979246),('H102',64,.9799643541951545)):
            with self.subTest(image=name):
                image=cv2.imread(inventory[name]['image_path'])
                if image is None:self.skipTest('Unversioned real original unavailable')
                ctx=cube_corners.prepare(image,None);im,hsv,box=ctx['smooth'],ctx['hsv'],ctx['local_bbox']
                cue=s.cue24(im,hsv,box,p.fit,p.residual)
                self.assertEqual(cue[1]['method'],'same_frame_RGB_step_cue')
                self.assertEqual(cue[1]['support'],23)
                self.assertLess(cue[1]['B_observability']['median_B_jump_DN'],-100)
                with patch.object(p,'_separator_evidence',wraps=p._separator_evidence)as check:
                    edge,details=s.measure_rgb_step(im,hsv,box,p._fit_edge,p.fit,p.residual,cue)
                self.assertEqual(check.call_count,1)
                self.assertEqual(edge[1]['support'],support)
                self.assertAlmostEqual(edge[1]['rms_px'],rms,places=6)
                self.assertEqual(details['separator_evidence']['support'],32)
                self.assertLess(details['signed_median_BGR_jump_DN'][0],0)
                self.assertLess(details['signed_median_BGR_jump_DN'][1],0)
                self.assertEqual(edge[1]['station_evidence'],'original_RGB8_contrast_pairs')
                terminal=details['terminal_support']
                self.assertEqual(terminal['gap_reference'],'full_source_bbox')
                self.assertEqual(terminal['observed_coverage'],[0.,1.])
                self.assertEqual(terminal['edge_ownership'],'RGB_outline_proposal')
                self.assertFalse(terminal['grants_measured_vertical_edge'])
                self.assertEqual(terminal['right_gap_px'],18 if name=='H061' else 11)
                self.assertEqual(terminal['left_gap_px'],5 if name=='H061' else 0)
                self.assertNotIn('scores',terminal)

    def retained_material(self,name):
        if name.startswith('new12/'):
            folder=CAPTURE;stem='frame_%02d'%int(name.rsplit('_',1)[1])
        else:
            group,stem=name.split('/');folder=BASE/group
        meta_path=folder/(stem+'.json')
        if not meta_path.exists():self.skipTest('Unversioned retained capture unavailable')
        meta=json.loads(meta_path.read_text());im=cv2.imread(str(folder/meta['image_file']))
        return cube_corners.prepare(im,meta['bbox_px'])

    def test_all_existing_G_cues_and_successful_B_G_top_edges_remain_exact(self):
        snapshot=AUDIT/'rgb_step_cue_probe.json'
        if not snapshot.exists():self.skipTest('Unversioned pre-port cue snapshot unavailable')
        checked=measured=0
        for row in json.loads(snapshot.read_text())['rows']:
            if not row.get('old_G_cue_exactly_retained'):continue
            with self.subTest(image=row['identity']):
                ctx=self.retained_material(row['identity']);im,hsv,box=ctx['smooth'],ctx['hsv'],ctx['local_bbox']
                before=row['cue'];current=s.cue24(im,hsv,box,p.fit,p.residual)
                self.assertEqual(list(current),before)
                self.assertEqual(current[1]['method'],'same_frame_G_peak_cue')
                method=s.measure if before[1]['B_observability']['positive_B_step_observable']else ridge_profiles.measure
                try:old_edge,old_details=method(im,hsv,box,p._fit_edge,p.fit,p.residual,cue=before)
                except ValueError as exc:
                    with self.assertRaises(ValueError):method(im,hsv,box,p._fit_edge,p.fit,p.residual,cue=current)
                else:
                    new_edge,new_details=method(im,hsv,box,p._fit_edge,p.fit,p.residual,cue=current)
                    self.assertEqual(new_edge,old_edge);self.assertEqual(new_details,old_details);measured+=1
                checked+=1
        self.assertEqual(checked,41)
        self.assertGreaterEqual(measured,28)
