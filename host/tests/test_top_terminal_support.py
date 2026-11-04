"""Fullbbox endpoint observations retain the original measured top line."""
import importlib.util
import json
import unittest
from unittest.mock import patch
from host.project_paths import ROOT
HAS_VISION=importlib.util.find_spec('cv2')is not None
if HAS_VISION:
    import cv2
    import numpy as np
    from device.k230.vision import physical_side as p,cube_corners,ridge_profiles as g,separator_profiles as s
LOCAL=ROOT/'data/vision/acceptance_20261104'


@unittest.skipUnless(HAS_VISION,'Dedicated local CV environment')
class FullTopTerminalSupportTests(unittest.TestCase):
    def context(self,name):
        path=LOCAL/'inventory.json'
        if not path.exists():self.skipTest('Unversioned source inventory unavailable')
        item=next(v for v in json.loads(path.read_text())['images']if v['id']==name)
        image=cv2.imread(item['image_path'])
        if image is None:self.skipTest('Unversioned original photograph unavailable')
        return cube_corners.prepare(image,None)

    def test_real_G_and_B_fullwidth_gaps_do_not_move_measured_top_edges(self):
        snapshot=LOCAL/'residual_followup_20261104/table_cases/top_terminal_audit.json'
        if not snapshot.exists():self.skipTest('Unversioned pre-port top-edge audit unavailable')
        originals={v['id']:v for v in json.loads(snapshot.read_text())['rows']}
        for name,method in(('H034',g.measure),('H100',s.measure)):
            with self.subTest(image=name):
                ctx=self.context(name);im,hsv,box=ctx['smooth'],ctx['hsv'],ctx['local_bbox']
                cue=s.cue24(im,hsv,box,p.fit,p.residual)
                edge,details=method(im,hsv,box,p._fit_edge,p.fit,p.residual,cue=cue)
                self.assertEqual(json.loads(json.dumps(edge)),originals[name]['top_edge'])
                terminal=details['terminal_support']
                self.assertEqual(terminal['gap_reference'],'full_source_bbox')
                self.assertEqual(terminal['observed_coverage'],[0.,1.])
                self.assertFalse(terminal['grants_measured_vertical_edge'])
                self.assertGreater(terminal['right_gap_px'],12)
                self.assertNotIn('scores',terminal)
                if name=='H034':
                    self.assertTrue(terminal['preserves_measured_core'])
                    core=details['measured_core_support']
                    self.assertLessEqual(terminal['first_point_px'][0],core['first_point_px'][0])
                    self.assertGreaterEqual(terminal['last_point_px'][0],core['last_point_px'][0])
                    self.assertFalse(terminal['original_RGB8_step_evidence'])
                    self.assertAlmostEqual(terminal['minimum_prominence_DN'],2.)
                else:
                    self.assertEqual(terminal['right_gap_px'],15)
                    self.assertEqual(terminal['contrast_span_px'],5.)
                    self.assertTrue(terminal['original_RGB8_step_evidence'])

    def test_a_dim_hole_between_core_inliers_cannot_retract_the_verified_span(self):
        image=np.zeros((96,200,3),np.uint8);image[:]=[220,60,0]
        image[38:43,:,1]=85;image[38:43,94:106,1]=60
        hsv=cv2.cvtColor(image,cv2.COLOR_BGR2HSV);box=[10,8,180,80]
        edge,details=g.measure(image,hsv,box,p._fit_edge,p.fit,p.residual,
            cue=([0.,1.,-40.],{'method':'synthetic_explicit_proposal'}))
        self.assertGreaterEqual(edge[1]['support'],45)
        self.assertLess(edge[1]['support'],64)
        core=details['measured_core_support'];terminal=details['terminal_support']
        self.assertTrue(terminal['preserves_measured_core'])
        self.assertLessEqual(terminal['first_point_px'][0],core['first_point_px'][0])
        self.assertGreaterEqual(terminal['last_point_px'][0],core['last_point_px'][0])
        self.assertEqual(terminal['left_gap_px'],0)
        self.assertEqual(terminal['right_gap_px'],0)

    def test_uniform_blue_cannot_obtain_G_quality_from_endpoint_observation(self):
        image=np.zeros((96,200,3),np.uint8);image[:]=[220,60,0]
        hsv=cv2.cvtColor(image,cv2.COLOR_BGR2HSV)
        with self.assertRaises(ValueError):
            g.measure(image,hsv,[10,8,180,80],p._fit_edge,p.fit,p.residual,
                cue=([0.,1.,-40.],{'method':'synthetic_explicit_proposal'}))
