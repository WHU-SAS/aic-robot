"""Frontal recovery uses actual image evidence and preserves good profiles.

Local photos are development fixtures, not independent grasp accuracy labels.
"""
import hashlib
import importlib.util
import json
import unittest
from unittest.mock import patch
from host.project_paths import ROOT

HAS_VISION=importlib.util.find_spec('cv2') is not None
if HAS_VISION:
    import cv2
    from device.k230.vision import physical_side as p,cube_corners
    from device.k230.vision.candidate_association import associate_fine_candidate

LIT=ROOT/'data/vision/lit_validation/20261104_100443_212133'
OLD=ROOT/'data/car/full_process_tracking_20261103/post_static_anchor_corrections_01/live_static_02'


@unittest.skipUnless(HAS_VISION,'Dedicated local CV environment')
class FrontalRecoveryPriorityTests(unittest.TestCase):
    def saved(self,folder,stem):
        paths=[folder/'config.json',folder/(stem+'.json'),folder/(stem+'.jpg')]
        if any(not v.is_file()for v in paths):self.skipTest('Local camera fixture unavailable')
        cfg=json.loads(paths[0].read_text())['car'];meta=json.loads(paths[1].read_text())
        self.assertEqual(hashlib.sha256(paths[2].read_bytes()).hexdigest(),meta['image_sha256'])
        return cfg,meta,cv2.imread(str(paths[2]))

    def solve(self,cfg,meta,image):
        return list(p.localize_steps(image,meta['frame'],meta['bbox_px'],
            cfg['intrinsics'],cfg['extrinsics'],cfg['table_plane']))

    def test_segmented_back_top_does_not_replace_a_good_profile(self):
        cfg,meta,image=self.saved(OLD,'verify_20')
        prepared=cube_corners.prepare(image,meta['bbox_px'])
        self.assertEqual(prepared['selector']['route_hint'],'profile')
        self.assertFalse(prepared['selector']['has_upper_back_proposal'])
        self.assertTrue(prepared['selector']['has_merged_upper_back_proposal'])
        with patch.object(p,'_cube_steps',side_effect=AssertionError('Good profile replaced by new cube route')):
            events=self.solve(cfg,meta,image)
        obs=events[-1]['observation']
        self.assertEqual(obs['status'],'localized',obs)
        self.assertEqual(obs['search']['recovery_route'],'profile')
        self.assertEqual([v['route']for v in events if v['stage']=='recovery_selected'],['profile'])

    def test_real_frontal_split_back_top_recovers_after_profile_failure(self):
        cfg,meta,image=self.saved(LIT,'frame_04')
        with patch.object(p,'_original_localize_steps',side_effect=AssertionError('Recovered front ran old full prefix')):
            events=self.solve(cfg,meta,image)
        obs=events[-1]['observation']
        self.assertEqual(obs['status'],'localized',obs)
        self.assertEqual([v['route']for v in events if v['stage']=='recovery_selected'],['profile','cube'])
        self.assertEqual(obs['quality']['evidence_type'],'nonplanar_cube_five_corners')
        self.assertLessEqual(obs['quality']['reprojection_rms_px'],3.)
        self.assertEqual(len(obs['payload']['cube_pose']['image_points_px']),5)
        self.assertEqual(associate_fine_candidate(obs,meta['paired_coarse'])['status'],'matched')
        self.assertFalse(obs['motion_performed'])
        self.assertFalse(obs['xy_compensation_applied'])


if __name__=='__main__':unittest.main()
