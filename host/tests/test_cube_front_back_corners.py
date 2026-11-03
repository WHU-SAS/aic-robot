"""Actual same-image G-face reuse plus independently measured upper corners.

Local originals are development regressions, not independent physical accuracy
or grasp labels. CV imports remain separate from the host deploy environment.
"""
import copy
import hashlib
import importlib.util
import json
import unittest
from contextlib import ExitStack
from unittest.mock import patch

from host.project_paths import ROOT

HAS_VISION=importlib.util.find_spec('cv2') is not None
if HAS_VISION:
    import cv2
    import numpy as np
    from device.k230.vision import (cube_corners as corners,physical_side as p,
        profile_recovery as profile,ridge_profiles as ridge,edge_profiles as profiles,
        cube_observation as cube)
    from device.k230.vision.runtime import localize_precise
    from device.k230.vision.candidate_association import associate_fine_candidate

SUPPLEMENT=ROOT/'data/vision/near_supplement/20261104_005624'
FRONTAL=ROOT/'data/car/full_process_tracking_20261103/post_static_anchor_corrections_01/live_static_06_final_timing'


@unittest.skipUnless(HAS_VISION,'Dedicated local CV environment')
class CubeFrontBackCornersTests(unittest.TestCase):
    def load(self,folder,stem):
        paths=[folder/'config.json',folder/(stem+'.json'),folder/(stem+'.jpg')]
        if any(not path.is_file()for path in paths):
            self.skipTest('Retained camera originals are local-only: '+str(folder))
        cfg=json.loads(paths[0].read_text())['car'];record=json.loads(paths[1].read_text())
        self.assertEqual(hashlib.sha256(paths[2].read_bytes()).hexdigest(),record['image_sha256'])
        image=cv2.imread(str(paths[2]));self.assertIsNotNone(image)
        return cfg,record,image

    def measured_front(self,cfg,record,image):
        """Retain real measurement objects without inventing a G/edge result."""
        observed=[]
        with ExitStack()as stack:
            original=ridge.measure
            def ridge_call(*args,**kwargs):
                edge,details=original(*args,**kwargs);observed.append(edge)
                return edge,details
            stack.enter_context(patch.object(ridge,'measure',ridge_call))
            spread=profiles.spread_edge
            def spread_call(*args,**kwargs):
                result=spread(*args,**kwargs)
                if result is not None:observed.append(result[0])
                return result
            stack.enter_context(patch.object(profiles,'spread_edge',spread_call))
            steps=p._edge_steps
            def step_call(*args,**kwargs):
                for event in steps(*args,**kwargs):
                    if event['stage']=='edge_complete':observed.append(event['edge'])
                    yield event
            stack.enter_context(patch.object(p,'_edge_steps',step_call))
            observation=list(profile.recover_steps(image,record['frame'],record['bbox_px'],
                cfg['intrinsics'],cfg['extrinsics'],cfg['table_plane'],refine_cube=False))[-1]['observation']
        self.assertEqual(observation['status'],'localized',observation)
        self.assertEqual(observation['quality']['evidence_type'],'physical_face_with_G_ridge')
        prepared=corners.prepare(image,record['bbox_px']);ox,oy=prepared['roi_px'][:2]
        q=[[v[0]-ox,v[1]-oy]for v in observation['payload']['face_quad_px']]
        edges=[]
        for a,b in zip(q,q[1:]+q[:1]):
            edge=min(observed,key=lambda e:p.residual(a,e[0])+p.residual(b,e[0]))
            self.assertLess(p.residual(a,edge[0])+p.residual(b,edge[0]),1e-3)
            edges.append(edge)
        return prepared,edges,q

    def measure(self,cfg,prepared,edges,q):
        return list(corners.front_back_steps(p,prepared['smooth'],prepared['hsv'],prepared,
            edges,q,cfg['intrinsics'],cfg['extrinsics'],cfg['table_plane']))

    def assert_runtime_candidate(self,cfg,record,measurement):
        intr=cfg['intrinsics'];K=[list(row)for row in intr['camera_matrix']]
        K[0]=[v*1280/intr['image_width']for v in K[0]];K[1]=[v*960/intr['image_height']for v in K[1]]
        raw=cube.build_result(measurement,K,intr['distortion_coefficients'],
            cfg['extrinsics']['T_Base_from_Camera_mm'],cfg['table_plane'])
        observation=localize_precise(None,record['frame'],intr,cfg['extrinsics'],cfg['table_plane'],
            localizer=lambda *args:raw)
        self.assertEqual(observation['status'],'localized',observation)
        self.assertEqual(observation['quality']['evidence_type'],'nonplanar_cube_face_back_top')
        self.assertEqual(observation['quality']['reprojection_limit_px'],3.)
        self.assertLessEqual(observation['quality']['reprojection_rms_px'],3.)
        pose=observation['payload']['cube_pose']
        self.assertEqual(len(pose['cube_projected_corners_px']),8)
        self.assertTrue(pose['table_geometry']['accepted'])
        self.assertLessEqual(pose['table_geometry']['object_up_deviation_deg'],20.)
        for key in('frame_id','capture_timestamp_ms','clock_domain','image_size_px'):
            self.assertEqual(observation[key],record['frame'][key])
        association=associate_fine_candidate(observation,record['paired_coarse'])
        self.assertEqual(association['status'],'matched',association)
        self.assertFalse(observation['motion_performed'])
        return observation

    def test_two_difficult_fronts_gain_seven_honest_source_edges_and_twelve_independent_half_errors(self):
        for number in(3,4):
            cfg,record,image=self.load(SUPPLEMENT,'frame_%02d'%number)
            prepared,edges,q=self.measured_front(cfg,record,image)
            before=copy.deepcopy((edges,q));events=self.measure(cfg,prepared,edges,q)
            self.assertEqual(events[-1]['stage'],'front_back_complete')
            self.assertEqual(events[-1]['candidate_count'],1,events)
            candidates=[e for e in events if e['stage']=='cube_candidate']
            self.assertEqual(len(candidates),1)
            measurement=candidates[0]['measurement']
            self.assertEqual(measurement['corner_labels'],['A','B','C','D','E','F'])
            self.assertEqual(len(measurement['object_points_mm']),6)
            self.assertEqual(len(measurement['observed_edges']),7)
            self.assertEqual(len(measurement['half_corner_errors_px']),12)
            self.assertLessEqual(max(measurement['half_corner_errors_px']),3.)
            for edge in measurement['observed_edges']:
                self.assertEqual(edge['quality']['sampled_stations'],64)
                self.assertGreaterEqual(edge['quality']['support'],45)
                self.assertLessEqual(edge['quality']['rms_px'],3.)
            top=measurement['observed_edges'][0]['quality']
            self.assertEqual(top['appearance_method'],'same_frame_G_ridge')
            self.assertIs(top['original_RGB8_step_evidence'],False)
            self.assertEqual(top,edges[0][1])
            body_kind=measurement['upper_proposals']['right_silhouette']['body_mask_kind']
            self.assertEqual(measurement['observed_edges'][-1]['quality']['body_mask_kind'],body_kind)
            self.assertEqual(measurement['observed_edges'][-1]['quality']['edge_ownership'],
                'same_current_LAB_component_upper_outline'if body_kind=='configured_LAB'
                else'same_current_HSV_body_upper_outline')
            self.assertEqual((edges,q),before)
            self.assert_runtime_candidate(cfg,record,measurement)

    def test_measurement_reuses_front_edges_and_never_invokes_a_pose_solver(self):
        cfg,record,image=self.load(SUPPLEMENT,'frame_03')
        prepared,edges,q=self.measured_front(cfg,record,image)
        with patch.object(corners,'_measure',wraps=corners._measure)as measure,\
             patch.object(p,'pose_candidates',side_effect=AssertionError('measurement must not solve')):
            events=self.measure(cfg,prepared,edges,q)
        self.assertEqual(events[-1]['candidate_count'],1,events)
        self.assertEqual(measure.call_count,3)
        self.assertEqual([e['edge_name']for e in events if e['stage']=='front_back_edge_complete'],
            ['left_top_depth','back_top','independent_upper_right_silhouette'])

    def test_public_twelve_ordinary_originals_keep_full_same_frame_geometry(self):
        for number in range(1,13):
            with self.subTest(frame=number):
                cfg,record,image=self.load(SUPPLEMENT,'frame_%02d'%number)
                observation=list(p.localize_steps(image,record['frame'],record['bbox_px'],
                    cfg['intrinsics'],cfg['extrinsics'],cfg['table_plane']))[-1]['observation']
                self.assertEqual(observation['status'],'localized',observation)
                self.assertEqual(associate_fine_candidate(observation,record['paired_coarse'])['status'],'matched')
                if number in(3,4):
                    self.assertEqual(observation['quality']['evidence_type'],'nonplanar_cube_face_back_top')
                    self.assertEqual(len(observation['payload']['observed_edges']),7)

    def test_original_frontal_three_retain_direct_G_evidence_and_valid_public_geometry(self):
        for number in(1,2,3):
            cfg,record,image=self.load(FRONTAL,'fresh_%02d'%number)
            direct=list(profile.recover_steps(image,record['frame'],record['bbox_px'],
                cfg['intrinsics'],cfg['extrinsics'],cfg['table_plane']))[-1]['observation']
            self.assertEqual(direct['quality']['evidence_type'],'physical_face_with_G_ridge')
            self.assertLessEqual(direct['quality']['reprojection_rms_px'],2.)
            observation=list(p.localize_steps(image,record['frame'],record['bbox_px'],
                cfg['intrinsics'],cfg['extrinsics'],cfg['table_plane']))[-1]['observation']
            self.assertEqual(observation['status'],'localized',observation)
            self.assertEqual(associate_fine_candidate(observation,record['paired_coarse'])['status'],'matched')

    def test_a_single_blue_plane_and_blue_halo_cannot_invent_back_top_corners(self):
        cfg,unused,unused=self.load(SUPPLEMENT,'frame_03')
        for halo in(False,True):
            image=np.zeros((960,1280,3),dtype=np.uint8);image[:]=[80,110,150]
            if halo:cv2.rectangle(image,(496,556),(684,804),(170,90,60),-1)
            cv2.rectangle(image,(500,560),(680,800),(240,50,5),-1)
            prepared=corners.prepare(image,[500,560,181,241]);x,y,w,h=prepared['local_bbox']
            vertices=[[x,y],[x+w-1,y],[x+w-1,y+h-1],[x,y+h-1]]
            edges=[]
            for a,b in zip(vertices,vertices[1:]+vertices[:1]):
                events=list(p._edge_steps(prepared['smooth'],a,b,hsv=prepared['hsv']))
                edges.append(events[-1]['edge'])
            q=[p.intersect(edges[i-1][0],edges[i][0])for i in range(4)]
            events=self.measure(cfg,prepared,edges,q)
            with self.subTest(halo=halo):
                self.assertEqual(events[-1]['candidate_count'],0,events)
                self.assertFalse(any(e['stage']=='cube_candidate'for e in events))
                self.assertTrue(events[-1]['rejections'])


if __name__=='__main__':unittest.main()
