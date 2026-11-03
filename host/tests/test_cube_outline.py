"""Current-frame exterior six-corner evidence and real local regressions.

Retained images are optional local fixtures. These checks do not establish
surveyed XYZ, native LAB parity, whole-system timing or physical grasp rate.
"""
import importlib.util
import json
import math
import unittest
from types import SimpleNamespace
from unittest.mock import patch

from host.project_paths import ROOT

HAS_VISION=importlib.util.find_spec('cv2') is not None
if HAS_VISION:
    import cv2
    from device.k230.vision import cube_outline as outline,cube_corners,cube_observation
    from device.k230.vision.runtime import localize_precise
    from device.k230.vision.candidate_association import associate_fine_candidate

LOCAL=ROOT/'data/vision/near_supplement/20261104_005624'
HISTORIC=ROOT/'data/car/full_process_tracking_20261103/post_static_anchor_corrections_01'


class OutlineTopologyTests(unittest.TestCase):
    def test_six_external_branches_label_left_image_face_without_pose(self):
        from device.k230.vision import cube_outline as module
        polygon=[[20.,70.],[140.,20.],[250.,70.],[250.,260.],[140.,320.],[20.,260.]]
        expected=polygon
        for shift in range(6):
            for direction in (1,-1):
                points=polygon[shift:]+polygon[:shift]
                if direction<0:points.reverse()
                reduced,unused=module._reduce_hull(points)
                actual,evidence=module._label_hexagon(None,
                    SimpleNamespace(_vertical_slope=lambda *args:0.),reduced,[0,0],{},{},{})
                self.assertEqual(actual,expected)
                self.assertGreater(evidence['selected_score'],1.99)

    def test_unobservable_short_external_side_is_not_invented(self):
        from device.k230.vision import cube_outline as module
        polygon=[[20.,50.],[30.,45.],[250.,50.],[250.,260.],[30.,265.],[20.,260.]]
        with self.assertRaisesRegex(ValueError,'shorter than25'):
            module._label_hexagon(None,SimpleNamespace(_vertical_slope=lambda *args:0.),
                polygon,[0,0],{},{},{})


@unittest.skipUnless(HAS_VISION,'Dedicated local CV environment')
class OutlineRetainedCaptureTests(unittest.TestCase):
    def materials(self,number):
        paths=[LOCAL/'config.json',LOCAL/('frame_%02d.json'%number)]
        if any(not p.exists()for p in paths):self.skipTest('Unversioned real capture unavailable')
        cfg=json.loads(paths[0].read_text())['car'];meta=json.loads(paths[1].read_text())
        im=cv2.imread(str(LOCAL/meta['image_file']))
        if im is None:self.skipTest('Unversioned original JPEG unavailable')
        return cfg,meta,im

    def candidates(self,cfg,meta,im,prepared=None,lab=None):
        stages=list(outline.measure_steps(im,meta['bbox_px'],cfg['intrinsics'],
            cfg['extrinsics'],cfg['table_plane'],prepared=prepared,lab_thresholds=lab))
        return [e['measurement']for e in stages if e['stage']=='cube_candidate'],stages

    def build(self,cfg,measurement):
        return cube_observation.build_result(measurement,cfg['intrinsics']['camera_matrix'],
            cfg['intrinsics']['distortion_coefficients'],cfg['extrinsics']['T_Base_from_Camera_mm'],
            cfg['table_plane'])

    def assert_packet(self,measurement):
        self.assertEqual(measurement['evidence_type'],'nonplanar_cube_six_corners')
        self.assertEqual(measurement['corner_labels'],['A','E','F','G','C','D'])
        self.assertEqual(measurement['object_points_mm'],outline.OBJECT_POINTS)
        self.assertEqual(len(measurement['half_corner_errors_px']),12)
        self.assertLessEqual(max(measurement['half_corner_errors_px']),3)
        points=measurement['image_points_px'];edges=measurement['observed_edges']
        self.assertEqual(len(edges),6)
        from device.k230.vision import physical_side as p
        for i,edge in enumerate(edges):
            quality=edge['quality']
            self.assertEqual(quality['sampled_stations'],64)
            self.assertGreaterEqual(quality['support'],45)
            self.assertLessEqual(quality['rms_px'],3)
            self.assertFalse(quality['broad_HSV_outside_nonblue_verified'])
            actual=p.intersect(edges[i-1]['line_px'],edge['line_px'])
            self.assertLess(math.dist(actual,points[i]),1e-5)
            for half in (0,1):
                actual=p.intersect(edges[i-1]['half_lines_px'][half],edge['half_lines_px'][half])
                self.assertAlmostEqual(math.dist(actual,points[i]),
                    measurement['half_corner_errors_px'][half*6+i],places=5)

    def test_two_similar_faces_and_reflective_exterior_pass_typed_policy(self):
        for number in (1,4,6):
            with self.subTest(image=number):
                cfg,meta,im=self.materials(number);packets,stages=self.candidates(cfg,meta,im)
                self.assertEqual(len(packets),1,stages)
                self.assert_packet(packets[0]);raw=self.build(cfg,packets[0])
                self.assertEqual(raw['evidence_type'],'nonplanar_cube_six_corners')
                self.assertLess(raw['cube_pose']['corner_reprojection_rms_px'],1.8)
                self.assertEqual(raw['cube_pose']['corner_count'],6)
                self.assertLessEqual(raw['table_geometry']['object_up_deviation_deg'],20)
                self.assertEqual(packets[0]['outline_proposal']['LAB_thresholds'],
                    cfg['tracking']['lab_thresholds'])
                if number==6:
                    edge=packets[0]['observed_edges'][3]
                    self.assertEqual(edge['quality']['appearance_method'],'original_RGB8_gradient_peak')
                    self.assertEqual(edge['image_source']['backend'],
                        'original_RGBcontrast_sampler_after_exterior_proposal')

    def test_sixcorner_current_source_pool_matches_without_relaxing_association(self):
        for number in (1,6):
            cfg,meta,im=self.materials(number);packets,stages=self.candidates(cfg,meta,im)
            raw=self.build(cfg,packets[0])
            observation=localize_precise(None,meta['frame'],cfg['intrinsics'],cfg['extrinsics'],
                cfg['table_plane'],localizer=lambda *args:raw)
            self.assertEqual(observation['status'],'localized',observation)
            association=associate_fine_candidate(observation,meta['paired_coarse'])
            self.assertEqual(association['status'],'matched',association)
            self.assertEqual(observation['quality']['evidence_type'],'nonplanar_cube_six_corners')

    def test_shared_prepared_avoids_second_HSV_conversion_and_blur(self):
        cfg,meta,im=self.materials(1);prepared=cube_corners.prepare(im,meta['bbox_px'])
        convert,blur=cv2.cvtColor,cv2.GaussianBlur
        with patch.object(cv2,'cvtColor',wraps=convert)as conversions,patch.object(cv2,'GaussianBlur',wraps=blur)as blurs:
            packets,stages=self.candidates(cfg,meta,im,prepared)
        self.assertEqual(len(packets),1,stages)
        self.assertEqual([c.args[1]for c in conversions.call_args_list],[cv2.COLOR_BGR2LAB])
        blurs.assert_not_called()
        plain,unused=self.candidates(cfg,meta,im)
        self.assertEqual(packets[0]['image_points_px'],plain[0]['image_points_px'])

    def test_default_and_explicit_current_LAB_thresholds_give_same_pixels(self):
        cfg,meta,im=self.materials(6)
        default,unused=self.candidates(cfg,meta,im)
        explicit,unused=self.candidates(cfg,meta,im,lab=cfg['tracking']['lab_thresholds'])
        self.assertEqual(default[0]['image_points_px'],explicit[0]['image_points_px'])
        self.assertEqual(default[0]['outline_proposal']['LAB_threshold_source'],'default_blue_approach')
        self.assertEqual(explicit[0]['outline_proposal']['LAB_threshold_source'],'explicit')
        self.assertEqual(default[0]['outline_proposal']['body_mask_kind'],'configured_LAB')
        self.assertIsNone(default[0]['outline_proposal']['HSV_body_thresholds'])

    def test_dark_blue_body_uses_one_HSV_proposal_before_any_pose(self):
        folder=ROOT/'data/vision/hard_cases/cases/12f2f1ada5f00041-8b75ad95e6fa'
        if not(folder/'image.jpg').exists():self.skipTest('Unversioned dark-blue capture unavailable')
        cfg=json.loads((folder/'calibration.json').read_text())
        self.assertEqual(cfg['role'],'original')
        im=cv2.imread(str(folder/'image.jpg'));prepared=cube_corners.prepare(im,None)
        x,y,w,h=prepared['local_bbox'];cx,cy=int(round(x+.5*w)),int(round(y+.6*h))
        lab=cv2.cvtColor(prepared['raw'],cv2.COLOR_BGR2LAB)
        self.assertGreater(int(lab[cy,cx,2]),outline.DEFAULT_LAB_THRESHOLDS[0][5]+128)
        self.assertEqual(prepared['raw'][cy,cx].tolist(),[60,33,0])
        packets,stages=self.candidates(cfg,{'bbox_px':prepared['bbox_px']},im,prepared)
        self.assertEqual(len(packets),1,stages);self.assert_packet(packets[0])
        proposal=packets[0]['outline_proposal']
        self.assertEqual(proposal['body_mask_kind'],'existing_HSV_body_when_LAB_center_absent')
        self.assertEqual(proposal['HSV_body_thresholds'],{'lower':[90,120,62],'upper':[140,255,255]})
        for edge in packets[0]['observed_edges']:
            self.assertIn('HSV',edge['quality']['edge_ownership'])
            self.assertEqual(edge['image_source']['LAB_mapping_role'],'initial_proposal_probe_only')
        raw=self.build(cfg,packets[0])
        self.assertLess(raw['cube_pose']['corner_reprojection_rms_px'],.83)
        self.assertLessEqual(max(packets[0]['half_corner_errors_px']),.38)
        # The archived image has no native source pool. Only original paired
        # calibration geometry is checked; no synthetic strict-match is made.

    def test_LAB_success_keeps_all_retained_positive_measurements_exact(self):
        baseline=LOCAL/'hexagon_review/production_results.json'
        if not baseline.exists():self.skipTest('Unversioned pre-fallback audit unavailable')
        rows=json.loads(baseline.read_text())['rows'];checked=0
        for row in rows:
            if row['status']!='geometry_pass':continue
            identity=row['identity']
            if identity.startswith('new12/'):
                cfg,meta,im=self.materials(int(identity.rsplit('_',1)[1]))
            elif identity.startswith(('live_static_03/','exposure_controlled/')):
                folder,stem=identity.split('/')
                root=HISTORIC/folder
                cfg=json.loads((root/'config.json').read_text())['car']
                meta=json.loads((root/(stem+'.json')).read_text());im=cv2.imread(str(root/meta['image_file']))
            else:continue
            packets,stages=self.candidates(cfg,meta,im)
            self.assertEqual(len(packets),1,stages)
            self.assertEqual(packets[0]['image_points_px'],row['measurement']['image_points_px'])
            self.assertEqual(packets[0]['outline_proposal']['body_mask_kind'],'configured_LAB')
            checked+=1
        self.assertEqual(checked,14)

    def test_fixed_rule_retains_the_six_observable_new_views(self):
        passes=[]
        for number in range(1,13):
            cfg,meta,im=self.materials(number);packets,unused=self.candidates(cfg,meta,im)
            if packets:
                self.build(cfg,packets[0]);passes.append(number)
        self.assertEqual(passes,[1,2,4,5,6,9])

    def test_real_opposite_yaw_uses_original_calibration_and_same_order(self):
        cfg,unused,unused=self.materials(1)
        for name,bbox in [('539685b43bb40b31-8b75ad95e6fa',[400,519,204,287]),
                          ('d28680f4805279e5-8b75ad95e6fa',[555,480,194,244])]:
            folder=ROOT/'data/vision/hard_cases/cases'/name
            if not(folder/'image.jpg').exists():self.skipTest('Unversioned opposite-yaw capture unavailable')
            cal=json.loads((folder/'calibration.json').read_text())
            self.assertEqual(cal['role'],'original')
            actual=dict(cfg);actual.update({k:cal[k]for k in ('intrinsics','extrinsics','table_plane')})
            packets,stages=self.candidates(actual,{'bbox_px':bbox},cv2.imread(str(folder/'image.jpg')))
            self.assertEqual(len(packets),1,stages);self.assert_packet(packets[0])
            raw=self.build(actual,packets[0]);self.assertLess(raw['cube_pose']['corner_reprojection_rms_px'],1.6)

    def test_explicit_real_and_synthetic_partial_targets_do_not_emit_sixcorners(self):
        cfg,unused,unused=self.materials(1)
        cases=[]
        for frame,bbox in [(30,[362,270,102,126]),(510,[424,126,107,134]),(900,[494,82,102,108])]:
            cases.append((LOCAL/'hexagon_review'/('real_hand_frame_%04d.jpg'%frame),bbox,cfg))
        folder=HISTORIC/'exposure_controlled'
        if not(folder/'config.json').exists():self.skipTest('Unversioned negative source metadata unavailable')
        old=json.loads((folder/'config.json').read_text())['car']
        meta=json.loads((folder/'manual_gain2_03.json').read_text())
        for name in ('SYNTHETIC_top_40percent','SYNTHETIC_left_side','SYNTHETIC_right_side'):
            cases.append((HISTORIC/'live_static_03/diagnostics'/(name+'.png'),meta['bbox_px'],old))
        tape=ROOT/'data/car/full_process_tracking_20261103/overshoot_repair_repeat_01'
        if not(tape/'evidence/config.json').exists():self.skipTest('Unversioned tape source unavailable')
        tape_cfg=json.loads((tape/'evidence/config.json').read_text())['car']
        cases.append((tape/'evidence/tracking_5_moving.jpg',[285,669,328,92],tape_cfg))
        for image,bbox,actual in cases:
            if not image.exists():self.skipTest('Unversioned negative source unavailable: '+str(image))
            with self.subTest(image=image.name):
                packets,stages=self.candidates(actual,{'bbox_px':bbox},cv2.imread(str(image)))
                self.assertEqual(packets,[],stages)
