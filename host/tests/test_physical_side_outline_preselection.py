"""Existing64-profile right-outline preselection, without extra pose budget.

Real local archives are optional regression fixtures, not surveyed XYZ/grasp
truth. Synthetic left-side occlusion is a known existing original-face accept;
this change must neither relabel it nor introduce a different measurement.
"""
import importlib.util
import inspect
import json
import unittest
from unittest.mock import patch

from host.project_paths import ROOT
HAS_VISION=importlib.util.find_spec('cv2')is not None
if HAS_VISION:
    import cv2
    from device.k230.vision import physical_side as p

LOCAL=ROOT/'data/vision/acceptance_20261104'


@unittest.skipUnless(HAS_VISION,'Dedicated local CV environment')
class PhysicalSideOutlinePreselectionTests(unittest.TestCase):
    def materials(self,identity):
        path=LOCAL/'inventory.json'
        if not path.exists():self.skipTest('Unversioned original-image inventory unavailable')
        item=next(v for v in json.loads(path.read_text())['images']if v['id']==identity)
        case=next(v for v in item['cases']if v['calibration_role']=='original')
        image=cv2.imread(item['image_path'])
        if image is None:self.skipTest('Unversioned original JPEG unavailable')
        config=json.loads((ROOT/case['calibration_json']).read_text())
        self.assertEqual(config['role'],'original')
        return image,case['frame'],config

    def run_original(self,identity,preselect=False,include_range=False):
        image,frame,cfg=self.materials(identity)
        events=list(p._original_localize_steps(image,frame,None,cfg['intrinsics'],
            cfg['extrinsics'],cfg['table_plane'],max_candidates=1,
            include_range=include_range,preselect_outline=preselect))
        return events[-1]['observation'],events

    def test_H102_reuses_same_four_edges_and_solves_once_after_preselection(self):
        counts=[];order=[];original_pose=p.pose_candidates;original_edge=p._edge_steps
        def poses(*args,**kwargs):
            order.append('pose');return original_pose(*args,**kwargs)
        def edges(*args,**kwargs):
            counts.append((list(args[1]),list(args[2])))
            yield from original_edge(*args,**kwargs)
        with patch.object(p,'_edge_steps',edges):
            baseline,unused=self.run_original('H102')
        baseline_count=len(counts);counts.clear()
        image,frame,cfg=self.materials('H102');events=[]
        with patch.object(p,'pose_candidates',poses)as calls,patch.object(p,'_edge_steps',edges):
            for event in p._original_localize_steps(image,frame,None,cfg['intrinsics'],
                    cfg['extrinsics'],cfg['table_plane'],max_candidates=1,preselect_outline=True):
                events.append(event)
                if event['stage']=='outline_profile_preselected':order.append('preselect')
        self.assertEqual(baseline['status'],'no_complete_face')
        observation=events[-1]['observation'];self.assertEqual(observation['status'],'localized',observation)
        self.assertEqual(order,['preselect','pose'])
        self.assertEqual(len(counts),baseline_count)
        evidence=next(e for e in events if e['stage']=='outline_profile_preselected')
        self.assertEqual(evidence['original_quality']['support'],46)
        self.assertEqual(evidence['quality']['support'],50)
        self.assertLess(evidence['quality']['rms_px'],evidence['original_quality']['rms_px'])
        self.assertAlmostEqual(observation['quality']['reprojection_rms_px'],1.4661706763534728,places=6)
        self.assertEqual(observation['search']['candidate_limit'],1)
        self.assertEqual(observation['search']['measured_seed_count'],1)

    def test_lower_RMS_with_lost_support_keeps_existing_successful_pixels(self):
        for identity in ('H060','H021','H072','H092'):
            with self.subTest(identity=identity):
                original,unused=self.run_original(identity,False)
                current,events=self.run_original(identity,True)
                self.assertEqual(original['status'],'localized')
                self.assertEqual(current['payload'],original['payload'])
                self.assertEqual(current['cube_center_Base_mm'],original['cube_center_Base_mm'])
                self.assertFalse(any(e['stage']=='outline_profile_preselected'for e in events))

    def test_absent_coherent_candidate_keeps_original_first_line(self):
        baseline,unused=self.run_original('H060',False)
        original=p._coherent_internal_edge;attempts=[]
        def unavailable(*args,**kwargs):
            if kwargs.get('outline'):
                attempts.append(True)
                raise ValueError('Test: no alternative current outline')
            return original(*args,**kwargs)
        with patch.object(p,'_coherent_internal_edge',unavailable):
            current,events=self.run_original('H060',True)
        self.assertEqual(len(attempts),1)
        self.assertEqual(current['payload'],baseline['payload'])
        self.assertFalse(any(e['stage']=='outline_profile_preselected'for e in events))

    def test_post_pose_repair_reuses_the_precomputed_profile_candidate(self):
        original=p._coherent_internal_edge;outline_calls=[]
        def coherent(*args,**kwargs):
            if kwargs.get('outline'):outline_calls.append(True)
            return original(*args,**kwargs)
        with patch.object(p,'_coherent_internal_edge',coherent):
            observation,events=self.run_original('H042',True)
        self.assertEqual(len(outline_calls),1)
        self.assertEqual(observation['status'],'no_complete_face')
        self.assertTrue(any(e['stage']=='outline_profile_recovered'for e in events))
        self.assertTrue(any(e.get('reason')=='Physical-side pose attempt budget exhausted'for e in events))

    def test_only_static_single_face_fallback_enables_preselection(self):
        image,frame,cfg=self.materials('H102');original=p._original_localize_steps
        for moving in (False,True):
            with self.subTest(include_range=moving),patch.object(p,'_original_localize_steps',wraps=original)as calls:
                events=list(p.localize_steps(image,frame,None,cfg['intrinsics'],cfg['extrinsics'],
                    cfg['table_plane'],include_range=moving))
                self.assertEqual(calls.call_count,1)
                bound=inspect.signature(original).bind(*calls.call_args.args,**calls.call_args.kwargs)
                bound.apply_defaults()
                self.assertEqual(bound.arguments['include_range'],moving)
                self.assertEqual(bound.arguments['preselect_outline'],not moving)
                if moving:
                    before,unused=self.run_original('H102',False,True)
                    self.assertEqual(events[-1]['observation'].get('payload'),before.get('payload'))
                else:self.assertEqual(events[-1]['observation']['status'],'localized')
        self.assertIs(inspect.signature(original).parameters['preselect_outline'].default,False)

    def test_current77_and_legacy_successes_only_add_H102_without_pixel_changes(self):
        saved=LOCAL/'residual_before_six/two_residuals_pareto.json'
        if not saved.exists():self.skipTest('Unversioned frozen2D experiment unavailable')
        rows=json.loads(saved.read_text())['rows']
        original=p._original_localize_steps
        def without_preselection(*args,**kwargs):
            kwargs['preselect_outline']=False
            return original(*args,**kwargs)
        for row in rows:
            image,frame,cfg=self.materials(row['id'])
            for kind in ('original1','public_static'):
                with self.subTest(identity=row['id'],kind=kind):
                    baseline=row['runs'][kind+'_baseline']['observation']
                    if kind=='original1':generator=p._original_localize_steps(image,frame,row['bbox_px'],
                        cfg['intrinsics'],cfg['extrinsics'],cfg['table_plane'],max_candidates=1,preselect_outline=True)
                    else:
                        # Isolate this image-only preselection from later
                        # front/back geometry improvements in other routes.
                        with patch.object(p,'_original_localize_steps',side_effect=without_preselection):
                            baseline=list(p.localize_steps(image,frame,row['bbox_px'],
                                cfg['intrinsics'],cfg['extrinsics'],cfg['table_plane']))[-1]['observation']
                        generator=p.localize_steps(image,frame,row['bbox_px'],cfg['intrinsics'],cfg['extrinsics'],cfg['table_plane'])
                    events=list(generator);observation=events[-1]['observation']
                    expected='localized'if row['id']=='H102'else baseline['status']
                    self.assertEqual(observation['status'],expected,observation)
                    if baseline['status']=='localized':
                        self.assertEqual(observation['payload'],baseline['payload'])
                        self.assertEqual(observation['cube_center_Base_mm'],baseline['cube_center_Base_mm'])
        self.assertEqual(len(rows),77)

    def test_explicit_negative_regressions_preserve_existing_geometry_and_risk(self):
        saved=LOCAL/'residual_before_six/two_residuals_pareto_regression.json'
        if not saved.exists():self.skipTest('Unversioned negative registry unavailable')
        capture=ROOT/'data/vision/near_supplement/20261104_005624'
        historic=ROOT/'data/car/full_process_tracking_20261103/post_static_anchor_corrections_01'
        template=json.loads((historic/'live_static_03/config.json').read_text())['car'];tested=0
        original=p._original_localize_steps
        def without_preselection(*args,**kwargs):
            kwargs['preselect_outline']=False
            return original(*args,**kwargs)
        for row in json.loads(saved.read_text())['rows']:
            if row['scope']not in('real_partial_hand','synthetic_partial','real_noncube'):continue
            identity=row['identity']
            if identity.startswith('real_hand/'):
                number=int(identity.split('/')[1]);image=capture/'hexagon_review'/('real_hand_frame_%04d.jpg'%number)
                bbox={30:[362,270,102,126],510:[424,126,107,134],900:[494,82,102,108]}[number];cfg=template
            elif identity.startswith('negative/SYNTHETIC'):
                image=historic/'live_static_03/diagnostics'/(identity.split('/')[1]+'.png')
                meta=json.loads((historic/'exposure_controlled/manual_gain2_03.json').read_text());bbox=meta['bbox_px']
                cfg=json.loads((historic/'exposure_controlled/config.json').read_text())['car']
            else:
                folder=ROOT/'data/car/full_process_tracking_20261103/overshoot_repair_repeat_01';image=folder/'evidence/tracking_5_moving.jpg';bbox=[285,669,328,92]
                cfg=json.loads((folder/'evidence/config.json').read_text())['car']
            im=cv2.imread(str(image))
            if im is None:self.skipTest('Unversioned negative capture unavailable')
            frame={'frame_id':1,'capture_timestamp_ms':0,'clock_domain':'k230_ticks_ms',
                'image_size_px':[1280,960],'timestamp_source':'offline_placeholder_not_capture_time'}
            for kind in('original1','public_static'):
                if kind=='original1':generator=p._original_localize_steps(im,frame,bbox,cfg['intrinsics'],cfg['extrinsics'],cfg['table_plane'],max_candidates=1,preselect_outline=True)
                else:generator=p.localize_steps(im,frame,bbox,cfg['intrinsics'],cfg['extrinsics'],cfg['table_plane'])
                current=list(generator)[-1]['observation'];baseline=row['runs'][kind+'_baseline']['observation']
                if kind=='public_static':
                    with patch.object(p,'_original_localize_steps',side_effect=without_preselection):
                        baseline=list(p.localize_steps(im,frame,bbox,cfg['intrinsics'],cfg['extrinsics'],cfg['table_plane']))[-1]['observation']
                self.assertEqual(current['status'],baseline['status'])
                if baseline['status']=='localized':
                    self.assertTrue(identity.endswith('SYNTHETIC_left_side'))
                    self.assertEqual(current['payload'],baseline['payload'])
            tested+=1
        self.assertEqual(tested,7)
