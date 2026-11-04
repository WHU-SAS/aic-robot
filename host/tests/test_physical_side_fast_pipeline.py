"""Public static priority, shared-image work and moving-range boundaries.

Real retained captures are regression material, not physical accuracy truth.
The supplemental12 test requires every available frame to pass, including
currently difficult equal-face views; it never marks those as expected fails.
"""
import hashlib
import importlib.util
import json
import unittest
from types import SimpleNamespace
from unittest.mock import patch
from host.project_paths import ROOT

HAS_VISION=importlib.util.find_spec('cv2')is not None
if HAS_VISION:
    import cv2
    from device.k230.vision import physical_side as p,cube_corners,profile_recovery
    from device.k230.vision import cube_outline,cube_observation
    from device.k230.vision import ridge_profiles,separator_profiles,physical_side_session
    from device.k230.vision.candidate_association import associate_fine_candidate

LOCAL=ROOT/'data/car/full_process_tracking_20261103/post_static_anchor_corrections_01'
SUPPLEMENT=ROOT/'data/vision/near_supplement/20261104_005624'


@unittest.skipUnless(HAS_VISION,'Dedicated local CV environment')
class PublicStaticFastPipelineTests(unittest.TestCase):
    def saved(self,directory,stem):
        required=[directory/'config.json',directory/(stem+'.json')]
        if any(not path.is_file()for path in required):
            self.skipTest('Retained local capture material unavailable: '+str(directory))
        config=json.loads(required[0].read_text())['car']
        record=json.loads(required[1].read_text());path=directory/record['image_file']
        if not path.is_file():self.skipTest('Retained full-camera image unavailable: '+str(path))
        jpeg=path.read_bytes()
        if record.get('image_sha256'):
            self.assertEqual(hashlib.sha256(jpeg).hexdigest(),record['image_sha256'])
        image=cv2.imread(str(path));self.assertIsNotNone(image)
        self.assertEqual(image.shape,(960,1280,3))
        coarse=record.get('paired_coarse')or record.get('coarse')or record['frame'].get('paired_coarse')
        self.assertIsInstance(coarse,dict)
        self.assertEqual(coarse['frame_id'],record['frame']['frame_id'])
        self.assertEqual(coarse['capture_timestamp_ms'],record['frame']['capture_timestamp_ms'])
        box=record.get('bbox_px')or coarse['bbox_px']
        return config,record,image,coarse,box,jpeg

    def solve(self,saved,**options):
        config,record,image,coarse,box,jpeg=saved
        return list(p.localize_steps(image,record['frame'],box,config['intrinsics'],
            config['extrinsics'],config['table_plane'],**options))

    def assert_typed_current(self,observation,record,coarse):
        self.assertEqual(observation['status'],'localized',observation)
        self.assertEqual(observation['reference_point'],'cube_geometric_center')
        self.assertFalse(observation['motion_performed'])
        self.assertFalse(observation['xy_compensation_applied'])
        for key in('frame_id','capture_timestamp_ms','clock_domain','image_size_px','timestamp_source'):
            if key in record['frame']:self.assertEqual(observation[key],record['frame'][key],key)
        quality,payload=observation['quality'],observation['payload']
        evidence=quality.get('evidence_type')
        if evidence in('nonplanar_cube_five_corners','nonplanar_cube_six_corners',
                        'nonplanar_cube_face_back_top'):
            face_back=evidence=='nonplanar_cube_face_back_top'
            count=5 if evidence=='nonplanar_cube_five_corners'else 6
            self.assertEqual(quality['reprojection_limit_px'],3.)
            self.assertLessEqual(quality['reprojection_rms_px'],3.)
            self.assertEqual(payload['status'],'complete_cube_candidate')
            self.assertNotIn('poses',payload)
            self.assertEqual(len(payload['cube_pose']['image_points_px']),count)
            self.assertEqual(len(payload['cube_pose']['object_points_mm']),count)
            self.assertEqual(len(payload['observed_edges']),7 if face_back else 6)
            edges=[e['quality']for e in payload['observed_edges']]
            self.assertEqual(len(payload['half_corner_errors_px']),2*count)
            self.assertTrue(all(0<=v<=3 for v in payload['half_corner_errors_px']))
            geometry=payload['cube_pose']['table_geometry']
            if face_back:
                self.assertEqual(payload['boundary_method'],
                    'seven_measured_edges_front_and_back_top_corners')
                self.assertEqual(payload['vision_path'],'shared_nonplanar_cube_face_back_top')
                self.assertEqual(payload['corner_labels'],['A','B','C','D','E','F'])
                self.assertLessEqual(geometry['object_up_deviation_deg'],20.)
                if observation['profile_recovery']['top_method']=='ridge':
                    self.assertEqual(payload['observed_edges'][0]['name'],'front_top')
                    top=edges[0]
                    self.assertEqual(top['appearance_method'],'same_frame_G_ridge')
                    self.assertEqual(top['station_evidence'],'G_peak_two_flanks_DN')
                    self.assertIs(top['original_RGB8_step_evidence'],False)
            elif count==6:
                self.assertEqual(payload['boundary_method'],'six_measured_outline_edges_six_corners')
                self.assertEqual(payload['vision_path'],'shared_nonplanar_cube_outline')
                self.assertEqual(payload['corner_labels'],['A','E','F','G','C','D'])
                self.assertLessEqual(geometry['object_up_deviation_deg'],20.)
            else:
                self.assertEqual(payload['boundary_method'],'six_measured_edges_five_corners')
                self.assertEqual(payload['vision_path'],'shared_nonplanar_cube_corners')
        else:
            self.assertIn(evidence,(None,'physical_face_with_G_ridge'))
            self.assertLessEqual(quality['reprojection_rms_px'],2.)
            self.assertEqual(payload['status'],'complete_face_candidate')
            self.assertEqual(len(payload['physical_edges']),4)
            edges=payload['physical_edges'];geometry=payload['poses'][0]['table_geometry']
            if evidence=='physical_face_with_G_ridge':
                self.assertEqual(quality['reprojection_limit_px'],2.)
                self.assertEqual(quality['separator_evidence'],'G_ridge_bidirectional_prominence')
                self.assertEqual(payload['boundary_method'],'four_measured_edges_with_G_ridge')
                self.assertIs(payload['seam_evidence']['original_RGB8_step_evidence'],False)
                self.assertGreaterEqual(payload['seam_evidence']['prominence_min_DN'],2.)
        for edge in edges:
            self.assertEqual(edge['sampled_stations'],64)
            self.assertGreaterEqual(edge['support'],45)
            self.assertLessEqual(edge['rms_px'],3.)
        self.assertTrue(geometry['accepted'])
        self.assertLessEqual(geometry['normal_deviation_deg'],20.)
        self.assertLessEqual(abs(geometry['height_error_mm']),8.)
        association=associate_fine_candidate(observation,coarse)
        self.assertEqual(association['status'],'matched',association)
        self.assertEqual(association['frame_id'],record['frame']['frame_id'])
        self.assertEqual(association['capture_timestamp_ms'],record['frame']['capture_timestamp_ms'])

    def test_current_cube_B_and_G_succeed_before_any_original_full_prefix(self):
        cases=[('cube',LOCAL/'live_static_03','static_01'),
               ('B',LOCAL/'live_static_02','verify_11'),
               ('G',LOCAL/'live_static_06_final_timing','fresh_01')]
        for kind,directory,stem in cases:
            saved=self.saved(directory,stem)
            with self.subTest(kind=kind),patch.object(p,'_original_localize_steps',
                    side_effect=AssertionError('Fast success must not run original complete prefix')),\
                 patch.object(p,'_seam_seed_steps',side_effect=AssertionError('Old full seam scan before fast success')):
                events=self.solve(saved)
            self.assertEqual(sum(e['stage']=='complete'for e in events),1)
            self.assertEqual(events[0]['stage'],'recovery_selected')
            self.assertEqual(events[0]['route'],'cube'if kind=='cube'else'profile')
            self.assertNotIn('original_face_fallback',[e['stage']for e in events])
            observation=events[-1]['observation'];self.assert_typed_current(observation,saved[1],saved[3])
            if kind=='B':self.assertEqual(observation['profile_recovery']['top_method'],'B')
            if kind=='G':self.assertEqual(observation['profile_recovery']['top_method'],'ridge')

    def test_rejected_cube_and_accepted_G_share_one_ROI_and_one_cue(self):
        saved=self.saved(LOCAL/'live_static_06_final_timing','fresh_01')
        contexts=[];real_prepare=cube_corners.prepare
        def prepare(*args,**kwargs):
            context=real_prepare(*args,**kwargs)
            # Orchestration fixture only: real unchanged G pixels/quality.
            context['selector']=dict(context['selector'],route_hint='cube')
            contexts.append(context);return context
        def rejected(*args,**kwargs):
            self.assertIs(args[6],contexts[0])
            self.assertFalse(kwargs.get('outline',False))
            yield {'stage':'complete','observation':{'status':'no_complete_face',
                'reason':'Orchestration-only cube rejection','search':{}}}
        with patch.object(cube_corners,'prepare',side_effect=prepare)as prepared,\
             patch.object(p,'_cube_steps',side_effect=rejected)as cubes,\
             patch.object(profile_recovery,'recover_steps',wraps=profile_recovery.recover_steps)as profiles,\
             patch.object(separator_profiles,'cue24',wraps=separator_profiles.cue24)as cue,\
             patch.object(ridge_profiles,'measure',wraps=ridge_profiles.measure)as G,\
             patch.object(cv2,'cvtColor',wraps=cv2.cvtColor)as colors,\
             patch.object(cv2,'GaussianBlur',wraps=cv2.GaussianBlur)as smooth,\
             patch.object(p,'_original_localize_steps',side_effect=AssertionError('Accepted profile ran old full prefix')):
            events=self.solve(saved)
        self.assertEqual(prepared.call_count,1);self.assertEqual(cubes.call_count,1)
        self.assertEqual(profiles.call_count,1);self.assertIs(profiles.call_args.args[8],contexts[0])
        self.assertEqual(cue.call_count,1);self.assertEqual(G.call_count,1)
        self.assertCountEqual([call.args[1]for call in colors.call_args_list],
            [cv2.COLOR_BGR2HSV,cv2.COLOR_BGR2GRAY])
        self.assertEqual(smooth.call_count,1)
        self.assertEqual([e['route']for e in events if e['stage']=='recovery_selected'],['cube','profile'])
        self.assert_typed_current(events[-1]['observation'],saved[1],saved[3])

    def test_outline_success_reuses_prepared_after_profile_and_never_runs_original(self):
        for n in(1,6):
            saved=self.saved(SUPPLEMENT,'frame_%02d'%n);contexts=[]
            real_prepare=cube_corners.prepare
            def prepare(*args,**kwargs):
                result=real_prepare(*args,**kwargs);contexts.append(result);return result
            with self.subTest(frame=n),patch.object(cube_corners,'prepare',side_effect=prepare)as prepared,\
                 patch.object(profile_recovery,'recover_steps',wraps=profile_recovery.recover_steps)as profiles,\
                 patch.object(cube_outline,'measure_steps',wraps=cube_outline.measure_steps)as outlines,\
                 patch.object(cube_observation,'solve_cube',wraps=cube_observation.solve_cube)as solve,\
                 patch.object(p,'_original_localize_steps',side_effect=AssertionError('Outline success ran original full prefix')):
                events=self.solve(saved)
            self.assertEqual(prepared.call_count,1);self.assertEqual(profiles.call_count,1)
            self.assertEqual(outlines.call_count,1);self.assertEqual(solve.call_count,1)
            self.assertIs(profiles.call_args.args[8],contexts[0])
            self.assertIs(outlines.call_args.kwargs['prepared'],contexts[0])
            self.assertEqual([e['route']for e in events if e['stage']=='recovery_selected'],['profile','outline'])
            self.assertNotIn('original_face_fallback',[e['stage']for e in events])
            observation=events[-1]['observation']
            self.assertEqual(observation['quality']['evidence_type'],'nonplanar_cube_six_corners')
            self.assert_typed_current(observation,saved[1],saved[3])

    def test_include_range_delegates_original_work_and_preserves_distance_only_result(self):
        directory=LOCAL/'evidence';manifest=directory/'manifest.json';config_path=directory/'config.json'
        if not manifest.is_file()or not config_path.is_file():self.skipTest('Original moving material unavailable')
        cfg=json.loads(config_path.read_text())['car']
        entry=next(e for e in json.loads(manifest.read_text())['frames']if e['frame']['frame_id']==30)
        image=cv2.imread(str(directory/entry['image_file']));frame=entry['frame'];box=frame['localization_bbox_px']
        args=(image,frame,box,cfg['intrinsics'],cfg['extrinsics'],cfg['table_plane'])
        expected=list(p._original_localize_steps(*args,include_range=True))[-1]['observation']
        with patch.object(cube_corners,'prepare',side_effect=AssertionError('Static ROI preparation in moving range')),\
             patch.object(profile_recovery,'recover_steps',side_effect=AssertionError('Static profile in moving range')),\
             patch.object(p,'_cube_steps',side_effect=AssertionError('Static cube in moving range')),\
             patch.object(p,'_original_localize_steps',wraps=p._original_localize_steps)as original:
            events=list(p.localize_steps(*args,include_range=True))
        self.assertEqual(original.call_count,1);self.assertIs(original.call_args.args[-1],True)
        actual=events[-1]['observation']
        self.assertEqual(actual['status'],expected['status'])
        self.assertEqual(actual['range_candidate'],expected['range_candidate'])
        self.assertEqual(actual['range_candidate']['not_arm_or_parking_authority'],True)
        self.assertNotIn('cube_center_Base_mm',actual)
        self.assertNotIn('recovery_selected',[e['stage']for e in events])
        self.assertEqual(actual['frame_id'],frame['frame_id'])
        self.assertEqual(actual['capture_timestamp_ms'],frame['capture_timestamp_ms'])

    def test_real_FineSession_keeps_owned_JPEG_and_current_public_G_result(self):
        saved=self.saved(LOCAL/'live_static_06_final_timing','fresh_01');cfg,record,image,coarse,box,jpeg=saved
        now=record['frame']['capture_timestamp_ms']+10
        session=physical_side_session.FineSession(cfg,lambda:now)
        def compressed(quality):
            self.assertEqual(quality,95)
            return SimpleNamespace(bytearray=lambda:bytearray(jpeg))
        with patch.object(physical_side_session,'localize_steps',wraps=p.localize_steps)as solver:
            session.start(SimpleNamespace(compressed=compressed),record['frame'],box)
            for _ in range(10000):
                if session.phase is None:break
                session.advance()
        self.assertIsNone(session.phase)
        self.assertEqual(solver.call_count,1)
        self.assertEqual(solver.call_args.args[1],record['frame'])
        self.assertEqual(solver.call_args.args[2],box)
        self.assertFalse(solver.call_args.kwargs['include_range'])
        self.assert_typed_current(session.latest,record,coarse)
        self.assertIn(session.latest['quality']['evidence_type'],
            ('physical_face_with_G_ridge','nonplanar_cube_face_back_top'))
        self.assertEqual(session.latest['profile_recovery']['top_method'],'ridge')
        self.assertEqual(session.take_evidence('static')['jpeg'],jpeg)

    def test_new12_every_current_frame_localizes_and_strictly_matches(self):
        if not SUPPLEMENT.is_dir():self.skipTest('Supplemental capture pack is local-only')
        required=[SUPPLEMENT/'config.json']+[SUPPLEMENT/('frame_%02d.%s'%(n,suffix))
            for n in range(1,13)for suffix in('json','jpg')]
        for path in required:self.assertTrue(path.is_file(),'Incomplete supplemental pack: '+str(path))
        checked=0
        for n in range(1,13):
            saved=self.saved(SUPPLEMENT,'frame_%02d'%n);checked+=1
            with self.subTest(frame=n):
                events=self.solve(saved)
                self.assertEqual(sum(e['stage']=='complete'for e in events),1)
                observation=events[-1]['observation']
                self.assert_typed_current(observation,saved[1],saved[3])
                if n in(3,4):
                    self.assertEqual(observation['quality']['evidence_type'],
                        'nonplanar_cube_face_back_top')
                    self.assertEqual(len(observation['payload']['observed_edges']),7)
                    self.assertEqual(len(observation['payload']['half_corner_errors_px']),12)
                if n==4:
                    self.assertNotIn('outline',[e['route']for e in events if e['stage']=='recovery_selected'])
                    self.assertEqual(observation['profile_recovery']['face_proposal']['side'],'right')
                    self.assertGreater(observation['profile_recovery']['face_proposal']['terminal_gap_px'],12)
                    details=observation['profile_recovery']['separator_profile']
                    self.assertIn('terminal_support',details)
                    self.assertNotIn('station_rows',details)
                    self.assertNotIn('measured_points',details)
        self.assertEqual(checked,12)


if __name__=='__main__':unittest.main()
