"""Observed-outline proposals share one image scan and retain strict gates."""
import importlib.util
import json
import unittest
from unittest.mock import patch
from host.project_paths import ROOT

HAS_VISION = importlib.util.find_spec('cv2') is not None
if HAS_VISION:
    import cv2
    import numpy as np
    from device.k230.vision import physical_side as vision
    from host.tests import test_car_near_vision as fixtures


@unittest.skipUnless(HAS_VISION, 'Dedicated vision environment')
class PhysicalSideSeedRecoveryTests(unittest.TestCase):
    def frame(self):
        return {'frame_id': 'saved_image', 'capture_timestamp_ms': None,
                'timestamp_source': 'unavailable', 'clock_domain': None,
                'image_size_px': [1280, 960]}

    def local_failure(self, name):
        run = ROOT/'data/logs'/name
        path = run/'board_evidence/pick.jpg'
        if not path.exists():
            self.skipTest('Original difficult image is local-only evidence')
        cfg = json.loads((run/'vision_context.json').read_text())
        return cv2.imread(str(path)), cfg

    def parking_failure(self, task):
        if task == 'robot_task_109450_0':
            folder = ROOT/'data/vision/hard_cases/cases/db85d27062cdb868-8b75ad95e6fa'
            if not (folder/'image.jpg').exists():
                self.skipTest('Original parking image is local-only evidence')
            return cv2.imread(str(folder/'image.jpg')), json.loads((folder/'calibration.json').read_text())
        folder = ROOT/'data/logs/robot_handoff_diagnosis_20261031_160409/evidence'/task
        path = folder/'stopped_verify.jpg'
        if not path.exists():
            self.skipTest('Original parking image is local-only evidence')
        return cv2.imread(str(path)), json.loads((folder/'config.json').read_text())['car']

    def solve(self, image, cfg, **options):
        # These are proposal/scan-reuse unit checks of the original algorithm.
        # The public moving-range delegation remains covered explicitly.
        solver=vision.localize_steps if options.get('include_range')else vision._original_localize_steps
        return list(solver(image, self.frame(), None,
            cfg['intrinsics'], cfg['extrinsics'], cfg['table_plane'], **options))

    def test_bottom_extension_requires_outside_radius_and_observed_long_support(self):
        seed = [[10., 10.], [80., 10.], [80., 80.], [10., 80.]]
        boundary = [[10., 10.], [80., 10.], [80., 90.], [10., 90.]]
        result = vision._contour_side_seed(seed, 2, boundary, boundary)
        self.assertIsNotNone(result)
        self.assertAlmostEqual(result[2][1], 90.)
        self.assertAlmostEqual(result[3][1], 90.)
        self.assertEqual(result[:2], seed[:2])
        # A slightly different threshold silhouette is not evidence the seed
        # missed a physical edge outside its original five-pixel search.
        close = [[10., 10.], [80., 10.], [80., 84.], [10., 84.]]
        self.assertIsNone(vision._contour_side_seed(seed, 2, close, close))
        # A convex hull spanning a gap cannot substitute for real contour.
        unrelated = [[10., 10.], [80., 10.], [80., 80.], [10., 80.]]
        self.assertIsNone(vision._contour_side_seed(seed, 2, boundary, unrelated))
        self.assertIsNone(vision._contour_side_seed(seed, 2, boundary))

    def test_outline_support_does_not_count_overlapping_fragments_twice(self):
        contour = [[10., 90.], [30., 90.], [10., 90.], [30., 90.], [10., 90.]]
        self.assertFalse(vision._outline_support([10., 90.], [80., 90.], contour))

    def test_dark_rounded_side_and_lit_steep_side_recover_without_a_second_scan(self):
        cases = ('pick_pc_20261031_134518_334178', 'pick_pc_20261031_134610_817761')
        for name in cases:
            image, cfg = self.local_failure(name)
            predictions = []
            for native in (True, False):
                with self.subTest(name=name, native=native), patch.object(vision, '_BATCH_EDGES', native), patch.object(
                        vision, '_native_contrasts', wraps=vision._native_contrasts) as contrasts:
                    steps = self.solve(image, cfg)
                out = steps[-1]['observation']
                self.assertEqual(out['status'], 'localized', out)
                self.assertLessEqual(out['quality']['reprojection_rms_px'], 2.)
                self.assertLessEqual(out['search']['measured_seed_count'], 3)
                self.assertEqual(sum(step['stage']=='seam_samples' for step in steps), 8)
                # A seam grid has24 rows; four physical edges have64 stations.
                grids = [call.args[1].shape for call in contrasts.call_args_list if call.args[1].shape[0]==24]
                self.assertEqual(len(grids), 1 if native else 0)
                face = out['payload']
                self.assertTrue(face['poses'][0]['table_geometry']['accepted'])
                self.assertTrue(all(edge['sampled_stations']==64 and edge['support']>=45
                                    for edge in face['physical_edges']))
                self.assertIsNone(out['capture_timestamp_ms'])
                predictions.append(out['cube_center_Base_mm'])
            for a, b in zip(*predictions):
                self.assertAlmostEqual(a, b, delta=.05)

    def test_original_global_seam_and_late_successful_seed_keep_their_predictions(self):
        # Historical predictions guard the proposal ordering. They are not
        # surveyed millimetre accuracy or human ground-truth coordinates.
        cases = {
            '0996bc70045d174e-24f37294c058': [248.01513795870346, 19.90805114640333, -51.79822424280818],
            '13433cccdc192952-31947523a89b': [220.4151079420831, 79.27522393278286, -54.494654226573424],
        }
        for case_id, expected in cases.items():
            folder = ROOT/'data/vision/hard_cases/cases'/case_id
            if not (folder/'image.jpg').exists():
                self.skipTest('Original difficult image is local-only evidence')
            cfg = json.loads((folder/'calibration.json').read_text())
            with self.subTest(case=case_id), patch.object(vision, '_BATCH_EDGES', True):
                out = self.solve(cv2.imread(str(folder/'image.jpg')), cfg)[-1]['observation']
                self.assertEqual(out['status'], 'localized', out)
                for actual, baseline in zip(out['cube_center_Base_mm'], expected):
                    self.assertAlmostEqual(actual, baseline, places=10)

    def test_parking_separator_recovery_reuses_scan_and_retains_full_quality_gates(self):
        for task in ('robot_task_49282_0', 'robot_task_345255_0', 'robot_task_109450_0'):
            image, cfg = self.parking_failure(task)
            predictions = []
            for native in (True, False):
                with self.subTest(task=task, native=native), patch.object(
                        vision, '_BATCH_EDGES', native), patch.object(
                        vision, '_native_contrasts', wraps=vision._native_contrasts) as contrasts, patch.object(
                        vision, 'pose_candidates', wraps=vision.pose_candidates) as poses:
                    steps = self.solve(image, cfg)
                out = steps[-1]['observation']
                self.assertEqual(out['status'], 'localized', out)
                self.assertLessEqual(out['quality']['reprojection_rms_px'], 2.)
                self.assertLessEqual(out['search']['measured_seed_count'], 3)
                self.assertLessEqual(poses.call_count, 3)
                self.assertFalse(out['search']['top_fallback'])
                self.assertEqual(sum(step['stage']=='seam_samples' for step in steps), 8)
                grids = [call.args[1].shape for call in contrasts.call_args_list
                         if call.args[1].shape[0]==24]
                self.assertEqual(len(grids), 1 if native else 0)
                for face in out['payload']['side_faces']:
                    geometry = face['poses'][0]['table_geometry']
                    self.assertTrue(geometry['accepted'])
                    self.assertLessEqual(geometry['normal_deviation_deg'], 20.)
                    self.assertLessEqual(abs(geometry['height_error_mm']), 8.)
                    for edge in face['physical_edges']:
                        self.assertEqual(edge['sampled_stations'], 64)
                        self.assertGreaterEqual(edge['support'], 45)
                        self.assertLessEqual(edge['rms_px'], 3.)
                self.assertIsNone(out['capture_timestamp_ms'])
                predictions.append(out['cube_center_Base_mm'])
            np.testing.assert_allclose(predictions[0], predictions[1], atol=.05, rtol=0.)

    def test_original_colour_proposals_keep_exact_face_and_center_after_recovery(self):
        # These are saved algorithm predictions, not measured ground truth.
        # In particular, a new priority seed must not replace an already valid
        # colour proposal merely because the old seam scan found no seed.
        report = ROOT/'data/vision/hard_cases/reports/before_parking_seed_20261031_original_1650'
        for case_id in ('3bd2154b2994291c-31947523a89b', 'a120997818745353-31947523a89b'):
            folder = ROOT/'data/vision/hard_cases/cases'/case_id
            baseline = report/(case_id+'.json')
            if not baseline.exists() or not (folder/'image.jpg').exists():
                self.skipTest('Original before report and image are local-only evidence')
            old = json.loads(baseline.read_text())['observation']
            cfg = json.loads((folder/'calibration.json').read_text())
            with self.subTest(case=case_id), patch.object(vision, '_BATCH_EDGES', True):
                out = self.solve(cv2.imread(str(folder/'image.jpg')), cfg)[-1]['observation']
            self.assertEqual(out['status'], 'localized', out)
            self.assertEqual(out['cube_center_Base_mm'], old['cube_center_Base_mm'])
            self.assertEqual(out['payload']['face_quad_px'], old['payload']['face_quad_px'])

    def test_static_side_shadow_reuses_profiles_after_failed_pose(self):
        folder = ROOT/'data/logs/robot_startup_delay_20261031_181627/evidence/robot_task_329851_0'
        if not (folder/'stopped_verify.jpg').exists():
            self.skipTest('Original parking images are local-only evidence')
        cfg = json.loads((folder/'config.json').read_text())['car']
        original = vision._coherent_internal_edge
        for label in ('stopped_verify', 'corrected_verify'):
            image = cv2.imread(str(folder/(label+'.jpg')))
            predictions = []
            for native in (True, False):
                recovered = []
                with self.subTest(label=label, native=native), patch.object(
                        vision, '_BATCH_EDGES', native), patch.object(
                        vision, '_native_contrasts', wraps=vision._native_contrasts) as contrasts, patch.object(
                        vision, 'pose_candidates', wraps=vision.pose_candidates) as poses:
                    def same_profiles(*args, **kwargs):
                        before = contrasts.call_count
                        result = original(*args, **kwargs)
                        if kwargs.get('outline'):
                            self.assertEqual(contrasts.call_count, before)
                            recovered.append(result)
                        return result
                    with patch.object(vision, '_coherent_internal_edge', side_effect=same_profiles):
                        steps = self.solve(image, cfg)
                out = steps[-1]['observation']
                self.assertEqual(out['status'], 'localized', out)
                self.assertEqual(poses.call_count, 2)
                self.assertLessEqual(out['search']['measured_seed_count'], 3)
                self.assertEqual(len(recovered), 1)
                self.assertTrue(any(step['stage']=='outline_profile_recovered' for step in steps))
                self.assertEqual(sum(step['stage']=='seam_samples' for step in steps), 8)
                self.assertLessEqual(out['quality']['reprojection_rms_px'], 2.)
                face = out['payload']
                self.assertEqual(face['side_selection'], 'single_verified_side')
                self.assertTrue(face['side_faces'])
                self.assertEqual(face['physical_edges'][1]['edge_ownership'], 'coherent_blue_outline')
                self.assertTrue(face['poses'][0]['table_geometry']['accepted'])
                for edge in face['physical_edges']:
                    self.assertEqual(edge['sampled_stations'], 64)
                    self.assertGreaterEqual(edge['support'], 45)
                    self.assertLessEqual(edge['rms_px'], 3.)
                predictions.append(out['cube_center_Base_mm'])
            np.testing.assert_allclose(predictions[0], predictions[1], atol=.05, rtol=0.)

    def test_static_failed_outline_fit_recovers_same_profiles_on_recent_true_cube_frames(self):
        base = ROOT/'data/car/full_process_tracking_20261102'
        calibration = base/'static_timing_04/candidate/config.json'
        if not calibration.exists():
            self.skipTest('Candidate calibration and source JPEGs are local-only evidence')
        cfg = json.loads(calibration.read_text())['car']
        original_cfg = json.dumps(cfg, sort_keys=True)
        coherent = vision._coherent_internal_edge
        for group in ('static_timing_03', 'static_timing_04/live'):
            folder = base/group
            if not (folder/'static.jpg').exists() or not (folder/'static.json').exists():
                self.skipTest('Original recent static frame is local-only evidence')
            image = cv2.imread(str(folder/'static.jpg'))
            frame = json.loads((folder/'static.json').read_text())['frame']
            predictions = []
            for native in (True, False):
                recovered = []
                with self.subTest(group=group, native=native), patch.object(
                        vision, '_BATCH_EDGES', native), patch.object(
                        vision, '_native_contrasts', wraps=vision._native_contrasts) as contrasts, patch.object(
                        vision, 'pose_candidates', wraps=vision.pose_candidates) as poses:
                    def no_rgb_rescan(*args, **kwargs):
                        before = contrasts.call_count
                        result = coherent(*args, **kwargs)
                        self.assertEqual(contrasts.call_count, before)
                        if kwargs.get('outline'):
                            recovered.append(result)
                        return result
                    with patch.object(vision, '_coherent_internal_edge', side_effect=no_rgb_rescan):
                        steps = list(vision._original_localize_steps(image, frame, None,
                            cfg['intrinsics'], cfg['extrinsics'], cfg['table_plane']))
                out = steps[-1]['observation']
                self.assertEqual(out['status'], 'localized', out)
                self.assertTrue(recovered)
                self.assertLessEqual(out['quality']['reprojection_rms_px'], 2.)
                self.assertLessEqual(out['search']['measured_seed_count'], 3)
                self.assertLessEqual(poses.call_count, 3)
                self.assertFalse(out['search']['top_fallback'])
                self.assertEqual(out['frame_id'], frame['frame_id'])
                self.assertEqual(out['capture_timestamp_ms'], frame['capture_timestamp_ms'])
                for face in out['payload']['side_faces']:
                    geometry = face['poses'][0]['table_geometry']
                    self.assertTrue(geometry['accepted'])
                    self.assertLessEqual(geometry['normal_deviation_deg'], 20.)
                    self.assertLessEqual(abs(geometry['height_error_mm']), 8.)
                    for edge in face['physical_edges']:
                        self.assertEqual(edge['sampled_stations'], 64)
                        self.assertGreaterEqual(edge['support'], 45)
                        self.assertLessEqual(edge['rms_px'], 3.)
                predictions.append(out['cube_center_Base_mm'])
            np.testing.assert_allclose(predictions[0], predictions[1], atol=.05, rtol=0.)
        self.assertEqual(json.dumps(cfg, sort_keys=True), original_cfg)

    def test_after_restart_current_capture_is_a_separate_new_frame_not_failed_run_evidence(self):
        base = ROOT/'data/car/full_process_tracking_20261102'
        folder = base/'first_field_failure'
        calibration = base/'static_timing_04/candidate/config.json'
        if any(not path.exists() for path in
                (folder/'current_static.jpg', folder/'current_static.jpg.json', calibration)):
            self.skipTest('After-restart current capture is local-only evidence')
        metadata = json.loads((folder/'current_static.jpg.json').read_text())
        self.assertEqual(metadata['scope'], 'new_static_capture_after_user_restart_not_failed_run_frame')
        cfg = json.loads(calibration.read_text())['car']
        image = cv2.imread(str(folder/'current_static.jpg'))
        predictions = []
        for native in (True, False):
            with self.subTest(native=native), patch.object(vision, '_BATCH_EDGES', native):
                steps = list(vision._original_localize_steps(image, metadata['frame'], None,
                    cfg['intrinsics'], cfg['extrinsics'], cfg['table_plane']))
            out = steps[-1]['observation']
            self.assertEqual(out['status'], 'localized', out)
            self.assertEqual(out['frame_id'], metadata['frame']['frame_id'])
            self.assertEqual(out['capture_timestamp_ms'], metadata['frame']['capture_timestamp_ms'])
            self.assertLessEqual(out['quality']['reprojection_rms_px'], 2.)
            self.assertLessEqual(out['search']['measured_seed_count'], 3)
            self.assertFalse(out['search']['top_fallback'])
            for face in out['payload']['side_faces']:
                self.assertTrue(face['poses'][0]['table_geometry']['accepted'])
                for edge in face['physical_edges']:
                    self.assertEqual(edge['sampled_stations'], 64)
                    self.assertGreaterEqual(edge['support'], 45)
                    self.assertLessEqual(edge['rms_px'], 3.)
            predictions.append(out['cube_center_Base_mm'])
        np.testing.assert_allclose(predictions[0], predictions[1], atol=.05, rtol=0.)

    def test_failed_double_peak_outline_cannot_combine_two_under_supported_lines(self):
        image = np.full((260, 240, 3), (200, 60, 0), dtype=np.uint8)
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        radius = 12  # 200px edge keeps the original 6 percent bounded search.
        rows = []
        for station in range(64):
            row = [0.]*(2*radius+1)
            row[radius+(-8 if station % 2 else 8)] = 1.
            row[radius+(8 if station % 2 else -8)] = .001
            rows.append(row)
        allowed = [[True]*(2*radius+1) for _ in range(64)]
        with patch.object(vision, '_BATCH_EDGES', True), patch.object(
                vision, '_contrast_grid', return_value=rows) as grids, patch.object(
                vision, '_blue_grid', return_value=allowed), patch.object(
                vision, '_coherent_internal_edge', wraps=vision._coherent_internal_edge) as recovered:
            with self.assertRaises(ValueError):
                list(vision._edge_steps(image, [120., 20.], [120., 220.], hsv=hsv,
                    coherent_outline=True))
        self.assertEqual(grids.call_count, 1)
        self.assertEqual(recovered.call_count, 1)
        self.assertTrue(recovered.call_args.kwargs['outline'])

    def test_failed_outline_fit_still_requires_45_blue_interior_stations(self):
        image = np.full((260, 240, 3), (200, 60, 0), dtype=np.uint8)
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        radius = 12
        rows = []
        for station in range(64):
            row = [0.]*(2*radius+1)
            row[radius-8] = 1. if station % 2 else .8
            row[radius+8] = .8 if station % 2 else 1.
            rows.append(row)
        allowed = [[station < 44]*(2*radius+1) for station in range(64)]
        with patch.object(vision, '_BATCH_EDGES', True), patch.object(
                vision, '_contrast_grid', return_value=rows) as grids, patch.object(
                vision, '_blue_grid', return_value=allowed), patch.object(
                vision, '_coherent_internal_edge', wraps=vision._coherent_internal_edge) as recovered:
            with self.assertRaises(ValueError):
                list(vision._edge_steps(image, [120., 20.], [120., 220.], hsv=hsv,
                    coherent_outline=True))
        self.assertEqual(grids.call_count, 1)
        self.assertEqual(recovered.call_count, 1)

    def test_blue_grid_native_and_portable_match_half_pixel_precision_and_frame_rounding(self):
        hsv = np.zeros((50, 50, 3), dtype=np.uint8)
        hsv[:, ::2] = (110, 200, 180)
        for x in (-.25, 8.50000001):
            for inside_only in (False, True):
                with self.subTest(x=x, inside_only=inside_only):
                    a, b, normal = [x, 4.], [x, 46.], [1., 0.]
                    with patch.object(vision, '_BATCH_EDGES', True):
                        native = vision._blue_grid(hsv, a, b, normal, 5, inside_only=inside_only)
                    with patch.object(vision, '_BATCH_EDGES', False):
                        portable = vision._blue_grid(hsv, a, b, normal, 5, inside_only=inside_only)
                    self.assertEqual(np.array(native).shape, (64, 11))
                    self.assertTrue(np.array_equal(np.array(native), np.array(portable)))

    def test_successful_original_outline_fit_does_not_attempt_coherent_recovery(self):
        image = np.full((260, 240, 3), (200, 60, 0), dtype=np.uint8)
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        radius = 12
        rows = [[1. if offset == radius else 0. for offset in range(2*radius+1)]
                for _ in range(64)]
        with patch.object(vision, '_BATCH_EDGES', True), patch.object(
                vision, '_contrast_grid', return_value=rows) as grids, patch.object(
                vision, '_coherent_internal_edge', side_effect=AssertionError('Original fit already passed')) as recovered:
            steps = list(vision._edge_steps(image, [120., 20.], [120., 220.], hsv=hsv,
                coherent_outline=True))
        edge = steps[-1]['edge']
        self.assertEqual(edge[1]['support'], 64)
        self.assertEqual(edge[1]['sampled_stations'], 64)
        self.assertLessEqual(edge[1]['rms_px'], 1e-4)
        self.assertEqual(grids.call_count, 1)
        recovered.assert_not_called()

    def test_uniform_blue_rectangle_has_no_internal_top_side_separator(self):
        fixture = fixtures.NearVisionTests()
        image, intr, ext, plane = fixture.scene()
        image[:] = (80, 140, 170)
        cv2.rectangle(image, (500, 450), (700, 750), (200, 60, 0), -1)
        cfg = {'intrinsics': intr, 'extrinsics': ext, 'table_plane': plane}
        with patch.object(vision, 'pose_candidates', wraps=vision.pose_candidates) as poses:
            out = self.solve(image, cfg)[-1]['observation']
        self.assertEqual(out['status'], 'no_complete_face', out)
        self.assertNotIn('cube_center_Base_mm', out)
        self.assertEqual(poses.call_count, 0)

    def test_coherent_recovery_cannot_replace_missing_distributed_support(self):
        radius = 5
        image = np.full((100, 200, 3), (200, 60, 0), dtype=np.uint8)
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        for contrast_support, blue_support in ((44, 64), (64, 44)):
            rows = [[0.]*(2*radius+1) for unused in range(64)]
            both = [[False]*(2*radius+1) for unused in range(64)]
            for station in range(contrast_support):
                rows[station][radius] = 1.
            for station in range(blue_support):
                both[station][radius] = True
            with self.subTest(contrast=contrast_support, blue=blue_support), patch.object(
                    vision, '_separator_evidence', side_effect=AssertionError('Too few stations reached separator')):
                with self.assertRaises(ValueError):
                    vision._coherent_internal_edge([20., 50.], [180., 50.], [0., 1.],
                        radius, rows, both, hsv, image)

    def test_include_range_keeps_original_proposal_path(self):
        fixture = fixtures.NearVisionTests()
        image, intr, ext, plane = fixture.scene()
        cfg = {'intrinsics': intr, 'extrinsics': ext, 'table_plane': plane}
        with patch.object(vision, '_seam_seed_steps', wraps=vision._seam_seed_steps) as seams, patch.object(
                vision, '_contour_side_seed', wraps=vision._contour_side_seed) as retries, patch.object(
                vision, '_recover_seam_steps', wraps=vision._recover_seam_steps) as recovered, patch.object(
                vision, '_edge_steps', wraps=vision._edge_steps) as edges:
            steps = self.solve(image, cfg, include_range=True)
        self.assertEqual(steps[-1]['observation']['status'], 'localized')
        self.assertTrue(seams.call_args_list)
        self.assertTrue(all(call.kwargs.get('reuse_local') is False for call in seams.call_args_list))
        self.assertTrue(all(call.kwargs.get('recover_internal') is False for call in seams.call_args_list))
        self.assertTrue(all(call.args[1] != 2 for call in retries.call_args_list))
        recovered.assert_not_called()
        self.assertTrue(all(not call.kwargs.get('coherent', False) for call in edges.call_args_list))
        self.assertTrue(all(not call.kwargs.get('coherent_outline', False) for call in edges.call_args_list))
        self.assertTrue(all(call.kwargs.get('profile') is None for call in edges.call_args_list))
        self.assertFalse(any(step['stage']=='outline_profile_recovered' for step in steps))


if __name__ == '__main__':
    unittest.main()
