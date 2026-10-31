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
        return list(vision.localize_steps(image, self.frame(), None,
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


if __name__ == '__main__':
    unittest.main()
