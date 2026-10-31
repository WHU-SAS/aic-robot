"""Observed-outline proposals share one image scan and retain strict gates."""
import importlib.util
import json
import unittest
from unittest.mock import patch
from host.project_paths import ROOT

HAS_VISION = importlib.util.find_spec('cv2') is not None
if HAS_VISION:
    import cv2
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

    def test_include_range_keeps_original_proposal_path(self):
        fixture = fixtures.NearVisionTests()
        image, intr, ext, plane = fixture.scene()
        cfg = {'intrinsics': intr, 'extrinsics': ext, 'table_plane': plane}
        with patch.object(vision, '_seam_seed_steps', wraps=vision._seam_seed_steps) as seams, patch.object(
                vision, '_contour_side_seed', wraps=vision._contour_side_seed) as retries:
            steps = self.solve(image, cfg, include_range=True)
        self.assertEqual(steps[-1]['observation']['status'], 'localized')
        self.assertTrue(seams.call_args_list)
        self.assertTrue(all(call.kwargs.get('reuse_local') is False for call in seams.call_args_list))
        self.assertTrue(all(call.args[1] != 2 for call in retries.call_args_list))


if __name__ == '__main__':
    unittest.main()
