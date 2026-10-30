"""Static rounded-contour retries retain the strict face and moving contracts."""
import copy
import importlib.util
import json
from pathlib import Path
import unittest
from unittest.mock import patch
from host.project_paths import ROOT

HAS_VISION = importlib.util.find_spec('cv2') is not None
if HAS_VISION:
    import cv2
    from device.k230.vision import physical_side as sides


@unittest.skipUnless(HAS_VISION,'Dedicated vision environment')
class StaticContourRetryTests(unittest.TestCase):
    def setUp(self):
        self.seed = [[20.,20.],[120.,20.],[120.,140.],[20.,140.]]

    def test_rounded_lower_endpoint_uses_long_actual_lateral_contour(self):
        contour = [[0.,20.],[20.,0.],[120.,0.],[140.,20.],
            [140.,120.],[120.,140.],[20.,140.],[0.,120.]]
        result = sides._contour_side_seed(self.seed,3,contour)
        expected = [[0.,20.],[120.,20.],[120.,140.],[0.,140.]]
        for actual,wanted in zip(result,expected):
            for a,b in zip(actual,wanted):
                self.assertAlmostEqual(a,b,delta=1e-4)
        self.assertEqual(self.seed[0],[20.,20.])

    def test_short_or_degenerate_failed_side_has_no_retry(self):
        for height in (0., 24.):
            seed = [[20.,20.],[120.,20.],[120.,20.+height],[20.,20.+height]]
            self.assertIsNone(sides._contour_side_seed(seed,3,
                [[0.,0.],[0.,140.],[120.,140.],[120.,0.]]))

    def test_reflection_extremum_without_long_parallel_support_is_not_a_side(self):
        contour = [[20.,20.],[120.,20.],[120.,140.],[20.,140.],
            [0.,139.],[0.,136.],[20.,135.]]
        self.assertIsNone(sides._contour_side_seed(self.seed,3,contour))

    def test_another_face_oblique_or_far_outer_boundary_cannot_expand_this_side(self):
        for contour in ([[0.,20.],[-65.,120.],[20.,140.],[120.,140.],[120.,20.]],
                        [[-80.,20.],[-80.,140.],[120.,140.],[120.,20.]]):
            self.assertIsNone(sides._contour_side_seed(self.seed,3,contour))

    def test_inner_crease_and_top_bottom_edges_do_not_request_lateral_retry(self):
        contour = [[40.,20.],[120.,20.],[120.,140.],[40.,140.]]
        self.assertIsNone(sides._contour_side_seed(self.seed,3,contour))
        for edge in (0,2):
            self.assertIsNone(sides._contour_side_seed(self.seed,edge,contour))

    def fixture(self,relative,label):
        folder = ROOT/relative
        if not (folder/'manifest.json').exists():
            self.skipTest('Retained original JPEG is local-only')
        manifest = json.loads((folder/'manifest.json').read_text())
        entry = next(item for item in manifest['frames'] if item['label']==label)
        return cv2.imread(str(folder/entry['image_file'])),entry['frame'],json.loads((folder/'config.json').read_text())

    def near(self):
        return self.fixture('data/car/one_shot_parking/near_speedup/evidence','stopped_verify')

    def run_image(self,image,frame,config,**options):
        return list(sides.localize_steps(image,frame,None,config['intrinsics'],
            config['extrinsics'],config['table_plane'],**options))

    def test_retained_rounded_cube_passes_all_original_checks_after_four_edge_retry(self):
        image,frame,config = self.near()
        for native in (True,False):
            with self.subTest(native=native),patch.object(sides,'_BATCH_EDGES',native):
                events = self.run_image(image,frame,config)
            result = events[-1]['observation']
            self.assertEqual(result['status'],'localized',result)
            self.assertEqual(result['frame_id'],frame['frame_id'])
            self.assertEqual(result['capture_timestamp_ms'],frame['capture_timestamp_ms'])
            self.assertEqual(result['search']['measured_seed_count'],2)
            self.assertEqual(result['search']['candidate_limit'],3)
            self.assertEqual(result['search']['rejected'][0]['reason'],'Insufficient edge contrast')
            self.assertEqual([e['edge_index'] for e in events if
                e['stage']=='edge_complete' and e['candidate']==1],[0,1,2,3])
            self.assertFalse(any(e['stage']=='edge_reused' and e['candidate']==1 for e in events))
            payload = result['payload']
            self.assertLess(payload['face_quad_px'][0][0],603.)
            self.assertLess(payload['face_quad_px'][3][0],604.)
            self.assertEqual(payload['side_selection'],'single_verified_side')
            self.assertLessEqual(payload['poses'][0]['face_reprojection_rms_px'],2.)
            self.assertTrue(payload['poses'][0]['table_geometry']['accepted'])
            self.assertTrue(all(e['support']>=45 and e['sampled_stations']==64 for e in payload['physical_edges']))

    def test_moving_range_has_no_retry_or_extra_work_and_does_not_gain_authority(self):
        image,frame,config = self.near()
        with patch.object(sides,'_contour_side_seed',side_effect=AssertionError('moving retry forbidden')):
            events = self.run_image(image,frame,config,include_range=True)
        result = events[-1]['observation']
        self.assertEqual(result['status'],'no_complete_face')
        self.assertNotIn('cube_center_Base_mm',result)
        self.assertEqual(result['search']['measured_seed_count'],1)
        self.assertEqual(result['search']['seed_count'],1)

    def test_original_success_after_first_failure_prevents_appended_retry(self):
        image,frame,config = self.near()
        original = sides._seam_seed_steps
        def original_seeds(*args):
            for event in original(*args):
                yield event
                if event['stage']=='seam_seed':
                    # A second pre-existing proposal lies inside the original
                    # search radius and succeeds before the appended fallback.
                    extra = copy.deepcopy(event)
                    line = extra['top_edge'][0]
                    extra['seed'][0] = sides.intersect(line,[1.,0.,-32.])
                    extra['seed'][3][0] = 32.
                    yield extra
        with patch.object(sides,'_seam_seed_steps',side_effect=original_seeds), \
                patch.object(sides,'_contour_side_seed',wraps=sides._contour_side_seed) as retry:
            events = self.run_image(image,frame,config)
        result = events[-1]['observation']
        self.assertEqual(result['status'],'localized')
        self.assertEqual(retry.call_count,1)
        self.assertEqual(result['search']['seed_count'],3)
        self.assertEqual(result['search']['measured_seed_count'],2)
        self.assertFalse(any(e.get('candidate')==2 for e in events))

    def test_existing_attempt_limit_is_not_extended_by_retry(self):
        image,frame,config = self.near()
        result = self.run_image(image,frame,config,max_candidates=1)[-1]['observation']
        self.assertEqual(result['status'],'no_complete_face')
        self.assertEqual(result['search']['measured_seed_count'],1)
        self.assertEqual(result['search']['candidate_limit'],1)

    def test_three_table_inconsistent_originals_stay_strict_failures_without_retry(self):
        cases = [('data/car/one_shot_parking/failed70_retry/current_evidence','current_stopped'),
            ('data/car/one_shot_parking/failed70/evidence','stopped_verify'),
            ('data/car/one_shot_parking/diagnostics/trial_20261030_213040/current_evidence','current_stopped')]
        for folder,label in cases:
            with self.subTest(folder=folder):
                image,frame,config = self.fixture(folder,label)
                result = self.run_image(image,frame,config)[-1]['observation']
                self.assertEqual(result['status'],'no_complete_face')
                self.assertNotIn('cube_center_Base_mm',result)
                self.assertEqual(result['search']['rejected'][0]['reason'],
                    'No side pose consistent with calibrated table')


if __name__=='__main__':
    unittest.main()
