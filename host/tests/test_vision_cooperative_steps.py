"""Cooperative full localization preserves candidate math and cancellation."""
import copy
from contextlib import ExitStack
import importlib.util
from types import SimpleNamespace
import unittest
from unittest.mock import patch

HAS_VISION = importlib.util.find_spec('cv2') is not None
if HAS_VISION:
    import cv2
    import numpy as np
    from device.k230.pick import vision
    from device.k230.vision import physical_side


@unittest.skipUnless(HAS_VISION, 'Dedicated vision environment')
class FullLocalizationStepsTests(unittest.TestCase):
    def setUp(self):
        self.image = SimpleNamespace(shape=(960, 1280, 3))
        self.K = [[1200., 0., 640.], [0., 1600., 580.], [0., 0., 1.]]
        self.D = [0.] * 5
        self.T = [[0., -1., 0., 7.], [1., 0., 0., 11.],
                  [0., 0., 1., 13.], [0., 0., 0., 1.]]
        self.plane = {'normal': [0., 0., 1.], 'offset_mm': -20.}

    def face(self):
        return {'status': 'complete_face_candidate', 'boundary_method': 'measured_edges',
                'face_quad_px': [[600., 500.], [690., 505.], [686., 600.], [598., 594.]],
                'poses': [{'cube_center_Camera_mm': [4., 5., 6.],
                           'face_reprojection_rms_px': .75}],
                'cube_center_Base_mm': [2., 15., 19.],
                'side_selection': 'larger_verified_side'}

    def steps(self, plane=True):
        return vision.localize_steps(self.image, self.K, self.D, self.T,
                                     self.plane if plane else None)

    def assert_one_raw_result(self, events, expected):
        completed = [event for event in events if event['stage'] == 'complete']
        self.assertEqual(len(completed), 1)
        self.assertIs(completed[0], events[-1])
        self.assertIs(completed[0]['result'], expected)
        self.assertNotIn('observation', completed[0])
        for event in events[:-1]:
            self.assertNotIn('result', event)
            self.assertNotIn('observation', event)

    def test_shared_success_exposes_only_final_raw_result_and_skips_legacy(self):
        raw = self.face()
        calls = []
        def shared(*args):
            calls.append('shared_started')
            try:
                yield {'stage': 'edge_samples', 'stations': 8}
                calls.append('shared_complete')
                yield {'stage': 'complete', 'observation': {'status': 'localized', 'payload': raw}}
            finally:
                calls.append('shared_closed')
        with patch.object(vision, '_shared_side_steps', side_effect=shared) as fast, \
                patch.object(vision, '_localize_legacy_steps', side_effect=AssertionError('No fallback')) as legacy:
            pending = self.steps()
            self.assertEqual(calls, [])
            progress = next(pending)
            self.assertEqual(progress, {'stage': 'shared_edge_samples', 'stations': 8})
            self.assertEqual(calls, ['shared_started'])
            events = [progress] + list(pending)
        self.assert_one_raw_result(events, raw)
        self.assertEqual(calls, ['shared_started', 'shared_complete', 'shared_closed'])
        self.assertIn('shared_side_processing_ms', raw)
        fast.assert_called_once_with(self.image, self.K, self.D, self.T, self.plane)
        legacy.assert_not_called()

    def test_failed_shared_observation_runs_legacy_after_shared_cleanup(self):
        raw = self.face()
        calls = []
        def shared(*args):
            calls.append('shared')
            try:
                yield {'stage': 'candidates', 'count': 0}
                yield {'stage': 'complete', 'observation': {
                    'status': 'no_complete_face', 'reason': 'No reliable measured side'}}
            finally:
                calls.append('shared_closed')
        def legacy(*args):
            calls.append('legacy')
            yield {'stage': 'legacy_hsv'}
            yield {'stage': 'complete', 'result': raw}
        with patch.object(vision, '_shared_side_steps', side_effect=shared), \
                patch.object(vision, '_localize_legacy_steps', side_effect=legacy) as fallback:
            events = list(self.steps())
        self.assert_one_raw_result(events, raw)
        self.assertEqual(calls, ['shared', 'shared_closed', 'legacy'])
        self.assertEqual(raw['shared_side_recovery']['fast_side_reason'], 'No reliable measured side')
        self.assertGreaterEqual(raw['shared_side_recovery']['fast_side_ms'], 0)
        self.assertGreaterEqual(raw['shared_side_recovery']['legacy_recovery_ms'], 0)
        fallback.assert_called_once_with(self.image, self.K, self.D, self.T, self.plane)

    def test_shared_value_error_preserves_original_legacy_recovery(self):
        raw = self.face()
        def shared(*args):
            yield {'stage': 'edge_samples', 'stations': 16}
            raise ValueError('Physical edge rejected')
        def legacy(*args):
            yield {'stage': 'legacy_direct_sides'}
            yield {'stage': 'complete', 'result': raw}
        with patch.object(vision, '_shared_side_steps', side_effect=shared), \
                patch.object(vision, '_localize_legacy_steps', side_effect=legacy):
            events = list(self.steps())
        self.assert_one_raw_result(events, raw)
        self.assertEqual(raw['shared_side_recovery']['fast_side_reason'], 'Physical edge rejected')

    def test_resource_and_runtime_errors_do_not_switch_to_legacy(self):
        for error in (MemoryError('No image buffer'), RuntimeError('CV resource unavailable')):
            with self.subTest(error=type(error).__name__):
                closed = []
                def shared(*args):
                    try:
                        yield {'stage': 'edge_samples', 'stations': 8}
                        raise error
                    finally:
                        closed.append(True)
                with patch.object(vision, '_shared_side_steps', side_effect=shared), \
                        patch.object(vision, '_localize_legacy_steps') as legacy:
                    pending = self.steps()
                    self.assertEqual(next(pending)['stage'], 'shared_edge_samples')
                    with self.assertRaises(type(error)):
                        next(pending)
                self.assertEqual(closed, [True])
                legacy.assert_not_called()

    def test_shared_generator_without_result_is_not_treated_as_candidate_failure(self):
        def shared(*args):
            yield {'stage': 'edge_samples', 'stations': 8}
        with patch.object(vision, '_shared_side_steps', side_effect=shared), \
                patch.object(vision, '_localize_legacy_steps') as legacy:
            with self.assertRaisesRegex(RuntimeError, 'without a result'):
                list(self.steps())
        legacy.assert_not_called()

    def test_no_table_preserves_legacy_events_without_shared_search(self):
        raw = self.face()
        def legacy(*args):
            yield {'stage': 'legacy_hsv'}
            yield {'stage': 'complete', 'result': raw}
        with patch.object(vision, '_shared_side_steps', side_effect=AssertionError('Shared requires table')) as shared, \
                patch.object(vision, '_localize_legacy_steps', side_effect=legacy) as fallback:
            events = list(self.steps(plane=False))
        self.assert_one_raw_result(events, raw)
        self.assertEqual(events[0], {'stage': 'legacy_hsv'})
        self.assertNotIn('shared_side_processing_ms', raw)
        self.assertNotIn('shared_side_recovery', raw)
        shared.assert_not_called()
        fallback.assert_called_once_with(self.image, self.K, self.D, self.T, None)

    def test_closing_outer_generator_closes_active_shared_generator(self):
        closed = []
        def shared(*args):
            try:
                yield {'stage': 'edge_samples', 'stations': 8}
                self.fail('Cancelled shared work resumed')
            finally:
                closed.append('shared')
        with patch.object(vision, '_shared_side_steps', side_effect=shared), \
                patch.object(vision, '_localize_legacy_steps') as legacy:
            pending = self.steps()
            next(pending)
            pending.close()
            self.assertEqual(list(pending), [])
        self.assertEqual(closed, ['shared'])
        legacy.assert_not_called()

    def test_closing_outer_generator_closes_legacy_both_without_table_and_in_recovery(self):
        for table in (False, True):
            with self.subTest(table=table):
                closed = []
                def shared(*args):
                    yield {'stage': 'complete', 'observation': {
                        'status': 'no_complete_face', 'reason': 'Measured side rejected'}}
                def legacy(*args):
                    try:
                        yield {'stage': 'legacy_hsv'}
                        self.fail('Cancelled legacy work resumed')
                    finally:
                        closed.append('legacy')
                with patch.object(vision, '_shared_side_steps', side_effect=shared), \
                        patch.object(vision, '_localize_legacy_steps', side_effect=legacy):
                    pending = self.steps(plane=table)
                    self.assertEqual(next(pending)['stage'], 'legacy_hsv')
                    pending.close()
                self.assertEqual(closed, ['legacy'])

    def test_shared_adapter_propagates_cancellation_to_physical_side_generator(self):
        closed = []
        def physical(*args):
            try:
                yield {'stage': 'edge_samples', 'stations': 8}
                self.fail('Cancelled physical-side work resumed')
            finally:
                closed.append('physical')
        with patch.object(physical_side, 'localize_steps', side_effect=physical) as native, \
                patch.object(vision, '_localize_legacy_steps') as legacy:
            pending = self.steps()
            self.assertEqual(next(pending)['stage'], 'shared_edge_samples')
            pending.close()
        self.assertEqual(closed, ['physical'])
        args = native.call_args.args
        self.assertIs(args[0], self.image)
        self.assertIsNone(args[2])
        self.assertEqual(args[3]['camera_matrix'], self.K)
        self.assertEqual(args[1]['capture_timestamp_ms'], None)
        legacy.assert_not_called()

    def legacy_run(self, variant, cooperative):
        trace = []
        base = self.face()
        base.pop('side_selection')
        base.pop('cube_center_Base_mm')
        candidates = {'basic': copy.deepcopy(base), 'dim': copy.deepcopy(base),
                      'gradient': copy.deepcopy(base), 'reflection': copy.deepcopy(base),
                      'direct': copy.deepcopy(base), 'selected': copy.deepcopy(base)}
        values = {'table_context': None, 'fast_sides': None, 'basic_face': None,
                  'dim_face': None, 'prefer_sides': None, 'shallow_face': False,
                  'internal_seam': None, 'split_face': None, 'gradient_side': None,
                  'detect_reflection_face': None}
        if variant == 'direct':values['fast_sides'] = candidates['direct']
        elif variant == 'preferred':
            values['basic_face'] = candidates['basic']
            values['prefer_sides'] = candidates['selected']
        elif variant in ('basic', 'table'):
            values['basic_face'] = candidates['basic']
            if variant == 'table':values['table_context'] = {'table': 'context'}
        elif variant == 'dim':values['dim_face'] = candidates['dim']
        elif variant == 'gradient':values['gradient_side'] = candidates['gradient']
        elif variant == 'reflection':values['detect_reflection_face'] = candidates['reflection']
        else:raise AssertionError('Unknown scenario')
        hsv = object()
        def cvt(image, code):
            trace.append('cvtColor')
            self.assertIs(image, self.image)
            self.assertEqual(code, cv2.COLOR_BGR2HSV)
            return hsv
        def primitive(name):
            def invoke(*args, **kwargs):
                trace.append(name)
                if name == 'constrain_face':return args[0]
                if name in ('basic_face', 'dim_face', 'internal_seam'):
                    self.assertIs(args[0], hsv)
                return values[name]
            return invoke
        with ExitStack() as stack:
            stack.enter_context(patch.object(vision.cv2, 'cvtColor', side_effect=cvt))
            for name in list(values) + ['constrain_face']:
                stack.enter_context(patch.object(vision, name, side_effect=primitive(name)))
            if cooperative:
                events = list(vision._localize_legacy_steps(self.image, self.K, self.D, self.T, self.plane))
                result = events[-1]['result']
                self.assert_one_raw_result(events, result)
            else:
                result = vision._localize_legacy(self.image, self.K, self.D, self.T, self.plane)
                events = None
        return result, trace, events

    def test_legacy_generator_and_wrapper_preserve_primitive_order_and_transform(self):
        expected = {
            'direct': ['cvtColor', 'table_context', 'fast_sides'],
            'preferred': ['cvtColor', 'table_context', 'fast_sides', 'basic_face', 'prefer_sides'],
            'basic': ['cvtColor', 'table_context', 'fast_sides', 'basic_face', 'prefer_sides',
                      'shallow_face', 'internal_seam', 'shallow_face'],
            'dim': ['cvtColor', 'table_context', 'fast_sides', 'basic_face', 'dim_face', 'prefer_sides',
                    'shallow_face', 'internal_seam', 'shallow_face'],
            'gradient': ['cvtColor', 'table_context', 'fast_sides', 'basic_face', 'dim_face',
                         'prefer_sides', 'split_face', 'gradient_side', 'prefer_sides', 'shallow_face'],
            'reflection': ['cvtColor', 'table_context', 'fast_sides', 'basic_face', 'dim_face',
                           'prefer_sides', 'split_face', 'gradient_side', 'detect_reflection_face',
                           'prefer_sides', 'shallow_face'],
            'table': ['cvtColor', 'table_context', 'fast_sides', 'basic_face', 'prefer_sides',
                      'shallow_face', 'internal_seam', 'shallow_face', 'shallow_face', 'constrain_face'],
        }
        for variant, order in expected.items():
            with self.subTest(variant=variant):
                actual, trace, events = self.legacy_run(variant, cooperative=True)
                sync, sync_trace, unused = self.legacy_run(variant, cooperative=False)
                self.assertEqual(trace, order)
                self.assertEqual(sync_trace, order)
                self.assertEqual(actual, sync)
                self.assertEqual(actual['cube_center_Camera_mm'], [4., 5., 6.])
                self.assertEqual(actual['cube_center_Base_mm'], [2., 15., 19.])
                self.assertEqual(actual['poses'][0]['face_reprojection_rms_px'], .75)
                self.assertEqual(events[0]['stage'], 'legacy_hsv')

    def synthetic_scene(self, yaw):
        K = np.array(self.K)
        rv, tv = np.array([.4, yaw, .02]), np.array([-10., -10., 250.])
        front = np.array([[0., 0., 0.], [20., 0., 0.], [20., 20., 0.], [0., 20., 0.]])
        right = np.array([[20., 0., 0.], [20., 0., 20.], [20., 20., 20.], [20., 20., 0.]])
        top = np.array([[0., 0., 20.], [20., 0., 20.], [20., 0., 0.], [0., 0., 0.]])
        image = np.full((960, 1280, 3), (80, 140, 170), np.uint8)
        for face, color in ((top, (245, 99, 0)), (right, (145, 36, 1)), (front, (180, 120, 10))):
            quad = cv2.projectPoints(face, rv, tv, K, np.zeros(5))[0].reshape(-1, 2)
            cv2.fillConvexPoly(image, np.rint(quad).astype(np.int32), color)
        image = cv2.GaussianBlur(image, (3, 3), .7)
        rotation = np.array([[0., 0., 1.], [-1., 0., 0.], [0., -1., 0.]]) @ cv2.Rodrigues(rv)[0].T
        transform = np.eye(4)
        transform[:3, :3] = rotation
        transform[:3, 3] = -rotation @ tv
        return image, transform.tolist()

    def test_real_synthetic_faces_match_public_sync_centers_rms_and_side_choice(self):
        for yaw in (.7, .95):
            with self.subTest(yaw=yaw):
                image, transform = self.synthetic_scene(yaw)
                expected = vision.localize(image, self.K, self.D, transform, self.plane)
                events = list(vision.localize_steps(image, self.K, self.D, transform, self.plane))
                actual = events[-1]['result']
                self.assert_one_raw_result(events, actual)
                np.testing.assert_allclose(actual['cube_center_Camera_mm'], expected['cube_center_Camera_mm'], atol=1e-9)
                np.testing.assert_allclose(actual['cube_center_Base_mm'], expected['cube_center_Base_mm'], atol=1e-9)
                np.testing.assert_allclose(actual['face_quad_px'], expected['face_quad_px'], atol=1e-9)
                self.assertAlmostEqual(actual['poses'][0]['face_reprojection_rms_px'],
                                       expected['poses'][0]['face_reprojection_rms_px'], places=9)
                self.assertEqual(actual['side_selection'], expected['side_selection'])
                self.assertLessEqual(actual['poses'][0]['face_reprojection_rms_px'], 2.)
                np.testing.assert_allclose(actual['cube_center_Base_mm'], [10., -10., -10.], atol=3.)


if __name__ == '__main__':
    unittest.main()
