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

    def steps(self, plane=True, **policy):
        return vision.localize_steps(self.image, self.K, self.D, self.T,
                                     self.plane if plane else None, **policy)

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

    def test_fast_only_failure_returns_raw_diagnostics_without_target_authority(self):
        search = {'stage': 'edge_validation', 'candidates': [{'reason': 'weak boundary'}]}
        failed = {'status': 'no_complete_face', 'reason': 'No reliable measured side',
                  'search': search, 'cube_center_Base_mm': [1., 2., 3.],
                  'range_candidate': {'distance_mm': 250.},
                  'payload': {'must_not_be_published': True}}
        closed = []
        def shared(*args):
            try:
                yield {'stage': 'edge_samples', 'stations': 16}
                yield {'stage': 'complete', 'observation': failed}
            finally:
                closed.append(True)
        with patch.object(vision, '_shared_side_steps', side_effect=shared) as fast, \
                patch.object(vision, '_localize_legacy_steps',
                    side_effect=AssertionError('Fast-only must not start recovery')) as legacy:
            events = list(self.steps(allow_legacy=False))
        raw = events[-1]['result']
        self.assert_one_raw_result(events, raw)
        self.assertEqual(set(raw), {'status', 'reason', 'search', 'shared_side_processing_ms'})
        self.assertEqual(raw['status'], 'no_complete_face')
        self.assertEqual(raw['reason'], failed['reason'])
        self.assertIs(raw['search'], search)
        self.assertGreaterEqual(raw['shared_side_processing_ms'], 0)
        self.assertEqual(closed, [True])
        fast.assert_called_once_with(self.image, self.K, self.D, self.T, self.plane)
        legacy.assert_not_called()

    def test_fast_only_success_keeps_original_raw_pose_and_shared_arguments(self):
        raw = self.face()
        expected = copy.deepcopy(raw)
        def shared(*args):
            yield {'stage': 'candidates', 'count': 1}
            yield {'stage': 'complete', 'observation': {'status': 'localized', 'payload': raw}}
        with patch.object(vision, '_shared_side_steps', side_effect=shared) as fast, \
                patch.object(vision, '_localize_legacy_steps') as legacy:
            events = list(self.steps(allow_legacy=False))
        self.assert_one_raw_result(events, raw)
        self.assertEqual({key: raw[key] for key in expected}, expected)
        self.assertIn('shared_side_processing_ms', raw)
        self.assertNotIn('shared_side_recovery', raw)
        fast.assert_called_once_with(self.image, self.K, self.D, self.T, self.plane)
        legacy.assert_not_called()

    def test_fast_only_scene_rejection_is_distinct_from_resource_and_runtime_errors(self):
        for error in (ValueError('Physical edge rejected'), MemoryError('No image buffer'),
                      RuntimeError('CV resource unavailable')):
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
                    pending = self.steps(allow_legacy=False)
                    self.assertEqual(next(pending)['stage'], 'shared_edge_samples')
                    if isinstance(error, ValueError):
                        remaining = list(pending)
                        self.assertEqual(len(remaining), 1)
                        raw = remaining[0]['result']
                        self.assertEqual(remaining[0]['stage'], 'complete')
                        self.assertEqual(raw['status'], 'no_complete_face')
                        self.assertEqual(raw['reason'], str(error))
                        self.assertIsNone(raw.get('search'))
                        self.assertNotIn('cube_center_Base_mm', raw)
                        self.assertNotIn('range_candidate', raw)
                        self.assertNotIn('payload', raw)
                    else:
                        with self.assertRaises(type(error)) as raised:
                            list(pending)
                        self.assertIs(raised.exception, error)
                self.assertEqual(closed, [True])
                legacy.assert_not_called()

    def test_fast_only_requires_table_before_starting_either_search(self):
        with patch.object(vision, '_shared_side_steps') as shared, \
                patch.object(vision, '_localize_legacy_steps') as legacy:
            with self.assertRaisesRegex(ValueError, '[Tt]able'):
                list(self.steps(plane=False, allow_legacy=False))
        shared.assert_not_called()
        legacy.assert_not_called()

    def test_fast_only_cancellation_closes_shared_search_without_result_or_fallback(self):
        closed = []
        def shared(*args):
            try:
                yield {'stage': 'edge_samples', 'stations': 8}
                self.fail('Cancelled fast-only search resumed')
            finally:
                closed.append(True)
        with patch.object(vision, '_shared_side_steps', side_effect=shared), \
                patch.object(vision, '_localize_legacy_steps') as legacy:
            pending = self.steps(allow_legacy=False)
            self.assertEqual(next(pending)['stage'], 'shared_edge_samples')
            pending.close()
            pending.close()
            self.assertEqual(list(pending), [])
        self.assertEqual(closed, [True])
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

    def test_real_synthetic_geometry_matches_public_sync_with_typed_evidence(self):
        for yaw in (.7, .95):
            with self.subTest(yaw=yaw):
                image, transform = self.synthetic_scene(yaw)
                expected = vision.localize(image, self.K, self.D, transform, self.plane)
                events = list(vision.localize_steps(image, self.K, self.D, transform, self.plane))
                actual = events[-1]['result']
                self.assert_one_raw_result(events, actual)
                np.testing.assert_allclose(actual['cube_center_Camera_mm'], expected['cube_center_Camera_mm'], atol=1e-9)
                np.testing.assert_allclose(actual['cube_center_Base_mm'], expected['cube_center_Base_mm'], atol=1e-9)
                self.assertEqual(actual['status'], expected['status'])
                if actual['status'] == 'complete_cube_candidate':
                    evidence = actual['evidence_type']
                    self.assertIn(evidence, ('nonplanar_cube_five_corners', 'nonplanar_cube_six_corners'))
                    self.assertEqual(evidence, expected['evidence_type'])
                    count = 6 if evidence == 'nonplanar_cube_six_corners' else 5
                    pose = actual['cube_pose']; other = expected['cube_pose']
                    self.assertEqual(len(pose['image_points_px']), count)
                    np.testing.assert_allclose(pose['image_points_px'], other['image_points_px'], atol=1e-9)
                    self.assertAlmostEqual(pose['corner_reprojection_rms_px'],
                                           other['corner_reprojection_rms_px'], places=9)
                    self.assertLessEqual(pose['corner_reprojection_rms_px'], 3.)
                    self.assertEqual(len(actual['observed_edges']), 6)
                    for edge in actual['observed_edges']:
                        self.assertEqual(edge['quality']['sampled_stations'], 64)
                        self.assertGreaterEqual(edge['quality']['support'], 45)
                        self.assertLessEqual(edge['quality']['rms_px'], 3.)
                    self.assertEqual(len(actual['half_corner_errors_px']), 2*count)
                    self.assertTrue(all(0 <= e <= 3 for e in actual['half_corner_errors_px']))
                    self.assertNotIn('face_quad_px', actual)
                else:
                    self.assertEqual(actual['status'], 'complete_face_candidate')
                    pose = actual['poses'][0]
                    np.testing.assert_allclose(actual['face_quad_px'], expected['face_quad_px'], atol=1e-9)
                    self.assertAlmostEqual(pose['face_reprojection_rms_px'],
                                           expected['poses'][0]['face_reprojection_rms_px'], places=9)
                    self.assertEqual(actual['side_selection'], expected['side_selection'])
                    self.assertLessEqual(pose['face_reprojection_rms_px'], 2.)
                self.assertTrue(pose['table_geometry']['accepted'])
                self.assertLessEqual(pose['table_geometry']['normal_deviation_deg'], 20.)
                self.assertLessEqual(abs(pose['table_geometry']['height_error_mm']), 8.)
                np.testing.assert_allclose(actual['cube_center_Base_mm'], [10., -10., -10.], atol=3.)

    def test_original_side_session_uses_same_frame_hint_through_real_solver(self):
        from device.k230.vision.cooperative_static_session import CooperativeStaticPickSession
        image, transform = self.synthetic_scene(.7)
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, (90, 120, 220), (140, 255, 255))
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        top_hint = list(cv2.boundingRect(max(contours, key=cv2.contourArea)))
        # A larger distant blue object must not redirect this stopped query.
        cv2.rectangle(image, (50, 500), (450, 800), (180, 120, 10), -1)
        config = {'intrinsics': {'camera_matrix': self.K, 'distortion_coefficients': self.D,
                                'image_width': 1280, 'image_height': 960},
                  'extrinsics': {'T_Base_from_Camera_mm': transform},
                  'table_plane': self.plane,
                  'tracking_task': {'static_localization_policy': 'shared_side_only'}}
        frame = {'frame_id': 7, 'capture_timestamp_ms': 100,
                 'image_size_px': [1280, 960], 'clock_domain': 'k230_ticks_ms',
                 'timestamp_source': 'software_snapshot_call'}
        def compressed(quality):
            ok, jpeg = cv2.imencode('.jpg', image, [cv2.IMWRITE_JPEG_QUALITY, quality])
            self.assertTrue(ok)
            return SimpleNamespace(bytearray=lambda: bytearray(jpeg.tobytes()))
        outcomes = []
        for hint in (None, top_hint):
            session = CooperativeStaticPickSession(config, lambda: 100)
            # Isolate the original face solver's hint-selection contract;
            # real fast cube/G sessions have their own public-path tests.
            with patch.object(physical_side, 'localize_steps', wraps=physical_side._original_localize_steps) as solver:
                session.start(SimpleNamespace(compressed=compressed), frame, hint)
                for _ in range(10000):
                    if session.phase is None:
                        break
                    session.advance()
                self.assertIsNone(session.phase)
                self.assertEqual(solver.call_args.args[2], hint)
            outcomes.append(session.latest)
        self.assertEqual(outcomes[0]['status'], 'no_complete_face')
        self.assertEqual(outcomes[1]['status'], 'localized', outcomes[1])
        self.assertEqual(outcomes[1]['frame_id'], 7)
        np.testing.assert_allclose(outcomes[1]['cube_center_Base_mm'], [10., -10., -10.], atol=3.)


if __name__ == '__main__':
    unittest.main()
